#include "gicp.h"

GicpLocalizer::GicpLocalizer(ros::NodeHandle &nh, ros::NodeHandle &private_nh) : 
    nh_(nh), private_nh_(private_nh), tf2_listener_(tf2_buffer_), 
    gicp_(new pclomp::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>())
    // gicp_(new pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>())
{

  key_value_stdmap_["state"] = "Initializing";
  init_params();

  // Publishers
  sensor_aligned_pose_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("points_aligned", 10);   // 配准后的点云
  gicp_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("ndt_pose", 10);      // 定位信息
  gicp_odom_pub_ = nh_.advertise<nav_msgs::Odometry>("ndt_path_odom", 10);         // path

  // exe_time_pub_ = nh_.advertise<std_msgs::Float32>("exe_time_ms", 10);   // 匹配时间
  diagnostics_pub_ = nh_.advertise<diagnostic_msgs::DiagnosticArray>("diagnostics", 10);

  // Subscribers
  initial_pose_sub_ = nh_.subscribe("initialpose", 100, &GicpLocalizer::callback_init_pose, this);   // 初始位姿, 由rviz发布
  map_points_sub_ = nh_.subscribe("points_map", 1, &GicpLocalizer::callback_pointsmap, this);    // 地图作为目标点云
  sensor_points_sub_ = nh_.subscribe("filtered_points", 1, &GicpLocalizer::callback_pointcloud, this);   // 实时点云作为源点云     // lidar

  diagnostic_thread_ = std::thread(&GicpLocalizer::timer_diagnostic, this);    // gicp匹配速度警示信息 
  diagnostic_thread_.detach();
} 

GicpLocalizer::~GicpLocalizer() {}

void GicpLocalizer::timer_diagnostic()
{
  ros::Rate rate(100);
  while (ros::ok()) {
    diagnostic_msgs::DiagnosticStatus diag_status_msg;
    diag_status_msg.name = "gicp_scan_matcher";
    diag_status_msg.hardware_id = "";

    for (const auto & key_value : key_value_stdmap_) {
      diagnostic_msgs::KeyValue key_value_msg;
      key_value_msg.key = key_value.first;
      key_value_msg.value = key_value.second;
      diag_status_msg.values.push_back(key_value_msg);
    }

    diag_status_msg.level = diagnostic_msgs::DiagnosticStatus::OK;
    diag_status_msg.message = "";
    if (key_value_stdmap_.count("state") && key_value_stdmap_["state"] == "Initializing") {   // count: 根据键查找map类型的数据个数
      diag_status_msg.level = diagnostic_msgs::DiagnosticStatus::WARN;
      diag_status_msg.message += "Initializing State. ";
    }
    if (key_value_stdmap_.count("skipping_publish_num") &&
      std::stoi(key_value_stdmap_["skipping_publish_num"]) > 1) {
      diag_status_msg.level = diagnostic_msgs::DiagnosticStatus::WARN;
      diag_status_msg.message += "skipping_publish_num > 1. ";
    }
    if (key_value_stdmap_.count("skipping_publish_num") &&
      std::stoi(key_value_stdmap_["skipping_publish_num"]) >= 5) {
      diag_status_msg.level = diagnostic_msgs::DiagnosticStatus::ERROR;
      diag_status_msg.message += "skipping_publish_num exceed limit. ";
    }

    diagnostic_msgs::DiagnosticArray diag_msg;
    diag_msg.header.stamp = ros::Time::now();
    diag_msg.status.push_back(diag_status_msg);
    diagnostics_pub_.publish(diag_msg);

    rate.sleep();
  }
}

void GicpLocalizer::callback_init_pose(
  const geometry_msgs::PoseWithCovarianceStamped::ConstPtr & initial_pose_msg_ptr)
{
  // 初始位姿
  // map的位姿旋转来与lidar一致
  // 2022/11/16 gj
  Eigen::AngleAxisd rotation_vecX(-M_PI/2, Eigen::Vector3d(1,0,0));    
  Eigen::AngleAxisd rotation_vecY(-M_PI/2, Eigen::Vector3d(0,1,0));  
  Eigen::AngleAxisd rotation_vecZ(0, Eigen::Vector3d(0,0,1));  

  Eigen::Quaterniond q = rotation_vecZ * rotation_vecY * rotation_vecX;    // 左乘

  geometry_msgs::PoseWithCovarianceStamped::Ptr my_init_pose_(new geometry_msgs::PoseWithCovarianceStamped);
  my_init_pose_->pose.pose.position.x = initial_pose_msg_ptr->pose.pose.position.x;
  my_init_pose_->pose.pose.position.y = initial_pose_msg_ptr->pose.pose.position.y;  
  my_init_pose_->pose.pose.position.z = initial_pose_msg_ptr->pose.pose.position.z;
  my_init_pose_->pose.pose.orientation.x = q.x();
  my_init_pose_->pose.pose.orientation.y = q.y();
  my_init_pose_->pose.pose.orientation.z = q.z();
  my_init_pose_->pose.pose.orientation.w = q.w();

  if (my_init_pose_->header.frame_id == map_frame_) {    // rviz显示的global frame是谁,initial_pose_msg_ptr->header.frame_id就是谁
    initial_pose_cov_msg_ = *my_init_pose_;
  } else {
    // get TF from pose_frame to map_frame  
    geometry_msgs::TransformStamped::Ptr TF_pose_to_map_ptr(new geometry_msgs::TransformStamped);
    get_transform(map_frame_, my_init_pose_->header.frame_id, TF_pose_to_map_ptr);

    // transform pose_frame to map_frame
    geometry_msgs::PoseWithCovarianceStamped::Ptr mapTF_initial_pose_msg_ptr(new geometry_msgs::PoseWithCovarianceStamped);
    tf2::doTransform(*my_init_pose_, *mapTF_initial_pose_msg_ptr, *TF_pose_to_map_ptr);
    // mapTF_initial_pose_msg_ptr->header.stamp = initial_pose_msg_ptr->header.stamp;
    initial_pose_cov_msg_ = *mapTF_initial_pose_msg_ptr;
  }
  // if click the initpose again, re init！
  init_pose = false; 
}

void GicpLocalizer::callback_pointsmap(
  const sensor_msgs::PointCloud2::ConstPtr & map_points_msg_ptr)
{
  const auto trans_epsilon = gicp_->getTransformationEpsilon();
  const auto cor_dist = gicp_->getMaxCorrespondenceDistance();
  const auto euc_eps = gicp_->getEuclideanFitnessEpsilon();
  const auto max_iterations = gicp_->getMaximumIterations();

#ifdef USE_GICP_OMP_SPEEDUP
  boost::shared_ptr<pclomp::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>> gicp_new(new pclomp::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>());
#else
  boost::shared_ptr<pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>> gicp_new(new pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ>());
#endif

  gicp_new->setTransformationEpsilon(trans_epsilon);
  gicp_new->setMaxCorrespondenceDistance(cor_dist);
  gicp_new->setEuclideanFitnessEpsilon(euc_eps);
  gicp_new->setMaximumIterations(max_iterations);

  pcl::PointCloud<pcl::PointXYZ>::Ptr map_points_ptr(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromROSMsg(*map_points_msg_ptr, *map_points_ptr);
  gicp_new->setInputTarget(map_points_ptr);
  // create Thread
  // detach
  pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  gicp_new->align(*output_cloud, Eigen::Matrix4f::Identity());

  // swap
  gicp_map_mtx_.lock();
  gicp_ = gicp_new; 
  gicp_map_mtx_.unlock();
} 

void GicpLocalizer::callback_pointcloud( 
  const sensor_msgs::PointCloud2::ConstPtr & sensor_points_sensorTF_msg_ptr)
{
  const auto exe_start_time = std::chrono::system_clock::now();
  // mutex Map
  std::lock_guard<std::mutex> lock(gicp_map_mtx_);

  const std::string sensor_frame = sensor_points_sensorTF_msg_ptr->header.frame_id;
  const auto sensor_ros_time = sensor_points_sensorTF_msg_ptr->header.stamp;

  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> sensor_points_sensorTF_ptr(
    new pcl::PointCloud<pcl::PointXYZ>);

  pcl::fromROSMsg(*sensor_points_sensorTF_msg_ptr, *sensor_points_sensorTF_ptr);
  // pcl::io::savePCDFileASCII("/home/gj/Desktop/1.pcd", *sensor_points_sensorTF_ptr);    // 保存一帧点云 用于测试
  
  // get TF base to sensor
  geometry_msgs::TransformStamped::Ptr TF_base_to_sensor_ptr(new geometry_msgs::TransformStamped);
  get_transform(base_frame_, sensor_frame, TF_base_to_sensor_ptr);

  const Eigen::Affine3d base_to_sensor_affine = tf2::transformToEigen(*TF_base_to_sensor_ptr);
  const Eigen::Matrix4f base_to_sensor_matrix = base_to_sensor_affine.matrix().cast<float>();

  boost::shared_ptr<pcl::PointCloud<pcl::PointXYZ>> sensor_points_baselinkTF_ptr(
    new pcl::PointCloud<pcl::PointXYZ>);

  pcl::transformPointCloud(
    *sensor_points_sensorTF_ptr, *sensor_points_baselinkTF_ptr, base_to_sensor_matrix);
  
  // set input point cloud
  gicp_->setInputSource(sensor_points_baselinkTF_ptr);

  if (gicp_->getInputTarget() == nullptr) { 
    ROS_WARN_STREAM_THROTTLE(1, "No MAP!");
    return;
  }

  // align
  Eigen::Matrix4f initial_pose_matrix;

  if (!init_pose){    // 初始化是false, 进过初始化回调也是false

    // 旋转初值
    // 2022/11/16
    Eigen::AngleAxisd rotation_vecX(-M_PI/2, Eigen::Vector3d(1,0,0));    
    Eigen::AngleAxisd rotation_vecY(-M_PI/2, Eigen::Vector3d(0,1,0));  
    Eigen::AngleAxisd rotation_vecZ(0, Eigen::Vector3d(0,0,1));  
    Eigen::Quaterniond q = rotation_vecZ * rotation_vecY * rotation_vecX;    // 左乘

    geometry_msgs::PoseWithCovarianceStamped::Ptr my_init_pose_(new geometry_msgs::PoseWithCovarianceStamped);
    my_init_pose_->pose.pose.position.x = 0;
    my_init_pose_->pose.pose.position.y = 0;  
    my_init_pose_->pose.pose.position.z = 0;
    my_init_pose_->pose.pose.orientation.x = q.x();
    my_init_pose_->pose.pose.orientation.y = q.y();
    my_init_pose_->pose.pose.orientation.z = q.z();
    my_init_pose_->pose.pose.orientation.w = q.w();
    initial_pose_cov_msg_ = *my_init_pose_;

    Eigen::Affine3d initial_pose_affine;
    tf2::fromMsg(initial_pose_cov_msg_.pose.pose, initial_pose_affine);
    initial_pose_matrix = initial_pose_affine.matrix().cast<float>();
    // for the first time, we don't know the pre_trans, so just use the init_trans, 
    // which means, the delta trans for the second time is 0
    pre_trans = initial_pose_matrix;
    init_pose = true;
  }else
  {
    // use predicted pose as init guess (currently we only impl linear model)
    initial_pose_matrix = pre_trans * delta_trans;    // 匹配初值
  }

  pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  const auto align_start_time = std::chrono::system_clock::now();
  key_value_stdmap_["state"] = "Aligning";
  gicp_->align(*output_cloud, initial_pose_matrix);    //!!! 匹配初值
  key_value_stdmap_["state"] = "Sleeping";
  const auto align_end_time = std::chrono::system_clock::now();
  const double align_time = std::chrono::duration_cast<std::chrono::microseconds>(align_end_time - align_start_time).count() /1000.0;

  const Eigen::Matrix4f result_pose_matrix = gicp_->getFinalTransformation();
  Eigen::Affine3d result_pose_affine;
  result_pose_affine.matrix() = result_pose_matrix.cast<double>();
  const geometry_msgs::Pose result_pose_msg = tf2::toMsg(result_pose_affine);

  const auto exe_end_time = std::chrono::system_clock::now();
  const double exe_time = std::chrono::duration_cast<std::chrono::microseconds>(exe_end_time - exe_start_time).count() / 1000.0;

  const int hasConverged = gicp_->hasConverged();   // 如果两个点云匹配正确的话 该函数返回1
  const float sm_score = gicp_->getFitnessScore();   // 分数越大，配准效果越差

  std::cout << "---: " << sm_score << std::endl; 

  bool is_converged = true;
  static size_t skipping_publish_num = 0;
  if ( hasConverged != true ) { 
  // if ( hasConverged != true || sm_score > sm_score_) { 
    is_converged = false;
    ++skipping_publish_num;
    std::cout << "Not Converged" << std::endl;
  } else { 
    skipping_publish_num = 0;
  }
  // calculate the delta tf from pre_trans to current_trans
  delta_trans = pre_trans.inverse() * result_pose_matrix;

  // 两帧位姿之间的增量
  // Eigen::Vector3f delta_translation = delta_trans.block<3, 1>(0, 3);
  // std::cout<<"delta x: "<<delta_translation(0) << " y: "<<delta_translation(1)<<
  //            " z: "<<delta_translation(2)<<std::endl;

  // Eigen::Matrix3f delta_rotation_matrix = delta_trans.block<3, 3>(0, 0);
  // Eigen::Vector3f delta_euler = delta_rotation_matrix.eulerAngles(2,1,0);
  // std::cout<<"delta yaw: "<<delta_euler(0) << " pitch: "<<delta_euler(1)<<
  //            " roll: "<<delta_euler(2)<<std::endl;

  pre_trans = result_pose_matrix; 
  
  // publish  
  geometry_msgs::PoseStamped result_pose_stamped_msg;
  result_pose_stamped_msg.header.stamp = sensor_ros_time;
  result_pose_stamped_msg.header.frame_id = map_frame_;
  result_pose_stamped_msg.pose = result_pose_msg;

  nav_msgs::Odometry result_odom_stamped_msg;
  result_odom_stamped_msg.header.stamp = sensor_ros_time;
  result_odom_stamped_msg.header.frame_id = map_frame_;
  result_odom_stamped_msg.child_frame_id = base_frame_;
  result_odom_stamped_msg.pose.pose = result_pose_msg;

  if (is_converged) { 
    gicp_pose_pub_.publish(result_pose_stamped_msg);
    gicp_odom_pub_.publish(result_odom_stamped_msg);
  }

  // publish tf(map frame to base frame)
  publish_tf(map_frame_, base_frame_, result_pose_stamped_msg);

  // publish aligned point cloud
  pcl::PointCloud<pcl::PointXYZ>::Ptr sensor_points_mapTF_ptr(new pcl::PointCloud<pcl::PointXYZ>);
  pcl::transformPointCloud(*sensor_points_baselinkTF_ptr, *sensor_points_mapTF_ptr, result_pose_matrix);

  sensor_msgs::PointCloud2 sensor_points_mapTF_msg;
  pcl::toROSMsg(*sensor_points_mapTF_ptr, sensor_points_mapTF_msg);
  sensor_points_mapTF_msg.header.stamp = sensor_ros_time;
  sensor_points_mapTF_msg.header.frame_id = map_frame_;
  sensor_aligned_pose_pub_.publish(sensor_points_mapTF_msg);

  // std_msgs::Float32 exe_time_msg;
  // exe_time_msg.data = exe_time;
  // exe_time_pub_.publish(exe_time_msg); 

  key_value_stdmap_["seq"] = std::to_string(sensor_points_sensorTF_msg_ptr->header.seq);
  key_value_stdmap_["sm_score"] = std::to_string(sm_score);
  key_value_stdmap_["skipping_publish_num"] = std::to_string(skipping_publish_num);

  std::cout << "------------------------------------------------" << std::endl;
  std::cout << "align_time: " << align_time << "ms" << std::endl;
  std::cout << "exe_time: " << exe_time << "ms" << std::endl;
  std::cout << "sm_score: " << sm_score << std::endl;
  std::cout << "skipping_publish_num: " << skipping_publish_num << std::endl;
}

void GicpLocalizer::init_params(){

  private_nh_.getParam("base_frame", base_frame_);
  ROS_INFO("base_frame_id: %s", base_frame_.c_str());

  double trans_epsilon = gicp_->getTransformationEpsilon();   //为终止条件设置最小转换差异
  double cor_dist = gicp_->getMaxCorrespondenceDistance();    //设置对应点对之间的最大距离
  double euc_eps = gicp_->getEuclideanFitnessEpsilon();    //设置收敛条件是均方误差和小于阈值
  int max_iterations = gicp_->getMaximumIterations();

  private_nh_.getParam("trans_epsilon", trans_epsilon);
  private_nh_.getParam("cor_dist", cor_dist);
  private_nh_.getParam("euc_eps", euc_eps);
  private_nh_.getParam("max_iterations", max_iterations);

  map_frame_ = "map";

  gicp_->setTransformationEpsilon(trans_epsilon);
  gicp_->setMaxCorrespondenceDistance(cor_dist);
  gicp_->setEuclideanFitnessEpsilon(euc_eps);
  gicp_->setMaximumIterations(max_iterations);

  ROS_INFO("trans_epsilon: %lf, cor_dist: %lf, euc_eps: %lf, max_iterations: %d", trans_epsilon,
    cor_dist, euc_eps, max_iterations);

  private_nh_.getParam("scan_match_score", sm_score_);
}


bool GicpLocalizer::get_transform(
  const std::string & target_frame, const std::string & source_frame,
  const geometry_msgs::TransformStamped::Ptr & transform_stamped_ptr, const ros::Time & time_stamp)
{
  if (target_frame == source_frame) {
    transform_stamped_ptr->header.stamp = time_stamp;
    transform_stamped_ptr->header.frame_id = target_frame;
    transform_stamped_ptr->child_frame_id = source_frame;
    transform_stamped_ptr->transform.translation.x = 0.0;
    transform_stamped_ptr->transform.translation.y = 0.0;
    transform_stamped_ptr->transform.translation.z = 0.0;
    transform_stamped_ptr->transform.rotation.x = 0.0;
    transform_stamped_ptr->transform.rotation.y = 0.0;
    transform_stamped_ptr->transform.rotation.z = 0.0;
    transform_stamped_ptr->transform.rotation.w = 1.0;
    return true;
  }

  try {
    *transform_stamped_ptr =
      tf2_buffer_.lookupTransform(target_frame, source_frame, time_stamp);
  } catch (tf2::TransformException & ex) {
    ROS_WARN("%s", ex.what());
    ROS_ERROR("Please publish TF %s to %s", target_frame.c_str(), source_frame.c_str());

    transform_stamped_ptr->header.stamp = time_stamp;
    transform_stamped_ptr->header.frame_id = target_frame;
    transform_stamped_ptr->child_frame_id = source_frame;
    transform_stamped_ptr->transform.translation.x = 0.0;
    transform_stamped_ptr->transform.translation.y = 0.0;
    transform_stamped_ptr->transform.translation.z = 0.0;
    transform_stamped_ptr->transform.rotation.x = 0.0;
    transform_stamped_ptr->transform.rotation.y = 0.0;
    transform_stamped_ptr->transform.rotation.z = 0.0;
    transform_stamped_ptr->transform.rotation.w = 1.0;
    return false;
  }
  return true;
}

bool GicpLocalizer::get_transform(
  const std::string & target_frame, const std::string & source_frame,
  const geometry_msgs::TransformStamped::Ptr & transform_stamped_ptr)
{
  if (target_frame == source_frame) {
    transform_stamped_ptr->header.stamp = ros::Time::now();
    transform_stamped_ptr->header.frame_id = target_frame;
    transform_stamped_ptr->child_frame_id = source_frame;
    transform_stamped_ptr->transform.translation.x = 0.0;
    transform_stamped_ptr->transform.translation.y = 0.0;
    transform_stamped_ptr->transform.translation.z = 0.0;
    transform_stamped_ptr->transform.rotation.x = 0.0;
    transform_stamped_ptr->transform.rotation.y = 0.0;
    transform_stamped_ptr->transform.rotation.z = 0.0;
    transform_stamped_ptr->transform.rotation.w = 1.0;
    return true;
  }

  try {
    *transform_stamped_ptr =
      tf2_buffer_.lookupTransform(target_frame, source_frame, ros::Time(0), ros::Duration(1.0));
  } catch (tf2::TransformException & ex) {
    ROS_WARN("%s", ex.what());
    ROS_ERROR("Please publish TF %s to %s", target_frame.c_str(), source_frame.c_str());

    transform_stamped_ptr->header.stamp = ros::Time::now();
    transform_stamped_ptr->header.frame_id = target_frame;
    transform_stamped_ptr->child_frame_id = source_frame;
    transform_stamped_ptr->transform.translation.x = 0.0;
    transform_stamped_ptr->transform.translation.y = 0.0;
    transform_stamped_ptr->transform.translation.z = 0.0;
    transform_stamped_ptr->transform.rotation.x = 0.0;
    transform_stamped_ptr->transform.rotation.y = 0.0;
    transform_stamped_ptr->transform.rotation.z = 0.0;
    transform_stamped_ptr->transform.rotation.w = 1.0;
    return false;
  }
  return true;
}

void GicpLocalizer::publish_tf(
  const std::string & frame_id, const std::string & child_frame_id,
  const geometry_msgs::PoseStamped & pose_msg)
{
  geometry_msgs::TransformStamped transform_stamped;
  transform_stamped.header.frame_id = frame_id;
  transform_stamped.child_frame_id = child_frame_id;
  transform_stamped.header.stamp = pose_msg.header.stamp;

  transform_stamped.transform.translation.x = pose_msg.pose.position.x;
  transform_stamped.transform.translation.y = pose_msg.pose.position.y;
  transform_stamped.transform.translation.z = pose_msg.pose.position.z;

  tf2::Quaternion tf_quaternion;
  tf2::fromMsg(pose_msg.pose.orientation, tf_quaternion);
  transform_stamped.transform.rotation.x = tf_quaternion.x();
  transform_stamped.transform.rotation.y = tf_quaternion.y();
  transform_stamped.transform.rotation.z = tf_quaternion.z();
  transform_stamped.transform.rotation.w = tf_quaternion.w();

  tf2_broadcaster_.sendTransform(transform_stamped);
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "ndt_localizer");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");
    
    GicpLocalizer gicp_localizer(nh, private_nh);

    ros::spin();

    return 0;
}