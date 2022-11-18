cmake_minimum_required(VERSION 2.8.3)
project(ndt_localizer)

set(CMAKE_CXX_STANDARD 11)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        tf2
        tf2_ros
        tf2_geometry_msgs
        std_msgs
        geometry_msgs
        sensor_msgs
        nav_msgs
        diagnostic_msgs
        pcl_conversions
        pcl_ros
        message_generation

        ndt_omp                 #openmp 1
        )

generate_messages(
        DEPENDENCIES
        std_msgs
)

find_package(PCL REQUIRED QUIET)

find_package(OpenMP)            #openmp 2
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

catkin_package(
        INCLUDE_DIRS include
        CATKIN_DEPENDS
        roscpp
        tf2
        tf2_ros
        tf2_geometry_msgs
        geometry_msgs
        sensor_msgs
        nav_msgs
        diagnostic_msgs
        pcl_conversions
        pcl_ros
        message_generation
)

include_directories(include ${catkin_INCLUDE_DIRS})

###########
## Build ##
###########
include_directories(include ${catkin_INCLUDE_DIRS})
SET(CMAKE_CXX_FLAGS "-O2 -g -Wall ${CMAKE_CXX_FLAGS}")

add_executable(voxel_grid_filter nodes/points_downsampler.cpp)
add_dependencies(voxel_grid_filter ${catkin_EXPORTED_TARGETS})
target_link_libraries(voxel_grid_filter ${catkin_LIBRARIES})

add_executable(map_loader nodes/map_loader.cpp)
target_link_libraries(map_loader ${catkin_LIBRARIES} ${PCL_LIBRARIES})

add_executable(ndt_localizer_node nodes/ndt.cpp)
# target_link_libraries(ndt_localizer_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})
target_link_libraries(ndt_localizer_node ${catkin_LIBRARIES} ${PCL_LIBRARIES} ndt_omp)        # openmp 3


add_executable(gicp_localizer_node nodes/gicp.cpp)
target_link_libraries(gicp_localizer_node ${catkin_LIBRARIES} ${PCL_LIBRARIES} ndt_omp)  