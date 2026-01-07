cmake_minimum_required(VERSION 3.0.2)
project(rslidar_slam)

add_compile_options(-std=c++14)


find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  image_transport
  pcl_conversions
  pcl_ros
  nav_msgs
  rosbag
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
)

find_package(PCL REQUIRED)
find_package(Boost REQUIRED)
find_package(Ceres REQUIRED)
find_path(GLOG_INCLUDE_DIR glog/logging.h
          PATHS /usr/include /usr/local/include
          DOC "Path to glog include directory")

find_library(GLOG_LIBRARY glog
             PATHS /usr/lib /usr/local/lib /usr/lib/x86_64-linux-gnu
             DOC "Path to glog library")

if(NOT GLOG_INCLUDE_DIR OR NOT GLOG_LIBRARY)
    message(FATAL_ERROR "Glog not found! Please install libgoogle-glog-dev")
endif()

# 创建 IMPORTED target 以兼容 glog::glog 语法
add_library(glog::glog UNKNOWN IMPORTED)
set_target_properties(glog::glog PROPERTIES
    IMPORTED_LOCATION "${GLOG_LIBRARY}"
    INTERFACE_INCLUDE_DIRECTORIES "${GLOG_INCLUDE_DIR}"
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES rslidar_slam
#  CATKIN_DEPENDS cv_bridge geometry_msgs image_transport nav_msgs rosbag roscpp rospy sensor_msgs std_msgs tf
#  DEPENDS system_lib
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${GLOG_INCLUDE_DIRS}
  ${CERES_INCLUDE_DIRS}
)

add_executable(feature_node src/feature.cpp)
target_link_libraries(feature_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  glog::glog
)


add_executable(odometry_node_icp src/lidarOdometryIcp.cpp)
target_link_libraries(odometry_node_icp
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  glog::glog
)


add_executable(odometry_node_scan2scan src/lidarOdometryScan2Scan.cpp)
target_link_libraries(odometry_node_scan2scan
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  glog::glog
  ${CERES_LIBRARIES}
)