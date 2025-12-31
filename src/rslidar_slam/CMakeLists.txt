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
find_package(glog 0.6.0 REQUIRED)

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
)

add_executable(feature_node src/feature.cpp)
target_link_libraries(feature_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${GLOG_LIBRARIES}
  glog::glog
)


add_executable(odometry_node src/odometry.cpp)
target_link_libraries(odometry_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  ${GLOG_LIBRARIES}
  glog::glog
)