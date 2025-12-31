#ifndef HEADER_H
#define HEADER_H

//cpp
#include <iostream>
#include <string>
#include <deque>
#include <memory>
#include <mutex>
#include <vector>
#include <math.h>

//ros
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>

//pcl
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/ndt.h>

//glog
#include <glog/logging.h>

//Eigen
#include <Eigen/Dense>
#include <Eigen/Geometry>

std::string lidarTopic;
std::string lidarType;
double min_distance, max_distance, plan_thres;
int N_SCAN;


// rslidar的点云格式
struct RsPointXYZIRT {
    PCL_ADD_POINT4D;
    float  intensity;
    uint16_t ring = 0;
    double timestamp = 0;

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(RsPointXYZIRT,
                                  (float, x, x)(float, y, y)(float, z, z)
                                  (float, intensity, intensity)(uint16_t, ring, ring)(double, timestamp, timestamp))



#endif //HEADER_H
