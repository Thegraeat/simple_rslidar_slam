#include "header.h"

class Odometry
{
private:
    ros::NodeHandle nh;
    ros::Subscriber subPlaneCloud;
    ros::Publisher pubOdomtery;
    ros::Publisher pubPath;
    ros::Publisher pubMapCloud;

    pcl::PointCloud<pcl::PointXYZI>::Ptr lastCloud;
    pcl::PointCloud<pcl::PointXYZI>::Ptr mapCloud;
    
    pcl::NormalDistributionsTransform<pcl::PointXYZI, pcl::PointXYZI> ndt;
    pcl::IterativeClosestPoint<pcl::PointXYZI, pcl::PointXYZI> icp;

    pcl::VoxelGrid<pcl::PointXYZI> downSizeFilter;

    bool firstFram = true;
    Eigen::Matrix4f T_w_curr = Eigen::Matrix4f::Identity();
    nav_msgs::Path path;

public:
    Odometry();
    ~Odometry();

    void planeHandler(const sensor_msgs::PointCloud2::ConstPtr& planeMsg);
};

void Odometry::planeHandler(const sensor_msgs::PointCloud2::ConstPtr& planeMsg)
{
    pcl::PointCloud<pcl::PointXYZI>::Ptr currentCloud(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::fromROSMsg(*planeMsg, *currentCloud);

    if(currentCloud->empty())
    {
        LOG(WARNING) << "current cloud is empty";
        return;
    }

    if (firstFram)
    {
        *lastCloud = *currentCloud;
        firstFram = false;

        LOG(INFO) << "First frame recived";
        return;
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr sourceCloud(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::PointCloud<pcl::PointXYZI>::Ptr targetCloud(new pcl::PointCloud<pcl::PointXYZI>());

    downSizeFilter.setInputCloud(currentCloud);
    downSizeFilter.filter(*sourceCloud);
    downSizeFilter.setInputCloud(lastCloud);
    downSizeFilter.filter(*targetCloud);
    
    ndt.setInputCloud(sourceCloud);
    ndt.setInputTarget(targetCloud);
    pcl::PointCloud<pcl::PointXYZI> tmpCloud;
    ndt.align(tmpCloud);

    Eigen::Matrix4f ndtResult  = Eigen::Matrix4f::Identity();
    if (ndt.hasConverged())
    {
        ndtResult = ndt.getFinalTransformation();
    }else
    {
        LOG(WARNING) << "NDT did not converge!";
    }
    
    icp.setInputSource(currentCloud);
    icp.setInputTarget(lastCloud);
    icp.align(tmpCloud, ndtResult);

    Eigen::Matrix4f icpResult = ndtResult;
    if(icp.hasConverged())
    {
        icpResult = icp.getFinalTransformation();
    }else
    {
        LOG(WARNING) << "ICP did not converge!";
    }

    Eigen::Matrix4f T_last_curr = icpResult;
    Eigen::Matrix4f T_w_last = T_w_curr;
    T_w_curr = T_w_last * T_last_curr;

    Eigen::Vector3f t_w_curr = T_w_curr.block<3, 1>(0, 3);
    Eigen::Matrix3f R_w_curr = T_w_curr.block<3, 3>(0, 0);
    Eigen::Quaternionf q_w_curr(R_w_curr);

    nav_msgs::Odometry odom;
    odom.header = planeMsg->header;
    odom.header.frame_id = "map";
    odom.child_frame_id = "lidar";
    odom.pose.pose.position.x = t_w_curr.x();
    odom.pose.pose.position.y = t_w_curr.y();
    odom.pose.pose.position.z = t_w_curr.z();
    odom.pose.pose.orientation.x = q_w_curr.x();
    odom.pose.pose.orientation.y = q_w_curr.y();
    odom.pose.pose.orientation.z = q_w_curr.z();
    odom.pose.pose.orientation.w = q_w_curr.w();
    pubOdomtery.publish(odom);


    geometry_msgs::PoseStamped pose;
    pose.header = planeMsg->header;
    pose.header.frame_id = "map";
    pose.pose = odom.pose.pose;
    path.poses.push_back(pose);
    pubPath.publish(path);


    *lastCloud = *currentCloud;

    pcl::PointCloud<pcl::PointXYZI>::Ptr currInWorld(new pcl::PointCloud<pcl::PointXYZI>());
    pcl::transformPointCloud(*currentCloud, *currInWorld, T_w_curr);
    *mapCloud += *currInWorld;

    downSizeFilter.setInputCloud(mapCloud);
    downSizeFilter.filter(*mapCloud);

    sensor_msgs::PointCloud2 mapMsg;
    pcl::toROSMsg(*mapCloud, mapMsg);
    mapMsg.header = planeMsg->header;
    mapMsg.header.frame_id = "map";
    pubMapCloud.publish(mapMsg);

    LOG(INFO) << "Frame processed. Trans: [" 
              << t_w_curr.x() << ", " << t_w_curr.y() << ", " << t_w_curr.z() << "]";

}

Odometry::Odometry()
    :lastCloud(new pcl::PointCloud<pcl::PointXYZI>()),
     mapCloud(new pcl::PointCloud<pcl::PointXYZI>())
{
    subPlaneCloud = nh.subscribe<sensor_msgs::PointCloud2>("/planeCloud", 10, &Odometry::planeHandler, this);
    pubOdomtery   = nh.advertise<nav_msgs::Odometry>("/Odomtery", 10);
    pubPath       = nh.advertise<nav_msgs::Path>("/path", 10);
    pubMapCloud   = nh.advertise<sensor_msgs::PointCloud2>("/map",10);

    ndt.setResolution(1.5f);
    ndt.setStepSize(0.1f);
    ndt.setTransformationEpsilon(0.01f);
    ndt.setMaximumIterations(30);

    icp.setMaxCorrespondenceDistance(1.0);
    icp.setMaximumIterations(50);
    icp.setTransformationEpsilon(1e-6);
    icp.setEuclideanFitnessEpsilon(1e-6);

    downSizeFilter.setLeafSize(0.2, 0.2, 0.2);

}

Odometry::~Odometry()
{
}


int main(int argc, char **argv)
{
    google::InitGoogleLogging(argv[0]);
    FLAGS_logtostderr = true;

    LOG(INFO) << "odometry node started";

    ros::init(argc, argv, "odometry_node");

    Odometry odometry;

    ros::spin();
    return 0;
}
