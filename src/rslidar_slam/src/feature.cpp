#include "header.h"

class Feature
{
private:
    ros::NodeHandle nh;
    ros::Subscriber subLidarFramCloud;   
    ros::Publisher pubPlaneCloud;    
    ros::Publisher pubOriginCloud;
    

public:
    Feature();
    ~Feature();
    void cloudHandler(const sensor_msgs::PointCloud2::ConstPtr& cloudMsg);
};



void Feature::cloudHandler(const sensor_msgs::PointCloud2::ConstPtr& cloudMsg)
{
    auto rsLidarCloud = std::make_shared<pcl::PointCloud<RsPointXYZIRT>>();
    pcl::fromROSMsg(*cloudMsg, *rsLidarCloud);

    std::vector<pcl::PointCloud<RsPointXYZIRT>> scanLines(N_SCAN);
    std::vector<std::vector<double>> curvatures(N_SCAN);

    //将每一帧点云中的每个点，放在对应的ring中
    for (const auto& pt : rsLidarCloud->points)
    {
        double dis = std::sqrt(pt.x * pt.x + pt.y * pt.y + pt.z *pt.z);
        if (dis < min_distance || dis > max_distance)
            continue;
        
        if(pt.ring < 0 || pt.ring >= N_SCAN)
            continue;
        
        scanLines[pt.ring].push_back(pt);
    }

    //根据ring计算曲率
    for (size_t i = 0; i < N_SCAN; ++i)
    {
        size_t numRingPoints = scanLines[i].size();
        curvatures[i].resize(numRingPoints, 0.0);

        if(numRingPoints < 10)
            continue;

        for (size_t j = 5; j < numRingPoints - 5; ++j) 
        {
            float diffX = 0, diffY = 0, diffZ = 0;
            for (int k = -5; k <= 5; ++k) 
            {
                if (k == 0) 
                    continue;
                diffX += scanLines[i].points[j + k].x;
                diffY += scanLines[i].points[j + k].y;
                diffZ += scanLines[i].points[j + k].z;
            }
            diffX -= 10.0f * scanLines[i].points[j].x;
            diffY -= 10.0f * scanLines[i].points[j].y;
            diffZ -= 10.0f * scanLines[i].points[j].z;
            curvatures[i][j] = diffX * diffX + diffY * diffY + diffZ * diffZ;
        }
    }

    //筛选平面点
    auto planeCloud = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
    
    for (size_t i = 0; i < N_SCAN; ++i)
    {
        for (size_t j = 5; j < scanLines[i].size() - 5; ++j)
        {
            if (curvatures[i][j] < plan_thres)
            {
                pcl::PointXYZI pt;
                pt.x = scanLines[i].points[j].x;
                pt.y = scanLines[i].points[j].y;
                pt.z = scanLines[i].points[j].z;
                pt.intensity = scanLines[i].points[j].intensity;
                planeCloud->push_back(pt);
            }
        }
        
    }

    //发布平面点
    auto planeMsg = std::make_shared<sensor_msgs::PointCloud2>();
    pcl::toROSMsg(*planeCloud, *planeMsg);
    planeMsg->header = cloudMsg->header;
    planeMsg->header.frame_id = "map";
    pubPlaneCloud.publish(*planeMsg);

    //发布原始点云
    auto originCloud = *cloudMsg;
    originCloud.header.frame_id = "map";
    pubOriginCloud.publish(originCloud);

    LOG_EVERY_N(INFO, 10)   << "origin point cloud:"
                            << originCloud.width * originCloud.height << "," 
                            << "plane point cloud:"
                            << planeCloud->size();

}

Feature::Feature()
{
    nh.param<std::string>("sensor_topic/lidar_topic", lidarTopic,"rslidar_points");
    nh.param<std::string>("lidar/lidar_type", lidarType,"rs");
    nh.param<int>("lidar/N_SCAN", N_SCAN, 16);
    nh.param<double>("lidar/min_distance", min_distance, 0.5);
    nh.param<double>("lidar/max_distance", max_distance, 20);
    nh.param<double>("feature/plan_thres", plan_thres, 0.1);

    subLidarFramCloud  = nh.subscribe<sensor_msgs::PointCloud2>(lidarTopic, 10, &Feature::cloudHandler, this);
    pubPlaneCloud      = nh.advertise<sensor_msgs::PointCloud2>("/planeCloud", 10);
    pubOriginCloud     = nh.advertise<sensor_msgs::PointCloud2>("/originCloud", 10);

}

Feature::~Feature()
{
}


int main(int argc, char **argv)
{
    google::InitGoogleLogging(argv[0]);
    FLAGS_logtostderr = true;

    LOG(INFO) << "feature node started";

    ros::init(argc, argv, "feature_node");

    Feature feature;

    ros::spin();
    return 0;
}
