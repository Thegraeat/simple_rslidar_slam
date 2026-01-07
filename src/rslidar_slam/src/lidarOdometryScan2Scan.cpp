#include "header.h"

struct planeCost
{
    const Eigen::Vector3d _point_curr, _point_last, _plane_normal;

    planeCost(const Eigen::Vector3d& point_curr,
              const Eigen::Vector3d& point_last,
              const Eigen::Vector3d& plane_normal)
        : _point_curr(point_curr), _point_last(point_last), _plane_normal(plane_normal) {}

    template <typename T>
    bool operator()(const T* const q, const T* const t, T* residual) const
    {
        Eigen::Matrix<T, 3, 1> point_currt {T(_point_curr.x()), T(_point_curr.y()), T(_point_curr.z())};
        Eigen::Matrix<T, 3, 1> point_last {T(_point_last.x()), T(_point_last.y()), T(_point_last.z())};
        Eigen::Matrix<T, 3, 1> plane_normalt {T(_plane_normal.x()), T(_plane_normal.y()), T(_plane_normal.z())};

        Eigen::Quaternion<T> rot_qt(q[3], q[0], q[1], q[2]);
        Eigen::Matrix<T, 3, 1> t_t {t[0], t[1], t[2]};

        Eigen::Matrix<T, 3, 1> point_transformed = rot_qt * point_currt + t_t;

        residual[0] = (point_transformed - point_last).dot(plane_normalt);
        return true;
    }
};

class lidarOdometryScan2Scan
{
private:
    ros::NodeHandle nh;
    ros::Subscriber subPlanePoints;
    ros::Publisher pubLaserOdometry;
    ros::Publisher pubLaserPath;
    ros::Publisher pubMap;
    ros::Timer timer;
    nav_msgs::Path laserPath;

    bool firstFrame;
    std::mutex mtxQueue;

    Eigen::Vector3d t_curr_last;
    Eigen::Quaterniond q_curr_last;

    Eigen::Quaterniond q_map_curr;
    Eigen::Vector3d t_map_curr;

    Eigen::Quaterniond q_map_last;
    Eigen::Vector3d t_map_last;

    std::queue<sensor_msgs::PointCloud2ConstPtr> planePointQueue;

    pcl::PointCloud<pcl::PointXYZI>::Ptr mapPoints;
    pcl::PointCloud<pcl::PointXYZI>::Ptr lastPlanePoints;
    pcl::PointCloud<pcl::PointXYZI>::Ptr currentPlanePoints;

    pcl::VoxelGrid<pcl::PointXYZI> downSizeFilterMap;

    double planeMaxThreshold;

    double para_q[4] = {0, 0, 0, 1}; 
    double para_t[3] = {0, 0, 0};
    
public:
    lidarOdometryScan2Scan();
    ~lidarOdometryScan2Scan();

    void run();
    void frameScan2Scan();
    void publishOdometry();
    void transform2Last(pcl::PointXYZI const *point_in, pcl::PointXYZI* point_out);
    void planePointsHandler(const sensor_msgs::PointCloud2ConstPtr& planePointsMsg);
};

void lidarOdometryScan2Scan::transform2Last(pcl::PointXYZI const *point_in, pcl::PointXYZI* point_out)
{
    Eigen::Vector3d pt_in(point_in->x, point_in->y, point_in->z);
    Eigen::Vector3d pt_out = q_curr_last * pt_in + t_curr_last;
    point_out->x = pt_out.x();
    point_out->y = pt_out.y();
    point_out->z = pt_out.z();
}

void lidarOdometryScan2Scan::publishOdometry()
{
    
}

void lidarOdometryScan2Scan::frameScan2Scan()
{
    ceres::LossFunction *loss_function = new ceres::HuberLoss(0.1);
    ceres::LocalParameterization *q_parameterization = new ceres::EigenQuaternionParameterization();
    ceres::Problem problem;
    problem.AddParameterBlock(para_q,4,q_parameterization);
    problem.AddParameterBlock(para_t,3);

    pcl::KdTreeFLANN<pcl::PointXYZI> kdtree;
    if (lastPlanePoints->points.size() > 10)
    {
        kdtree.setInputCloud(lastPlanePoints);
        for (size_t i = 0; i < 2; i++)
        {
            for (size_t j = 0; j < currentPlanePoints->points.size(); j++)
            {
                pcl::PointXYZI pointSeed;
                transform2Last(&currentPlanePoints->points[j], &pointSeed);

                std::vector<int> pointSearchInd;
                std::vector<float> pointSearchSqDis;
                kdtree.nearestKSearch(pointSeed, 1, pointSearchInd, pointSearchSqDis);

                int p_id = pointSearchInd[0];
                std::vector<int> planeInd;
                std::vector<float> planeSqDis;
                kdtree.nearestKSearch(lastPlanePoints->points[p_id], 5, planeInd, planeSqDis);

                if (planeInd.size() < 5)
                    continue;
                
            }
            
        }
        
    }
    else
    {
        LOG(WARNING) << "last plane points is too small";
        return;
    }
    
}

void lidarOdometryScan2Scan::run()
{
    if (planePointQueue.empty())
        return;
    
    mtxQueue.lock();
    pcl::fromROSMsg(*planePointQueue.front(), *currentPlanePoints);
    planePointQueue.pop();
    mtxQueue.unlock();

    if (firstFrame)
    {
        lastPlanePoints = currentPlanePoints;
        *mapPoints += *lastPlanePoints;
        firstFrame = false;
        return;
    }

    frameScan2Scan();

    *mapPoints += *currentPlanePoints;
    lastPlanePoints = currentPlanePoints;

    publishOdometry();
}

void lidarOdometryScan2Scan::planePointsHandler(const sensor_msgs::PointCloud2::ConstPtr& planePointsMsg)
{
    std::lock_guard<std::mutex> lock(mtxQueue);
    planePointQueue.push(planePointsMsg);
}

lidarOdometryScan2Scan::lidarOdometryScan2Scan()
 : currentPlanePoints(new pcl::PointCloud<pcl::PointXYZI>()),
   lastPlanePoints(new pcl::PointCloud<pcl::PointXYZI>()),
   mapPoints(new pcl::PointCloud<pcl::PointXYZI>()),
   firstFrame(true),
   q_curr_last(Eigen::Quaterniond::Identity()),
   t_curr_last(Eigen::Vector3d::Zero())

{
    subPlanePoints = nh.subscribe<sensor_msgs::PointCloud2>("/planeCloud", 100, &lidarOdometryScan2Scan::planePointsHandler, this);

    pubLaserOdometry = nh.advertise<nav_msgs::Odometry>("/laser_odom_scan2scan", 100);
    pubLaserPath = nh.advertise<nav_msgs::Path>("/laser_path_scan2scan", 100);
    pubMap = nh.advertise<sensor_msgs::PointCloud2>("/map_scan2scan", 100);

    timer = nh.createTimer(ros::Duration(0.1), [&](const ros::TimerEvent&){run();});

}

lidarOdometryScan2Scan::~lidarOdometryScan2Scan()
{
}


int main(int argc, char **argv)
{
    google::InitGoogleLogging(argv[0]);
    FLAGS_logtostderr = true;

    LOG(INFO) << "lidar odometry scan2scan node started";

    ros::init(argc, argv, "odometry_node_scan2scan");

    lidarOdometryScan2Scan odometryScan2Scan;

    ros::spin();
    return 0;
}