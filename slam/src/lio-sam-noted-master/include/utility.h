#pragma once
#ifndef _UTILITY_LIDAR_ODOMETRY_H_
#define _UTILITY_LIDAR_ODOMETRY_H_

#include <ros/ros.h>

#include <std_msgs/Header.h>
#include <std_msgs/Float64MultiArray.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/NavSatFix.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <std_msgs/Bool.h>

//#include <opencv/cv.h>
#include <opencv2/opencv.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/impl/search.hpp>
#include <pcl/range_image/range_image.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/registration/icp.h>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h> 
#include <pcl_conversions/pcl_conversions.h>

#include <tf/LinearMath/Quaternion.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_broadcaster.h>
 
#include <vector>
#include <cmath>
#include <algorithm>
#include <queue>
#include <deque>
#include <iostream>
#include <fstream>
#include <ctime>
#include <cfloat>
#include <iterator>
#include <sstream>
#include <string>
#include <limits>
#include <iomanip>
#include <array>
#include <thread>
#include <mutex>
#include <experimental/filesystem> 
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>

using namespace std;
namespace fs = experimental::filesystem;
typedef pcl::PointXYZI PointType;

enum class SensorType { VELODYNE, OUSTER };
enum class DataType {CUS0, CUS1, CUS2, CUS3, CUS4, CUS5};
class ParamServer
{
public:

    ros::NodeHandle nh;

    std::string robot_id;

    //Topics
    string pointCloudTopic;
    string imuTopic;
    string odomTopic;
    string gpsTopic;

    //Frames
    string lidarFrame;
    string baselinkFrame;
    string odometryFrame;
    string mapFrame;

    // GPS Settings
    bool useImuHeadingInitialization;
    bool useGpsElevation;
    float gpsCovThreshold;
    float poseCovThreshold;
    bool GPS_Trigger;

    // Save pcd
    bool savePCD;
    string savePCDDirectory;

    // Lidar Sensor Configuration
    SensorType sensor;
    DataType datatype;
    int N_SCAN;
    int Horizon_SCAN;
    int downsampleRate;
    float lidarMinRange;
    float lidarMaxRange;

    // IMU
    float imuAccNoise;
    float imuGyrNoise;
    float imuAccBiasN;
    float imuGyrBiasN;
    float imuGravity;
    float imuRPYWeight;
    vector<double> extRotV;
    vector<double> extRPYV;
    vector<double> extTransV;
    Eigen::Matrix3d extRot;
    Eigen::Matrix3d extRPY;
    Eigen::Vector3d extTrans;
    Eigen::Quaterniond extQRPY;

    // LOAM
    float edgeThreshold;
    float surfThreshold;
    int edgeFeatureMinValidNum;
    int surfFeatureMinValidNum;

    // voxel filter paprams
    float odometrySurfLeafSize;
    float mappingCornerLeafSize;
    float mappingSurfLeafSize ;

    float z_tollerance; 
    float rotation_tollerance;

    // CPU Params
    int numberOfCores;
    double mappingProcessInterval;

    // Surrounding map
    float surroundingkeyframeAddingDistThreshold; 
    float surroundingkeyframeAddingAngleThreshold; 
    float surroundingKeyframeDensity;
    float surroundingKeyframeSearchRadius;
    
    // Loop closure
    bool loopClosureEnableFlag;
    int loopClosureDetectInterval;
    int loopClosureDetectStartTime;
    int surroundingKeyframeSize;
    float historyKeyframeSearchRadius;
    float historyKeyframeSearchTimeDiff;
    int   historyKeyframeSearchNum;
    float historyKeyframeFitnessScore;
    float loopFactorWeight;

    // global map visualization radius
    float globalMapVisualizationSearchRadius;
    float globalMapVisualizationPoseDensity;
    float globalMapVisualizationLeafSize;

    string readBagDirectory;

    float imuFrequency;
    float lidarFrequency;
    float readBagFrequency;
    float odomThreshold;
    bool TranslationCompensation;
    int dropFirstLidarMsgNum;
    int dropLastLidarMsgNum;
    int IgnoredRingNumeberFrom_0_Range;
    vector<double> IgnoredRingNumeberSingle;
    bool GpsInitialFlag = false;
    float gpsAddInterval;
    bool PoseDataFileFirstFlag = false;

    // debug gps weight
    double gps_noise;
    ParamServer()
    {
        nh.param<std::string>("/robot_id", robot_id, "roboat");

        nh.param<std::string>("lio_sam/pointCloudTopic", pointCloudTopic, "points_raw");
        nh.param<std::string>("lio_sam/imuTopic", imuTopic, "imu_correct");
        nh.param<std::string>("lio_sam/odomTopic", odomTopic, "odometry/imu");
        nh.param<std::string>("lio_sam/gpsTopic", gpsTopic, "odometry/gps");

        nh.param<std::string>("lio_sam/lidarFrame", lidarFrame, "base_link");
        nh.param<std::string>("lio_sam/baselinkFrame", baselinkFrame, "base_link");
        nh.param<std::string>("lio_sam/odometryFrame", odometryFrame, "odom");
        nh.param<std::string>("lio_sam/mapFrame", mapFrame, "map");

        nh.param<bool>("lio_sam/useImuHeadingInitialization", useImuHeadingInitialization, false);
        nh.param<bool>("lio_sam/GPS_Trigger", GPS_Trigger, false);
        nh.param<bool>("lio_sam/useGpsElevation", useGpsElevation, false);
        nh.param<float>("lio_sam/gpsCovThreshold", gpsCovThreshold, 2.0);
        nh.param<float>("lio_sam/poseCovThreshold", poseCovThreshold, 25.0);


        nh.param<bool>("lio_sam/savePCD", savePCD, false);
        nh.param<std::string>("lio_sam/savePCDDirectory", savePCDDirectory, "/Downloads/LOAM/");

        nh.param<std::string>("lio_sam/readBagDirectory", readBagDirectory, "");

        nh.param<float>("lio_sam/imuFrequency", imuFrequency, 100.0);
        nh.param<float>("lio_sam/lidarFrequency", lidarFrequency, 10.0);
        nh.param<float>("lio_sam/readBagFrequency", readBagFrequency, 1);
        nh.param<float>("lio_sam/odomThreshold", odomThreshold, 1.0);
        nh.param<int>("lio_sam/dropFirstLidarMsgNum", dropFirstLidarMsgNum, 0);
        nh.param<int>("lio_sam/dropLastLidarMsgNum", dropLastLidarMsgNum, 0);
        nh.param<int>("lio_sam/IgnoredRingNumeberFrom_0_Range", IgnoredRingNumeberFrom_0_Range, 0);
        nh.param<bool>("lio_sam/TranslationCompensation", TranslationCompensation, false);

        std::string sensorStr;
        nh.param<std::string>("lio_sam/sensor", sensorStr, "");
        if (sensorStr == "velodyne")
        {
            sensor = SensorType::VELODYNE;
        }
        else if (sensorStr == "ouster")
        {
            sensor = SensorType::OUSTER;
        }
        else
        {
            ROS_ERROR_STREAM(
                "Invalid sensor type (must be either 'velodyne' or 'ouster'): " << sensorStr);
            ros::shutdown();
        }

        std::string datatypeStr;
        nh.param<std::string>("lio_sam/DataType", datatypeStr, "");
        if (datatypeStr == "CUS0")
            datatype = DataType::CUS0;
        else if (datatypeStr == "CUS1")
            datatype = DataType::CUS1;
        else if (datatypeStr == "CUS2")
            datatype = DataType::CUS2; 
        else if (datatypeStr == "CUS3")
            datatype = DataType::CUS3;
        else if (datatypeStr == "CUS4")
            datatype = DataType::CUS4;
        else if (datatypeStr == "CUS5")
            datatype = DataType::CUS5;
        else
        {
            ROS_ERROR_STREAM("Invalid CUS type, shut down " << sensorStr);
            ros::shutdown();
        }


        nh.param<int>("lio_sam/N_SCAN", N_SCAN, 16);
        nh.param<int>("lio_sam/Horizon_SCAN", Horizon_SCAN, 1800);
        nh.param<int>("lio_sam/downsampleRate", downsampleRate, 1);
        nh.param<float>("lio_sam/lidarMinRange", lidarMinRange, 1.0);
        nh.param<float>("lio_sam/lidarMaxRange", lidarMaxRange, 1000.0);

        nh.param<float>("lio_sam/imuAccNoise", imuAccNoise, 0.01);
        nh.param<float>("lio_sam/imuGyrNoise", imuGyrNoise, 0.001);
        nh.param<float>("lio_sam/imuAccBiasN", imuAccBiasN, 0.0002);
        nh.param<float>("lio_sam/imuGyrBiasN", imuGyrBiasN, 0.00003);
        nh.param<float>("lio_sam/imuGravity", imuGravity, 9.80511);
        nh.param<float>("lio_sam/imuRPYWeight", imuRPYWeight, 0.01);
        nh.param<vector<double>>("lio_sam/extrinsicRot", extRotV, vector<double>());
        nh.param<vector<double>>("lio_sam/extrinsicRPY", extRPYV, vector<double>());
        nh.param<vector<double>>("lio_sam/extrinsicTrans", extTransV, vector<double>());
        nh.param<vector<double>>("lio_sam/IgnoredRingNumeberSingle", IgnoredRingNumeberSingle, vector<double>());
        extRot = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(extRotV.data(), 3, 3);
        extRPY = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(extRPYV.data(), 3, 3);
        extTrans = Eigen::Map<const Eigen::Matrix<double, -1, -1, Eigen::RowMajor>>(extTransV.data(), 3, 1);
        extQRPY = Eigen::Quaterniond(extRPY);

        nh.param<float>("lio_sam/edgeThreshold", edgeThreshold, 0.1);
        nh.param<float>("lio_sam/surfThreshold", surfThreshold, 0.1);
        nh.param<int>("lio_sam/edgeFeatureMinValidNum", edgeFeatureMinValidNum, 10);
        nh.param<int>("lio_sam/surfFeatureMinValidNum", surfFeatureMinValidNum, 100);

        nh.param<float>("lio_sam/odometrySurfLeafSize", odometrySurfLeafSize, 0.2);
        nh.param<float>("lio_sam/mappingCornerLeafSize", mappingCornerLeafSize, 0.2);
        nh.param<float>("lio_sam/mappingSurfLeafSize", mappingSurfLeafSize, 0.2);

        nh.param<float>("lio_sam/z_tollerance", z_tollerance, FLT_MAX);
        nh.param<float>("lio_sam/rotation_tollerance", rotation_tollerance, FLT_MAX);

        nh.param<int>("lio_sam/numberOfCores", numberOfCores, 2);
        nh.param<double>("lio_sam/mappingProcessInterval", mappingProcessInterval, 0.15);

        nh.param<float>("lio_sam/surroundingkeyframeAddingDistThreshold", surroundingkeyframeAddingDistThreshold, 1.0);
        nh.param<float>("lio_sam/surroundingkeyframeAddingAngleThreshold", surroundingkeyframeAddingAngleThreshold, 0.2);
        nh.param<float>("lio_sam/surroundingKeyframeDensity", surroundingKeyframeDensity, 1.0);
        nh.param<float>("lio_sam/surroundingKeyframeSearchRadius", surroundingKeyframeSearchRadius, 50.0);

        nh.param<bool>("lio_sam/loopClosureEnableFlag", loopClosureEnableFlag, false);
        nh.param<int>("lio_sam/loopClosureDetectInterval", loopClosureDetectInterval, 5);
        nh.param<int>("lio_sam/loopClosureDetectStartTime", loopClosureDetectStartTime, 1000);
        
        nh.param<int>("lio_sam/surroundingKeyframeSize", surroundingKeyframeSize, 50);
        nh.param<float>("lio_sam/historyKeyframeSearchRadius", historyKeyframeSearchRadius, 10.0);
        nh.param<float>("lio_sam/historyKeyframeSearchTimeDiff", historyKeyframeSearchTimeDiff, 30.0);
        nh.param<int>("lio_sam/historyKeyframeSearchNum", historyKeyframeSearchNum, 25);
        nh.param<float>("lio_sam/historyKeyframeFitnessScore", historyKeyframeFitnessScore, 0.3);
        nh.param<float>("lio_sam/loopFactorWeight", loopFactorWeight, 1.0);

        nh.param<float>("lio_sam/globalMapVisualizationSearchRadius", globalMapVisualizationSearchRadius, 1e3);
        nh.param<float>("lio_sam/globalMapVisualizationPoseDensity", globalMapVisualizationPoseDensity, 10.0);
        nh.param<float>("lio_sam/globalMapVisualizationLeafSize", globalMapVisualizationLeafSize, 1.0);

        nh.param<float>("lio_sam/gpsAddInterval", gpsAddInterval, 2.0);

        nh.param<double>("lio_sam/gps_noise", gps_noise, 1e-6);
        usleep(100);
    }

    


sensor_msgs::PointCloud2 publishCloud(ros::Publisher *thisPub, pcl::PointCloud<PointType>::Ptr thisCloud, ros::Time thisStamp, std::string thisFrame)
{
    sensor_msgs::PointCloud2 tempCloud;
    pcl::toROSMsg(*thisCloud, tempCloud);
    tempCloud.header.stamp = thisStamp;
    tempCloud.header.frame_id = thisFrame;
    if (thisPub->getNumSubscribers() != 0)
        thisPub->publish(tempCloud);
    return tempCloud;
}

template<typename T>
double ROS_TIME(T msg)
{
    return msg->header.stamp.toSec();
}


template<typename T>
void imuAngular2rosAngular(sensor_msgs::Imu *thisImuMsg, T *angular_x, T *angular_y, T *angular_z)
{
    *angular_x = thisImuMsg->angular_velocity.x;
    *angular_y = thisImuMsg->angular_velocity.y;
    *angular_z = thisImuMsg->angular_velocity.z;
}


template<typename T>
void imuAccel2rosAccel(sensor_msgs::Imu *thisImuMsg, T *acc_x, T *acc_y, T *acc_z)
{
    *acc_x = thisImuMsg->linear_acceleration.x;
    *acc_y = thisImuMsg->linear_acceleration.y;
    *acc_z = thisImuMsg->linear_acceleration.z;
}


template<typename T>
void imuRPY2rosRPY(sensor_msgs::Imu *thisImuMsg, T *rosRoll, T *rosPitch, T *rosYaw)
{
    double imuRoll, imuPitch, imuYaw;
    tf::Quaternion orientation;
    tf::quaternionMsgToTF(thisImuMsg->orientation, orientation);
    tf::Matrix3x3(orientation).getRPY(imuRoll, imuPitch, imuYaw);

    *rosRoll = imuRoll;
    *rosPitch = imuPitch;
    *rosYaw = imuYaw;
}

sensor_msgs::Imu imuConverter(const sensor_msgs::Imu& imu_in)
{
        sensor_msgs::Imu imu_out = imu_in;
        // rotate acceleration
        Eigen::Vector3d acc(imu_in.linear_acceleration.x, imu_in.linear_acceleration.y, imu_in.linear_acceleration.z);
        acc = extRot * acc;
        imu_out.linear_acceleration.x = acc.x();
        imu_out.linear_acceleration.y = acc.y();
        imu_out.linear_acceleration.z = acc.z();
        // rotate gyroscope
        Eigen::Vector3d gyr(imu_in.angular_velocity.x, imu_in.angular_velocity.y, imu_in.angular_velocity.z);
        gyr = extRot * gyr;
        imu_out.angular_velocity.x = gyr.x();
        imu_out.angular_velocity.y = gyr.y();
        imu_out.angular_velocity.z = gyr.z();
        // rotate roll pitch yaw
        // dunag update Converter
        double imuRollInit, imuPitchInit, imuYawInit, orign_imuYawInit;
        imuRPY2rosRPY(&imu_out, &imuRollInit, &imuPitchInit, &orign_imuYawInit);

        // std::cout << "step 0:\n" 
        //     << imuRollInit << ","
        //     << imuPitchInit << ","
        //     << imuYawInit << ","
        //     << orign_imuYawInit << " // "
        //     << imu_out.orientation.x << ","
        //     << imu_out.orientation.y << ","
        //     << imu_out.orientation.z << ","
        //     << imu_out.orientation.w << std::endl;

        Eigen::Quaterniond q_from(imu_in.orientation.w, imu_in.orientation.x, imu_in.orientation.y, imu_in.orientation.z);
        Eigen::Quaterniond q_final = q_from * extQRPY;
        imu_out.orientation.x = q_final.x();
        imu_out.orientation.y = q_final.y();
        imu_out.orientation.z = q_final.z();
        imu_out.orientation.w = q_final.w();
        // std::cout << "data is: " << sqrt(q_final.x()*q_final.x() + q_final.y()*q_final.y() + q_final.z()*q_final.z() + q_final.w()*q_final.w()) <<std::endl;
        if (sqrt(q_final.x()*q_final.x() + q_final.y()*q_final.y() + q_final.z()*q_final.z() + q_final.w()*q_final.w()) < 0.1)
        {
            ROS_ERROR("Invalid quaternion, please use a 9-axis IMU!");
            ros::shutdown();
        }

        // std::cout << "step 1:\n" 
        //     << imuRollInit << ","
        //     << imuPitchInit << ","
        //     << imuYawInit << ","
        //     << orign_imuYawInit << " // "
        //     << imu_out.orientation.x << ","
        //     << imu_out.orientation.y << ","
        //     << imu_out.orientation.z << ","
        //     << imu_out.orientation.w << std::endl;

        // std::cout << "step 2:\n"
        //           << imuRollInit << ","
        //           << imuPitchInit << ","
        //           << imuYawInit << ","
        //           << orign_imuYawInit << " // " 
        //           << imu_out.orientation.x << ","
        //           << imu_out.orientation.y << ","
        //           << imu_out.orientation.z << ","
        //           << imu_out.orientation.w << std::endl;

        return imu_out;
    }
};



float pointDistance(PointType p)
{
    return sqrt(p.x*p.x + p.y*p.y + p.z*p.z);
}


float pointDistance(PointType p1, PointType p2)
{
    return sqrt((p1.x-p2.x)*(p1.x-p2.x) + (p1.y-p2.y)*(p1.y-p2.y) + (p1.z-p2.z)*(p1.z-p2.z));
}

bool directoryExists(const std::string& path)
{
    struct stat info;
    return (stat(path.c_str(), &info)==0 && S_ISDIR(info.st_mode));
}

bool createDirectory(const std::string& path)
{
    int result = mkdir(path.c_str(), 0777);
    return (result == 0);
}
#endif
