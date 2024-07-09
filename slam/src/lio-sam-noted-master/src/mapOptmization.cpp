#include "utility.h"
#include "icp.h"
#include <iostream>
#include <fstream>
#include "lio_sam/cloud_info.h"
#include "lio_sam/save_map.h"

#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/filters/crop_box.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/dataset.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Marginals.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>

#include <gtsam/nonlinear/ISAM2.h>
#include "geo_utils.h"

#include <sys/resource.h>

#define GirdLength 500
#define PI 3.1415926535897932

using namespace gtsam;

using symbol_shorthand::B; // Bias
using symbol_shorthand::G; // GPS pose
using symbol_shorthand::V; // Vel
using symbol_shorthand::X; // Pose3 

/*
 * A point cloud type that has 6D pose info ([x,y,z,roll,pitch,yaw] intensity is time stamp)
 */
struct PointXYZIRPYT
{
    PCL_ADD_POINT4D
    PCL_ADD_INTENSITY; // preferred way of adding a XYZ+padding
    float roll;
    float pitch;
    float yaw;
    double time;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW // make sure our new allocators are aligned
} EIGEN_ALIGN16;                    // enforce SSE padding for correct memory alignment

POINT_CLOUD_REGISTER_POINT_STRUCT(PointXYZIRPYT,
                                  (float, x, x)(float, y, y)(float, z, z)(std::uint8_t, intensity, intensity)(float, roll, roll)(float, pitch, pitch)(float, yaw, yaw)(double, time, time))

typedef PointXYZIRPYT PointTypePose;

class mapOptimization : public ParamServer
{

public:
    // gtsam
    NonlinearFactorGraph gtSAMgraph;

    Values initialEstimate;
    Values optimizedEstimate;
    ISAM2 *isam;
    Values isamCurrentEstimate;
    Eigen::MatrixXd poseCovariance;

    ros::Publisher pubLaserCloudSurround;
    ros::Publisher pubLaserOdometryGlobal;
    ros::Publisher pubLaserOdometryIncremental;
    ros::Publisher pubKeyPoses;
    ros::Publisher pubPath;
    ros::Publisher fusedPathPub;

    ros::Publisher pubHistoryKeyFrames;
    ros::Publisher pubIcpKeyFrames;
    ros::Publisher pubRecentKeyFrames;
    ros::Publisher pubRecentKeyFrame;
    ros::Publisher pubCloudRegisteredRaw;
    ros::Publisher pubLoopConstraintEdge;

    ros::Publisher pubMOIsFinished; // IO control

    ros::Subscriber subCloud;
    ros::Subscriber subGPS;
    ros::Subscriber subLoop;
    ros::Subscriber subImu;

    ros::Subscriber subSaveMapACT;

    ros::ServiceServer srvSaveMap;

    lio_sam::cloud_info cloudInfo;

    vector<pcl::PointCloud<PointType>::Ptr> cornerCloudKeyFrames;
    vector<pcl::PointCloud<PointType>::Ptr> surfCloudKeyFrames;
    map<int, pcl::PointCloud<PointType>::Ptr> cornerCloudKeyFramesMap;
    map<int, pcl::PointCloud<PointType>::Ptr> surfCloudKeyFramesMap;
    std::mutex corner_mtx;
    std::mutex surf_mtx;

    pcl::PointCloud<PointType>::Ptr cloudKeyPoses3D;
    pcl::PointCloud<PointTypePose>::Ptr cloudKeyPoses6D;
    pcl::PointCloud<PointType>::Ptr copy_cloudKeyPoses3D;
    pcl::PointCloud<PointTypePose>::Ptr copy_cloudKeyPoses6D;

    pcl::PointCloud<PointType>::Ptr laserCloudCornerLast;   // corner feature set from odoOptimization
    pcl::PointCloud<PointType>::Ptr laserCloudSurfLast;     // surf feature set from odoOptimization
    pcl::PointCloud<PointType>::Ptr laserCloudCornerLastDS; // downsampled corner featuer set from odoOptimization
    pcl::PointCloud<PointType>::Ptr laserCloudSurfLastDS;   // downsampled surf featuer set from odoOptimization

    pcl::PointCloud<PointType>::Ptr laserCloudOri;
    pcl::PointCloud<PointType>::Ptr coeffSel;

    std::vector<PointType> laserCloudOriCornerVec; // corner point holder for parallel computation
    std::vector<PointType> coeffSelCornerVec;
    std::vector<bool> laserCloudOriCornerFlag;
    std::vector<PointType> laserCloudOriSurfVec; // surf point holder for parallel computation
    std::vector<PointType> coeffSelSurfVec;
    std::vector<bool> laserCloudOriSurfFlag;

    map<int, pair<pcl::PointCloud<PointType>, pcl::PointCloud<PointType>>> laserCloudMapContainer;
    pcl::PointCloud<PointType>::Ptr laserCloudCornerFromMap;
    pcl::PointCloud<PointType>::Ptr laserCloudSurfFromMap;
    pcl::PointCloud<PointType>::Ptr laserCloudCornerFromMapDS;
    pcl::PointCloud<PointType>::Ptr laserCloudSurfFromMapDS;

    pcl::KdTreeFLANN<PointType>::Ptr kdtreeCornerFromMap;
    pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurfFromMap;

    pcl::KdTreeFLANN<PointType>::Ptr kdtreeSurroundingKeyPoses;
    pcl::KdTreeFLANN<PointType>::Ptr kdtreeHistoryKeyPoses;

    pcl::VoxelGrid<PointType> downSizeFilterCorner;
    pcl::VoxelGrid<PointType> downSizeFilterSurf;
    pcl::VoxelGrid<PointType> downSizeFilterICP;
    pcl::VoxelGrid<PointType> downSizeFilterSurroundingKeyPoses; // for surrounding key poses of scan-to-map optimization

    ros::Time timeLaserInfoStamp;
    double timeLaserInfoCur;

    float transformTobeMapped[6];
    float recordInitBeforeLMO[6];

    std::mutex mtx;
    std::mutex mtxLoopInfo;

    bool isDegenerate = false;
    bool firstDegenerate = false;
    bool firstLoopClosure = false;
    int firstDegenerateKeyFrameID = -1;
    cv::Mat matP;

    int laserCloudCornerFromMapDSNum = 0;
    int laserCloudSurfFromMapDSNum = 0;
    int laserCloudCornerLastDSNum = 0;
    int laserCloudSurfLastDSNum = 0;

    int keyFramesNum = 0;
    int totalFramesNum = 0;

    bool aLoopIsClosed = false;
    map<int, int> loopIndexContainer;
    vector<pair<int, int>> loopIndexQueue;
    vector<gtsam::Pose3> loopPoseQueue;
    vector<gtsam::noiseModel::Diagonal::shared_ptr> loopNoiseQueue;
    deque<std_msgs::Float64MultiArray> loopInfoVec;

    nav_msgs::Path globalPath;
    nav_msgs::Path rospath;

    Eigen::Affine3f transPointAssociateToMap;
    Eigen::Affine3f incrementalOdometryAffineFront;
    Eigen::Affine3f incrementalOdometryAffineBack;

    Eigen::Vector3d InitPoseXYZ;
    Eigen::Vector3d GpsTransValues;

    bool writeGPSPose6DJsonInit = false;
    bool mapSaved = false;

    std::ofstream gps_pose;
    std::ofstream out;

    mapOptimization()
    {
        ISAM2Params parameters;
        parameters.relinearizeThreshold = 0.1;
        parameters.relinearizeSkip = 1;
        isam = new ISAM2(parameters);

        pubKeyPoses = nh.advertise<sensor_msgs::PointCloud2>("lio_sam/mapping/trajectory", 1);
        pubLaserCloudSurround = nh.advertise<sensor_msgs::PointCloud2>("lio_sam/mapping/map_global", 1);
        pubLaserOdometryGlobal = nh.advertise<nav_msgs::Odometry>("lio_sam/mapping/odometry", 1);
        pubLaserOdometryIncremental = nh.advertise<nav_msgs::Odometry>("lio_sam/mapping/odometry_incremental", 1);
        pubPath = nh.advertise<nav_msgs::Path>("lio_sam/mapping/path", 1);
        fusedPathPub = nh.advertise<nav_msgs::Path>("/fused_gps_path", 100);

        pubMOIsFinished = nh.advertise<std_msgs::Bool>("lio_sam/mapping/MOIsFinished", 1);

        subCloud = nh.subscribe<lio_sam::cloud_info>("lio_sam/feature/cloud_info", 1, &mapOptimization::laserCloudInfoHandler, this, ros::TransportHints().tcpNoDelay());
        subGPS = nh.subscribe<sensor_msgs::NavSatFix>(gpsTopic + "_read", 2000, &mapOptimization::gpsHandler, this, ros::TransportHints().tcpNoDelay());
        subLoop = nh.subscribe<std_msgs::Float64MultiArray>("lio_loop/loop_closure_detection", 1, &mapOptimization::loopInfoHandler, this, ros::TransportHints().tcpNoDelay());
        subSaveMapACT = nh.subscribe<std_msgs::Bool>("lio_sam/save_map_request_from_read", 1, &mapOptimization::saveMapServiceACT, this, ros::TransportHints().tcpNoDelay());

        srvSaveMap = nh.advertiseService("lio_sam/save_map", &mapOptimization::saveMapService, this);

        pubHistoryKeyFrames = nh.advertise<sensor_msgs::PointCloud2>("lio_sam/mapping/icp_loop_closure_history_cloud", 1);
        pubIcpKeyFrames = nh.advertise<sensor_msgs::PointCloud2>("lio_sam/mapping/icp_loop_closure_corrected_cloud", 1);
        pubLoopConstraintEdge = nh.advertise<visualization_msgs::MarkerArray>("/lio_sam/mapping/loop_closure_constraints", 1);

        pubRecentKeyFrames = nh.advertise<sensor_msgs::PointCloud2>("lio_sam/mapping/map_local", 1);
        pubRecentKeyFrame = nh.advertise<sensor_msgs::PointCloud2>("lio_sam/mapping/cloud_registered", 1);
        pubCloudRegisteredRaw = nh.advertise<sensor_msgs::PointCloud2>("lio_sam/mapping/cloud_registered_raw", 1);

        downSizeFilterCorner.setLeafSize(mappingCornerLeafSize, mappingCornerLeafSize, mappingCornerLeafSize);
        downSizeFilterSurf.setLeafSize(mappingSurfLeafSize, mappingSurfLeafSize, mappingSurfLeafSize);
        downSizeFilterICP.setLeafSize(mappingSurfLeafSize, mappingSurfLeafSize, mappingSurfLeafSize);
        downSizeFilterSurroundingKeyPoses.setLeafSize(surroundingKeyframeDensity, surroundingKeyframeDensity, surroundingKeyframeDensity); // for surrounding key poses of scan-to-map optimization
        gps_pose.open(savePCDDirectory + "first_pose.json", std::ios::trunc | std::ios::out);
        allocateMemory();
    }
    ~mapOptimization(){

    }

    // 预先分配内存
    void allocateMemory()
    {
        cloudKeyPoses3D.reset(new pcl::PointCloud<PointType>());
        cloudKeyPoses6D.reset(new pcl::PointCloud<PointTypePose>());
        copy_cloudKeyPoses3D.reset(new pcl::PointCloud<PointType>());
        copy_cloudKeyPoses6D.reset(new pcl::PointCloud<PointTypePose>());

        kdtreeSurroundingKeyPoses.reset(new pcl::KdTreeFLANN<PointType>());
        kdtreeHistoryKeyPoses.reset(new pcl::KdTreeFLANN<PointType>());

        laserCloudCornerLast.reset(new pcl::PointCloud<PointType>());   // corner feature set from odoOptimization
        laserCloudSurfLast.reset(new pcl::PointCloud<PointType>());     // surf feature set from odoOptimization
        laserCloudCornerLastDS.reset(new pcl::PointCloud<PointType>()); // downsampled corner featuer set from odoOptimization
        laserCloudSurfLastDS.reset(new pcl::PointCloud<PointType>());   // downsampled surf featuer set from odoOptimization

        laserCloudOri.reset(new pcl::PointCloud<PointType>());
        coeffSel.reset(new pcl::PointCloud<PointType>());

        laserCloudOriCornerVec.resize(N_SCAN * Horizon_SCAN);
        coeffSelCornerVec.resize(N_SCAN * Horizon_SCAN);
        laserCloudOriCornerFlag.resize(N_SCAN * Horizon_SCAN);
        laserCloudOriSurfVec.resize(N_SCAN * Horizon_SCAN);
        coeffSelSurfVec.resize(N_SCAN * Horizon_SCAN);
        laserCloudOriSurfFlag.resize(N_SCAN * Horizon_SCAN);

        std::fill(laserCloudOriCornerFlag.begin(), laserCloudOriCornerFlag.end(), false);
        std::fill(laserCloudOriSurfFlag.begin(), laserCloudOriSurfFlag.end(), false);

        laserCloudCornerFromMap.reset(new pcl::PointCloud<PointType>());
        laserCloudSurfFromMap.reset(new pcl::PointCloud<PointType>());
        laserCloudCornerFromMapDS.reset(new pcl::PointCloud<PointType>());
        laserCloudSurfFromMapDS.reset(new pcl::PointCloud<PointType>());

        kdtreeCornerFromMap.reset(new pcl::KdTreeFLANN<PointType>());
        kdtreeSurfFromMap.reset(new pcl::KdTreeFLANN<PointType>());

        for (int i = 0; i < 6; ++i)
        {
            transformTobeMapped[i] = 0;
            recordInitBeforeLMO[i] = 0;
        }

        matP = cv::Mat(6, 6, CV_32F, cv::Scalar::all(0));
    }

    void laserCloudInfoHandler(const lio_sam::cloud_infoConstPtr &msgIn)
    {
        totalFramesNum++;

        std_msgs::Bool isFinished;

        timeLaserInfoStamp = msgIn->header.stamp;
        timeLaserInfoCur = msgIn->header.stamp.toSec();

        cloudInfo = *msgIn;
        pcl::fromROSMsg(msgIn->cloud_corner, *laserCloudCornerLast);
        pcl::fromROSMsg(msgIn->cloud_surface, *laserCloudSurfLast);
        // std::cout << "input laserCloudCornerLast size is : " << laserCloudCornerLast->size() << std::endl;
        // std::cout << "input laserCloudSurfLast size is : " << laserCloudSurfLast->size() << std::endl;
        std::lock_guard<std::mutex> lock(mtx);

        static double timeLastProcessing = -1;
        if (timeLaserInfoCur - timeLastProcessing >= mappingProcessInterval)
        {
            timeLastProcessing = timeLaserInfoCur;

            updateInitialGuess();

            extractSurroundingKeyFrames();

            downsampleCurrentScan();

            scan2MapOptimization();

            saveKeyFramesAndFactor();

            correctPoses();

            publishOdometry();

            publishFrames();
        }

        isFinished.data = true;
        pubMOIsFinished.publish(isFinished);
    }

    Eigen::Vector3d Quaterniond2Euler(const double x, const double y, const double z, const double w)
    {
        Eigen::Quaterniond q;
        q.x() = x;
        q.y() = y;
        q.z() = z;
        q.w() = w;
        Eigen::Vector3d euler = q.toRotationMatrix().eulerAngles(2, 1, 0);
        cout << "Quaterniond2Euler result is:" << endl;
        cout << "x = " << euler[2] / PI * 180 << endl;
        cout << "y = " << euler[1] / PI * 180 << endl;
        cout << "z = " << euler[0] / PI * 180 << endl;
        return euler;
    }

    void gpsHandler(const sensor_msgs::NavSatFix::ConstPtr &gpsMsg)
    {
        Eigen::Vector3d LLA(gpsMsg->latitude, gpsMsg->longitude, gpsMsg->altitude);
        if (!GpsInitialFlag)
        {
            InitPoseXYZ[0] = gpsMsg->latitude;
            InitPoseXYZ[1] = gpsMsg->longitude;
            InitPoseXYZ[2] = gpsMsg->altitude;
            GpsInitialFlag = true;
        }
        Eigen::Vector3d ENU_NOW;
        ImuGpsLocalization::ConvertLLAToENU(InitPoseXYZ, LLA, &ENU_NOW);
        if (!GpsInitialFlag)
            return;
        GpsTransValues =  ENU_NOW;

        rospath.header.frame_id = odometryFrame;
        rospath.header.stamp = gpsMsg->header.stamp;
        geometry_msgs::PoseStamped pose;
        pose.header = rospath.header;
        pose.pose.position.x = GpsTransValues(0);
        pose.pose.position.y = GpsTransValues(1);
        pose.pose.position.z = GpsTransValues(2);
        pose.pose.orientation.x = 0;
        pose.pose.orientation.y = 0;
        pose.pose.orientation.z = 0;
        pose.pose.orientation.w = 1;
        rospath.poses.push_back(pose);
        fusedPathPub.publish(rospath);

        // writeGPSPose3D(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z, gpsMsg->header.stamp.toSec());
        writeGPSPose6D(gpsMsg);
    }

    void pointAssociateToMap(PointType const *const pi, PointType *const po)
    {
        po->x = transPointAssociateToMap(0, 0) * pi->x + transPointAssociateToMap(0, 1) * pi->y + transPointAssociateToMap(0, 2) * pi->z + transPointAssociateToMap(0, 3);
        po->y = transPointAssociateToMap(1, 0) * pi->x + transPointAssociateToMap(1, 1) * pi->y + transPointAssociateToMap(1, 2) * pi->z + transPointAssociateToMap(1, 3);
        po->z = transPointAssociateToMap(2, 0) * pi->x + transPointAssociateToMap(2, 1) * pi->y + transPointAssociateToMap(2, 2) * pi->z + transPointAssociateToMap(2, 3);
        po->intensity = pi->intensity;
    }

    pcl::PointCloud<PointType>::Ptr transformPointCloud(pcl::PointCloud<PointType>::Ptr cloudIn, PointTypePose *transformIn)
    {
        pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());

        int cloudSize = cloudIn->size();
        cloudOut->resize(cloudSize);

        Eigen::Affine3f transCur = pcl::getTransformation(transformIn->x, transformIn->y, transformIn->z, transformIn->roll, transformIn->pitch, transformIn->yaw);

#pragma omp parallel for num_threads(numberOfCores)
        for (int i = 0; i < cloudSize; ++i)
        {
            const auto &pointFrom = cloudIn->points[i];
            cloudOut->points[i].x = transCur(0, 0) * pointFrom.x + transCur(0, 1) * pointFrom.y + transCur(0, 2) * pointFrom.z + transCur(0, 3);
            cloudOut->points[i].y = transCur(1, 0) * pointFrom.x + transCur(1, 1) * pointFrom.y + transCur(1, 2) * pointFrom.z + transCur(1, 3);
            cloudOut->points[i].z = transCur(2, 0) * pointFrom.x + transCur(2, 1) * pointFrom.y + transCur(2, 2) * pointFrom.z + transCur(2, 3);
            cloudOut->points[i].intensity = pointFrom.intensity;
        }
        return cloudOut;
    }

    gtsam::Pose3 pclPointTogtsamPose3(PointTypePose thisPoint)
    {
        return gtsam::Pose3(gtsam::Rot3::RzRyRx(double(thisPoint.roll), double(thisPoint.pitch), double(thisPoint.yaw)),
                            gtsam::Point3(double(thisPoint.x), double(thisPoint.y), double(thisPoint.z)));
    }
    gtsam::Pose3 trans2gtsamPose(float transformIn[])
    {
        return gtsam::Pose3(gtsam::Rot3::RzRyRx(transformIn[0], transformIn[1], transformIn[2]),
                            gtsam::Point3(transformIn[3], transformIn[4], transformIn[5]));
    }

    Eigen::Affine3f pclPointToAffine3f(PointTypePose thisPoint)
    {
        return pcl::getTransformation(thisPoint.x, thisPoint.y, thisPoint.z, thisPoint.roll, thisPoint.pitch, thisPoint.yaw);
    }

    Eigen::Affine3f trans2Affine3f(float transformIn[])
    {
        return pcl::getTransformation(transformIn[3], transformIn[4], transformIn[5], transformIn[0], transformIn[1], transformIn[2]);
    }

    PointTypePose trans2PointTypePose(float transformIn[])
    {
        PointTypePose thisPose6D;
        thisPose6D.x = transformIn[3];
        thisPose6D.y = transformIn[4];
        thisPose6D.z = transformIn[5];
        thisPose6D.roll = transformIn[0];
        thisPose6D.pitch = transformIn[1];
        thisPose6D.yaw = transformIn[2];
        return thisPose6D;
    }
    
    bool saveMapServiceACT(lio_sam::save_mapRequest &req, lio_sam::save_mapResponse &res)
    {
        if (mapSaved)
            return true;
        else
        {
            const std_msgs::Bool::ConstPtr msg;
            saveMapServiceACT(msg);
            return true;
        }
    }
    void saveMapServiceACT(const std_msgs::Bool::ConstPtr &msgIn)
    {
        string saveMapDirectory = savePCDDirectory;

        cout << "****************************************************" << endl;
        cout << "Saving map to pcd files ..." << endl;

        writeGlobalPosesJson();
        writeGPSPose6D();

        // pcl::io::savePCDFileBinary(saveMapDirectory + "/trajectory.pcd", *cloudKeyPoses3D);
        pcl::io::savePCDFileBinary(saveMapDirectory + "/transformations.pcd", *cloudKeyPoses6D);

        extractAndSaveCornerAndSurfMap();
        // extractAndSaveGlobalMap_sync_clear_keyframes_vector();

        cout << "****************************************************" << endl;
        cout << "Saving map to pcd files completed\n"
             << endl;

        cout << "save map service ends" << endl;

        mapSaved = true;

        cout << "shutdown now, Bye!" << endl;
        ros::shutdown();
    }

    void extractAndSaveCornerAndSurfMap()
    {
        pcl::PointCloud<PointType>::Ptr globalCornerCloud(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr globalSurfCloud(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr globalCloud(new pcl::PointCloud<PointType>());


        for (int i = 0; i < (int)cloudKeyPoses3D->size(); i++)
        {
            pcl::PointCloud<PointType>::Ptr tempCornerCloud(new pcl::PointCloud<PointType>());
            pcl::PointCloud<PointType>::Ptr tempSurfCloud(new pcl::PointCloud<PointType>());

            *tempCornerCloud = *transformPointCloud(getCornerKeyFrameAt(i), &cloudKeyPoses6D->points[i]);
            *tempSurfCloud = *transformPointCloud(getSurfKeyFrameAt(i), &cloudKeyPoses6D->points[i]);

            *globalCornerCloud += *tempCornerCloud;

            *globalSurfCloud += *tempSurfCloud;
        }
        *globalCloud = *globalCornerCloud +  *globalSurfCloud;

        pcl::io::savePCDFileBinary(savePCDDirectory + "/CornerMap.pcd", *globalCornerCloud);
        pcl::io::savePCDFileBinary(savePCDDirectory + "/SurfMap.pcd", *globalSurfCloud);
        pcl::io::savePCDFileBinary(savePCDDirectory + "/Global.pcd", *globalCloud);

        globalCornerCloud->points.resize(0);
        globalCornerCloud->points.shrink_to_fit();
        globalCornerCloud.reset();

        globalSurfCloud->points.resize(0);
        globalSurfCloud->points.shrink_to_fit();
        globalSurfCloud.reset();

        globalCloud->points.resize(0);
        globalCloud->points.shrink_to_fit();
        globalCloud.reset();

    }

    void extractAndSaveGlobalMap_sync_clear_keyframes_vector()
    {
        pcl::PointCloud<PointType>::Ptr globalMapCloud(new pcl::PointCloud<PointType>());

        for (int i = 0; i < (int)cloudKeyPoses3D->size(); i++)
        {
            pcl::PointCloud<PointType>::Ptr tempCornerCloud(new pcl::PointCloud<PointType>());
            pcl::PointCloud<PointType>::Ptr tempSurfCloud(new pcl::PointCloud<PointType>());

            *tempCornerCloud = *transformPointCloud(cornerCloudKeyFrames[i], &cloudKeyPoses6D->points[i]);
            *tempSurfCloud = *transformPointCloud(surfCloudKeyFrames[i], &cloudKeyPoses6D->points[i]);

            cornerCloudKeyFrames[i]->points.resize(0);
            cornerCloudKeyFrames[i]->points.shrink_to_fit();
            surfCloudKeyFrames[i]->points.resize(0);
            surfCloudKeyFrames[i]->points.shrink_to_fit();

            *globalMapCloud += *tempCornerCloud;
            *globalMapCloud += *tempSurfCloud;

            tempCornerCloud->points.resize(0);
            tempCornerCloud->points.shrink_to_fit();

            tempSurfCloud->points.resize(0);
            tempSurfCloud->points.shrink_to_fit();
        }
        
        pcl::io::savePCDFileBinary(savePCDDirectory + "/GlobalMap.pcd", *globalMapCloud);

        globalMapCloud->points.resize(0);
        globalMapCloud->points.shrink_to_fit();
    }

    bool saveMapService(lio_sam::save_mapRequest &req, lio_sam::save_mapResponse &res)
    {
        if (mapSaved)
            return true;

        writeGlobalPosesJson();
        writeGPSPose6D();
        string saveMapDirectory = savePCDDirectory;

        cout << "****************************************************" << endl;
        cout << "Saving map to pcd files ..." << endl;
        
        pcl::io::savePCDFileBinary(saveMapDirectory + "/trajectory.pcd", *cloudKeyPoses3D);
        pcl::io::savePCDFileBinary(saveMapDirectory + "/transformations.pcd", *cloudKeyPoses6D);
        // extract global point cloud map
        pcl::PointCloud<PointType>::Ptr globalCornerCloud(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr globalCornerCloudDS(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr globalSurfCloud(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr globalSurfCloudDS(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr globalMapCloud(new pcl::PointCloud<PointType>());

        pcl::PointCloud<PointType>::Ptr tempCornerCloud(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr tempSurfCloud(new pcl::PointCloud<PointType>());
        for (int i = 0; i < (int)cloudKeyPoses3D->size(); i++)
        {
            *tempCornerCloud = *transformPointCloud(cornerCloudKeyFrames[i], &cloudKeyPoses6D->points[i]);
            *tempSurfCloud = *transformPointCloud(surfCloudKeyFrames[i], &cloudKeyPoses6D->points[i]);
            *globalCornerCloud += *tempCornerCloud;
            *globalSurfCloud += *tempSurfCloud;
        }
        if (req.resolution != 0)
        {
            cout << "\n\nSave resolution: " << req.resolution << endl;

            downSizeFilterCorner.setInputCloud(globalCornerCloud);
            downSizeFilterCorner.setLeafSize(req.resolution, req.resolution, req.resolution);
            downSizeFilterCorner.filter(*globalCornerCloudDS);
            pcl::io::savePCDFileBinary(saveMapDirectory + "/CornerMap.pcd", *globalCornerCloudDS);
            // down-sample and save surf cloud
            downSizeFilterSurf.setInputCloud(globalSurfCloud);
            downSizeFilterSurf.setLeafSize(req.resolution, req.resolution, req.resolution);
            downSizeFilterSurf.filter(*globalSurfCloudDS);
            pcl::io::savePCDFileBinary(saveMapDirectory + "/SurfMap.pcd", *globalSurfCloudDS);
        }
        else
        {
            // save corner cloud
            pcl::io::savePCDFileBinary(saveMapDirectory + "/CornerMap.pcd", *globalCornerCloud);
            // save surf cloud
            pcl::io::savePCDFileBinary(saveMapDirectory + "/SurfMap.pcd", *globalSurfCloud);
        }

        // save global point cloud map
        *globalMapCloud += *globalCornerCloud;
        *globalMapCloud += *globalSurfCloud;

        if (globalMapCloud->size() > 1000000)
        {
            std::string file_path = saveMapDirectory + "/segment/";
            string cmd = "mkdir " + file_path;
            int result = system(cmd.c_str());
            if (result != 0)
            {
                cout << "failed make folder : " << file_path << endl;
                return 0;
            }
            cout << "success make segment pcds folder " << endl;

            pcl::PointXYZI min_pt, max_pt;
            pcl::getMinMax3D(*globalMapCloud, min_pt, max_pt);
            double x_length = max_pt.x - min_pt.x;
            double y_length = max_pt.y - min_pt.y;

            double grid_length = GirdLength;
            double grid_width = GirdLength;

            int cols = ceil(x_length / grid_length);
            int rows = ceil(y_length / grid_width);

            // Iterate through the grid and apply CropBox filter to each subregion
            for (int i = 0; i < rows; ++i)
            {
                for (int j = 0; j < cols; ++j)
                {
                    pcl::CropBox<PointType> crop_filter;
                    crop_filter.setInputCloud(globalMapCloud);

                    // Define the cropping box dimensions for the current subregion
                    double min_x = j * grid_width;
                    double max_x = (j + 1) * grid_width;
                    double min_y = i * grid_length;
                    double max_y = (i + 1) * grid_length;

                    crop_filter.setMin(Eigen::Vector4f(min_pt.x + min_x, min_pt.y + min_y, -std::numeric_limits<float>::max(), 1.0));
                    crop_filter.setMax(Eigen::Vector4f(min_pt.x + max_x, min_pt.y + max_y, std::numeric_limits<float>::max(), 1.0));

                    // Apply the filter to obtain the segmented point cloud
                    pcl::PointCloud<PointType>::Ptr segmented_cloud(new pcl::PointCloud<PointType>);
                    crop_filter.filter(*segmented_cloud);

                    if (segmented_cloud->size() == 0)
                    {
                        PointType zero_point;
                        zero_point.x = 0;
                        zero_point.y = 0;
                        zero_point.z = 0;
                        zero_point.intensity = 0;
                        segmented_cloud->push_back(zero_point);
                    }
                    // Save the segmented point cloud to a PCD file
                    std::string filename = file_path + "segmented_cloud_" + std::to_string(i) + "_" + std::to_string(j) + ".pcd";
                    pcl::io::savePCDFileASCII(filename, *segmented_cloud);
                }
            }
        }
        else
        {
            int ret = pcl::io::savePCDFileBinary(saveMapDirectory + "/GlobalMap.pcd", *globalMapCloud);
            res.success = ret == 0;
        }

        downSizeFilterCorner.setLeafSize(mappingCornerLeafSize, mappingCornerLeafSize, mappingCornerLeafSize);
        downSizeFilterSurf.setLeafSize(mappingSurfLeafSize, mappingSurfLeafSize, mappingSurfLeafSize);

        cout << "****************************************************" << endl;
        cout << "Saving map to pcd files completed\n"
             << endl;

        return true;
    }

    void visualizeGlobalMapThread()
    {
        ros::Rate rate(0.2);
        while (ros::ok())
        {
            rate.sleep();
            publishGlobalMap();
        }
        if (savePCD == false)
            return;

        lio_sam::save_mapRequest req;
        lio_sam::save_mapResponse res;
        if (!saveMapServiceACT(req, res))
        {
            cout << "Fail to save map" << endl;
        }
    }

    void publishGlobalMap()
    {
        if (pubLaserCloudSurround.getNumSubscribers() == 0)
            return;
        if (cloudKeyPoses3D->points.empty() == true)
            return;

        pcl::KdTreeFLANN<PointType>::Ptr kdtreeGlobalMap(new pcl::KdTreeFLANN<PointType>());
        pcl::PointCloud<PointType>::Ptr globalMapKeyPoses(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr globalMapKeyPosesDS(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr globalMapKeyFrames(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr globalMapKeyFramesDS(new pcl::PointCloud<PointType>());

        // kd-tree to find near key frames to visualize
        std::vector<int> pointSearchIndGlobalMap;
        std::vector<float> pointSearchSqDisGlobalMap;
        // search near key frames to visualize
        mtx.lock();
        kdtreeGlobalMap->setInputCloud(cloudKeyPoses3D);
        kdtreeGlobalMap->radiusSearch(cloudKeyPoses3D->back(), globalMapVisualizationSearchRadius, pointSearchIndGlobalMap, pointSearchSqDisGlobalMap, 0);
        mtx.unlock();
        for (int i = 0; i < (int)pointSearchIndGlobalMap.size(); ++i)
            globalMapKeyPoses->push_back(cloudKeyPoses3D->points[pointSearchIndGlobalMap[i]]);
        // downsample near selected key frames
        pcl::VoxelGrid<PointType> downSizeFilterGlobalMapKeyPoses;                                                                                            // for global map visualization
        downSizeFilterGlobalMapKeyPoses.setLeafSize(globalMapVisualizationPoseDensity, globalMapVisualizationPoseDensity, globalMapVisualizationPoseDensity); // for global map visualization
        downSizeFilterGlobalMapKeyPoses.setInputCloud(globalMapKeyPoses);
        downSizeFilterGlobalMapKeyPoses.filter(*globalMapKeyPosesDS);
        for (auto &pt : globalMapKeyPosesDS->points)
        {
            kdtreeGlobalMap->nearestKSearch(pt, 1, pointSearchIndGlobalMap, pointSearchSqDisGlobalMap);
            pt.intensity = cloudKeyPoses3D->points[pointSearchIndGlobalMap[0]].intensity;
        }

        // extract visualized and downsampled key frames
        for (int i = 0; i < (int)globalMapKeyPosesDS->size(); ++i)
        {
            if (pointDistance(globalMapKeyPosesDS->points[i], cloudKeyPoses3D->back()) > globalMapVisualizationSearchRadius)
                continue;
            int thisKeyInd = (int)globalMapKeyPosesDS->points[i].intensity;
            // add for memory opt
            *globalMapKeyFrames += *transformPointCloud(getCornerKeyFrameAt(thisKeyInd), &cloudKeyPoses6D->points[thisKeyInd]);
            *globalMapKeyFrames += *transformPointCloud(getSurfKeyFrameAt(thisKeyInd), &cloudKeyPoses6D->points[thisKeyInd]);

            // *globalMapKeyFrames += *transformPointCloud(cornerCloudKeyFrames[thisKeyInd], &cloudKeyPoses6D->points[thisKeyInd]);
            // *globalMapKeyFrames += *transformPointCloud(surfCloudKeyFrames[thisKeyInd], &cloudKeyPoses6D->points[thisKeyInd]);
        }
        // downsample visualized points
        pcl::VoxelGrid<PointType> downSizeFilterGlobalMapKeyFrames;                                                                                   // for global map visualization
        downSizeFilterGlobalMapKeyFrames.setLeafSize(globalMapVisualizationLeafSize, globalMapVisualizationLeafSize, globalMapVisualizationLeafSize); // for global map visualization
        downSizeFilterGlobalMapKeyFrames.setInputCloud(globalMapKeyFrames);
        downSizeFilterGlobalMapKeyFrames.filter(*globalMapKeyFramesDS);
        publishCloud(&pubLaserCloudSurround, globalMapKeyFramesDS, timeLaserInfoStamp, odometryFrame);
    }


    void loopInfoHandler(const std_msgs::Float64MultiArray::ConstPtr &loopMsg)
    {
        std::lock_guard<std::mutex> lock(mtxLoopInfo);
        if (loopMsg->data.size() != 2)
            return;
        loopInfoVec.push_back(*loopMsg);
        while (loopInfoVec.size() > 5)
            loopInfoVec.pop_front();
    }

    void performLoopClosure()
    {
        if (cloudKeyPoses3D->points.empty() == true)
            return;

        *copy_cloudKeyPoses3D = *cloudKeyPoses3D;
        *copy_cloudKeyPoses6D = *cloudKeyPoses6D;

        int loopKeyCur;
        int loopKeyPre;

        if (detectLoopClosureExternal(&loopKeyCur, &loopKeyPre) == false)
            if (detectLoopClosureDistance(&loopKeyCur, &loopKeyPre) == false)
                return;
        pcl::PointCloud<PointType>::Ptr cureKeyframeCloud(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr prevKeyframeCloud(new pcl::PointCloud<PointType>());
        {
            loopFindNearKeyframes(cureKeyframeCloud, loopKeyCur, 0);

            loopFindNearKeyframes(prevKeyframeCloud, loopKeyPre, historyKeyframeSearchNum);
            if (cureKeyframeCloud->size() < 300 || prevKeyframeCloud->size() < 1000)
            {
                return;
            }
            if (pubHistoryKeyFrames.getNumSubscribers() != 0)
                publishCloud(&pubHistoryKeyFrames, prevKeyframeCloud, timeLaserInfoStamp, odometryFrame);
        }
        Eigen::Affine3f correctionLidarFrame;
        float noiseScore = 100;
        if(datatype == DataType::CUS0 || datatype == DataType::CUS1 || datatype == DataType::CUS3 || datatype == DataType::CUS5)
        {
            static pcl::IterativeClosestPoint<PointType, PointType> icp;
            icp.setMaxCorrespondenceDistance(historyKeyframeSearchRadius * 2);
            icp.setMaximumIterations(150);
            icp.setTransformationEpsilon(1e-6);
            icp.setEuclideanFitnessEpsilon(1e-6);
            icp.setRANSACIterations(0);

            // Align clouds
            icp.setInputSource(cureKeyframeCloud);
            icp.setInputTarget(prevKeyframeCloud);
            pcl::PointCloud<PointType>::Ptr unused_result(new pcl::PointCloud<PointType>());
            icp.align(*unused_result);
            // writeLoopClosureInfo(loopKeyPre, loopKeyCur, icp.getFitnessScore());

            if (icp.hasConverged() == false || icp.getFitnessScore() > historyKeyframeFitnessScore)
            {
                if (icp.hasConverged() == false)
                {
                    // cout << "icp.hasConverged() == false" << endl;
                }
                if (icp.getFitnessScore() > historyKeyframeFitnessScore)
                {
                    // cout << "icp.getFitnessScore() > historyKeyframeFitnessScore & icpscore：" << icp.getFitnessScore() << endl;
                }
                return;
            }
            if (pubIcpKeyFrames.getNumSubscribers() != 0)
            {
                pcl::PointCloud<PointType>::Ptr closed_cloud(new pcl::PointCloud<PointType>());
                pcl::transformPointCloud(*cureKeyframeCloud, *closed_cloud, icp.getFinalTransformation());
                publishCloud(&pubIcpKeyFrames, closed_cloud, timeLaserInfoStamp, odometryFrame);
            }
            correctionLidarFrame = icp.getFinalTransformation();
            noiseScore = icp.getFitnessScore();
        }
        else if (datatype == DataType::CUS2 || datatype == DataType::CUS4)
        {
            ICP icp;
            icp.setMaxCorrespondenceDistance_reg(historyKeyframeSearchRadius * 2);
            icp.setMaxCorrespondenceDistance_score(historyKeyframeFitnessScore);
            icp.setMaximumIterations(100);
            icp.setTransformationEpsilon(1e-6);

            icp.setInputSource(cureKeyframeCloud);
            icp.setInputTarget(prevKeyframeCloud);
            pcl::PointCloud<PointType>::Ptr unused_result(new pcl::PointCloud<PointType>());
            icp.align(unused_result);
            // writeLoopClosureInfo(loopKeyPre, loopKeyCur, icp.getFitnessScore());

            if (!icp.resultCanBeUsed(historyKeyframeFitnessScore, 0.7))
            {
                return;
            }
            if (pubIcpKeyFrames.getNumSubscribers() != 0)
            {
                pcl::PointCloud<PointType>::Ptr closed_cloud(new pcl::PointCloud<PointType>());
                pcl::transformPointCloud(*cureKeyframeCloud, *closed_cloud, icp.getFinalTransformation());
                publishCloud(&pubIcpKeyFrames, closed_cloud, timeLaserInfoStamp, odometryFrame);
            }
            correctionLidarFrame = icp.getFinalTransformation();
            noiseScore = icp.getFitnessScore();
        }
        // Get pose transformation
        float x, y, z, roll, pitch, yaw;
        // transform from world origin to wrong pose
        Eigen::Affine3f tWrong = pclPointToAffine3f(copy_cloudKeyPoses6D->points[loopKeyCur]);
        // transform from world origin to corrected pose
        Eigen::Affine3f tCorrect = correctionLidarFrame * tWrong; // pre-multiplying -> successive rotation about a fixed frame
        pcl::getTranslationAndEulerAngles(tCorrect, x, y, z, roll, pitch, yaw);
        gtsam::Pose3 poseFrom = Pose3(Rot3::RzRyRx(roll, pitch, yaw), Point3(x, y, z));
        gtsam::Pose3 poseTo = pclPointTogtsamPose3(copy_cloudKeyPoses6D->points[loopKeyPre]);
        gtsam::Vector Vector6(6);
        Vector6 << noiseScore, noiseScore, noiseScore, noiseScore, noiseScore, noiseScore;
        noiseModel::Diagonal::shared_ptr constraintNoise = noiseModel::Diagonal::Variances(Vector6);

        // Add pose constraint
        loopIndexQueue.push_back(make_pair(loopKeyCur, loopKeyPre));
        loopPoseQueue.push_back(poseFrom.between(poseTo));
        loopNoiseQueue.push_back(constraintNoise);
        loopIndexContainer[loopKeyCur] = loopKeyPre;
    }
    bool detectLoopClosureDistance(int *latestID, int *closestID)
    {
        int loopKeyCur = copy_cloudKeyPoses3D->size() - 1;
        int loopKeyPre = -1;

        // find the closest history key frame
        std::vector<int> pointSearchIndLoop;
        std::vector<float> pointSearchSqDisLoop;
        kdtreeHistoryKeyPoses->setInputCloud(copy_cloudKeyPoses3D);

        kdtreeHistoryKeyPoses->radiusSearch(copy_cloudKeyPoses3D->back(), historyKeyframeSearchRadius, pointSearchIndLoop, pointSearchSqDisLoop, 0);
        for (int i = 0; i < (int)pointSearchIndLoop.size(); ++i)
        {
            int id = pointSearchIndLoop[i];
            if (abs(copy_cloudKeyPoses6D->points[id].time - timeLaserInfoCur) > historyKeyframeSearchTimeDiff)
            {
                loopKeyPre = id;
                break;
            }
        }
        if (loopKeyPre == -1 || loopKeyCur == loopKeyPre)
        {
            return false;
        }

        *latestID = loopKeyCur;
        *closestID = loopKeyPre;

        return true;
    }

    bool detectLoopClosureExternal(int *latestID, int *closestID)
    {
        int loopKeyCur = -1;
        int loopKeyPre = -1;

        std::lock_guard<std::mutex> lock(mtxLoopInfo);
        if (loopInfoVec.empty())
            return false;
        double loopTimeCur = loopInfoVec.front().data[0];
        double loopTimePre = loopInfoVec.front().data[1];
        loopInfoVec.pop_front();
        if (abs(loopTimeCur - loopTimePre) < historyKeyframeSearchTimeDiff)
            return false;
        int cloudSize = copy_cloudKeyPoses6D->size();
        if (cloudSize < 2)
            return false;

        loopKeyCur = cloudSize - 1;
        for (int i = cloudSize - 1; i >= 0; --i)
        {
            if (copy_cloudKeyPoses6D->points[i].time >= loopTimeCur)
                loopKeyCur = round(copy_cloudKeyPoses6D->points[i].intensity);
            else
                break;
        }

        // previous key
        loopKeyPre = 0;
        for (int i = 0; i < cloudSize; ++i)
        {
            if (copy_cloudKeyPoses6D->points[i].time <= loopTimePre)
                loopKeyPre = round(copy_cloudKeyPoses6D->points[i].intensity);
            else
                break;
        }
        if (loopKeyCur == loopKeyPre)
            return false;
        auto it = loopIndexContainer.find(loopKeyCur);
        if (it != loopIndexContainer.end())
            return false;
        *latestID = loopKeyCur;
        *closestID = loopKeyPre;

        return true;
    }

    void loopFindNearKeyframes(pcl::PointCloud<PointType>::Ptr &nearKeyframes, const int &key, const int &searchNum)
    {
        // extract near keyframes
        nearKeyframes->clear();
        int cloudSize = copy_cloudKeyPoses6D->size();
        for (int i = -searchNum; i <= searchNum; ++i)
        {
            int keyNear = key + i;
            if (keyNear < 0 || keyNear >= cloudSize)
                continue;
            // add for memory opt
            *nearKeyframes += *transformPointCloud(getCornerKeyFrameAt(keyNear), &copy_cloudKeyPoses6D->points[keyNear]);
            *nearKeyframes += *transformPointCloud(getSurfKeyFrameAt(keyNear), &copy_cloudKeyPoses6D->points[keyNear]);

            // *nearKeyframes += *transformPointCloud(cornerCloudKeyFrames[keyNear], &copy_cloudKeyPoses6D->points[keyNear]);
            // *nearKeyframes += *transformPointCloud(surfCloudKeyFrames[keyNear], &copy_cloudKeyPoses6D->points[keyNear]);
        }
        if (nearKeyframes->empty())
        {
            return;
        }

        // downsample near keyframes
        pcl::PointCloud<PointType>::Ptr cloud_temp(new pcl::PointCloud<PointType>());
        downSizeFilterICP.setInputCloud(nearKeyframes);
        downSizeFilterICP.filter(*cloud_temp);
        *nearKeyframes = *cloud_temp;
    }
    void visualizeLoopClosure()
    {
        if (loopIndexContainer.empty())
            return;

        visualization_msgs::MarkerArray markerArray;
        visualization_msgs::Marker markerNode;
        markerNode.header.frame_id = odometryFrame;
        markerNode.header.stamp = timeLaserInfoStamp;
        markerNode.action = visualization_msgs::Marker::ADD;
        markerNode.type = visualization_msgs::Marker::SPHERE_LIST;
        markerNode.ns = "loop_nodes";
        markerNode.id = 0;
        markerNode.pose.orientation.w = 1;
        markerNode.scale.x = 0.3;
        markerNode.scale.y = 0.3;
        markerNode.scale.z = 0.3;
        markerNode.color.r = 0;
        markerNode.color.g = 0.8;
        markerNode.color.b = 1;
        markerNode.color.a = 1;

        visualization_msgs::Marker markerEdge;
        markerEdge.header.frame_id = odometryFrame;
        markerEdge.header.stamp = timeLaserInfoStamp;
        markerEdge.action = visualization_msgs::Marker::ADD;
        markerEdge.type = visualization_msgs::Marker::LINE_LIST;
        markerEdge.ns = "loop_edges";
        markerEdge.id = 1;
        markerEdge.pose.orientation.w = 1;
        markerEdge.scale.x = 0.1;
        markerEdge.color.r = 0.9;
        markerEdge.color.g = 0.9;
        markerEdge.color.b = 0;
        markerEdge.color.a = 1;
        for (auto it = loopIndexContainer.begin(); it != loopIndexContainer.end(); ++it)
        {
            int key_cur = it->first;
            int key_pre = it->second;
            geometry_msgs::Point p;
            p.x = copy_cloudKeyPoses6D->points[key_cur].x;
            p.y = copy_cloudKeyPoses6D->points[key_cur].y;
            p.z = copy_cloudKeyPoses6D->points[key_cur].z;

            markerNode.points.push_back(p);
            markerEdge.points.push_back(p);
            p.x = copy_cloudKeyPoses6D->points[key_pre].x;
            p.y = copy_cloudKeyPoses6D->points[key_pre].y;
            p.z = copy_cloudKeyPoses6D->points[key_pre].z;
            markerNode.points.push_back(p);
            markerEdge.points.push_back(p);
        }

        markerArray.markers.push_back(markerNode);
        markerArray.markers.push_back(markerEdge);

        pubLoopConstraintEdge.publish(markerArray);
    }

    void updateInitialGuess()
    {
        // save current transformation before any processing
        incrementalOdometryAffineFront = trans2Affine3f(transformTobeMapped);

        static Eigen::Affine3f lastImuTransformation;
        // initialization
        if (cloudKeyPoses3D->points.empty())
        {
            transformTobeMapped[0] = cloudInfo.imuRollInit;
            transformTobeMapped[1] = cloudInfo.imuPitchInit;  
            transformTobeMapped[2] = cloudInfo.imuYawInit;          
            if (!useImuHeadingInitialization)
                transformTobeMapped[2] = 0;
            lastImuTransformation = pcl::getTransformation(0, 0, 0, cloudInfo.imuRollInit, cloudInfo.imuPitchInit, cloudInfo.imuYawInit);
            return;
        }

        // use imu pre-integration estimation for pose guess
        static bool lastImuPreTransAvailable = false;
        static Eigen::Affine3f lastImuPreTransformation;
        if (cloudInfo.odomAvailable == true)
        {
            Eigen::Affine3f transBack = pcl::getTransformation(cloudInfo.initialGuessX, cloudInfo.initialGuessY, cloudInfo.initialGuessZ,
                                                               cloudInfo.initialGuessRoll, cloudInfo.initialGuessPitch, cloudInfo.initialGuessYaw);
            if (lastImuPreTransAvailable == false)
            {
                lastImuPreTransformation = transBack;
                lastImuPreTransAvailable = true;
            }
            else
            {
                Eigen::Affine3f transIncre = lastImuPreTransformation.inverse() * transBack;
                Eigen::Affine3f transTobe = trans2Affine3f(transformTobeMapped);
                Eigen::Affine3f transFinal = transTobe * transIncre;
                pcl::getTranslationAndEulerAngles(transFinal, transformTobeMapped[3], transformTobeMapped[4], transformTobeMapped[5],
                                                  transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]);

                lastImuPreTransformation = transBack;
                lastImuTransformation = pcl::getTransformation(0, 0, 0, cloudInfo.imuRollInit, cloudInfo.imuPitchInit, cloudInfo.imuYawInit);
            
                return;
            }
        }

        // use imu incremental estimation for pose guess (only rotation)
        if (cloudInfo.imuAvailable == true)
        {
            Eigen::Affine3f transBack;
            transBack = pcl::getTransformation(0, 0, 0, cloudInfo.imuRollInit, cloudInfo.imuPitchInit, cloudInfo.imuYawInit);
            Eigen::Affine3f transIncre = lastImuTransformation.inverse() * transBack;

            Eigen::Affine3f transTobe = trans2Affine3f(transformTobeMapped);
            Eigen::Affine3f transFinal = transTobe * transIncre;
            pcl::getTranslationAndEulerAngles(transFinal, transformTobeMapped[3], transformTobeMapped[4], transformTobeMapped[5],
                                              transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]);
            lastImuTransformation = pcl::getTransformation(0, 0, 0, cloudInfo.imuRollInit, cloudInfo.imuPitchInit, cloudInfo.imuYawInit);
            return;
        }
    }

    void extractForLoopClosure()
    {
        pcl::PointCloud<PointType>::Ptr cloudToExtract(new pcl::PointCloud<PointType>());
        int numPoses = cloudKeyPoses3D->size();
        for (int i = numPoses - 1; i >= 0; --i)
        {
            if ((int)cloudToExtract->size() <= surroundingKeyframeSize)
                cloudToExtract->push_back(cloudKeyPoses3D->points[i]);
            else
                break;
        }

        extractCloud(cloudToExtract);
    }

    void extractNearby()
    {
        pcl::PointCloud<PointType>::Ptr surroundingKeyPoses(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr surroundingKeyPosesDS(new pcl::PointCloud<PointType>());
        std::vector<int> pointSearchInd;     
        std::vector<float> pointSearchSqDis; 

        // extract all the nearby key poses and downsample them
        kdtreeSurroundingKeyPoses->setInputCloud(cloudKeyPoses3D); // create kd-tree
        kdtreeSurroundingKeyPoses->radiusSearch(cloudKeyPoses3D->back(), (double)surroundingKeyframeSearchRadius, pointSearchInd, pointSearchSqDis);
        for (int i = 0; i < (int)pointSearchInd.size(); ++i)
        {
            int id = pointSearchInd[i];
            surroundingKeyPoses->push_back(cloudKeyPoses3D->points[id]);
        }

        downSizeFilterSurroundingKeyPoses.setInputCloud(surroundingKeyPoses);
        downSizeFilterSurroundingKeyPoses.filter(*surroundingKeyPosesDS);
        for (auto &pt : surroundingKeyPosesDS->points)
        {
            kdtreeSurroundingKeyPoses->nearestKSearch(pt, 1, pointSearchInd, pointSearchSqDis);
            pt.intensity = cloudKeyPoses3D->points[pointSearchInd[0]].intensity;
        }

        // also extract some latest key frames in case the robot rotates in one position
        int numPoses = cloudKeyPoses3D->size();
        for (int i = numPoses - 1; i >= 0; --i)
        {
            if (timeLaserInfoCur - cloudKeyPoses6D->points[i].time < 10.0)
                surroundingKeyPosesDS->push_back(cloudKeyPoses3D->points[i]);
            else
                break;
        }
        extractCloud(surroundingKeyPosesDS);
    }

    void extractCloud(pcl::PointCloud<PointType>::Ptr cloudToExtract)
    {
        laserCloudCornerFromMap->clear();
        laserCloudSurfFromMap->clear();
        for (int i = 0; i < (int)cloudToExtract->size(); ++i)
        {
            if (pointDistance(cloudToExtract->points[i], cloudKeyPoses3D->back()) > surroundingKeyframeSearchRadius)
                continue;
            int thisKeyInd = (int)cloudToExtract->points[i].intensity;
            if (laserCloudMapContainer.find(thisKeyInd) != laserCloudMapContainer.end())
            {
                *laserCloudCornerFromMap += laserCloudMapContainer[thisKeyInd].first;
                *laserCloudSurfFromMap += laserCloudMapContainer[thisKeyInd].second;
            }
            else
            {
                pcl::PointCloud<PointType> laserCloudCornerTemp = *transformPointCloud(getCornerKeyFrameAt(thisKeyInd), &cloudKeyPoses6D->points[thisKeyInd]);
                pcl::PointCloud<PointType> laserCloudSurfTemp = *transformPointCloud(getSurfKeyFrameAt(thisKeyInd), &cloudKeyPoses6D->points[thisKeyInd]);

                *laserCloudCornerFromMap += laserCloudCornerTemp;
                *laserCloudSurfFromMap += laserCloudSurfTemp;
                laserCloudMapContainer[thisKeyInd] = make_pair(laserCloudCornerTemp, laserCloudSurfTemp);
            }
        }

        // Downsample the surrounding corner key frames (or map)
        downSizeFilterCorner.setInputCloud(laserCloudCornerFromMap);
        downSizeFilterCorner.filter(*laserCloudCornerFromMapDS);
        laserCloudCornerFromMapDSNum = laserCloudCornerFromMapDS->size();
        // Downsample the surrounding surf key frames (or map)
        downSizeFilterSurf.setInputCloud(laserCloudSurfFromMap);
        downSizeFilterSurf.filter(*laserCloudSurfFromMapDS);
        laserCloudSurfFromMapDSNum = laserCloudSurfFromMapDS->size();

        // clear map cache if too large
        if (laserCloudMapContainer.size() > 1000)
            laserCloudMapContainer.clear();
    }

    pcl::PointCloud<PointType>::Ptr getCornerKeyFrameAt(int key_index)
    {
        std::lock_guard<std::mutex> lock(corner_mtx);

        pcl::PointCloud<PointType>::Ptr tempCloud(new pcl::PointCloud<PointType>);

        if (cornerCloudKeyFramesMap.find(key_index) != cornerCloudKeyFramesMap.end())
        {
            pcl::copyPointCloud(*cornerCloudKeyFramesMap[key_index], *tempCloud);
        }
        else
        {
            string file_path = savePCDDirectory + "key_frames/corner/";

            if (!fs::exists(file_path))
            {
                if (!fs::create_directories(file_path))
                {
                    ROS_WARN("key frame path doesn't exist, and fails to exist!");
                }
            }

            file_path = file_path + to_string(key_index) + ".pcd";

            if (pcl::io::loadPCDFile<PointType>(file_path, *tempCloud) == -1)
            {
                PCL_ERROR("Couldn't read file your_point_cloud_file.pcd \n");
                return NULL;
            }
        }

        return tempCloud;
    }
    pcl::PointCloud<PointType>::Ptr getSurfKeyFrameAt(int key_index)
    {
        std::lock_guard<std::mutex> lock(surf_mtx);

        pcl::PointCloud<PointType>::Ptr tempCloud(new pcl::PointCloud<PointType>);

        if (surfCloudKeyFramesMap.find(key_index) != surfCloudKeyFramesMap.end())
        {
            pcl::copyPointCloud(*surfCloudKeyFramesMap[key_index], *tempCloud);
        }
        else
        {
            string file_path = savePCDDirectory + "key_frames/surf/";

            if (!fs::exists(file_path))
            {
                if (!fs::create_directories(file_path))
                {
                    ROS_WARN("key frame path doesn't exist, and fails to exist!");
                }
            }

            file_path = file_path + to_string(key_index) + ".pcd";

            if (pcl::io::loadPCDFile<PointType>(file_path, *tempCloud) == -1)
            {
                PCL_ERROR("Couldn't read file your_point_cloud_file.pcd \n");
            }
        }

        return tempCloud;
    }
    bool setCornerKeyFrame(pcl::PointCloud<PointType>::Ptr thisCornerKeyFrame, int key_index)
    {
        string file_path = savePCDDirectory + "key_frames/corner/";

        if (!fs::exists(file_path))
        {
            if (!fs::create_directories(file_path))
            {
                ROS_WARN("key frame path doesn't exist, and fails to exist!");
            }
        }

        file_path = file_path + to_string(key_index) + ".pcd";

        pcl::io::savePCDFileBinary(file_path, *thisCornerKeyFrame);

        if (cornerCloudKeyFramesMap.size() <= 500)
        {
            cornerCloudKeyFramesMap[key_index] = thisCornerKeyFrame;
        }
        else
        {
            auto it = cornerCloudKeyFramesMap.begin();
            for (int i = 0; i < 100; ++i)
            {
                it = cornerCloudKeyFramesMap.erase(it);
            }
            cornerCloudKeyFramesMap[key_index] = thisCornerKeyFrame;
        }
        return true;
    }
    bool setSurfKeyFrame(pcl::PointCloud<PointType>::Ptr thisSurfKeyFrame, int key_index)
    {
        string file_path = savePCDDirectory + "key_frames/surf/";

        if (!fs::exists(file_path))
        {
            if (!fs::create_directories(file_path))
            {
                ROS_WARN("key frame path doesn't exist, and fails to exist!");
            }
        }

        file_path = file_path + to_string(key_index) + ".pcd";

        pcl::io::savePCDFileBinary(file_path, *thisSurfKeyFrame);

        if (surfCloudKeyFramesMap.size() <= 500)
        {
            surfCloudKeyFramesMap[key_index] = thisSurfKeyFrame;
        }
        else
        {
            auto it = surfCloudKeyFramesMap.begin();
            for (int i = 0; i < 100; ++i)
            {
                it = surfCloudKeyFramesMap.erase(it);
            }
            surfCloudKeyFramesMap[key_index] = thisSurfKeyFrame;
        }
        return true;
    }
    void extractSurroundingKeyFrames()
    {
        if (cloudKeyPoses3D->points.empty() == true)
            return;
        extractNearby();
    }

    void downsampleCurrentScan()
    {
        // Downsample cloud from current scan
        laserCloudCornerLastDS->clear();
        // std::cout << "laserCloudCornerLast size is : " << laserCloudCornerLast->size() << std::endl;
        downSizeFilterCorner.setInputCloud(laserCloudCornerLast);
        downSizeFilterCorner.filter(*laserCloudCornerLastDS);
        laserCloudCornerLastDSNum = laserCloudCornerLastDS->size();

        laserCloudSurfLastDS->clear();
        // std::cout << "laserCloudSurfLast size is : " << laserCloudSurfLast->size() << std::endl;
        downSizeFilterSurf.setInputCloud(laserCloudSurfLast);
        downSizeFilterSurf.filter(*laserCloudSurfLastDS);
        laserCloudSurfLastDSNum = laserCloudSurfLastDS->size();
    }

    void updatePointAssociateToMap()
    {
        transPointAssociateToMap = trans2Affine3f(transformTobeMapped);
    }

    void cornerOptimization()
    {
        updatePointAssociateToMap();
#pragma omp parallel for num_threads(numberOfCores)
        for (int i = 0; i < laserCloudCornerLastDSNum; i++)
        {
            PointType pointOri, pointSel, coeff;
            std::vector<int> pointSearchInd;
            std::vector<float> pointSearchSqDis;

            pointOri = laserCloudCornerLastDS->points[i];
            pointAssociateToMap(&pointOri, &pointSel);
            kdtreeCornerFromMap->nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis);

            cv::Mat matA1(3, 3, CV_32F, cv::Scalar::all(0));
            cv::Mat matD1(1, 3, CV_32F, cv::Scalar::all(0));
            cv::Mat matV1(3, 3, CV_32F, cv::Scalar::all(0));

            if (pointSearchSqDis[4] < 1.0)
            {
                float cx = 0, cy = 0, cz = 0;
                for (int j = 0; j < 5; j++)
                {
                    cx += laserCloudCornerFromMapDS->points[pointSearchInd[j]].x;
                    cy += laserCloudCornerFromMapDS->points[pointSearchInd[j]].y;
                    cz += laserCloudCornerFromMapDS->points[pointSearchInd[j]].z;
                }
                cx /= 5;
                cy /= 5;
                cz /= 5;

                float a11 = 0, a12 = 0, a13 = 0, a22 = 0, a23 = 0, a33 = 0;
                for (int j = 0; j < 5; j++)
                {
                    float ax = laserCloudCornerFromMapDS->points[pointSearchInd[j]].x - cx;
                    float ay = laserCloudCornerFromMapDS->points[pointSearchInd[j]].y - cy;
                    float az = laserCloudCornerFromMapDS->points[pointSearchInd[j]].z - cz;

                    a11 += ax * ax;
                    a12 += ax * ay;
                    a13 += ax * az;
                    a22 += ay * ay;
                    a23 += ay * az;
                    a33 += az * az;
                }
                a11 /= 5;
                a12 /= 5;
                a13 /= 5;
                a22 /= 5;
                a23 /= 5;
                a33 /= 5;

                matA1.at<float>(0, 0) = a11;
                matA1.at<float>(0, 1) = a12;
                matA1.at<float>(0, 2) = a13;
                matA1.at<float>(1, 0) = a12;
                matA1.at<float>(1, 1) = a22;
                matA1.at<float>(1, 2) = a23;
                matA1.at<float>(2, 0) = a13;
                matA1.at<float>(2, 1) = a23;
                matA1.at<float>(2, 2) = a33;
                cv::eigen(matA1, matD1, matV1);
                if (matD1.at<float>(0, 0) > 3 * matD1.at<float>(0, 1))
                {

                    float x0 = pointSel.x;
                    float y0 = pointSel.y;
                    float z0 = pointSel.z;
                    float x1 = cx + 0.1 * matV1.at<float>(0, 0);
                    float y1 = cy + 0.1 * matV1.at<float>(0, 1);
                    float z1 = cz + 0.1 * matV1.at<float>(0, 2);
                    float x2 = cx - 0.1 * matV1.at<float>(0, 0);
                    float y2 = cy - 0.1 * matV1.at<float>(0, 1);
                    float z2 = cz - 0.1 * matV1.at<float>(0, 2);
                    
                    float a012 = sqrt(((x0 - x1) * (y0 - y2) - (x0 - x2) * (y0 - y1)) * ((x0 - x1) * (y0 - y2) - (x0 - x2) * (y0 - y1)) + ((x0 - x1) * (z0 - z2) - (x0 - x2) * (z0 - z1)) * ((x0 - x1) * (z0 - z2) - (x0 - x2) * (z0 - z1)) + ((y0 - y1) * (z0 - z2) - (y0 - y2) * (z0 - z1)) * ((y0 - y1) * (z0 - z2) - (y0 - y2) * (z0 - z1)));

                    float l12 = sqrt((x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2) + (z1 - z2) * (z1 - z2));

                    float la = ((y1 - y2) * ((x0 - x1) * (y0 - y2) - (x0 - x2) * (y0 - y1)) + (z1 - z2) * ((x0 - x1) * (z0 - z2) - (x0 - x2) * (z0 - z1))) / a012 / l12;

                    float lb = -((x1 - x2) * ((x0 - x1) * (y0 - y2) - (x0 - x2) * (y0 - y1)) - (z1 - z2) * ((y0 - y1) * (z0 - z2) - (y0 - y2) * (z0 - z1))) / a012 / l12;

                    float lc = -((x1 - x2) * ((x0 - x1) * (z0 - z2) - (x0 - x2) * (z0 - z1)) + (y1 - y2) * ((y0 - y1) * (z0 - z2) - (y0 - y2) * (z0 - z1))) / a012 / l12;

                    float ld2 = a012 / l12;
                    float s = 1 - 0.9 * fabs(ld2);

                    coeff.x = s * la;
                    coeff.y = s * lb;
                    coeff.z = s * lc;
                    coeff.intensity = s * ld2;
                    if (s > 0.1)
                    {
                        laserCloudOriCornerVec[i] = pointOri;
                        coeffSelCornerVec[i] = coeff;
                        laserCloudOriCornerFlag[i] = true;
                    }
                }
            }
        }
    }

    void surfOptimization()
    {
        updatePointAssociateToMap();

#pragma omp parallel for num_threads(numberOfCores)
        for (int i = 0; i < laserCloudSurfLastDSNum; i++)
        {
            PointType pointOri, pointSel, coeff;
            std::vector<int> pointSearchInd;
            std::vector<float> pointSearchSqDis;
            pointOri = laserCloudSurfLastDS->points[i];
            pointAssociateToMap(&pointOri, &pointSel);
            kdtreeSurfFromMap->nearestKSearch(pointSel, 5, pointSearchInd, pointSearchSqDis);

            Eigen::Matrix<float, 5, 3> matA0;
            Eigen::Matrix<float, 5, 1> matB0;
            Eigen::Vector3f matX0;
            matA0.setZero();
            matB0.fill(-1);
            matX0.setZero();
            if (pointSearchSqDis[4] < 1.0)
            {
                for (int j = 0; j < 5; j++)
                {
                    matA0(j, 0) = laserCloudSurfFromMapDS->points[pointSearchInd[j]].x;
                    matA0(j, 1) = laserCloudSurfFromMapDS->points[pointSearchInd[j]].y;
                    matA0(j, 2) = laserCloudSurfFromMapDS->points[pointSearchInd[j]].z;
                }
                matX0 = matA0.colPivHouseholderQr().solve(matB0);
                float pa = matX0(0, 0);
                float pb = matX0(1, 0);
                float pc = matX0(2, 0);
                float pd = 1;

                float ps = sqrt(pa * pa + pb * pb + pc * pc);
                pa /= ps;
                pb /= ps;
                pc /= ps;
                pd /= ps;

                bool planeValid = true;
                for (int j = 0; j < 5; j++)
                {
                    if (fabs(pa * laserCloudSurfFromMapDS->points[pointSearchInd[j]].x +
                             pb * laserCloudSurfFromMapDS->points[pointSearchInd[j]].y +
                             pc * laserCloudSurfFromMapDS->points[pointSearchInd[j]].z + pd) > 0.2)
                    {
                        planeValid = false;
                        break;
                    }
                }
                if (planeValid)
                {
                    float pd2 = pa * pointSel.x + pb * pointSel.y + pc * pointSel.z + pd;
                    float s = 1 - 0.9 * fabs(pd2) / sqrt(sqrt(pointSel.x * pointSel.x + pointSel.y * pointSel.y + pointSel.z * pointSel.z));

                    coeff.x = s * pa;
                    coeff.y = s * pb;
                    coeff.z = s * pc;
                    coeff.intensity = s * pd2;

                    if (s > 0.1)
                    {
                        laserCloudOriSurfVec[i] = pointOri;
                        coeffSelSurfVec[i] = coeff;
                        laserCloudOriSurfFlag[i] = true;
                    }
                }
            }
        }
    }
    
    void combineOptimizationCoeffs()
    {
        // combine corner coeffs
        for (int i = 0; i < laserCloudCornerLastDSNum; ++i)
        {
            if (laserCloudOriCornerFlag[i] == true)
            {
                laserCloudOri->push_back(laserCloudOriCornerVec[i]);
                coeffSel->push_back(coeffSelCornerVec[i]);
            }
        }
        // combine surf coeffs
        for (int i = 0; i < laserCloudSurfLastDSNum; ++i)
        {
            if (laserCloudOriSurfFlag[i] == true)
            {
                laserCloudOri->push_back(laserCloudOriSurfVec[i]);
                coeffSel->push_back(coeffSelSurfVec[i]);
            }
        }
        // reset flag for next iteration
        std::fill(laserCloudOriCornerFlag.begin(), laserCloudOriCornerFlag.end(), false);
        std::fill(laserCloudOriSurfFlag.begin(), laserCloudOriSurfFlag.end(), false);
    }

    bool LMOptimization(int iterCount)
    {
        // This optimization is from the original loam_velodyne by Ji Zhang, need to cope with coordinate transformation
        // lidar <- camera      ---     camera <- lidar
        // x = z                ---     x = y
        // y = x                ---     y = z
        // z = y                ---     z = x
        // roll = yaw           ---     roll = pitch
        // pitch = roll         ---     pitch = yaw
        // yaw = pitch          ---     yaw = roll

        // lidar -> camera
        float srx = sin(transformTobeMapped[1]);
        float crx = cos(transformTobeMapped[1]);
        float sry = sin(transformTobeMapped[2]);
        float cry = cos(transformTobeMapped[2]);
        float srz = sin(transformTobeMapped[0]);
        float crz = cos(transformTobeMapped[0]);

        int laserCloudSelNum = laserCloudOri->size();
        if (laserCloudSelNum < 50)
        {
            return false;
        }

        cv::Mat matA(laserCloudSelNum, 6, CV_32F, cv::Scalar::all(0));
        cv::Mat matAt(6, laserCloudSelNum, CV_32F, cv::Scalar::all(0));
        cv::Mat matAtA(6, 6, CV_32F, cv::Scalar::all(0));
        cv::Mat matB(laserCloudSelNum, 1, CV_32F, cv::Scalar::all(0));
        cv::Mat matAtB(6, 1, CV_32F, cv::Scalar::all(0));
        cv::Mat matX(6, 1, CV_32F, cv::Scalar::all(0));

        PointType pointOri, coeff;

        for (int i = 0; i < laserCloudSelNum; i++)
        {
            // lidar -> camera
            pointOri.x = laserCloudOri->points[i].y;
            pointOri.y = laserCloudOri->points[i].z;
            pointOri.z = laserCloudOri->points[i].x;
            // lidar -> camera
            coeff.x = coeffSel->points[i].y;
            coeff.y = coeffSel->points[i].z;
            coeff.z = coeffSel->points[i].x;
            coeff.intensity = coeffSel->points[i].intensity;
            // in camera
            float arx = (crx * sry * srz * pointOri.x + crx * crz * sry * pointOri.y - srx * sry * pointOri.z) * coeff.x + (-srx * srz * pointOri.x - crz * srx * pointOri.y - crx * pointOri.z) * coeff.y + (crx * cry * srz * pointOri.x + crx * cry * crz * pointOri.y - cry * srx * pointOri.z) * coeff.z;

            float ary = ((cry * srx * srz - crz * sry) * pointOri.x + (sry * srz + cry * crz * srx) * pointOri.y + crx * cry * pointOri.z) * coeff.x + ((-cry * crz - srx * sry * srz) * pointOri.x + (cry * srz - crz * srx * sry) * pointOri.y - crx * sry * pointOri.z) * coeff.z;

            float arz = ((crz * srx * sry - cry * srz) * pointOri.x + (-cry * crz - srx * sry * srz) * pointOri.y) * coeff.x + (crx * crz * pointOri.x - crx * srz * pointOri.y) * coeff.y + ((sry * srz + cry * crz * srx) * pointOri.x + (crz * sry - cry * srx * srz) * pointOri.y) * coeff.z;
            // lidar -> camera
            matA.at<float>(i, 0) = arz;
            matA.at<float>(i, 1) = arx;
            matA.at<float>(i, 2) = ary;
            matA.at<float>(i, 3) = coeff.z;
            matA.at<float>(i, 4) = coeff.x;
            matA.at<float>(i, 5) = coeff.y;
            matB.at<float>(i, 0) = -coeff.intensity;
        }
        cv::transpose(matA, matAt);
        matAtA = matAt * matA;
        matAtB = matAt * matB;
        cv::solve(matAtA, matAtB, matX, cv::DECOMP_QR);

        if (iterCount == 0)
        {
            cv::Mat matE(1, 6, CV_32F, cv::Scalar::all(0));
            cv::Mat matV(6, 6, CV_32F, cv::Scalar::all(0)); 
            cv::Mat matV2(6, 6, CV_32F, cv::Scalar::all(0));
            cv::eigen(matAtA, matE, matV);
            matV.copyTo(matV2);

            isDegenerate = false;
            float eignThre[6] = {100, 100, 100, 100, 100, 100};
            for (int i = 5; i >= 0; i--)
            {
                if (matE.at<float>(0, i) < eignThre[i])
                {
                    for (int j = 0; j < 6; j++)
                    {
                        matV2.at<float>(i, j) = 0;
                    }
                    isDegenerate = true;
                    // cout << " Degenerate 1 :" << isDegenerate << endl;
                }
                else
                {
                    break;
                }
            }
            matP = matV.inv() * matV2;
        }
        if (isDegenerate)
        {
            cv::Mat matX2(6, 1, CV_32F, cv::Scalar::all(0));
            matX.copyTo(matX2);
            matX = matP * matX2;
        }
        transformTobeMapped[0] += matX.at<float>(0, 0);
        transformTobeMapped[1] += matX.at<float>(1, 0);
        transformTobeMapped[2] += matX.at<float>(2, 0);
        transformTobeMapped[3] += matX.at<float>(3, 0);
        transformTobeMapped[4] += matX.at<float>(4, 0);
        transformTobeMapped[5] += matX.at<float>(5, 0);
        float deltaR = sqrt(
            pow(pcl::rad2deg(matX.at<float>(0, 0)), 2) +
            pow(pcl::rad2deg(matX.at<float>(1, 0)), 2) +
            pow(pcl::rad2deg(matX.at<float>(2, 0)), 2));
        float deltaT = sqrt(
            pow(matX.at<float>(3, 0) * 100, 2) +
            pow(matX.at<float>(4, 0) * 100, 2) +
            pow(matX.at<float>(5, 0) * 100, 2));
        if (deltaR < 0.05 && deltaT < 0.05)
        {
            return true; // converged
        }
        return false; // keep optimizing
    }

    void scan2MapOptimization()
    {
        if (cloudKeyPoses3D->points.empty())
            return;
        // std::cout << "laserCloudCornerLastDSNum is : " << laserCloudCornerLastDSNum << std::endl; 
        // std::cout << "laserCloudSurfLastDSNum is : " << laserCloudSurfLastDSNum << std::endl; 
        if (laserCloudCornerLastDSNum > edgeFeatureMinValidNum && laserCloudSurfLastDSNum > surfFeatureMinValidNum)
        {
            kdtreeCornerFromMap->setInputCloud(laserCloudCornerFromMapDS);
            kdtreeSurfFromMap->setInputCloud(laserCloudSurfFromMapDS);
            for (int iterCount = 0; iterCount < 30; iterCount++)
            {
                laserCloudOri->clear();
                coeffSel->clear();

                cornerOptimization();
                surfOptimization();

                combineOptimizationCoeffs();

                if (LMOptimization(iterCount) == true)
                    break;
            }
            transformUpdate();
        }
        else
        {
            ROS_WARN("Not enough features! Only %d edge and %d planar features available.", laserCloudCornerLastDSNum, laserCloudSurfLastDSNum);
        }
    }

    void transformUpdate()
    {
        if (cloudInfo.imuAvailable == true)
        {
            if (std::abs(cloudInfo.imuPitchInit) < 1.4)
            {
                double imuWeight = imuRPYWeight;
                tf::Quaternion imuQuaternion;
                tf::Quaternion transformQuaternion;
                double rollMid, pitchMid, yawMid;

                // slerp roll
                transformQuaternion.setRPY(transformTobeMapped[0], 0, 0);
                imuQuaternion.setRPY(cloudInfo.imuRollInit, 0, 0);
                tf::Matrix3x3(transformQuaternion.slerp(imuQuaternion, imuWeight)).getRPY(rollMid, pitchMid, yawMid);
                transformTobeMapped[0] = rollMid;

                // slerp pitch
                transformQuaternion.setRPY(0, transformTobeMapped[1], 0);
                imuQuaternion.setRPY(0, cloudInfo.imuPitchInit, 0);
                tf::Matrix3x3(transformQuaternion.slerp(imuQuaternion, imuWeight)).getRPY(rollMid, pitchMid, yawMid);
                transformTobeMapped[1] = pitchMid;
            }
        }
        transformTobeMapped[0] = constraintTransformation(transformTobeMapped[0], rotation_tollerance);
        transformTobeMapped[1] = constraintTransformation(transformTobeMapped[1], rotation_tollerance);
        transformTobeMapped[5] = constraintTransformation(transformTobeMapped[5], z_tollerance);
        incrementalOdometryAffineBack = trans2Affine3f(transformTobeMapped);
    }

    float constraintTransformation(float value, float limit)
    {
        if (value < -limit)
            value = -limit;
        if (value > limit)
            value = limit;

        return value;
    }

    bool saveFrame()
    {
        if (cloudKeyPoses3D->points.empty())
            return true;
        Eigen::Affine3f transStart = pclPointToAffine3f(cloudKeyPoses6D->back());
        Eigen::Affine3f transFinal = pcl::getTransformation(transformTobeMapped[3], transformTobeMapped[4], transformTobeMapped[5],
                                                            transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]);
        Eigen::Affine3f transBetween = transStart.inverse() * transFinal;
        float x, y, z, roll, pitch, yaw;
        pcl::getTranslationAndEulerAngles(transBetween, x, y, z, roll, pitch, yaw);
        if (abs(roll) < surroundingkeyframeAddingAngleThreshold &&
            abs(pitch) < surroundingkeyframeAddingAngleThreshold &&
            abs(yaw) < surroundingkeyframeAddingAngleThreshold &&
            sqrt(x * x + y * y + z * z) < surroundingkeyframeAddingDistThreshold)
            return false;

        return true;
    }

    void addOdomFactor()
    {
        if (cloudKeyPoses3D->points.empty())
        {
            noiseModel::Diagonal::shared_ptr priorNoise = noiseModel::Diagonal::Variances((Vector(6) << 1e-2, 1e-2, M_PI * M_PI, 1e8, 1e8, 1e8).finished()); // rad*rad, meter*meter
            gtSAMgraph.add(PriorFactor<Pose3>(0, trans2gtsamPose(transformTobeMapped), priorNoise));

            initialEstimate.insert(0, trans2gtsamPose(transformTobeMapped));
        }
        else
        {
            noiseModel::Diagonal::shared_ptr odometryNoise = noiseModel::Diagonal::Variances((Vector(6) << 1e-6, 1e-6, 1e-6, 1e-4, 1e-4, 1e-4).finished());
            gtsam::Pose3 poseFrom = pclPointTogtsamPose3(cloudKeyPoses6D->points.back());
            gtsam::Pose3 poseTo = trans2gtsamPose(transformTobeMapped);
            gtSAMgraph.add(BetweenFactor<Pose3>(cloudKeyPoses3D->size() - 1, cloudKeyPoses3D->size(), poseFrom.between(poseTo), odometryNoise));

            initialEstimate.insert(cloudKeyPoses3D->size(), poseTo);
        }
    }

    void addGPSFactor()
    {
        if (cloudKeyPoses3D->points.empty())
        {
            std::cout << "cloudKeyPoses3D->points.empty()" << std::endl;
            return;
        }
        else
        {
            if (pointDistance(cloudKeyPoses3D->front(), cloudKeyPoses3D->back()) < 5.0)
            {
                // std::cout << "pointDistance(cloudKeyPoses3D->front(), cloudKeyPoses3D->back()) < 5.0" << std::endl;
                // return;
            }
        }

        if (poseCovariance(3, 3) < poseCovThreshold && poseCovariance(4, 4) < poseCovThreshold)
        {
            // std::cout << "poseCovariance(3, 3) < poseCovThreshold && poseCovariance(4, 4) < poseCovThreshold" << std::endl;
            // std::cout << "poseCovariance(3, 3) is " << poseCovariance(3, 3) << std::endl;
            // std::cout << "poseCovariance(4, 4) is " << poseCovariance(4, 4) << std::endl;
            // return;
        }
        // last gps position
        static PointType lastGPSPoint;
        float noise_x = 0.15;
        float noise_y = 0.15;
        // float noise_z = 0.15;
        if (noise_x > gpsCovThreshold || noise_y > gpsCovThreshold)
        {
            // std::cout << "noise_x > gpsCovThreshold || noise_y > gpsCovThreshold" << std::endl;
            // return;
        }

        float gps_x = GpsTransValues[0];
        float gps_y = GpsTransValues[1];
        float gps_z = GpsTransValues[2];

        // if (!useGpsElevation)
        // {
        //     gps_z = transformTobeMapped[5];
        //     noise_z = 0.01;
        // }

        // GPS not properly initialized (0,0,0)
       if (abs(gps_x) < 1e-6 && abs(gps_y) < 1e-6)
        {
            // std::cout << "abs(gps_x) < 1e-6 && abs(gps_y) < 1e-6" << std::endl;
            return;
        }

        // Add GPS every a few meters
        PointType curGPSPoint;
        curGPSPoint.x = gps_x;
        curGPSPoint.y = gps_y;
        curGPSPoint.z = gps_z;
        if (pointDistance(curGPSPoint, lastGPSPoint) < gpsAddInterval)
        {
            // std::cout << "pointDistance(curGPSPoint, lastGPSPoint): " << pointDistance(curGPSPoint, lastGPSPoint) << std::endl;
            return;
        }
        else
            lastGPSPoint = curGPSPoint;

        gtsam::Vector gpsNoiseVector(3);
        gpsNoiseVector << gps_noise, gps_noise, gps_noise;
        noiseModel::Diagonal::shared_ptr gpsNoiseModel = noiseModel::Diagonal::Variances(gpsNoiseVector);
        gtsam::GPSFactor gps_factor(cloudKeyPoses3D->size(), gtsam::Point3(gps_x, gps_y, gps_z), gpsNoiseModel);

        gtSAMgraph.add(gps_factor);
        aLoopIsClosed = true;

    }

    void addLoopFactor()
    {
        // cout << "loopClosureDetectInterval" << loopClosureDetectInterval << endl;
        if (keyFramesNum % loopClosureDetectInterval == 0 && totalFramesNum >= loopClosureDetectStartTime)
        {
            performLoopClosure();
            visualizeLoopClosure();
        }

        if (loopIndexQueue.empty())
            return;
        for (int i = 0; i < (int)loopIndexQueue.size(); ++i)
        {
            int indexFrom = loopIndexQueue[i].first;
            int indexTo = loopIndexQueue[i].second;
            gtsam::Pose3 poseBetween = loopPoseQueue[i];
            gtsam::noiseModel::Diagonal::shared_ptr noiseBetween = loopNoiseQueue[i];
            gtSAMgraph.add(BetweenFactor<Pose3>(indexFrom, indexTo, poseBetween, noiseBetween));
            
        }
        loopIndexQueue.clear();
        loopPoseQueue.clear();
        loopNoiseQueue.clear();
        aLoopIsClosed = true;
    }

    void saveKeyFramesAndFactor()
    {
        if (saveFrame() == false)
        {
            return;
        }
        // io save
        keyFramesNum++;

        addOdomFactor();
        
        if (GPS_Trigger)
        {
            addGPSFactor();
        }
        // loop factor
        addLoopFactor();

        // update iSAM
        isam->update(gtSAMgraph, initialEstimate);
        isam->update();
        if (aLoopIsClosed == true)
        {
            isam->update();
            isam->update();
            isam->update();
            isam->update();
            isam->update();
        }
        gtSAMgraph.resize(0);
        initialEstimate.clear();

        // save key poses
        PointType thisPose3D;
        PointTypePose thisPose6D;
        Pose3 latestEstimate;
        isamCurrentEstimate = isam->calculateEstimate();

        latestEstimate = isamCurrentEstimate.at<Pose3>(isamCurrentEstimate.size() - 1);
        thisPose3D.x = latestEstimate.translation().x();
        thisPose3D.y = latestEstimate.translation().y();
        thisPose3D.z = latestEstimate.translation().z();
        thisPose3D.intensity = cloudKeyPoses3D->size(); // this can be used as index
        cloudKeyPoses3D->push_back(thisPose3D);
        thisPose6D.x = thisPose3D.x;
        thisPose6D.y = thisPose3D.y;
        thisPose6D.z = thisPose3D.z;
        thisPose6D.intensity = thisPose3D.intensity; // this can be used as index
        thisPose6D.roll = latestEstimate.rotation().roll();
        thisPose6D.pitch = latestEstimate.rotation().pitch();
        thisPose6D.yaw = latestEstimate.rotation().yaw();
        thisPose6D.time = timeLaserInfoCur;
        cloudKeyPoses6D->push_back(thisPose6D);

        poseCovariance = isam->marginalCovariance(isamCurrentEstimate.size() - 1);

        // save updated transform
        transformTobeMapped[0] = latestEstimate.rotation().roll();
        transformTobeMapped[1] = latestEstimate.rotation().pitch();
        transformTobeMapped[2] = latestEstimate.rotation().yaw();
        transformTobeMapped[3] = latestEstimate.translation().x();
        transformTobeMapped[4] = latestEstimate.translation().y();
        transformTobeMapped[5] = latestEstimate.translation().z();

        // save all the received edge and surf points
        pcl::PointCloud<PointType>::Ptr thisCornerKeyFrame(new pcl::PointCloud<PointType>());
        pcl::PointCloud<PointType>::Ptr thisSurfKeyFrame(new pcl::PointCloud<PointType>());
        pcl::copyPointCloud(*laserCloudCornerLastDS, *thisCornerKeyFrame);
        pcl::copyPointCloud(*laserCloudSurfLastDS, *thisSurfKeyFrame);

        // save key frame cloud
        // add for memory opt
        setCornerKeyFrame(thisCornerKeyFrame, keyFramesNum - 1);
        setSurfKeyFrame(thisSurfKeyFrame, keyFramesNum - 1);
        updatePath(thisPose6D);
    }

    void correctPoses()
    {

        if (cloudKeyPoses3D->points.empty())
            return;
        if (aLoopIsClosed == true)
        {
            // clear map cache
            laserCloudMapContainer.clear();
            // clear path
            globalPath.poses.clear();
            // update key poses
            int numPoses = isamCurrentEstimate.size();
            for (int i = 0; i < numPoses; ++i)
            {
                cloudKeyPoses3D->points[i].x = isamCurrentEstimate.at<Pose3>(i).translation().x();
                cloudKeyPoses3D->points[i].y = isamCurrentEstimate.at<Pose3>(i).translation().y();
                cloudKeyPoses3D->points[i].z = isamCurrentEstimate.at<Pose3>(i).translation().z();

                cloudKeyPoses6D->points[i].x = cloudKeyPoses3D->points[i].x;
                cloudKeyPoses6D->points[i].y = cloudKeyPoses3D->points[i].y;
                cloudKeyPoses6D->points[i].z = cloudKeyPoses3D->points[i].z;
                cloudKeyPoses6D->points[i].roll = isamCurrentEstimate.at<Pose3>(i).rotation().roll();
                cloudKeyPoses6D->points[i].pitch = isamCurrentEstimate.at<Pose3>(i).rotation().pitch();
                cloudKeyPoses6D->points[i].yaw = isamCurrentEstimate.at<Pose3>(i).rotation().yaw();
                updatePath(cloudKeyPoses6D->points[i]);
            }
            aLoopIsClosed = false;
        }
    }

    void updatePath(const PointTypePose &pose_in)
    {
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header.stamp = ros::Time().fromSec(pose_in.time);
        pose_stamped.header.frame_id = odometryFrame;
        pose_stamped.pose.position.x = pose_in.x;
        pose_stamped.pose.position.y = pose_in.y;
        pose_stamped.pose.position.z = pose_in.z;
        tf::Quaternion q = tf::createQuaternionFromRPY(pose_in.roll, pose_in.pitch, pose_in.yaw);
        pose_stamped.pose.orientation.x = q.x();
        pose_stamped.pose.orientation.y = q.y();
        pose_stamped.pose.orientation.z = q.z();
        pose_stamped.pose.orientation.w = q.w();

        globalPath.poses.push_back(pose_stamped);
    }

    void publishOdometry()
    {
        // Publish odometry for ROS (global)
        nav_msgs::Odometry laserOdometryROS;
        laserOdometryROS.header.stamp = timeLaserInfoStamp;
        laserOdometryROS.header.frame_id = odometryFrame;
        laserOdometryROS.child_frame_id = "odom_mapping";
        laserOdometryROS.pose.pose.position.x = transformTobeMapped[3];
        laserOdometryROS.pose.pose.position.y = transformTobeMapped[4];
        laserOdometryROS.pose.pose.position.z = transformTobeMapped[5];
        laserOdometryROS.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]);
        pubLaserOdometryGlobal.publish(laserOdometryROS);

        // Publish TF
        static tf::TransformBroadcaster br;
        tf::Transform t_odom_to_lidar = tf::Transform(tf::createQuaternionFromRPY(transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]),
                                                      tf::Vector3(transformTobeMapped[3], transformTobeMapped[4], transformTobeMapped[5]));
        tf::StampedTransform trans_odom_to_lidar = tf::StampedTransform(t_odom_to_lidar, timeLaserInfoStamp, odometryFrame, "lidar_link");
        br.sendTransform(trans_odom_to_lidar);

        // Publish odometry for ROS (incremental)
        static bool lastIncreOdomPubFlag = false;
        static nav_msgs::Odometry laserOdomIncremental; // incremental odometry msg
        static Eigen::Affine3f increOdomAffine;         // incremental odometry in affine

        if (lastIncreOdomPubFlag == false)
        {
            lastIncreOdomPubFlag = true;
            laserOdomIncremental = laserOdometryROS;
            increOdomAffine = trans2Affine3f(transformTobeMapped);
        }
        else
        {
            // incrementalOdometryAffineFront： last transformToBeMapped
            // incrementalOdometryAffineBack： now transformToBeMapped
            Eigen::Affine3f affineIncre = incrementalOdometryAffineFront.inverse() * incrementalOdometryAffineBack;
            increOdomAffine = increOdomAffine * affineIncre;
            float x, y, z, roll, pitch, yaw;
            pcl::getTranslationAndEulerAngles(increOdomAffine, x, y, z, roll, pitch, yaw);
            if (cloudInfo.imuAvailable == true)
            {
                if (std::abs(cloudInfo.imuPitchInit) < 1.4)
                {
                    double imuWeight = 0.1;
                    tf::Quaternion imuQuaternion;
                    tf::Quaternion transformQuaternion;
                    double rollMid, pitchMid, yawMid;

                    // slerp roll
                    transformQuaternion.setRPY(roll, 0, 0);
                    imuQuaternion.setRPY(cloudInfo.imuRollInit, 0, 0);
                    tf::Matrix3x3(transformQuaternion.slerp(imuQuaternion, imuWeight)).getRPY(rollMid, pitchMid, yawMid);
                    roll = rollMid;

                    // slerp pitch
                    transformQuaternion.setRPY(0, pitch, 0);
                    imuQuaternion.setRPY(0, cloudInfo.imuPitchInit, 0);
                    tf::Matrix3x3(transformQuaternion.slerp(imuQuaternion, imuWeight)).getRPY(rollMid, pitchMid, yawMid);
                    pitch = pitchMid;
                }
            }
            laserOdomIncremental.header.stamp = timeLaserInfoStamp;
            laserOdomIncremental.header.frame_id = odometryFrame;
            laserOdomIncremental.child_frame_id = "odom_mapping";
            laserOdomIncremental.pose.pose.position.x = x;
            laserOdomIncremental.pose.pose.position.y = y;
            laserOdomIncremental.pose.pose.position.z = z;
            laserOdomIncremental.pose.pose.orientation = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);

            if (isDegenerate)
                laserOdomIncremental.pose.covariance[0] = 1;
            else
                laserOdomIncremental.pose.covariance[0] = 0;
        }
        pubLaserOdometryIncremental.publish(laserOdomIncremental);
    }
    void publishFrames()
    {
        if (cloudKeyPoses3D->points.empty())
            return;
        // publish key poses
        publishCloud(&pubKeyPoses, cloudKeyPoses3D, timeLaserInfoStamp, odometryFrame);
        // Publish surrounding key frames
        publishCloud(&pubRecentKeyFrames, laserCloudSurfFromMap, timeLaserInfoStamp, odometryFrame);
        // publish registered key frame
        pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());
        PointTypePose thisPose6D = trans2PointTypePose(transformTobeMapped);
        // 把当前点云转换到世界坐标系下去
        *cloudOut += *transformPointCloud(laserCloudCornerLast, &thisPose6D);
        *cloudOut += *transformPointCloud(laserCloudSurfLast, &thisPose6D);
        // 发送当前点云
        publishCloud(&pubRecentKeyFrame, cloudOut, timeLaserInfoStamp, odometryFrame);
        // publish registered high-res raw cloud
        if (pubCloudRegisteredRaw.getNumSubscribers() != 0)
        {
            pcl::PointCloud<PointType>::Ptr cloudOut(new pcl::PointCloud<PointType>());
            pcl::fromROSMsg(cloudInfo.cloud_deskewed, *cloudOut);
            PointTypePose thisPose6D = trans2PointTypePose(transformTobeMapped);
            *cloudOut = *transformPointCloud(cloudOut, &thisPose6D);
            // 发送原始点云
            publishCloud(&pubCloudRegisteredRaw, cloudOut, timeLaserInfoStamp, odometryFrame);
        }
        // publish path
        if (pubPath.getNumSubscribers() != 0)
        {
            globalPath.header.stamp = timeLaserInfoStamp;
            globalPath.header.frame_id = odometryFrame;
            pubPath.publish(globalPath);
        }
    }

    void writeLoopClosureInfo(int prev, int cur, double icpScore)
    {
        std::ofstream out(savePCDDirectory + "loop_info.txt", std::ios::app | std::ios::out);

        if (!out)
        {
            std::cerr << "loop_info.txt Error opening file!" << std::endl;
            return;
        }
        out << "<curKFID,prevKFID><" << cur << "," << prev << ">";
        out.precision(16);
        out << "    <curTime><" << timeLaserInfoCur << ">";
        out.precision(6);
        out << "<icpScore>" << icpScore << endl;

        out.close();
    }

    void writeGPSPose3D(float x, float y, float z, double time)
    {
        if (!out)
        {
            std::cerr << "Failed to open file for writing gps 3D poses!" << std::endl;
            return;
        }

        out.precision(16);
        out << "time_stamp: " << time << " gps_pose: " << x << ", " << y << ", " << z << "(x y z)" << endl;

        out.flush();
    }
    void writeGPSPose6D(const sensor_msgs::NavSatFix::ConstPtr &gpsMsg)
    {

        if (!gps_pose)
        {
            std::cerr << "Failed to open file for writing gps info!" << std::endl;
            return;
        }

        if (writeGPSPose6DJsonInit == false)
        {
            gps_pose << "{" << endl;
            writeGPSPose6DJsonInit = true;
        }
        else
        {
            gps_pose << "," << endl;
        }
        gps_pose.precision(17);

        gps_pose << "\"" << gpsMsg->header.stamp << "\":" << endl
                 << "{" << endl;

        gps_pose << "\"gps\": ["
                 << gpsMsg->latitude << ","
                 << gpsMsg->longitude << ","
                 << gpsMsg->altitude << "],"
                 << endl;

        gps_pose << "\"orientation\": ["
                 << tf::createQuaternionMsgFromRollPitchYaw(transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]).x << "," << tf::createQuaternionMsgFromRollPitchYaw(transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]).y << "," << tf::createQuaternionMsgFromRollPitchYaw(transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]).z << "," << tf::createQuaternionMsgFromRollPitchYaw(transformTobeMapped[0], transformTobeMapped[1], transformTobeMapped[2]).w << "]" << endl
                 << "}";

         gps_pose.flush();
    }
    void writeGPSPose6D()
    {
        if (!gps_pose)
        {
            std::cerr << "Error opening file!" << std::endl;
            return;
        }

        gps_pose << endl
                 << "}";
        gps_pose.close();
    }
    void writeGlobalPosesJson()
    {
        std::ofstream out(savePCDDirectory + "pose_data.json", std::ios::trunc | std::ios::out);

        if (!out)
        {
            std::cerr << "Error opening file!" << std::endl;
            return;
        }
        out << "{" << endl;
        out.precision(19);

        float x, y, z, roll, pitch, yaw;
        double time;

        geometry_msgs::Quaternion orientation;

        for (size_t i = 0; i < cloudKeyPoses6D->size(); i++)
        {
            x = cloudKeyPoses6D->points[i].x;
            y = cloudKeyPoses6D->points[i].y;
            z = cloudKeyPoses6D->points[i].z;
            roll = cloudKeyPoses6D->points[i].roll;
            pitch = cloudKeyPoses6D->points[i].pitch;
            yaw = cloudKeyPoses6D->points[i].yaw;

            orientation = tf::createQuaternionMsgFromRollPitchYaw(roll, pitch, yaw);
            time = cloudKeyPoses6D->points[i].time;
            if (!PoseDataFileFirstFlag)
            {
                out << "\"" << time << "\" "
                    << ":" << endl
                    << "{" << endl
                    << "\"position\": ["
                    << x << ","
                    << y << ","
                    << z << "],"
                    << endl
                    << "\"orientation\": ["
                    << orientation.x << "," << orientation.y << "," << orientation.z << "," << orientation.w << "]"
                    << endl
                    << "}";
                PoseDataFileFirstFlag = true;
            }
            else
            {
                out << "," << endl
                    << "\"" << time << "\" "
                    << ":" << endl
                    << "{" << endl
                    << "\"position\": ["
                    << x << ","
                    << y << ","
                    << z << "],"
                    << endl
                    << "\"orientation\": ["
                    << orientation.x << "," << orientation.y << "," << orientation.z << "," << orientation.w << "]"
                    << endl
                    << "}";
            }
        }

        out << endl
            << "}" << endl;
        out.close();
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lio_sam");

    mapOptimization MO;

    ROS_INFO("\033[1;32m----> Map Optimization Started.\033[0m");

    // std::thread loopthread(&mapOptimization::loopClosureThread, &MO);
    std::thread visualizeMapThread(&mapOptimization::visualizeGlobalMapThread, &MO);

    ros::spin();

    // loopthread.join();
    visualizeMapThread.join();

    return 0;
}
