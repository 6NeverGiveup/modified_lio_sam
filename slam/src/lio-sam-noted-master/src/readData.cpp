#include "utility.h"

#include <unistd.h>
#include <mutex>
#include <condition_variable>

#include <ros/ros.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/Bool.h>

#include <boost/make_shared.hpp>

struct VelodynePointXYZIRT
{
    PCL_ADD_POINT4D
    std::uint16_t intensity;
    std::uint16_t ring;
    double timestamp;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(VelodynePointXYZIRT,
                                  (float, x, x)(float, y, y)(float, z, z)(std::uint16_t, intensity, intensity)(std::uint16_t, ring, ring)(double, timestamp, timestamp))

class ReadData : public ParamServer
{
public:
    ros::NodeHandle nh;

    // publish and subcribe
    ros::Publisher pubRawLidarPoints;
    ros::Publisher pubRawImuData;
    ros::Publisher pubRawGPSData;
    ros::Publisher pubAllMsgFinished;

    ros::Subscriber subImu;
    ros::Subscriber subLaserCloud;

    ros::Subscriber subMOIsFinished;

    // data cache
    rosbag::Bag bag;
    sensor_msgs::Imu::Ptr imu_msg;
    sensor_msgs::PointCloud2::Ptr lidar_msg;
    sensor_msgs::NavSatFix::Ptr gps_msg;

    string curTopicName;
    int count_lidar = 0;
    int count_imu = 0;
    int count_gps = 0;
    bool cacheDataFinished = false;
    bool isSending = true;

    std::deque<sensor_msgs::Imu::Ptr> imuSendQueue1;
    std::deque<sensor_msgs::PointCloud2::Ptr> lidarSendQueue1;
    std::deque<sensor_msgs::NavSatFix::Ptr> gpsSendQueue1;
    std::deque<sensor_msgs::Imu::Ptr> imuSendQueue2;
    std::deque<sensor_msgs::PointCloud2::Ptr> lidarSendQueue2;
    std::deque<sensor_msgs::NavSatFix::Ptr> gpsSendQueue2;
    // send msg
    sensor_msgs::Imu imuMsg;
    sensor_msgs::NavSatFix gpsMsg;
    sensor_msgs::PointCloud2 msg1;
    sensor_msgs::PointCloud2 msg2;

    int count_lidar_msg = 0;
    int count_imu_msg = 0;
    int count_gps_msg = 0;

    int totalLidarMsgNum = 0;

    std::mutex MO;
    bool MOIsFinished = true;
    bool firstMsgSended = false;

    // read and send sync control
    std::condition_variable cv;
    std::mutex cvLock;

    ReadData()
    {
        pubRawLidarPoints = nh.advertise<sensor_msgs::PointCloud2>(pointCloudTopic + "_read", 5);
        pubRawImuData = nh.advertise<sensor_msgs::Imu>(imuTopic + "_read", 2000);
        pubRawGPSData = nh.advertise<sensor_msgs::NavSatFix>(gpsTopic + "_read", 2000);

        pubAllMsgFinished = nh.advertise<std_msgs::Bool>("lio_sam/save_map_request_from_read", 1);
        subMOIsFinished = nh.subscribe<std_msgs::Bool>("lio_sam/mapping/MOIsFinished", 1, &ReadData::flagHandler, this, ros::TransportHints().tcpNoDelay());

        time_t start_givemetime = time(NULL);
        printf("开始时间是： %s", ctime(&start_givemetime));
    }

    void flagHandler(const std_msgs::Bool::ConstPtr &msgIn)
    {
        setMOIsFinished(msgIn->data);
    }
    void setMOIsFinished(bool mo)
    {
        std::lock_guard<std::mutex> mo_lock(MO);
        MOIsFinished = mo;
    }
    bool getMOIsFinished()
    {
        std::lock_guard<std::mutex> mo_lock(MO);
        return MOIsFinished;
    }

    void cacheDataTread()
    {
        std::unique_lock<std::mutex> lock(cvLock);
        ros::Rate rate(readBagFrequency);

        bag.open(readBagDirectory, rosbag::bagmode::Read);

        totalLidarMsgNum = 0;
        for (rosbag::MessageInstance const m : rosbag::View(bag))
        {
            if (m.getTopic() == pointCloudTopic)
            {
                totalLidarMsgNum++;
            }
        }
        if (totalLidarMsgNum <= 10)
        {
            cout << "there are only " << totalLidarMsgNum << " lidar msgs in the bag, return!" << endl;
            ros::shutdown();
        }
        cout << "Read data is ready, there are totally " << totalLidarMsgNum << " lidar msgs in the bag !" << endl;

        for (rosbag::MessageInstance const m : rosbag::View(bag))
        {
            rate.sleep();

            if (ros::ok())
            {
                curTopicName = m.getTopic();

                if (curTopicName == imuTopic)
                {
                    imu_msg = m.instantiate<sensor_msgs::Imu>();
                    if (datatype == DataType::CUS2 || datatype == DataType::CUS4)
                    {
                        imu_msg->linear_acceleration.z = imu_msg->linear_acceleration.z * -1.0;
                    }
                    pushToImuSendQueue(imu_msg);
                }

                if (curTopicName == pointCloudTopic)
                {
                    lidar_msg = m.instantiate<sensor_msgs::PointCloud2>();
                    if (datatype == DataType::CUS2)
                    {
                        // ros::Time current_scan_time = lidar_msg->header.stamp;
                        pcl::PointCloud<VelodynePointXYZIRT>::Ptr laserCloudIn(new pcl::PointCloud<VelodynePointXYZIRT>);
                        pcl::fromROSMsg(*lidar_msg, *laserCloudIn);

                        Eigen::Quaternionf quaternion_1;
                        quaternion_1.x() = 0.0046067;
                        quaternion_1.y() = -0.0222089;
                        quaternion_1.z() = -0.1002373;
                        quaternion_1.w() = 0.994705;

                        Eigen::Affine3f transform = Eigen::Affine3f::Identity();
                        transform.translation() << 0.896555, -0.00838634, 1.65593;
                        transform.rotate(quaternion_1);

                        pcl::PointCloud<VelodynePointXYZIRT>::Ptr transformed_cloud(new pcl::PointCloud<VelodynePointXYZIRT>);
                        pcl::transformPointCloud(*laserCloudIn, *transformed_cloud, transform);

                        sensor_msgs::PointCloud2 point_mmsg;
                        pcl::toROSMsg(*transformed_cloud, point_mmsg);
                        point_mmsg.header.stamp = lidar_msg->header.stamp;
                        point_mmsg.header.frame_id = lidar_msg->header.frame_id;
                        *lidar_msg = point_mmsg;
                        // duhang
                    }
                    if (datatype == DataType::CUS4)
                    {
                        // ros::Time current_scan_time = lidar_msg->header.stamp;
                        pcl::PointCloud<VelodynePointXYZIRT>::Ptr laserCloudIn(new pcl::PointCloud<VelodynePointXYZIRT>);
                        pcl::fromROSMsg(*lidar_msg, *laserCloudIn);

                        Eigen::Quaternionf quaternion_1;
                        quaternion_1.x() = 0.5;
                        quaternion_1.y() = -0.5;
                        quaternion_1.z() = 0.5;
                        quaternion_1.w() = 0.5;

                        Eigen::Affine3f transform = Eigen::Affine3f::Identity();
                        transform.translation() << 1.2281884865107312e-02, -5.3995959957319184e-01, -9.4994715935839347e-01;
                        transform.rotate(quaternion_1);

                        pcl::PointCloud<VelodynePointXYZIRT>::Ptr transformed_cloud(new pcl::PointCloud<VelodynePointXYZIRT>);
                        pcl::transformPointCloud(*laserCloudIn, *transformed_cloud, transform);

                        Eigen::Quaternionf quaternion_2;
                        quaternion_2.x() = 0.5;
                        quaternion_2.y() = -0.5;
                        quaternion_2.z() = 0.5;
                        quaternion_2.w() = -0.5;

                        Eigen::Affine3f transform2 = Eigen::Affine3f::Identity();
                        transform2.translation() << 1.9133004604023864e+00, 3.4678061600596360e-02, 1.4415251511205553e+00;
                        transform2.rotate(quaternion_2);

                        pcl::PointCloud<VelodynePointXYZIRT>::Ptr transformed_cloud2(new pcl::PointCloud<VelodynePointXYZIRT>);
                        pcl::transformPointCloud(*transformed_cloud, *transformed_cloud2, transform2);

                        sensor_msgs::PointCloud2 point_mmsg;
                        pcl::toROSMsg(*transformed_cloud2, point_mmsg);
                        point_mmsg.header.stamp = lidar_msg->header.stamp;
                        point_mmsg.header.frame_id = lidar_msg->header.frame_id;
                        *lidar_msg = point_mmsg;
                        // zhaoting
                    }
                    pushToLidarSendQueue(lidar_msg);
                }

                if (curTopicName == gpsTopic)
                {
                    gps_msg = m.instantiate<sensor_msgs::NavSatFix>();
                    pushToGpsSendQueue(gps_msg);
                }

                if (count_lidar != 0 && count_lidar % 10 == 0)
                {
                    cv.notify_one();
                    cv.wait(lock, [this]
                            { return dataEmpty(); });
                }
            }
        }
        bag.close();
        cacheDataFinished = true;
        cv.notify_one();
        cout << "cache data procedure is finished" << endl;
    }

    void pushToImuSendQueue(sensor_msgs::Imu::Ptr imu_msg)
    {
        if (isSending)
        {
            imuSendQueue2.push_back(imu_msg);
        }
        else
        {
            imuSendQueue1.push_back(imu_msg);
        }
        count_imu++;
    }

    void pushToLidarSendQueue(sensor_msgs::PointCloud2::Ptr lidar_msg)
    {
        if (isSending)
        {
            lidarSendQueue2.push_back(lidar_msg);
        }
        else
        {
            lidarSendQueue1.push_back(lidar_msg);
        }
        count_lidar++;
    }

    void pushToGpsSendQueue(sensor_msgs::NavSatFix::Ptr gps_msg)
    {
        if (isSending)
        {
            gpsSendQueue2.push_back(gps_msg);
        }
        else
        {
            gpsSendQueue1.push_back(gps_msg);
        }
        count_gps++;
    }

    bool dataEmpty()
    {
        if (count_lidar == 0)
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    bool dataFull()
    {
        if (count_lidar == 10 || cacheDataFinished)
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    void sendMsgThread()
    {
        cout << "send message thread start" << endl;

        while (cacheDataFinished == false || !imuSendQueue1.empty() || !lidarSendQueue1.empty() || !gpsSendQueue1.empty() ||
               !imuSendQueue2.empty() || !lidarSendQueue2.empty() || !gpsSendQueue2.empty())
        {
            if (!cacheDataFinished == false && lidarSendQueue1.empty() && lidarSendQueue2.empty())
                break;

            {
                std::unique_lock<std::mutex> lock(cvLock);
                cv.wait(lock, [this]
                        { return dataFull(); });
                isSending = !isSending;
                count_lidar = 0;
                cv.notify_one();
            }

            if (isSending)
            {
                sendImuAndLidar(imuSendQueue1, lidarSendQueue1, gpsSendQueue1);
            }
            else
            {
                sendImuAndLidar(imuSendQueue2, lidarSendQueue2, gpsSendQueue2);
            }
        }

        pubRawLidarPoints.publish(msg2);
        count_lidar_msg++;
        waitForMO();

        std_msgs::Bool allMsgsFinished;
        allMsgsFinished.data = true;
        pubAllMsgFinished.publish(allMsgsFinished);

        cout << "send message thread end, total lidar msgs sended to the slam is: " << count_lidar_msg << endl;
        cout << "Request for saving map service actively!" << endl;
        time_t end_givemetime = time(NULL);
        printf("结束时间是： %s", ctime(&end_givemetime));
    }

    void sendImuAndLidar(std::deque<sensor_msgs::Imu::Ptr> &imuMsgQue,
                         std::deque<sensor_msgs::PointCloud2::Ptr> &lidarMsgQue,
                         std::deque<sensor_msgs::NavSatFix::Ptr> &gpsMsgQue)
    {
        // for (const auto &imuptr : imuMsgQue)
        // {
        //     std::cout << "imu data ::: " << imuptr->header.stamp << std::endl;
        // }
        // std::cout << "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ " << std::endl;
        // for (const auto &gpsptr : gpsMsgQue)
        // {
        //     std::cout << "gps data ::: " << gpsptr->header.stamp << std::endl;
        // }
        // std::cout << "--------------------------------------- " << std::endl;
        // for (const auto &pcdptr : lidarMsgQue)
        // {
        //     std::cout << "Lidar ::: " << pcdptr->header.stamp << std::endl;
        // }
        // // std::cout << "-------------============--------------- " << std::endl;

        while (!imuMsgQue.empty() && imuMsgQue.front()->header.stamp.toSec() > lidarMsgQue.front()->header.stamp.toSec() + 1 / lidarFrequency)
        {
            lidarMsgQue.pop_front();
        }

        if (!firstMsgSended)
        {
            msg2 = *(lidarMsgQue.front());
            lidarMsgQue.pop_front();
            firstMsgSended = true;
        }

        while (lidarMsgQue.size() >= 1)
        {
            msg1 = msg2;
            msg2 = *(lidarMsgQue.front());
            lidarMsgQue.pop_front();

            if (count_lidar_msg >= dropFirstLidarMsgNum && count_lidar_msg <= totalLidarMsgNum - dropLastLidarMsgNum)
            {
                while (!imuMsgQue.empty() && imuMsgQue.front()->header.stamp.toSec() < msg2.header.stamp.toSec() + 1 / lidarFrequency)
                {
                    imuMsg = *(imuMsgQue.front());
                    imuMsgQue.pop_front();
                    pubRawImuData.publish(imuMsg);
                    count_imu_msg++;
                    // std::cout << "send imuMsg time: " << imuMsg.header.stamp << std::endl;
                }

                while (!gpsMsgQue.empty() && gpsMsgQue.front()->header.stamp.toSec() < msg2.header.stamp.toSec())
                {
                    gpsMsg = *(gpsMsgQue.front());
                    gpsMsgQue.pop_front();
                    pubRawGPSData.publish(gpsMsg);
                    count_gps_msg++;
                }

                setMOIsFinished(false);
                pubRawLidarPoints.publish(msg1);
                count_lidar_msg++;
                // std::cout << "send pcd time: " << msg1.header.stamp << std::endl;

                waitForMO();
            }

            // cout << "lidar msg sended: " << count_lidar_msg << endl;
            if (count_lidar_msg % 50 == 0 && count_lidar_msg != 0)
            {
                cout << "lidar msg sended: " << count_lidar_msg << "/total: " << totalLidarMsgNum - dropFirstLidarMsgNum - dropLastLidarMsgNum << endl;
            }
        }

        imuMsgQue.clear();
        gpsMsgQue.clear();
    }

    void waitForMO()
    {
        while (getMOIsFinished() == false)
        {
            sleep(0.2);
        }
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lio_sam");
    ROS_INFO("\033[1;32m----> Read Data Started.\033[0m");

    sleep(2);

    ReadData RD;

    std::thread readThread(&ReadData::cacheDataTread, &RD);
    std::thread sendThread(&ReadData::sendMsgThread, &RD);

    ros::spin();

    return 0;
}