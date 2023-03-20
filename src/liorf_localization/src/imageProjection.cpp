#include "utility.h"
#include "liorf_localization/cloud_info.h"
// <!-- 111 -->
struct RobosensePointXYZIRT
{
    PCL_ADD_POINT4D
    float intensity;
    uint16_t ring;
    double time;        // Maybe:  time  -->  timestamp
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(RobosensePointXYZIRT, 
      (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)
      (uint16_t, ring, ring)(double, time, time)
)

struct OthersPointTYPE
{
    PCL_ADD_POINT4D
    float intensity;
    uint16_t ring;
    // double timestamp;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;
POINT_CLOUD_REGISTER_POINT_STRUCT(OthersPointTYPE, 
      (float, x, x)(float, y, y)(float, z, z)(float, intensity, intensity)
      (uint16_t, ring, ring)
    //   (double, timestamp, timestamp)
)

// Use the robosense point format as a common representation
using PointXYZIRT = RobosensePointXYZIRT;

const int queueLength = 2000;

class ImageProjection : public ParamServer
{
private:

    std::mutex imuLock;
    std::mutex odoLock;

    ros::Subscriber subLaserCloud;
    ros::Publisher  pubLaserCloud;
    
    ros::Publisher pubExtractedCloud;
    ros::Publisher pubLaserCloudInfo;

    ros::Subscriber subImu;
    std::deque<sensor_msgs::Imu> imuQueue;

    ros::Subscriber subOdom;
    std::deque<nav_msgs::Odometry> odomQueue;

    std::deque<sensor_msgs::PointCloud2> cloudQueue;
    sensor_msgs::PointCloud2 currentCloudMsg;

    double *imuTime = new double[queueLength];
    double *imuRotX = new double[queueLength];
    double *imuRotY = new double[queueLength];
    double *imuRotZ = new double[queueLength];

    int imuPointerCur;
    bool firstPointFlag;
    Eigen::Affine3f transStartInverse;

    pcl::PointCloud<PointXYZIRT>::Ptr laserCloudIn;
    pcl::PointCloud<PointType>::Ptr   fullCloud;

    int deskewFlag;

    bool odomDeskewFlag;
    float odomIncreX;
    float odomIncreY;
    float odomIncreZ;

    liorf_localization::cloud_info cloudInfo;
    double timeScanCur;
    double timeScanEnd;
    std_msgs::Header cloudHeader;

public:
    ImageProjection() : deskewFlag(-1) {    

        subImu        = nh.subscribe<sensor_msgs::Imu>(imuTopic, 2000, &ImageProjection::imuHandler, this, ros::TransportHints().tcpNoDelay());
        subLaserCloud = nh.subscribe<sensor_msgs::PointCloud2>(pointCloudTopic, 5, &ImageProjection::cloudHandler, this, ros::TransportHints().tcpNoDelay());
        // 发布imu里程计，预积分发布
        subOdom       = nh.subscribe<nav_msgs::Odometry>(odomTopic+"_incremental", 2000, &ImageProjection::odometryHandler, this, ros::TransportHints().tcpNoDelay());

        // 提取有效激光点，未进行特征提取， 只是info带更多信息
        // pubExtractedCloud = nh.advertise<sensor_msgs::PointCloud2> ("liorf_localization/deskew/cloud_deskewed", 1);  
        pubLaserCloudInfo = nh.advertise<liorf_localization::cloud_info> ("liorf_localization/deskew/cloud_info", 1);   // 发布到mapping进行 scan2map

        allocateMemory();
        resetParameters();

        pcl::console::setVerbosityLevel(pcl::console::L_ERROR);
    }

    void allocateMemory()
    {
        laserCloudIn.reset(new pcl::PointCloud<PointXYZIRT>());
        fullCloud.reset(new pcl::PointCloud<PointType>());

        resetParameters();
    }

    void resetParameters()
    {
        laserCloudIn->clear();
        fullCloud->clear();

        imuPointerCur = 0;
        firstPointFlag = true;
        odomDeskewFlag = false;

        for (int i = 0; i < queueLength; ++i)
        {
            imuTime[i] = 0;
            imuRotX[i] = 0;
            imuRotY[i] = 0;
            imuRotZ[i] = 0;
        }

    }

    ~ImageProjection(){}

    void imuHandler(const sensor_msgs::Imu::ConstPtr& imuMsg)
    {
        // 只有旋转，imuPreintegration.cpp文件中，还有两个imu2Lidar，lidar2imu变量，这俩变量只有平移，没有旋转
        sensor_msgs::Imu thisImu = imuConverter(*imuMsg);      

        std::lock_guard<std::mutex> lock1(imuLock);
        imuQueue.push_back(thisImu);

        // debug IMU data
        // cout << std::setprecision(6);
        // cout << "IMU acc: " << endl;
        // cout << "x: " << thisImu.linear_acceleration.x << 
        //       ", y: " << thisImu.linear_acceleration.y << 
        //       ", z: " << thisImu.linear_acceleration.z << endl;
        // cout << "IMU gyro: " << endl;
        // cout << "x: " << thisImu.angular_velocity.x << 
        //       ", y: " << thisImu.angular_velocity.y << 
        //       ", z: " << thisImu.angular_velocity.z << endl;
        // double imuRoll, imuPitch, imuYaw;
        // tf::Quaternion orientation;
        // tf::quaternionMsgToTF(thisImu.orientation, orientation);
        // tf::Matrix3x3(orientation).getRPY(imuRoll, imuPitch, imuYaw);
        // cout << "IMU roll pitch yaw: " << endl;
        // cout << "roll: " << imuRoll << ", pitch: " << imuPitch << ", yaw: " << imuYaw << endl << endl;
    }

    void odometryHandler(const nav_msgs::Odometry::ConstPtr& odometryMsg)
    {
        std::lock_guard<std::mutex> lock2(odoLock);
        odomQueue.push_back(*odometryMsg);
    }

    void cloudHandler(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg)
    {
        if (!cachePointCloud(laserCloudMsg))
            return;

        // if (!deskewInfo())
        //     return;

        projectPointCloud();

        publishClouds();

        resetParameters();
    }

    bool cachePointCloud(const sensor_msgs::PointCloud2ConstPtr& laserCloudMsg)
    {
        // cache point cloud
        cloudQueue.push_back(*laserCloudMsg);
        if (cloudQueue.size() <= 2)
            return false;

        // convert cloud
        currentCloudMsg = std::move(cloudQueue.front());
        cloudQueue.pop_front();
        if (sensor == SensorType::OTHERS)     // 手动检索
        {
            pcl::PointCloud<OthersPointTYPE>::Ptr tmpCloudIn(new pcl::PointCloud<OthersPointTYPE>());
            pcl::moveFromROSMsg(currentCloudMsg, *tmpCloudIn);
            laserCloudIn->points.resize(tmpCloudIn->size());
            laserCloudIn->is_dense = tmpCloudIn->is_dense;

            if (laserCloudIn->is_dense==false) {
                std::vector<int> idx;
                pcl::removeNaNFromPointCloud(*tmpCloudIn, *tmpCloudIn, idx);
                laserCloudIn->is_dense = true;
            }

            int32_t cloudSize = tmpCloudIn->size();
            double scanPeriod = 0.1;     // 雷达频率， 10hz
            float startOri = -atan2(tmpCloudIn->points[0].y, tmpCloudIn->points[0].x);        //该帧的起始角度
            float endOri = -atan2(tmpCloudIn->points[cloudSize - 1].y, tmpCloudIn->points[cloudSize - 1].x)  +  2 * M_PI;    //终止角度                                                          
            if (endOri - startOri > 3 * M_PI) 
            {
                endOri -= 2 * M_PI;
            }
            else if (endOri - startOri < M_PI)
            {
                endOri += 2 * M_PI;
            }

            int count = cloudSize;
            bool halfPassed = false;
            for (int32_t i=0; i < cloudSize; ++i) {
                auto &src = tmpCloudIn->points[i];
                auto &dst = laserCloudIn->points[i];

                float cur_angle = atan(src.z / sqrt(src.x*src.x + src.y*src.y)) * 180 / M_PI; 
                int scanID = 0;
                if (N_SCAN == 16)
                {
                    // +-15°的垂直视场，垂直角度分辨率2°，-15°时的scanID = 0
                    /*
                    * 垂直视场角，可以算作每个点的
                    * 如果是16线激光雷达，结算出的angle应该在-15~15之间
                    */
                    scanID = int((cur_angle + 15) / 2 + 0.5);
                    if (scanID > (N_SCAN - 1) || scanID < 0)   // 无效点，该帧点云size减1
                    {
                        count--;
                        continue;
                    }
                }
                else if (N_SCAN == 32)
                {
                    scanID = int((cur_angle + 92.0/3.0) * 3.0 / 4.0);
                    if (scanID > (N_SCAN - 1) || scanID < 0)
                    {
                        count--;
                        continue;
                    }
                }
                else if (N_SCAN == 64)
                {   
                    if (cur_angle >= -8.83)
                        scanID = int((2 - cur_angle) * 3.0 + 0.5);
                    else
                        scanID = N_SCAN / 2 + int((-8.83 - cur_angle) * 2.0 + 0.5);

                    // use [0 50]  > 50 remove outlies 
                    if (cur_angle > 2 || cur_angle < -24.33 || scanID > 50 || scanID < 0)
                    {
                        count--;
                        continue;
                    }
                }
                else
                {
                    ROS_ERROR_STREAM("Unknown sensor N_SCAN: " << int(N_SCAN));
                    ros::shutdown();
                }

                float ori = -atan2(src.y, src.x);       // 该点的旋转角度
                if (!halfPassed)    // 该帧点云扫描没有过半，则halfPassed为false
                { 
                    if (ori < startOri - M_PI / 2)
                    {
                        ori += 2 * M_PI;
                    }
                    else if (ori > startOri + M_PI * 3 / 2)
                    {   
                        ori -= 2 * M_PI;
                    }

                    if (ori - startOri > M_PI)
                    {   
                        halfPassed = true;
                    }
                }
                else
                {
                    ori += 2 * M_PI;
                    if (ori < endOri - M_PI * 3 / 2)
                    {
                        ori += 2 * M_PI;
                    }
                    else if (ori > endOri + M_PI / 2)
                    {
                        ori -= 2 * M_PI; 
                    }
                }
                // 某个点在 该帧下的角度位置
                // relTime是0-1之间
                float relTime = (ori - startOri) / (endOri - startOri);

                dst.x = src.x;
                dst.y = src.y;
                dst.z = src.z;
                // dst.intensity = 0.0;
                dst.intensity = src.intensity;
                dst.ring = scanID;
                // dst.ring = src.ring;
                dst.time = scanPeriod * relTime;
            }
        }
        else if (sensor == SensorType::ROBOSENSE) {           // 自带信息 
            pcl::PointCloud<RobosensePointXYZIRT>::Ptr tmpRobosenseCloudIn(new pcl::PointCloud<RobosensePointXYZIRT>());
            // Convert to robosense format
            pcl::moveFromROSMsg(currentCloudMsg, *tmpRobosenseCloudIn);
            laserCloudIn->points.resize(tmpRobosenseCloudIn->size());
            laserCloudIn->is_dense = tmpRobosenseCloudIn->is_dense;
            if (laserCloudIn->is_dense==false) {
                std::vector<int> idx;
                pcl::removeNaNFromPointCloud(*tmpRobosenseCloudIn, *tmpRobosenseCloudIn, idx);
                laserCloudIn->is_dense = true;
            }
            double start_stamptime = tmpRobosenseCloudIn->points[0].time;
            for (size_t i = 0; i < tmpRobosenseCloudIn->size(); i++) {
                auto &src = tmpRobosenseCloudIn->points[i];
                auto &dst = laserCloudIn->points[i];

                dst.x = src.x;
                dst.y = src.y;
                dst.z = src.z;
                dst.intensity = src.intensity;
                dst.ring = src.ring;
                dst.time = src.time - start_stamptime;         //在该帧里的时间
            }
        } else {
            ROS_ERROR_STREAM("Unknown sensor type: " << int(sensor));
            ros::shutdown();
        }
        
        // get timestamp
        cloudHeader = currentCloudMsg.header;
        timeScanCur = cloudHeader.stamp.toSec();        // 得到该帧雷达的起止时间
        timeScanEnd = timeScanCur + laserCloudIn->points.back().time;
    
        return true;
    }

    bool deskewInfo()
    {
        std::lock_guard<std::mutex> lock1(imuLock);
        std::lock_guard<std::mutex> lock2(odoLock);

        // make sure IMU data available for the scan
        if (imuQueue.empty() || imuQueue.front().header.stamp.toSec() > timeScanCur || imuQueue.back().header.stamp.toSec() < timeScanEnd)
        {
            ROS_DEBUG("Waiting for IMU data ...");
            return false;
        }

        imuDeskewInfo();        // 对齐cloud时间戳，积分得到角度和角速度

        odomDeskewInfo();

        return true;
    }

    void imuDeskewInfo()
    {
        cloudInfo.imuAvailable = false;

        while (!imuQueue.empty())       // 对齐cloud
        {
            if (imuQueue.front().header.stamp.toSec() < timeScanCur - 0.01)
                imuQueue.pop_front();
            else
                break;
        }

        if (imuQueue.empty())
            return;

        imuPointerCur = 0;

        for (int i = 0; i < (int)imuQueue.size(); ++i)
        {
            sensor_msgs::Imu thisImuMsg = imuQueue[i];
            double currentImuTime = thisImuMsg.header.stamp.toSec();

            if (imuType) {
                // get roll, pitch, and yaw estimation for this scan
                if (currentImuTime <= timeScanCur)
                    imuRPY2rosRPY(&thisImuMsg, &cloudInfo.imuRollInit, &cloudInfo.imuPitchInit, &cloudInfo.imuYawInit);
            }

            if (currentImuTime > timeScanEnd + 0.01)
                break;

            if (imuPointerCur == 0){
                imuRotX[0] = 0;
                imuRotY[0] = 0;
                imuRotZ[0] = 0;
                imuTime[0] = currentImuTime;
                ++imuPointerCur;
                continue;
            }

            // get angular velocity  角速度
            double angular_x, angular_y, angular_z;
            imuAngular2rosAngular(&thisImuMsg, &angular_x, &angular_y, &angular_z);

            // integrate rotation  积分得到角度
            double timeDiff = currentImuTime - imuTime[imuPointerCur-1];
            imuRotX[imuPointerCur] = imuRotX[imuPointerCur-1] + angular_x * timeDiff;
            imuRotY[imuPointerCur] = imuRotY[imuPointerCur-1] + angular_y * timeDiff;
            imuRotZ[imuPointerCur] = imuRotZ[imuPointerCur-1] + angular_z * timeDiff;
            imuTime[imuPointerCur] = currentImuTime;
            ++imuPointerCur;
        }

        --imuPointerCur;

        if (imuPointerCur <= 0)
            return;

        cloudInfo.imuAvailable = true;
    }

    void odomDeskewInfo()       // imu里程计
    {
        cloudInfo.odomAvailable = false;
        static float sync_diff_time = (imuRate >= 300) ? 0.01 : 0.20;
        while (!odomQueue.empty())
        {
            if (odomQueue.front().header.stamp.toSec() < timeScanCur - sync_diff_time)
                odomQueue.pop_front();
            else
                break;
        }

        if (odomQueue.empty())
            return;

        if (odomQueue.front().header.stamp.toSec() > timeScanCur)
            return;

        // get start odometry at the beinning of the scan
        nav_msgs::Odometry startOdomMsg;

        for (int i = 0; i < (int)odomQueue.size(); ++i)
        {
            startOdomMsg = odomQueue[i];

            if (ROS_TIME(&startOdomMsg) < timeScanCur)
                continue;
            else
                break;
        }

        tf::Quaternion orientation;
        tf::quaternionMsgToTF(startOdomMsg.pose.pose.orientation, orientation);

        double roll, pitch, yaw;
        tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);

        // Initial guess used in mapOptimization
        cloudInfo.initialGuessX = startOdomMsg.pose.pose.position.x;
        cloudInfo.initialGuessY = startOdomMsg.pose.pose.position.y;
        cloudInfo.initialGuessZ = startOdomMsg.pose.pose.position.z;
        cloudInfo.initialGuessRoll  = roll;
        cloudInfo.initialGuessPitch = pitch;
        cloudInfo.initialGuessYaw   = yaw;

        cloudInfo.odomAvailable = true;

        // get end odometry at the end of the scan
        odomDeskewFlag = false;

        if (odomQueue.back().header.stamp.toSec() < timeScanEnd)
            return;

        nav_msgs::Odometry endOdomMsg;

        for (int i = 0; i < (int)odomQueue.size(); ++i)
        {
            endOdomMsg = odomQueue[i];

            if (ROS_TIME(&endOdomMsg) < timeScanEnd)
                continue;
            else
                break;
        }

        if (int(round(startOdomMsg.pose.covariance[0])) != int(round(endOdomMsg.pose.covariance[0])))
            return;

        Eigen::Affine3f transBegin = pcl::getTransformation(startOdomMsg.pose.pose.position.x, startOdomMsg.pose.pose.position.y, startOdomMsg.pose.pose.position.z, roll, pitch, yaw);

        tf::quaternionMsgToTF(endOdomMsg.pose.pose.orientation, orientation);
        tf::Matrix3x3(orientation).getRPY(roll, pitch, yaw);
        Eigen::Affine3f transEnd = pcl::getTransformation(endOdomMsg.pose.pose.position.x, endOdomMsg.pose.pose.position.y, endOdomMsg.pose.pose.position.z, roll, pitch, yaw);

        Eigen::Affine3f transBt = transBegin.inverse() * transEnd;

        float rollIncre, pitchIncre, yawIncre;
        pcl::getTranslationAndEulerAngles(transBt, odomIncreX, odomIncreY, odomIncreZ, rollIncre, pitchIncre, yawIncre);

        odomDeskewFlag = true;
    }

    void findRotation(double pointTime, float *rotXCur, float *rotYCur, float *rotZCur)
    {
        *rotXCur = 0; *rotYCur = 0; *rotZCur = 0;

        int imuPointerFront = 0;
        while (imuPointerFront < imuPointerCur)
        {
            if (pointTime < imuTime[imuPointerFront])
                break;
            ++imuPointerFront;
        }

        if (pointTime > imuTime[imuPointerFront] || imuPointerFront == 0)
        {
            *rotXCur = imuRotX[imuPointerFront];
            *rotYCur = imuRotY[imuPointerFront];
            *rotZCur = imuRotZ[imuPointerFront];
        } else {
            int imuPointerBack = imuPointerFront - 1;
            double ratioFront = (pointTime - imuTime[imuPointerBack]) / (imuTime[imuPointerFront] - imuTime[imuPointerBack]);
            double ratioBack = (imuTime[imuPointerFront] - pointTime) / (imuTime[imuPointerFront] - imuTime[imuPointerBack]);
            *rotXCur = imuRotX[imuPointerFront] * ratioFront + imuRotX[imuPointerBack] * ratioBack;
            *rotYCur = imuRotY[imuPointerFront] * ratioFront + imuRotY[imuPointerBack] * ratioBack;
            *rotZCur = imuRotZ[imuPointerFront] * ratioFront + imuRotZ[imuPointerBack] * ratioBack;
        }
    }

    void findPosition(double relTime, float *posXCur, float *posYCur, float *posZCur)
    {
        *posXCur = 0; *posYCur = 0; *posZCur = 0;

        // If the sensor moves relatively slow, like walking speed, positional deskew seems to have little benefits. Thus code below is commented.

        // if (cloudInfo.odomAvailable == false || odomDeskewFlag == false)
        //     return;

        // float ratio = relTime / (timeScanEnd - timeScanCur);

        // *posXCur = ratio * odomIncreX;
        // *posYCur = ratio * odomIncreY;
        // *posZCur = ratio * odomIncreZ;
    }

    PointType deskewPoint(PointType *point, double relTime)
    {
        if (deskewFlag == -1 || cloudInfo.imuAvailable == false)
            return *point;

        double pointTime = timeScanCur + relTime;

        float rotXCur, rotYCur, rotZCur;
        findRotation(pointTime, &rotXCur, &rotYCur, &rotZCur);

        float posXCur, posYCur, posZCur;
        findPosition(relTime, &posXCur, &posYCur, &posZCur);

        if (firstPointFlag == true)
        {
            transStartInverse = (pcl::getTransformation(posXCur, posYCur, posZCur, rotXCur, rotYCur, rotZCur)).inverse();
            firstPointFlag = false;
        }

        // transform points to start
        Eigen::Affine3f transFinal = pcl::getTransformation(posXCur, posYCur, posZCur, rotXCur, rotYCur, rotZCur);
        Eigen::Affine3f transBt = transStartInverse * transFinal;

        PointType newPoint;
        newPoint.x = transBt(0,0) * point->x + transBt(0,1) * point->y + transBt(0,2) * point->z + transBt(0,3);
        newPoint.y = transBt(1,0) * point->x + transBt(1,1) * point->y + transBt(1,2) * point->z + transBt(1,3);
        newPoint.z = transBt(2,0) * point->x + transBt(2,1) * point->y + transBt(2,2) * point->z + transBt(2,3);
        newPoint.intensity = point->intensity;

        return newPoint;
    }

    void projectPointCloud()
    {
        int cloudSize = laserCloudIn->points.size();
        // range image projection
        for (int i = 0; i < cloudSize; ++i)
        {
            PointType thisPoint;
            thisPoint.x = laserCloudIn->points[i].x;
            thisPoint.y = laserCloudIn->points[i].y;
            thisPoint.z = laserCloudIn->points[i].z;
            thisPoint.intensity = laserCloudIn->points[i].intensity;

            float range = pointDistance(thisPoint);
            if (range < lidarMinRange || range > lidarMaxRange)
                continue;

            int rowIdn = laserCloudIn->points[i].ring;
            if (rowIdn < 0 || rowIdn >= N_SCAN)
                continue;

            if (rowIdn % downsampleRate != 0)
                continue;

            if (i % point_filter_num != 0)
                continue;

            // thisPoint = deskewPoint(&thisPoint, laserCloudIn->points[i].time);

            fullCloud->push_back(thisPoint);
        }
    }
    
    void publishClouds()
    {
        cloudInfo.header = cloudHeader;
        // cloudInfo.cloud_deskewed  = publishCloud(pubExtractedCloud, fullCloud, cloudHeader.stamp, lidarFrame);

        sensor_msgs::PointCloud2 tempCloud;
        pcl::toROSMsg(*fullCloud, tempCloud);
        tempCloud.header.stamp = cloudHeader.stamp;
        tempCloud.header.frame_id = lidarFrame;
        cloudInfo.cloud_deskewed = tempCloud;
        pubLaserCloudInfo.publish(cloudInfo);
    }
};


//对于一些只订阅一个话题的简单节点来说，我们使用ros::spin()进入接收循环，
//每当有订阅的话题发布时，进入回调函数接收和处理消息数据。
//但是更多的时候，一个节点往往要接收和处理不同来源的数据，并且这些数据的产生频率也各不相同，
//当我们在一个回调函数里耗费太多时间时，会导致其他回调函数被阻塞，导致数据丢失。
//这种场合需要给一个节点开辟多个线程，保证数据流的畅通。
int main(int argc, char** argv)
{
    ros::init(argc, argv, "liorf_localization");

    ImageProjection IP;
    
    ROS_INFO("\033[1;32m----> Image Projection Started.\033[0m");

    ros::MultiThreadedSpinner spinner(3);
    spinner.spin();
    
    return 0;
}
