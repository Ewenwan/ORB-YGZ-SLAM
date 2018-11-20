/**
* This file is part of ORB-SLAM2.
* 工程入口函数 系统类 头文件===================
*/


#ifndef YGZ_SYSTEM_H_
#define YGZ_SYSTEM_H_

#include "Common.h"
#include "ORBVocabulary.h"
#include "IMU/imudata.h"
#include "IMU/configparam.h"

// System
// 协调各个线程一同工作

namespace ygz {

    class Viewer;

    class FrameDrawer;

    class Map;

    class MapPoint;

    class Tracking;

    class LocalMapping;

    class LoopClosing;

    class MapDrawer;

    class KeyFrameDatabase;

    class System {
    public:

        // Input sensor  枚举  输入 传感器类型
        enum eSensor {
            MONOCULAR = 0,// 单目0
            STEREO = 1,// 双目1
            RGBD = 2// 深度2
        };

    public:

        // Initialize the SLAM system. It launches the Local Mapping, Loop Closing and Viewer threads.
        System(const string &strVocFile, 
                        const string &strSettingsFile, 
                        const eSensor sensor,
                        const bool bUseViewer = true, 
                        ConfigParam *pParams = NULL);

        ~System();

        // Proccess the given stereo frame. Images must be synchronized and rectified.
        // Input images: RGB (CV_8UC3) or grayscale (CV_8U). RGB is converted to grayscale.
        // Returns the camera pose (empty if tracking fails).
// 双目跟踪  返回相机位姿 ======
        cv::Mat TrackStereo(const cv::Mat &imLeft, 
                                                const cv::Mat &imRight, 
                                                const double &timestamp);

        // Process the given rgbd frame. Depthmap must be registered to the RGB frame.
        // Input image: RGB (CV_8UC3) or grayscale (CV_8U). RGB is converted to grayscale.
        // Input depthmap: Float (CV_32F).
        // Returns the camera pose (empty if tracking fails).
// 深度 跟踪  返回相机位姿====
        cv::Mat TrackRGBD(const cv::Mat &im, const cv::Mat &depthmap, const double &timestamp);

        // Proccess the given monocular frame
        // Input images: RGB (CV_8UC3) or grayscale (CV_8U). RGB is converted to grayscale.
        // Returns the camera pose (empty if tracking fails).
// 单目 跟踪  返回相机位姿========
        cv::Mat TrackMonocular(const cv::Mat &im, const double &timestamp);

        /**
         *@brief Track Monocular with IMU   单目IMU跟踪======
         * @param[in] im input image
         * @param[in] vimu the imu measurements from last image frame
         * @param[in] timestamp time
         * @return the esitmated pose
        */
        cv::Mat TrackMonoVI(const cv::Mat &im, const std::vector<IMUData> &vimu, const double &timestamp);


        // This stops local mapping thread (map building) and performs only camera tracking.
        void ActivateLocalizationMode();// 定位 + 跟踪 模式

        // This resumes local mapping thread and performs SLAM again.
        void DeactivateLocalizationMode();//   定位 +建图 + 跟踪 模式

        // Returns true if there have been a big map change (loop closure, global BA)
        // since last call to this function
        bool MapChanged(); // 地图发生了大的变换

        // Reset the system (clear map)
        void Reset();// 重置====

        // All threads will be requested to finish.
        // It waits until all threads have finished.
        // This function must be called before saving the trajectory.
        void Shutdown();

        // Save camera trajectory in the TUM RGB-D dataset format.
        // Only for stereo and RGB-D. This method does not work for monocular.
        // Call first Shutdown()
        // See format details at: http://vision.in.tum.de/data/datasets/rgbd-dataset
        void SaveTrajectoryTUM(const string &filename);

        // Save keyframe poses in the TUM RGB-D dataset format.
        // This method works for all sensor input.
        // Call first Shutdown()
        // See format details at: http://vision.in.tum.de/data/datasets/rgbd-dataset
        void SaveKeyFrameTrajectoryTUM(const string &filename);

        // Save camera trajectory in the KITTI dataset format.
        // Only for stereo and RGB-D. This method does not work for monocular.
        // Call first Shutdown()
        // See format details at: http://www.cvlibs.net/datasets/kitti/eval_odometry.php
        void SaveTrajectoryKITTI(const string &filename);

        // save keyframe trajectory
        void SaveKeyFrameTrajectoryNavState(const string &filename);

        // TODO: Save/Load functions    地图载入保存函数=================
        // SaveMap(const string &filename);
        // LoadMap(const string &filename);

        // Information from most recent processed frame
        // You can call this right after TrackMonocular (or stereo or RGBD)
        int GetTrackingState();

        std::vector<MapPoint *> GetTrackedMapPoints();

        std::vector<cv::KeyPoint> GetTrackedKeyPointsUn();

        // check if local map can accept keyframes (not busy)
        bool GetLocalMapAcceptKF();

        // Proccess the given monocular frame with IMU measurements
        // IMU measurements should be between LAST and CURRENT frame
        // (i.e. the timestamp of IMU measrements should be smaller than the timestamp of image)
        // Input images: RGB (CV_8UC3) or grayscale (CV_8U). RGB is converted to grayscale.
        // Returns the camera pose (empty if tracking fails).

        ConfigParam *mpParams =nullptr;  // IMU related params

    private:

        // Input sensor
        eSensor mSensor;// enum 枚举变量  输入相机类型 单目 双目 深度

        // ORB vocabulary used for place recognition and feature matching.
// 词典对象指针 用于 地点识别  特征匹配 orb特征==
        ORBVocabulary *mpVocabulary =nullptr;

        // KeyFrame database for place recognition (relocalization and loop detection).
// 关键帧 数据库 对象指针  用于 地点识别 定位 回环检测====
        KeyFrameDatabase *mpKeyFrameDatabase =nullptr;

        // Map structure that stores the pointers to all KeyFrames and MapPoints.
// 地图对象指针  存储 关键帧 和 地图点====
        Map *mpMap =nullptr;

        // Tracker. It receives a frame and computes the associated camera pose.
        // It also decides when to insert a new keyframe, create some new MapPoints and
        // performs relocalization if tracking fails.
// 跟踪对象 指针 ========
        Tracking *mpTracker =nullptr;

        // Local Mapper. It manages the local map and performs local bundle adjustment.
// 建图对象 指针 =====
        LocalMapping *mpLocalMapper =nullptr;

        // Loop Closer. It searches loops with every new keyframe. If there is a loop it performs
        // a pose graph optimization and full bundle adjustment (in a new thread) afterwards.
// 回环检测对象指针 ======
        LoopClosing *mpLoopCloser =nullptr;

        // The viewer draws the map and the current camera pose. It uses Pangolin.
        Viewer *mpViewer =nullptr;// 可视化对象指针 ======

        FrameDrawer *mpFrameDrawer =nullptr;// 显示帧对象 指针 =====
        MapDrawer *mpMapDrawer =nullptr;// 显示地图对象 指针 ===

        // System threads: Local Mapping, Loop Closing, Viewer.
        // The Tracking thread "lives" in the main execution thread that creates the System object.
        std::thread *mptLocalMapping =nullptr;// 建图线程         指针
        std::thread *mptLoopClosing =nullptr;   // 闭环检测线程  指针
        std::thread *mptViewer =nullptr;              // 可视化线程      指针

        // Reset flag 线程重启标志 ====
        std::mutex mMutexReset;// 互斥量   保护 mbReset 变量
        bool mbReset =false;

        // Change mode flags   系统模式=
        std::mutex mMutexMode;// 互斥量=
        bool mbActivateLocalizationMode;// 跟踪 + 定位
        bool mbDeactivateLocalizationMode;//   跟踪 + 定位+ 建图

        // Tracking state 跟踪线程 状态===
        int mTrackingState;
        std::vector<MapPoint *> mTrackedMapPoints;// 当前帧跟踪到的地图点 3d
        std::vector<cv::KeyPoint> mTrackedKeyPoints;// 2d关键点
        std::mutex mMutexState;// 互斥量=
    };

}// namespace ygz

#endif // SYSTEM_H
