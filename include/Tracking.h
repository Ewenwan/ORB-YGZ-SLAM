/**
* This file is part of ORB-SLAM2.
* 前端跟踪线程实现================
* 跟踪上一帧 / 参考关键帧 / 重定位 / 局部地图
  特征点法跟踪、特征点直接法跟踪、单目-IMU跟踪=====
*/


#ifndef YGZ_TRACKING_H_
#define YGZ_TRACKING_H_

#include "Frame.h"
#include "ORBVocabulary.h"
#include "KeyFrameDatabase.h"
#include "ORBextractor.h"
#include "Initializer.h"
#include "MapDrawer.h"
#include "SparseImageAlign.h"


#include "IMU/imudata.h"
#include "IMU/IMUPreintegrator.h"
#include "IMU/configparam.h"


// Tracking 线程
// 改的最多的地方===========================
namespace ygz {

    class Viewer;                   // 可视化类
    class FrameDrawer;     // 显示帧类 
    class Map;                       // 地图类
    class LocalMapping;    // 局部建图类
    class LoopClosing;       // 闭环检测类
    class System;                 // 系统类头文件

    class Tracking {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW  // EIGEN矩阵======

// 类构造函数===================
        Tracking(System *pSys, 
                          ORBVocabulary *pVoc, 
                          FrameDrawer *pFrameDrawer, 
                          MapDrawer *pMapDrawer, 
                          Map *pMap,
                          KeyFrameDatabase *pKFDB, 
                          const string &strSettingPath,
                          const int sensor, 
                          ConfigParam *pParams);

        ~Tracking();// 析构函数=========

 // 双目/RGBD/单目相机 跟踪函数==================================================

   // Preprocess the input and call Track(). Extract features and performs stereo matching.
        SE3f GrabImageStereo(const cv::Mat &imRectLeft,
                                                      const cv::Mat &imRectRight, 
                                                      const double &timestamp);
        SE3f GrabImageRGBD(const cv::Mat &imRGB, 
                                                    const cv::Mat &imD, 
                                                    const double &timestamp);
        SE3f GrabImageMonocular(const cv::Mat &im, 
                                                               const double &timestamp);
// 设置 其他类指针============共内部使用===========================
        void SetLocalMapper(LocalMapping *pLocalMapper);// 设置 局部建图类 对象指针
        void SetLoopClosing(LoopClosing *pLoopClosing);      // 设置 闭环检测类 对象指针
        void SetViewer(Viewer *pViewer);                                        // 设置 可视化类   对象指针

        // Load new settings
        // The focal lenght should be similar or scale prediction will fail when projecting points
        // TODO: Modify MapPoint::PredictScale to take into account focal lenght
        void ChangeCalibration(const string &strSettingPath);

// 仅仅定位模式，不建图============================================
        // Use this function if you have deactivated local mapping and you only want to localize the camera.
        void InformOnlyTracking(const bool &flag);

    public:

        // Tracking states  跟踪任务状态 枚举变量=============
        enum eTrackingState {
            SYSTEM_NOT_READY = -1,  // 系统未准备好
            NO_IMAGES_YET = 0,           // 没有图像传入
            NOT_INITIALIZED = 1,        // 未初始化
            OK = 2,                                      // 跟踪状态正常
            LOST = 3                                  // 跟踪丢失，触发重定位
        };
        eTrackingState mState;                             // 当前跟踪状态
        eTrackingState mLastProcessedState;// 上次跟踪状态

        // Input sensor 传感器类型===========================
        int mSensor;

        // Current Frame
        Frame mCurrentFrame;// 当前帧 
        cv::Mat mImGray;           // 灰度图

// 单目相机初始化相关 变量========================
        // Initialization Variables (Monocular)
        // 初始化时前两帧相关变量
        std::vector<int> mvIniLastMatches;
        std::vector<int> mvIniMatches;// 跟踪初始化时前两帧之间的匹配
        std::vector<cv::Point2f> mvbPrevMatched;
        std::vector<Vector3f> mvIniP3D;
        Frame mInitialFrame;

// 存储 所有 相机位姿 的链表变量=============
        // Lists used to recover the full camera trajectory at the end of the execution.
        // Basically we store the reference keyframe for each frame and its relative transformation
        list <SE3f> mlRelativeFramePoses;// 关键帧位姿
        list<KeyFrame *> mlpReferences;  // 关键帧 引用
        list<double> mlFrameTimes;         // 关键帧 时间戳
        list<bool> mlbLost;                           // 关键帧 状态 丢是否

        // True if local mapping is deactivated and we are performing only localization
        bool mbOnlyTracking;// 仅定位模式   定位+建图===============

        void Reset();// 重置=====

// IMU 相关========================================================================
        ConfigParam *mpParams = nullptr;      // VIO params 参数

        // Whether use IMU
        bool mbUseIMU = false;
        bool mbVisionWeak = false;

        // Predict the NavState of Current Frame by IMU 预测导航状态===
        void PredictNavStateByIMU(bool trackLastKF);

        IMUPreintegrator mIMUPreIntInTrack;// IMU预积分

        bool TrackLocalMapWithIMU(bool bTrackLastKF = false);// 局部地图跟踪===

        bool TrackLocalMapDirectWithIMU(bool bTrackLastKF = false);

// 单目-IMU数据====================================
        SE3f GrabImageMonoVI(const cv::Mat &im, 
                                                         const std::vector<IMUData> &vimu, 
                                                         const double &timestamp);

        // IMU Data since last KF. Append when new data is provided
        // Should be cleared in 1. initialization beginning, 2. new keyframe created.
        std::vector<IMUData> mvIMUSinceLastKF;

        IMUPreintegrator
        GetIMUPreIntSinceLastKF(Frame *pCurF, KeyFrame *pLastKF, const std::vector<IMUData> &vIMUSInceLastKF);

        IMUPreintegrator GetIMUPreIntSinceLastFrame(Frame *pCurF, Frame *pLastF);


    protected:

        // Main tracking function. It is independent of the input sensor.
// 跟踪接口，根据相机类型，跳转到不同相机的 跟踪函数=====
        void Track();

        // Map initialization for stereo and RGB-D
        void StereoInitialization(); // 双目/深度 相机 地图初始化====
        // Map initialization for monocular
        void MonocularInitialization();// 单目地图初始化 初始化获得 初始位姿
        void CreateInitialMapMonocular();// 单目 最小化重投影 优化位姿

        void CheckReplacedInLastFrame();// 上一帧地图点 是否有替换点 有替换点的则进行更新
// Local Mapping 线程可能会将关键帧中某些 MapPoints 进行替换，
// 由于 tracking 中需要用到 mLastFrame，这里检查并更新上一帧中被替换的 MapPoints

        void UpdateLastFrame();// 丰富上一帧的地图点（临时）=======
          // 这些MapPoints在TrackWithMotionModel的UpdateLastFrame函数里生成（仅双目和rgbd）
          // 生成点时 会记录 在 临时点集中 mlpTemporalPoints  链表结构 
          // 这里生成的仅仅是为了提高双目或rgbd摄像头的帧间跟踪效果，用完以后就扔了，没有添加到地图中

        // 根据速度设置当前帧位姿，然后再从mLastFrame中寻找匹配点的投影
        // 投影用 ORBmatcher::SearchByProjection 实现
        // 注意会设置当前帧与地图点的关联
        bool TrackWithMotionModel();// 跟踪上一帧 运动跟踪模式   当前帧和上一帧 做匹配=====

        bool TrackReferenceKeyFrame();// 跟踪关键帧模式  当前帧和 最近一个关键帧 做匹配====

        // 用SVO中的sparse alignment 来更新当前帧位姿 稀疏关键点直接法配准=======
        // 但是这里不会处理特征点的关联和投影关系
        bool TrackWithSparseAlignment(bool bMapUpdated);// 直接法跟踪====new=====

        bool Relocalization();// 重定位  当前帧 和 所有关键帧 做匹配=======================

        // 特征点法的 Local Map 追踪=================================================
        // 与局部地图的特征匹配
        void UpdateLocalMap(); // 跟踪局部地图前 更新 局部地图：更新局部地图点、局部关键帧
        void UpdateLocalPoints();  // 更新 局部地图 点
        void UpdateLocalKeyFrames(); // 更新局部关键帧

        void SearchLocalPoints();// 跟踪局部地图前 搜索局部地图点 =======
                                          // 局部地图点 搜寻和当前帧 关键点描述子 的匹配 
                                          // 有匹配的加入到 当前帧 特征点对应的地图点中
        bool TrackLocalMap();// 跟踪局部地图========================

        // 直接法的 Local Map 追踪 ===============================new=================
        // 与局部地图的直接匹配
        bool TrackLocalMapDirect();   // 直接法跟踪局部地图
        void SearchLocalPointsDirect();// 直接法搜索地图点

        // 从地图观测中选取近的几个观测===============new=====
        /**
         * @param[in] observations 地图点的观测数据
         * @param[in] n 选取的数量
         */
        vector<pair<KeyFrame *, size_t> > SelectNearestKeyframe(
                                  const map<KeyFrame *, size_t> &observations,
                                  int n = 5);

        bool NeedNewKeyFrame(); // 跟踪局部地图之后 判断是否需要新建关键帧====
        void CreateNewKeyFrame();// 创建关键帧==============================

        // In case of performing only localization, this flag is true when there are no matches to
        // points in the map. Still tracking will continue if there are enough matches with temporal points.
        // In that case we are doing visual odometry. The system will try to do relocalization to recover
        // "zero-drift" localization to the map.
        bool mbVO = false;
	// mbVO 是 mbOnlyTracking 为true时的才有的一个变量
	// mbVO 为0表示此帧匹配了很多的MapPoints，跟踪很正常，
	// mbVO 为1表明此帧匹配了很少的MapPoints，少于10个，要跪的节奏 

        //Other Thread Pointers
        LocalMapping *mpLocalMapper = nullptr;// 局部建图 指针
        LoopClosing *mpLoopClosing = nullptr;    // 回环检测 指针

        //ORB
        // orb特征提取器，不管单目还是双目，mpORBextractorLeft都要用到
        // 如果是双目，则外加要用到mpORBextractorRight
        // 如果是单目，在初始化的时候使用 mpIniORBextractor 而不是 mpORBextractorLeft，
        // mpIniORBextractor 属性中提取的特征点个数是 mpORBextractorLeft的两倍
        ORBextractor *mpORBextractorLeft = nullptr, *mpORBextractorRight = nullptr;
        ORBextractor *mpIniORBextractor = nullptr;

        //BoW   ORB特征字典 & 
        ORBVocabulary *mpORBVocabulary = nullptr;// ORB特征字典
        KeyFrameDatabase *mpKeyFrameDB = nullptr;// 关键帧数据库，回环检测

        // Initalization (only for monocular)
        Initializer *mpInitializer = nullptr;// 单目初始化器，主要是为了生成 初始的地图点====

        //Local Map 局部地图======窗口===============================
        KeyFrame *mpReferenceKF = nullptr;// 参考关键帧
        std::vector<KeyFrame *> mvpLocalKeyFrames;// 局部 关键帧
        std::vector<MapPoint *> mvpLocalMapPoints;// 局部地图点

        set<MapPoint *> mvpDirectMapPointsCache; // 缓存之前匹配到的地图点 直接法======new===
        int mnCacheHitTh = 150;   // cache 命中点的阈值

        // System 系统对象=====
        System *mpSystem = nullptr;// 系统对象=====

        //Drawers 可视化===
        Viewer *mpViewer = nullptr;
        FrameDrawer *mpFrameDrawer = nullptr;// 显示帧
        MapDrawer *mpMapDrawer = nullptr;        // 显示地图

        //Map  地图=====
        Map *mpMap = nullptr;

        //Calibration matrix  相机校正 矩阵 
        Matrix3f mK = Matrix3f::Identity();// 内参数
        cv::Mat mDistCoef;// 畸变参数
        float mbf = 0;// 基线 × 视差   z= m*b/d

        //New KeyFrame rules (according to fps)
        int mMinFrames = 0;// 新建关键帧 准则====
        int mMaxFrames = 0;

        // Threshold close/far points
        // Points seen as close by the stereo/RGBD sensor are considered reliable
        // and inserted from just one frame. Far points requiere a match in two keyframes.
        float mThDepth = 0;// 双目/rgbd 最大深度值 阈值====6m

        // For RGB-D inputs only. For some datasets (e.g. TUM) the depthmap values are scaled.
        float mDepthMapFactor = 1;// 深度图 值放大因子======1000

        //Current matches in frame
        int mnMatchesInliers = 0;// 当前帧 匹配点数量 ====

        //Last Frame, KeyFrame and Relocalisation Info
        KeyFrame *mpLastKeyFrame = nullptr; // 上一个关键帧
        Frame mLastFrame;                                         // 上一帧
        unsigned int mnLastKeyFrameId;           // 上一个关键帧 id
        unsigned int mnLastRelocFrameId;       // 上一个重定位时的帧id

        //Motion Model   相机运动速度  直接 换成 SE3量 G2O中的 原来是 Mat
        SE3f mVelocity;
        bool mbVelocitySet = false;

        //Color order (true RGB, false BGR, ignored if grayscale)
        bool mbRGB = false;// 颜色顺序===

        list<MapPoint *> mlpTemporalPoints;  // 双目和RGBD中会生成一些额外的地图点以使匹配更加稳定
// 每一帧产生的临时点集合  链表结构===================

        // sparse image alignment SVO 直接法 稀疏直接法 
        SparseImgAlign *mpAlign = nullptr; 

        bool mbDirectFailed = false;    // 直接方法是否失败了？
    };

} //namespace ygz

#endif // TRACKING_H
