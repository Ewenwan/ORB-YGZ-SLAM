/**
* This file is part of ORB-SLAM2.
* ORB主要借鉴了PTAM的思想，借鉴的工作主要有
* Rubble的ORB特征点；
* DBow2的place recognition用于闭环检测；
* Strasdat的闭环矫正和covisibility graph思想；
* 以及Kuemmerle和Grisetti的g2o用于优化。
* 
* 
* 系统入口:
* 1】输入图像    得到 相机位置
*       单目 GrabImageMonocular(im);
*       双目 GrabImageStereo(imRectLeft, imRectRight);
*       深度 GrabImageMonocular(imRectLeft, imRectRight);
*  单目-imu  GrabImageMonoVI(im, vimu, timestamp);  
* 2】转换为灰度图
*       单目 mImGray
*       双目 mImGray, imGrayRight
*       深度 mImGray, imDepth
* 
* 3】构造 帧Frame
*       单目 未初始化  Frame(mImGray, mpIniORBextractor)
*       单目 已初始化  Frame(mImGray, mpORBextractorLeft)
*       双目      Frame(mImGray, imGrayRight, mpORBextractorLeft, mpORBextractorRight)
*       深度      Frame(mImGray, imDepth,        mpORBextractorLeft)
* 
* 4】跟踪 Track
*   数据流进入 Tracking线程   Tracking.cc
* 
* 
* 
* ORB-SLAM利用三个线程分别进行追踪、地图构建和闭环检测。

一、追踪

    ORB特征提取
    初始姿态估计（速度估计）
    姿态优化（Track local map，利用邻近的地图点寻找更多的特征匹配，优化姿态）
    选取关键帧

二、地图构建

    加入关键帧（更新各种图）
    验证最近加入的地图点（去除Outlier）
    生成新的地图点（三角法）
    局部Bundle adjustment（该关键帧和邻近关键帧，去除Outlier）
    验证关键帧（去除重复帧）

三、闭环检测

    选取相似帧（bag of words）
    检测闭环（计算相似变换（3D<->3D，存在尺度漂移，因此是相似变换），RANSAC计算内点数）
    融合三维点，更新各种图
    图优化（传导变换矩阵），更新地图所有点


*/


#include <iomanip>
#include "System.h"
#include "Converter.h"
#include "LocalMapping.h"
#include "Tracking.h"
#include "LoopClosing.h"
#include "MapDrawer.h"
#include "Viewer.h"
#include "FrameDrawer.h"
#include "KeyFrameDatabase.h"


namespace ygz {

    System::~System() {
        if (mpParams)
            delete mpParams;
    }

    bool System::GetLocalMapAcceptKF() {
        return (mpLocalMapper->AcceptKeyFrames() && !mpLocalMapper->isStopped()) ||
               mpLocalMapper->GetUpdatingInitPoses();
        //return mpLocalMapper->ForsyncCheckNewKeyFrames();
    }

// 单目-imu系统=====================================
    cv::Mat System::TrackMonoVI(const cv::Mat &im, 
                                const std::vector<IMUData> &vimu,
                                const double &timestamp) 
{
        if (mSensor != MONOCULAR) 
         {
            cerr << "ERROR: you called TrackMonocular but input sensor was not set to Monocular." << endl;
            exit(-1);
        }


        // Check mode change  模式切换===========================
        {
            unique_lock<mutex> lock(mMutexMode);
            if (mbActivateLocalizationMode) {
                mpLocalMapper->RequestStop();

                // Wait until Local Mapping has effectively stopped
                while (!mpLocalMapper->isStopped()) {
                    usleep(1000);
                }

                mpTracker->InformOnlyTracking(true);
                mbActivateLocalizationMode = false;
            }
            if (mbDeactivateLocalizationMode) {
                mpTracker->InformOnlyTracking(false);
                mpLocalMapper->Release();
                mbDeactivateLocalizationMode = false;
            }
        }

        // Check reset 重置====
        {
            unique_lock<mutex> lock(mMutexReset);
            if (mbReset) {
                mpTracker->Reset();
                mbReset = false;
            }
        }

//  单目-IMU 跟踪=========================================================
        SE3f Tcw = mpTracker->GrabImageMonoVI(im, vimu, timestamp);

        unique_lock<mutex> lock2(mMutexState);
        mTrackingState = mpTracker->mState;
        mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
        mTrackedKeyPoints = mpTracker->mCurrentFrame.mvKeys;
        return Converter::toCvMat(Tcw);
    }


// 字符串处理===============
    bool has_suffix(
            const std::string &str, const std::string &suffix) {
        std::size_t index = str.find(suffix, str.size() - suffix.size());
        return (index != std::string::npos);
    }

// 默认初始化函数  单词表文件 txt/bin文件    配置文件     传感器：单目、双目、深度
    System::System(const string &strVocFile, 
                                     const string &strSettingsFile, 
                                     const eSensor sensor,
                                     const bool bUseViewer, 
                                     ConfigParam *pParams)
            : mSensor(sensor), 
              mpViewer(static_cast<Viewer *>(NULL)), mbReset(false), 
              mbActivateLocalizationMode(false),
              mbDeactivateLocalizationMode(false) 
{
          // 输出欢迎 信息  Output welcome message
        LOG(INFO) <<
                  "This is YGZ-SLAM, forked from ORB-SLAM proposed by Raul Mur-Artal, University of Zaragoza." << endl
                  <<
                  "This program comes with ABSOLUTELY NO WARRANTY;" << endl <<
                  "This is free software, and you are welcome to redistribute it" << endl <<
                  "under certain conditions. See ORB-SLAM's LICENSE.txt." << endl << endl;

// 输入相机=======================================
        LOG(INFO) << "Input sensor was set to: ";
        if (mSensor == MONOCULAR)
            LOG(INFO) << "Monocular" << endl;
        else if (mSensor == STEREO)
            LOG(INFO) << "Stereo" << endl;
        else if (mSensor == RGBD)
            LOG(INFO) << "RGB-D" << endl;

// 参数配置文件====================================
        //Check settings file
        cv::FileStorage fsSettings(strSettingsFile.c_str(), cv::FileStorage::READ);
        if (!fsSettings.isOpened()) {
            cerr << "Failed to open settings file at: " << strSettingsFile << endl;
            exit(-1);
        }

// 载入 特征字典文件==============================================
        //Load ORB Vocabulary
        LOG(INFO) << endl << "Loading ORB Vocabulary. This should be super fast ..." << endl;
        mpVocabulary = new ORBVocabulary();
        bool bVocLoad = false; // chose loading method based on file extension
        if (has_suffix(strVocFile, ".txt"))
            bVocLoad = mpVocabulary->loadFromTextFile(strVocFile);
        else if (has_suffix(strVocFile, ".bin"))
            bVocLoad = mpVocabulary->loadFromBinaryFile(strVocFile);
        else
            bVocLoad = false;
        if (!bVocLoad) {
            LOG(ERROR) << "Wrong path to vocabulary. " << endl;
            LOG(ERROR) << "Falied to open at: " << strVocFile << endl;
            exit(-1);
        }
        LOG(INFO) << "Vocabulary loaded!" << endl << endl;

// 参数配置器===============
        mpParams = pParams;
        if (!mpParams)
            mpParams = new ConfigParam(strSettingsFile);

// 关键帧数据库=============
        //Create KeyFrame Database
        mpKeyFrameDatabase = new KeyFrameDatabase(*mpVocabulary);

        //Create the Map   地图=================
        mpMap = new Map();

        //Create Drawers. These are used by the Viewer
        mpFrameDrawer = new FrameDrawer(mpMap);// 帧显示===============
        mpMapDrawer = new MapDrawer(mpMap, strSettingsFile);// 地图绘制===

        //Initialize the Tracking thread  初始化 跟踪线程=======================
        //(it will live in the main thread of execution, the one that called this constructor)
        mpTracker = new Tracking(this, mpVocabulary, mpFrameDrawer, mpMapDrawer,
                                 mpMap, mpKeyFrameDatabase, strSettingsFile, mSensor, mpParams);

        //Initialize the Local Mapping thread and launch 初始化 局部建图线程=======
        mpLocalMapper = new LocalMapping(mpMap, mSensor == MONOCULAR, mpParams);
        mptLocalMapping = new thread(&LocalMapping::Run, mpLocalMapper);

        //Initialize the Loop Closing thread and launch  初始化回环检测线程========
        mpLoopCloser = new LoopClosing(mpMap, mpKeyFrameDatabase, mpVocabulary, mSensor != MONOCULAR, mpParams);
        mptLoopClosing = new thread(&LoopClosing::Run, mpLoopCloser);

        //Initialize the Viewer thread and launch
        if (bUseViewer) {  // 跟踪线程 可视化=====
            mpViewer = new Viewer(this, mpFrameDrawer, mpMapDrawer, mpTracker, strSettingsFile);
            mptViewer = new thread(&Viewer::Run, mpViewer);
            mpTracker->SetViewer(mpViewer);
        }
// 9. 线程之间传递指针 Set pointers between threads=========
        mpTracker->SetLocalMapper(mpLocalMapper);
        mpTracker->SetLoopClosing(mpLoopCloser);
        mpLocalMapper->SetTracker(mpTracker);
        mpLocalMapper->SetLoopCloser(mpLoopCloser);
        mpLoopCloser->SetTracker(mpTracker);
        mpLoopCloser->SetLocalMapper(mpLocalMapper);

    }



// 双目跟踪 常 Mat量  左图  右图 时间戳   返回相机位姿=========================================================== 
    cv::Mat System::TrackStereo(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timestamp) 
   {
        if (mSensor != STEREO) {
            cerr << "ERROR: you called TrackStereo but input sensor was not set to STEREO." << endl;
            exit(-1);
        }

        // Check mode change   模式
        {
            unique_lock<mutex> lock(mMutexMode);
            if (mbActivateLocalizationMode) {
                mpLocalMapper->RequestStop();

                // Wait until Local Mapping has effectively stopped
                while (!mpLocalMapper->isStopped()) {
                    usleep(1000);
                }

                mpTracker->InformOnlyTracking(true);
                mbActivateLocalizationMode = false;
            }
            if (mbDeactivateLocalizationMode) {
                mpTracker->InformOnlyTracking(false);
                mpLocalMapper->Release();
                mbDeactivateLocalizationMode = false;
            }
        }

        // Check reset 重置
        {
            unique_lock<mutex> lock(mMutexReset);
            if (mbReset) {
                mpTracker->Reset();
                mbReset = false;
            }
        }

        // 双目跟踪=========
        SE3f Tcw = mpTracker->GrabImageStereo(imLeft, imRight, timestamp);

        unique_lock<mutex> lock2(mMutexState);
        mTrackingState = mpTracker->mState;
        mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
        mTrackedKeyPoints = mpTracker->mCurrentFrame.mvKeys;
        return Converter::toCvMat(Tcw.matrix());
    }

// 深度相机 跟踪===================================================================
    cv::Mat System::TrackRGBD(const cv::Mat &im, const cv::Mat &depthmap, const double &timestamp) {
        if (mSensor != RGBD) {
            cerr << "ERROR: you called TrackRGBD but input sensor was not set to RGBD." << endl;
            exit(-1);
        }

        // Check mode change 模式
        {
            unique_lock<mutex> lock(mMutexMode);
            if (mbActivateLocalizationMode) {
                mpLocalMapper->RequestStop();

                // Wait until Local Mapping has effectively stopped
                while (!mpLocalMapper->isStopped()) {
                    usleep(1000);
                }

                mpTracker->InformOnlyTracking(true);
                mbActivateLocalizationMode = false;
            }
            if (mbDeactivateLocalizationMode) {
                mpTracker->InformOnlyTracking(false);
                mpLocalMapper->Release();
                mbDeactivateLocalizationMode = false;
            }
        }

        // Check reset  重置
        {
            unique_lock<mutex> lock(mMutexReset);
            if (mbReset) {
                mpTracker->Reset();
                mbReset = false;
            }
        }
	      // 初始化-----------------------------
	      // 当前帧 特征点个数 大于500 进行初始化
	      // 设置第一帧为关键帧  位姿为 [I 0] 
	      // 根据第一帧视差求得的深度 计算3D点
	      // 生成地图 添加地图点 地图点观测帧 地图点最好的描述子 更新地图点的方向和距离 
	      //                 关键帧的地图点 当前帧添加地图点  地图添加地图点
	      // 显示地图
	   //  ---- 计算参考帧到当前帧 的变换 Tcr = mTcw  * mTwr---------------
	   
	      // 后面的帧-------------------------------------------------------------------------------------------------
	      // 有运动 则跟踪上一帧 跟踪失败进行 跟踪参考关键帧
	      // 没运动 或者 最近才进行过 重定位 则 跟踪 最近的一个关键帧 参考关键帧
	      // 参考关键帧 跟踪失败 则进行 重定位  跟踪所有关键帧
	      // ----- 跟踪局部地图
	        //  ---- 计算参考帧到当前帧 的变换 Tcr = mTcw  * mTwr---------------
  // 3. RGBD相机跟踪======================================================
        SE3f Tcw = mpTracker->GrabImageRGBD(im, depthmap, timestamp);

        unique_lock<mutex> lock2(mMutexState);
        mTrackingState = mpTracker->mState;
        mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
        mTrackedKeyPoints = mpTracker->mCurrentFrame.mvKeys;
        return Converter::toCvMat(Tcw.matrix());
    }

// 单目跟踪=========================================================
    cv::Mat System::TrackMonocular(const cv::Mat &im, const double &timestamp) 
{
        if (mSensor != MONOCULAR) {
            cerr << "ERROR: you called TrackMonocular but input sensor was not set to Monocular." << endl;
            exit(-1);
        }

        // Check mode change
        {
            unique_lock<mutex> lock(mMutexMode);
            if (mbActivateLocalizationMode) {
                mpLocalMapper->RequestStop();

                // Wait until Local Mapping has effectively stopped
                while (!mpLocalMapper->isStopped()) {
                    usleep(1000);
                }

                mpTracker->InformOnlyTracking(true);
                mbActivateLocalizationMode = false;
            }
            if (mbDeactivateLocalizationMode) {
                mpTracker->InformOnlyTracking(false);
                mpLocalMapper->Release();
                mbDeactivateLocalizationMode = false;
            }
        }

        // Check reset
        {
            unique_lock<mutex> lock(mMutexReset);
            if (mbReset) {
                mpTracker->Reset();
                mbReset = false;
            }
        }
    // 3. 单目跟踪==============================================================
	      // 初始化-------
	      // 连续两帧特征点个数大于100个 且两帧 关键点匹配点对数 大于100个  
	      // 初始帧 [I  0] 第二帧 基础矩阵/单应恢复 [R t] 全局优化  同时得到对应的 3D点
	      // 创建地图 使用 最小化重投影误差BA 进行 地图优化 优化位姿 和地图点
	      // 深度距离中值 倒数 归一化第二帧位姿的 平移向量 和 地图点的 三轴坐标
	      // 显示更新	 
	      // 后面的帧----------
        SE3f Tcw = mpTracker->GrabImageMonocular(im, timestamp);

        unique_lock<mutex> lock2(mMutexState);
        mTrackingState = mpTracker->mState;
        mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
        mTrackedKeyPoints = mpTracker->mCurrentFrame.mvKeys;
        return Converter::toCvMat(Tcw.matrix());
    }

    void System::ActivateLocalizationMode() {
        unique_lock<mutex> lock(mMutexMode);
        mbActivateLocalizationMode = true;
    }

    void System::DeactivateLocalizationMode() {
        unique_lock<mutex> lock(mMutexMode);
        mbDeactivateLocalizationMode = true;
    }

    bool System::MapChanged() {
        static int n = 0;
        int curn = mpMap->GetLastBigChangeIdx();
        if (n < curn) {
            n = curn;
            return true;
        } else
            return false;
    }

    void System::Reset() {
        unique_lock<mutex> lock(mMutexReset);
        mbReset = true;
    }

    void System::Shutdown() {
        mpLocalMapper->RequestFinish();
        mpLoopCloser->RequestFinish();
        if (mpViewer) {
            mpViewer->RequestFinish();
            while (!mpViewer->isFinished())
                usleep(5000);
        }

        // Wait until all thread have effectively stopped
        while (!mpLocalMapper->isFinished() || !mpLoopCloser->isFinished() || mpLoopCloser->isRunningGBA()) {
            usleep(5000);
        }

        if (mpViewer)
            pangolin::BindToContext("ORB-SLAM2: Map Viewer");
    }

    void System::SaveTrajectoryTUM(const string &filename) {
        cout << endl << "Saving camera trajectory to " << filename << " ..." << endl;
        if (mSensor == MONOCULAR) {
            cerr << "ERROR: SaveTrajectoryTUM cannot be used for monocular." << endl;
            return;
        }

        vector<KeyFrame *> vpKFs = mpMap->GetAllKeyFrames();
        sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId);

        // Transform all keyframes so that the first keyframe is at the origin.
        // After a loop closure the first keyframe might not be at the origin.
        SE3f Two = vpKFs[0]->GetPoseInverse();

        ofstream f;
        f.open(filename.c_str());
        f << fixed;

        // Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
        // We need to get first the keyframe pose and then concatenate the relative transformation.
        // Frames not localized (tracking failure) are not saved.

        // For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
        // which is true when tracking failed (lbL).
        list<KeyFrame *>::iterator lRit = mpTracker->mlpReferences.begin();
        list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
        list<bool>::iterator lbL = mpTracker->mlbLost.begin();
        for (auto lit = mpTracker->mlRelativeFramePoses.begin(),
                     lend = mpTracker->mlRelativeFramePoses.end(); lit != lend; lit++, lRit++, lT++, lbL++) {
            if (*lbL)
                continue;

            KeyFrame *pKF = *lRit;

            SE3f Trw;

            // If the reference keyframe was culled, traverse the spanning tree to get a suitable keyframe.
            while (pKF->isBad()) {
                Trw = Trw * pKF->mTcp;
                pKF = pKF->GetParent();
            }

            Trw = Trw * pKF->GetPose() * Two;

            cv::Mat Tcw = Converter::toCvMat(((*lit) * Trw).matrix());
            cv::Mat Rwc = Tcw.rowRange(0, 3).colRange(0, 3).t();
            cv::Mat twc = -Rwc * Tcw.rowRange(0, 3).col(3);

            vector<float> q = Converter::toQuaternion(Rwc);

            f << setprecision(6) << *lT << " " << setprecision(9) << twc.at<float>(0) << " " << twc.at<float>(1) << " "
              << twc.at<float>(2) << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;
        }
        f.close();
        cout << endl << "trajectory saved!" << endl;
    }


    void System::SaveKeyFrameTrajectoryTUM(const string &filename) {
        cout << endl << "Saving keyframe trajectory to " << filename << " ..." << endl;

        vector<KeyFrame *> vpKFs = mpMap->GetAllKeyFrames();
        sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId);

        // Transform all keyframes so that the first keyframe is at the origin.
        // After a loop closure the first keyframe might not be at the origin.
        //cv::Mat Two = vpKFs[0]->GetPoseInverse();

        ofstream f;
        f.open(filename.c_str());
        f << fixed;

        for (size_t i = 0; i < vpKFs.size(); i++) {
            KeyFrame *pKF = vpKFs[i];

            // pKF->SetPose(pKF->GetPose()*Two);

            if (pKF->isBad())
                continue;

            cv::Mat R = Converter::toCvMat(pKF->GetRotation()).t();
            vector<float> q = Converter::toQuaternion(R);
            Vector3f t = pKF->GetCameraCenter();
            f << setprecision(6) << pKF->mTimeStamp << setprecision(7) << " " << t[0] << " " << t[1] << " " << t[2]
              << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;

        }

        f.close();
        cout << endl << "trajectory saved!" << endl;
    }

    void System::SaveTrajectoryKITTI(const string &filename) {
        cout << endl << "Saving camera trajectory to " << filename << " ..." << endl;
        if (mSensor == MONOCULAR) {
            cerr << "ERROR: SaveTrajectoryKITTI cannot be used for monocular." << endl;
            return;
        }

        vector<KeyFrame *> vpKFs = mpMap->GetAllKeyFrames();
        sort(vpKFs.begin(), vpKFs.end(), KeyFrame::lId);

        // Transform all keyframes so that the first keyframe is at the origin.
        // After a loop closure the first keyframe might not be at the origin.
        SE3f Two = vpKFs[0]->GetPoseInverse();

        ofstream f;
        f.open(filename.c_str());
        f << fixed;

        // Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
        // We need to get first the keyframe pose and then concatenate the relative transformation.
        // Frames not localized (tracking failure) are not saved.

        // For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
        // which is true when tracking failed (lbL).
        list<KeyFrame *>::iterator lRit = mpTracker->mlpReferences.begin();
        list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
        for (auto lit = mpTracker->mlRelativeFramePoses.begin(), lend = mpTracker->mlRelativeFramePoses.end();
             lit != lend; lit++, lRit++, lT++) {
            KeyFrame *pKF = *lRit;

            SE3f Trw;

            while (pKF->isBad()) {
                //  cout << "bad parent" << endl;
                Trw = Trw * pKF->mTcp;
                pKF = pKF->GetParent();
            }

            Trw = Trw * pKF->GetPose() * Two;

            cv::Mat Tcw = Converter::toCvMat((*lit) * Trw);
            cv::Mat Rwc = Tcw.rowRange(0, 3).colRange(0, 3).t();
            cv::Mat twc = -Rwc * Tcw.rowRange(0, 3).col(3);

            f << setprecision(9) << Rwc.at<float>(0, 0) << " " << Rwc.at<float>(0, 1) << " " << Rwc.at<float>(0, 2)
              << " " << twc.at<float>(0) << " " <<
              Rwc.at<float>(1, 0) << " " << Rwc.at<float>(1, 1) << " " << Rwc.at<float>(1, 2) << " " << twc.at<float>(1)
              << " " <<
              Rwc.at<float>(2, 0) << " " << Rwc.at<float>(2, 1) << " " << Rwc.at<float>(2, 2) << " " << twc.at<float>(2)
              << endl;
        }
        f.close();
        cout << endl << "trajectory saved!" << endl;
    }

    int System::GetTrackingState() {
        unique_lock<mutex> lock(mMutexState);
        return mTrackingState;
    }

    vector<MapPoint *> System::GetTrackedMapPoints() {
        unique_lock<mutex> lock(mMutexState);
        return mTrackedMapPoints;
    }

    vector<cv::KeyPoint> System::GetTrackedKeyPointsUn() {
        unique_lock<mutex> lock(mMutexState);
        return mTrackedKeyPoints;
    }

} //namespace ORB_SLAM
