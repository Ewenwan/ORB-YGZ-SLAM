/**
* This file is part of ORB-SLAM2.
*  回环检测=Loop closing 线程
*/

#ifndef YGZ_LOOPCLOSING_H_
#define YGZ_LOOPCLOSING_H_

#include "Common.h"
#include "ORBVocabulary.h"
#include "IMU/configparam.h"

// Loop closing 线程
// 基本没动，除了加IMU部分

namespace ygz {

    class Tracking;
    class LocalMapping;
    class KeyFrameDatabase;

    class KeyFrame;//========
    class Map;//===
    class MapPoint;//==

    class LoopClosing {
    public:

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        typedef pair<set<KeyFrame *>, int> ConsistentGroup; // 具有连续性的候选帧
        typedef map<KeyFrame *, 
                                     g2o::Sim3, 
                                     std::less<KeyFrame *>,
                                     Eigen::aligned_allocator<std::pair<const KeyFrame *,
                                                                                                       g2o::Sim3> > 
                                    >  KeyFrameAndPose;

        LoopClosing(Map *pMap, KeyFrameDatabase *pDB, 
                                  ORBVocabulary *pVoc, const bool bFixScale, ConfigParam *pParams);

        void SetTracker(Tracking *pTracker); // 跟踪器====

        void SetLocalMapper(LocalMapping *pLocalMapper);// 局部建图=====

        // Main function
        void Run();//Loop closing 线程 主函数====

        void InsertKeyFrame(KeyFrame *pKF);// 插入关键帧====

        void RequestReset();// 请求重置====

        // This function will run in a separate thread  全局优化=====
        void RunGlobalBundleAdjustment(unsigned long nLoopKF);

        bool isRunningGBA() {
            unique_lock<std::mutex> lock(mMutexGBA);
            return mbRunningGBA; // 全局优化 是否在运行===
        }

        bool isFinishedGBA() {
            unique_lock<std::mutex> lock(mMutexGBA);
            return mbFinishedGBA;// 全局优化 是否 结束===
        }

        void RequestFinish();// 请求完成====

        bool isFinished();// 回环检测是否完成

// IMU 参数=======================================
        ConfigParam *mpParams;
        bool mbUseIMU;
        bool GetMapUpdateFlagForTracking();
        void SetMapUpdateFlagInTracking(bool bflag);

    protected:

        bool CheckNewKeyFrames();
        bool DetectLoop();// 检测闭环==================
        bool ComputeSim3();// 计算闭环处的相似变换=====

        void SearchAndFuse(const KeyFrameAndPose &CorrectedPosesMap);

        void CorrectLoop();

        void ResetIfRequested();

        bool mbResetRequested;
        std::mutex mMutexReset;

        bool CheckFinish();

        void SetFinish();

        bool mbFinishRequested;
        bool mbFinished;
        std::mutex mMutexFinish;

        Map *mpMap;
        Tracking *mpTracker;

        KeyFrameDatabase *mpKeyFrameDB;
        ORBVocabulary *mpORBVocabulary;

        LocalMapping *mpLocalMapper;

        std::list<KeyFrame *> mlpLoopKeyFrameQueue; // 闭环帧队列=======

        std::mutex mMutexLoopQueue;

        // Loop detector parameters
        float mnCovisibilityConsistencyTh;

        // Loop detector variables
        KeyFrame *mpCurrentKF;
        KeyFrame *mpMatchedKF;
        std::vector<ConsistentGroup> mvConsistentGroups;// 具有连续性的候选帧 群组
        std::vector<KeyFrame *> mvpEnoughConsistentCandidates;
        std::vector<KeyFrame *> mvpCurrentConnectedKFs;// 当前帧相连的 关键帧
        std::vector<MapPoint *> mvpCurrentMatchedPoints;//当前帧 闭环检测得到的 匹配地图点
        std::vector<MapPoint *> mvpLoopMapPoints;// 当前帧 相邻关键帧上的 地图点 闭环处的地图点
        cv::Mat mScw;
        g2o::Sim3 mg2oScw;

        long unsigned int mLastLoopKFid;

        // Variables related to Global Bundle Adjustment
        bool mbRunningGBA = false;
        bool mbFinishedGBA = true;
        bool mbStopGBA = false;
        std::mutex mMutexGBA;
        std::thread *mpThreadGBA;

        // Fix scale in the stereo/RGB-D case
        bool mbFixScale;  // 双目/深度 固定的 空间尺度
        bool mnFullBAIdx;

// ===================add 
        std::mutex mMutexMapUpdateFlag;
        bool mbMapUpdateFlagForTracking = false;
    };

} //namespace ygz

#endif // LOOPCLOSING_H
