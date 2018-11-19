/**
* This file is part of ORB-SLAM2.
*
*  后端建图和优化线程
* LocalMapping作用是将Tracking中送来的关键帧放在mlNewKeyFrame列表中；
* 处理新关键帧，地图点检查剔除，生成新地图点，Local BA，关键帧剔除。
* 主要工作在于维护局部地图，也就是SLAM中的Mapping。
* 
* Tracking线程 只是判断当前帧是否需要加入关键帧，并没有真的加入地图，
* 因为Tracking线程的主要功能是局部定位，
* 
* 而处理地图中的关键帧，地图点，包括如何加入，
* 如何删除的工作是在LocalMapping线程完成的
* 
*/

#ifndef YGZ_LOCALMAPPING_H_
#define YGZ_LOCALMAPPING_H_

#include "Common.h"

// 后端建图和优化线程

namespace ygz {

    class Tracking;
    class LoopClosing;
    class Map;

    class KeyFrame;//===========add =====
    class KeyFrameDatabase;//
    class ConfigParam;//
    class MapPoint;//

    class LocalMapping {

// IMU 新添加 =================================================================
    public:
        ConfigParam *mpParams= nullptr; // IMU配置参数====
        bool mbUseIMU;

        std::thread *mptLocalMappingVIOInit =nullptr;   // 单目 初始化线程

        // KeyFrames in Local Window, for Local BA
        // Insert in ProcessNewKeyFrame()
        void AddToLocalWindow(KeyFrame *pKF);

        void DeleteBadInLocalWindow(void);

        mutex mMutexVINSIniting; // IMU锁=======
        bool mbVINSIniting;
        bool GetVINSIniting(void);
        void SetVINSIniting(bool flag);
        bool mbResetVINSInit;
        bool GetResetVINSInit(void);
        bool SetResetVINSInit(bool flag);
        void VINSInitThread(void);
        bool TryInitVIO(void);
        bool GetVINSInited(void);
        void SetVINSInited(bool flag);
        bool GetFirstVINSInited(void);
        void SetFirstVINSInited(bool flag);
        Vector3d GetGravityVec(void);
        double GetVINSInitScale(void) { return mnVINSInitScale; }

        bool GetMapUpdateFlagForTracking();
        void SetMapUpdateFlagInTracking(bool bflag);
        KeyFrame *GetMapUpdateKF();

        std::mutex mMutexUpdatingInitPoses;

        bool GetUpdatingInitPoses(void);
        void SetUpdatingInitPoses(bool flag);

        //  imu初始化相关========================
    protected:
        double mnStartTime;
        bool mbFirstTry;
        double mnVINSInitScale;
        Vector3d mGravityVec; // gravity vector in world frame

        std::mutex mMutexVINSInitFlag;
        bool mbVINSInited;

        std::mutex mMutexFirstVINSInitFlag;
        bool mbFirstVINSInited;

        unsigned int mnLocalWindowSize;
        std::list<KeyFrame *> mlLocalKeyFrames;

        std::mutex mMutexMapUpdateFlag;
        bool mbMapUpdateFlagForTracking;
        KeyFrame *mpMapUpdateKF;

        bool mbUpdatingInitPoses;

        std::mutex mMutexCopyInitKFs;
        bool mbCopyInitKFs;

        bool GetFlagCopyInitKFs() {
            unique_lock<mutex> lock(mMutexCopyInitKFs);
            return mbCopyInitKFs;
        }

        void SetFlagCopyInitKFs(bool flag) {
            unique_lock<mutex> lock(mMutexCopyInitKFs);
            mbCopyInitKFs = flag;
        }

//  原 orb ==================================================================
    public:
        LocalMapping(Map *pMap, const float bMonocular, ConfigParam *pParams);

        void SetLoopCloser(LoopClosing *pLoopCloser);// 闭环检测
        void SetTracker(Tracking *pTracker);// 跟踪器

        // Main function
        void Run();// 局部建图 主函数==============
        void InsertKeyFrame(KeyFrame *pKF); // 插入关键帧====

        // Thread Synch    线程同步=================
        void RequestStop();
        void RequestReset();
        bool Stop();
        void Release();
        bool isStopped();
        bool stopRequested();
        bool AcceptKeyFrames();

        void SetAcceptKeyFrames(bool flag);

        bool SetNotStop(bool flag);

        void InterruptBA();

        void RequestFinish();

        bool isFinished();

        int KeyframesInQueue() {
            unique_lock<std::mutex> lock(mMutexNewKFs);
            return mlNewKeyFrames.size();
        }

    protected:

        bool CheckNewKeyFrames();

        void ProcessNewKeyFrame();
 // 处理新关键帧：ProcessNewKeyFrame()   
/*a. 根据词典 计算当前关键帧Bow，便于后面三角化恢复新地图点；
b. 将TrackLocalMap中跟踪局部地图匹配上的地图点绑定到当前关键帧
    （在Tracking线程中只是通过匹配进行局部地图跟踪，优化当前关键帧姿态），
    也就是在graph 中加入当前关键帧作为node，并更新edge。
    
    而CreateNewMapPoint()中则通过当前关键帧，在局部地图中添加与新的地图点；

c. 更新加入当前关键帧之后关键帧之间的连接关系，包括更新Covisibility图和Essential图
 （最小生成树spanning tree，共视关系好的边subset of edges from covisibility graph 
   with high covisibility (θ=100)， 闭环边）。
*/

 // 而CreateNewMapPoint()中则通过当前关键帧，在局部地图中添加与新的地图点；
        void CreateNewMapPoints();

// 对于ProcessNewKeyFrame和CreateNewMapPoints中最近添加的MapPoints进行检查剔除
        void MapPointCulling();

        void SearchInNeighbors();

        void KeyFrameCulling();

        Matrix3f ComputeF12(KeyFrame *&pKF1, KeyFrame *&pKF2);

        bool mbMonocular;

        void ResetIfRequested();

        bool mbResetRequested;
        std::mutex mMutexReset;

        bool CheckFinish();

        void SetFinish();

        bool mbFinishRequested;
        bool mbFinished;
        std::mutex mMutexFinish;

        Map *mpMap =nullptr;

        LoopClosing *mpLoopCloser =nullptr;
        Tracking *mpTracker =nullptr;

        std::list<KeyFrame *> mlNewKeyFrames; ///< 等待处理的关键帧列表

        KeyFrame *mpCurrentKeyFrame =nullptr;

        std::list<MapPoint *> mlpRecentAddedMapPoints;

        std::mutex mMutexNewKFs;

        bool mbAbortBA;
        bool mbStopped;
        bool mbStopRequested;
        bool mbNotStop;
        std::mutex mMutexStop;

        bool mbAcceptKeyFrames;
        std::mutex mMutexAccept;
    };

} //namespace ygz

#endif // LOCALMAPPING_H
