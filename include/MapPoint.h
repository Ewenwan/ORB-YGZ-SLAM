/**
* This file is part of ORB-SLAM2.
* 地图点  普通帧上的地图点
*               关键帧上的地图点
* 创建 地图点    观测帧集合   最优的描述子
* 坏点 检测 被观测次数 小于2  删除地图点
* 地图点距离 参考帧相机中心的 相对 坐标
* 地图点 相对参考帧 相机中心  在 图像金字塔上 每层 各个尺度空间的距离
* 
*/

#ifndef YGZ_MAPPOINT_H
#define YGZ_MAPPOINT_H

#include "Common.h"

namespace ygz {

class KeyFrame;// 关键帧
class Map;// 地图
class Frame;//普通帧

// 现在这个 map point 锁的有点太频繁
    class MapPoint {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // 构造函数（地图点3D坐标及其参考帧）
	// 创建关键帧地图点   世界坐标点     所属关键帧    所属地图
	// 参考帧是关键帧，该地图点将于许多帧关键帧对应，建立关键帧之间的共视关系
        MapPoint(const Vector3f &Pos, KeyFrame *pRefKF, Map *pMap);
        // 参考帧是普通帧，该地图点只与当前普通帧的特征点对应 
        // 创建普通帧地图点    世界坐标点       所属地图   所属普通 帧      帧id
        MapPoint(const Vector3f &Pos, Map *pMap, Frame *pFrame, const int &idxF);


        std::map<KeyFrame *, size_t> GetObservations();
        // 观察到该地图点的 所有 关键帧，+观测次数
        int Observations();// 观测关系
        void AddObservation(KeyFrame *pKF, size_t idx);// 添加观测关系
        void EraseObservation(KeyFrame *pKF);// 删除观测关系

        int GetIndexInKeyFrame(KeyFrame *pKF);
        bool IsInKeyFrame(KeyFrame *pKF);

        void SetBadFlag();// 不好的标记
        bool isBad();

        void Replace(MapPoint *pMP);// 更新替换地图点
        MapPoint *GetReplaced();// 获取替换者

        void IncreaseVisible(int n = 1);
        void IncreaseFound(int n = 1);

 //GetFoundRatio低表示该地图点在很多关键帧的视野范围内，但是没有匹配上很多特征点。
        float GetFoundRatio();
        inline int GetFound() {
            return mnFound;
        }

// 1 在观测到该地图点的多个特征点中（对应多个关键帧），
     // 挑选出区分度最高的描述子，作为这个地图点的描述子；
        void ComputeDistinctiveDescriptors();
        cv::Mat GetDescriptor();
// 2. 计算地图点平均观测方向和深度 （在所有观测帧上）
        void UpdateNormalAndDepth();

        float GetMinDistanceInvariance();
        float GetMaxDistanceInvariance();
        int PredictScale(const float &currentDist, KeyFrame*pKF);
        int PredictScale(const float &currentDist, Frame*pF);

// 更新尺度============================
        void UpdateScale(float scale) {
            SetWorldPos(mWorldPos * scale);
            mfMinDistance *= scale;
            mfMaxDistance *= scale;
        }
        // accessors, will be locked
        void SetWorldPos(const Vector3f &Pos);
        Vector3f GetWorldPos();
        Vector3f GetNormal();
        KeyFrame *GetReferenceKeyFrame();


    public:
        long unsigned int mnId = 0; ///< Global ID for MapPoint
        static long unsigned int nNextId;
        long int mnFirstKFid = -1; ///< 创建该MapPoint的关键帧ID
        long int mnFirstFrame = -1; ///< 创建该MapPoint的帧ID（即每一关键帧有一个帧ID）
        int nObs = 0;//    观测到该地图点的相机数  关键帧数量

        // Variables used by the tracking 跟踪 到的 参数 
        // predicted position in tracking
        float mTrackProjX = 0;//匹配点 x偏移
        float mTrackProjY = 0;//匹配点 y偏移
        float mTrackProjXR = 0;
        bool mbTrackInView = false;
        int mnTrackScaleLevel = 0;
        float mTrackViewCos = 0;
        long unsigned int mnTrackReferenceForFrame = 0;
        long unsigned int mnLastFrameSeen = 0;

        // Variables used by local mapping
        long unsigned int mnBALocalForKF = 0;
        long unsigned int mnFuseCandidateForKF = 0;// 地图点 和 哪一帧的 地图点有融合 的标记

        // Variables used by loop closing
        long unsigned int mnLoopPointForKF = 0;
        long unsigned int mnCorrectedByKF = 0;
        long unsigned int mnCorrectedReference = 0;
        long unsigned int mnBAGlobalForKF = 0;
        Vector3f mPosGBA = Vector3f(0, 0, 0);

        static std::mutex mGlobalMutex; // 整个地图的锁

    private:

        // Position in absolute coordinates
        Vector3f mWorldPos; ///< MapPoint在世界坐标系下的坐标

        // Keyframes observing the point and associated index in keyframe
        std::map<KeyFrame *, size_t> mObservations;// 观测关键帧序列

        // Mean viewing direction
        Vector3f mNormalVector = Vector3f(0, 0, 0);    // 平均 法线 or 观测角

        // Best descriptor to fast matching
        cv::Mat mDescriptor; ///< 通过 ComputeDistinctiveDescriptors() 得到的 最优描述子

        // Reference KeyFrame
        KeyFrame *mpRefKF = nullptr;     // 如果reference被cull掉，是否应该这个地图点也删掉？
        float mfInvDepth = 0;           // Reference frame 中的逆深度
        float mfInvDepthMax = 0, mfInvDepthMin = 0; // 逆深度的最大最小值

        // Tracking counters
        int mnVisible = 1;
        int mnFound = 1;

        // Bad flag (we do not currently erase MapPoint from memory)
        bool mbBad = false;// 坏点标志  被观测到的次数  小于2
        MapPoint *mpReplaced = nullptr;

        // Scale invariance distances
        float mfMinDistance = 0;
        float mfMaxDistance = 0;

        Map *mpMap = nullptr;

        std::mutex mMutexPos;      // 位置锁
        std::mutex mMutexFeatures; // Feature的锁

    };

} //namespace ORB_SLAM

#endif // MAPPOINT_H
