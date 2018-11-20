/**
* This file is part of ORB-SLAM2.
* 全局/局部 优化 使用G2O图优化
  IMU全局、局部、本征图、相似变换、单目深度...优化
*/

#ifndef YGZ_OPTIMIZER_H_
#define YGZ_OPTIMIZER_H_

#include "Map.h"
#include "MapPoint.h"
#include "KeyFrame.h"
#include "LoopClosing.h"
#include "Frame.h"
#include "Common.h"

// 所有和优化相关的函数

namespace ygz {

    class LoopClosing;

    class Optimizer {
    public:

        // IMU 相关===============导航状态全局BA优化=========================
        void static GlobalBundleAdjustmentNavState(
                                                  Map *pMap, 
                                                  const Vector3d &gw, 
                                                  int nIterations, 
                                                  bool *pbStopFlag,
                                                  const unsigned long nLoopKF, 
                                                  const bool bRobust);

        int static
        PoseOptimization(Frame *pFrame, 
                                            KeyFrame *pLastKF, 
                                            const IMUPreintegrator &imupreint, 
                                            const Vector3d &gw,
                                            const bool &bComputeMarg = false);

        int static
        PoseOptimization(Frame *pFrame, 
                                             Frame *pLastFrame, 
                                             const IMUPreintegrator &imupreint, 
                                             const Vector3d &gw,
                                             const bool &bComputeMarg = false);
        // =导航状态局部BA优化=========================
        void static
        LocalBundleAdjustmentNavState(KeyFrame *pKF, 
                                                                           const std::list<KeyFrame *> &lLocalKeyFrames, 
                                                                           bool *pbStopFlag,
                                                                           Map *pMap, 
                                                                           const Vector3d &gw, 
                                                                          LocalMapping *pLM = NULL);

        Vector3d static OptimizeInitialGyroBias(const std::list<KeyFrame *> &lLocalKeyFrames);

        Vector3d static OptimizeInitialGyroBias(const std::vector<KeyFrame *> &vLocalKeyFrames);

        Vector3d static OptimizeInitialGyroBias(const std::vector<Frame> &vFrames);

        Vector3d static OptimizeInitialGyroBias(const vector<SE3d> &vTwc, 
                                                                                         const vector<IMUPreintegrator> &vImuPreInt);


        // 视觉相关的 优化===========================================================
        void static BundleAdjustment(const std::vector<KeyFrame *> &vpKF, 
                                                                       const std::vector<MapPoint *> &vpMP,
                                                                       int nIterations = 5, 
                                                                       bool *pbStopFlag = NULL, 
                                                                       const unsigned long nLoopKF = 0,
                                                                       const bool bRobust = true);
        // 全局BA优化===
        void static GlobalBundleAdjustemnt(Map *pMap,
                                                                                   int nIterations = 5, 
                                                                                   bool *pbStopFlag = NULL,
                                                                                   const unsigned long nLoopKF = 0, 
                                                                                   const bool bRobust = true);
       // 局部BA优化====
        void static LocalBundleAdjustment(KeyFrame *pKF, 
                                                                                  bool *pbStopFlag, 
                                                                                  Map *pMap, 
                                                                                  LocalMapping *pLM = NULL);
         // 位姿 优化====
        int static PoseOptimization(Frame *pFrame);
         
        // 本征图，所有节点的最小生成树图，全局优化=========
        // 优化图 3d位姿 + 3d平移  双目/深度 优化6维
        // 单目 在优化 3d位姿 + 3d平移上 再 优化一维 深度信息
        // if bFixScale is true, 6DoF optimization (stereo,rgbd), 7DoF otherwise (mono)
        void static OptimizeEssentialGraph(
                                           Map *pMap, 
                                           KeyFrame *pLoopKF, 
                                           KeyFrame *pCurKF,
                                           const LoopClosing::KeyFrameAndPose &NonCorrectedSim3,
                                           const LoopClosing::KeyFrameAndPose &CorrectedSim3,
                                           const map<KeyFrame *, set<KeyFrame *> > &LoopConnections,
                                           const bool &bFixScale, LoopClosing *pLC = NULL);

        // if bFixScale is true, optimize SE3 (stereo,rgbd), Sim3 otherwise (mono)
       // 相似变换优化=================================
        static int OptimizeSim3(
                                KeyFrame *pKF1, 
                                KeyFrame *pKF2, 
                                std::vector<MapPoint *> &vpMatches1,
                                g2o::Sim3 &g2oS12, 
                                const float th2, 
                                const bool bFixScale);
    };

} //namespace ygz

#endif // OPTIMIZER_H
