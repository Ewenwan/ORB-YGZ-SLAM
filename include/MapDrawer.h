/**
* This file is part of ORB-SLAM2.
* 地图的可视化 显示 地图点 关键帧 当前相机 大小参数配置
*/

#ifndef YGZ_MAPDRAWER_H_
#define YGZ_MAPDRAWER_H_

#include "Map.h"
#include "MapPoint.h"
#include "KeyFrame.h"
#include "Common.h"

// 地图的可视化

namespace ygz {

    class MapDrawer {
    public:
        MapDrawer(Map *pMap, const string &strSettingPath);

        Map *mpMap;

        void DrawMapPoints();// 显示地图点
        void DrawKeyFrames(const bool bDrawKF, const bool bDrawGraph);//显示关键帧
        void DrawCurrentCamera(pangolin::OpenGlMatrix &Twc);// 显示当前相机
        void SetCurrentCameraPose(const SE3d &Tcw);// 设置当前相机位姿
        void SetReferenceKeyFrame(KeyFrame *pKF);// 设置参考关键帧
        void GetCurrentOpenGLCameraMatrix(pangolin::OpenGlMatrix &M);// 

    private:

        float mKeyFrameSize; // 关键帧数量
        float mKeyFrameLineWidth;// 关键帧 线长度
        float mGraphLineWidth;// 图线长度
        float mPointSize;// 点大小
        float mCameraSize;// 相机大小
        float mCameraLineWidth;// 相机线长度

        SE3d mCameraPose; // 相机位姿  T using Sophus::SE3d;    Common.h

        std::mutex mMutexCamera; // 锁===
    };

} //namespace ygz

#endif // MAPDRAWER_H
