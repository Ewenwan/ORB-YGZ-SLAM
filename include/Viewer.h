/**
* This file is part of ORB-SLAM2.
 可视化类，显示地图点、关键帧、当前相机位置、上次处理的帧、当前帧看到的地图点
*/


#ifndef YGZ_VIEWER_H_
#define YGZ_VIEWER_H_

#include "FrameDrawer.h"
#include "MapDrawer.h"
#include "Tracking.h"
#include "System.h"

// 可视化类，基本没动
namespace ygz {

    class Tracking;// 跟踪器
    class FrameDrawer;// 帧显示器
    class MapDrawer;   // 地图点显示器
    class System;            // 系统对象

    class Viewer {
    public:
// 类构造函数====================
        Viewer(System *pSystem, 
                       FrameDrawer *pFrameDrawer, 
                       MapDrawer *pMapDrawer, 
                       Tracking *pTracking,
                       const string &strSettingPath);

        // Main thread function. Draw points, keyframes, the current camera pose and the last processed
        // frame. Drawing is refreshed according to the camera fps. We use Pangolin.
        void Run(); // 可视化主线程函数========
        // 显示地图点、关键帧、当前相机位置、上次处理的帧、当前帧看到的地图点=====

        void RequestFinish();// 请求完成
        void RequestStop();// 请求结束
        bool isFinished();// 是否完成
        bool isStopped();//是否停止
        void Release();// 释放

    private:

        bool Stop();// 停止
        System *mpSystem;// 系统对象
        FrameDrawer *mpFrameDrawer;//帧显示器
        MapDrawer *mpMapDrawer;// 地图点显示器
        Tracking *mpTracker;// 跟踪器

        // 1/fps in ms
        double mT; // 没帧 时间   帧率导数 ===
        float mImageWidth, mImageHeight;// 图像尺寸

        float mViewpointX, mViewpointY, mViewpointZ, mViewpointF;// 视点坐标

        bool CheckFinish();// 检测是否完成

        void SetFinish();//设置完成

        bool mbFinishRequested;
        bool mbFinished;
        std::mutex mMutexFinish;

        bool mbStopped;
        bool mbStopRequested;
        std::mutex mMutexStop;

    };

}


#endif // VIEWER_H
	

