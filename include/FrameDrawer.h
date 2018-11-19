/**
* This file is part of ORB-SLAM2.
* 关键帧显示
*/

#ifndef YGZ_FRAMEDRAWER_H_
#define YGZ_FRAMEDRAWER_H_

#include "Tracking.h"
#include "MapPoint.h"
#include "Map.h"

// Viewer里用到的画出帧信息的结构，并没做什么修改（除了窗口标题）

namespace ygz {

    class Tracking;// 声明跟踪类
    class Viewer;   // 声明 可视化类

    class FrameDrawer {
    public:
        FrameDrawer(Map *pMap);

        // Update info from the last processed frame.
        void Update(Tracking *pTracker);

        // Draw last processed frame.
        cv::Mat DrawFrame();// 显示上一帧

    protected:

        void DrawTextInfo(cv::Mat &im, int nState, cv::Mat &imText);//显示 文本信息

        // Info of the frame to be drawn
        cv::Mat mIm;
        int N;
        vector<cv::KeyPoint> mvCurrentKeys;
        vector<bool> mvbMap, mvbVO;
        bool mbOnlyTracking;
        int mnTracked, mnTrackedVO;
        vector<cv::KeyPoint> mvIniKeys;
        vector<int> mvIniMatches;
        int mState;
        vector<int> mvMatchedFrom;// 标识每个特征点是从哪个Keyframe选取的========

        Map *mpMap;

        std::mutex mMutex;
    };

} //namespace ygz

#endif // YGZ_FRAMEDRAWER_H
