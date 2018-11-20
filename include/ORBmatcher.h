/**
* This file is part of ORB-SLAM2.
* 
* 该类负责特征点与特征点之间，地图点与特征点之间通过投影关系、词袋模型或者Sim3位姿匹配。
* 用来辅助完成单目初始化，三角化恢复新的地图点，tracking，relocalization以及loop closing，
* 因此比较重要。
* 
各类之间的匹配   局部匹配  全局匹配等 

何时用投影匹配，何时用DBow2进行匹配？
在Relocalization和LoopClosing中进行匹配的是在很多帧关键帧集合中匹配，
属于Place Recognition，因此需要用DBow，

而 投影匹配 适用于两帧之间，
或者投影范围内（局部地图，前一个关键帧对应地图点）的MapPoints与当前帧之间。
*/


#ifndef ORBMATCHER_H
#define ORBMATCHER_H


#include "MapPoint.h"
#include "KeyFrame.h"
#include "Frame.h"
#include "Common.h"

// 匹配算法

namespace ygz {

    const int WarpHalfPatchSize = 4;
    const int WarpPatchSize = 8;

    class ORBmatcher {
    public:

        ORBmatcher(float nnratio = 0.6, bool checkOri = true);// 类构造函数

        // Computes the Hamming distance between two ORB descriptors
        // 两个ORB之间的距离
        // 计算两个 特征点 的 ORB二进制 描述子之间的 汉明字符串匹配距离
        static int DescriptorDistance(const cv::Mat &a, const cv::Mat &b);

// a. 局部地图跟踪===================================================================
        // Search matches between Frame keypoints and projected MapPoints. Returns number of matches
        // Used to track the local map (Tracking)
        // 和local map之间的projection  当前帧 和 局部地图 之间的投影匹配======
        // 在TrackReferenceKeyFrame和TrackWithMotionModel中，
        // 仅仅是两帧之间跟踪，会跟丢地图点，
        // 这里通过跟踪局部地图，在当前帧中恢复出一些当前帧的地图点。
        /**
         * @brief 通过投影，对Local MapPoint进行跟踪
         *
         * 将Local MapPoint投影到当前帧中, 由此增加当前帧的 MapPoints \n
         * 在 SearchLocalPoints() 中已经将 Local MapPoints重投影（isInFrustum()）到当前帧 \n
         * 并标记了这些点是否在当前帧的视野中，即mbTrackInView \n
         * 对这些MapPoints，在其投影点附近根据描述子距离选取匹配，以及最终的方向投票机制进行剔除
         * @param  F           当前帧
         * @param  vpMapPoints Local MapPoints 局部地图
         * @param  th          阈值，窗口的大小倍数
         * @param checkLevel   是否检测金字塔的层数？(在类DSO提取算法中，只有第0层金字塔的特征点)
         * @return             成功匹配的数量
         * @see SearchLocalPoints() isInFrustum()
         */
        int SearchByProjection(
                                                        Frame &F, 
                                                        const std::vector<MapPoint *> &vpMapPoints, 
                                                        const float th = 3, bool checkLevel = true
        );

// b. 运动跟踪模式，跟踪上一帧======================================================
        // Project MapPoints tracked in last frame into the current frame and search matches.
        // Used to track from previous frame (Tracking)
        // 和上一帧之间的projection
        // @param[th] 窗口的大小
        /**
         * @brief 通过投影，对上一帧的特征点进行匹配
         *
         * 上一帧中包含了MapPoints，对这些MapPoints进行tracking，由此增加当前帧的MapPoints \n
         * 1. 将上一帧的MapPoints投影到当前帧(根据速度模型可以估计当前帧的Tcw)
         * 2. 在投影点附近根据描述子距离选取匹配，以及最终的方向投票机制进行剔除
         * @param  CurrentFrame 当前帧
         * @param  LastFrame    上一帧
         * @param  th                     阈值
         * @param  bMono          是否为单目
         * @param  checkLevel   是否检查层数
         * @return                           成功匹配的数量
         * @see SearchByBoW()
         */
        int SearchByProjection(Frame &CurrentFrame, 
                                                        const Frame &LastFrame, 
                                                        const float th, const bool bMono,
                                                        bool checkLevel = true);

        // Project MapPoints seen in KeyFrame into the Frame and search matches.
        // Used in relocalisation (Tracking)
//  c. 在当前帧中匹配所有关键帧中的地图点，用于重定位 Relocalization======================
       // DBow2 匹配  使用 特征词典 向量进行匹配 
       int SearchByProjection(Frame &CurrentFrame, 
                                                        KeyFrame *pKF, 
                                                        const std::set<MapPoint *> &sAlreadyFound,
                                                        const float th, const int ORBdist);
// d. 闭环检测匹配跟踪，相似变换跟踪================================================
        //  DBow2 匹配  使用 特征词典 向量进行匹配 
        // Project MapPoints using a Similarity Transformation and search matches.
        // Used in loop detection (Loop Closing)
        int SearchByProjection(KeyFrame *pKF, 
                                                       cv::Mat Scw, 
                                                       const std::vector<MapPoint *> &vpPoints,
                                                        std::vector<MapPoint *> &vpMatched, int th);

        // Search matches between MapPoints in a KeyFrame and ORB in a Frame.
        // Brute force constrained to ORB that belong to the same vocabulary node (at a certain level)
        // Used in Relocalisation and Loop Detection
        /**
         * @brief 通过词包，对关键帧的特征点进行跟踪
         *
         * KeyFrame中包含了MapPoints，对这些MapPoints进行tracking \n
         * 由于每一个MapPoint对应有描述子，因此可以通过描述子距离进行跟踪 \n
         * 为了加速匹配过程，将关键帧和当前帧的描述子划分到特定层的nodes中 \n
         * 对属于同一node的描述子计算距离进行匹配 \n
         * 通过距离阈值、比例阈值和角度投票进行剔除误匹配
         * @param  pKF               KeyFrame      关键帧
         * @param  F                 Current Frame 当前帧
         * @param  vpMapPointMatches F中MapPoints对应的匹配，NULL表示未匹配
         * @return                   成功匹配的数量
         */
        int SearchByBoW(KeyFrame *pKF, 
                                            Frame &F, 
                                            std::vector<MapPoint *> &vpMapPointMatches);
        // 闭环检测匹配跟踪 使用的 词包 跟踪====
        int SearchByBoW(KeyFrame *pKF1, 
                                             KeyFrame *pKF2,
                                             std::vector<MapPoint *> &vpMatches12);

        // Matching for the Map Initialization (only used in the monocular case)
// 单目初始化时 的匹配====================================
        int SearchForInitialization(Frame &F1, 
                                                             Frame &F2,
                                                             std::vector<cv::Point2f> &vbPrevMatched,
                                                             std::vector<int> &vnMatches12, int windowSize = 10);

        // Matching to triangulate new MapPoints. Check Epipolar Constraint.
// 利用三角化，在两个关键帧之间恢复出一些地图点=========================
        int SearchForTriangulation(KeyFrame *pKF1, 
                                                              KeyFrame *pKF2, 
                                                              Matrix3f &F12,
                                                              std::vector<pair<size_t, size_t> > &vMatchedPairs, 
                                                              const bool bOnlyStereo);

        // Search matches between MapPoints seen in KF1 and KF2 transforming by a Sim3 [s12*R12|t12]
        // In the stereo and RGB-D case, s12=1
// 回环检测处 有相识变换时的三角变换=====================================
        int SearchBySim3(KeyFrame *pKF1, 
                                            KeyFrame *pKF2, 
                                            std::vector<MapPoint *> &vpMatches12, 
                                            const float &s12,
                                            const cv::Mat &R12, 
                                            const cv::Mat &t12, const float th);

  //  两个重载的Fuse融合函数，用于地图点的融合=====================================
    // 地图点能匹配上当前关键帧的地图点，也就是地图点重合了，选择观测数多的地图点替换；
    // 地图点能匹配上当前帧的特征点，但是该特征点还没有生成地图点，则生成新的地图点）。
        // Project MapPoints into KeyFrame and search for duplicated MapPoints.
        int Fuse(KeyFrame *pKF, const vector<MapPoint *> &vpMapPoints, const float th = 3.0);
       //重载的函数是为了减小尺度漂移的影响，需要知道当前关键帧的sim3位姿。
        // Project MapPoints into KeyFrame using a given Sim3 and search for duplicated MapPoints.
        int Fuse(KeyFrame *pKF, cv::Mat Scw, const std::vector<MapPoint *> &vpPoints, float th,
                 vector<MapPoint *> &vpReplacePoint);

        /************************************/
        // 直接法的匹配
        // 用直接法判断能否从在当前图像上找到某地图点的投影
        // 这个函数经常会有 误拒的情况，需要进一步检查。
        bool FindDirectProjection(KeyFrame *ref, 
                                                             Frame *curr, MapPoint *mp,
                                                             Vector2f &px_curr, int &search_level);

    public:
        // 常量
        static const int TH_LOW;
        static const int TH_HIGH;
        static const int HISTO_LENGTH;

    private:
       // 极线 检测======
        bool CheckDistEpipolarLine(const cv::KeyPoint &kp1, const cv::KeyPoint &kp2, 
                                                                 const Matrix3f &F12,
                                                                 const KeyFrame *pKF);
        // 距离
        float RadiusByViewingCos(const float &viewCos);
// 方向一致性===============================================
// 统计方向直方图最高的三个bin保留，其他范围内的匹配点剔除。
// 另外，若最高的比第二高的高10倍以上，则只保留最高的bin中的匹配点。
// 若最高的比第 三高的高10倍以上，则 保留最高的和第二高bin中的匹配点。
        void ComputeThreeMaxima(std::vector<int> *histo, const int L, int &ind1, int &ind2, int &ind3);

// 直接法匹配相关==============================================================
        // 计算affine wrap 仿射变换 矩阵
        void GetWarpAffineMatrix(
                KeyFrame *ref,
                Frame *curr,
                const Vector2f &px_ref,
                MapPoint *mp,
                int level,
                const SE3f &TCR,
                Eigen::Matrix2f &ACR
        );

        // perform affine warp
        void WarpAffine(
                const Eigen::Matrix2f &ACR,
                const cv::Mat &img_ref,
                const Vector2f &px_ref,
                const int &level_ref,
                const KeyFrame *ref,
                const int &search_level,
                const int &half_patch_size,
                uint8_t *patch
        );

        // 计算最好的金字塔层数===============================
        // 选择一个分辨率，使得warp不要太大
        // ORB每层金字塔默认是1.2倍缩放，所以每缩小一层是1.2*1.2=1.44,取倒数为0.694444444444
        inline int GetBestSearchLevel(
                const Eigen::Matrix2f &ACR,
                const int &max_level,
                const KeyFrame *ref
        ) {
            int search_level = 0;
            float D = ACR.determinant();
            while (D > 3.0 && search_level < max_level) {
                search_level += 1;
                D *= ref->mvInvLevelSigma2[1];
            }
            return search_level;
        }

        // 双线性插值================================================
        inline uchar GetBilateralInterpUchar(
                const double &x, const double &y, const Mat &gray) {
            const double xx = x - floor(x);
            const double yy = y - floor(y);
            uchar *data = &gray.data[int(y) * gray.step + int(x)];
            return uchar(
                    (1 - xx) * (1 - yy) * data[0] +
                    xx * (1 - yy) * data[1] +
                    (1 - xx) * yy * data[gray.step] +
                    xx * yy * data[gray.step + 1]
            );
        }

        // 匹配局部地图用的 patch, 默认8x8============================
        uchar _patch[WarpPatchSize * WarpPatchSize];
        // 带边界的，左右各1个像素
        uchar _patch_with_border[(WarpPatchSize + 2) * (WarpPatchSize + 2)];

        float mfNNratio;
        bool mbCheckOrientation;
    };

}// namespace ygz

#endif // ORBMATCHER_H
