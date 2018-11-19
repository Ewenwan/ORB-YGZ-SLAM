/**
* This file is part of ORB-SLAM2.
* 关键帧
* 
* 普通帧里面精选出来的具有代表性的帧
* 普通Frame的数据都是公开访问的 关键帧Keyframe多数操作是带锁的,会被三个线程 同时访问
* 
*/

#ifndef YGZ_KEYFRAME_H_
#define YGZ_KEYFRAME_H_

#include "Common.h"
#include "Thirdparty/DBoW2/DBoW2/BowVector.h"
#include "Thirdparty/DBoW2/DBoW2/FeatureVector.h"
#include "ORBVocabulary.h"
#include "ORBextractor.h"

#include "IMU/imudata.h"
#include "IMU/NavState.h"
#include "IMU/IMUPreintegrator.h"

namespace ygz {

	class Map;// 地图
	class MapPoint;// 地图点
	class Frame;// 普通帧
	class KeyFrameDatabase;//关键帧数据库  存储关键点 位姿态等信息 用于匹配

    /* KeyFrame
     * 关键帧，和普通的Frame不一样，但是可以由Frame来构造
     * Frame的数据都是公开访问的，但Keyframe多数操作 是带锁的
     * 许多数据会被 三个线程 同时访问，所以 用锁 的地方很普遍
     */
    class KeyFrame {
    public:

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        // 从普通Frame 转 KeyFrame的构造函数，普通帧的 量全部复制过来
        KeyFrame(Frame &F, Map *pMap, KeyFrameDatabase *pKFDB);

        // VIO constructor  带有IMU数据的关键帧===========================new=
        KeyFrame(Frame &F, Map *pMap, 
                             KeyFrameDatabase *pKFDB, 
                             std::vector<IMUData> vIMUData,  // imu数据
                             KeyFrame *pLastKF = NULL);          // 上一帧

        // Pose functions====================================================
        // KeyFrame的Pose可能被多个线程更新，因此全部要上锁
        void SetPose(const SE3f &Tcw);//设置位姿
        SE3f GetPose();// 获取位姿
        SE3f GetPoseInverse();// 获取位姿 逆变换
        Vector3f GetCameraCenter();// 普通相机中心
        Vector3f GetStereoCenter();  // 双目相机中心
        Matrix3f GetRotation();           // 旋转矩阵
        Vector3f GetTranslation();      // 平移向量

        // 计关键点 描述子 的  词典线性表示向量=======
        void ComputeBoW();

// 图操作===============================================================
        // Covisibility graph functions  每个关键帧处的相机位置之间的关系“图”
        void AddConnection(KeyFrame *pKF, const int &weight);// 权值：边的可信度（每条边都有自己的权值）
        // cov graph operations
        void EraseConnection(KeyFrame *pKF);// 删除  连接关系
        void UpdateConnections();                         // 更新 连接关系
        void UpdateBestCovisibles();// 最好的
        std::set<KeyFrame *> GetConnectedKeyFrames();// 获取相关连的 关键帧集合
        std::vector<KeyFrame *> GetVectorCovisibleKeyFrames();// 关键帧向量
        std::vector<KeyFrame *> GetBestCovisibilityKeyFrames(const int &N);
        std::vector<KeyFrame *> GetCovisiblesByWeight(const int &w);// 依据权重获取
        int GetWeight(KeyFrame *pKF);// 获取边的权重======


        // Spanning tree functions      图中的  最小生成树======局部图===========
        void AddChild(KeyFrame *pKF);//  添加孩子
        void EraseChild(KeyFrame *pKF);
        void ChangeParent(KeyFrame *pKF);// 跟换 父节点
        std::set<KeyFrame *> GetChilds();// 获取孩子节点
        KeyFrame *GetParent();// 获取父节点
        bool hasChild(KeyFrame *pKF);// 判断是否有孩子节点

        // Loop Edges 闭环检测边==============================全局图=======
        void AddLoopEdge(KeyFrame *pKF);// 添加闭环边======
        std::set<KeyFrame *> GetLoopEdges();

        // MapPoint observation functions 地图点 函数=======================
        void AddMapPoint(MapPoint *pMP, const size_t &idx);// 添加 地图点
        void EraseMapPointMatch(const size_t &idx);// 删除地图点匹配
        void EraseMapPointMatch(MapPoint *pMP);
        void ReplaceMapPointMatch(const size_t &idx, MapPoint *pMP);// 替换
        std::set<MapPoint *> GetMapPoints();// 获取
        std::vector<MapPoint *> GetMapPointMatches();
        int TrackedMapPoints(const int &minObs);// 跟踪到的地图点
        MapPoint *GetMapPoint(const size_t &idx);


        // KeyPoint functions   关键点函数======================================
        std::vector<size_t> GetFeaturesInArea(const float &x, 
                                                                                       const float &y,
                                                                                        const float &r) const;

        Vector3f UnprojectStereo(int i);

        // Image  在图像里吗
        bool IsInImage(const float &x, const float &y) const;

        // Enable/Disable bad flag changes  设置标志  检查标志======
        void SetNotErase();
        void SetErase();
        // Set/check bad flag
        void SetBadFlag();
        bool isBad();

        // 单目 环境 深度中值 Compute Scene Depth (q=2 median). Used in monocular.
        float ComputeSceneMedianDepth(const int q);

        // 边权值比较函数
        static bool weightComp(int a, int b) {
            return a > b;
        }

        static bool lId(KeyFrame *pKF1, KeyFrame *pKF2) {
            return pKF1->mnId  <  pKF2->mnId; // 帧 id 比较
        }

// 怎么又有坐标变换====================================================
        // coordinate transform: world, camera, pixel
        inline Vector3f World2Camera(const Vector3f &p_w, const SE3f &T_c_w) const {
            return T_c_w * p_w; // 世界坐标系3d点 转换到 相机坐标系 3d点===
        }

        inline Vector3f Camera2World(const Vector3f &p_c, const SE3f &T_c_w) const {
            return T_c_w.inverse() * p_c;// 相机坐标系 3d点 转换到 世界坐标系3d点===
        }
        // 相机坐标系 3d点 转换到 图像平面 2d点====
        inline Vector2f Camera2Pixel(const Vector3f &p_c) const {
            return Vector2f(
                    fx * p_c(0, 0) / p_c(2, 0) + cx,
                    fy * p_c(1, 0) / p_c(2, 0) + cy
            );
        }
      // 图像平面 2d点 转换到 相机坐标系 3d点
        inline Vector3f Pixel2Camera(const Vector2f &p_p, float depth = 1) const {
            return Vector3f(
                    (p_p(0, 0) - cx) * depth / fx,
                    (p_p(1, 0) - cy) * depth / fy,
                    depth
            );
        }
      // 图像平面 2d点 转换到 相机坐标系 3d点 再到 世界坐标系3d点 
        inline Vector3f Pixel2World(const Vector2f &p_p, const SE3f &T_c_w, float depth = 1) const {
            return Camera2World(Pixel2Camera(p_p, depth), T_c_w);
        }
     // 世界坐标系3d点  转换到 相机坐标系 3d点 再到 图像平面 2d点 
        inline Vector2f World2Pixel(const Vector3f &p_w, const SE3f &T_c_w) const {
            return Camera2Pixel(World2Camera(p_w, T_c_w));
        }

// IMU 相关=========================================================
        KeyFrame *GetPrevKeyFrame(void); // 获取 前一个关键帧
        KeyFrame *GetNextKeyFrame(void);// 获取  后一个关键帧
        void SetPrevKeyFrame(KeyFrame *pKF);// 设置 前一个关键帧
        void SetNextKeyFrame(KeyFrame *pKF);// 设置 后一个关键帧
        std::vector<IMUData> GetVectorIMUData(void);  // 获取IMU数据
        void AppendIMUDataToFront(KeyFrame *pPrevKF);// 添加imu数据??
        void ComputePreInt(void);
        const IMUPreintegrator &GetIMUPreInt(void); // IMU预积分
        void UpdateNavStatePVRFromTcw(const SE3d &Tcw, const SE3d &Tbc);
        void UpdatePoseFromNS(const SE3d &Tbc);
        void UpdateNavState(const IMUPreintegrator &imupreint, const Vector3d &gw);
        void SetNavState(const NavState &ns);// 导航状态========
        const NavState &GetNavState(void);
        void SetNavStateVel(const Vector3d &vel); // 速度
        void SetNavStatePos(const Vector3d &pos);//位置
        void SetNavStateRot(const Matrix3d &rot);// 旋转
        void SetNavStateRot(const SO3d &rot);
        void SetNavStateBiasGyr(const Vector3d &bg);// 角速度偏置
        void SetNavStateBiasAcc(const Vector3d &ba);// 加速度偏置
        void SetNavStateDeltaBg(const Vector3d &dbg);
        void SetNavStateDeltaBa(const Vector3d &dba);
        void SetInitialNavStateAndBias(const NavState &ns);

        // Variables used by loop closing
        NavState mNavStateGBA;       //mTcwGBA
        NavState mNavStateBefGBA;    //mTcwBefGBA

    public:

        // The following variables are accesed from only 1 thread or never change (no mutex needed).
        // nNextID名字改为nLastID更合适，表示上一个KeyFrame的ID号
        static long unsigned int nNextId;
        // 在 nNextID 的基础上加1 就得到了mnID，为 当前KeyFrame的ID号
        long unsigned int mnId =0;
        // 每个KeyFrame基本属性是它是一个Frame，KeyFrame初始化的时候需要Frame，

        // mnFrameId记录了该KeyFrame是由哪个Frame初始化的=================
        const long unsigned int mnFrameId;

        // 时间
        const double mTimeStamp =0;

        // Grid (to speed up feature matching)
        // 和Frame类中的定义相同
       // 640 *480 图像 分成 64 × 48 个格子 加速 特征匹配 ==============
        const int mnGridCols;
        const int mnGridRows;
        const float mfGridElementWidthInv;// 每一像素 占有的 格子数量
        const float mfGridElementHeightInv;

        // Variables used by the tracking  跟踪使用的变量==============
        long unsigned int mnTrackReferenceForFrame = 0;
        long unsigned int mnFuseTargetForKF = 0;
        // 做过相邻匹配 标志 在 LocalMapping::SearchInNeighbors() 使用

        // Variables used by the local mapping 局部建图使用的变量===========
        long unsigned int mnBALocalForKF = 0;// 最小化重投影误差 BA参数
        long unsigned int mnBAFixedForKF = 0;

        // Variables used by the keyframe database 关键帧数据库 变量参数=======
        long unsigned int mnLoopQuery = 0;
        int mnLoopWords = 0;
        float mLoopScore = 0;
        long unsigned int mnRelocQuery = 0;
        int mnRelocWords = 0;
        float mRelocScore = 0;

        // Variables used by loop closing   闭环检测 变量 ===================
        SE3f mTcwGBA;
        SE3f mTcwBefGBA;
        long unsigned int mnBAGlobalForKF;

        // Calibration parameters  校准参数 相机内参 基线*焦距 基线 深度=======
        const float fx, fy, cx, cy, invfx, invfy, mbf, mb, mThDepth;

        // Number of KeyPoints 关键点数量
        const int N;

        // KeyPoints, stereo coordinate and descriptors (all associated by an index)
        // 特征点位置 =================================================
        const std::vector<cv::KeyPoint> mvKeys;// 关键点 
        const std::vector<float> mvuRight; // 左图关键点在右图的 匹配点横坐标  单目 -1 不用
        const std::vector<float> mvDepth; // 深度值
        const cv::Mat mDescriptors; // 描述子，矩阵，一行代表一个关键点的描述子

        //BoW 词带模型 ================================================
        DBoW2::BowVector mBowVec; ///< Vector of words to represent images
        DBoW2::FeatureVector mFeatVec; ///< Vector of nodes with indexes of local features

        // Pose relative to parent (this is computed when bad flag is activated)
        SE3f mTcp; //  父位姿 到 当前关键帧 转换矩阵

        // Scale 图像金字塔尺度信息 ===========================
        const int mnScaleLevels;
        const float mfScaleFactor;
        const float mfLogScaleFactor;
        const std::vector<float> mvScaleFactors;
        const std::vector<float> mvLevelSigma2;
        const std::vector<float> mvInvLevelSigma2;

        // Image bounds and calibration   用于确定画格子时的边界 矫正====
        const int mnMinX;
        const int mnMinY;
        const int mnMaxX;
        const int mnMaxY;
        const Matrix3f mK;

        // image pyramid
        vector<cv::Mat> mvImagePyramid; // 图像金字塔

    protected:
        // The following variables need to be accessed trough a mutex to be thread safe.
        // data used in imu
        std::mutex mMutexPrevKF;    // 线程安全 数据锁=======
        std::mutex mMutexNextKF;
        KeyFrame *mpPrevKeyFrame;
        KeyFrame *mpNextKeyFrame;

        // P, V, R, bg, ba, delta_bg, delta_ba (delta_bx is for optimization update)
        std::mutex mMutexNavState; // IMU 导航状态 数据========
        NavState mNavState;

        // IMU Data from lask KeyFrame to this KeyFrame
        std::mutex mMutexIMUData;
        std::vector<IMUData> mvIMUData; // 上一帧到本帧间获取的 IMU数据
        IMUPreintegrator mIMUPreInt;

        // SE3 Pose and camera center
        SE3f Tcw; // 世界坐标系 到 相机坐标系 位姿
        SE3f Twc;// 相机坐标系 到 世界坐标系
        Vector3f Ow;// 相机中心 
        Vector3f Cw; // Stereo middel point. Only for visualization

        // MapPoints associated to keypoints
        std::vector<MapPoint *> mvpMapPoints;// 关键点匹配的 地图点================

        // BoW 特征词典 ======================
        KeyFrameDatabase *mpKeyFrameDB;
        ORBVocabulary *mpORBvocabulary;

        // Grid over the image to speed up feature matching
       // 格点 64*48个容器 每个容器内是一个容器 存储着 关键点 的 id
        std::vector<std::vector<std::vector<size_t> > > mGrid; // 三维数值
       /// 外层 64 中层 48 内层 多个关键点id 

        std::map<KeyFrame *, int> mConnectedKeyFrameWeights; 
       ///< 与该关键帧连接的关键帧与权重
        std::vector<KeyFrame *> mvpOrderedConnectedKeyFrames; 
        ///< 按权值排序后的邻居关键帧==========================邻居帧关系好坏========
        std::vector<int> mvOrderedWeights; ///< 排序后的权重(从大到小)

        // Spanning Tree and Loop Edges  生成树============================
        // std::set是集合，相比vector，进行插入数据这样的操作时会自动排序(有序集合)
        bool mbFirstConnection = false;
        KeyFrame *mpParent = nullptr;
        std::set<KeyFrame *> mspChildrens;// 局部 关联，下游关键帧======
        std::set<KeyFrame *> mspLoopEdges;// 全局 关联， 远房亲戚======

        // Bad flags  不好的标志===============
        bool mbNotErase = false;
        bool mbToBeErased = false;
        bool mbBad = false;

        float mHalfBaseline =0; // 双目半基线  Only for visualization

        Map *mpMap =nullptr;

        std::mutex mMutexPose; // 位姿 数据锁=========
        std::mutex mMutexConnections;// 连接关系锁====
        std::mutex mMutexFeatures;// 特征锁===========

    };

} //namespace ygz

#endif // KEYFRAME_H
