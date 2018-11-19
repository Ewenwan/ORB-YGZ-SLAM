/**
* This file is part of ORB-SLAM2.
* 普通帧 每一幅 图像都会生成 一个帧
* 
* 
*/

#ifndef YGZ_FRAME_H_
#define YGZ_FRAME_H_

// ORB 里的帧

#include "Thirdparty/DBoW2/DBoW2/BowVector.h"
#include "Thirdparty/DBoW2/DBoW2/FeatureVector.h"
#include "ORBVocabulary.h"
#include "ORBextractor.h"

#include "IMU/imudata.h"
#include "IMU/NavState.h"
#include "IMU/IMUPreintegrator.h"

namespace ygz {

    // 格点大小，这个最好能随着图像大小做一些改变
    // 640 *480 的图像  分成   64*48 个 10×10的网格
// // 这是grid，将图像分成格子，保证提取的特征点比较均匀
    #define FRAME_GRID_ROWS 48
    #define FRAME_GRID_COLS 64

    class MapPoint;
    class KeyFrame;
    class ORBextractor;

    class Frame {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
// 支持的传感器类型放在这边了==================
        typedef enum
      {
            Monocular = 0, Stereo, RGBD // 0,1,2 枚举变量
        } SensorType;

    public:

        Frame();// 构造函数

        // Copy constructor. 拷贝构造函数
        Frame(const Frame &frame);

        // Constructor for stereo cameras.
       // 双目相机帧 构造函数=================================
        Frame(const cv::Mat &imLeft,    // 左图 
                      const cv::Mat &imRight, // 右图 
                      const double &timeStamp, // 时间戳
                      ORBextractor *extractorLeft,// 左图特征提取器
                      ORBextractor *extractorRight, // 右图特征提取器
                      ORBVocabulary *voc,                   // 特征词典
                      Matrix3f &K,                                    // 相机内参数K
                      cv::Mat &distCoef,                        // 相机畸变参数
                      const float &bf,                          // 双目相机基线
                      const float &thDepth);           //  ??

        // Constructor for RGB-D cameras.   RGB-D相机帧===========
        Frame(const cv::Mat &imGray,     // 灰度图
                     const cv::Mat &imDepth,   // 深度图 
                     const double &timeStamp, 
                     ORBextractor *extractor,
                     ORBVocabulary *voc, 
                     Matrix3f &K, 
                     cv::Mat &distCoef, 
                     const float &bf, 
                     const float &thDepth);

        // Constructor for Monocular cameras.   单目相机帧===========
        Frame(const cv::Mat &imGray,
                      const double &timeStamp, 
                      ORBextractor *extractor, 
                      ORBVocabulary *voc, Matrix3f &K,
                      cv::Mat &distCoef, 
                      const float &bf, 
                      const float &thDepth);

        // Constructor for Monocular VI   单目-IMU帧 ===========================new=====
        Frame(const cv::Mat &imGray,                               // 灰度图
                      const double &timeStamp,                      // 时间戳
                      const std::vector<IMUData> &vimu,     // imu数据
                      ORBextractor *extractor,
                      ORBVocabulary *voc,
                      Matrix3f &K, 
                      cv::Mat &distCoef, 
                      const float &bf,
                      const float &thDepth,
                     KeyFrame *pLastKF = NULL);

        // 提取特征的所有步骤====================new===========
        void ExtractFeatures();

        // Extract ORB on the image. 0 for left image and 1 for right image.
        // 提取的关键点存放在mvKeys ,描述子放在 mDescriptors 中
        // ORB是直接调 orbExtractor提取的
        void ExtractORB(int flag, const cv::Mat &im);

        // 计算图像金字塔========================new============
        void ComputeImagePyramid();

        // Compute Bag of Words representation.
        // 存放在mBowVec中，  计算 特征描述子 的 特征字典表示向量=========
        void ComputeBoW();

        // Set the camera pose. 设置  相机位姿==============输入参数类型变=======
        void SetPose(const SE3f &Tcw);

        // Computes rotation, translation and camera center matrices from the camera pose.
        void UpdatePoseMatrices();//  更新相机描述矩阵 旋转 平移 中心点============

        // Returns the camera center.   返回 世界坐标系下相机中心点 ================
        inline Vector3f GetCameraCenter() {
            return mOw;
        }

        // Returns inverse of rotation  返回 世界坐标系转到相机坐标系下 的 旋转矩阵R 
        inline Matrix3f GetRotationInverse() {
            return mRwc;
        }

        // Check if a MapPoint is in the frustum of the camera
        // and fill variables of the MapPoint to be used by the tracking
        // 判断路标点是否在视野中，反投影过来
        // 会检测像素坐标、距离和视线角=======================================
        bool isInFrustum(MapPoint *pMP, float viewingCosLimit);

        // Compute the cell of a keypoint (return false if outside the grid)
        // 检测某个关键点 是否 在某个网格内
        bool PosInGrid(const cv::KeyPoint &kp, int &posX, int &posY);

        // 获取一定区域内的点，需要事先用AssignFeaturesToGrid指定填充网格
        vector<size_t> GetFeaturesInArea(const float &x, 
                                         const float &y, 
                                         const float &r, 
                                         const int minLevel = -1,
                                         const int maxLevel = -1) const;

        // Search a match for each keypoint in the left image to a keypoint in the right image.
        // If there is a match, depth is computed and the right coordinate associated to the left keypoint is stored.
        // 左右目之间的双目匹配。先用ORB特征匹配，再进行微调========================
        // 需要双目已经做了Rectify,极线水平，误差允许在+-2像素之间
        // 计算特征匹配点对 根据视差计算深度
        void ComputeStereoMatches();

        // Associate a "right" coordinate to a keypoint if there is valid depth in the depthmap.
        // 通过 RGBD 数据提供 伪双目数据，便于统一接口================================
        void ComputeStereoFromRGBD(const cv::Mat &imDepth);

        // Backprojects a keypoint (if stereo/depth info available) into 3D world coordinates.
        // 计算第i个特征点 在相机系的坐标，第i关键点 映射到 3D 相机坐标系下====================
        Vector3f UnprojectStereo(const int &i);

        // Computes image bounds for the undistorted image (called in the constructor).
        // 计算一些图像的边界(在ygz中不使用，因为默认图像已经是缩放过的)
        void ComputeImageBounds(const cv::Mat &imLeft);

        // Assign keypoints to the grid for speed up feature matching (called in the constructor).
        void AssignFeaturesToGrid(); // 分配 特征点到 划分的网格内 加速匹配==================



// 坐标变换=======================================================================
        // coordinate transform: world, camera, pixel
// 世界坐标系下的点转到 相机坐标系下=====
        inline Vector3f World2Camera(const Vector3f &p_w, const SE3f &T_c_w) const {
            return T_c_w * p_w;
        }
// 相机坐标系下 3d 点 转 世界坐标系 ======
        inline Vector3f Camera2World(const Vector3f &p_c, const SE3f &T_c_w) const {
            return T_c_w.inverse() * p_c;
        }
 // 相机坐标系下 3d点 转换到 图像坐标系下 2d像素点=====
        inline Vector2f Camera2Pixel(const Vector3f &p_c) const {
            return Vector2f(
                    fx * p_c(0, 0) / p_c(2, 0) + cx,
                    fy * p_c(1, 0) / p_c(2, 0) + cy
            );
        }
// 图像坐标系下 2d像素点 转 相机坐标系下 3d点 ========
        inline Vector3f Pixel2Camera(const Vector2f &p_p, double depth = 1) const {
            return Vector3f(
                    (p_p(0, 0) - cx) * depth / fx,
                    (p_p(1, 0) - cy) * depth / fy,
                    depth
            );
        }
// 图像坐标系下 2d像素点 转 相机坐标系下 3d点 再转 世界坐标系 ========
        inline Vector3f Pixel2World(const Vector2f &p_p, const SE3f &T_c_w, double depth = 1) const {
            return Camera2World(Pixel2Camera(p_p, depth), T_c_w);
        }
// 世界坐标系下的点转到 相机坐标系下 再转到 图像坐标系下 2d像素点=====
        Vector2f World2Pixel(const Vector3f &p_w, const SE3f &T_c_w) const {
            return Camera2Pixel(World2Camera(p_w, T_c_w));
        }


// 惯性传感器IMU 的更新==================================================

        // IMU related
        // compute the imu preintegration   IMU预积分  =========================
        void ComputeIMUPreIntSinceLastFrame(const Frame *pLastF,
                                                                                           IMUPreintegrator &imupreint) const;

        // update the pose matrix from navigation state  从导航状态更新 位姿矩阵=====
        void UpdatePoseFromNS(const SE3d &Tbc);

        // update the nav state from camera pose 相机位姿 矩阵 更新 导航状态=======
        void UpdateNSFromPose();

        // set initial navigation, and set bias to zero
        void SetInitialNavStateAndBias(const NavState &ns);// 初始化imu导航状态 和 偏置====

        // update the navigation status using preintegration   imu 预积分 更新导航状态========
        void UpdateNavState(const IMUPreintegrator &imupreint, 
                                                    const Vector3d &gw);

        // get navigation state  获取imu 导航状态======
        NavState GetNavState(void) const {
            return mNavState;
        }

        // set navigation state 设置imu 导航状态======
        void SetNavState(const NavState &ns) {
            mNavState = ns;
        }

        // set gyro bias 设置imu 的陀螺仪偏置======
        void SetNavStateBiasGyr(const Vector3d &bg);

        // set accelerate bias    设置加速度偏置======
        void SetNavStateBiasAcc(const Vector3d &ba);


    public:
        // Vocabulary used for relocalization.
 // 重定位 orb 词典 支持将 描述子 转成 用 单词线性表示的 向量
        ORBVocabulary *mpORBvocabulary = nullptr;     // 字典

        // Feature extractor. The right is used only in the stereo case.
        ORBextractor *mpORBextractorLeft = nullptr;       // ORB特征提取器
        ORBextractor *mpORBextractorRight = nullptr;     // 两个图像的特征提取器

        cv::Mat mImGray;    // 灰度图/双目左图
        cv::Mat mImRight;   // 双目右图
        cv::Mat mImDepth;   // RGBD深度图

        // 图像金字塔
        vector<cv::Mat> mvImagePyramid;

        // Frame timestamp. 帧时间戳 来自 原始拍摄图像的 时间戳
        double mTimeStamp = -1;

        // Calibration matrix and OpenCV distortion parameters.
        //  相机内参数 K
        Matrix3f mK = Matrix3f::Zero();
        static float fx;
        static float fy;
        static float cx;
        static float cy;
        static float invfx; // 1/fx
        static float invfy; // 1/fy

        static Mat map1, map2;  // for undistortion 双目矫正
        cv::Mat mDistCoef;             // 畸变校正参数 k1 k2 p1 p2 k3
        static bool mbNeedUndistort;  // 如果传进来的图像是去过畸变的，就没必要再做一次

        // Stereo baseline multiplied by fx.
        float mbf = 0;       // 双目中的基线乘焦距
       // 除以视差 得到深度信息
       //  z = bf /d      b 双目相机基线长度  f为焦距  d为视差(同一点在两相机像素平面 水平方向像素单位差值)
      
        // Stereo baseline in meters.
        float mb;                   //  基线长度  单位米

        // Threshold close/far points. Close points are inserted from 1 view.
        // Far points are inserted as in the monocular case from 2 views.
        float mThDepth;             // 远点的阈值  深度阈值  一般为相机有效最大距离

        // Number of KeyPoints.
        int N; ///< KeyPoints数量 关键点数量

        // Vector of keypoints (original for visualization) and undistorted (actually used by the system).
        // In the stereo case, mvKeysUn is redundant as images must be rectified.
        // In the RGB-D case, RGB images can be distorted.
        // mvKeys:原始左图像提取出的特征点
        // mvKeysRight:原始右图像提取出的特征点
        std::vector<cv::KeyPoint> mvKeys, mvKeysRight;
//   std::vector<cv::KeyPoint> mvKeysUn;//校正后的关键点

        vector<int> mvMatchedFrom;  // 标识每个特征点是从哪个Keyframe选取的=========

        // 是否已经提取了特征
        bool mbFeatureExtracted = false;    // flag to indicate if the ORB features are detected

        // Corresponding stereo coordinate and depth for each keypoint.
        // "Monocular" keypoints have a negative value.
        // 对于双目，mvuRight存储了左目像素点 在 右目中 的 对应点的横坐标
        // mvDepth对应的深度值
        // 单目摄像头，这两个容器中存的都是-1
        std::vector<float> mvuRight;
        std::vector<float> mvDepth;

        // Bag of Words Vector structures.
        DBoW2::BowVector mBowVec;// 词带向量
        DBoW2::FeatureVector mFeatVec;// 特征向量

        // ORB descriptor, each row associated to a keypoint.
        // 左目摄像头 和 右目摄像 特征点对应的描述子（矩阵形式，第i行表示第i个特征）
        cv::Mat mDescriptors, mDescriptorsRight;

        // MapPoints associated to keypoints, NULL pointer if no association.
        // 每个特征点 对应的 MapPoint 地图点======
        std::vector<MapPoint *> mvpMapPoints;

        // Flag to identify outlier associations.
        // 特征点 观测不到 地图Map中的 3D点
        std::vector<bool> mvbOutlier;

        // Keypoints are assigned to cells in a grid to reduce matching complexity when projecting MapPoints.
        // Grid的倒数,用来快速确认Grid的位置
        static float mfGridElementWidthInv;
        static float mfGridElementHeightInv;

        // 这是grid，将图像分成格子，保证提取的特征点比较均匀
        // #define FRAME_GRID_ROWS 48
        // #define FRAME_GRID_COLS 64
        std::vector<std::size_t> mGrid[FRAME_GRID_COLS][FRAME_GRID_ROWS];
        bool mbGridSet = false;       // Grid是否已经被设置

        // Camera pose.  相机位姿
        SE3f mTcw;  // World 到 Camera  世界坐标系 到 相机坐标系=========
        bool mbPoseSet = false;      // Pose 是否已经设置

        // Current and Next Frame id.
        static long unsigned int nNextId; ///< Next Frame id. 下一帧 id
        long unsigned int mnId = 0; ///< Current Frame id.        当前帧id

        // Reference Keyframe.
        KeyFrame *mpReferenceKF = nullptr;//指针，指向当前帧 的 参考关键帧

        // Scale pyramid info.  图像金字塔参数================
        int mnScaleLevels = 0;// 图像提金字塔的 层数
        float mfScaleFactor = 0;// 图像提金字塔的 尺度因子
        float mfLogScaleFactor; // 对数scale缩放值

        // 各层金字塔的参数
        vector<float> mvScaleFactors;               // 缩放倍数
        vector<float> mvInvScaleFactors;         // 缩放倍数倒数
        vector<float> mvLevelSigma2;                // 平方
        vector<float> mvInvLevelSigma2;          // 倒数平方

        // Undistorted Image Bounds (computed once).
        // 用于确定画格子时的边界
        static float mnMinX;
        static float mnMaxX;
        static float mnMinY;
        static float mnMaxY;

        static bool mbInitialComputations;

        // IMU Data from last Frame to this Frame
// 上一帧 到 这一帧 期间 获取的 IMU数据======================
        std::vector<IMUData> mvIMUDataSinceLastFrame;

        // For pose optimization, use as prior and prior information(inverse covariance)
        // 用IMU确定出来的先验
        Matrix<double, 15, 15> mMargCovInv = Matrix<double, 15, 15>::Zero();
        NavState mNavStatePrior;// 导航状态先验值

        // 传感器类型
        SensorType mSensor;

        // Rotation, translation and camera center
        // 相机的一些量，都可以从 Tcw 里推导出来
        Matrix3f mRcw; ///旋转矩阵 世界坐标系 到  相机坐标系  Rotation from world to camera
        Vector3f mtcw; ///平移向量 世界坐标系 到  相机坐标系 Translation from world to camera
        Matrix3f mRwc; ///旋转矩阵  相机坐标系 到 世界坐标系< Rotation from camera to world
        Vector3f mOw; //世界坐标系下相机中心点 ==mtwc,Translation from camera to world

        // IMU 用的量，Rwb, twb, v, ba, bg,共15维，存储在NavState当中
// 3d位置 3d姿态 3d速度 3d加速度偏置 3d角速度偏置=====3×5=15====
        NavState mNavState; // Navigation state used in VIO
    };

}// namespace ygz

#endif // FRAME_H
