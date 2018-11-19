/**
* This file is part of ORB-SLAM2.
*  单目相机初始化
*  基础矩阵F(随机采样序列 8点法求解) 和 单应矩阵计算( 采用归一化的直接线性变换（normalized DLT）)  相机运动
* 用于平面场景的单应性矩阵H和用于非平面场景的基础矩阵F，
* 然后通过一个评分规则来选择合适的模型，恢复相机的旋转矩阵R和平移向量t。
* 
*/
#ifndef INITIALIZER_H
#define INITIALIZER_H

#include "Frame.h"

// 初始化相关
// THIS IS THE INITIALIZER FOR MONOCULAR SLAM. NOT USED IN THE STEREO OR RGBD CASE.
// Change the cv::Mat matrix operations into eigen to accelerate
// NOTE ORB里对初始化的检查颇多，实际当中由于后端有BA，事实上并没有太多必要
namespace ygz {

    /**
     * @brief 单目SLAM初始化相关，双目和RGBD不会使用这个类
     */
    class Initializer {
        typedef pair<int, int> Match;// pair  键值对 

    public:

        // Fix the reference frame  固定参考帧
        // 用reference frame来初始化，这个reference frame就是SLAM正式开始的第一帧
        Initializer(const Frame &ReferenceFrame, float sigma = 1.0, int iterations = 200);


    // 单目初始化 两种方法========================================
    // 计算 基础矩阵 F 和 单应矩阵 H       2D-2D点对映射关系
    // Xc = H * Xr               p2转置 * F * p1 = 0
    // 选择一种方法 恢复 运动
        // 用current frame,也就是用SLAM逻辑上的第二帧来初始化整个SLAM，
        // 得到最开始两帧之间的R t, 以及3d点云
        bool Initialize(const Frame &CurrentFrame, 
                                     const vector<int> &vMatches12,
                                     Matrix3f &R21, 
                                     Vector3f &t21, 
                                     vector<Vector3f> &vP3D, 
                                     vector<bool> &vbTriangulated);

    private:

         // 假设场景为 平面情况下通过 前两帧求取Homography矩阵
         // (current frame 2 到 reference frame 1),并得到该模型的评分
         //  随机采样8点对 调用 ComputeH21计算单应，
         //  CheckHomography 计算得分  迭代求 得分最高的 H
        void FindHomography(vector<bool> &vbMatchesInliers, float &score, Matrix3f &H21);

        // 假设场景为非平面情况下通过前两帧求取Fundamental矩阵
        // (current frame 2 到 reference frame 1),并得到该模型的评分
        // 随机采样8点对 调用 ComputeF21计算基础矩阵 
        //  CheckFundamental 计算得分 迭代求 得分最高的 F
        void FindFundamental(vector<bool> &vbInliers, float &score, Matrix3f &F21);

        // 被FindHomography函数调用具体来算 单应Homography矩阵
        Matrix3f ComputeH21(const vector<Vector2f> &vP1, const vector<Vector2f> &vP2);

        // 被FindFundamental函数调用具体来算 基础Fundamental矩阵
        Matrix3f ComputeF21(const vector<Vector2f> &vP1, const vector<Vector2f> &vP2);

        // 被 FindHomography 函数调用，具体来算 假设使用Homography模型 的 得分
        float CheckHomography(const Matrix3f &H21, const Matrix3f &H12, 
                                                           vector<bool> &vbMatchesInliers, float sigma);

        // 被FindFundamental函数调用，具体来算假设使用Fundamental模型的得分
        float CheckFundamental(const Matrix3f &F21, 
                                                            vector<bool> &vbMatchesInliers, float sigma);

        // 分解F矩阵，并从分解后的多个解中找出合适的R，t
        // -F ----> 本质矩阵E 从本质矩阵恢复  R  t
        bool ReconstructF(vector<bool> &vbMatchesInliers, Matrix3f &F21, Matrix3f &K,
                                               Matrix3f &R21, Vector3f &t21, vector<Vector3f> &vP3D, 
                                               vector<bool> &vbTriangulated,
                                               float minParallax, int minTriangulated);

        // 分解H矩阵，并从分解后的多个解中找出合适的R，t
        bool ReconstructH(vector<bool> &vbMatchesInliers, Matrix3f &H21, Matrix3f &K,
                                                Matrix3f &R21, Vector3f &t21, vector<Vector3f> &vP3D, 
                                                vector<bool> &vbTriangulated,
                                                float minParallax, int minTriangulated);

        // 通过三角化方法，利用 反投影矩阵 将 特征点 恢复 为 3D点
        void Triangulate(
                const Vector2f &kp1, const Vector2f &kp2,
                const Eigen::Matrix<float, int(3), int(4)> &P1,
                const Eigen::Matrix<float, int(3), int(4)> &P2,
                Vector3f &x3D);
 // 三角化计算 深度 获取3D点坐标
    /*
 平面二维点摄影矩阵到三维点  P1 = K × [I 0]    P2 = K * [R  t]
  kp1 = P1 * p3dC1       p3dC1  特征点匹配对 对应的 世界3维点
  kp2 = P2 * p3dC1  
  kp1 叉乘  P1 * p3dC1 =0
  kp2 叉乘  P2 * p3dC1 =0  
 p = ( x,y,1)
 其叉乘矩阵为
     //  叉乘矩阵 = [0  -1  y;
    //                       1   0  -x; 
    //                      -y   x  0 ]  
  一个方程得到两个约束
  对于第一行 0  -1  y; 会与P的三行分别相乘 得到四个值 与齐次3d点坐标相乘得到 0
  有 (y * P.row(2) - P.row(1) ) * D =0
      (-x *P.row(2) + P.row(0) ) * D =0 ===> (x *P.row(2) - P.row(0) ) * D =0
    两个方程得到 4个约束
    A × D = 0
    对A进行奇异值分解 求解线性方程 得到 D  （D是3维齐次坐标，需要除以第四个尺度因子 归一化）
 */

	// 归一化三维空间点和帧间位移t
	// 标准化点坐标
	// 标准化矩阵  * 点坐标    =   标准化后的的坐标              去均值点坐标 * 绝对矩倒数
	//  点坐标    =    标准化矩阵 逆矩阵 * 标准化后的的坐标
        void Normalize(const vector<cv::KeyPoint> &vKeys, 
                                         vector<Vector2f> &vNormalizedPoints, Matrix3f &T);

        // ReconstructF调用该函数进行cheirality check，从而进一步找出F分解后最合适的解
        int CheckRT(const Matrix3f &R, const Vector3f &t, 
                                  const vector<cv::KeyPoint> &vKeys1,
                                  const vector<cv::KeyPoint> &vKeys2,
                                  const vector<Match> &vMatches12, vector<bool> &vbInliers,
                                  const Matrix3f &K, vector<Vector3f> &vP3D, float th2, 
                                 vector<bool> &vbGood, float &parallax);

	// F矩阵通过结合内参可以得到Essential矩阵，该函数用于分解E矩阵，将得到4组解
	// 从本质矩阵 恢复 R t
	// E = t^R = U C  V   ,U   V 为正交矩阵   C 为奇异值矩阵 C =  diag(1, 1, 0)
        void DecomposeE(const Matrix3f &E, Matrix3f &R1, Matrix3f &R2, Vector3f &t);


        // Keypoints from Reference Frame (Frame 1)
        vector<cv::KeyPoint> mvKeys1; ///< 存储Reference Frame中的特征点

        // Keypoints from Current Frame (Frame 2)
        vector<cv::KeyPoint> mvKeys2; ///< 存储Current Frame中的特征点

        // Current Matches from Reference to Current
        // Reference Frame: 1, Current Frame: 2
        vector<Match> mvMatches12; ///< Match的数据结构是 pair<int, int>  数对 
                                              /// mvMatches12 只记录Reference到Current匹配上的特征点对
        vector<bool> mvbMatched1; 
                     ///< 记录Reference Frame的每个特征点在Current Frame是否有匹配的特征点

        // Calibration 矫正相关参数=================================
        Matrix3f mK; ///< 相机内参

        // Standard Deviation and Variance
        float mSigma, mSigma2; ///< 测量误差

        // Ransac max iterations   随机采样序列 最大迭代次数
        int mMaxIterations; ///< 算Fundamental和Homography矩阵时RANSAC迭代次数
       //  基础矩阵F(随机采样序列 8点法求解) 和 单应矩阵计算(随机采样序列 4点法求解)  相机运动
   
        // Ransac sets   随机点对序列
        vector<vector<size_t> > mvSets;
        ///< 二维容器，外层容器的大小为迭代次数，内层容器大小为每次迭代算H或F矩阵需要的点

    };

} //namespace ygz

#endif // INITIALIZER_H
