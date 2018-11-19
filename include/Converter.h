/**
* This file is part of ORB-SLAM2.
  加入 imu 部分=============各种数据类型转换
*/

#ifndef YGZ_CONVERTER_H_
#define YGZ_CONVERTER_H_

#include "Common.h"
#include "IMU/NavState.h"
#include "IMU/imudata.h"
#include "IMU/IMUPreintegrator.h"

// ORB里用的一些Convert函数，虽然我自己不用，但是有一些ORB里的旧代码用了，我也懒得改。。

namespace ygz {

    class Converter 
{
    public:
        // 矩阵形式的描述转向量形式的描述
        static std::vector<cv::Mat> toDescriptorVector(const cv::Mat &Descriptors);

        // 4*4 mat形式的 位姿态  R t 转化成 G2O 6自由度顶点优化变量类型  李代数形式 SE3Quat  
        static g2o::SE3Quat toSE3Quat(const cv::Mat &cvT);

        /** unimplemented */
        static g2o::SE3Quat toSE3Quat(const g2o::Sim3 &gSim3);// 李代数形式

      // g2o 类型转Mat=======
        static cv::Mat toCvMat(const g2o::SE3Quat &SE3);
        static cv::Mat toCvMat(const g2o::Sim3 &Sim3);

       // Eigen 转 Mat========
        static cv::Mat toCvMat(const Eigen::Matrix<double, 4, 4> &m);
        static cv::Mat toCvMat(const Eigen::Matrix<float, 4, 4> &m);
        static cv::Mat toCvMat(const Eigen::Matrix3d &m);
        static cv::Mat toCvMat(const Eigen::Matrix3f &m);
        static cv::Mat toCvMat(const Eigen::Matrix<double, 3, 1> &m);
        static cv::Mat toCvMat(const Eigen::Matrix<float, 3, 1> &m);
        static cv::Mat toCvSE3(const Eigen::Matrix<double, 3, 3> &R,
                                                    const Eigen::Matrix<double, 3, 1> &t);

        // Mat 转 Eigen======
        static Eigen::Matrix<double, 3, 1>  toVector3d(const cv::Mat &cvVector);
        static Eigen::Matrix<double, 3, 1>  toVector3d(const cv::Point3f &cvPoint);
        static Eigen::Matrix<double, 3, 3>  toMatrix3d(const cv::Mat &cvMat3);
       // 转四元素==========
        static std::vector<float> toQuaternion(const cv::Mat &M);

        // 从IMU integration 数据转换为 NaviState
        static void updateNS(NavState &ns, 
                                                     const IMUPreintegrator &imupreint, 
                                                     const Vector3d &gw);
        // 各种其他矩阵转cv::Mat
        // 讲道理 矩阵干脆都用eigen存就行了
        static cv::Mat toCvMat(const SE3d &SE3);
        static cv::Mat toCvMat(const SE3f &SE3);

    };

}// namespace ygz

#endif // CONVERTER_H
