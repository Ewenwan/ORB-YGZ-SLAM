#ifndef YGZ_COMMON_H_
#define YGZ_COMMON_H_
 
// 常用的一些头文件  =================

// std  ==========================================
#include <vector> // 动态数值 
#include <list>   // 链表
#include <memory> // 提供了内存操作相关的一些函数及声明
#include <string> // 字符串
#include <iostream>// 输入输出 流
#include <set>     // 集合
#include <unordered_map>// 无序字典 哈希函数
#include <map>          // 有序字典 红黑二叉树
#include <algorithm>    // 算法
#include <mutex>        // 互斥锁
#include <thread>       // 线程

using namespace std;    // 命名空间

// for Eigen  矩阵运算库================================
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/StdVector> // for vector of Eigen objects 
#include <Eigen/Dense>     // linear algebra

using Eigen::Vector2d;// ================= =Vector2f
using Eigen::Vector3d;
using Eigen::Vector2f;// ====向量
using Eigen::Vector3f;
using Eigen::Vector4f;
using Eigen::Matrix2f;//===矩阵
using Eigen::Matrix3f;
using Eigen::Matrix4f;
using Eigen::Matrix2d;
using Eigen::Matrix3d;
using Eigen::Matrix4d;// 4*4
using Eigen::Quaternionf;// 四元素
// other things I need in optimiztion 
typedef Eigen::Matrix<double, 6, 1> Vector6d; // 6维 向量

// for Sophus  李代数 ===================================
#include <Thirdparty/sophus/sophus/se3.hpp>// 李代数
#include <Thirdparty/sophus/sophus/so3.hpp>// 李群

using Sophus::SO3f;
using Sophus::SE3f;
using Sophus::SO3d;
using Sophus::SE3d;

// for g2o 图优化 ========================================
#include "Thirdparty/g2o/g2o/types/types_six_dof_expmap.h" // 6自由度
#include "Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"// 7自由度
#include "Thirdparty/g2o/g2o/types/se3quat.h"               // SE3Quat类型表示 变换T 4×4  
//#include "Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"// 重复了======

// for cv opencv 图像处理 ================================
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>

using cv::Mat;

// pangolin  可视化 =======================================
#include <pangolin/pangolin.h>

// glog  google 日志 ======================================
#include <glog/logging.h>

#endif
