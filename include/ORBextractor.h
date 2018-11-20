/**
* This file is part of ORB-SLAM2.
*  扩展特征点类型 原版 ORB-SLAM、SVO里的网格FAST、类DSO的变网格FAST ======
*作者对opencv中的orb源码进行了修改，将特征进行四叉树均匀化
*  2d 上下左右四点 四叉 
* 特征提取也就是对图像进行一定的操作，也就是对像素点进行一些操作，
* 跟相邻的一些像素点进行比较，通过一些模板进行滤波卷积等操作，再通过阈值进行一些控制，
* 找到了可以代表该图像的某些位置，这也就是特征提取。 
* 
*/

#ifndef YGZ_ORBEXTRACTOR_H_
#define YGZ_ORBEXTRACTOR_H_

#include "Common.h"

// ORB 提取器

namespace ygz {

    class Frame;

    // ORB里用的四叉树均匀化=============
    class ExtractorNode {
    public:
        ExtractorNode() : bNoMore(false) {}

        void DivideNode(ExtractorNode &n1, ExtractorNode &n2, 
                                            ExtractorNode &n3, ExtractorNode &n4);

        std::vector<cv::KeyPoint> vKeys;
        cv::Point2i UL, UR, BL, BR;
        std::list<ExtractorNode>::iterator lit;
        bool bNoMore;
    };

    class ORBextractor {
    public:

        enum {
            HARRIS_SCORE = 0, FAST_SCORE = 1
        };

        // select the keypoint method, support original ORB-SLAM, Grid FAST in SVO and a dynamic grid FAST (DSO like)
        // 使用哪种方法提取特征点，支持原版ORB、SVO里的网格FAST、
        // 以及类DSO的变网格FAST
        // 相对来说，原版 ORB-SLAM 的特征提取重复性较好，但比较慢。
        // 后两个快一些，但重复性差一些
        typedef enum {
            ORBSLAM_KEYPOINT, FAST_KEYPOINT, DSO_KEYPOINT
        } KeyPointMethod;

        ORBextractor(int nfeatures,             // 特征点总数 =500
                                    float scaleFactor,     // 尺度因子 = 1.2f
                                    int nlevels,                  // 金字塔总层数 = 8
                                    int iniThFAST,            // 快速角点提取阈值大小 =20
                                    int minThFAST);       // 最小的fast角点提取阈值大小  =7 数量不够时启用
// int features_num = 500, float scale_factor = 1.2f, 
// int levels_num = 8,int default_fast_threshold = 20, int min_fast_threshold = 7
// 特征点总数  尺度因子  金字塔总层数  快速角点提取阈值大 小	
// 为了防止用默认阈值fast角点检测检测的特征数过少，
// 添加设置min_fast_threshold最小的fast特征检测阈值，以保证检测的特征数目。


        ~ORBextractor() {}

        // Compute the ORB features and descriptors on an image.
        // ORB are dispersed on the image using an octree.
        // Mask is ignored in the current implementation.
        // 调用接口，给定图像，建立金字塔并计算关键点，接口与OpenCV相容
        void operator()(
                cv::InputArray image, 
                cv::InputArray mask,  // 被忽略====
                std::vector<cv::KeyPoint> &keypoints, // 关键点
                cv::OutputArray descriptors); // 描述子

        // detect features for frame
        // 给某个单独的Frame调用的接口
        void operator()(Frame *frame,
                        std::vector<cv::KeyPoint> &keypoints,
                        cv::OutputArray descriptors,
                        KeyPointMethod method,
                        bool leftEye = true     
// 是否是左眼（如果是双目的左眼，就要考虑现有的特征点，如果是右眼就可以随便提
        );

        int inline GetLevels() {
            return nlevels;// 返回 金字塔层数======
        }

        float inline GetScaleFactor() {
            return scaleFactor;// 返回尺度因子
        }

        std::vector<float> inline GetScaleFactors() {
            return mvScaleFactor;// 返回每一层级的 尺度因子
        }

        std::vector<float> inline GetInverseScaleFactors() {
            return mvInvScaleFactor;// 返回每一层级的尺度因子 的 倒数
        }

        std::vector<float> inline GetScaleSigmaSquares() {
            return mvLevelSigma2;// 
        }

        std::vector<float> inline GetInverseScaleSigmaSquares() {
            return mvInvLevelSigma2; // 平方
        }

// 图像金字塔 每一层 一张图像 大小   H* 1/sc   W*1/sc
        std::vector<cv::Mat> mvImagePyramid;

        void ComputePyramid(cv::Mat image);

    protected:
        // use the original fast lib to compute keypoints, faster than Opencv's implementation
        void ComputeKeyPointsFast(std::vector<std::vector<cv::KeyPoint>> &allKeypoints,
                                  std::vector<cv::KeyPoint> &exist_kps);

	// 通过八叉树的方式计算特征点
        // compute keypoints using orb slam's octree based implementation
        void ComputeKeyPointsOctTree(std::vector<std::vector<cv::KeyPoint> > &allKeypoints);

        // compute keypoints using dso's pixel selector
        // 注意这里计算的allkeypoints都是在当前金字塔分辨率下的，在提描述的时候才会乘scale
        // 但是dso可能会强行选出一些梯度不好的地方，导致align的时候出错
        void
        ComputeKeyPointsDSO(std::vector<std::vector<cv::KeyPoint>> &allKeypoints, 
                                                         std::vector<cv::KeyPoint> &exist_kps);

        // Single level DSO
        // 单层的DSO，只有原始图像分辨率下的特征点
        void ComputeKeyPointsDSOSingleLevel(
                std::vector<cv::KeyPoint> &allKeypoints,
                std::vector<cv::KeyPoint> &exist_kps
        );

        // 将提取的特征点按照四叉树均匀分布======
        std::vector<cv::KeyPoint> DistributeOctTree(const std::vector<cv::KeyPoint> &vToDistributeKeys, 
                                                                                                const int &minX,
                                                                                                const int &maxX,
                                                                                                const int &minY,
                                                                                                const int &maxY,
                                                                                                const int &nFeatures, 
                                                                                                const int &level);

        // 老版 Opencv 的特征点=======================
        void ComputeKeyPointsOld(std::vector<std::vector<cv::KeyPoint> > &allKeypoints);
        std::vector<cv::Point> pattern;

        // Shi-Tomasi 分数，这个分数越高则特征越优先
        float ShiTomasiScore(const cv::Mat &img, const int &u, const int &v) const;


        int nfeatures =0;// 需要的特征点总数
        double scaleFactor =0;// 尺度因子
        int nlevels =0;//金字塔层数
        int iniThFAST =0;//FAST 角点提取 阈值
        int minThFAST =0;//FAST 角点提取 最小 阈值

	// 金字塔共n层，与SIFT不同，每层仅有一副图像；
	//  第s层的尺度为sc= Fator^c，Fator初始尺度(默认为1.2)，原图在第0层；sc0= 1  sc1 = 1.2^1 sc2 =1.2 ^2
	// 每一层图像大小 H* 1/sc   W*1/sc
	// 在每一层上按公式计算需要提取的特征点数n,在本层上按Fast角点响应值排序，提取前2n个特征点，
	// 然后根据Harris   角点响应值排序， 取前n个特征点，作为本层的特征点；


        std::vector<int> mnFeaturesPerLevel;// 每一层 特征点 个数

        std::vector<int> umax;  //用于计算特征方向时，每个v坐标对应最大的u坐标
        std::vector<float> mvScaleFactor;//  每一层级的 尺度因子sc
        std::vector<float> mvInvScaleFactor;// 每一层级的 尺度因子 倒数  1/sc   是乘以 原图像尺寸的
        std::vector<float> mvLevelSigma2;//  尺度因子的 平方
        std::vector<float> mvInvLevelSigma2; // 尺度因子 平方 的 倒数

        // grid fast====================================================
        /// 注意这里网格大小对特征重复性影响非常明显，一定不要调得太大！
        /// 特征重复性会直接影响到新地图点的生成。
        /// 在5的时候可以生成大约100+个点，
        /// 10的时候就只有20-50个点了,20时一般为个位数
        /// 然而网格太小则使得地图点过多，影响性能
        const int mnCellSize = 5;   // fixed grid size
        int mnGridCols =0;
        int mnGridRows =0;
        std::vector<bool> mvbGridOccupancy;

        cv::Mat map1, map2; // for distortion
        cv::Mat mOccupancy;

        int mnGridSize;     // dynamic grid size used in DSO
    };

} //namespace ygz

#endif

