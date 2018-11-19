/**
* This file is part of ORB-SLAM2.
 * 关键帧数据库
 * 存储所有关键帧

*/

#ifndef YGZ_KEYFRAMEDATABASE_H
#define YGZ_KEYFRAMEDATABASE_H

#include "ORBVocabulary.h"
#include "Common.h"

// Keyframe 数据库，基本没动
namespace ygz {

    class KeyFrame;
    class Frame;


    class KeyFrameDatabase {
    public:

        KeyFrameDatabase(const ORBVocabulary &voc);// 使用 预先训练好的特征词典 初始化关键帧数据库

        void add(KeyFrame *pKF);   // 添加关键帧 指针

        void erase(KeyFrame *pKF); // 删除关键帧

        void clear();              // 清空关键帧数据库======

        // Loop Detection  闭环 候选帧  
        std::vector<KeyFrame *> DetectLoopCandidates(KeyFrame *pKF, float minScore);

        // Relocalization  跟丢 重定位 候选帧 
        std::vector<KeyFrame *> DetectRelocalizationCandidates(Frame *F);

    protected:

        // Associated vocabulary
        const ORBVocabulary *mpVoc; ///< 预先训练好的词典

        // Inverted file
        std::vector<list < KeyFrame * > >    mvInvertedFile; // 链表数组
///< 倒排索引，mvInvertedFile[i] 表示包含了第i个word id的所有 关键帧

        // Mutex
        std::mutex mMutex;
    };

} //namespace ygz

#endif
