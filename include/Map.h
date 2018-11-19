/**
* This file is part of ORB-SLAM2.
* 地图 管理 关键帧 地图点
*  添加/删除  关键帧/地图点
* 
*/

#ifndef YGZ_MAP_H_
#define YGZ_MAP_H_

#include "Common.h"


// 地图
namespace ygz {

    class MapPoint;
    class KeyFrame;

    // Sort KeyFrames with mnId in mspKeyFrames
    class KFIdComapre {
    public: // 关键帧比较==== 供给
        bool operator()(const KeyFrame *kfleft, const KeyFrame *kfright) const;
    };

    class Map {
    public:
        Map();

        void AddKeyFrame(KeyFrame *pKF);// 添加关键帧
        void AddMapPoint(MapPoint *pMP);// 添加地图点
        void EraseMapPoint(MapPoint *pMP);// 删除地图点
        void EraseKeyFrame(KeyFrame *pKF);//删除关键帧
        void SetReferenceMapPoints(const std::vector<MapPoint *> &vpMPs);// 参考地图点
        void InformNewBigChange();// 地图数据 大变化  回环修正
        int GetLastBigChangeIdx();// 上次地图数据大变化 id

        std::vector<KeyFrame *> GetAllKeyFrames();// 得到所有关键帧
        std::vector<MapPoint *> GetAllMapPoints();// 得到所有地图点
        std::vector<MapPoint *> GetReferenceMapPoints();// 参考地图点

        long unsigned int MapPointsInMap();// 地图中地图点的数量
        long unsigned KeyFramesInMap();// 关键帧数量
        long unsigned int GetMaxKFid();// 最大 关键帧id

        void clear();// 清空地图==============

        vector<KeyFrame *> mvpKeyFrameOrigins;// 原关键帧===

        std::mutex mMutexMapUpdate;// 地图更新锁=====

        // This avoid that two points are created simultaneously in separate threads (id conflict)
        std::mutex mMutexPointCreation;// 地图点构建锁

    protected:
        std::set<MapPoint *> mspMapPoints;//所有地图点
        std::set<KeyFrame *, KFIdComapre> mspKeyFrames;//所有关键帧

        std::vector<MapPoint *> mvpReferenceMapPoints;//所有参考地图点

        long unsigned int mnMaxKFid;

        // Index related to a big change in the map (loop closure, global BA)
        int mnBigChangeIdx;//   回环修正 导致地图发生大的变换

        std::mutex mMutexMap;
    };

} //namespace ygz

#endif // MAP_H
