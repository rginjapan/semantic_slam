//
// Created by zss on 18-4-18.
//
#include "Map.h"
namespace myslam
{
    Map::Map():mnMaxKFid(0),mnBigChangeIdx(0)
    {
    }

    void Map::AddKeyFrame(KeyFrame *pKF)
    {
        unique_lock<mutex> lock(mMutexMap);
        mspKeyFrames.insert(pKF);
        if(pKF->mnId>mnMaxKFid)
            mnMaxKFid=pKF->mnId;
    }

    void Map::AddMapPoint(MapPoint *pMP)
    {
        unique_lock<mutex> lock(mMutexMap);
        mspMapPoints.insert(pMP);
    }

    void Map::EraseMapPoint(MapPoint *pMP)
    {
        unique_lock<mutex> lock(mMutexMap);
        mspMapPoints.erase(pMP);
    }

    void Map::EraseKeyFrame(KeyFrame *pKF)
    {
        unique_lock<mutex> lock(mMutexMap);
        mspKeyFrames.erase(pKF);

    }

    void Map::SetReferenceMapPoints(const vector<MapPoint *> &vpMPs)
    {
        unique_lock<mutex> lock(mMutexMap);
        mvpReferenceMapPoints = vpMPs;
    }
    void Map::AddObjectMapPoints(MapPoint *pMP)
    {
        unique_lock<mutex> lock(mMutexMap);
        mvpObjectMapPoints.insert(pMP);
    }

    void Map::AddObjMapPoints(MapPoint *pMP)
    {
        unique_lock<mutex> lock(mMutexMap);
        ObjectMapPoints.insert(pMP);
    }


    void Map::InformNewBigChange()
    {
        unique_lock<mutex> lock(mMutexMap);
        mnBigChangeIdx++;
    }

    vector<KeyFrame*> Map::GetAllKeyFrames()
    {
        unique_lock<mutex> lock(mMutexMap);
        return vector<KeyFrame*>(mspKeyFrames.begin(),mspKeyFrames.end());
    }

    vector<MapPoint*> Map::GetAllMapPoints()
    {
        unique_lock<mutex> lock(mMutexMap);
        return vector<MapPoint*>(mspMapPoints.begin(),mspMapPoints.end());
    }

    vector<MapPoint*> Map::GetAlobjlMapPoints()
    {
        unique_lock<mutex> lock(mMutexMap);
        return vector<MapPoint*>(ObjectMapPoints.begin(),ObjectMapPoints.end());
    }

    vector<Object*> Map::GetObjects()
    {
        unique_lock<mutex> lock(mMutexMap);
        return objs_real;
    }


    long unsigned int Map::MapPointsInMap()//画
    {
        unique_lock<mutex> lock(mMutexMap);
        return mspMapPoints.size();
    }

    long unsigned int Map::KeyFramesInMap()//画,判断什么时候执行local BA
    {
        unique_lock<mutex> lock(mMutexMap);
        return mspKeyFrames.size();
    }

    vector<MapPoint*> Map::GetReferenceMapPoints()//画
    {
        unique_lock<mutex> lock(mMutexMap);
        return mvpReferenceMapPoints;
    }

    long unsigned int Map::GetMaxKFid()//优化图
    {
        unique_lock<mutex> lock(mMutexMap);
        return mnMaxKFid;
    }

    void Map::clear()//reset
    {
        for(set<MapPoint*>::iterator sit=mspMapPoints.begin(), send=mspMapPoints.end(); sit!=send; sit++)
            delete *sit;

        for(set<KeyFrame*>::iterator sit=mspKeyFrames.begin(), send=mspKeyFrames.end(); sit!=send; sit++)
            delete *sit;

        mspMapPoints.clear();
        mspKeyFrames.clear();
        mnMaxKFid = 0;
        mvpReferenceMapPoints.clear();
        mvpKeyFrameOrigins.clear();
    }
}

