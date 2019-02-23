/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef MAP_H
#define MAP_H

#include "KeyFrame.h"
#include "MapPoint.h"
#include <set>
#include <mutex>

namespace myslam
{
    class MapPoint;
    class KeyFrame;
    class Object;

class Map
{
public:
    Map();
    std::mutex mMutexMapUpdate;
    long unsigned  KeyFramesInMap();
    void AddKeyFrame(KeyFrame* pKF);
    void AddMapPoint(MapPoint* pMP);
    void EraseMapPoint(MapPoint* pMP);
    void EraseObjectPoint(MapPoint* pMP);
    void EraseObjectPoint_(MapPoint* pMP);
    void EraseKeyFrame(KeyFrame* pKF);
    void SetReferenceMapPoints(const std::vector<MapPoint*> &vpMPs);
    void AddObjectMapPoints(MapPoint* pMP);
    void AddObjMapPoints(MapPoint *pMP);
    void clear();

    void InformNewBigChange();
    int GetLastBigChangeIdx();

    std::vector<KeyFrame*> GetAllKeyFrames();
    std::vector<MapPoint*> GetAllMapPoints();
    std::vector<MapPoint*> GetAlobjlMapPoints();
    std::vector<MapPoint*> GetReferenceMapPoints();
    std::vector<MapPoint*> GetObjectMapPoints();
    std::vector<MapPoint*> GetEraseMapPoints();
    std::vector<MapPoint*> GetObjectMapPoints_();
    std::vector<Object*> GetObjects();


    long unsigned int MapPointsInMap();

    long unsigned int GetMaxKFid();
    vector<KeyFrame*> mvpKeyFrameOrigins;

    std::mutex mMutexPointCreation;

    vector<Object*> objs_real;


protected:
    std::set<MapPoint*> mspMapPoints;
    std::set<KeyFrame*> mspKeyFrames;

    std::vector<MapPoint*> mvpReferenceMapPoints;
    std::set<MapPoint*> mvpObjectMapPoints;
    std::set<MapPoint*> ObjectMapPoints;

    long unsigned int mnMaxKFid;

    // Index related to a big change in the map (loop closure, global BA)
    int mnBigChangeIdx;

    std::mutex mMutexMap;

};

}

#endif // MAP_H
