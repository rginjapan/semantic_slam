
#ifndef MAPPOINT_H
#define MAPPOINT_H

#include"KeyFrame.h"
#include"Frame.h"
#include"Map.h"
#include <set>

#include<opencv2/core/core.hpp>
#include<mutex>

namespace myslam
{
    class KeyFrame;
    class Map;
    class Frame;
    class Object;
class MapPoint
{
public:
    MapPoint(const cv::Mat &Pos,KeyFrame* pRefKF, Map* pMap);
    MapPoint(const cv::Mat &Pos,Map* pMap, Frame* pFrame, const int &idxF);

    void SetWorldPos(const cv::Mat &Pos);
    cv::Mat GetWorldPos();
    cv::Mat GetNormal();

    ////能找到对应的特征点
    bool have_feature;
    cv::KeyPoint feature;
    KeyFrame* GetReferenceKeyFrame();

    int Observations();
    MapPoint* GetReplaced();
    void IncreaseFound(int n=1);

    void AddObservation(KeyFrame* pKF,size_t idx);
    void EraseObservation(KeyFrame* pKF);

    bool IsInKeyFrame(KeyFrame* pKF);

    float GetFoundRatio();
    void SetBadFlag();

    std::map<KeyFrame*,size_t> GetObservations();
    void ComputeDistinctiveDescriptors();
    void UpdateNormalAndDepth();
    float GetMinDistanceInvariance();
    float GetMaxDistanceInvariance();

    bool isBad();
    void IncreaseVisible(int n=1);
    int PredictScale(const float &currentDist, KeyFrame*pKF);
    int PredictScale(const float &currentDist, Frame* pF);

    cv::Mat GetDescriptor();
    void Replace(MapPoint* pMP);
    int GetIndexInKeyFrame(KeyFrame* pKF);


public:

    long unsigned int mnId;
    static long unsigned int nNextId;
    long int mnFirstKFid;
    long int mnFirstFrame;
    int nObs;
    int object_id;
    std::map<int,int> object_id_vector;
    bool First_obj;
    int addid;
    set<int> frame_id;

    long unsigned int mnLastFrameSeen;

    // Variables used by the tracking
    float mTrackProjX;
    float mTrackProjY;
    float mTrackProjXR;
    bool mbTrackInView;
    int mnTrackScaleLevel;
    float mTrackViewCos;

    long unsigned int mnTrackReferenceForFrame;

    // Variables used by local mapping
    long unsigned int mnBALocalForKF;
    long unsigned int mnFuseCandidateForKF;

    // Variables used by loop closing
    long unsigned int mnLoopPointForKF;
    long unsigned int mnCorrectedByKF;
    long unsigned int mnCorrectedReference;
    cv::Mat mPosGBA;
    long unsigned int mnBAGlobalForKF;
    static std::mutex mGlobalMutex;





protected:
    // Position in absolute coordinates
    cv::Mat mWorldPos;

    // Keyframes observing the point and associated index in keyframe
    std::map<KeyFrame*,size_t> mObservations;

    // Mean viewing direction
    cv::Mat mNormalVector;

    // Best descriptor to fast matching
    cv::Mat mDescriptor;

    // Reference KeyFrame
    KeyFrame* mpRefKF;

    // Tracking counters
    int mnVisible;
    int mnFound;

    // Bad flag (we do not currently erase MapPoint from memory)
    bool mbBad;
    MapPoint* mpReplaced;

    // Scale invariance distances
    float mfMinDistance;
    float mfMaxDistance;

    Map* mpMap;

    std::mutex mMutexPos;
    std::mutex mMutexFeatures;

};

} //namespace ORB_SLAM

#endif // MAPPOINT_H
