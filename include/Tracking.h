
#ifndef TRACKING_H
#define TRACKING_H

#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>


#include"Viewer.h"
#include"FrameDrawer.h"
#include"Map.h"
#include"LocalMapping.h"
#include"LoopClosing.h"
#include"Frame.h"
#include "ORBVocabulary.h"
#include"KeyFrameDatabase.h"
#include"ORBextractor.h"
#include "MapDrawer.h"
#include "System.h"

#include <mutex>

namespace myslam
{
    class Viewer;
    class FrameDrawer;
    class Map;
    class LocalMapping;
    class LoopClosing;
    class System;
    class Object;

    class Tracking
{  

public:
    Tracking(System* pSys, ORBVocabulary* pVoc, FrameDrawer* pFrameDrawer, MapDrawer* pMapDrawer, Map* pMap,
             KeyFrameDatabase* pKFDB, const string &strSettingPath);
    void SetViewer(Viewer* pViewer);
    void SetLocalMapper(LocalMapping* pLocalMapper);
    void SetLoopClosing(LoopClosing* pLoopClosing);
    float maxdistance(vector<MapPoint*> vpmapoints,cv::Mat pos);
    float mindistance(vector<MapPoint*> vpmapoints,cv::Mat pos);
    float meandistance(cv::Mat mean,cv::Mat pos);
    bool polynomial_curve_fit(std::vector<cv::Point>& key_point, int n, cv::Mat& A);
    cv::Mat GrabImageRGBD(const cv::Mat &imRGB,const cv::Mat &imD, const double &timestamp);
    void ComputeBoxDepthFromRGBD(int ID,const cv::Mat &imDepth,std::vector<vector<float>> &mbDepth);
    vector<vector<cv::Mat>> get_object_vector(vector<Object*> &objs);
    std::vector<Object*> GetObjects();
        void InformOnlyTracking(const bool &flag);


public:
    // Tracking states
    enum eTrackingState{
        SYSTEM_NOT_READY=-1,
        NO_IMAGES_YET=0,
        NOT_INITIALIZED=1,
        OK=2,
        LOST=3
    };
    std::vector<vector<float>> mbDepth;
    eTrackingState mState;
    eTrackingState mLastProcessedState;
    // Current Frame
    Frame mCurrentFrame;
    cv::Mat mImGray;
    // Lists used to recover the full camera trajectory at the end of the execution.
    list<cv::Mat> mlRelativeFramePoses;
    list<KeyFrame*> mlpReferences;
    list<double> mlFrameTimes;
    list<bool> mlbLost;
    void Reset();
    Frame mInitialFrame;
        bool mbOnlyTracking;
    std::vector<int> mvIniMatches;
        bool Good_Frame;

    static bool AppearNewObject;

protected:
    void Track(const cv::Mat &imRGB,const cv::Mat &imD);
    void StereoInitialization(const cv::Mat &imRGB,const cv::Mat imD);
    void CheckReplacedInLastFrame();
    bool TrackReferenceKeyFrame(const cv::Mat &imRGB,const cv::Mat imD);
    void UpdateLastFrame();
    bool TrackWithMotionModel(const cv::Mat &imRGB,const cv::Mat imD);

    bool Relocalization();

    void UpdateLocalMap();
    void UpdateLocalPoints();
    void UpdateLocalKeyFrames();

    bool TrackLocalMap();
    void SearchLocalPoints();

    bool NeedNewKeyFrame();
    void CreateNewKeyFrame();

    LocalMapping* mpLocalMapper;
    LoopClosing* mpLoopClosing;

    ORBVocabulary* mpORBVocabulary;
    KeyFrameDatabase* mpKeyFrameDB;
    //Local Map
    KeyFrame* mpReferenceKF;
    std::vector<KeyFrame*> mvpLocalKeyFrames;
    std::vector<Frame*> mvpLocalObjectFrames;
    std::vector<MapPoint*> mvpObjectMapPoints;
    std::vector<MapPoint*> mvcullingMapPoints;
    std::vector<MapPoint*> mvpLocalMapPoints;
    // System
    System* mpSystem;
    //Drawers
    Viewer* mpViewer;
    FrameDrawer* mpFrameDrawer;
    MapDrawer* mpMapDrawer;
    //Map
    Map* mpMap;
    //Calibration matrix
    cv::Mat mK;
    cv::Mat mDistCoef;
    float mbf;
    //New KeyFrame rules (according to fps)
    int mMinFrames;
    int mMaxFrames;
    //ORB
    ORBextractor* mpORBextractor;
    // Threshold close/far points
    // Points seen as close by the stereo/RGBD sensor are considered reliable
    // and inserted from just one frame. Far points requiere a match in two keyframes.
    float mThDepth;

    // For RGB-D inputs only. For some datasets (e.g. TUM) the depthmap values are scaled.
    float mDepthMapFactor;

    //Current matches in frame
    int mnMatchesInliers;

    KeyFrame* mpLastKeyFrame;
    Frame mLastFrame;
    Frame LastFrame;
    Frame ok_LastFrame;
    unsigned int mnLastKeyFrameId;
    unsigned int mnLastRelocFrameId;
    //Motion Model
    cv::Mat mVelocity;

    list<MapPoint*> mlpTemporalPoints;



};

}

#endif // TRACKING_H
