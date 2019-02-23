
#ifndef FRAME_H
#define FRAME_H

#include<vector>
#include <bitset>
#include "MapPoint.h"
#include "Thirdparty/DBoW2/DBoW2/BowVector.h"
#include "Thirdparty/DBoW2/DBoW2/FeatureVector.h"
#include "ORBVocabulary.h"
#include "KeyFrame.h"
#include "ORBextractor.h"
#include <opencv2/opencv.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>

namespace myslam
{
#define FRAME_GRID_ROWS 48
#define FRAME_GRID_COLS 64
    class MapPoint;
    class KeyFrame;
    class Object;
    class Frame
{
public:

    Frame();
    Frame(const Frame &frame);

    // Constructor for RGB-D cameras.
    Frame(const cv::Mat &imGray, const cv::Mat &imDepth, const double &timeStamp, ORBextractor* extractor,ORBVocabulary* voc, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth);
    inline cv::Mat GetCameraCenter(){
        return mOw.clone();
    }
    void ExtractORB(const cv::Mat &im);
    void ComputeBoW();
    void SetPose(cv::Mat Tcw);
    cv::Mat UnprojectStereo(const int &i);
    cv::Mat UnprojectStereo_camera(const int &i);
    void UpdatePoseMatrices();
    // Compute the cell of a keypoint (return false if outside the grid)
    bool PosInGrid(const cv::KeyPoint &kp, int &posX, int &posY);
    vector<size_t> GetFeaturesInArea(const float &x, const float  &y, const float  &r, const int minLevel=-1, const int maxLevel=-1) const;

    void ComputeStereoFromRGBD(const cv::Mat &imDepth);
    // Returns inverse of rotation
    inline cv::Mat GetRotationInverse(){
        return mRwc.clone();
    }

    bool isInFrustum(MapPoint* pMP, float viewingCosLimit);
    cv::Mat GetRotation();
    vector<Object*> get_frame_object(const cv::Mat &imRGB,const cv::Mat &imdepth);
    void get_object_feature();


public:

    //直线
    typedef Eigen::Matrix<double,6,1> Vector6d;
    typedef Eigen::Matrix<double,3,1> Vector3d;
    std::vector<Vector6d> LineVector6d;
    std::vector<Vector3d> F_Point;
    std::vector<Vector3d> E_Point;
    std::vector<Vector6d> LineVector6d_pro_camera;
    std::vector<Vector3d> F_Point_camera;
    std::vector<Vector3d> E_Point_camera;


    ORBVocabulary* mpORBvocabulary;
    ORBextractor* mpORBextractor;

    static long unsigned int nNextId;
    long unsigned int mnId;

    cv::Mat image;

    // Camera pose.
    cv::Mat mTcw;
    std::vector<cv::KeyPoint> mvKeysUn;
    // MapPoints associated to keypoints, NULL pointer if no association.
    std::vector<MapPoint*> mvpMapPoints;
    // Current and Next Frame id.

    std::vector<cv::KeyPoint> mvKeys;
    std::vector<float> mvuRight;

    std::vector<float> mvDepth;
    std::vector<Object*> objects;

    // Reference Keyframe.
    KeyFrame* mpReferenceKF;

    // Flag to identify outlier associations.
    std::vector<bool> mvbOutlier;
    double mTimeStamp;

    cv::Mat mK;
    static float fx;
    static float fy;
    static float cx;
    static float cy;
    static float invfx;
    static float invfy;
    cv::Mat mDistCoef;

    // Stereo baseline multiplied by fx.
    float mbf;

    // Stereo baseline in meters.
    float mb;

    float mThDepth;
    // Number of KeyPoints.
    int N;

    static float mfGridElementWidthInv;
    static float mfGridElementHeightInv;
    std::vector<std::size_t> mGrid[FRAME_GRID_COLS][FRAME_GRID_ROWS];

    DBoW2::BowVector mBowVec;
    DBoW2::FeatureVector mFeatVec;

    cv::Mat mDescriptors;

    // Scale pyramid info.
    int mnScaleLevels;
    float mfScaleFactor;
    float mfLogScaleFactor;
    vector<float> mvScaleFactors;
    vector<float> mvInvScaleFactors;
    vector<float> mvLevelSigma2;
    vector<float> mvInvLevelSigma2;


    static float mnMinX;
    static float mnMaxX;
    static float mnMinY;
    static float mnMaxY;

    static bool mbInitialComputations;

private:

    // Undistort keypoints given OpenCV distortion parameters.
    // Only for the RGB-D case. Stereo must be already rectified!
    // (called in the constructor).
    void UndistortKeyPoints();

    // Computes image bounds for the undistorted image (called in the constructor).
    void ComputeImageBounds(const cv::Mat &imLeft);

    // Assign keypoints to the grid for speed up feature matching (called in the constructor).
    void AssignFeaturesToGrid();

    // Rotation, translation and camera center
    cv::Mat mRcw;
    cv::Mat mtcw;
    cv::Mat mRwc;
    cv::Mat mOw; //==mtwc
};

}// namespace ORB_SLAM

#endif // FRAME_H
