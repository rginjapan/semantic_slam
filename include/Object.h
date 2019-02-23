//
// Created by zss on 18-4-19.
//

#ifndef MYSLAM_OBJECT_H
#define MYSLAM_OBJECT_H

#include "System.h"
#include "bitset"
#include "MapPoint.h"
namespace myslam
{
    class Frame;
    class MapPoint;
    class KeyFrame;
    class Object
   {
   public:
       Object(const cv::Mat &Pos,const int class_id,float x, float y);

    public:
        int mnId;
        int LastAddId;
         cv::Mat _Pos;

        const int _class_id;
        bool First_obj;
        int confidence;
        int add_id;
        bool bad;
        bool current;
        vector< MapPoint*>  MapPonits;
        vector< MapPoint*>  pro_MapPonits;
        vector<cv::Mat> pro_MapPoints_camera;
        vector< MapPoint*>  co_MapPonits;

        float left;
        float right;
        float top;
        float bottom;
   };

}
#endif //MYSLAM_OBJECT_H
