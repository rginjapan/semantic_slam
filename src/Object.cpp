//
// Created by zss on 18-4-19.
//
#include "Object.h"
#include "vector"
#include "math.h"
#define PI 3.14159265 //圆周率
namespace myslam
{

    Object::Object(const cv::Mat &Pos,const int class_id,float x,float y):mnId(0),_Pos(Pos),_class_id(class_id),confidence(0),
                                                                          add_id(0),bad(false),left(0),right(0),top(0),bottom(0),LastAddId(0),current(false)
    {
        Pos.copyTo(_Pos);
    }

}

