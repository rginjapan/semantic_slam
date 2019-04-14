//
// Created by zss on 19-3-6.
//

#ifndef K_MEANS_POINT_H
#define K_MEANS_POINT_H

class K_Keypoint
{

public:
    K_Keypoint(cv::KeyPoint kp, double depthval, bool background) :kp(kp),depthval(depthval),background(background) {}
public:
    double depthval;
    cv::KeyPoint kp;
    bool background;

};
#endif //K_MEANS_POINT_H
