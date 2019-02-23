//
// Created by zss on 18-4-16.
//
#include "common_include.h"
#ifndef MYSLAM_CONFIG_H
#define MYSLAM_CONFIG_H
namespace myslam
{
    void LoadImages(const string &strAssociationFilename, vector <string> &vstrImageFilenamesRGB,
                    vector <string> &vstrImageFilenamesD, vector<double> &vTimestamps);
    void LoadDetections(const string fileName,vector<vector<int>> &_mat);
    void Detections_Align_To_File(vector<string> vstrImageFilenamesRGB,int nImages,vector<vector<int>> &yolo_mat,vector<vector<int>> &yolo_mat2);

}
#endif //MYSLAM_CONFIG_H
