//
// Created by zss on 19-3-6.
//
#include "kmeans.h"
const String imageFolder = "/home/zss/CLionProjects/k-means/";
const int numOfCluster =2;
const int MAX_PIX_VALUE = 255;///////////
//存放所有点
vector<K_Keypoint> points;
//存放所有簇中心
vector<K_Keypoint> centers;
//存放所有点颜色特征(i,j)->i*rows+j
vector<double> pixVec;

//初始化k-means聚类中心
void Kmeans::initializeCenters(const Mat& img)
{
    srand((unsigned)time(NULL));
    for (int i = 0; i < numOfCluster; i++)
    {
        int randomX = rand() % img.rows;
        int randomY = rand() % img.cols;
        uchar depthval = img.at<uchar>(randomX, randomY);
        K_Keypoint cp(randomX, randomY, (double)depthval);
        centers.push_back(cp);
    }
}

//将图像中的所有点装入points中
void Kmeans::initializePoints(const Mat& img)
{
    for (int i = 0; i < img.rows; i++)
    {
        const uchar* data = img.ptr<uchar>(i);
        for (int j = 0; j < img.cols; j++)
        {
            uchar depthval = data[j];
            K_Keypoint p(i,j, (double)depthval);
            points.push_back(p);
        }
    }
}

void Kmeans::begin_kmeans(const Mat& img)
{
    cvtColor(img, img, CV_RGB2GRAY);//转化为灰度图像

    initializeCenters(img);
    initializePoints(img);

    Kmeans* km=new Kmeans(points, centers, numOfCluster, 1);

    km->kmeans();
}
