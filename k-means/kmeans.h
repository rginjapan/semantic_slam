//
// Created by zss on 19-3-6.
//

#ifndef K_MEANS_KMEANS_H
#define K_MEANS_KMEANS_H

#include<opencv2/opencv.hpp>
#include<random>
#include<time.h>
#include<stdio.h>
#include<stdlib.h>
#include<list>
#include<iostream>
#include<math.h>
#include"point.h"

using namespace cv;
using namespace std;

class Kmeans{
private:
    //存储所有点
    vector<K_Keypoint> points;
    //存储簇的中心点
    vector<K_Keypoint> centers;
    //存储每个点到相应的簇
    vector<K_Keypoint>* clusters;
    //向量的维数
    int dimension;
    //簇的个数
    int k;
public:
    //构造函数
    Kmeans(vector<K_Keypoint> points, vector<K_Keypoint> centers, int k, int dimension)
    {
        this->points = points;
        this->centers = centers;
        this->dimension = dimension;
        this->k = k;
        clusters = new vector<K_Keypoint>[k];
    }

    //析构函数
    ~Kmeans()
    {
        delete clusters;
    }

    //获取簇
    vector<K_Keypoint>* getClusters()
    {
        return this->clusters;
    }

    //计算两个向量之间的欧式距离
    double getDistanceBetweenTwoPoints(const K_Keypoint& point1, const K_Keypoint& point2)
    {
        double sum = 0;
        //double tmp;
        //for (int i = 0; i < dimension; i++)
        //{
        //tmp = pow(point1.pixVal - point2.pixVal,2);
        //sum += tmp;
        //}
        sum = pow(point1.depthval - point2.depthval, 2);
        return sqrt(sum);
    }

    //计算每个点到离它最近的簇中心点，结果保存到vector中
    vector<int> getClosetClusterCenterLabel()
    {
        double min;
        int label;
        vector<int> labels;
        for (int i = 0; i < points.size(); i++)
        {
            label = 0;
            min = getDistanceBetweenTwoPoints(points[i], centers[0]);
            for (int j = 1; j < centers.size(); j++)
            {
                double tmp = getDistanceBetweenTwoPoints(points[i], centers[j]);
                if (tmp < min)
                {
                    min = tmp;
                    label = j;
                }
            }
            labels.push_back(label);
        }
        return labels;
    }

    //将每个点放入它离的最近的中心点对应的簇中
    void computeClusters(const vector<int>& labels)
    {
        for (int i = 0; i < k; i++)
        {
            clusters[i].clear();
        }
        for (int i = 0; i < labels.size(); i++)
        {
            int label = labels[i];
            clusters[label].push_back(points[i]);
        }
    }

    //重新计算所有簇的中心点的灰度值
    void computeCenters()
    {
        centers.clear();
        for (int i = 0; i < k; i++)
        {
            double sum = 0;
            for (int j = 0; j < clusters[i].size(); j++)
            {
                sum += clusters[i][j].depthval;
            }
            double meanVal = sum / clusters[i].size();
            K_Keypoint cp(-1, -1, meanVal);
            centers.push_back(cp);
        }
    }

    //确定新的中心点后重新计算一次cost
    double computeCost()
    {
        double sum = 0;
        for (int i = 0; i < k; i++)
        {
            vector<K_Keypoint> tmpVec=clusters[i];
            for (int j = 0; j < tmpVec.size(); j++)
            {
                sum += getDistanceBetweenTwoPoints(tmpVec[j], centers[i]);
            }
        }
        return sum / points.size();
    }

    //迭代执行k-means算法的步骤
    void kmeans()
    {
        double oldCost, newCost;
        vector<int> labels=getClosetClusterCenterLabel();
        computeClusters(labels);
        newCost = computeCost();

        computeCenters();
        labels = getClosetClusterCenterLabel();
        computeClusters(labels);
        oldCost = newCost;
        newCost = computeCost();

        while (oldCost != newCost)
        {
            oldCost = newCost;
            computeCenters();
            labels = getClosetClusterCenterLabel();
            computeClusters(labels);
            newCost = computeCost();
        }
        cout <<"Final Cost: "<< newCost << endl;
    }

    void initializeCenters(const Mat& img);

    void initializePoints(const Mat& img);

    void begin_kmeans(const Mat& img);
};

#endif //K_MEANS_KMEANS_H
