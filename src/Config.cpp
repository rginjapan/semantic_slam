//
// Created by zss on 18-4-16.
//
#include "Config.h"
#include "fstream"
namespace myslam
{
    void LoadImages(const string &strAssociationFilename, vector <string> &vstrImageFilenamesRGB,
                    vector <string> &vstrImageFilenamesD, vector<double> &vTimestamps)
    {
        ifstream fAssociation;
        fAssociation.open(strAssociationFilename.c_str());
        while (!fAssociation.eof()) {
            string s;
            getline(fAssociation, s);
            if (!s.empty()) {
                stringstream ss;
                ss << s;//输入到一个字符流
                double t;
                string sRGB, sD;
                ss >> t;//字符流往外输出
                vTimestamps.push_back(t);
                ss >> sRGB;
                vstrImageFilenamesRGB.push_back(sRGB);
                //cout<<"t1="<<std::fixed<<t<<endl;
                //cout<<"sRGB="<<sRGB<<endl;
                ss >> t;
                ss >> sD;
                vstrImageFilenamesD.push_back(sD);
                //cout<<"t2="<<std::fixed<<t<<endl;
                //cout<<"sD="<<sD<<endl;

            }
        }
    }

    void LoadDetections(const string fileName,vector<vector<int>> &_mat)
    {

        ifstream infile(fileName.c_str(), ios::in);
        if(!infile.is_open())
        {
            cout<<"yolo_detection file open fail"<<endl;
            exit(233);
        }
        vector<int> row;
        int tmp;
        string line;
        while (getline(infile, line))//todo 每一行读取，默认遇到换行符结束
        {
            istringstream istr(line);
            while (istr >> tmp) {
                row.push_back(tmp);
            }
            _mat.push_back(row);
            row.clear();
            istr.clear();
            line.clear();
        }
        infile.close();
    }

    void Detections_Align_To_File(vector<string> vstrImageFilenamesRGB,int nImages,vector<vector<int>> &yolo_mat,vector<vector<int>> &yolo_mat2)
    {
        ifstream infile;
        infile.open("/home/zss/myslam/Filename_desk.txt");
        if(!infile.is_open())
        {
            cout<<"Filename open fail"<<endl;
            exit(333);
        }
        string middle_num;
        int num=0;//2095
        int file_num=0;//613
        while (!infile.eof())
        {
            infile>>middle_num;
            for (int i = 0; i <nImages ; ++i)
            {
                if(strcmp(middle_num.c_str(),vstrImageFilenamesRGB[i].c_str())==0)
                {
                    for (int j = 0; j <yolo_mat.size(); ++j)
                    {
                        if(yolo_mat[j][0]==file_num)
                        {
                            if(yolo_mat[j][2]!=60&&yolo_mat[j][2]!=0&&yolo_mat[j][2]!=14&&yolo_mat[j][2]!=42&&yolo_mat[j][2]!=67&&yolo_mat[j][2]!=16&&yolo_mat[j][2]!=56&&yolo_mat[j][2]!=63)
                            {
                                yolo_mat2.push_back(yolo_mat[j]);
                                yolo_mat2[num][0]=i;
                                num++;
                            }

                        }
                    }
                    continue;
                }
            }
            file_num++;
        }
        infile.close();
    }


}

