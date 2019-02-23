//
// Created by zss on 18-4-18.
//
#include "MapDrawer.h"
#include "Tracking.h"
#include "Object.h"

namespace myslam
{
    MapDrawer::MapDrawer(Map* pMap, const string &strSettingPath,Tracking *pTracking):mpMap(pMap),mpTracker(pTracking)
    {
        cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);

        mKeyFrameSize = fSettings["Viewer.KeyFrameSize"];
        mKeyFrameLineWidth = fSettings["Viewer.KeyFrameLineWidth"];
        mGraphLineWidth = fSettings["Viewer.GraphLineWidth"];
        mPointSize = fSettings["Viewer.PointSize"];
        mCameraSize = fSettings["Viewer.CameraSize"];
        mCameraLineWidth = fSettings["Viewer.CameraLineWidth"];

    }

    void MapDrawer::DrawMapPoints()
     {
        const vector<MapPoint*> &vpMPs = mpMap->GetAllMapPoints();//map.h
        const vector<MapPoint*> &vpRefMPs = mpMap->GetReferenceMapPoints();//map.h
        const vector<MapPoint*> &vpRefMPs_obj = mpMap->GetAlobjlMapPoints();
        //const vector<MapPoint*> &vpObjMPs=mpMap->GetObjectMapPoints();
       // const vector<MapPoint*> &vpObjMPs_=mpMap->GetObjectMapPoints_();
        const vector<Object*> &Objs=mpMap->GetObjects();
        //const vector<MapPoint*> &vpObjMPs_landmark=mpMap->GetObjectMapPoints_landmark();
        //cout<<"objects:"<<vpObjMPs.size()<<endl;
        //cout<<"mappoints:"<<vpMPs.size()<<endl;
        //cout<<"reference:"<<vpRefMPs.size()<<endl;
        set<MapPoint*> spRefMPs(vpRefMPs.begin(), vpRefMPs.end());

        if(Objs.empty())
            return;


//        glPointSize(mPointSize*2);
//        glBegin(GL_POINTS);
//        glColor3f(0.0,0.0,0.0);
//
//        for(size_t i=0, iend=vpRefMPs_obj.size(); i<iend;i++)
//        {
//            if(vpRefMPs_obj[i]->isBad() )
//                continue;
//            cv::Mat pos = vpRefMPs_obj[i]->GetWorldPos();
//            glVertex3f(pos.at<float>(0),pos.at<float>(1),pos.at<float>(2));
//        }
//        glEnd();

        //todo 1 draw AllMapPoints
//        glPointSize(mPointSize);
//        glBegin(GL_POINTS);
//        glColor3f(0.0,0.0,0.0);
//
//        for(size_t i=0, iend=vpMPs.size(); i<iend;i++)
//        {
//            if(vpMPs[i]->isBad() || spRefMPs.count(vpMPs[i]))
//                continue;
//            for(set<MapPoint*>::iterator sit=spRefMPs.begin(), send=spRefMPs.end(); sit!=send; sit++)
//            {
//                if(spRefMPs.count(*sit))
//                break;
//            }
//            cv::Mat pos = vpMPs[i]->GetWorldPos();
//            glVertex3f(pos.at<float>(0),pos.at<float>(1),pos.at<float>(2));
//        }
//        glEnd();
//
//        //todo 2 draw ReferenceMapPoints
//        glPointSize(mPointSize);
//        glBegin(GL_POINTS);
//        glColor3f(1.0,0.0,0.0);
//
//        for(set<MapPoint*>::iterator sit=spRefMPs.begin(), send=spRefMPs.end(); sit!=send; sit++)
//        {
//            if((*sit)->isBad())
//                continue;
//            cv::Mat pos = (*sit)->GetWorldPos();
//            glVertex3f(pos.at<float>(0),pos.at<float>(1),pos.at<float>(2));
//
//        }
//
//        glEnd();
        //cout<<vpObjMPs.size()<<endl;

        //todo 3 draw ObjectMapPoints
        glPointSize(2*mPointSize);
        glBegin(GL_POINTS);
        glColor3f(0.0,0.0,0.0);
        for(size_t i=0, iend=Objs.size(); i<iend;i++)
        {
//            for (size_t j = 0; j < Objs[i]->MapPonits.size(); j++)
//            {
//                cv::Mat pos = Objs[i]->MapPonits[j]->GetWorldPos();
//                glVertex3f(pos.at<float>(0), pos.at<float>(1), pos.at<float>(2));
//            }
//        }
//        glEnd();

            if(i==0)//todo 黑色
            {
                //vector<MapPoint*> vpObjMPs=Objs[i]->MapPonits;
                for (size_t j = 0; j <Objs[i]->MapPonits.size() ; j++) {
                    glColor3f(0.0,1.0,1.0);
                    //vector<MapPoint*> vpObjMPs.
                    cv::Mat pos = Objs[i]->MapPonits[j]->GetWorldPos();
//                    glVertex3f(pos.at<float>(0),pos.at<float>(1),pos.at<float>(2));
                }
            }
            else if(i==1)//todo 红色
            {
                glColor3f(1.0,0.0,0.0);
                //vector<MapPoint*> vpObjMPs=Objs[i]->MapPonits;
                for (size_t j = 0; j <Objs[i]->co_MapPonits.size() ; j++) {
//                    cout<<"hahahaha"<<Objs[i]->pro_MapPonits.size()<<endl;
                    cv::Mat pos = Objs[i]->co_MapPonits[j]->GetWorldPos();
                    glVertex3f(pos.at<float>(0),pos.at<float>(1),pos.at<float>(2));
                }
                glColor3f(1.0,0.65,0.0);
                for (int k = 0; k <Objs[i]->pro_MapPonits.size(); ++k) {
//                    cout<<"hahahaha"<<Objs[i]->pro_MapPonits.size()<<endl;
                    cv::Mat pos = Objs[i]->pro_MapPonits[k]->GetWorldPos();
                    glVertex3f(pos.at<float>(0),pos.at<float>(1),pos.at<float>(2));
                }
            }
            else if(i==2)//todo 绿色
            {
                glColor3f(0.0,1.0,0.0);
                //vector<MapPoint*> vpObjMPs=Objs[i]->MapPonits;
                for (size_t j = 0; j <Objs[i]->co_MapPonits.size() ; j++) {
//                    cout<<"hahahaha"<<Objs[i]->pro_MapPonits.size()<<endl;
                    cv::Mat pos = Objs[i]->co_MapPonits[j]->GetWorldPos();
                    glVertex3f(pos.at<float>(0),pos.at<float>(1),pos.at<float>(2));
                }
                glColor3f(0.0,0.0,1.0);
                for (int k = 0; k <Objs[i]->co_MapPonits.size(); ++k) {
                    cv::Mat pos = Objs[i]->pro_MapPonits[k]->GetWorldPos();
                    glVertex3f(pos.at<float>(0),pos.at<float>(1),pos.at<float>(2));
                }
            }
            else if(i==3)//todo 蓝色
            {
                glColor3f(0.0,0.0,1.0);
                //vector<MapPoint*> vpObjMPs=Objs[i]->MapPonits;
                for (size_t j = 0; j <Objs[i]->MapPonits.size() ; j++) {
                    cv::Mat pos = Objs[i]->MapPonits[j]->GetWorldPos();
//                    glVertex3f(pos.at<float>(0),pos.at<float>(1),pos.at<float>(2));
                }
            }
            else if(i==4)//todo 青色
            {
                glColor3f(0.0,0.0,0.0);
                //vector<MapPoint*> vpObjMPs=Objs[i]->MapPonits;
                for (size_t j = 0; j <Objs[i]->MapPonits.size() ; j++) {
                    cv::Mat pos = Objs[i]->MapPonits[j]->GetWorldPos();
                    glVertex3f(pos.at<float>(0),pos.at<float>(1),pos.at<float>(2));
                }
            }
            else if(i==5)//todo 黄色
            {
                glColor3f(1.0,1.0,0.0);
                //vector<MapPoint*> vpObjMPs=Objs[i]->MapPonits;
                for (size_t j = 0; j <Objs[i]->MapPonits.size() ; j++) {
                    cv::Mat pos = Objs[i]->MapPonits[j]->GetWorldPos();
                    glVertex3f(pos.at<float>(0),pos.at<float>(1),pos.at<float>(2));
                }
            }
            else if(i==6)//todo 粉色
            {
                glColor3f(1.0,0.0,1.0);
                //vector<MapPoint*> vpObjMPs=Objs[i]->MapPonits;
                for (size_t j = 0; j <Objs[i]->MapPonits.size() ; j++) {
                    cv::Mat pos = Objs[i]->MapPonits[j]->GetWorldPos();
                    glVertex3f(pos.at<float>(0),pos.at<float>(1),pos.at<float>(2));
                }
            }
            else if(i==7)//todo 灰色
            {
                glColor3f(0.5,0.5,0.5);
                //vector<MapPoint*> vpObjMPs=Objs[i]->MapPonits;
                for (size_t j = 0; j <Objs[i]->MapPonits.size() ; j++) {
                    cv::Mat pos = Objs[i]->MapPonits[j]->GetWorldPos();
                    glVertex3f(pos.at<float>(0),pos.at<float>(1),pos.at<float>(2));
                }
            }
            else if(i==8)//todo　棕色
            {
                glColor3f(0.5,0,0);
                //vector<MapPoint*> vpObjMPs=Objs[i]->MapPonits;
                for (size_t j = 0; j <Objs[i]->MapPonits.size() ; j++) {
                    cv::Mat pos = Objs[i]->MapPonits[j]->GetWorldPos();
                    glVertex3f(pos.at<float>(0),pos.at<float>(1),pos.at<float>(2));
                }
            }
            else if(i==9)//todo
            {
                glColor3f(0,0.5,0);
                //vector<MapPoint*> vpObjMPs=Objs[i]->MapPonits;
                for (size_t j = 0; j <Objs[i]->MapPonits.size() ; j++) {
                    cv::Mat pos = Objs[i]->MapPonits[j]->GetWorldPos();
                    glVertex3f(pos.at<float>(0),pos.at<float>(1),pos.at<float>(2));
                }
            }
            else if(i==10)//todo 墨蓝色
            {
                glColor3f(0,0,0.5);
                //vector<MapPoint*> vpObjMPs=Objs[i]->MapPonits;
                for (size_t j = 0; j <Objs[i]->MapPonits.size() ; j++) {
                    cv::Mat pos = Objs[i]->MapPonits[j]->GetWorldPos();
                    glVertex3f(pos.at<float>(0),pos.at<float>(1),pos.at<float>(2));
                }
            }
            else if(i==11)//todo
            {
                glColor3f(0,0.5,0.5);
                //vector<MapPoint*> vpObjMPs=Objs[i]->MapPonits;
                for (size_t j = 0; j <Objs[i]->MapPonits.size() ; j++) {
                    cv::Mat pos = Objs[i]->MapPonits[j]->GetWorldPos();
                    glVertex3f(pos.at<float>(0),pos.at<float>(1),pos.at<float>(2));
                }
            }
            else if(i==12)//todo　土黄色
            {
                glColor3f(0.5,0.5,0);
                //vector<MapPoint*> vpObjMPs=Objs[i]->MapPonits;
                for (size_t j = 0; j <Objs[i]->MapPonits.size() ; j++) {
                    cv::Mat pos = Objs[i]->MapPonits[j]->GetWorldPos();
                    glVertex3f(pos.at<float>(0),pos.at<float>(1),pos.at<float>(2));
                }
            }
            else if(i==13)//todo 紫色
            {
                glColor3f(0.5,0,0.5);
                //vector<MapPoint*> vpObjMPs=Objs[i]->MapPonits;
                for (size_t j = 0; j <Objs[i]->MapPonits.size() ; j++) {
                    cv::Mat pos = Objs[i]->MapPonits[j]->GetWorldPos();
                    glVertex3f(pos.at<float>(0),pos.at<float>(1),pos.at<float>(2));
                }
            }
            else if(i==14)//todo　绿蓝之间
            {
                glColor3f(0.8,0,0.5);
                //vector<MapPoint*> vpObjMPs=Objs[i]->MapPonits;
                for (size_t j = 0; j <Objs[i]->MapPonits.size() ; j++) {
                    cv::Mat pos = Objs[i]->MapPonits[j]->GetWorldPos();
                    glVertex3f(pos.at<float>(0),pos.at<float>(1),pos.at<float>(2));
                }
            }
        }
        glEnd();

//        todo 4 draw Objectcenter
//        glLineWidth(mKeyFrameLineWidth);
//        glBegin(GL_LINES);
//        glColor3f(1.0,0.0,0.0);
//        glVertex3f(0,0,0);
//        glVertex3f(2.5,0,0);
//        glColor3f(0.0,1.0,0.0);
//        glVertex3f(0,0,0);
//        glVertex3f(0,2.5,0);
//        glColor3f(0.0,0.0,1.0);
//        glVertex3f(0,0,0);
//        glVertex3f(0,0,2.5);
//        glEnd();
//
//        glPointSize(4*mPointSize);
//        glBegin(GL_POINTS);
//        glColor3f(1.0,0.0,1.0);
//
//
//
//        for(size_t i=0, iend=vpObjMPs_.size(); i<iend;i++)
//        {
//            if(vpObjMPs_[i]->class_id==62)
//            {
//                //glColor3f(0.0,0.0,0.0);
//                cv::Mat pos = vpObjMPs_[i]->GetWorldPos();
//                glVertex3f(pos.at<float>(0),pos.at<float>(1),pos.at<float>(2));
//            }
//            else if(vpObjMPs_[i]->class_id==63)
//            {
//                //glColor3f(0.5,0.5,0.5);
//                cv::Mat pos = vpObjMPs_[i]->GetWorldPos();
//                glVertex3f(pos.at<float>(0),pos.at<float>(1),pos.at<float>(2));
//            }
//            else if(vpObjMPs_[i]->class_id==64)//
//            {
//                glColor3f(0.0,1.0,0.0);
//                cv::Mat pos = vpObjMPs_[i]->GetWorldPos();
//                glVertex3f(pos.at<float>(0),pos.at<float>(1),pos.at<float>(2));
//            }
//            else if(vpObjMPs_[i]->class_id==65)
//            {
//                glColor3f(0.0,0.0,1.0);
//                cv::Mat pos = vpObjMPs_[i]->GetWorldPos();
//                glVertex3f(pos.at<float>(0),pos.at<float>(1),pos.at<float>(2));
//            }
//            else if(vpObjMPs_[i]->class_id==66)
//            {
//                glColor3f(1.0,1.0,0.0);
//                cv::Mat pos = vpObjMPs_[i]->GetWorldPos();
//                glVertex3f(pos.at<float>(0),pos.at<float>(1),pos.at<float>(2));
//            }
//            else if(vpObjMPs_[i]->class_id==41)//
//            {
//                glColor3f(0.5,0.0,0.0);
//                cv::Mat pos = vpObjMPs_[i]->GetWorldPos();
//                glVertex3f(pos.at<float>(0),pos.at<float>(1),pos.at<float>(2));
//            }
//            else if(vpObjMPs_[i]->class_id==56)//
//            {
//                glColor3f(0.0,1.0,1.0);
//                cv::Mat pos = vpObjMPs_[i]->GetWorldPos();
//                glVertex3f(pos.at<float>(0),pos.at<float>(1),pos.at<float>(2));
//            }
//            else if(vpObjMPs_[i]->class_id==73)//
//            {
//                glColor3f(1.0,0.0,1.0);
//                cv::Mat pos = vpObjMPs_[i]->GetWorldPos();
//                glVertex3f(pos.at<float>(0),pos.at<float>(1),pos.at<float>(2));
//            }
//
//        }
//        glEnd();

        //todo 3 draw ObjectMapPoints
//        glPointSize(mPointSize*1.6);
//        glBegin(GL_POINTS);
//        glColor3f(0.0,0.0,0.0);
//
//        for(size_t i=0, iend=vpObjMPs_landmark.size(); i<iend;i++)
//        {
//            cv::Mat pos = vpObjMPs_landmark[i]->GetWorldPos();
//            glVertex3f(pos.at<float>(0),pos.at<float>(1),pos.at<float>(2));
//        }
//        glEnd();


}

    void MapDrawer::Drawobject3Dbox(pangolin::OpenGlMatrix &Twc)
    {
        const vector<Object*> vObjs = mpMap->GetObjects();
        vector<cv::Mat> object_cen;
        vector<cv::Mat> pro_object_cen;
        for(size_t i=0; i<vObjs.size(); i++)
        {
            if(i==0||i==3)
                continue;
            Object* Obj = vObjs[i];
            if(Obj->co_MapPonits.size()<6)
                continue;

            vector<float> x_pt;
            vector<float> y_pt;
            vector<float> z_pt;

            vector<float> pro_x_pt;
            vector<float> pro_y_pt;
            vector<float> pro_z_pt;

            float sum_x=0;
            float sum_y=0;
            float sum_z=0;

            float pro_sum_x=0;
            float pro_sum_y=0;
            float pro_sum_z=0;

//            cout<<"co:"<<Obj->co_MapPonits.size()<<" "<<Obj->pro_MapPoints_camera.size()<<" "<<Obj->pro_MapPonits.size()<<endl;
            for (int j = 0; j <Obj->co_MapPonits.size() ; ++j) {
                MapPoint *pp=Obj->co_MapPonits[j];
                cv::Mat pp_pt=pp->GetWorldPos();

                sum_x+=pp_pt.at<float>(0);
                sum_y+=pp_pt.at<float>(1);
                sum_z+=pp_pt.at<float>(2);

                x_pt.push_back(pp_pt.at<float>(0));
                y_pt.push_back(pp_pt.at<float>(1));
                z_pt.push_back(pp_pt.at<float>(2));
            }

            for (int k = 0; k <Obj->pro_MapPonits.size() ; ++k) {
                MapPoint *pp=Obj->pro_MapPonits[k];
                cv::Mat pp_pt=pp->GetWorldPos();

                pro_sum_x+=pp_pt.at<float>(0);
                pro_sum_y+=pp_pt.at<float>(1);
                pro_sum_z+=pp_pt.at<float>(2);

                pro_x_pt.push_back(pp_pt.at<float>(0));
                pro_y_pt.push_back(pp_pt.at<float>(1));
                pro_z_pt.push_back(pp_pt.at<float>(2));
            }

            sort(x_pt.begin(),x_pt.end());
            sort(y_pt.begin(),y_pt.end());
            sort(z_pt.begin(),z_pt.end());

            sort(pro_x_pt.begin(),pro_x_pt.end());
            sort(pro_y_pt.begin(),pro_y_pt.end());
            sort(pro_z_pt.begin(),pro_z_pt.end());

            float x_min=x_pt[0]; float pro_x_min=pro_x_pt[0];
            float x_max=x_pt[x_pt.size()-1];float pro_x_max=pro_x_pt[pro_x_pt.size()-1];

            float y_min=y_pt[0];float pro_y_min=pro_y_pt[0];
            float y_max=y_pt[y_pt.size()-1];float pro_y_max=pro_y_pt[pro_y_pt.size()-1];

            float z_min=z_pt[0];float pro_z_min=pro_z_pt[0];
            float z_max=z_pt[z_pt.size()-1];float pro_z_max=pro_z_pt[pro_z_pt.size()-1];

            float average_x=sum_x/Obj->co_MapPonits.size();
            float average_y=sum_y/Obj->co_MapPonits.size();
            float average_z=sum_z/Obj->co_MapPonits.size();

            float pro_average_x=pro_sum_x/Obj->pro_MapPonits.size();
            float pro_average_y=pro_sum_y/Obj->pro_MapPonits.size();
            float pro_average_z=pro_sum_z/Obj->pro_MapPonits.size();

            cv::Mat obj_cen=(cv::Mat_<float>(3,1)<< average_x, average_y, average_z);
            object_cen.push_back(obj_cen);

            cv::Mat pro_obj_cen=(cv::Mat_<float>(3,1)<< pro_average_x, pro_average_y, pro_average_z);
            pro_object_cen.push_back(pro_obj_cen);

            glPointSize(4*mPointSize);
            glBegin(GL_POINTS);
            if(i==1)
                glColor3f(1.0f,0.0f,0.0f);
            else if(i==2)
                glColor3f(0.0f,1.0f,0.0f);
            glVertex3f(average_x,average_y,average_z);
            glEnd();
            glPointSize(4*mPointSize);
            glBegin(GL_POINTS);
            if(i==1)
                glColor3f(1.0,0.65,0.0);
            else if(i==2)
                glColor3f(0.0f,0.0f,1.0f);
            glVertex3f(pro_average_x,pro_average_y,pro_average_z);
            glEnd();


            glLineWidth(mKeyFrameLineWidth);
            if(i==1)
                glColor3f(1.0f,0.0f,0.0f);
            else if(i==2)
                glColor3f(0.0f,1.0f,0.0f);
            glBegin(GL_LINES);

            glVertex3f(x_min,y_min,z_min);
            glVertex3f(x_max,y_min,z_min);
            glVertex3f(x_min,y_min,z_min);
            glVertex3f(x_min,y_max,z_min);
            glVertex3f(x_min,y_min,z_min);
            glVertex3f(x_min,y_min,z_max);

            glVertex3f(x_max,y_max,z_min);
            glVertex3f(x_max,y_min,z_min);
            glVertex3f(x_max,y_max,z_min);
            glVertex3f(x_min,y_max,z_min);
            glVertex3f(x_max,y_max,z_min);
            glVertex3f(x_max,y_max,z_max);

            glVertex3f(x_max,y_min,z_max);
            glVertex3f(x_max,y_min,z_min);
            glVertex3f(x_max,y_min,z_max);
            glVertex3f(x_min,y_min,z_max);
            glVertex3f(x_max,y_min,z_max);
            glVertex3f(x_max,y_max,z_max);

            glVertex3f(x_min,y_max,z_max);
            glVertex3f(x_min,y_min,z_max);
            glVertex3f(x_min,y_max,z_max);
            glVertex3f(x_min,y_max,z_min);
            glVertex3f(x_min,y_max,z_max);
            glVertex3f(x_max,y_max,z_max);


            glEnd();


            glLineWidth(mKeyFrameLineWidth);
            if(i==1)
                glColor3f(1.0,0.65,0.0);
            else if(i==2)
                glColor3f(0.0f,0.0f,1.0f);
            glBegin(GL_LINES);



            glVertex3f(pro_x_min,pro_y_min,pro_z_min);
            glVertex3f(pro_x_max,pro_y_min,pro_z_min);
            glVertex3f(pro_x_min,pro_y_min,pro_z_min);
            glVertex3f(pro_x_min,pro_y_max,pro_z_min);
            glVertex3f(pro_x_min,pro_y_min,pro_z_min);
            glVertex3f(pro_x_min,pro_y_min,pro_z_max);

            glVertex3f(pro_x_max,pro_y_max,pro_z_min);
            glVertex3f(pro_x_max,pro_y_min,pro_z_min);
            glVertex3f(pro_x_max,pro_y_max,pro_z_min);
            glVertex3f(pro_x_min,pro_y_max,pro_z_min);
            glVertex3f(pro_x_max,pro_y_max,pro_z_min);
            glVertex3f(pro_x_max,pro_y_max,pro_z_max);

            glVertex3f(pro_x_max,pro_y_min,pro_z_max);
            glVertex3f(pro_x_max,pro_y_min,pro_z_min);
            glVertex3f(pro_x_max,pro_y_min,pro_z_max);
            glVertex3f(pro_x_min,pro_y_min,pro_z_max);
            glVertex3f(pro_x_max,pro_y_min,pro_z_max);
            glVertex3f(pro_x_max,pro_y_max,pro_z_max);

            glVertex3f(pro_x_min,pro_y_max,pro_z_max);
            glVertex3f(pro_x_min,pro_y_min,pro_z_max);
            glVertex3f(pro_x_min,pro_y_max,pro_z_max);
            glVertex3f(pro_x_min,pro_y_max,pro_z_min);
            glVertex3f(pro_x_min,pro_y_max,pro_z_max);
            glVertex3f(pro_x_max,pro_y_max,pro_z_max);

            glEnd();
//          cout<<"object"<<i<<":"<<x_max-x_min<<" "<<y_max-y_min<<" "<<z_max-z_min<<endl;
        }
        glLineWidth(mKeyFrameLineWidth);
        glColor3f(0.0f,0.0f,0.0f);
        glBegin(GL_LINES);
        for (int k = 0; k <object_cen.size() ; ++k) {
            for (int i = 0; i <object_cen.size() ; ++i) {
                if(k==i)
                    continue;

                glVertex3f(object_cen[k].at<float>(0),object_cen[k].at<float>(1),object_cen[k].at<float>(2));
                glVertex3f(object_cen[i].at<float>(0),object_cen[i].at<float>(1),object_cen[i].at<float>(2));

                glVertex3f(pro_object_cen[k].at<float>(0),pro_object_cen[k].at<float>(1),pro_object_cen[k].at<float>(2));
                glVertex3f(pro_object_cen[i].at<float>(0),pro_object_cen[i].at<float>(1),pro_object_cen[i].at<float>(2));

            }
        }
        glEnd();
    }

    void MapDrawer::DrawKeyFrames(const bool bDrawKF, const bool bDrawGraph)
    {
        const float &w = mKeyFrameSize;
        const float h = w*0.75;
        const float z = w*0.6;

        const vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();

        if(bDrawKF)
        {
            for(size_t i=0; i<vpKFs.size(); i++)
            {
                KeyFrame* pKF = vpKFs[i];
                cv::Mat Twc = pKF->GetPoseInverse().t();

                glPushMatrix();

                glMultMatrixf(Twc.ptr<GLfloat>(0));

                glLineWidth(mKeyFrameLineWidth);
                glColor3f(0.0f,0.0f,1.0f);
                glBegin(GL_LINES);
                glVertex3f(0,0,0);
                glVertex3f(w,h,z);
                glVertex3f(0,0,0);
                glVertex3f(w,-h,z);
                glVertex3f(0,0,0);
                glVertex3f(-w,-h,z);
                glVertex3f(0,0,0);
                glVertex3f(-w,h,z);

                glVertex3f(w,h,z);
                glVertex3f(w,-h,z);

                glVertex3f(-w,h,z);
                glVertex3f(-w,-h,z);

                glVertex3f(-w,h,z);
                glVertex3f(w,h,z);

                glVertex3f(-w,-h,z);
                glVertex3f(w,-h,z);
                glEnd();

                glPopMatrix();
            }
        }

        if(bDrawGraph)
        {
            glLineWidth(mGraphLineWidth);
            glColor4f(0.0f,1.0f,0.0f,0.6f);
            glBegin(GL_LINES);

            for(size_t i=0; i<vpKFs.size(); i++)
            {
                // Covisibility Graph
                const vector<KeyFrame*> vCovKFs = vpKFs[i]->GetCovisiblesByWeight(100);
                cv::Mat Ow = vpKFs[i]->GetCameraCenter();
                if(!vCovKFs.empty())
                {
                    for(vector<KeyFrame*>::const_iterator vit=vCovKFs.begin(), vend=vCovKFs.end(); vit!=vend; vit++)
                    {
                        if((*vit)->mnId<vpKFs[i]->mnId)
                            continue;
                        cv::Mat Ow2 = (*vit)->GetCameraCenter();
                        glVertex3f(Ow.at<float>(0),Ow.at<float>(1),Ow.at<float>(2));
                        glVertex3f(Ow2.at<float>(0),Ow2.at<float>(1),Ow2.at<float>(2));
                    }
                }

                // Spanning tree
                KeyFrame* pParent = vpKFs[i]->GetParent();
                if(pParent)
                {
                    cv::Mat Owp = pParent->GetCameraCenter();
                    glVertex3f(Ow.at<float>(0),Ow.at<float>(1),Ow.at<float>(2));
                    glVertex3f(Owp.at<float>(0),Owp.at<float>(1),Owp.at<float>(2));
                }

                // Loops
                set<KeyFrame*> sLoopKFs = vpKFs[i]->GetLoopEdges();
                for(set<KeyFrame*>::iterator sit=sLoopKFs.begin(), send=sLoopKFs.end(); sit!=send; sit++)
                {
                    if((*sit)->mnId<vpKFs[i]->mnId)
                        continue;
                    cv::Mat Owl = (*sit)->GetCameraCenter();
                    glVertex3f(Ow.at<float>(0),Ow.at<float>(1),Ow.at<float>(2));
                    glVertex3f(Owl.at<float>(0),Owl.at<float>(1),Owl.at<float>(2));
                }
            }

            glEnd();
        }
    }

    void MapDrawer::DrawCurrentCamera(pangolin::OpenGlMatrix &Twc)
    {
        const float &w = mCameraSize;
        const float h = w*0.75;
        const float z = w*0.6;

        glPushMatrix();

#ifdef HAVE_GLES
        glMultMatrixf(Twc.m);
#else
        glMultMatrixd(Twc.m);
#endif

        glLineWidth(mCameraLineWidth);
        glColor3f(0.0f,1.0f,0.0f);
        glBegin(GL_LINES);
        glVertex3f(0,0,0);
        glVertex3f(w,h,z);
        glVertex3f(0,0,0);
        glVertex3f(w,-h,z);
        glVertex3f(0,0,0);
        glVertex3f(-w,-h,z);
        glVertex3f(0,0,0);
        glVertex3f(-w,h,z);

        glVertex3f(w,h,z);
        glVertex3f(w,-h,z);

        glVertex3f(-w,h,z);
        glVertex3f(-w,-h,z);

        glVertex3f(-w,h,z);
        glVertex3f(w,h,z);

        glVertex3f(-w,-h,z);
        glVertex3f(w,-h,z);
        glEnd();

        glPopMatrix();
    }


    void MapDrawer::SetCurrentCameraPose(const cv::Mat &Tcw)
    {
        unique_lock<mutex> lock(mMutexCamera);
        mCameraPose = Tcw.clone();
    }

    void MapDrawer::GetCurrentOpenGLCameraMatrix(pangolin::OpenGlMatrix &M)
    {
        if(!mCameraPose.empty())
        {
            cv::Mat Rwc(3,3,CV_32F);
            cv::Mat twc(3,1,CV_32F);
            {
                unique_lock<mutex> lock(mMutexCamera);
                Rwc = mCameraPose.rowRange(0,3).colRange(0,3).t();
                twc = -Rwc*mCameraPose.rowRange(0,3).col(3);
            }

            M.m[0] = Rwc.at<float>(0,0);
            M.m[1] = Rwc.at<float>(1,0);
            M.m[2] = Rwc.at<float>(2,0);
            M.m[3]  = 0.0;

            M.m[4] = Rwc.at<float>(0,1);
            M.m[5] = Rwc.at<float>(1,1);
            M.m[6] = Rwc.at<float>(2,1);
            M.m[7]  = 0.0;

            M.m[8] = Rwc.at<float>(0,2);
            M.m[9] = Rwc.at<float>(1,2);
            M.m[10] = Rwc.at<float>(2,2);
            M.m[11]  = 0.0;

            M.m[12] = twc.at<float>(0);
            M.m[13] = twc.at<float>(1);
            M.m[14] = twc.at<float>(2);
            M.m[15]  = 1.0;
        }
        else
            M.SetIdentity();
    }


}

