//
// Created by zss on 18-4-16.
//
#include "Tracking.h"
#include "ORBmatcher.h"
#include "Optimizer.h"
#include "common_include.h"
#include "PnPsolver.h"
#include <chrono>
#include <stdfix.h>
#include "System.h"
#include "Object.h"
#include "math.h"
namespace myslam
{
    bool Tracking::AppearNewObject;

    Tracking::Tracking(System *pSys, ORBVocabulary* pVoc, FrameDrawer *pFrameDrawer, MapDrawer *pMapDrawer, Map *pMap, KeyFrameDatabase* pKFDB, const string &strSettingPath):
            mState(NO_IMAGES_YET), mbOnlyTracking(false),mpORBVocabulary(pVoc), mpKeyFrameDB(pKFDB),mpSystem(pSys), mpViewer(NULL),
            mpFrameDrawer(pFrameDrawer), mpMapDrawer(pMapDrawer), mpMap(pMap), mnLastRelocFrameId(0),Good_Frame(true)
    {
        //todo 1 Load camera parameters
        cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
        float fx = fSettings["Camera.fx"];
        float fy = fSettings["Camera.fy"];
        float cx = fSettings["Camera.cx"];
        float cy = fSettings["Camera.cy"];

        cv::Mat K = cv::Mat::eye(3,3,CV_32F);
        K.at<float>(0,0) = fx;
        K.at<float>(1,1) = fy;
        K.at<float>(0,2) = cx;
        K.at<float>(1,2) = cy;
        K.copyTo(mK);

        cv::Mat DistCoef(4,1,CV_32F);
        DistCoef.at<float>(0) = fSettings["Camera.k1"];
        DistCoef.at<float>(1) = fSettings["Camera.k2"];
        DistCoef.at<float>(2) = fSettings["Camera.p1"];
        DistCoef.at<float>(3) = fSettings["Camera.p2"];
        const float k3 = fSettings["Camera.k3"];
        if(k3!=0)
        {
            DistCoef.resize(5);
            DistCoef.at<float>(4) = k3;
        }
        DistCoef.copyTo(mDistCoef);
        mbf = fSettings["Camera.bf"];
        float fps = fSettings["Camera.fps"];
        if(fps==0)
            fps=30;
        mMinFrames = 0;
        mMaxFrames = fps;


        //todo 2 load ORB parameters and initialization
        int nFeatures = fSettings["ORBextractor.nFeatures"];
        float fScaleFactor = fSettings["ORBextractor.scaleFactor"];
        int nLevels = fSettings["ORBextractor.nLevels"];
        int fIniThFAST = fSettings["ORBextractor.iniThFAST"];
        int fMinThFAST = fSettings["ORBextractor.minThFAST"];
        mpORBextractor = new ORBextractor(nFeatures,fScaleFactor,nLevels,fIniThFAST,fMinThFAST);

        //todo 3 load depth scale parameters
        mThDepth = mbf*(float)fSettings["ThDepth"]/fx;
        mDepthMapFactor = fSettings["DepthMapFactor"];
        if(fabs(mDepthMapFactor)<1e-5)
           mDepthMapFactor=1;
        else
            mDepthMapFactor = 1.0f/mDepthMapFactor;
    }


    void Tracking::SetLocalMapper(LocalMapping *pLocalMapper)
    {
        mpLocalMapper=pLocalMapper;
    }

    void Tracking::SetLoopClosing(LoopClosing *pLoopClosing)
    {
        mpLoopClosing=pLoopClosing;
    }

    void Tracking::SetViewer(Viewer *pViewer)
    {
        mpViewer=pViewer;
    }
    float Tracking::maxdistance(vector<MapPoint*> vpmapoints,cv::Mat pos)
    {
        float max=0;
        for (int i = 0; i <vpmapoints.size() ; ++i) {
                float x = vpmapoints[i]->GetWorldPos().at<float>(0, 0) - pos.at<float>(0, 0);
                float y = vpmapoints[i]->GetWorldPos().at<float>(1, 0) - pos.at<float>(1, 0);
                float z = vpmapoints[i]->GetWorldPos().at<float>(2, 0) - pos.at<float>(2, 0);
                if (sqrt(pow(x, 2) + pow(y, 2) + pow(z, 2)) > max)
                {
                    max=sqrt(pow(x, 2) + pow(y, 2) + pow(z, 2));
                    if(max>0.7)
                        cout<<"~~~"<<vpmapoints[i]->GetWorldPos()<<endl<<pos<<endl;

                }
            }

        return max;
    }

    float Tracking::mindistance(vector<MapPoint*> vpmapoints,cv::Mat pos)
    {
        float min=9999;
        for (int i = 0; i <vpmapoints.size() ; ++i) {
            for (int j = 0; j <vpmapoints.size() ; ++j) {
                if(i==j)
                    continue;
                float x = vpmapoints[i]->GetWorldPos().at<float>(0, 0) - pos.at<float>(0, 0);
                float y = vpmapoints[i]->GetWorldPos().at<float>(1, 0) - pos.at<float>(1, 0);
                float z = vpmapoints[i]->GetWorldPos().at<float>(2, 0) - pos.at<float>(2, 0);
                if (sqrt(pow(x, 2) + pow(y, 2) + pow(z, 2)) < min)
                {
                    min=sqrt(pow(x, 2) + pow(y, 2) + pow(z, 2));
                }
            }

        }
        return min;
    }
    float Tracking::meandistance(cv::Mat mean,cv::Mat pos)
    {
        float x = mean.at<float>(0, 0) - pos.at<float>(0, 0);
        float y = mean.at<float>(1, 0) - pos.at<float>(1, 0);
        float z = mean.at<float>(2, 0) - pos.at<float>(2, 0);
        return sqrt(pow(x, 2) + pow(y, 2) + pow(z, 2));
    }
    cv::Mat Tracking::GrabImageRGBD(const cv::Mat &imRGB,const cv::Mat &imD, const double &timestamp)
    {
        //todo 1 load a pair of images
        mImGray = imRGB;
        cv::Mat imDepth = imD;
        cvtColor(mImGray,mImGray,CV_RGB2GRAY);
        if((fabs(mDepthMapFactor-1.0f)>1e-5) || imDepth.type()!=CV_32F)
            imDepth.convertTo(imDepth,CV_32F,mDepthMapFactor);
        //todo 2 create current frame
        AppearNewObject=false;
        Good_Frame=true;

        mCurrentFrame = Frame(mImGray,imDepth,timestamp,mpORBextractor,mpORBVocabulary,mK,mDistCoef,mbf,mThDepth);
        mCurrentFrame.image=imRGB.clone();

        //todo 3 track
        Track(mImGray,imDepth);

        vector<MapPoint*> vpMPs = mpMap->GetAllMapPoints();
        for (int k = 0; k <vpMPs.size() ; ++k) {
            if(vpMPs[k]->object_id_vector.count(0))
            {
                mpMap->AddObjMapPoints(vpMPs[k]);
                //cout<<"初始化"<<endl;
                //cout<<vpMPs[k]->GetWorldPos()<<endl;
            }
        }

        //todo 4 return mTcw
        return mCurrentFrame.mTcw.clone();
    }

    void Tracking::Track(const cv::Mat &imRGB,const cv::Mat &imD)
    {

        //todo 1 initialized
        if(mState==NO_IMAGES_YET)
        {
            mState = NOT_INITIALIZED;
        }
        mLastProcessedState=mState;
        // Get Map Mutex -> Map cannot be changed
        unique_lock<mutex> lock(mpMap->mMutexMapUpdate);
        if(mState==NOT_INITIALIZED)
        {
            StereoInitialization(imRGB,imD);
            mpFrameDrawer->Update(this);
            if(mState!=OK)
                return;
        }
        //todo 2 track
        else
        {
            //todo 2.1 track frame
            bool bOK;
            if(mState==OK)
            {
                //todo 检查并更新上一帧被替换的MapPoints
                //todo 更新Fuse函数和SearchAndFuse函数替换的MapPoints
                //todo Local Mapping might have changed some MapPoints tracked in last frame
                //todo CheckReplacedInLastFrame() 最后一帧地图点是否有替换点有替换点的则进行替换
                CheckReplacedInLastFrame();
                if(mVelocity.empty() || mCurrentFrame.mnId<mnLastRelocFrameId+2)
                {
                    bOK = TrackReferenceKeyFrame(imRGB,imD);
                }
                else
                {
                     bOK = TrackWithMotionModel(imRGB,imD);
                      if(!bOK)
                         bOK = TrackReferenceKeyFrame(imRGB,imD);

                }

            }
            else
            {
                bOK = Relocalization();
            }
            mCurrentFrame.mpReferenceKF = mpReferenceKF;//todo 当前帧的参考关键帧是局部地图的参考关键帧

            //todo 2.2 track local map
            if(bOK)
            {
                std::chrono::steady_clock::time_point t1 = std::chrono ::steady_clock::now();
                bOK = TrackLocalMap();
                std::chrono::steady_clock::time_point t2 = std::chrono ::steady_clock::now();
                double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2-t1).count();
                //cout<<"TrackLocalMap时间："<<ttrack<<endl;

            }

            if(bOK)
                mState = OK;
            else
                mState=LOST;
            mpFrameDrawer->Update(this);

            //todo 2.3 insert keyframes

            if(bOK)
            {
                if(!mLastFrame.mTcw.empty())
                {
                    cv::Mat LastTwc = cv::Mat::eye(4,4,CV_32F);
                    mLastFrame.GetRotationInverse().copyTo(LastTwc.rowRange(0,3).colRange(0,3));
                    mLastFrame.GetCameraCenter().copyTo(LastTwc.rowRange(0,3).col(3));
                    mVelocity = mCurrentFrame.mTcw*LastTwc;//mTcw是相机的SE(3)姿态
                }
                else
                    mVelocity = cv::Mat();
                mpMapDrawer->SetCurrentCameraPose(mCurrentFrame.mTcw);//////////todo 为了画图//////////
                for(int i=0; i<mCurrentFrame.N; i++)
                {
                    MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
                    if(pMP)
                        if(pMP->Observations()<1)
                        {
                            mCurrentFrame.mvbOutlier[i] = false;
                            mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
                        }
                }
                // todo Delete temporal MapPoints
                for(list<MapPoint*>::iterator lit = mlpTemporalPoints.begin(), lend =  mlpTemporalPoints.end(); lit!=lend; lit++)
                {
                    MapPoint* pMP = *lit;
                    delete pMP;
                }
                mlpTemporalPoints.clear();
                if(NeedNewKeyFrame())

                {

                    CreateNewKeyFrame();

                }
                for(int i=0; i<mCurrentFrame.N;i++)
                {
                    if(mCurrentFrame.mvpMapPoints[i] && mCurrentFrame.mvbOutlier[i])
                        mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
                }
            }

            //todo 2.4 Reset if the camera get lost soon after initialization
            if(mState==LOST)
            {
                if(mpMap->KeyFramesInMap()<=5)
                {
                    cout << "Track lost soon after initialisation, reseting..." << endl;
                    mpSystem->Reset();
                    return;
                }
            }

            if(!mCurrentFrame.mpReferenceKF)
                mCurrentFrame.mpReferenceKF = mpReferenceKF;
            mLastFrame = Frame(mCurrentFrame);
        }

        //todo 3 花轨迹  Store frame pose information to retrieve the complete camera trajectory afterwards.
        if(!mCurrentFrame.mTcw.empty())
        {
            cv::Mat Tcr = mCurrentFrame.mTcw*mCurrentFrame.mpReferenceKF->GetPoseInverse();
            //cout<<"参考关键帧："<<mCurrentFrame.mpReferenceKF->mnId<<endl;
            //todo 当前相对于世界*世界相对于参考帧
            mlRelativeFramePoses.push_back(Tcr);
            mlpReferences.push_back(mpReferenceKF);
            mlFrameTimes.push_back(mCurrentFrame.mTimeStamp);
            mlbLost.push_back(mState==LOST);
        }
        else
        {
            // This can happen if tracking is lost
            mlRelativeFramePoses.push_back(mlRelativeFramePoses.back());
            mlpReferences.push_back(mlpReferences.back());
            mlFrameTimes.push_back(mlFrameTimes.back());//todo 取最后一个元素
            mlbLost.push_back(mState==LOST);
        }

    }

    void Tracking::StereoInitialization(const cv::Mat &imRGB,const cv::Mat imD)
    {
        if(mCurrentFrame.N>500)
        {
            // todo Set Frame pose to the origin
            mCurrentFrame.SetPose(cv::Mat::eye(4,4,CV_32F));
            KeyFrame* pKFini = new KeyFrame(mCurrentFrame,mpMap,mpKeyFrameDB);
            mpMap->AddKeyFrame(pKFini);
            //todo 获取每帧的物体
            cout<<"当前帧："<<mCurrentFrame.mnId<<endl;
            vector<Object*> objs;
            objs=mCurrentFrame.get_frame_object(imRGB,imD);
            //AppearNewObject=true;
            //todo 获取每个物体的mappoint
            for(int i=0; i<mCurrentFrame.N;i++)
            {
                float z = mCurrentFrame.mvDepth[i];
                if(z>0)
                {
                    cv::Mat x3D = mCurrentFrame.UnprojectStereo(i);
                    MapPoint* pNewMP = new MapPoint(x3D,pKFini,mpMap);
                    pNewMP->AddObservation(pKFini,i);
                    pNewMP->addid=mCurrentFrame.mnId;
                   for (int k = 0;k <objs.size() ;++k)
                    {
                        objs[k]->First_obj==true;
                        objs[k]->mnId=k;
                        if (mCurrentFrame.mvKeysUn[i].pt.x > objs[k]->left &&
                            mCurrentFrame.mvKeysUn[i].pt.x < objs[k]->right &&
                            mCurrentFrame.mvKeysUn[i].pt.y >  objs[k]->top &&
                            mCurrentFrame.mvKeysUn[i].pt.y < objs[k]->bottom)
                        {
                            float depth=x3D.at<float>(2,0);
                            float middle_point=objs[k]->_Pos.at<float>(2,0);
                            if(abs(depth-middle_point)<0.1)////去除背景点
                            {
                                mpMap->AddObjectMapPoints(pNewMP);
                                pNewMP->object_id=objs[k]->mnId;
                                pNewMP->object_id_vector.insert(make_pair(pNewMP->object_id,1));
                                pNewMP->frame_id.insert(mCurrentFrame.mnId);
                                if(objs[k]->First_obj==true)
                                    pNewMP->First_obj=true;
                                objs[k]->MapPonits.push_back(pNewMP);
                            }
                        }
                    }
                    pKFini->AddMapPoint(pNewMP,i);//todo 不是pushback有影响吗在前面在后面
                    pNewMP->ComputeDistinctiveDescriptors();
                    pNewMP->UpdateNormalAndDepth();
                    mpMap->AddMapPoint(pNewMP);
                    mCurrentFrame.mvpMapPoints[i]=pNewMP;
                }
            }
            cout<<mpMap->objs_real.size()<<endl;
            for (int j = 0; j <mpMap->objs_real.size() ; ++j) {
                if(mpMap->objs_real[j]->mnId==0)
                    cout<<mpMap->objs_real[j]->MapPonits.size()<<endl;
            }

            for (int k = 0; k <objs.size() ; ++k)
            {
                //// 用所有地图点位置的平均值作为物体的位置
//                cv::Mat avg_pos=(cv::Mat_<float>(3,1)<<(0,0,0));
//                for (int i = 0; i <objs[k]->MapPonits.size() ; ++i) {
//                    cv::Mat x3d=objs[k]->MapPonits[i]->GetWorldPos();
//                    avg_pos+=x3d;
//                }
//                avg_pos/=objs[k]->MapPonits.size();
//                objs[k]->_Pos=avg_pos;
                mpMap->objs_real.push_back(objs[k]);
                mpMap->objs_real[k]->mnId=k;
                mpMap->objs_real[k]->confidence++;
                mpMap->objs_real[k]->add_id=mpMap->objs_real[k]->LastAddId=mCurrentFrame.mnId;
                mCurrentFrame.objects.push_back(objs[k]);
            }
            pKFini->objects=mCurrentFrame.objects;
            //todo 更新上一帧
            mpLocalMapper->InsertKeyFrame(pKFini);
            mLastFrame = Frame(mCurrentFrame);
            mnLastKeyFrameId=mCurrentFrame.mnId;
            mpLastKeyFrame = pKFini;

            //todo 跟踪局部地图
            mvpLocalKeyFrames.push_back(pKFini);
            mvpLocalMapPoints=mpMap->GetAllMapPoints();//todo 因为第一帧之后没有跟踪局部地图，所以单独给赋值
            mpReferenceKF = pKFini;
            mCurrentFrame.mpReferenceKF = pKFini;

            //todo 为了画图
            mpMap->SetReferenceMapPoints(mvpLocalMapPoints);
            mpMap->mvpKeyFrameOrigins.push_back(pKFini);
            mpMapDrawer->SetCurrentCameraPose(mCurrentFrame.mTcw);
            mState=OK;
        }
    }

    void Tracking::CheckReplacedInLastFrame()
    {
        for(int i =0; i<mLastFrame.N; i++)
        {
            MapPoint* pMP = mLastFrame.mvpMapPoints[i];
            if(pMP)
            {
                MapPoint* pRep = pMP->GetReplaced();
                if(pRep)
                {
                    mLastFrame.mvpMapPoints[i] = pRep;
                }
            }
        }
    }

    bool Tracking::TrackReferenceKeyFrame(const cv::Mat &imRGB,const cv::Mat imD)
    {
        //todo 1 compute BOW
        mCurrentFrame.ComputeBoW();
        //todo 2 ORB matching
        ORBmatcher matcher(0.7,true);
        vector<MapPoint*> vpMapPointMatches;
        int nmatches = matcher.SearchByBoW(mpReferenceKF,mCurrentFrame,vpMapPointMatches);
        if(nmatches<15)
            return false;
        //todo 3 PoseOptimization
        mCurrentFrame.mvpMapPoints = vpMapPointMatches;
        mCurrentFrame.SetPose(mLastFrame.mTcw);
        //todo 获取每帧物体
        cout<<"当前帧："<<mCurrentFrame.mnId<<endl;
        vector<Object*> objs;
        objs=mCurrentFrame.get_frame_object(imRGB,imD);

        //todo 获取每个物体的mappoint

        int num2=0;
        for (int k = 0; k <objs.size() ; ++k)
        {
            int _num = 0;
            int _num2 = 0;
            bool zss = true;
            for (int i = mpMap->objs_real.size() - 1; (i >= 0) && (zss == true); i--)
            {
                if (objs[k]->_class_id == mpMap->objs_real[i]->_class_id)
                {
                    _num++;
                    float x = objs[k]->_Pos.at<float>(0, 0) - mpMap->objs_real[i]->_Pos.at<float>(0, 0);
                    float y = objs[k]->_Pos.at<float>(1, 0) - mpMap->objs_real[i]->_Pos.at<float>(1, 0);
                    float z = objs[k]->_Pos.at<float>(2, 0) - mpMap->objs_real[i]->_Pos.at<float>(2, 0);
                    if (sqrt(pow(x, 2) + pow(y, 2) + pow(z, 2)) > 0.4)//
                    {
                        continue;
                    }
                    else//todo 重复物体出现
                    {
                        _num2++;

                        if( mpMap->objs_real[i]->LastAddId!=mCurrentFrame.mnId)//todo 为了防止同一帧同一个物体加两次
                        {
                            mpMap->objs_real[i]->LastAddId=mCurrentFrame.mnId;
                            mpMap->objs_real[i]->confidence++;
                            objs[k]->current=true;
                        }

                        objs[k]->mnId=mpMap->objs_real[i]->mnId;
                        zss = false;
                    }
                }
                else
                    continue;
            }
            if (_num == 0 || _num2 == 0)//todo 出现新物体
            {
                AppearNewObject = true;
                objs[k]->mnId = mpMap->objs_real.size();
                objs[k]->add_id =objs[k]->LastAddId= mCurrentFrame.mnId;
                objs[k]->confidence++;
                objs[k]->First_obj==true;
                mpMap->objs_real.push_back(objs[k]);
                objs[k]->current=true;
            }
        }

        for(int i =0; i<mCurrentFrame.N; i++)
        {
            if (mCurrentFrame.mvpMapPoints[i])
            {
                MapPoint *pMP = mCurrentFrame.mvpMapPoints[i];
                if(!pMP->isBad())
                {
                    set<int> object_num;
                    for (int k = 0; k < objs.size(); ++k)
                    {
                        if(object_num.count(objs[k]->mnId))
                            continue;
                        if (mCurrentFrame.mvKeysUn[i].pt.x > objs[k]->left &&//todo 这个计算的是当前帧的
                            mCurrentFrame.mvKeysUn[i].pt.x < objs[k]->right &&
                            mCurrentFrame.mvKeysUn[i].pt.y > objs[k]->top &&
                            mCurrentFrame.mvKeysUn[i].pt.y < objs[k]->bottom)
                        {
                            float z = mCurrentFrame.mvDepth[i];
                            if(z>0&&pMP->frame_id.count(mCurrentFrame.mnId-1))
                            {
                                cv::Mat pro_x3D=mCurrentFrame.UnprojectStereo(i);/////for 画图
                                cv::Mat pro_x3Dc=mCurrentFrame.UnprojectStereo_camera(i);
                                MapPoint* pro_MP = new MapPoint(pro_x3D,mpMap,&mCurrentFrame,i);//todo 只有这里使用帧创建地图点而不是关键帧
                                objs[k]->pro_MapPonits.push_back(pro_MP);
                                objs[k]->pro_MapPoints_camera.push_back(pro_x3Dc);///for 优化
                                objs[k]->co_MapPonits.push_back(pMP);
                                cout<<mCurrentFrame.mnId<<":"<<k<<":co_obs"<<endl;
                            }


                            cv::Mat x3D = pMP->GetWorldPos();
                            float depth = x3D.at<float>(2, 0);
                            float middle_point = objs[k]->_Pos.at<float>(2, 0);

                            if (abs(depth - middle_point) < 0.1)
                            {
                                if(objs[k]->First_obj==true)
                                    pMP->First_obj==true;
                                pMP->object_id = objs[k]->mnId;
                                pMP->frame_id.insert(mCurrentFrame.mnId);
                                map<int,int>::iterator sit;
                                sit = pMP->object_id_vector.find(pMP->object_id);
                                if(sit != pMP->object_id_vector.end())
                                {
                                    int sit_sec=sit->second;
                                    pMP->object_id_vector.erase(pMP->object_id);
                                    pMP->object_id_vector.insert(make_pair(pMP->object_id, sit_sec+1));//todo 观测次数不对
                                }
                                else
                                {
                                    pMP->object_id_vector.insert(make_pair(pMP->object_id,1));
                                }
                                mpMap->AddObjectMapPoints(pMP);
                                objs[k]->MapPonits.push_back(pMP);
                            }
                        }
                        object_num.insert(objs[k]->mnId);
                    }

                }
            }
        }

        for (int l = 0; l <objs.size() ; ++l) {
            if(objs[l]->current==true)
                mCurrentFrame.objects.push_back(objs[l]);
            if(objs[l]->MapPonits.size()==0)
                continue;
            int zzss=0;
            int www=0;
            for (int j = 0; j <mpMap->objs_real.size(); ++j)
            {
                if(mpMap->objs_real[j]->mnId==objs[l]->mnId)
                {
                    zzss++;
                    for (int i = 0; i <objs[l]->MapPonits.size() ; ++i)
                    {
                        int num=0;
                        for (int m = 0; m <mpMap->objs_real[j]->MapPonits.size() ; ++m)
                        {
                            cv::Mat obj_pos=objs[l]->MapPonits[i]->GetWorldPos();
                            cv::Mat obj_real_pos=mpMap->objs_real[j]->MapPonits[m]->GetWorldPos();
                            if(cv::countNonZero(obj_pos-obj_real_pos)==0)////cv::mat不可以直接==判断  重要
                            {
                                mpMap->objs_real[j]->MapPonits[m]->have_feature=objs[l]->MapPonits[i]->have_feature;
                                mpMap->objs_real[j]->MapPonits[m]->feature=objs[l]->MapPonits[i]->feature;//// 同一个点的特征没有更新
                                num++;
                                break;
                            }
                        }
                        if(num==0)
                        {
                            www++;
                            mpMap->objs_real[j]->MapPonits.push_back(objs[l]->MapPonits[i]);
                        }
                    }
                    for (int k = 0; k <objs[l]->pro_MapPonits.size(); ++k) {
                        int num=0;
                        for (int m = 0; m <mpMap->objs_real[j]->pro_MapPonits.size() ; ++m)
                        {
                            cv::Mat obj_pos=objs[l]->pro_MapPonits[k]->GetWorldPos();
                            cv::Mat obj_real_pos=mpMap->objs_real[j]->pro_MapPonits[m]->GetWorldPos();
                            if(cv::countNonZero(obj_pos-obj_real_pos)==0)////cv::mat不可以直接==判断  重要
                            {
                                mpMap->objs_real[j]->pro_MapPonits[m]->have_feature=objs[l]->pro_MapPonits[k]->have_feature;
                                mpMap->objs_real[j]->pro_MapPonits[m]->feature=objs[l]->pro_MapPonits[k]->feature;//// 同一个点的特征没有更新
                                num++;
                                break;
                            }
                        }
                        if(num==0)
                        {
                            www++;
                            mpMap->objs_real[j]->pro_MapPonits.push_back(objs[l]->pro_MapPonits[k]);
                            mpMap->objs_real[j]->pro_MapPoints_camera.push_back(objs[l]->pro_MapPoints_camera[k]);
                            mpMap->objs_real[j]->co_MapPonits.push_back(objs[l]->co_MapPonits[k]);
                        }

                    }
                }
                if(zzss!=0)
                    break;
            }
        }
        Optimizer::PoseOptimization(&mCurrentFrame);
        int nmatchesMap = 0;
        for(int i =0; i<mCurrentFrame.N; i++)
        {
            if(mCurrentFrame.mvpMapPoints[i])
            {
                MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
                if(mCurrentFrame.mvbOutlier[i])
                {
                    mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
                    mCurrentFrame.mvbOutlier[i]=false;
                    pMP->mbTrackInView = false;
                    pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                    nmatches--;
                }

                else
                {
                    pMP->have_feature=true;
                    pMP->feature=mCurrentFrame.mvKeysUn[i];
                    if(mCurrentFrame.mvpMapPoints[i]->Observations()>0)
                        nmatchesMap++;
                }

            }
        }
        cout<<mpMap->objs_real.size()<<endl;
//        for (int n = 0; n <mpMap->objs_real.size() ; ++n) {
//            //// 用所有地图点位置的平均值作为物体的位置
//            cv::Mat avg_pos=(cv::Mat_<float>(3,1)<<(0,0,0));
//            for (int i = 0; i <mpMap->objs_real[n]->MapPonits.size() ; ++i) {
//                cv::Mat x3d=mpMap->objs_real[n]->MapPonits[i]->GetWorldPos();
//                avg_pos+=x3d;
//            }
//            avg_pos/=mpMap->objs_real[n]->MapPonits.size();
//            mpMap->objs_real[n]->_Pos=avg_pos;
//        }

        cout<<"当前帧物体："<<mCurrentFrame.objects.size()<<endl;
        //todo 4 Discard outliers
        return nmatchesMap>=10;
    }

    void Tracking::UpdateLastFrame()//todo 在跟踪运动模型中，为了跟踪临时添加一些 mappoint
    {
        KeyFrame* pRef = mLastFrame.mpReferenceKF;
        cv::Mat Tlr = mlRelativeFramePoses.back();
        mLastFrame.SetPose(Tlr*pRef->GetPose());
        if(mnLastKeyFrameId==mLastFrame.mnId|| true)
            return;

        // Create "visual odometry" MapPoints
        // We sort points according to their measured depth by the RGB-D sensor
        vector<pair<float,int> > vDepthIdx;
        vDepthIdx.reserve(mLastFrame.N);
        for(int i=0; i<mLastFrame.N;i++)
        {
            float z = mLastFrame.mvDepth[i];
            if(z>0)
            {
                vDepthIdx.push_back(make_pair(z,i));
            }
        }

        if(vDepthIdx.empty())
            return;
        sort(vDepthIdx.begin(),vDepthIdx.end());

        // We insert all close points (depth<mThDepth)
        // If less than 100 close points, we insert the 100 closest ones.
        int nPoints = 0;
        for(size_t j=0; j<vDepthIdx.size();j++)
        {
            int i = vDepthIdx[j].second;
            bool bCreateNew = false;
            MapPoint* pMP = mLastFrame.mvpMapPoints[i];
            if(!pMP)
                bCreateNew = true;
            else if(pMP->Observations()<1)
            {
                bCreateNew = true;
            }

            if(bCreateNew)
            {
                cv::Mat x3D = mLastFrame.UnprojectStereo(i);
                MapPoint* pNewMP = new MapPoint(x3D,mpMap,&mLastFrame,i);//todo 只有这里使用帧创建地图点而不是关键帧
                mLastFrame.mvpMapPoints[i]=pNewMP;
                mlpTemporalPoints.push_back(pNewMP);
                nPoints++;
            }
            else
            {
                nPoints++;
            }

            if(vDepthIdx[j].first>mThDepth && nPoints>100)
                break;
        }
    }

    bool Tracking::TrackWithMotionModel(const cv::Mat &imRGB,const cv::Mat imD)
    {

        //todo 1 ORB matching
        ORBmatcher matcher(0.9,true);


        //todo 2 update last frame
        UpdateLastFrame();

        //todo 3 update current frame
        mCurrentFrame.SetPose(mVelocity*mLastFrame.mTcw);
        fill(mCurrentFrame.mvpMapPoints.begin(),mCurrentFrame.mvpMapPoints.end(),static_cast<MapPoint*>(NULL));

        cout<<"当前帧："<<mCurrentFrame.mnId<<endl;
        vector<Object*> objs;
        objs=mCurrentFrame.get_frame_object(imRGB,imD);

        int num2=0;
        for (int k = 0; k <objs.size() ; ++k)
        {
            int _num=0;
            int _num2=0;
            bool zss=true;
            bool jh=true;
            bool jh_2=true;
            bool jh_pig=true;
            bool jh_pig_2=true;
            for (int i =mpMap->objs_real.size()-1; (i>=0)&&(zss==true); i--)
            {
                if(objs[k]->_class_id==mpMap->objs_real[i]->_class_id)
                {
                    _num++;
                    double x=objs[k]->_Pos.at<float>(0,0)-mpMap->objs_real[i]->_Pos.at<float>(0,0);
                    double y=objs[k]->_Pos.at<float>(1,0)-mpMap->objs_real[i]->_Pos.at<float>(1,0);
                    double z=objs[k]->_Pos.at<float>(2,0)-mpMap->objs_real[i]->_Pos.at<float>(2,0);
                    if(sqrt(pow(x,2)+pow(y,2)+pow(z,2))>0.4)
                        continue;
                    else
                    {
                        _num2++;

                        if( mpMap->objs_real[i]->LastAddId!=mCurrentFrame.mnId)
                        {
                            mpMap->objs_real[i]->LastAddId=mCurrentFrame.mnId;
                            mpMap->objs_real[i]->confidence++;
                            objs[k]->current=true;
                        }
                        objs[k]->mnId=mpMap->objs_real[i]->mnId;
                        zss=false;
                    }

                }
                else
                    continue;
            }
            if(objs[k]->_class_id==63&&(_num==0||_num2==0))
            {
                for (int i =mpMap->objs_real.size()-1; (i>=0)&&(jh==true); i--)
                {
                    if(mpMap->objs_real[i]->_class_id==62)
                    {
                        float x=objs[k]->_Pos.at<float>(0,0)-mpMap->objs_real[i]->_Pos.at<float>(0,0);
                        float y=objs[k]->_Pos.at<float>(1,0)-mpMap->objs_real[i]->_Pos.at<float>(1,0);
                        float z=objs[k]->_Pos.at<float>(2,0)-mpMap->objs_real[i]->_Pos.at<float>(2,0);
                        if(sqrt(pow(x,2)+pow(y,2)+pow(z,2))>0.4)
                        {
                            continue;
                        }
                        else
                        {
                            //cout<<"63-62"<<endl;
                            objs[k]->bad=true;

                            if( mpMap->objs_real[i]->LastAddId!=mCurrentFrame.mnId)
                            {
                                mpMap->objs_real[i]->LastAddId=mCurrentFrame.mnId;
                                mpMap->objs_real[i]->confidence++;
                                objs[k]->current=true;
                            }
                            objs[k]->mnId=mpMap->objs_real[i]->mnId;
                            jh=false;
                        }

                    }
                    else
                        continue;
                }

            }
            if(objs[k]->_class_id==62&&(_num==0||_num2==0))
            {
                for (int i =mpMap->objs_real.size()-1; (i>=0)&&(jh_2==true); i--)
                {
                    if(mpMap->objs_real[i]->_class_id==63)
                    {
                        float x=objs[k]->_Pos.at<float>(0,0)-mpMap->objs_real[i]->_Pos.at<float>(0,0);
                        float y=objs[k]->_Pos.at<float>(1,0)-mpMap->objs_real[i]->_Pos.at<float>(1,0);
                        float z=objs[k]->_Pos.at<float>(2,0)-mpMap->objs_real[i]->_Pos.at<float>(2,0);
                        if(sqrt(pow(x,2)+pow(y,2)+pow(z,2))>0.4)
                        {
                            continue;
                        }
                        else
                        {
                            //cout<<"62-63"<<endl;
                            objs[k]->bad=true;

                            if( mpMap->objs_real[i]->LastAddId!=mCurrentFrame.mnId)
                            {
                                mpMap->objs_real[i]->LastAddId=mCurrentFrame.mnId;
                                mpMap->objs_real[i]->confidence++;
                                objs[k]->current=true;
                            }
                            objs[k]->mnId=mpMap->objs_real[i]->mnId;
                            jh_2=false;
                        }

                    }
                    else
                        continue;
                }

            }
            if(objs[k]->_class_id==66&&(_num==0||_num2==0))
            {
                for (int i =mpMap->objs_real.size()-1; (i>=0)&&(jh_pig==true); i--)
                {
                    if(mpMap->objs_real[i]->_class_id==63)
                    {
                        float x=objs[k]->_Pos.at<float>(0,0)-mpMap->objs_real[i]->_Pos.at<float>(0,0);
                        float y=objs[k]->_Pos.at<float>(1,0)-mpMap->objs_real[i]->_Pos.at<float>(1,0);
                        float z=objs[k]->_Pos.at<float>(2,0)-mpMap->objs_real[i]->_Pos.at<float>(2,0);
                        if(sqrt(pow(x,2)+pow(y,2)+pow(z,2))>0.4)
                        {
                            continue;
                        }
                        else
                        {
                            //cout<<"66-63"<<endl;
                            objs[k]->bad=true;

                            if( mpMap->objs_real[i]->LastAddId!=mCurrentFrame.mnId)
                            {
                                mpMap->objs_real[i]->LastAddId=mCurrentFrame.mnId;
                                mpMap->objs_real[i]->confidence++;
                                objs[k]->current=true;
                            }
                            objs[k]->mnId=mpMap->objs_real[i]->mnId;
                            jh_pig=false;
                        }

                    }
                    else
                        continue;
                }

            }
            if(objs[k]->_class_id==63&&(_num==0||_num2==0))
            {
                for (int i =mpMap->objs_real.size()-1; (i>=0)&&(jh_pig_2==true); i--)
                {
                    if(mpMap->objs_real[i]->_class_id==66)
                    {
                        float x=objs[k]->_Pos.at<float>(0,0)-mpMap->objs_real[i]->_Pos.at<float>(0,0);
                        float y=objs[k]->_Pos.at<float>(1,0)-mpMap->objs_real[i]->_Pos.at<float>(1,0);
                        float z=objs[k]->_Pos.at<float>(2,0)-mpMap->objs_real[i]->_Pos.at<float>(2,0);
                        if(sqrt(pow(x,2)+pow(y,2)+pow(z,2))>0.4)
                        {
                            continue;
                        }
                        else
                        {
                            //cout<<"63-66"<<endl;
                            objs[k]->bad=true;

                            if( mpMap->objs_real[i]->LastAddId!=mCurrentFrame.mnId)
                            {
                                mpMap->objs_real[i]->LastAddId=mCurrentFrame.mnId;
                                mpMap->objs_real[i]->confidence++;
                                objs[k]->current=true;
                            }
                            objs[k]->mnId=mpMap->objs_real[i]->mnId;
                            jh_pig_2=false;
                        }

                    }
                    else
                        continue;
                }

            }

            if((_num==0||_num2==0)&&objs[k]->bad==false)
            {
                AppearNewObject=true;
                objs[k]->mnId=mpMap->objs_real.size();
                objs[k]->add_id=objs[k]->LastAddId=mCurrentFrame.mnId;
                objs[k]->confidence++;
                objs[k]->First_obj==true;
                mpMap->objs_real.push_back(objs[k]);
                objs[k]->current=true;
            }
        }

        if(mCurrentFrame.mnId>=2)
        {
            for (int i = 0; i <mpMap->objs_real.size() ; ++i)
            {
                if(mpMap->objs_real[i]->add_id==mCurrentFrame.mnId-2)
                {
                    if(mpMap->objs_real[i]->confidence<2)
                    {
                        cout<<"***"<<endl;
                        vector<MapPoint*> vpMPs = mpMap->GetAllMapPoints();
                        for(int j=0; j<vpMPs.size();j++)
                        {
                            if(vpMPs[j]->object_id==mpMap->objs_real[i]->mnId)
                            {
                                //cout<<"shanchu"<<endl;
                                vpMPs[j]->First_obj==false;
                            }
                            map<int,int>::iterator sit;
                            sit = vpMPs[j]->object_id_vector.find(mpMap->objs_real[i]->mnId);
                            if(sit != vpMPs[j]->object_id_vector.end())
                            {
                                vpMPs[j]->object_id_vector.erase(mpMap->objs_real[i]->mnId);//todo 处理删除掉的物体的点
                            }
                            for (int k = i; k <mpMap->objs_real.size() ; ++k)
                            {
                                map<int,int>::iterator sit1;
                                sit1 = vpMPs[j]->object_id_vector.find(mpMap->objs_real[k]->mnId);
                                if(sit1 != vpMPs[j]->object_id_vector.end())
                                {
                                    vpMPs[j]->object_id_vector.erase(mpMap->objs_real[k]->mnId);
                                    vpMPs[j]->object_id_vector.insert(make_pair(mpMap->objs_real[k]->mnId-1,sit1->second));//todo 删除到物体之后其余的点的号
                                }
                            }

                        }
                        mpMap->objs_real.erase(mpMap->objs_real.begin()+i);//todo 删除到物体之后其余的物体的号
                        for (int j = i; j <mpMap->objs_real.size() ; ++j)
                        {
                            mpMap->objs_real[j]->mnId-=1;
                        }
                    }
                }
            }
        }

        //todo 4 Project points seen in previous frame
        int th=15;
        int nmatches = matcher.SearchByProjection(mCurrentFrame,mLastFrame,mpMap,th);//去掉==单目
        if(nmatches<20)
        {
            fill(mCurrentFrame.mvpMapPoints.begin(),mCurrentFrame.mvpMapPoints.end(),static_cast<MapPoint*>(NULL));
            nmatches = matcher.SearchByProjection(mCurrentFrame,mLastFrame,mpMap,2*th);//todo //////////////在这剔除误匹配////////////////////
        }
        if(nmatches<20)
            return false;



        for(int i =0; i<mCurrentFrame.N; i++)
        {
            if (mCurrentFrame.mvpMapPoints[i])
            {
                MapPoint *pMP = mCurrentFrame.mvpMapPoints[i];
                if(!pMP->isBad())
                {
                    set<int>object_num;
                    for (int k = 0; k < objs.size(); ++k)
                    {
                        if(object_num.count(objs[k]->mnId))
                            continue;

                        if (mCurrentFrame.mvKeysUn[i].pt.x > objs[k]->left &&
                            mCurrentFrame.mvKeysUn[i].pt.x < objs[k]->right &&
                            mCurrentFrame.mvKeysUn[i].pt.y > objs[k]->top &&
                            mCurrentFrame.mvKeysUn[i].pt.y < objs[k]->bottom)
                        {

                            float z = mCurrentFrame.mvDepth[i];
//                            cout<<"存在："<<std::count(pMP->frame_id.begin(), pMP->frame_id.end(),mCurrentFrame.mnId-1)<<endl;
                            if(z>0&&pMP->frame_id.count(mCurrentFrame.mnId-1))
                            {
                                cv::Mat pro_x3D=mCurrentFrame.UnprojectStereo(i);/////for 画图
                                cv::Mat pro_x3Dc=mCurrentFrame.UnprojectStereo_camera(i);
                                MapPoint* pro_MP = new MapPoint(pro_x3D,mpMap,&mCurrentFrame,i);//todo 只有这里使用帧创建地图点而不是关键帧
                                objs[k]->pro_MapPonits.push_back(pro_MP);
                                objs[k]->pro_MapPoints_camera.push_back(pro_x3Dc);///for 优化
                                objs[k]->co_MapPonits.push_back(pMP);
                                cout<<mCurrentFrame.mnId<<":"<<k<<":co_obs"<<endl;
                            }

                            cv::circle(mCurrentFrame.image,mCurrentFrame.mvKeysUn[i].pt,2,cv::Scalar(0,255,0),-1);
                            cv::Mat x3D = pMP->GetWorldPos();
                            float depth = x3D.at<float>(2, 0);
                            float middle_point = objs[k]->_Pos.at<float>(2, 0);
                            if (abs(depth - middle_point) < 0.1)
                            {
                                if(objs[k]->First_obj==true)
                                {
                                    pMP->First_obj==true;
                                }

                                pMP->object_id = objs[k]->mnId;
                                pMP->frame_id.insert(mCurrentFrame.mnId);
                                map<int,int>::iterator sit;
                                sit = pMP->object_id_vector.find(pMP->object_id);
                                if(sit != pMP->object_id_vector.end())
                                {
                                    int sit_sec=sit->second;
                                    pMP->object_id_vector.erase(pMP->object_id);
                                    pMP->object_id_vector.insert(make_pair(pMP->object_id, sit_sec+1));
                                }
                                else
                                {
                                    pMP->object_id_vector.insert(make_pair(pMP->object_id,1));
                                }
                                mpMap->AddObjectMapPoints(pMP);
                                objs[k]->MapPonits.push_back(pMP);
                            }
                        }
                        object_num.insert(objs[k]->mnId);
                    }
                }
            }
        }
        for (int l = 0; l <objs.size() ; ++l) {
            if(objs[l]->current==true)
                mCurrentFrame.objects.push_back(objs[l]);
            if(objs[l]->MapPonits.size()==0)
                continue;
            int zzss=0;
            int www=0;
            for (int j = 0; j <mpMap->objs_real.size(); ++j)
            {
                if(mpMap->objs_real[j]->mnId==objs[l]->mnId)
                {
                    zzss++;
                    for (int i = 0; i <objs[l]->MapPonits.size() ; ++i)
                    {
                        int num=0;
                        for (int m = 0; m <mpMap->objs_real[j]->MapPonits.size() ; ++m)
                        {
                            cv::Mat obj_pos=objs[l]->MapPonits[i]->GetWorldPos();
                            cv::Mat obj_real_pos=mpMap->objs_real[j]->MapPonits[m]->GetWorldPos();
                            if(cv::countNonZero(obj_pos-obj_real_pos)==0)////cv::mat不可以直接==判断  重要
                            {
                                mpMap->objs_real[j]->MapPonits[m]->have_feature=objs[l]->MapPonits[i]->have_feature;
                                mpMap->objs_real[j]->MapPonits[m]->feature=objs[l]->MapPonits[i]->feature;//// 同一个点的特征没有更新
                                num++;
                                break;
                            }
                        }
                        if(num==0)
                        {
                            www++;
                            mpMap->objs_real[j]->MapPonits.push_back(objs[l]->MapPonits[i]);
                        }
                    }
                    for (int k = 0; k <objs[l]->pro_MapPonits.size(); ++k) {
                        int num=0;
                        for (int m = 0; m <mpMap->objs_real[j]->pro_MapPonits.size() ; ++m)
                        {
                            cv::Mat obj_pos=objs[l]->pro_MapPonits[k]->GetWorldPos();
                            cv::Mat obj_real_pos=mpMap->objs_real[j]->pro_MapPonits[m]->GetWorldPos();
                            if(cv::countNonZero(obj_pos-obj_real_pos)==0)////cv::mat不可以直接==判断  重要
                            {
                                mpMap->objs_real[j]->pro_MapPonits[m]->have_feature=objs[l]->pro_MapPonits[k]->have_feature;
                                mpMap->objs_real[j]->pro_MapPonits[m]->feature=objs[l]->pro_MapPonits[k]->feature;//// 同一个点的特征没有更新
                                num++;
                                break;
                            }
                        }
                        if(num==0)
                        {
                            www++;
                            mpMap->objs_real[j]->pro_MapPonits.push_back(objs[l]->pro_MapPonits[k]);
                            mpMap->objs_real[j]->pro_MapPoints_camera.push_back(objs[l]->pro_MapPoints_camera[k]);
                            mpMap->objs_real[j]->co_MapPonits.push_back(objs[l]->co_MapPonits[k]);
                        }

                    }
                }
                if(zzss!=0)
                    break;
            }
        }
        //todo 5 Optimize frame pose with all matches
        Optimizer::cout_num=true;
        Optimizer::PoseOptimization(&mCurrentFrame);
        Optimizer::cout_num=false;

        int nmatchesMap = 0;
        for(int i =0; i<mCurrentFrame.N; i++)
        {

            if(mCurrentFrame.mvpMapPoints[i])
            {
                MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
                if(mCurrentFrame.mvbOutlier[i])
                {

                    mCurrentFrame.mvpMapPoints[i]=static_cast<MapPoint*>(NULL);
                    mCurrentFrame.mvbOutlier[i]=false;
                    pMP->mbTrackInView = false;
                    pMP->mnLastFrameSeen = mCurrentFrame.mnId;
                    nmatches--;
                }
                else
                {
                    pMP->have_feature=true;
                    pMP->feature=mCurrentFrame.mvKeysUn[i];
                    if(mCurrentFrame.mvpMapPoints[i]->Observations()>0)
                        nmatchesMap++;
                }
            }
        }
        cout<<mpMap->objs_real.size()<<endl;
//        for (int n = 0; n <mpMap->objs_real.size() ; ++n) {
//            //// 用所有地图点位置的平均值作为物体的位置
//            cv::Mat avg_pos=(cv::Mat_<float>(3,1)<<(0,0,0));
//            for (int i = 0; i <mpMap->objs_real[n]->MapPonits.size() ; ++i) {
//                cv::Mat x3d=mpMap->objs_real[n]->MapPonits[i]->GetWorldPos();
//                avg_pos+=x3d;
//            }
//            avg_pos/=mpMap->objs_real[n]->MapPonits.size();
//            mpMap->objs_real[n]->_Pos=avg_pos;
//        }
        cout<<"当前帧物体："<<mCurrentFrame.objects.size()<<endl;
        return nmatchesMap>=10;
    }

    bool Tracking::TrackLocalMap()
    {
        //todo 1 update local map
        UpdateLocalMap();

        //todo 2 Search Local Points
        SearchLocalPoints();

        //todo 3 Optimize Pose
//        Optimizer::cout_num=true;
        Optimizer::PoseOptimization(&mCurrentFrame);

        cv::Mat R=mCurrentFrame.mTcw.rowRange(0,3).colRange(0,3);
        cv::Mat t=mCurrentFrame.mTcw.rowRange(0,3).col(3);

        for (int i = 0; i <mCurrentFrame.objects.size() ; ++i)
        {
            for (int j = 0; j <mpMap->objs_real.size() ; ++j)
            {
                if(mCurrentFrame.objects[i]->mnId==mpMap->objs_real[j]->mnId)
                {
//                    cv::Mat cen_pos=mpMap->objs_real[j]->_Pos;
//                    float PcX = cen_pos.at<float>(0);
//                    float PcY= cen_pos.at<float>(1);
//                    float PcZ = cen_pos.at<float>(2);
//
//                    float zz = 1.0f/PcZ;
//                    float uu=mCurrentFrame.fx*PcX*zz+mCurrentFrame.cx;
//                    float vv=mCurrentFrame.fy*PcY*zz+mCurrentFrame.cy;
//                        cv::circle(mCurrentFrame.image,cv::Point2f(uu,vv),5,cv::Scalar(0,0,256),-1);
////                      cv::putText(mCurrentFrame.image,to_string(mpMap->objs_real[j]->mnId), cv::Point2f(uu,vv), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255,255,0), 1);
                    vector<int> u_vec;////为了选择边界框
                    vector<int> v_vec;

                    for (int k = 0; k <mpMap->objs_real[j]->MapPonits.size() ; ++k)
                    {
                        MapPoint *pMP = mpMap->objs_real[j]->MapPonits[k];
                        if(pMP->have_feature)
                        {
                            cv::KeyPoint fea=pMP->feature;///// 之前的错误是只对添加到地图的那一帧的特征点计算重投影误差
                            float k_u=fea.pt.x;
                            float k_v=fea.pt.y;

                            cv::Mat P=pMP->GetWorldPos();
                            cv::Mat PPc = R*P+t;
                            float PPcX = PPc.at<float>(0);
                            float PPcY= PPc.at<float>(1);
                            float PPcZ = PPc.at<float>(2);

                            float invz = 1.0f/PPcZ;
                            float u=mCurrentFrame.fx*PPcX*invz+mCurrentFrame.cx;
                            float v=mCurrentFrame.fy*PPcY*invz+mCurrentFrame.cy;

                            float dist=(u-k_u)*(u-k_u)+(v-k_v)*(v-k_v);
                            dist=sqrt(dist);
//                            u_vec.push_back(u);
//                            v_vec.push_back(v);
//                            cv::line(mCurrentFrame.image,cv::Point2f(u,v),cv::Point2f(k_u,k_v),cv::Scalar(0,100,256),1.0);
                            cv::circle(mCurrentFrame.image,cv::Point2f(u,v),dist,cv::Scalar(256,0,0),-1);
//                      cv::putText(mCurrentFrame.image,to_string(mpMap->objs_real[j]->mnId), cv::Point2f(u,v), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255,255,0), 1);
                        }
                    }
//                    sort(u_vec.begin(),u_vec.end());
//                    sort(v_vec.begin(),v_vec.end());////从小到大
//
//                    int leftt=u_vec[0];
//                    int rightt=u_vec[u_vec.size()-1];
//                    int topp=v_vec[0];
//                    int bottomm=v_vec[v_vec.size()-1];
//                    cv::line(mCurrentFrame.image,cv::Point2f(leftt,topp),cv::Point2f(rightt,topp),cv::Scalar(0,100,256),1.5);///bgr
//                    cv::line(mCurrentFrame.image,cv::Point2f(leftt,topp),cv::Point2f(leftt,bottomm),cv::Scalar(0,100,256),1.5);///bgr
//                    cv::line(mCurrentFrame.image,cv::Point2f(rightt,topp),cv::Point2f(rightt,bottomm),cv::Scalar(0,100,256),1.5);///bgr
//                    cv::line(mCurrentFrame.image,cv::Point2f(leftt,bottomm),cv::Point2f(rightt,bottomm),cv::Scalar(0,100,256),1.5);///bgr

//                   cout<<hf<<"  "<<nf<<endl;
                }

            }

        }
//        cv::imshow("img",mCurrentFrame.image);
//        cv::waitKey(0);
        //todo 4 Discard outliers
        mnMatchesInliers = 0;
        for(int i=0; i<mCurrentFrame.N; i++)
        {
            if(mCurrentFrame.mvpMapPoints[i])
            {
                if(!mCurrentFrame.mvbOutlier[i])
                {
                    mCurrentFrame.mvpMapPoints[i]->IncreaseFound();
                        if(mCurrentFrame.mvpMapPoints[i]->Observations()>0)
                        {
                            mnMatchesInliers++;
                        }
                }
            }
        }
        //  499 297 297 297 0 mnmatchesinlier 297
        if(mCurrentFrame.mnId<mnLastRelocFrameId+mMaxFrames && mnMatchesInliers<50)
            return false;
        //cout<<"-------------------------TrackLocalMap():"<<mnMatchesInliers<<"-----------------------------------"<<endl;
        if(mnMatchesInliers<30)
            return false;
        else
            return true;
    }

    bool Tracking::NeedNewKeyFrame()
    {

        // If Local Mapping is freezed by a Loop Closure do not insert keyframes
        if(mpLocalMapper->isStopped() || mpLocalMapper->stopRequested())
            return false;
        const int nKFs = mpMap->KeyFramesInMap();
        // Do not insert keyframes if not enough frames have passed from last relocalisation
        if(mCurrentFrame.mnId<mnLastRelocFrameId+mMaxFrames && nKFs>mMaxFrames)
            return false;

        // todo Tracked MapPoints in the reference keyframe
        int nMinObs = 3;
        if(nKFs<=2)
            nMinObs=2;
        int nRefMatches = mpReferenceKF->TrackedMapPoints(nMinObs);//todo 参考关键帧中地图点被三个以上关键帧观测的个数
        // Local Mapping accept keyframes?
        bool bLocalMappingIdle = mpLocalMapper->AcceptKeyFrames();

        // todo Check how many "close" points are being tracked and how many could be potentially created.
        int nNonTrackedClose = 0;
        int nTrackedClose= 0;
        for(int i =0; i<mCurrentFrame.N; i++)
         {
             if(mCurrentFrame.mvDepth[i]>0 && mCurrentFrame.mvDepth[i]<mThDepth)
              {
                 if(mCurrentFrame.mvpMapPoints[i] && !mCurrentFrame.mvbOutlier[i])
                     nTrackedClose++;
                 else
                     nNonTrackedClose++;
               }
         }
        bool bNeedToInsertClose = (nTrackedClose<100) && (nNonTrackedClose>70);//todo 什么东西

        // Thresholds
        float thRefRatio = 0.75f;
        if(nKFs<2)
            thRefRatio = 0.4f;
        //todo condition 1（任意一个发生）:
        //todo  a.很久没有新的关键帧了
        //todo  b. LocalMapping空闲
        //todo c.当前追踪不行了
        //todo condition 2:
        //todo  当前帧的MapPoint和ref keyframe重复率低
        // Condition 1a: More than "MaxFrames" have passed from last keyframe insertion
        const bool c1a = mCurrentFrame.mnId>=mnLastKeyFrameId+mMaxFrames;//todo
        // Condition 1b: More than "MinFrames" have passed and Local Mapping is idle
        const bool c1b = (mCurrentFrame.mnId>=mnLastKeyFrameId+mMinFrames && bLocalMappingIdle);//todo
        //Condition 1c: tracking is weak
        const bool c1c = (mnMatchesInliers<nRefMatches*0.25 || bNeedToInsertClose) ;//todo
        // Condition 2: Few tracked points compared to reference keyframe. Lots of visual odometry compared to map matches.
        const bool c2 = ((mnMatchesInliers<nRefMatches*thRefRatio|| bNeedToInsertClose) && mnMatchesInliers>15);


        bool ok1 = AppearNewObject ;

        if((c1a||c1b||c1c)&&c2)
        {
            if(bLocalMappingIdle)
                return true;
            else
            {
                mpLocalMapper->InterruptBA();//todo 需要新的关键帧
                if(mpLocalMapper->KeyframesInQueue()<3)
                    return true;
                else
                     return false;
            }
        }
        else
            return false;
    }

    void Tracking::CreateNewKeyFrame()
    {

        if(!mpLocalMapper->SetNotStop(true))
            return;
        KeyFrame* pKF = new KeyFrame(mCurrentFrame,mpMap,mpKeyFrameDB);
        //todo mvpMapPoints(F.mvpMapPoints)初始化时把当前帧的mvpMapPoints给了pkf的mvpMapPoints，所以已经存在的好的地图点只需添加addobservation，不需要再addmappoint
        mpReferenceKF = pKF;//todo 局部地图中用,当前关键帧作为参考关键帧
        mCurrentFrame.mpReferenceKF = pKF;//todo 当前帧的参考关键帧
        mCurrentFrame.UpdatePoseMatrices();

        //todo 所有深度大于0的特征点
        vector<pair<float,int> > vDepthIdx;
        vDepthIdx.reserve(mCurrentFrame.N);
        for(int i=0; i<mCurrentFrame.N; i++)
        {
            float z = mCurrentFrame.mvDepth[i];
            if(z>0)
            {
                vDepthIdx.push_back(make_pair(z,i));
            }
        }
        pKF->objects=mCurrentFrame.objects;

        if(!vDepthIdx.empty())
        {
            //todo 按深度值排序,从小到大，为了剔除远处的点
            sort(vDepthIdx.begin(),vDepthIdx.end());
            int nPoints = 0;
            for(size_t j=0; j<vDepthIdx.size();j++)
            {
                int i = vDepthIdx[j].second;//todo 第几个特征点
                bool bCreateNew = false;
                //todo 如果深度大于0的特征点没有对应的地图点或者地图点没有被关键帧观测到，就重新创建
                MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
                if(!pMP)
                    bCreateNew = true;
                else if(pMP->Observations()<1)
                {
                    bCreateNew = true;
                    mCurrentFrame.mvpMapPoints[i] = static_cast<MapPoint*>(NULL);
                }
                if(bCreateNew)
                {
                    cv::Mat x3D = mCurrentFrame.UnprojectStereo(i);
                    MapPoint* pNewMP = new MapPoint(x3D,pKF,mpMap);
                    pNewMP->AddObservation(pKF,i);//todo 新创建的的点添加关键帧观测
                    pKF->AddMapPoint(pNewMP,i);
                    pNewMP->ComputeDistinctiveDescriptors();
                    pNewMP->UpdateNormalAndDepth();
                    mpMap->AddMapPoint(pNewMP);
                    mCurrentFrame.mvpMapPoints[i]=pNewMP;//todo 重新对应上
                    nPoints++;
                }
                else
                {
                   //pMP->AddObservation(pKF,i);
                    nPoints++;//todo 已存在的点不用添加一次观测吗？？？
                }

                if(vDepthIdx[j].first>mThDepth && nPoints>100)//todo 排序的目的，不要深度值太大的点，而且当前关键帧观测的点数达到100
                    break;
            }
        }
        mpLocalMapper->InsertKeyFrame(pKF);//todo 在局部地图中插入关键帧
        mpLocalMapper->SetNotStop(false);
        mnLastKeyFrameId = mCurrentFrame.mnId;//todo 上一个关键帧的id是当前帧的id
        mpLastKeyFrame = pKF;//todo 上一个关键帧就是刚创建的这个关键帧
        //cout<<"CreateNewKeyFrame:"<<mCurrentFrame.mnId<<"  "<<pKF->mnId<<endl;
    }

    void Tracking::SearchLocalPoints()
    {
        // Do not search map points already matched
        for(vector<MapPoint*>::iterator vit=mCurrentFrame.mvpMapPoints.begin(), vend=mCurrentFrame.mvpMapPoints.end(); vit!=vend; vit++)
        {
            MapPoint* pMP = *vit;
            if(pMP)
            {
                if(pMP->isBad())
                {
                    *vit = static_cast<MapPoint*>(NULL);
                }
                else
                {
                    pMP->IncreaseVisible();//todo 可见次数加１
                    pMP->mnLastFrameSeen = mCurrentFrame.mnId;//todo 上次看见这个点是在当前帧，用于去除已经匹配的点
                    pMP->mbTrackInView = false;//todo 已经匹配的点不参与
                }
            }
        }
        int nToMatch=0;
        // Project points in frame and check its visibility
        for(vector<MapPoint*>::iterator vit=mvpLocalMapPoints.begin(), vend=mvpLocalMapPoints.end(); vit!=vend; vit++)
        {
            MapPoint* pMP = *vit;
            if(pMP->mnLastFrameSeen == mCurrentFrame.mnId)
                continue;
            if(pMP->isBad())
                continue;
            // Project (this fills MapPoint variables for matching)
            //todo 判断一个点是否在视野内。如果当前的地图点在视野里，那么观测到该点的帧数加1，nToMatch计数器+1
            if(mCurrentFrame.isInFrustum(pMP,0.5))
            {
                pMP->IncreaseVisible();
                nToMatch++;
            }

        }
        // todo 只有在视野范围内的MapPoints才参与之后的投影匹配
        if(nToMatch>0)
        {
            ORBmatcher matcher(0.8);
            int th=3;
            // If the camera has been relocalised recently, perform a coarser search
            if(mCurrentFrame.mnId<mnLastRelocFrameId+2)
                th=5;
            matcher.SearchByProjection(mCurrentFrame,mvpLocalMapPoints,mpMap,th);
        }

    }

    void Tracking::UpdateLocalMap()
    {
        // todo This is for visualization
        mpMap->SetReferenceMapPoints(mvpLocalMapPoints);

        // Update
        UpdateLocalKeyFrames();
        UpdateLocalPoints();
    }

    void Tracking::UpdateLocalPoints()
    {
        mvpLocalMapPoints.clear();
        mvpObjectMapPoints.clear();
        mvcullingMapPoints.clear();
        vector<Object*> objects;
        objects=mCurrentFrame.objects;



        for(vector<KeyFrame*>::const_iterator itKF=mvpLocalKeyFrames.begin(), itEndKF=mvpLocalKeyFrames.end(); itKF!=itEndKF; itKF++)
        {
            KeyFrame* pKF = *itKF;
            const vector<MapPoint*> vpMPs = pKF->GetMapPointMatches();
            for(vector<MapPoint*>::const_iterator itMP=vpMPs.begin(), itEndMP=vpMPs.end(); itMP!=itEndMP; itMP++)
            {
                MapPoint* pMP = *itMP;
                if(!pMP)
                    continue;
                if(pMP->mnTrackReferenceForFrame==mCurrentFrame.mnId)
                    continue;
                if(!pMP->isBad())
                {
                    mvpLocalMapPoints.push_back(pMP);
                    pMP->mnTrackReferenceForFrame=mCurrentFrame.mnId;
                }
            }
        }


//        //todo 尝试4 将物体点放入局部点
//        for (int i = 0; i <objects.size() ; ++i)
//        {
//            for (int j = 0; j <mpMap->objs_real.size() ; ++j)
//            {
//                if(objects[i]->mnId==mpMap->objs_real[j]->mnId)
//                {
//                    for (int k = 0; k <mpMap->objs_real[j]->MapPonits.size() ; ++k)
//                    {
//                        int num=0;
//                        for (int l = 0; l <mvpLocalMapPoints.size() ; ++l)
//                        {
//                            if(mpMap->objs_real[j]->MapPonits[k]==mvpLocalMapPoints[l])///地图点相等合理吗？
//                            {
//                                num++;
//                                break;
//                            }
//                        }
//                        if(num==0)
//                        {
//                            if(!mpMap->objs_real[j]->MapPonits[k]->isBad())
//                            {
//                                mvpLocalMapPoints.push_back(mpMap->objs_real[j]->MapPonits[k]);
//                                mvpObjectMapPoints.push_back(mpMap->objs_real[j]->MapPonits[k]);
//                            }
//
//                        }
//
//                    }
//                }
//
//            }
//
//        }
    }

    void Tracking::UpdateLocalKeyFrames()//todo mvpLocalKeyFrames
    {
        // Each map point vote for the keyframes in which it has been observed
        //todo 1 得到当前帧地图点对应的观测关键帧
        //todo 第一个表示当前帧与哪个关键帧有共视关系，第二个参数表示共视点的个数（权重）。
        map<KeyFrame*,int> keyframeCounter;
        for(int i=0; i<mCurrentFrame.N; i++)
        {
            if(mCurrentFrame.mvpMapPoints[i])
            {
                MapPoint* pMP = mCurrentFrame.mvpMapPoints[i];
                if(!pMP->isBad())
                {
                    const map<KeyFrame*,size_t> observations = pMP->GetObservations();
                    for(map<KeyFrame*,size_t>::const_iterator it=observations.begin(), itend=observations.end(); it!=itend; it++)
                        keyframeCounter[it->first]++;
                }
                else
                {
                    mCurrentFrame.mvpMapPoints[i]=NULL;
                }
            }
        }
        //cout<<keyframeCounter.size()<<endl;
        if(keyframeCounter.empty())
            return;
        int max=0;
        KeyFrame* pKFmax= static_cast<KeyFrame*>(NULL);
        mvpLocalKeyFrames.clear();
        mvpLocalKeyFrames.reserve(3*keyframeCounter.size());
        // All keyframes that observe a map point are included in the local map. Also check which keyframe shares most points
        //todo 2 得到共同观测点数最多的关键帧
        for(map<KeyFrame*,int>::const_iterator it=keyframeCounter.begin(), itEnd=keyframeCounter.end(); it!=itEnd; it++)
        {
            KeyFrame* pKF = it->first;

            if(pKF->isBad())
                continue;

            if(it->second>max)
            {
                max=it->second;
                pKFmax=pKF;
            }

            mvpLocalKeyFrames.push_back(it->first);
            pKF->mnTrackReferenceForFrame = mCurrentFrame.mnId;//todo 为了判断是不是第一次共识关系加入的
        }
        //todo 3 得到局部关键帧相连的关键帧10个,父关键帧，子关键帧
        for(vector<KeyFrame*>::const_iterator itKF=mvpLocalKeyFrames.begin(), itEndKF=mvpLocalKeyFrames.end(); itKF!=itEndKF; itKF++)
        {
            // todo Limit the number of keyframes
            // todo 可以提高
            if(mvpLocalKeyFrames.size()>80)
                break;

            KeyFrame* pKF = *itKF;
            const vector<KeyFrame*> vNeighs = pKF->GetBestCovisibilityKeyFrames(10);
            for(vector<KeyFrame*>::const_iterator itNeighKF=vNeighs.begin(), itEndNeighKF=vNeighs.end(); itNeighKF!=itEndNeighKF; itNeighKF++)
            {
                KeyFrame* pNeighKF = *itNeighKF;
                if(!pNeighKF->isBad())
                {
                    if(pNeighKF->mnTrackReferenceForFrame!=mCurrentFrame.mnId)
                    {
                        mvpLocalKeyFrames.push_back(pNeighKF);
                        pNeighKF->mnTrackReferenceForFrame=mCurrentFrame.mnId;
                        break;
                    }
                }
            }

            const set<KeyFrame*> spChilds = pKF->GetChilds();
            for(set<KeyFrame*>::const_iterator sit=spChilds.begin(), send=spChilds.end(); sit!=send; sit++)
            {
                KeyFrame* pChildKF = *sit;
                if(!pChildKF->isBad())
                {
                    if(pChildKF->mnTrackReferenceForFrame!=mCurrentFrame.mnId)
                    {
                        mvpLocalKeyFrames.push_back(pChildKF);
                        pChildKF->mnTrackReferenceForFrame=mCurrentFrame.mnId;
                        break;
                    }
                }
            }

            KeyFrame* pParent = pKF->GetParent();
            if(pParent)
            {
                if(pParent->mnTrackReferenceForFrame!=mCurrentFrame.mnId)
                {
                    mvpLocalKeyFrames.push_back(pParent);
                    pParent->mnTrackReferenceForFrame=mCurrentFrame.mnId;
                    break;
                }
            }

        }

        if(pKFmax)
        {
            mpReferenceKF = pKFmax;//todo 和当前帧共同可见点数最多的参考关键帧
            mCurrentFrame.mpReferenceKF = mpReferenceKF;
        }

    }

    bool Tracking::Relocalization()
    {
        // Compute Bag of Words Vector
        mCurrentFrame.ComputeBoW();

        // Relocalization is performed when tracking is lost
        // Track Lost: Query KeyFrame Database for keyframe candidates for relocalisation
        vector<KeyFrame*> vpCandidateKFs = mpKeyFrameDB->DetectRelocalizationCandidates(&mCurrentFrame);

        if(vpCandidateKFs.empty())
            return false;

        const int nKFs = vpCandidateKFs.size();

        // We perform first an ORB matching with each candidate
        // If enough matches are found we setup a PnP solver
        ORBmatcher matcher(0.75,true);

        vector<PnPsolver*> vpPnPsolvers;
        vpPnPsolvers.resize(nKFs);

        vector<vector<MapPoint*> > vvpMapPointMatches;
        vvpMapPointMatches.resize(nKFs);

        vector<bool> vbDiscarded;
        vbDiscarded.resize(nKFs);

        int nCandidates=0;

        for(int i=0; i<nKFs; i++)
        {
            KeyFrame* pKF = vpCandidateKFs[i];
            if(pKF->isBad())
                vbDiscarded[i] = true;
            else
            {
                int nmatches = matcher.SearchByBoW(pKF,mCurrentFrame,vvpMapPointMatches[i]);
                if(nmatches<15)
                {
                    vbDiscarded[i] = true;
                    continue;
                }
                else
                {
                    PnPsolver* pSolver = new PnPsolver(mCurrentFrame,vvpMapPointMatches[i]);
                    pSolver->SetRansacParameters(0.99,10,300,4,0.5,5.991);
                    vpPnPsolvers[i] = pSolver;
                    nCandidates++;
                }
               // cout<<"------------relocalization track:"<<nmatches<<"---------------------------"<<endl;
            }
        }

        // Alternatively perform some iterations of P4P RANSAC
        // Until we found a camera pose supported by enough inliers
        bool bMatch = false;
        ORBmatcher matcher2(0.9,true);

        while(nCandidates>0 && !bMatch)
        {
            for(int i=0; i<nKFs; i++)
            {
                if(vbDiscarded[i])
                    continue;

                // Perform 5 Ransac Iterations
                vector<bool> vbInliers;
                int nInliers;
                bool bNoMore;

                PnPsolver* pSolver = vpPnPsolvers[i];
                cv::Mat Tcw = pSolver->iterate(5,bNoMore,vbInliers,nInliers);

                // If Ransac reachs max. iterations discard keyframe
                if(bNoMore)
                {
                    vbDiscarded[i]=true;
                    nCandidates--;
                }

                // If a Camera Pose is computed, optimize
                if(!Tcw.empty())
                {
                    Tcw.copyTo(mCurrentFrame.mTcw);

                    set<MapPoint*> sFound;

                    const int np = vbInliers.size();

                    for(int j=0; j<np; j++)
                    {
                        if(vbInliers[j])
                        {
                            mCurrentFrame.mvpMapPoints[j]=vvpMapPointMatches[i][j];
                            sFound.insert(vvpMapPointMatches[i][j]);
                        }
                        else
                            mCurrentFrame.mvpMapPoints[j]=NULL;
                    }

                    int nGood = Optimizer::PoseOptimization(&mCurrentFrame);

                    if(nGood<10)
                        continue;

                    for(int io =0; io<mCurrentFrame.N; io++)
                        if(mCurrentFrame.mvbOutlier[io])
                        {
                            mCurrentFrame.mvpMapPoints[io]=static_cast<MapPoint*>(NULL);
                            //mCurrentFrame.mvpMapPoints[io]->object_class=mCurrentFrame.mvKeys[io].class_id;
                        }

                    // If few inliers, search by projection in a coarse window and optimize again
                    if(nGood<50)
                    {
                        int nadditional =matcher2.SearchByProjection(mCurrentFrame,vpCandidateKFs[i],sFound,10,100);

                        if(nadditional+nGood>=50)
                        {
                            nGood = Optimizer::PoseOptimization(&mCurrentFrame);

                            // If many inliers but still not enough, search by projection again in a narrower window
                            // the camera has been already optimized with many points
                            if(nGood>30 && nGood<50)
                            {
                                sFound.clear();
                                for(int ip =0; ip<mCurrentFrame.N; ip++)
                                    if(mCurrentFrame.mvpMapPoints[ip])
                                        sFound.insert(mCurrentFrame.mvpMapPoints[ip]);
                                nadditional =matcher2.SearchByProjection(mCurrentFrame,vpCandidateKFs[i],sFound,3,64);

                                // Final optimization
                                if(nGood+nadditional>=50)
                                {
                                    nGood = Optimizer::PoseOptimization(&mCurrentFrame);

                                    for(int io =0; io<mCurrentFrame.N; io++)
                                        if(mCurrentFrame.mvbOutlier[io])
                                        {mCurrentFrame.mvpMapPoints[io]=NULL;
                                            //mCurrentFrame.mvpMapPoints[io]->object_class=mCurrentFrame.mvKeys[io].class_id;
                                        }
                                }
                            }
                        }
                    }


                    // If the pose is supported by enough inliers stop ransacs and continue
                    if(nGood>=50)
                    {
                        bMatch = true;
                        break;
                    }
                }
            }
        }

        if(!bMatch)
        {
            return false;
        }
        else
        {
            mnLastRelocFrameId = mCurrentFrame.mnId;
            return true;
        }

    }

    void Tracking::Reset()
    {

        cout << "System Reseting" << endl;
        if(mpViewer)
        {
            mpViewer->RequestStop();
            while(!mpViewer->isStopped())
                usleep(3000);
        }

        // Reset Local Mapping
        cout << "Reseting Local Mapper...";
        mpLocalMapper->RequestReset();
        cout << " done" << endl;

        // Reset Loop Closing
        cout << "Reseting Loop Closing...";
        mpLoopClosing->RequestReset();
        cout << " done" << endl;

        // Clear BoW Database
        cout << "Reseting Database...";
        mpKeyFrameDB->clear();
        cout << " done" << endl;

        // Clear Map (this erase MapPoints and KeyFrames)
        mpMap->clear();

        KeyFrame::nNextId = 0;
        Frame::nNextId = 0;
        mState = NO_IMAGES_YET;
        mlRelativeFramePoses.clear();
        mlpReferences.clear();
        mlFrameTimes.clear();
        mlbLost.clear();

        if(mpViewer)
            mpViewer->Release();
    }
    void Tracking::InformOnlyTracking(const bool &flag)
    {
        mbOnlyTracking = flag;
    }

}

