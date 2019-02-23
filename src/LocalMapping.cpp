//
// Created by zss on 18-4-18.
//
#include "LocalMapping.h"

#include "ORBmatcher.h"
#include "Optimizer.h"
#include "Object.h"
namespace myslam
{
    LocalMapping::LocalMapping(Map *pMap):  mbResetRequested(false), mbFinishRequested(false), mbFinished(true), mpMap(pMap),
            mbAbortBA(false), mbStopped(false), mbStopRequested(false), mbNotStop(false), mbAcceptKeyFrames(true), mbMonocular(false)
    {
    }

    void LocalMapping::SetLoopCloser(LoopClosing* pLoopCloser)
    {
        mpLoopCloser = pLoopCloser;
    }

    void LocalMapping::SetTracker(Tracking *pTracker)
    {
        mpTracker=pTracker;
    }

    void LocalMapping::Run()//system new thread
    {
        mbFinished = false;
        while(1)
        {
            // Tracking will see that Local Mapping is busy
            SetAcceptKeyFrames(false);

            // Check if there are keyframes in the queue
            if(CheckNewKeyFrames())
            {
                // BoW conversion and insertion in Map
                ProcessNewKeyFrame();//todo 找出当前关键帧
                // Check recent MapPoints
                //todo 对于ProcessNewKeyFrame和CreateNewMapPoints中最近添加的MapPoints进行检查剔除
                MapPointCulling();
                // Triangulate new MapPoints
                CreateNewMapPoints();
                if(!CheckNewKeyFrames())//todo 空了
                {
                    // Find more matches in neighbor keyframes and fuse point duplications
                    SearchInNeighbors();//todo 更新并融合当前关键帧以及两级相连（共视关键帧及其共视关键帧）的关键帧的地图点
                }

                mbAbortBA = false;
                if(!CheckNewKeyFrames() && !stopRequested())
                {
                    //cout<<mpMap->KeyFramesInMap()<<endl;
                    // todo ***只要添加一个关键帧进来就执行一次lcoal BA***
                    // todo ***98个关键帧***
                    if(mpMap->KeyFramesInMap()>2)
                        Optimizer::LocalBundleAdjustment(mpCurrentKeyFrame,&mbAbortBA, mpMap);
                    // Check redundant local Keyframes
                    KeyFrameCulling();
                }

                mpLoopCloser->InsertKeyFrame(mpCurrentKeyFrame);
            }
            else if(Stop())
            {
                // Safe area to stop
                while(isStopped() && !CheckFinish())
                {
                    usleep(3000);
                }
                if(CheckFinish())
                    break;
            }

            ResetIfRequested();

            // Tracking will see that Local Mapping is busy
            SetAcceptKeyFrames(true);

            if(CheckFinish())
                break;

            usleep(3000);
        }

        SetFinish();
    }

    void LocalMapping::InsertKeyFrame(KeyFrame *pKF)
    {
        unique_lock<mutex> lock(mMutexNewKFs);
        mlNewKeyFrames.push_back(pKF);
        mbAbortBA=true;
    }

    bool LocalMapping::CheckNewKeyFrames()//run
    {
        unique_lock<mutex> lock(mMutexNewKFs);
        return(!mlNewKeyFrames.empty());
    }

    void LocalMapping::ProcessNewKeyFrame()//run
    {
        {
            unique_lock<mutex> lock(mMutexNewKFs);
            mpCurrentKeyFrame = mlNewKeyFrames.front();
            mlNewKeyFrames.pop_front();
        }

        // todo 1:Compute Bags of Words structures 便于后面三角化恢复新地图点
        mpCurrentKeyFrame->ComputeBoW();

        // todo 2:Associate MapPoints to the new keyframe and update normal and descriptor
        //todo 将TrackLocalMap中匹配上的地图点绑定到当前关键帧（在Tracking线程中只是通过匹配进行局部地图跟踪，优化当前关键帧姿态），地图点没有和关键帧对应上
        //todo 在graph中加入当前关键帧作为node，并更新edge。而CreateNewMapPoint()中则通过当前关键帧，在局部地图中添加与新的地图点；
        const vector<MapPoint*> vpMapPointMatches = mpCurrentKeyFrame->GetMapPointMatches();
        int zss=0;
        for(size_t i=0; i<vpMapPointMatches.size(); i++)
        {
            MapPoint* pMP = vpMapPointMatches[i];
            if(pMP)
            {
                if(!pMP->isBad())
                {
                    //todo 指的是关键帧是否在点的观测里
                    if(!pMP->IsInKeyFrame(mpCurrentKeyFrame))//todo 为什么会没有呢,现在知道了吧
                    {
                        pMP->AddObservation(mpCurrentKeyFrame, i);
                        pMP->UpdateNormalAndDepth();//todo 因为需要更新吗，所以在这里统一加观测
                        pMP->ComputeDistinctiveDescriptors();
                    }
                    else // this can only happen for new stereo points inserted by the Tracking
                    {
                        //todo 对，这个是最近添加的点，不是之前存在的点
                        mlpRecentAddedMapPoints.push_back(pMP);
                    }
                }
            }
            else
                zss++;
        }
        //cout<<"总数："<<vpMapPointMatches.size()<<endl;
        //cout<<"外点数："<<zss<<endl;

        // todo 3:Update links in the Covisibility Graph
        // todo 更新加入当前关键帧之后关键帧之间的连接关系，包括更新Covisibility图和Essential图
        // todo （最小生成树spanning tree，共视关系好的边subset of edges from covisibility graph with high covisibility (θ=100)， 闭环边）。
        mpCurrentKeyFrame->UpdateConnections();

        // todo 4:Insert Keyframe in Map
        mpMap->AddKeyFrame(mpCurrentKeyFrame);
    }

    void LocalMapping::MapPointCulling()//run
    {
        //cout<<"-------MapPointCulling-------"<<endl;
        // Check Recent Added MapPoints
        list<MapPoint*>::iterator lit = mlpRecentAddedMapPoints.begin();
        const unsigned long int nCurrentKFid = mpCurrentKeyFrame->mnId;

        int nThObs;
        if(mbMonocular)
            nThObs = 2;
        else
            nThObs = 3;
        const int cnThObs = nThObs;
        while(lit!=mlpRecentAddedMapPoints.end())
        {
            MapPoint* pMP = *lit;
            //todo 1. 直接剔除坏点，只可能是CreateNewMapPoints中最近添加的MapPoints
            if(pMP->isBad())
            {
                lit = mlpRecentAddedMapPoints.erase(lit);
            }
             //todo 2. 跟踪到该点的帧数比预计可以观测到该点的帧数的比例小于0.25，则剔除该点
            else if(pMP->GetFoundRatio()<0.25f )
            {
                pMP->SetBadFlag();
                lit = mlpRecentAddedMapPoints.erase(lit);
            }
            //todo 3. 从该点建立开始，到现在已经过了2个关键帧以上，但是观测到该点的关键帧数小于cnThObs(单目是2，非单目是3)，则剔除该点
            else if(((int)nCurrentKFid-(int)pMP->mnFirstKFid)>=2 && pMP->Observations()<=cnThObs)
            {
                pMP->SetBadFlag();
                lit = mlpRecentAddedMapPoints.erase(lit);
            }
            //todo 4. 从建立该点开始，已经过了3个关键帧而没有被剔除，则认为是质量高的点，因此没有SetBadFlag()，仅从队列中删除，放弃继续对该MapPoint的检测
            else if(((int)nCurrentKFid-(int)pMP->mnFirstKFid)>=3)
                lit = mlpRecentAddedMapPoints.erase(lit);
            else
                lit++;
        }
        //cout<<"已经是坏的点:"<<already_erase_points<<endl;
        //cout<<"已经是坏的物体点:"<<already_erase_object_points<<endl;
        //cout<<"剔除点:"<<erase_points<<endl;
        //cout<<"剔除物体点:"<<erase_object_points<<endl;
    }

    void LocalMapping::CreateNewMapPoints()//run
    {
        //cout<<"mpCurrentKeyFrame:"<<mpCurrentKeyFrame->objects.size()<<endl;
        // todo Retrieve neighbor keyframes in covisibility graph
        int nn = 10;
        if(mbMonocular)
            nn=20;
        const vector<KeyFrame*> vpNeighKFs = mpCurrentKeyFrame->GetBestCovisibilityKeyFrames(nn);

        ORBmatcher matcher(0.6,false);

        cv::Mat Rcw1 = mpCurrentKeyFrame->GetRotation();
        cv::Mat Rwc1 = Rcw1.t();
        cv::Mat tcw1 = mpCurrentKeyFrame->GetTranslation();
        cv::Mat Tcw1(3,4,CV_32F);
        Rcw1.copyTo(Tcw1.colRange(0,3));
        tcw1.copyTo(Tcw1.col(3));
        cv::Mat Ow1 = mpCurrentKeyFrame->GetCameraCenter();

        const float &fx1 = mpCurrentKeyFrame->fx;
        const float &fy1 = mpCurrentKeyFrame->fy;
        const float &cx1 = mpCurrentKeyFrame->cx;
        const float &cy1 = mpCurrentKeyFrame->cy;
        const float &invfx1 = mpCurrentKeyFrame->invfx;
        const float &invfy1 = mpCurrentKeyFrame->invfy;

        const float ratioFactor = 1.5f*mpCurrentKeyFrame->mfScaleFactor;

        int nnew=0;

        // todo Search matches with epipolar restriction and triangulate
        for(size_t i=0; i<vpNeighKFs.size(); i++)
        {
            if(i>0 && CheckNewKeyFrames())//todo 又出现新的关键帧就不再创建地图点
                return;

            KeyFrame* pKF2 = vpNeighKFs[i];//todo 邻居关键帧

            // todo Check first that baseline is not too short
            cv::Mat Ow2 = pKF2->GetCameraCenter();
            cv::Mat vBaseline = Ow2-Ow1;
            const float baseline = cv::norm(vBaseline);

            if(!mbMonocular)
            {
                if(baseline<pKF2->mb)
                    continue;
            }
            else
            {
                const float medianDepthKF2 = pKF2->ComputeSceneMedianDepth(2);
                const float ratioBaselineDepth = baseline/medianDepthKF2;

                if(ratioBaselineDepth<0.01)
                    continue;
            }

            // todo Compute Fundamental Matrix
            cv::Mat F12 = ComputeF12(mpCurrentKeyFrame,pKF2);

            // todo Search matches that fullfil epipolar constraint
            vector<pair<size_t,size_t> > vMatchedIndices;
            matcher.SearchForTriangulation(mpCurrentKeyFrame,pKF2,F12,vMatchedIndices,false);

            cv::Mat Rcw2 = pKF2->GetRotation();
            cv::Mat Rwc2 = Rcw2.t();
            cv::Mat tcw2 = pKF2->GetTranslation();
            cv::Mat Tcw2(3,4,CV_32F);
            Rcw2.copyTo(Tcw2.colRange(0,3));
            tcw2.copyTo(Tcw2.col(3));

            const float &fx2 = pKF2->fx;
            const float &fy2 = pKF2->fy;
            const float &cx2 = pKF2->cx;
            const float &cy2 = pKF2->cy;
            const float &invfx2 = pKF2->invfx;
            const float &invfy2 = pKF2->invfy;

            // todo Triangulate each match
            const int nmatches = vMatchedIndices.size();
            for(int ikp=0; ikp<nmatches; ikp++)
            {
                const int &idx1 = vMatchedIndices[ikp].first;
                const int &idx2 = vMatchedIndices[ikp].second;

                const cv::KeyPoint &kp1 = mpCurrentKeyFrame->mvKeysUn[idx1];
                const float kp1_ur=mpCurrentKeyFrame->mvuRight[idx1];
                bool bStereo1 = kp1_ur>=0;

                const cv::KeyPoint &kp2 = pKF2->mvKeysUn[idx2];
                const float kp2_ur = pKF2->mvuRight[idx2];
                bool bStereo2 = kp2_ur>=0;

                //todo  Check parallax between rays
                cv::Mat xn1 = (cv::Mat_<float>(3,1) << (kp1.pt.x-cx1)*invfx1, (kp1.pt.y-cy1)*invfy1, 1.0);
                cv::Mat xn2 = (cv::Mat_<float>(3,1) << (kp2.pt.x-cx2)*invfx2, (kp2.pt.y-cy2)*invfy2, 1.0);

                cv::Mat ray1 = Rwc1*xn1;
                cv::Mat ray2 = Rwc2*xn2;
                const float cosParallaxRays = ray1.dot(ray2)/(cv::norm(ray1)*cv::norm(ray2));

                float cosParallaxStereo = cosParallaxRays+1;
                float cosParallaxStereo1 = cosParallaxStereo;
                float cosParallaxStereo2 = cosParallaxStereo;

                if(bStereo1)
                    cosParallaxStereo1 = cos(2*atan2(mpCurrentKeyFrame->mb/2,mpCurrentKeyFrame->mvDepth[idx1]));
                else if(bStereo2)

                    cosParallaxStereo2 = cos(2*atan2(pKF2->mb/2,pKF2->mvDepth[idx2]));

                cosParallaxStereo = min(cosParallaxStereo1,cosParallaxStereo2);

                cv::Mat x3D;
                if(cosParallaxRays<cosParallaxStereo && cosParallaxRays>0 && (bStereo1 || bStereo2 || cosParallaxRays<0.9998))
                {
                    // todo Linear Triangulation Method
                    cv::Mat A(4,4,CV_32F);
                    A.row(0) = xn1.at<float>(0)*Tcw1.row(2)-Tcw1.row(0);
                    A.row(1) = xn1.at<float>(1)*Tcw1.row(2)-Tcw1.row(1);
                    A.row(2) = xn2.at<float>(0)*Tcw2.row(2)-Tcw2.row(0);
                    A.row(3) = xn2.at<float>(1)*Tcw2.row(2)-Tcw2.row(1);

                    cv::Mat w,u,vt;
                    cv::SVD::compute(A,w,u,vt,cv::SVD::MODIFY_A| cv::SVD::FULL_UV);

                    x3D = vt.row(3).t();

                    if(x3D.at<float>(3)==0)
                        continue;

                    // todo Euclidean coordinates
                    x3D = x3D.rowRange(0,3)/x3D.at<float>(3);

                }
                else if(bStereo1 && cosParallaxStereo1<cosParallaxStereo2)
                {
                    x3D = mpCurrentKeyFrame->UnprojectStereo(idx1);
                }
                else if(bStereo2 && cosParallaxStereo2<cosParallaxStereo1)
                {
                    x3D = pKF2->UnprojectStereo(idx2);
                }
                else
                    continue; //No stereo and very low parallax

                cv::Mat x3Dt = x3D.t();

                //todo Check triangulation in front of cameras
                float z1 = Rcw1.row(2).dot(x3Dt)+tcw1.at<float>(2);
                if(z1<=0)
                    continue;

                float z2 = Rcw2.row(2).dot(x3Dt)+tcw2.at<float>(2);
                if(z2<=0)
                    continue;

                //todo Check reprojection error in first keyframe
                const float &sigmaSquare1 = mpCurrentKeyFrame->mvLevelSigma2[kp1.octave];
                const float x1 = Rcw1.row(0).dot(x3Dt)+tcw1.at<float>(0);
                const float y1 = Rcw1.row(1).dot(x3Dt)+tcw1.at<float>(1);
                const float invz1 = 1.0/z1;

                if(!bStereo1)
                {
                    float u1 = fx1*x1*invz1+cx1;
                    float v1 = fy1*y1*invz1+cy1;
                    float errX1 = u1 - kp1.pt.x;
                    float errY1 = v1 - kp1.pt.y;
                    if((errX1*errX1+errY1*errY1)>5.991*sigmaSquare1)
                        continue;
                }
                else
                {
                    float u1 = fx1*x1*invz1+cx1;
                    float u1_r = u1 - mpCurrentKeyFrame->mbf*invz1;
                    float v1 = fy1*y1*invz1+cy1;
                    float errX1 = u1 - kp1.pt.x;
                    float errY1 = v1 - kp1.pt.y;
                    float errX1_r = u1_r - kp1_ur;
                    if((errX1*errX1+errY1*errY1+errX1_r*errX1_r)>7.8*sigmaSquare1)
                        continue;
                }

                //todo Check reprojection error in second keyframe
                const float sigmaSquare2 = pKF2->mvLevelSigma2[kp2.octave];
                const float x2 = Rcw2.row(0).dot(x3Dt)+tcw2.at<float>(0);
                const float y2 = Rcw2.row(1).dot(x3Dt)+tcw2.at<float>(1);
                const float invz2 = 1.0/z2;
                if(!bStereo2)
                {
                    float u2 = fx2*x2*invz2+cx2;
                    float v2 = fy2*y2*invz2+cy2;
                    float errX2 = u2 - kp2.pt.x;
                    float errY2 = v2 - kp2.pt.y;
                    if((errX2*errX2+errY2*errY2)>5.991*sigmaSquare2)
                        continue;
                }
                else
                {
                    float u2 = fx2*x2*invz2+cx2;
                    float u2_r = u2 - mpCurrentKeyFrame->mbf*invz2;
                    float v2 = fy2*y2*invz2+cy2;
                    float errX2 = u2 - kp2.pt.x;
                    float errY2 = v2 - kp2.pt.y;
                    float errX2_r = u2_r - kp2_ur;
                    if((errX2*errX2+errY2*errY2+errX2_r*errX2_r)>7.8*sigmaSquare2)
                        continue;
                }

                //todo Check scale consistency
                cv::Mat normal1 = x3D-Ow1;
                float dist1 = cv::norm(normal1);

                cv::Mat normal2 = x3D-Ow2;
                float dist2 = cv::norm(normal2);

                if(dist1==0 || dist2==0)
                    continue;

                const float ratioDist = dist2/dist1;
                const float ratioOctave = mpCurrentKeyFrame->mvScaleFactors[kp1.octave]/pKF2->mvScaleFactors[kp2.octave];

                if(ratioDist*ratioFactor<ratioOctave || ratioDist>ratioOctave*ratioFactor)
                    continue;

                // todo Triangulation is succesfull
                MapPoint* pMP = new MapPoint(x3D,mpCurrentKeyFrame,mpMap);
                pMP->addid=mpCurrentKeyFrame->mnId;

                pMP->AddObservation(mpCurrentKeyFrame,idx1);
                pMP->AddObservation(pKF2,idx2);

                mpCurrentKeyFrame->AddMapPoint(pMP,idx1);
                pKF2->AddMapPoint(pMP,idx2);

                pMP->ComputeDistinctiveDescriptors();
                pMP->UpdateNormalAndDepth();
                mpMap->AddMapPoint(pMP);
                mlpRecentAddedMapPoints.push_back(pMP);

                nnew++;
            }
        }
    }

    void LocalMapping::SearchInNeighbors()//run
    {
        // todo Retrieve neighbor keyframes
        int nn = 10;
        if(mbMonocular)
            nn=20;
        const vector<KeyFrame*> vpNeighKFs = mpCurrentKeyFrame->GetBestCovisibilityKeyFrames(nn);
        vector<KeyFrame*> vpTargetKFs;
        for(vector<KeyFrame*>::const_iterator vit=vpNeighKFs.begin(), vend=vpNeighKFs.end(); vit!=vend; vit++)
        {
            KeyFrame* pKFi = *vit;
            if(pKFi->isBad() || pKFi->mnFuseTargetForKF == mpCurrentKeyFrame->mnId)
                continue;
            vpTargetKFs.push_back(pKFi);
            pKFi->mnFuseTargetForKF = mpCurrentKeyFrame->mnId;

            // todo Extend to some second neighbors
            const vector<KeyFrame*> vpSecondNeighKFs = pKFi->GetBestCovisibilityKeyFrames(5);
            for(vector<KeyFrame*>::const_iterator vit2=vpSecondNeighKFs.begin(), vend2=vpSecondNeighKFs.end(); vit2!=vend2; vit2++)
            {
                KeyFrame* pKFi2 = *vit2;
                if(pKFi2->isBad() || pKFi2->mnFuseTargetForKF==mpCurrentKeyFrame->mnId || pKFi2->mnId==mpCurrentKeyFrame->mnId)
                    continue;
                vpTargetKFs.push_back(pKFi2);
            }
        }


        // todo Search matches by projection from current KF in target KFs
        ORBmatcher matcher;
        vector<MapPoint*> vpMapPointMatches = mpCurrentKeyFrame->GetMapPointMatches();
        for(vector<KeyFrame*>::iterator vit=vpTargetKFs.begin(), vend=vpTargetKFs.end(); vit!=vend; vit++)
        {
            KeyFrame* pKFi = *vit;

            matcher.Fuse(pKFi,vpMapPointMatches);//todo 当前关键帧的点往目标关键帧投影
        }

        // todo Search matches by projection from target KFs in current KF
        vector<MapPoint*> vpFuseCandidates;
        vpFuseCandidates.reserve(vpTargetKFs.size()*vpMapPointMatches.size());

        for(vector<KeyFrame*>::iterator vitKF=vpTargetKFs.begin(), vendKF=vpTargetKFs.end(); vitKF!=vendKF; vitKF++)
        {
            KeyFrame* pKFi = *vitKF;

            vector<MapPoint*> vpMapPointsKFi = pKFi->GetMapPointMatches();

            for(vector<MapPoint*>::iterator vitMP=vpMapPointsKFi.begin(), vendMP=vpMapPointsKFi.end(); vitMP!=vendMP; vitMP++)
            {
                MapPoint* pMP = *vitMP;
                if(!pMP)
                    continue;
                if(pMP->isBad() || pMP->mnFuseCandidateForKF == mpCurrentKeyFrame->mnId)
                    continue;
                pMP->mnFuseCandidateForKF = mpCurrentKeyFrame->mnId;
                vpFuseCandidates.push_back(pMP);
            }
        }

        matcher.Fuse(mpCurrentKeyFrame,vpFuseCandidates);//todo 目标关键帧的点往当前关键帧投影


        // todo Update points
        vpMapPointMatches = mpCurrentKeyFrame->GetMapPointMatches();
        for(size_t i=0, iend=vpMapPointMatches.size(); i<iend; i++)
        {
            MapPoint* pMP=vpMapPointMatches[i];
            if(pMP)
            {
                if(!pMP->isBad())
                {
                    pMP->ComputeDistinctiveDescriptors();
                    pMP->UpdateNormalAndDepth();
                }
            }
        }

        // todo Update connections in covisibility graph
        mpCurrentKeyFrame->UpdateConnections();
    }

    cv::Mat LocalMapping::ComputeF12(KeyFrame *&pKF1, KeyFrame *&pKF2)//create new map points
    {
        cv::Mat R1w = pKF1->GetRotation();
        cv::Mat t1w = pKF1->GetTranslation();
        cv::Mat R2w = pKF2->GetRotation();
        cv::Mat t2w = pKF2->GetTranslation();

        cv::Mat R12 = R1w*R2w.t();
        cv::Mat t12 = -R1w*R2w.t()*t2w+t1w;

        cv::Mat t12x = SkewSymmetricMatrix(t12);

        const cv::Mat &K1 = pKF1->mK;
        const cv::Mat &K2 = pKF2->mK;


        return K1.t().inv()*t12x*R12*K2.inv();
    }

    void LocalMapping::RequestStop()
    {
        unique_lock<mutex> lock(mMutexStop);
        mbStopRequested = true;
        unique_lock<mutex> lock2(mMutexNewKFs);
        mbAbortBA = true;
    }

    bool LocalMapping::Stop()//run
    {
        unique_lock<mutex> lock(mMutexStop);
        if(mbStopRequested && !mbNotStop)
        {
            mbStopped = true;
            cout << "Local Mapping STOP" << endl;
            return true;
        }

        return false;
    }

    bool LocalMapping::isStopped()
    {
        unique_lock<mutex> lock(mMutexStop);
        return mbStopped;
    }

    bool LocalMapping::stopRequested()
    {
        unique_lock<mutex> lock(mMutexStop);
        return mbStopRequested;
    }

    void LocalMapping::Release()
    {
        unique_lock<mutex> lock(mMutexStop);
        unique_lock<mutex> lock2(mMutexFinish);
        if(mbFinished)
            return;
        mbStopped = false;
        mbStopRequested = false;
        for(list<KeyFrame*>::iterator lit = mlNewKeyFrames.begin(), lend=mlNewKeyFrames.end(); lit!=lend; lit++)
            delete *lit;
        mlNewKeyFrames.clear();

        cout << "Local Mapping RELEASE" << endl;
    }

    bool LocalMapping::AcceptKeyFrames()
    {
        unique_lock<mutex> lock(mMutexAccept);
        return mbAcceptKeyFrames;
    }

    void LocalMapping::SetAcceptKeyFrames(bool flag)//run
    {
        unique_lock<mutex> lock(mMutexAccept);
        mbAcceptKeyFrames=flag;
    }

    bool LocalMapping::SetNotStop(bool flag)
    {
        unique_lock<mutex> lock(mMutexStop);

        if(flag && mbStopped)
            return false;

        mbNotStop = flag;

        return true;
    }

    void LocalMapping::InterruptBA()//need new keyframe
    {
        mbAbortBA = true;
    }

    void LocalMapping::KeyFrameCulling()//run
    {
        // Check redundant keyframes (only local keyframes)
        // A keyframe is considered redundant if the 90% of the MapPoints it sees, are seen
        // in at least other 3 keyframes (in the same or finer scale)
        // We only consider close stereo points
        vector<KeyFrame*> vpLocalKeyFrames = mpCurrentKeyFrame->GetVectorCovisibleKeyFrames();

        for(vector<KeyFrame*>::iterator vit=vpLocalKeyFrames.begin(), vend=vpLocalKeyFrames.end(); vit!=vend; vit++)
        {
            KeyFrame* pKF = *vit;
            if(pKF->mnId==0)
                continue;
            const vector<MapPoint*> vpMapPoints = pKF->GetMapPointMatches();

            int nObs = 3;
            const int thObs=nObs;
            int nRedundantObservations=0;
            int nMPs=0;
            for(size_t i=0, iend=vpMapPoints.size(); i<iend; i++)
            {
                MapPoint* pMP = vpMapPoints[i];
                if(pMP)
                {
                    if(!pMP->isBad())
                    {
                        if(!mbMonocular)
                        {
                            if(pKF->mvDepth[i]>pKF->mThDepth || pKF->mvDepth[i]<0)
                                continue;
                        }

                        nMPs++;
                        if(pMP->Observations()>thObs)
                        {
                            const int &scaleLevel = pKF->mvKeysUn[i].octave;
                            const map<KeyFrame*, size_t> observations = pMP->GetObservations();
                            int nObs=0;
                            for(map<KeyFrame*, size_t>::const_iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
                            {
                                KeyFrame* pKFi = mit->first;
                                if(pKFi==pKF)
                                    continue;
                                const int &scaleLeveli = pKFi->mvKeysUn[mit->second].octave;

                                if(scaleLeveli<=scaleLevel+1)
                                {
                                    nObs++;
                                    if(nObs>=thObs)
                                        break;
                                }
                            }
                            if(nObs>=thObs)
                            {
                                nRedundantObservations++;
                            }
                        }
                    }
                }
            }

            if(nRedundantObservations>0.9*nMPs)
                pKF->SetBadFlag();
        }
    }

    cv::Mat LocalMapping::SkewSymmetricMatrix(const cv::Mat &v)//ComputeF12
    {
        return (cv::Mat_<float>(3,3) <<             0, -v.at<float>(2), v.at<float>(1),
                v.at<float>(2),               0,-v.at<float>(0),
                -v.at<float>(1),  v.at<float>(0),              0);
    }

    void LocalMapping::RequestReset()//reset
    {
        {
            unique_lock<mutex> lock(mMutexReset);
            mbResetRequested = true;
        }

        while(1)
        {
            {
                unique_lock<mutex> lock2(mMutexReset);
                if(!mbResetRequested)
                    break;
            }
            usleep(3000);
        }
    }

    void LocalMapping::ResetIfRequested()//run
    {
        unique_lock<mutex> lock(mMutexReset);
        if(mbResetRequested)
        {
            mlNewKeyFrames.clear();
            mlpRecentAddedMapPoints.clear();
            mbResetRequested=false;
        }
    }

    void LocalMapping::RequestFinish()//shut down
    {
        unique_lock<mutex> lock(mMutexFinish);
        mbFinishRequested = true;
    }

    bool LocalMapping::CheckFinish()//run
    {
        unique_lock<mutex> lock(mMutexFinish);
        return mbFinishRequested;
    }

    void LocalMapping::SetFinish()//run
    {
        unique_lock<mutex> lock(mMutexFinish);
        mbFinished = true;
        unique_lock<mutex> lock2(mMutexStop);
        mbStopped = true;
    }

    bool LocalMapping::isFinished()//shot down
    {
        unique_lock<mutex> lock(mMutexFinish);
        return mbFinished;
    }
}

