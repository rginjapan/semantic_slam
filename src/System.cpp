//
// Created by zss on 18-4-16.
//
#include "common_include.h"
#include "System.h"


namespace myslam
{

    System::System(const string &strSettingsFile) : mpViewer(static_cast<Viewer *>(NULL)),mbReset(false),mbActivateLocalizationMode(false),
                                                    mbDeactivateLocalizationMode(false)
    {
        cv::FileStorage fsSettings(strSettingsFile.c_str(), cv::FileStorage::READ);
        if (!fsSettings.isOpened()) {
            cerr << "Failed to open settings file at: " << strSettingsFile << endl;
            exit(-1);
        }
        //todo 1 Load ORB Vocabulary
        mpVocabulary = new ORBVocabulary();
        bool bVocLoad = mpVocabulary->loadFromTextFile("/home/zss/ORB_SLAM2/Vocabulary/ORBvoc.txt");
        if (!bVocLoad)
        {
            cerr << "Falied to open vocabulary" << endl;
            exit(-1);
        }
        cout << "Vocabulary loaded!" << endl << endl;

        //todo 2 Create KeyFrame Database
        mpKeyFrameDatabase = new KeyFrameDatabase(*mpVocabulary);

        //todo 3 Create the Map
        mpMap = new Map();

        //todo 4 Create Drawers
        mpFrameDrawer = new FrameDrawer(mpMap);
        mpMapDrawer = new MapDrawer(mpMap, strSettingsFile,mpTracker);//多了一个 mpTracker

        //todo 5 Initialize the Tracking thread
        mpTracker = new Tracking(this, mpVocabulary, mpFrameDrawer, mpMapDrawer, mpMap, mpKeyFrameDatabase, strSettingsFile);

        //todo 6 Initialize the Local Mapping thread and launch
        mpLocalMapper = new LocalMapping(mpMap);//没有==单目
        mptLocalMapping = new thread(&myslam::LocalMapping::Run, mpLocalMapper);

        //todo 7 Initialize the Loop closing thread and launch
        mpLoopCloser = new LoopClosing(mpMap, mpKeyFrameDatabase, mpVocabulary);//没有！=单目
        mptLoopClosing = new thread(&myslam::LoopClosing::Run, mpLoopCloser);

        //todo 8 Initialize the Viewer thread and launch
        mpViewer = new Viewer(this, mpFrameDrawer, mpMapDrawer, mpTracker, strSettingsFile);
        mptViewer = new thread(&Viewer::Run, mpViewer);
        mpTracker->SetViewer(mpViewer);

        //todo 9 Set pointers between threads
        mpTracker->SetLocalMapper(mpLocalMapper);
        mpTracker->SetLoopClosing(mpLoopCloser);

        mpLocalMapper->SetTracker(mpTracker);
        mpLocalMapper->SetLoopCloser(mpLoopCloser);

        mpLoopCloser->SetTracker(mpTracker);
        mpLoopCloser->SetLocalMapper(mpLocalMapper);

    }
    cv::Mat System::TrackRGBD(const cv::Mat &im, const cv::Mat &depthmap, const double &timestamp)
    {
        // todo Check mode change
        {
            unique_lock<mutex> lock(mMutexMode);
            if(mbActivateLocalizationMode)
            {
                mpLocalMapper->RequestStop();

                // Wait until Local Mapping has effectively stopped
                while(!mpLocalMapper->isStopped())
                {
                    usleep(1000);
                }

                mpTracker->InformOnlyTracking(true);/////////////////////////////////////////////
                mbActivateLocalizationMode = false;
            }
            if(mbDeactivateLocalizationMode)//////初始化
            {
                mpTracker->InformOnlyTracking(false);////////////////////////////////////////////
                mpLocalMapper->Release();
                mbDeactivateLocalizationMode = false;
            }
        }

        {
            unique_lock<mutex> lock(mMutexReset);
            if(mbReset)
            {
                mpTracker->Reset();
                mbReset = false;
            }
        }
        //todo 1 return Tcw
        std::chrono::steady_clock::time_point t3 = std::chrono ::steady_clock::now();

        cv::Mat Tcw = mpTracker->GrabImageRGBD(im,depthmap,timestamp);
        std::chrono::steady_clock::time_point t4 = std::chrono ::steady_clock::now();
        double ttrack0= std::chrono::duration_cast<std::chrono::duration<double> >(t4 - t3).count();
        //cout<<"GrabImageRGBD时间："<<ttrack0<<endl;

        //todo 2 check tracking state
        unique_lock<mutex> lock2(mMutexState);
        mTrackingState = mpTracker->mState;
        mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
        mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;
        return Tcw;
    }
    void System::Shutdown()
    {
        mpLocalMapper->RequestFinish();
        mpLoopCloser->RequestFinish();
        if(mpViewer)
        {
            mpViewer->RequestFinish();
            while(!mpViewer->isFinished())
                usleep(5000);
        }
        // Wait until all thread have effectively stopped
        while(!mpLocalMapper->isFinished() || !mpLoopCloser->isFinished() || mpLoopCloser->isRunningGBA())
        {
            usleep(5000);
        }

    }

    void System::Reset()
    {
        unique_lock<mutex> lock(mMutexReset);
        mbReset = true;
    }

    void System::SaveTrajectoryTUM(const string &filename)
    {
        vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
        sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);
        // After a loop closure the first keyframe might not be at the origin.
        cv::Mat Two = vpKFs[0]->GetPoseInverse();
        ofstream f;
        f.open(filename.c_str());
        f << fixed;
        // For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag which is true when tracking failed (lbL).
        list<myslam::KeyFrame*>::iterator lRit = mpTracker->mlpReferences.begin();
        list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
        list<bool>::iterator lbL = mpTracker->mlbLost.begin();
        for(list<cv::Mat>::iterator lit=mpTracker->mlRelativeFramePoses.begin(),
                    lend=mpTracker->mlRelativeFramePoses.end();lit!=lend;lit++, lRit++, lT++, lbL++)
        {
            if(*lbL)
                continue;
            KeyFrame* pKF = *lRit;
            cv::Mat Trw = cv::Mat::eye(4,4,CV_32F);
            // If the reference keyframe was culled, traverse the spanning tree to get a suitable keyframe.
            while(pKF->isBad())
            {
                Trw = Trw*pKF->mTcp;
                pKF = pKF->GetParent();
            }
            Trw = Trw*pKF->GetPose()*Two;
            cv::Mat Tcw = (*lit)*Trw;
            cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t();
            cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3);

            vector<float> q = Converter::toQuaternion(Rwc);
            f << setprecision(6) << *lT << " " <<  setprecision(9) << twc.at<float>(0) << " " << twc.at<float>(1) << " " << twc.at<float>(2) << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;
        }
        f.close();
        cout << endl << "trajectory saved!" << endl;
    }

    void System::ActivateLocalizationMode()
    {
        unique_lock<mutex> lock(mMutexMode);
        mbActivateLocalizationMode = true;
    }

    void System::DeactivateLocalizationMode()
    {
        unique_lock<mutex> lock(mMutexMode);
        mbDeactivateLocalizationMode = true;
    }

}




