//
// Created by zss on 18-4-17.
//

#include "Frame.h"
#include "Converter.h"
#include <thread>
#include <include/Object.h>
#include "System.h"
#include <memory>
#define PI 3.14159265 //圆周率
using namespace std;
using namespace cv;
namespace myslam
{
    long unsigned int Frame::nNextId=0;
    bool Frame::mbInitialComputations=true;
    float Frame::cx, Frame::cy, Frame::fx, Frame::fy, Frame::invfx, Frame::invfy;
    float Frame::mnMinX, Frame::mnMinY, Frame::mnMaxX, Frame::mnMaxY;
    float Frame::mfGridElementWidthInv, Frame::mfGridElementHeightInv;
    Frame::Frame()
    {}
    Frame::Frame(const Frame &frame)
            :mpORBvocabulary(frame.mpORBvocabulary), mpORBextractor(frame.mpORBextractor),
             mTimeStamp(frame.mTimeStamp), mK(frame.mK.clone()), mDistCoef(frame.mDistCoef.clone()), mbf(frame.mbf), mb(frame.mb),
             mThDepth(frame.mThDepth), N(frame.N), mvKeys(frame.mvKeys),
             mvKeysUn(frame.mvKeysUn),mvuRight(frame.mvuRight), mvDepth(frame.mvDepth), mBowVec(frame.mBowVec), mFeatVec(frame.mFeatVec),
             mDescriptors(frame.mDescriptors.clone()), mvpMapPoints(frame.mvpMapPoints), mvbOutlier(frame.mvbOutlier), mnId(frame.mnId),
             mpReferenceKF(frame.mpReferenceKF), mnScaleLevels(frame.mnScaleLevels),
             mfScaleFactor(frame.mfScaleFactor), mfLogScaleFactor(frame.mfLogScaleFactor),
             mvScaleFactors(frame.mvScaleFactors), mvInvScaleFactors(frame.mvInvScaleFactors),
             mvLevelSigma2(frame.mvLevelSigma2), mvInvLevelSigma2(frame.mvInvLevelSigma2)
    {
        for(int i=0;i<FRAME_GRID_COLS;i++)
            for(int j=0; j<FRAME_GRID_ROWS; j++)
                mGrid[i][j]=frame.mGrid[i][j];

        if(!frame.mTcw.empty())
            SetPose(frame.mTcw);
    }

    Frame::Frame(const cv::Mat &imGray, const cv::Mat &imDepth, const double &timeStamp, ORBextractor* extractor,ORBVocabulary* voc, cv::Mat &K, cv::Mat &distCoef, const float &bf, const float &thDepth)
            :mpORBvocabulary(voc),mpORBextractor(extractor), mTimeStamp(timeStamp), mK(K.clone()),mDistCoef(distCoef.clone()), mbf(bf), mThDepth(thDepth)
    {
        // Frame ID
        mnId=nNextId++;

        // Scale Level Info
        mnScaleLevels = mpORBextractor->GetLevels();
        mfScaleFactor = mpORBextractor->GetScaleFactor();
        mfLogScaleFactor = log(mfScaleFactor);
        mvScaleFactors = mpORBextractor->GetScaleFactors();
        mvInvScaleFactors = mpORBextractor->GetInverseScaleFactors();
        mvLevelSigma2 = mpORBextractor->GetScaleSigmaSquares();
        mvInvLevelSigma2 = mpORBextractor->GetInverseScaleSigmaSquares();

        // ORB extraction
        ExtractORB(imGray);

        N = mvKeys.size();

        if(mvKeys.empty())
            return;
        UndistortKeyPoints();

        ComputeStereoFromRGBD(imDepth);


        mvpMapPoints = vector<MapPoint*>(N,static_cast<MapPoint*>(NULL));
        mvbOutlier = vector<bool>(N,false);

        // This is done only for the first Frame (or after a change in the calibration)
        if(mbInitialComputations)
        {
            //计算的是去除畸变后，图像的边界
            ComputeImageBounds(imGray);

            mfGridElementWidthInv=static_cast<float>(FRAME_GRID_COLS)/static_cast<float>(mnMaxX-mnMinX);
            mfGridElementHeightInv=static_cast<float>(FRAME_GRID_ROWS)/static_cast<float>(mnMaxY-mnMinY);

            fx = K.at<float>(0,0);
            fy = K.at<float>(1,1);
            cx = K.at<float>(0,2);
            cy = K.at<float>(1,2);
            invfx = 1.0f/fx;
            invfy = 1.0f/fy;

            mbInitialComputations=false;
        }
        mb = mbf/fx;
        //目的是加快匹配速度
        AssignFeaturesToGrid();


    }

    void Frame::AssignFeaturesToGrid()
    {
        int nReserve = 0.5f*N/(FRAME_GRID_COLS*FRAME_GRID_ROWS);
        for(unsigned int i=0; i<FRAME_GRID_COLS;i++)
            for (unsigned int j=0; j<FRAME_GRID_ROWS;j++)
                mGrid[i][j].reserve(nReserve);
        for(int i=0;i<N;i++)
        {
            const cv::KeyPoint &kp = mvKeysUn[i];

            int nGridPosX, nGridPosY;
            if(PosInGrid(kp,nGridPosX,nGridPosY))
                mGrid[nGridPosX][nGridPosY].push_back(i);
        }
    }

    void Frame::ExtractORB(const cv::Mat &im)
    {
            (*mpORBextractor)(im,cv::Mat(),mvKeys,mDescriptors);

    }


    void Frame::SetPose(cv::Mat Tcw)
    {
        mTcw = Tcw.clone();
        UpdatePoseMatrices();
    }

    void Frame::UpdatePoseMatrices()
    {
        mRcw = mTcw.rowRange(0,3).colRange(0,3);
        mRwc = mRcw.t();
        mtcw = mTcw.rowRange(0,3).col(3);
        mOw = -mRcw.t()*mtcw;
    }

    bool Frame::isInFrustum(MapPoint *pMP, float viewingCosLimit)//tracking-searchlocalpoints
    {
        //todo 策略1：将MapPoint投影到当前帧, 并判断是否在图像内。
        //todo 策略2：计算MapPoint到相机中心的距离, 并判断是否在尺度变化的距离内。
        //todo 策略3：计算当前视角和平均视角夹角(CreateInitialMapMonocular的UpdateNormalAndDepth的函数计算得到)的余弦值, 若小于cos(60), 即夹角大于60度则返回。
        //todo 最后根据深度预测尺度（对应特征点在一层），并标记该点将来要被投影(在函数SearchByProjection中被使用)。如果以上条件满足就代表当前的地图点在视野里。

        pMP->mbTrackInView = false;

        // 3D in absolute coordinates
        cv::Mat P = pMP->GetWorldPos();

        // 3D in camera coordinates
        const cv::Mat Pc = mRcw*P+mtcw;
        const float &PcX = Pc.at<float>(0);
        const float &PcY= Pc.at<float>(1);
        const float &PcZ = Pc.at<float>(2);

        // Check positive depth
        if(PcZ<0.0f)
            return false;

        // Project in image and check it is not outside
        const float invz = 1.0f/PcZ;
        const float u=fx*PcX*invz+cx;
        const float v=fy*PcY*invz+cy;

        if(u<mnMinX || u>mnMaxX)
            return false;
        if(v<mnMinY || v>mnMaxY)
            return false;
        // Check distance is in the scale invariance region of the MapPoint
        const float maxDistance = pMP->GetMaxDistanceInvariance();
        const float minDistance = pMP->GetMinDistanceInvariance();
        const cv::Mat PO = P-mOw;
        const float dist = cv::norm(PO);
        if(dist<minDistance || dist>maxDistance)
            return false;
        // Check viewing angle
        cv::Mat Pn = pMP->GetNormal();
        const float viewCos = PO.dot(Pn)/dist;
        if(viewCos<viewingCosLimit)
            return false;
        // Predict scale in the image
        const int nPredictedLevel = pMP->PredictScale(dist,this);
        // Data used by the tracking
        pMP->mbTrackInView = true;
        pMP->mTrackProjX = u;
        pMP->mTrackProjXR = u - mbf*invz;
        pMP->mTrackProjY = v;
        pMP->mnTrackScaleLevel= nPredictedLevel;
        pMP->mTrackViewCos = viewCos;
        return true;
    }

    vector<size_t> Frame::GetFeaturesInArea(const float &x, const float  &y, const float  &r, const int minLevel, const int maxLevel) const//search by projectiom
    {
        vector<size_t> vIndices;
        vIndices.reserve(N);

        const int nMinCellX = max(0,(int)floor((x-mnMinX-r)*mfGridElementWidthInv));
        if(nMinCellX>=FRAME_GRID_COLS)
            return vIndices;

        const int nMaxCellX = min((int)FRAME_GRID_COLS-1,(int)ceil((x-mnMinX+r)*mfGridElementWidthInv));
        if(nMaxCellX<0)
            return vIndices;

        const int nMinCellY = max(0,(int)floor((y-mnMinY-r)*mfGridElementHeightInv));
        if(nMinCellY>=FRAME_GRID_ROWS)
            return vIndices;

        const int nMaxCellY = min((int)FRAME_GRID_ROWS-1,(int)ceil((y-mnMinY+r)*mfGridElementHeightInv));
        if(nMaxCellY<0)
            return vIndices;

        const bool bCheckLevels = (minLevel>0) || (maxLevel>=0);

        for(int ix = nMinCellX; ix<=nMaxCellX; ix++)
        {
            for(int iy = nMinCellY; iy<=nMaxCellY; iy++)
            {
                const vector<size_t> vCell = mGrid[ix][iy];
                if(vCell.empty())
                    continue;

                for(size_t j=0, jend=vCell.size(); j<jend; j++)
                {
                    const cv::KeyPoint &kpUn = mvKeysUn[vCell[j]];
                    if(bCheckLevels)
                    {
                        if(kpUn.octave<minLevel)
                            continue;
                        if(maxLevel>=0)
                            if(kpUn.octave>maxLevel)
                                continue;
                    }

                    const float distx = kpUn.pt.x-x;
                    const float disty = kpUn.pt.y-y;

                    if(fabs(distx)<r && fabs(disty)<r)
                        vIndices.push_back(vCell[j]);
                }
            }
        }

        return vIndices;
    }

    bool Frame::PosInGrid(const cv::KeyPoint &kp, int &posX, int &posY)//AssignFeaturesToGrid
    {
        posX = round((kp.pt.x-mnMinX)*mfGridElementWidthInv);
        posY = round((kp.pt.y-mnMinY)*mfGridElementHeightInv);
        //Keypoint's coordinates are undistorted, which could cause to go out of the image
        if(posX<0 || posX>=FRAME_GRID_COLS || posY<0 || posY>=FRAME_GRID_ROWS)
            return false;
        return true;
    }

    void Frame::ComputeBoW()
    {
        if(mBowVec.empty())
        {
            vector<cv::Mat> vCurrentDesc = Converter::toDescriptorVector(mDescriptors);
            mpORBvocabulary->transform(vCurrentDesc,mBowVec,mFeatVec,4);
        }
    }

    void Frame::UndistortKeyPoints()
    {
        if(mDistCoef.at<float>(0)==0.0)
        {
            mvKeysUn=mvKeys;
            return;
        }

        // Fill matrix with points
        cv::Mat mat(N,2,CV_32F);
        for(int i=0; i<N; i++)
        {
            mat.at<float>(i,0)=mvKeys[i].pt.x;
            mat.at<float>(i,1)=mvKeys[i].pt.y;
        }

        // Undistort points
        mat=mat.reshape(2);
        cv::undistortPoints(mat,mat,mK,mDistCoef,cv::Mat(),mK);
        mat=mat.reshape(1);

        // Fill undistorted keypoint vector
        mvKeysUn.resize(N);
        for(int i=0; i<N; i++)
        {
            cv::KeyPoint kp = mvKeys[i];
            kp.pt.x=mat.at<float>(i,0);
            kp.pt.y=mat.at<float>(i,1);
            mvKeysUn[i]=kp;
        }
    }

    void Frame::ComputeImageBounds(const cv::Mat &imLeft)
    {
        if(mDistCoef.at<float>(0)!=0.0)
        {
            cv::Mat mat(4,2,CV_32F);
            mat.at<float>(0,0)=0.0; mat.at<float>(0,1)=0.0;
            mat.at<float>(1,0)=imLeft.cols; mat.at<float>(1,1)=0.0;
            mat.at<float>(2,0)=0.0; mat.at<float>(2,1)=imLeft.rows;
            mat.at<float>(3,0)=imLeft.cols; mat.at<float>(3,1)=imLeft.rows;

            // Undistort corners
            mat=mat.reshape(2);
            cv::undistortPoints(mat,mat,mK,mDistCoef,cv::Mat(),mK);
            mat=mat.reshape(1);

            mnMinX = min(mat.at<float>(0,0),mat.at<float>(2,0));
            mnMaxX = max(mat.at<float>(1,0),mat.at<float>(3,0));
            mnMinY = min(mat.at<float>(0,1),mat.at<float>(1,1));
            mnMaxY = max(mat.at<float>(2,1),mat.at<float>(3,1));

        }
        else
        {
            mnMinX = 0.0f;
            mnMaxX = imLeft.cols;
            mnMinY = 0.0f;
            mnMaxY = imLeft.rows;
        }
        //cout<<mnMinX<<","<<mnMaxX<<","<<mnMinY<<","<<mnMaxY<<endl;
        //cout<<imLeft.cols<<","<<imLeft.rows<<endl;
    }

    void Frame::ComputeStereoFromRGBD(const cv::Mat &imDepth)
    {
        mvuRight = vector<float>(N,-1);
        mvDepth = vector<float>(N,-1);

        for(int i=0; i<N; i++)
        {
            const cv::KeyPoint &kp = mvKeys[i];
            const cv::KeyPoint &kpU = mvKeysUn[i];

            const float &v = kp.pt.y;
            const float &u = kp.pt.x;

            const float d = imDepth.at<float>(v,u);//todo 深度按照原关键点坐标计算

            if(d>0)
            {
                mvDepth[i] = d;
                mvuRight[i] = kpU.pt.x-mbf/d;//todo 得到的是有面相机对应点的横坐标
            }
        }
    }

    cv::Mat Frame::UnprojectStereo(const int &i)
    {
        const float z = mvDepth[i];
        if(z>0)
        {
            const float u = mvKeysUn[i].pt.x;//校正后的特征点
            const float v = mvKeysUn[i].pt.y;
            const float x = (u-cx)*z*invfx;//相机坐标系
            const float y = (v-cy)*z*invfy;
            cv::Mat x3Dc = (cv::Mat_<float>(3,1)<< x, y, z);
            return mRwc*x3Dc+mOw;

        }
        else
            return cv::Mat();
    }

    cv::Mat Frame::UnprojectStereo_camera(const int &i)
    {
        const float z = mvDepth[i];
        if(z>0)
        {
            const float u = mvKeysUn[i].pt.x;//校正后的特征点
            const float v = mvKeysUn[i].pt.y;
            const float x = (u-cx)*z*invfx;//相机坐标系
            const float y = (v-cy)*z*invfy;
            cv::Mat x3Dc = (cv::Mat_<float>(3,1)<< x, y, z);
            return x3Dc;

        }
        else
            return cv::Mat();
    }

    bool cmp( pair<float, int> first, pair<float, int> next )
    {
        return first.first < next.first ;
    }

    vector<Object*> Frame::get_frame_object(const cv::Mat &imRGB,const cv::Mat &imdepth)
    {
        vector<Object*> objs;
        float u_max=0;
        float v_max=0;
        int num_kp=0;
        for (int i = 0; i <yolo_mat2.size() ; ++i)
        {
            if(yolo_mat2[i][0]==this->mnId)
            {
                int class_id;
                cv::Mat x3D;
                class_id=yolo_mat2[i][2];

                //上下左右
                int left=yolo_mat2[i][3];
                int right=yolo_mat2[i][4];
                int top=yolo_mat2[i][5];
                int bottom=yolo_mat2[i][6];

                string name;
                if(class_id==73)
                    name="book";
                else if(class_id==66)
                    name="keyboard";
                else if(class_id==64)
                    name="mouse";
                else if(class_id==62)
                    name="tvmonitor";
                else if(class_id==41)
                    name="cup";
                else if(class_id==56)
                    name="chair";
                else if(class_id==67)
                    name="cell phone";
                else if(class_id==65)
                    name="remote";


                cv::line(image,cv::Point2f(left,top),cv::Point2f(right,top),cv::Scalar(100,256,0),1.5);///bgr
                cv::line(image,cv::Point2f(left,top),cv::Point2f(left,bottom),cv::Scalar(100,256,0),1.5);///bgr
                cv::line(image,cv::Point2f(right,top),cv::Point2f(right,bottom),cv::Scalar(100,256,0),1.5);///bgr
                cv::line(image,cv::Point2f(left,bottom),cv::Point2f(right,bottom),cv::Scalar(100,256,0),1.5);///bgr
                cv::putText(image, name, cv::Point2f(left+3,top+11), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(100,256,0), 1);

                cv::Point2f middle_point;
                float x;
                float y;
                float u=middle_point.x=(right+left)/2;
                float v=middle_point.y=(bottom+top)/2;
//                cv::circle(image,cv::Point2f(u,v),5,cv::Scalar(256,100,0),-1);
//                cv::putText(image,"1", cv::Point2f(u,v), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255,255,0), 1);

                float dmax=0;
                float d;
                for (int i = v-5; i <v+5; i++)//y
                {
                    for (int j =u-5; j <u+5; j++)//x
                    {
                        d = imdepth.at<float>(i,j);
                        //cout<<d<<endl;
                        if (d > dmax)//todo 后续可以改成有深度值的
                        {
                            dmax = d;
                            u_max = j;
                            v_max = i;
                        }
                        else
                            continue;
                    }
                }

                d=dmax;
                x = (u_max-this->cx)*d*this->invfx;//相机坐标系
                y = (v_max-this->cy)*d*this->invfy;

                if(d==0)
                {
                    //cout<<"!!!"<<endl;
                    continue;
                }
                cv::Mat x3Dc;
                //cout<<"深度"<<d<<endl;
                x3Dc = (cv::Mat_<float>(3,1)<< x, y, d);
                x3D=mRwc*x3Dc+mOw;
                //cout<<"深度："<<x3D.at<float>(2,0)<<endl;
                Object* obj=new Object(x3D,class_id,u_max,v_max);
                obj->_Pos=x3D;
                obj->left=yolo_mat2[i][3];
                obj->right=yolo_mat2[i][4];
                obj->top=yolo_mat2[i][5];
                obj->bottom=yolo_mat2[i][6];
                objs.push_back(obj);
            }
        }
        return objs;

    }

}