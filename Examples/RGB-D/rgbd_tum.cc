#include<iostream>
#include<algorithm>
#include<chrono>
#include<opencv2/core/core.hpp>
#include<Config.h>
#include<include/System.h>

vector<vector<int>> yolo_mat2;
using namespace std;

int main(int argc, char **argv)
{
    if(argc != 5)
    {
        cerr << endl << "Usage: ./rgbd_tum path_to_vocabulary path_to_settings path_to_sequence path_to_association" << endl;
        return 1;
    }
    //todo 1 import images
    vector<string> vstrImageFilenamesRGB;
    vector<string> vstrImageFilenamesD;
    vector<double> vTimestamps;
    string strAssociationFilename = string(argv[4]);
    myslam::LoadImages(strAssociationFilename, vstrImageFilenamesRGB, vstrImageFilenamesD, vTimestamps);
    int nImages = vstrImageFilenamesRGB.size();
    if(vstrImageFilenamesRGB.empty())
    {
        cerr << endl << "No images found in provided path." << endl;
        return 1;
    }
    else if(vstrImageFilenamesD.size()!=vstrImageFilenamesRGB.size())
    {
        cerr << endl << "Different number of images for rgb and depth." << endl;
        return 1;
    }
    //todo 2 import detections
    vector <vector<int>> yolo_mat;

    myslam::LoadDetections(argv[1],yolo_mat);

    //todo 3 check detections
    myslam::Detections_Align_To_File(vstrImageFilenamesRGB,nImages,yolo_mat,yolo_mat2);

    //todo 4 creat SLAM system initialization
    myslam::System SLAM(argv[2]);
    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);
    //cout<<yolo_mat.size()<<endl;
    //cout<<yolo_mat2.size()<<endl;
    //todo 5 Main process loop
    cv::Mat imRGB, imD;
    for(int ni=0; ni<nImages; ni++)
    {
//        cv::waitKey(0);
        imRGB = cv::imread(string(argv[3])+"/"+vstrImageFilenamesRGB[ni],CV_LOAD_IMAGE_UNCHANGED);
        imD = cv::imread(string(argv[3])+"/"+vstrImageFilenamesD[ni],CV_LOAD_IMAGE_UNCHANGED);
        double tframe = vTimestamps[ni];
        if(imRGB.empty())
        {
            cerr << endl << "Failed to load image at: "
                 << string(argv[3]) << "/" << vstrImageFilenamesRGB[ni] << endl;
            return 1;
        }

        //std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t1 = std::chrono ::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t1 = std::chrono::monotonic_clock::now();
#endif
        SLAM.TrackRGBD(imRGB,imD,tframe);
        //std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#ifdef COMPILEDWITHC11
        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();
#else
        std::chrono::monotonic_clock::time_point t2 = std::chrono::monotonic_clock::now();
#endif

        double ttrack= std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();
        //cout<<"总时间："<<ttrack<<endl;
        vTimesTrack[ni]=ttrack;
        double T=0;
        if(ni<nImages-1)
            T = vTimestamps[ni+1]-tframe;
        else if(ni>0)
            T = tframe-vTimestamps[ni-1];
        if(ttrack<T)
            usleep((T-ttrack)*1e6);


    }
    // todo 6 stop all threads
    SLAM.Shutdown();
    // todo 7 tracking time statistic
    sort(vTimesTrack.begin(),vTimesTrack.end());
    float totaltime = 0;
    for(int ni=0; ni<nImages; ni++)
    {
        totaltime+=vTimesTrack[ni];
    }
    cout << "-------" << endl << endl;
    cout << "median tracking time: " << vTimesTrack[nImages/2] << endl;
    cout << "mean tracking time: " << totaltime/nImages << endl;

    // todo 7 save trajectory
    SLAM.SaveTrajectoryTUM("/home/zss/rgbd_dataset_freiburg1_desk/CameraTrajectory.txt");
    return 0;
}
