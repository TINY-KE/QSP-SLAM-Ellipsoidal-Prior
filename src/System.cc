/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/



#include "System.h"
#include "Converter.h"
#include <chrono>
#include <thread>
#include <pangolin/pangolin.h>
#include <iomanip>
#include <opencv2/core/eigen.hpp>
#include <time.h>
#include "src/config/Config.h"

ORB_SLAM2::Map* expMap;

bool has_suffix(const std::string &str, const std::string &suffix) {
    std::size_t index = str.find(suffix, str.size() - suffix.size());
    return (index != std::string::npos);
}

namespace ORB_SLAM2
{

System::System(const string &strVocFile, const string &strSettingsFile, const string &strSequencePath,
               const eSensor sensor) :
               mSensor(sensor), mpViewer(static_cast<Viewer*>(NULL)),
               mbReset(false),mbActivateLocalizationMode(false), mbDeactivateLocalizationMode(false)
{
    // Output welcome message
    // cout << endl <<
    // "DSP-SLAM: Object Oriented SLAM with Deep Shape Priors." << endl <<
    // "This program comes with ABSOLUTELY NO WARRANTY;" << endl  <<
    // "This is free software, and you are welcome to redistribute it" << endl <<
    // "under certain conditions. See LICENSE.txt." << endl << endl;

    cout << endl <<
    "QSP-SLAM: EllipsoidSLAM with Deep Shape Prior, 2023, Beihang University." << endl;

    cout << "Input sensor was set to: ";

    if(mSensor==MONOCULAR)
        cout << "Monocular" << endl;
    else if(mSensor==STEREO)
        cout << "Stereo" << endl;
    else if(mSensor==RGBD)
        cout << "RGB-D" << endl;

    //Check settings file
    cv::FileStorage fsSettings(strSettingsFile.c_str(), cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
       cerr << "Failed to open settings file at: " << strSettingsFile << endl;
       cerr << " * State : " << fsSettings.state << endl;
       exit(-1);
    }
    
    // Initialize global settings.
    Config::Init();
    Config::SetParameterFile(strSettingsFile);

    //TODO:这里有待优化
    Config::CheckParams("configs/standard_param.yaml");

    //Load ORB Vocabulary
    cout << endl << "Loading ORB Vocabulary. This could take a while..." << endl;

    mpVocabulary = new ORBVocabulary();

    bool bVocLoad = false; // chose loading method based on file extension
    if (has_suffix(strVocFile, ".txt"))
        bVocLoad = mpVocabulary->loadFromTextFile(strVocFile);
    else
        bVocLoad = mpVocabulary->loadFromBinaryFile(strVocFile);

    if(!bVocLoad)
    {
        cerr << "Wrong path to vocabulary. " << endl;
        cerr << "Falied to open at: " << strVocFile << endl;
        exit(-1);
    }
    cout << "Vocabulary loaded!" << endl << endl;

    // Set Object-related variables
    cout << "1/14 py::initialize_interpreter" << std::endl;
    py::initialize_interpreter();

    py::module warn = py::module::import("warnings");
    warn.attr("filterwarnings")("ignore");

    cv::FileStorage fSettings(strSettingsFile, cv::FileStorage::READ);

    cout << "2/14 import sys" << std::endl;
    py::module sys = py::module::import("sys");

    cout << "3/14 sys.path.append(\"./\")" << std::endl;
    sys.attr("path").attr("append")("./");
    sys.attr("path").attr("append")("./reconstruct");

    cout << "4/14 import reconstruct.utils as io_utils" << std::endl;
    py::module io_utils = py::module::import("reconstruct.utils");

    string pyCfgPath = fSettings["DetectorConfigPath"].string();

    cout << "5/14 读取python所需的json文件: " << pyCfgPath << std::endl;
    pyCfg = io_utils.attr("get_configs")(pyCfgPath);

    cout << "6/14 pyDecoder = get_decoder(pyCfg)" << std::endl;

    pyDecoder = io_utils.attr("get_decoder")(pyCfg);

    cout << "7/14 生成物体检测的reconstruct.get_sequence: "<< strSequencePath<< std::endl;
    pySequence = py::module::import("reconstruct").attr("get_sequence")(strSequencePath, pyCfg);
    cout << "8/14  import reconstruct.get_sequence success "<< std::endl;
    

    // For multi decoders
    py::module deep_sdf_utils = py::module::import("deep_sdf.workspace");
    cout << "9/14  import deep_sdf.workspace success "<< std::endl;

    vector<int> yolo_classes;
    vector<std::string> decoder_paths;
    fSettings["YoloClasses"] >> yolo_classes;
    std::cout << "10/14 : yolo_classes: ";
    for (int i : yolo_classes) {
        std::cout << i << " ";
    }
    std::cout << std::endl;
    fSettings["DecoderPaths"] >> decoder_paths;
    std::cout << "11/14 : decoder_paths: ";
    for (const std::string& path : decoder_paths) {
        std::cout << path << " ";
    }
    std::cout << std::endl;

    py::module optim  = py::module::import("reconstruct.optimizer");
    cout << "12/14  import reconstruct.optimizer success "<< std::endl;
    for (int i_class = 0; i_class < yolo_classes.size(); i_class++){
        int class_id = yolo_classes[i_class];
        cout << "mmPyDecoders Add class: " << class_id << std::endl;
        py::object decoder = deep_sdf_utils.attr("config_decoder")(decoder_paths[i_class]);
        // py::object* decoder_ptr = &decoder;
        mmPyDecoders[class_id] = std::move(decoder);
    }
    cout<< "13/14  construct mmPyDecoders success "<< std::endl;

    std::cout << "14/14 ";
    InitThread();

    //Create KeyFrame Database
    mpKeyFrameDatabase = new KeyFrameDatabase(*mpVocabulary);

    //Create the Map
    mpMap = new Map();

    //Create Drawers. These are used by the Viewer
    mpFrameDrawer = new FrameDrawer(mpMap);
    mpMapDrawer = new MapDrawer(mpMap, strSettingsFile);
    mpObjectDrawer = new ObjectDrawer(mpMap, mpMapDrawer, strSettingsFile);
    mpMapDrawer->SetObjectDrawer(mpObjectDrawer);

    //Initialize the Tracking thread
    //(it will live in the main thread of execution, the one that called this constructor)
    mpTracker = new Tracking(this, mpVocabulary, mpFrameDrawer, mpMapDrawer,
                             mpMap, mpKeyFrameDatabase, strSettingsFile, mSensor);

    // 地平面 
    // SetGroundPlaneMannually(Vector4d(0.00667181,  -0.015002,   0.999865,  0.0274608));
    mpTracker->SetGroundPlaneMannually(Vector4d(0,  0,   1,  0));
    mpMap->addPlane(&(mpTracker->mGroundPlane));
    
    //Initialize the Local Mapping thread and launch
    // Set an option: Multithread or single thread
    
    mpLocalMapper = new LocalMapping(this, mpMap, mpObjectDrawer, mSensor==MONOCULAR);



    mbMapInSameThread = 0;
    if(fSettings["System.LocalMappingInSameThread"].isNamed()) {
        int MapInSameThread = fSettings["System.mode"];
        mbMapInSameThread = MapInSameThread;
    }

    if (!mbMapInSameThread) {
        mptLocalMapping = new thread(&ORB_SLAM2::LocalMapping::Run, mpLocalMapper);
    }
    else {
        mpLocalMapper->InitSet();
    }
    

    // 暂时不考虑回环
    //Initialize the Loop Closing thread and launch
    // Only enable loop closing for KITTI
    if (mSensor == STEREO)
    {
        mpLoopCloser = new LoopClosing(mpMap, mpKeyFrameDatabase, mpVocabulary, mSensor!=MONOCULAR);
        mptLoopClosing = new thread(&ORB_SLAM2::LoopClosing::Run, mpLoopCloser);
    }
    else
    {
        mpLoopCloser = nullptr;
    }

    //Initialize the Viewer thread and launch
    mpViewer = new Viewer(this, mpFrameDrawer, mpMapDrawer, mpObjectDrawer, mpTracker,strSettingsFile);
    mptViewer = new thread(&Viewer::Run, mpViewer);
    mpTracker->SetViewer(mpViewer);

    //Set pointers between threads
    mpTracker->SetLocalMapper(mpLocalMapper);
    mpTracker->SetLoopClosing(mpLoopCloser);

    mpLocalMapper->SetTracker(mpTracker);
    mpLocalMapper->SetLoopCloser(mpLoopCloser);

    if (mpLoopCloser)
    {
        mpLoopCloser->SetTracker(mpTracker);
        mpLoopCloser->SetLocalMapper(mpLocalMapper);
    }

    OpenDepthEllipsoid();

    mpTracker->OpenGroundPlaneEstimation();     // Open Groundplane Estimation.

    expMap = mpMap;

    // // Release GIL
    // PyEval_ReleaseThread(PyThreadState_Get());
}

cv::Mat System::TrackStereo(const cv::Mat &imLeft, const cv::Mat &imRight, const double &timestamp)
{
    if(mSensor!=STEREO)
    {
        cerr << "ERROR: you called TrackStereo but input sensor was not set to STEREO." << endl;
        exit(-1);
    }

    // Check mode change
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

            mpTracker->InformOnlyTracking(true);
            mbActivateLocalizationMode = false;
        }
        if(mbDeactivateLocalizationMode)
        {
            mpTracker->InformOnlyTracking(false);
            mpLocalMapper->Release();
            mbDeactivateLocalizationMode = false;
        }
    }

    // Check reset
    {
        unique_lock<mutex> lock(mMutexReset);
        if(mbReset)
        {
            mpTracker->Reset();
            mbReset = false;
        }
    }

    cv::Mat Tcw = mpTracker->GrabImageStereo(imLeft, imRight, timestamp);

    unique_lock<mutex> lock2(mMutexState);
    mTrackingState = mpTracker->mState;
    mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
    mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;
    return Tcw;
}

cv::Mat System::TrackRGBD(const cv::Mat &im, const cv::Mat &depthmap, const double &timestamp)
{
    if(mSensor!=RGBD)
    {
        cerr << "ERROR: you called TrackRGBD but input sensor was not set to RGBD." << endl;
        exit(-1);
    }    

    // Check mode change
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

            mpTracker->InformOnlyTracking(true);
            mbActivateLocalizationMode = false;
        }
        if(mbDeactivateLocalizationMode)
        {
            mpTracker->InformOnlyTracking(false);
            mpLocalMapper->Release();
            mbDeactivateLocalizationMode = false;
        }
    }

    // Check reset
    {
    unique_lock<mutex> lock(mMutexReset);
        if(mbReset)
        {
            mpTracker->Reset();
            mbReset = false;
        }
    }

    // cout << "Before GrabImageRGBD" << endl;
    // printMemoryUsage();

    // KEY： 加入图像、深度和时间戳
    cv::Mat Tcw = mpTracker->GrabImageRGBD(im,depthmap,timestamp);

    // cout << "After GrabImageRGBD" << endl;
    // printMemoryUsage();
    
    // 更新状态
    unique_lock<mutex> lock2(mMutexState);
    mTrackingState = mpTracker->mState;
    mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
    mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;

    // 如果跟踪成功，则进行平面检测
    if (mTrackingState==Tracking::OK) {
        
    }

    if (mbMapInSameThread) {
        mpLocalMapper->RunOneTime();
    }

    return Tcw;
}

cv::Mat System::TrackMonocular(const cv::Mat &im, const double &timestamp)
{
    if(mSensor!=MONOCULAR)
    {
        cerr << "ERROR: you called TrackMonocular but input sensor was not set to Monocular." << endl;
        exit(-1);
    }

    // Check mode change
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

            mpTracker->InformOnlyTracking(true);
            mbActivateLocalizationMode = false;
        }
        if(mbDeactivateLocalizationMode)
        {
            mpTracker->InformOnlyTracking(false);
            mpLocalMapper->Release();
            mbDeactivateLocalizationMode = false;
        }
    }

    // Check reset
    {
    unique_lock<mutex> lock(mMutexReset);
    if(mbReset)
    {
        mpTracker->Reset();
        mbReset = false;
    }
    }

    cv::Mat Tcw = mpTracker->GrabImageMonocular(im,timestamp);

    unique_lock<mutex> lock2(mMutexState);
    mTrackingState = mpTracker->mState;
    mTrackedMapPoints = mpTracker->mCurrentFrame.mvpMapPoints;
    mTrackedKeyPointsUn = mpTracker->mCurrentFrame.mvKeysUn;

    if (mbMapInSameThread) {
        // while (mpLocalMapper->)
        mpLocalMapper->RunWhileNewKeyFrame();
    }

    return Tcw;
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

bool System::MapChanged()
{
    static int n=0;
    int curn = mpMap->GetLastBigChangeIdx();
    if(n<curn)
    {
        n=curn;
        return true;
    }
    else
        return false;
}

void System::Reset()
{
    unique_lock<mutex> lock(mMutexReset);
    mbReset = true;
}

void System::Shutdown()
{
    mpLocalMapper->RequestFinish();
    bool bLoopCloserStopped = false;
    if (mpLoopCloser)
        mpLoopCloser->RequestFinish();
    else
        bLoopCloserStopped = true;

    if(mpViewer)
    {
        mpViewer->RequestFinish();
        while(!mpViewer->isFinished())
            usleep(5000);
    }

    // Wait until all thread have effectively stopped
    while(!mpLocalMapper->isFinished() || !bLoopCloserStopped)
    {
        if (!bLoopCloserStopped)
        {
            usleep(5000);
            bLoopCloserStopped = (mpLoopCloser->isFinished() && !mpLoopCloser->isRunningGBA());
        }
        usleep(5000);
    }


    if(mpViewer)
        pangolin::BindToContext("QSP-SLAM: Map Viewer");

    PyGILState_Ensure();
}

void System::SaveTrajectoryTUM(const string &filename)
{
    cout << endl << "Saving camera trajectory to " << filename << " ..." << endl;
    if(mSensor==MONOCULAR)
    {
        cerr << "ERROR: SaveTrajectoryTUM cannot be used for monocular." << endl;
        return;
    }

    vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    cv::Mat Two = vpKFs[0]->GetPoseInverse();

    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    // Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
    // We need to get first the keyframe pose and then concatenate the relative transformation.
    // Frames not localized (tracking failure) are not saved.

    // For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
    // which is true when tracking failed (lbL).
    list<ORB_SLAM2::KeyFrame*>::iterator lRit = mpTracker->mlpReferences.begin();
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


void System::SaveKeyFrameTrajectoryTUM(const string &filename)
{
    cout << endl << "Saving keyframe trajectory to " << filename << " ..." << endl;

    vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    //cv::Mat Two = vpKFs[0]->GetPoseInverse();

    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    for(size_t i=0; i<vpKFs.size(); i++)
    {
        KeyFrame* pKF = vpKFs[i];

       // pKF->SetPose(pKF->GetPose()*Two);

        if(pKF->isBad())
            continue;

        cv::Mat R = pKF->GetRotation().t();
        vector<float> q = Converter::toQuaternion(R); // qx, qy, qz, qw
        cv::Mat t = pKF->GetCameraCenter();
        f << setprecision(6) << pKF->mTimeStamp << setprecision(7) << " " << t.at<float>(0) << " " << t.at<float>(1) << " " << t.at<float>(2)
          << " " << q[0] << " " << q[1] << " " << q[2] << " " << q[3] << endl;

    }

    f.close();
    cout << endl << "trajectory saved!" << endl;
}

void System::SaveTrajectoryKITTI(const string &filename)
{
    cout << endl << "Saving camera trajectory to " << filename << " ..." << endl;

    vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();
    sort(vpKFs.begin(),vpKFs.end(),KeyFrame::lId);

    // Transform all keyframes so that the first keyframe is at the origin.
    // After a loop closure the first keyframe might not be at the origin.
    cv::Mat Two = vpKFs[0]->GetPoseInverse();

    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    // Frame pose is stored relative to its reference keyframe (which is optimized by BA and pose graph).
    // We need to get first the keyframe pose and then concatenate the relative transformation.
    // Frames not localized (tracking failure) are not saved.

    // For each frame we have a reference keyframe (lRit), the timestamp (lT) and a flag
    // which is true when tracking failed (lbL).
    list<ORB_SLAM2::KeyFrame*>::iterator lRit = mpTracker->mlpReferences.begin();
    list<double>::iterator lT = mpTracker->mlFrameTimes.begin();
    for(list<cv::Mat>::iterator lit=mpTracker->mlRelativeFramePoses.begin(), lend=mpTracker->mlRelativeFramePoses.end();lit!=lend;lit++, lRit++, lT++)
    {
        ORB_SLAM2::KeyFrame* pKF = *lRit;

        cv::Mat Trw = cv::Mat::eye(4,4,CV_32F);

        while(pKF->isBad())
        {
          //  cout << "bad parent" << endl;
            Trw = Trw*pKF->mTcp;
            pKF = pKF->GetParent();
        }

        Trw = Trw*pKF->GetPose()*Two;

        cv::Mat Tcw = (*lit)*Trw;
        cv::Mat Rwc = Tcw.rowRange(0,3).colRange(0,3).t();
        cv::Mat twc = -Rwc*Tcw.rowRange(0,3).col(3);

        f << setprecision(9) << Rwc.at<float>(0,0) << " " << Rwc.at<float>(0,1)  << " " << Rwc.at<float>(0,2) << " "  << twc.at<float>(0) << " " <<
             Rwc.at<float>(1,0) << " " << Rwc.at<float>(1,1)  << " " << Rwc.at<float>(1,2) << " "  << twc.at<float>(1) << " " <<
             Rwc.at<float>(2,0) << " " << Rwc.at<float>(2,1)  << " " << Rwc.at<float>(2,2) << " "  << twc.at<float>(2) << endl;
    }
    f.close();
    cout << endl << "trajectory saved!" << endl;
}

int System::GetTrackingState()
{
    unique_lock<mutex> lock(mMutexState);
    return mTrackingState;
}

vector<MapPoint*> System::GetTrackedMapPoints()
{
    unique_lock<mutex> lock(mMutexState);
    return mTrackedMapPoints;
}

vector<cv::KeyPoint> System::GetTrackedKeyPointsUn()
{
    unique_lock<mutex> lock(mMutexState);
    return mTrackedKeyPointsUn;
}

void System::SetImageNames(vector<string>& vstrImageFilenamesRGB)
{
    mvstrImageFilenamesRGB.resize(vstrImageFilenamesRGB.size());
    mvstrImageFilenamesRGB = std::vector<string>(vstrImageFilenamesRGB.begin(), vstrImageFilenamesRGB.end());
    mpTracker->SetImageNames(vstrImageFilenamesRGB);
}

Map* System::getMap() {
    return mpMap;
}

MapDrawer* System::getMapDrawer() {
    return mpMapDrawer;
}

FrameDrawer* System::getFrameDrawer() {
    return mpFrameDrawer;
}

Viewer* System::getViewer() {
    return mpViewer;
}

Tracking* System::getTracker() {
    return mpTracker;
}

} //namespace ORB_SLAM
