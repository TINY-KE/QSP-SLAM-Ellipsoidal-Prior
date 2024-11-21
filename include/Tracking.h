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


#ifndef TRACKING_H
#define TRACKING_H

#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>

#include"Viewer.h"
#include"FrameDrawer.h"
#include"Map.h"
#include"LocalMapping.h"
#include"LoopClosing.h"
#include"Frame.h"
#include "ORBVocabulary.h"
#include"KeyFrameDatabase.h"
#include"ORBextractor.h"
#include "Initializer.h"
#include "MapDrawer.h"
#include "System.h"
#include "MapObject.h"

#include "Initializer.h"
#include "Optimizer.h"

#include <src/symmetry/Symmetry.h>
#include <src/pca/EllipsoidExtractor.h>

#include <src/plane/PlaneExtractor.h>
#include <src/plane/PlaneExtractorManhattan.h>
#include <src/config/Config.h>

#include <src/Relationship/Relationship.h>

#include <src/dense_builder/builder.h>

#include "utils/file_operate.h"

#include <mutex>
#include <ctime>

#include "include/core/PriorInfer.h"
#include "MapPublisher.h"


namespace ORB_SLAM2
{

class Viewer;
class FrameDrawer;
class MapPublisher;
class Map;
class LocalMapping;
class LoopClosing;
class System;
class MapObject;

class PlaneExtractor;
class PlaneExtractorManhattan;
class Initializer;
class Optimizer;
class Symmetry;

enum OBJECT_MODEL
{
    POINT_MODEL = 0,
    QUADRIC_MODEL = 1
};

class Tracking
{  

public:
    Tracking(System* pSys, ORBVocabulary* pVoc, FrameDrawer* pFrameDrawer, MapDrawer* pMapDrawer, MapPublisher*  pMapPublisher, Map* pMap,
             KeyFrameDatabase* pKFDB, const string &strSettingPath, const int sensor);

    // Preprocess the input and call Track(). Extract features and performs stereo matching.
    cv::Mat GrabImageStereo(const cv::Mat &imRectLeft,const cv::Mat &imRectRight, const double &timestamp);
    cv::Mat GrabImageRGBD(const cv::Mat &imRGB,const cv::Mat &imD, const double &timestamp);
    cv::Mat GrabImageMonocular(const cv::Mat &im, const double &timestamp);

    void SetLocalMapper(LocalMapping* pLocalMapper);
    void SetLoopClosing(LoopClosing* pLoopClosing);
    void SetViewer(Viewer* pViewer);

    // Load new settings
    // The focal lenght should be similar or scale prediction will fail when projecting points
    // TODO: Modify MapPoint::PredictScale to take into account focal length
    void ChangeCalibration(const string &strSettingPath);

    // Use this function if you have deactivated local mapping and you only want to localize the camera.
    void InformOnlyTracking(const bool &flag);

    // Object SLAM by Jingwen
    // KITTI (stereo+LiDAR)
    // 获得双目+激光雷达点云的检测结果，创建对应的物体观测和地图物体，添加到相应关键帧中
    void GetObjectDetectionsLiDAR(KeyFrame *pKF);
    void ObjectDataAssociation_onlyforstereo(KeyFrame *pKF);
    // Freiburg Cars and Redwood (Mono)
    int maskErrosion;
    std::string detection_path;  // path to associated detected instances
    cv::Mat GetCameraIntrinsics();
    void GetObjectDetectionsMono(KeyFrame *pKF);
    void GetObjectDetectionsRGBD(KeyFrame *pKF);
    void AssociateObjectsByProjection(KeyFrame *pKF);  // assocating detection to object by projecting map points
    int associateDetWithObject(ORB_SLAM2::KeyFrame *pKF, MapObject* pMO, int d_i, ObjectDetection* detKF1, vector<MapPoint*>& mvpMapPoints);
    void SetImageNames(vector<string>& vstrImageFilenamesRGB);

    void TaskRelationship(ORB_SLAM2::Frame* pFrame);
    void RefineObjectsWithRelations(ORB_SLAM2::Frame *pFrame);

    void ManageMemory();
    /** --------------------------------
     * Object Observation 物体观测相关
     * ---------------------------------*/
    void UpdateObjectObservation_GenerateEllipsoid(ORB_SLAM2::Frame *pFrame, KeyFrame* pKF, bool withAssociation);

    /** --------------------------------
     * Ellipsoid 椭球体相关
     * ---------------------------------*/
    void OpenDepthEllipsoid();
    /** --------------------------------
     * Ground Plane 地面平面相关
     * ---------------------------------*/
    void OpenGroundPlaneEstimation();
    void TaskGroundPlane();
    // Mannual Set Groundplane
    void SetGroundPlaneMannually(const Eigen::Vector4d &param);
    //将平面添加到地图中,用于可视化； 并且将平面用于椭球体的生成
    void ActivateGroundPlane(g2o::plane &groundplane);
    void ProcessGroundPlaneEstimation();

    void UpdateDepthAndInferEllipsoidEstimation(ORB_SLAM2::Frame* pFrame, KeyFrame* pKF, bool withAssociation);
    void UpdateDepthEllipsoidEstimation(ORB_SLAM2::Frame* pFrame, KeyFrame* pKF, bool withAssociation);

    Builder* GetBuilder();
    bool SavePointCloudMap(const string& path);

public:
    void GenerateObservationStructure(ORB_SLAM2::Frame* pFrame);
    std::vector<Frame*> GetAllFramesWithKeyframe();

public:

    // Tracking states
    enum eTrackingState{
        SYSTEM_NOT_READY=-1,
        NO_IMAGES_YET=0,
        NOT_INITIALIZED=1,
        OK=2,
        LOST=3
    };

    eTrackingState mState;
    eTrackingState mLastProcessedState;

    // Input sensor
    int mSensor;

    // Current Frame
    Frame mCurrentFrame;

    Eigen::Matrix3d mCalib;
    int mRows, mCols;

    cv::Mat mImGray;
    cv::Mat mImDepth;
    vector<cv::Mat> mvImObjectMasks;
    vector<vector<int>> mvImObjectBboxs;

    // Initialization Variables (Monocular)
    std::vector<int> mvIniLastMatches;
    std::vector<int> mvIniMatches;
    std::vector<cv::Point2f> mvbPrevMatched;
    std::vector<cv::Point3f> mvIniP3D;
    Frame mInitialFrame;

    // Lists used to recover the full camera trajectory at the end of the execution.
    // Basically we store the reference keyframe for each frame and its relative transformation
    list<cv::Mat> mlRelativeFramePoses;
    list<KeyFrame*> mlpReferences;
    list<double> mlFrameTimes;
    list<bool> mlbLost;

    // True if local mapping is deactivated and we are performing only localization
    bool mbOnlyTracking;

    void Reset();

    void SetFrameByFrame();

protected:

    // Main tracking function. It is independent of the input sensor.
    void Track();

    // Map initialization for stereo and RGB-D
    void StereoInitialization();

    // Map initialization for monocular
    void MonocularInitialization();
    void CreateInitialMapMonocular();

    void CheckReplacedInLastFrame();
    bool TrackReferenceKeyFrame();
    void UpdateLastFrame();
    bool TrackWithMotionModel();

    bool Relocalization();

    void UpdateLocalMap();
    void UpdateLocalPoints();
    void UpdateLocalKeyFrames();

    bool TrackLocalMap();
    void SearchLocalPoints();

    bool NeedNewKeyFrame();
    void CreateNewKeyFrame();

    // In case of performing only localization, this flag is true when there are no matches to
    // points in the map. Still tracking will continue if there are enough matches with temporal points.
    // In that case we are doing visual odometry. The system will try to do relocalization to recover
    // "zero-drift" localization to the map.
    bool mbVO;

    string DetectorConfigFile;

    //Other Thread Pointers
    LocalMapping* mpLocalMapper;
    LoopClosing* mpLoopClosing;

    //ORB
    ORBextractor* mpORBextractorLeft, *mpORBextractorRight;
    ORBextractor* mpIniORBextractor;

    //BoW
    ORBVocabulary* mpORBVocabulary;
    KeyFrameDatabase* mpKeyFrameDB;

    // Initalization (only for monocular)
    Initializer* mpInitializer;

    //Local Map
    KeyFrame* mpReferenceKF;
    std::vector<KeyFrame*> mvpLocalKeyFrames;
    std::vector<MapPoint*> mvpLocalMapPoints;
    
    // System
    System* mpSystem;
    
    //Drawers
    Viewer* mpViewer;
    FrameDrawer* mpFrameDrawer;
    MapDrawer* mpMapDrawer;
    MapPublisher*  mpMapPublisher;

    //Map
    Map* mpMap;

    //Calibration matrix
    cv::Mat mK;
    cv::Mat mDistCoef;
    float mbf;
    camera_intrinsic mCamera;

    // todo: 实际上只对关键帧对应的Frame进行记录
    std::vector<Frame*> mvpFrames;

    // std::map<int, Observations> mmObjectObservations;

    //New KeyFrame rules (according to fps)
    int mMinFrames;
    int mMaxFrames;

    // Threshold close/far points
    // Points seen as close by the stereo/RGBD sensor are considered reliable
    // and inserted from just one frame. Far points requiere a match in two keyframes.
    float mThDepth;

    // For RGB-D inputs only. For some datasets (e.g. TUM) the depthmap values are scaled.
    float mDepthMapFactor;

    //Current matches in frame
    int mnMatchesInliers;

    //Last Frame, KeyFrame and Relocalisation Info
    KeyFrame* mpLastKeyFrame;
    Frame mLastFrame;
    unsigned int mnLastKeyFrameId;
    unsigned int mnLastRelocFrameId;

    //Motion Model
    cv::Mat mVelocity;

    //Color order (true RGB, false BGR, ignored if grayscale)
    bool mbRGB;

    Builder* mpBuilder;     // a dense pointcloud builder from visualization


    list<MapPoint*> mlpTemporalPoints;

    vector<string> mvstrImageFilenamesRGB;

    // bool mbMapInSameThread;

    std::mutex mMutexFrameByFrame;
    bool frame_by_frame;

    Optimizer* mpOptimizer;

    EllipsoidExtractor* mpEllipsoidExtractor;

    // Plane
    int miGroundPlaneState; // 0: Closed  1: estimating 2: estimated 3: set by mannual
    PlaneExtractor* pPlaneExtractor;
    PlaneExtractorManhattan* pPlaneExtractorManhattan;
    int miMHPlanesState; // 0: Closed 1: estimating 2: estimated

    RelationExtractor* mpRelationExtractor;

    // std::vector<g2o::SE3Quat> mvSavedFramePosesTwc;

    bool mbOpenDepthEllipsoid;
    bool mbOpenOptimization;

    int minimum_match_to_associate;

    bool show_ellipsold_process;

    bool minimux_points_to_judge_good;

    bool add_suffix_to_pcd;
    bool associate_object_with_ellipsold;

    bool add_depth_pcd_to_map_object;

    double associate_IoU_thresold;

    bool associate_debug;

// [整合]
public:
    void AssociateObjectsByProjection_mono_nouse(KeyFrame *pKF);  // assocating detection to object by projecting map points

    std::string mStrSettingPath; 
    
    void InferObjectsWithSemanticPrior(ORB_SLAM2::Frame* pFrame, bool use_input_pri, bool replace_detection);


    // 设置相机的真实位姿,但只用于第一帧
    void SetRealPose();

    // 只要导入PlaneExtractor这个库, 就会报错
    // #include <src/plane/PlaneExtractor.h>
    // #include <src/plane/PlaneExtractorManhattan.h>
    // PlaneExtractor* pPlaneExtractor;
    // PlaneExtractorManhattan* pPlaneExtractorManhattan;

    g2o::plane mGroundPlane;

    // 可视化平面
    void VisualizeManhattanPlanes();
};

} //namespace ORB_SLAM

#endif // TRACKING_H
