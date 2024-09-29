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

#ifndef LOCALMAPPING_H
#define LOCALMAPPING_H

#include "KeyFrame.h"
#include "Map.h"
#include "LoopClosing.h"
#include "Tracking.h"
#include "Optimizer.h"
#include "KeyFrameDatabase.h"
#include "System.h"
#include <mutex>

#include "utils/file_operate.h"

#include <sys/resource.h>

#include <pybind11/embed.h>
#include <pybind11/eigen.h>

namespace py = pybind11;

namespace ORB_SLAM2
{

class Tracking;
class LoopClosing;
class Map;
class System;
class MapObject;
class Optimizer;
class Frame;

class LocalMapping
{
public:
    LocalMapping(System *pSys, Map* pMap, ObjectDrawer* pObjectDrawer, const float bMonocular);

    void SetLoopCloser(LoopClosing* pLoopCloser);

    void SetTracker(Tracking* pTracker);

    // Main function
    void Run();

    void InsertKeyFrame(KeyFrame* pKF);

    // Thread Synch
    void RequestStop();
    void RequestReset();
    bool Stop();
    void Release();
    bool isStopped();
    bool stopRequested();
    bool AcceptKeyFrames();
    void SetAcceptKeyFrames(bool flag);
    bool SetNotStop(bool flag);

    void InterruptBA();

    void RequestFinish();
    bool isFinished();

    int KeyframesInQueue(){
        unique_lock<std::mutex> lock(mMutexNewKFs);
        return mlNewKeyFrames.size();
    }

    void InitSet();
    bool RunOneTime();
    bool RunWhileNewKeyFrame();

    // Object SLAM by Jingwen
    KeyFrame* mpLastKeyFrame;
    std::list<MapObject*> mlpRecentAddedMapObjects;
    void GetNewObservations();
    void CreateNewMapObjects();
    void MapObjectCulling();
    void CreateNewObjectsFromDetections();
    void AssociateObjects3D();
    void ProcessDetectedObjects();

    void GlobalOptimization();

    void UpdateObjectsToMap();
    
    // Merge pMO_j into pMO_i
    void MergeMapObject(MapObject* pMO_i, MapObject* pMO_j);

    void SetOptimizer(Optimizer* optimizer);


    // map<int, int> classId2decoderId;
    map<int, py::object> mmPyOptimizers;
    map<int, py::object> mmPyMeshExtractors;

    py::object pyOptimizer;
    py::object pyMeshExtractor;

    int nLastReconKFID;

protected:

    bool CheckNewKeyFrames();
    void ProcessNewKeyFrame();
    void CreateNewMapPoints();

    void MapPointCulling();
    void SearchInNeighbors();

    void KeyFrameCulling();

    cv::Mat ComputeF12(KeyFrame* &pKF1, KeyFrame* &pKF2);

    cv::Mat SkewSymmetricMatrix(const cv::Mat &v);

    bool mbMonocular;

    void ResetIfRequested();
    bool mbResetRequested;
    std::mutex mMutexReset;

    bool CheckFinish();
    void SetFinish();
    bool mbFinishRequested;
    bool mbFinished;
    std::mutex mMutexFinish;

    Map* mpMap;
    ObjectDrawer* mpObjectDrawer;

    LoopClosing* mpLoopCloser;
    Tracking* mpTracker;

    Optimizer* mpOptimizer;

    std::list<KeyFrame*> mlNewKeyFrames;

    KeyFrame* mpCurrentKeyFrame;

    std::list<MapPoint*> mlpRecentAddedMapPoints;

    std::mutex mMutexNewKFs;

    bool mbAbortBA;  // 

    bool mbStopped;
    bool mbStopRequested;
    bool mbNotStop;
    std::mutex mMutexStop;

    bool mbAcceptKeyFrames;
    std::mutex mMutexAccept;

    int nKFInserted;

    bool use_ellipsold_pose_for_shape_optimization;
    int flip_sample_num;
    double flip_sample_angle;

    bool create_single_object;
    bool show_ellipsold_process;
    bool keep_raw_pose;

    bool add_depth_pcd_to_map_object;
    bool use_depth_pcd_to_reconstruct;

    double dist_filt_param;

    int min_valid_points, min_valid_rays;

    int cam_width, cam_height;

private:
    std::vector<Frame*> mvpFrames;

};

} //namespace ORB_SLAM

#endif // LOCALMAPPING_H
