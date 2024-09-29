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

#ifndef OPTIMIZER_H
#define OPTIMIZER_H

#include "Map.h"
#include "MapPoint.h"
#include "KeyFrame.h"
#include "LoopClosing.h"
#include "Frame.h"


#include "src/symmetry/Symmetry.h"
#include "src/Relationship/Relationship.h"
#include "include/core/SupportingPlane.h"
#include "BasicEllipsoidEdges.h"

#include "Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"


namespace ORB_SLAM2
{

// class LoopClosing;

class KeyFrame;

typedef map<KeyFrame*,g2o::Sim3,std::less<KeyFrame*>,
    Eigen::aligned_allocator<std::pair<const KeyFrame*, g2o::Sim3> >> KeyFrameAndPose;

class EllipObject
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    inline double dp_prior(int label)
    {
        // 需要知道总共的 label 种类. 然后有新观测则将其翻倍.
        int total_ob = measurementIDs.size();

        int label_ob;
        if(classVoter.find(label)!=classVoter.end())
            label_ob = classVoter[label];
        else 
            label_ob = 0;
        
        int label_ob_new = label_ob + 1;
        
        double dp = double(label_ob_new) / (total_ob+1);
        return dp;
    }

    int instance_id;
    std::map<int,int> classVoter;  // 类别投票器
    std::vector<int> measurementIDs;  
    ellipsoid* pEllipsoid;
};
typedef std::vector<EllipObject> Objects;

class Relation;    // TOBECHECK: typedef出来的能否用class做声明?
typedef std::vector<Relation> Relations;

class Optimizer
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    Optimizer();

    void static BundleAdjustment(const std::vector<KeyFrame*> &vpKF, const std::vector<MapPoint*> &vpMP,
                                 int nIterations = 5, bool *pbStopFlag=NULL, const unsigned long nLoopKF=0,
                                 const bool bRobust = true);
    void static JointBundleAdjustment(const std::vector<KeyFrame*> &vpKF, const std::vector<MapPoint*> &vpMP,
                                 const std::vector<MapObject*> &vpMO, int nIterations = 5, bool *pbStopFlag=NULL,
                                 const unsigned long nLoopKF=0, const bool bRobust = true);
    void static GlobalBundleAdjustemnt(Map* pMap, int nIterations=5, bool *pbStopFlag=NULL,
                                       const unsigned long nLoopKF=0, const bool bRobust = true);
    void static GlobalJointBundleAdjustemnt(Map* pMap, int nIterations=5, bool *pbStopFlag=NULL,
                                       const unsigned long nLoopKF=0, const bool bRobust = true);
    void static LocalBundleAdjustment(KeyFrame* pKF, bool *pbStopFlag, Map *pMap);
    void static LocalJointBundleAdjustment(KeyFrame* pKF, bool *pbStopFlag, Map *pMap);
    // void static LocalJointBundleAdjustment(KeyFrame* pKF, bool *pbStopFlag, Map *pMap, py::object *pyOptimizer);
    int static PoseOptimization(Frame* pFrame);

    // if bFixScale is true, 6DoF optimization (stereo,rgbd), 7DoF otherwise (mono)
    void static OptimizeEssentialGraph(Map* pMap, KeyFrame* pLoopKF, KeyFrame* pCurKF,
                                       const KeyFrameAndPose &NonCorrectedSim3,
                                       const KeyFrameAndPose &CorrectedSim3,
                                       const map<KeyFrame *, set<KeyFrame *>> &LoopConnections,
                                       const bool &bFixScale);

    // if bFixScale is true, optimize SE3 (stereo,rgbd), Sim3 otherwise (mono)
    static int OptimizeSim3(KeyFrame* pKF1, KeyFrame* pKF2, std::vector<MapPoint *> &vpMatches1,
                            g2o::Sim3 &g2oS12, const float th2, const bool bFixScale);
    static int nBAdone;

    void SetGroundPlane(Vector4d& normal);

    void GetOptimizedResult(Objects& objs, Measurements& mms);

public:
    // Optimize with probabilistic data association
    void GlobalObjectGraphOptimizationWithPDA(std::vector<Frame*> &pFrames, Map *pMap, const Matrix3d& calib, int iRows, int iCols);
    // void GlobalObjectGraphOptimizationWithPDA(Map *pMap, const Matrix3d& calib, int iRows, int iCols);

private:
    void UpdateDataAssociation(Measurements& mms, Objects& objs, int model = 0);
    
    // 基于切平面约束完成椭球体的全局优化
    void OptimizeWithDataAssociationUsingMultiplanes(std::vector<Frame *> &pFrames, 
                    Measurements& mms, Objects& objs, Trajectory& camTraj, const Matrix3d& calib, int iRows, int iCols);

    void LoadRelations(Relations& rls, SupportingPlanes& spls);

private:
    std::map<int, std::vector<float>> mMapObjectConstrain;

    bool mbGroundPlaneSet;
    Vector4d mGroundPlaneNormal;

    bool mbRelationLoaded;
    Relations mRelations;
    SupportingPlanes mSupportingPlanes;

    // 保存优化结果
    Objects mObjects;
    Measurements mMeasurements;
};


    int GetTotalObjectIndex(std::vector<Frame *> &pFrames, int frame_index, int index_in_frame);
    bool checkVisibility(g2o::EdgeSE3EllipsoidProj *edge, g2o::VertexSE3Expmap *vSE3, 
    g2o::VertexEllipsoid *vEllipsoid, Eigen::Matrix3d &mCalib, int rows, int cols);


} //namespace ORB_SLAM

#endif // OPTIMIZER_H
