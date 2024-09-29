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

#ifndef MAPDRAWER_H
#define MAPDRAWER_H

#include "Map.h"
#include "MapPoint.h"
#include "KeyFrame.h"
#include "Converter.h"
#include "ObjectDrawer.h"
#include <pangolin/pangolin.h>
#include <GL/glu.h>
#include<mutex>

namespace ORB_SLAM2
{

class ObjectDrawer;

class MapDrawer
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    MapDrawer(Map* pMap, const string &strSettingPath);

    void DrawMapPoints(float mappointSize);
    void DrawKeyFrames(const bool bDrawKF, const bool bDrawGraph);
    void DrawCurrentCamera(pangolin::OpenGlMatrix &Twc);
    void SetCurrentCameraPose(const cv::Mat &Tcw);

    // By Jingwen
    std::vector<std::tuple<float, float, float>> mvObjectColors;
    ObjectDrawer* mpObjectDrawer;
    void SetObjectDrawer(ObjectDrawer *pObjectDrawer);
    Eigen::Matrix4f GetCurrentCameraMatrix();
    void SetReferenceKeyFrame(KeyFrame *pKF);


    /**From EllipsoldSLAM*/
    // bool updateObjects();
    // bool updateCameraState();

    // bool drawObjects(double prob_thresh = 0);
    // bool drawCameraState();
    // bool drawGivenCameraState(g2o::SE3Quat* state, const Vector3d& color);

    void drawEllipsoid();
    bool drawEllipsoids(double prob_thresh = 0);
    bool drawObservationEllipsoids(double prob_thresh = 0);

    bool drawEllipsoidsObjects(double prob_thresh = 0);

    bool drawPlanes(int visual_group=0);
    // bool drawPoints();
    // void setCalib(Eigen::Matrix3d& calib);

    // bool drawTrajectory();
    // bool drawTrajectoryDetail();
    // bool drawGivenTrajDetail(std::vector<g2o::SE3Quat*>& traj, const Vector3d& color);
    // bool drawTrajectoryWithName(const string& name);

    void SE3ToOpenGLCameraMatrix(g2o::SE3Quat &matIn, pangolin::OpenGlMatrix &M); // inverse matIn
    // void SE3ToOpenGLCameraMatrixOrigin(g2o::SE3Quat &matIn, pangolin::OpenGlMatrix &M); // don't inverse matIn
    void GetCurrentOpenGLCameraMatrix(pangolin::OpenGlMatrix &M);

    void drawPointCloudLists(float pointSize=1); // draw all the point cloud lists 
    void drawPointCloudLists(float pointSize, std::string pcd_name);
    void drawPointCloud(PointCloud *pPoints, float pointSize = 1);
    void drawPointCloudWithOptions(const std::map<std::string,bool> &options, float pointcloudSize=1); // draw the point cloud lists with options opened

    // void drawBoundingboxes();
    // void drawConstrainPlanes(double prob_thresh = 0, int type = 0);

    // void drawArrows();
    void drawLine(const Vector3d& start, const Vector3d& end, const Vector3d& color, double width, double alpha = 1.0);

    // void SetTransformTge(const g2o::SE3Quat& Tge);

    // void drawAxisNormal();

    // void drawEllipsoidsLabelText(double prob_thresh, bool show_ellipsoids = true, bool show_observation = true);


    // By Lijian
    void DrawDepthPointCloud();

    // 可能被ObjectDrawer调用
    void drawEllipsoidInVector(ellipsoid* e, int color_mode = 0); // 0: Red, 1: Green, 2:Blue

private:
    // void drawPlaneWithEquationDense(plane* p);
    void drawPlaneWithEquation(plane* p);
    void drawAllEllipsoidsInVector(std::vector<ellipsoid*>& ellipsoids, int color_mode = 0);
    
    // void drawLabelTextOfEllipsoids(std::vector<ellipsoid*>& ellipsoids);

    // pangolin::OpenGlMatrix getGLMatrixFromCenterAndNormal(Vector3f& center, Vector3f& normal);

    // void drawOneBoundingbox(Matrix3Xd& corners, Vector3d& color, double alpha = 1.0);

    // bool drawGivenTrajWithColor(std::vector<g2o::SE3Quat*>& traj, const Vector3d& color);
    // bool drawGivenTrajWithColorLines(std::vector<g2o::SE3Quat*>& traj, const Vector3d& color);

    // void eigenMatToOpenGLMat(const Eigen::Matrix4d& matEigen, pangolin::OpenGlMatrix &M);

private:

    float mKeyFrameSize;
    float mKeyFrameLineWidth;
    float mGraphLineWidth;
    float mPointSize;
    float mCameraSize;
    float mCameraLineWidth;

    cv::Mat mCameraPose;
    std::mutex mMutexCamera;

    Map* mpMap;
    Eigen::Matrix3d mCalib;

    g2o::SE3Quat mTge;  // estimation to groundtruth coordinite, for visualization only.
    bool mbOpenTransform;
};

} //namespace ORB_SLAM

#endif // MAPDRAWER_H
