#ifndef ELLIPSOIDSLAM_SYMMETRY_H
#define ELLIPSOIDSLAM_SYMMETRY_H

#include <core/Ellipsoid.h>
#include <core/Geometry.h>

#include <Eigen/Core>
using namespace Eigen;

#include "BorderExtractor.h"
#include "SymmetrySolver.h"
#include <opencv2/core/core.hpp>

namespace ORB_SLAM2 {

class SymmetryOutputData {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    bool result;

    PointCloud *pCloud; // object point cloud
    Vector4d planeVec;
    Vector4d planeVec2; // the second plane of dual reflection

    double prob;
    PointCloud *pBorders; // object borders, used in the sparse mode; invalid now.

    Vector3d center; // for visualizing the symmetry plane as an finite plane
    int symmetryType;
};

class Symmetry {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Symmetry();

    // API: estimate a symmetry plane from a point cloud using all the points
    // symType: 1 reflection symmetry; 2 dual reflection symmetry
    SymmetrySolverData estimateSymmetry(Vector4d &bbox, PointCloud *pCloud, VectorXd &pose, cv::Mat &projDepth, camera_intrinsic &camera,
                                        int symType = 1);

    void static releaseData(SymmetryOutputData &data);

    cv::Mat static getProjDepthMat(cv::Mat &depth, camera_intrinsic &camera);

  public:
    void SetGroundPlane(Vector4d &normal);

    void SetBorders(ORB_SLAM2::PointCloud *pBorders);
    void SetConfigResolution(double res);
    double GetConfigResolution();
    void SetConfigFilterPointNum(int num);

  private:
    bool mbOpenSparseEstimation;
    PointCloud *mpBorders;

    BorderExtractor *mpExtractor;

    int miParamFilterPointNum;

    Vector4d mGroundPlaneNormal;
    bool mbGroundPlaneSet;

    g2o::ellipsoid mObjInitialGuess;
    bool mbObjInitialGuessSet;

    std::vector<g2o::plane *> mvpInitPlanes;
    bool mbInitPlanesSet;
};

} // namespace ORB_SLAM2

#endif // ELLIPSOIDSLAM_SYMMETRY_H
