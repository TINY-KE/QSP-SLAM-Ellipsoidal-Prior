#ifndef ELLIPSOIDSLAM_TEXTUREEDGES_H
#define ELLIPSOIDSLAM_TEXTUREEDGES_H

#include "include/core/Ellipsoid.h"
#include "include/core/BasicEllipsoidEdges.h"
#include "Thirdparty/g2o/g2o/core/base_multi_edge.h"
#include "Thirdparty/g2o/g2o/types/types_six_dof_expmap.h"
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <vector>

namespace g2o
{
double CalculateEllipsoidTextureCostEdges(const g2o::ellipsoid& e, const std::vector<cv::KeyPoint>& keyPoints, 
        const Matrix3d &calib, const cv::Mat& dtmap, std::vector<cv::KeyPoint>& out_keypointsflected);
        
double CalculateEllipsoidTextureCost(const g2o::ellipsoid& e, const std::vector<cv::KeyPoint>& keyPoints, const cv::Mat& keyPointsDiscri, 
        const Matrix3d &calib, const cv::Mat& gray, std::vector<cv::KeyPoint>& out_keypointsflected);

// camera-object 3D error
class EdgeSE3EllipsoidTexture : public BaseBinaryEdge<1, double, VertexSE3Expmap, VertexEllipsoidXYZABCYaw>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgeSE3EllipsoidTexture():mbParamSet(false){};
    virtual bool read(std::istream &is);
    virtual bool write(std::ostream &os) const;
    void computeError();

    // set
    void setParam(const cv::Mat& imGray, const Vector4d& bbox, const std::vector<cv::KeyPoint>& keypoints, 
        const cv::Mat& descri, const Matrix3d& K);
    
private:
    Matrix3d mCalib;
    cv::Mat mGray;
    Vector4d mBbox;
    std::vector<cv::KeyPoint> mKeyPoints;
    cv::Mat mKeyPointsDiscri;

    bool mbParamSet;
};

// 使用新的 DT 描述子
// camera-object 3D error
class EdgeSE3EllipsoidTextureDT : public BaseBinaryEdge<1, double, VertexSE3Expmap, VertexEllipsoidXYZABCYaw>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    EdgeSE3EllipsoidTextureDT():mbParamSet(false){};
    virtual bool read(std::istream &is);
    virtual bool write(std::ostream &os) const;
    void computeError();

    // set
    void setParam(const cv::Mat& dtMat, const Vector4d& bbox, const std::vector<cv::KeyPoint>& keypoints, const Matrix3d& K);
    
private:
    Matrix3d mCalib;
    cv::Mat mDTMat;
    Vector4d mBbox;
    std::vector<cv::KeyPoint> mKeyPoints;
    
    bool mbParamSet;
};

} // namespace g2o
#endif