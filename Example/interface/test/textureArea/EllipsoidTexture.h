#ifndef ELLIPSOIDSLAM_ELLIPSOIDTEXTURE_H
#define ELLIPSOIDSLAM_ELLIPSOIDTEXTURE_H

#include <include/core/Ellipsoid.h>
#include <Map.h>
#include <opencv2/core/core.hpp>

class Circle3D : public g2o::ellipsoid
{
public:
    Circle3D() : g2o::ellipsoid() {}
    Circle3D(const g2o::ellipsoid &e) : g2o::ellipsoid(e) {}
};

class EllipsoidTex : public g2o::ellipsoid
{
public:
    EllipsoidTex(){};
    EllipsoidTex(const g2o::ellipsoid &e) : g2o::ellipsoid(e){};

    Eigen::VectorXd GetCircle();
    Circle3D GenerateCircle(double theta);
    Eigen::MatrixXd Find3DPoint(const Vector2d &uv, const g2o::SE3Quat &camera_wc, const Eigen::Matrix3d &calib) const;
    bool FindNearest3DPoint(Eigen::Vector3d &point, const Vector2d &uv, const g2o::SE3Quat &camera_wc, const Eigen::Matrix3d &calib) const;
    double GetAnglesOfPoint(const Eigen::Vector3d &point_3d) const;
    cv::Mat GenerateAreaMat(int rows, int cols, const g2o::SE3Quat &campose_wc, const Eigen::Matrix3d &calib, ORB_SLAM2::Map *pMap);
    std::vector<cv::Point2i> generateFlectedPoints(const std::vector<cv::Point2i> &pixels, const g2o::SE3Quat &campose_wc, const Eigen::Matrix3d &calib);
    Eigen::Vector3d flectPoint(const Eigen::Vector3d &near_p);

private:
    Vector3d GenerateNormCenterToPixel(const Vector2d& uv, const Matrix3d& calib) const;

};

#endif  // ELLIPSOIDSLAM_ELLIPSOIDTEXTURE_H