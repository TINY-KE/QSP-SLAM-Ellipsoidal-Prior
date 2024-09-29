#ifndef ELLIPSOIDSLAM_TEXTUREOPTIMIZER_H
#define ELLIPSOIDSLAM_TEXTUREOPTIMIZER_H

#include <map>
#include <vector>
#include "EllipsoidTexture.h"
#include "TextureFeature.h"
#include "TextureEdges.h"
#include <opencv2/core/eigen.hpp>
#include "Frame.h"
#include "Optimizer.h"

class TextureOptimizerResult
{
public:
    std::map<double, double, std::less<double>> thetaValueMap;
    std::vector<cv::KeyPoint> keyPoints;
    vector<cv::KeyPoint> keyPointsFlected;

    bool empty() const {return keyPoints.size()==0;}

    double chi2;
};

class KeyPointSon : public cv::KeyPoint
{
public:
    KeyPointSon(const cv::KeyPoint &kp);
    bool operator==(const KeyPointSon &kp) const;
    static vector<KeyPointSon> Generate(const vector<cv::KeyPoint> &kps);
};

class TextureOptimizer
{

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    TextureOptimizer();

    std::vector<g2o::ellipsoid*> graphOptimize(std::vector<ORB_SLAM2::Frame *> &pFrames, Measurements& mms, 
    Objects& objs, Matrix3d& mCalib, int rows, int cols);

    g2o::ellipsoid optimizeWithTexture(const g2o::ellipsoid &e, const Vector4d &bbox, const Matrix3d &calib, const cv::Mat &im);
    TextureOptimizerResult GetResult();
    cv::Mat drawXYPlot(const std::map<double, double> &data, double x_min, double x_max, double y_min = 0, double y_max = 0);
    cv::Mat highlightThetaOnPlot(cv::Mat &in, double theta, double x_min, double x_max, double y_min, double y_max);

    void ExtractKeyPointsFromBbox(const cv::Mat& gray, const Vector4d& bbox, std::vector<cv::KeyPoint>& keypoints_points, cv::Mat& descri);
    cv::Mat VisualizeResultOnImage(const cv::Mat& im, const g2o::ellipsoid& e, const Vector4d& bbox, const Matrix3d& calib, const TextureOptimizerResult& result = TextureOptimizerResult());

    void SetPri(bool state);
    void SetTexture(bool state);
    void SetTextureDT(bool state);
    void SetGroundplane(bool state);
    void SetGroundPlaneNormal(const Vector4d& normal);

    void GenerateDtMatFromGray(const cv::Mat& gray, cv::Mat& dtMap, cv::Mat& edges);

    void SetEdgeAnalysis(bool state);

    std::map<double, double, std::less<double>> GetThetaValueMap(g2o::ellipsoid& e_local, const Matrix3d& calib, std::vector<cv::KeyPoint>& keyPoints, cv::Mat& dtMat, int sample_num = 90);
    std::vector<g2o::ellipsoid*> SampleRotationWithTexture(std::vector<Frame *> &pFrames, Measurements& mms, 
        Objects& objs, Matrix3d& mCalib, int rows, int cols);
private:
    TextureOptimizerResult mResult;

    // Optimize settings
    bool mbOpenPri;
    bool mbOpenTexture;
    bool mbOpenTextureDT;
    bool mbOpenGroundplane;
    Vector4d mGroundNormal;

    // 
    bool mbOpenEdgeAnalysis;

    // Hist图像的默认参数
    const int PARAM_HIST_X = 400;
    const int PARAM_HIST_Y = 400;

private:
    void SaveDescriptor(const cv::Mat &mat, const string &name);
    void drawEllipseOnImage(const Vector5d &ellipse, cv::Mat &im, const cv::Scalar &color = cv::Scalar(0, 0, 255));
    cv::Mat GenerateMaskFromBbox(int rows, int cols, const Vector4d &bbox);
    void drawKeypointsPairs(const cv::Mat &im, cv::Mat &imOut, const std::vector<cv::KeyPoint> &kps1, const std::vector<cv::KeyPoint> &kps2, const cv::Scalar &scalar);
};

#endif