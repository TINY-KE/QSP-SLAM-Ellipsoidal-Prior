#ifndef ELLIPSOIDSLAM_TESTFUNCTIONS_H
#define ELLIPSOIDSLAM_TESTFUNCTIONS_H

#include <iostream>
#include "TextureOptimizer.h"

#include "../../func/func.h"
#include "src/tum_rgbd/io.h"

#include "include/core/Initializer.h"
#include "Optimizer.h"

#include "include/core/PriorInfer.h"
#include <boost/algorithm/string.hpp>

class FrameData
{
public:
    double timestamp;
    cv::Mat image;
    g2o::SE3Quat cam_pose_Twc;
    Vector4d bbox;
    int label;
};

const int OPTIMIZE_TYPE_STANDARD = 0x01;  // 0000 0001
const int OPTIMIZE_TYPE_GROUND = 0x02;  // 0000 0010
const int OPTIMIZE_TYPE_PRI = 0x04;  // 0000 0100
const int OPTIMIZE_TYPE_TEXTURE = 0x08;  // 0000 1000
const int OPTIMIZE_TYPE_TEXTURE_DT = 0x10;  // 0001 0000
const int OPTIMIZE_TYPE_QUADRICSLAM = 0x20;  // 0010 0000


string histToTxt(const std::map<double,double,std::less<double>>& hist);

void drawEllipseOnImage(const Vector5d& ellipse, cv::Mat& im, const cv::Scalar& color);

MatrixXd GeneratePoseMat(const std::vector<FrameData>& frameDatas);

MatrixXd GenerateDetectionMat(const std::vector<FrameData>& frameDatas);

g2o::ellipsoid InitWithSVD(const std::vector<FrameData>& frameDatas, Matrix3d& calib, int rows, int cols);

g2o::ellipsoid InitWithGroundPlanePri(const std::vector<FrameData>& frameDatas, Matrix3d& calib, int rows, int cols);

g2o::ellipsoid InitializeQuadrics(const std::vector<FrameData>& frameDatas, Matrix3d& calib, int rows, int cols, int METHOD = 1);

std::vector<Frame *> GenerateFrames(const std::vector<FrameData>& frameDatas);

Measurements GenerateMeasurements(const std::vector<FrameData>& frameDatas, std::vector<Frame *>& pFrames);

Objects GenerateObjects(const g2o::ellipsoid& e, Measurements& mms);

void InitTextureMeasurements(Measurements& mms);

void VisualizeTextureResults(Measurements& mms, Objects& objs, const Matrix3d& calib, int rows, int cols);

void DebugTestingRunningTime(Measurements& mms, Objects& objs, const Matrix3d& calib, int rows, int cols);

g2o::ellipsoid OptimizeFrameDatas(const g2o::ellipsoid& e, std::vector<Frame *>& pFrames, Measurements& mms, Objects& objs,
    Matrix3d& calib, int rows, int cols, int type);


g2o::ellipsoid OptimizeFrameDatasTexture(const g2o::ellipsoid& e, std::vector<Frame *>& pFrames, Measurements& mms, Objects& objs,
    Matrix3d& calib, int rows, int cols);

System* initSystem(int argc,char* argv[], TUMRGBD::Dataset& dataset);

std::vector<FrameData> GenerateFrameDatas(const std::map<double, int>& mapFrameObj, TUMRGBD::Dataset& dataset);

void drawProjectionOnImage(const g2o::SE3Quat& pose_cw, g2o::ellipsoid e_opt, cv::Mat& im, const Matrix3d& calib);

void VisualizeOneFrameData(FrameData& data, System* pSLAM);
void VisualizeFrameData(std::vector<FrameData>& frameDatas, System* pSLAM);

void VisualizeFrameData(std::vector<FrameData>& frameDatas, System* pSLAM, const Matrix3d& calib, 
    const std::vector<g2o::ellipsoid*>& vElips);

void OutPutFrameData(const std::vector<FrameData>& frameDatas);

std::map<double, int> LoadFrameObjSettingsStatic();

// 准备从已有的文件中读取
std::map<int,std::map<double, int>> LoadFrameObjSettingsFromFile(const string& fileName);

// std::vector<std::map<double, int>> LoadFrameObjSettingsFromFile(const string& dir)
// {

// }

void OutputObjectObservation(const std::map<double, int>& mapFrameObj);

#endif