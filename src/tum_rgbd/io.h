// Basic input and output of the TUM-RGBD dataset.
#ifndef ELLIPSOIDSLAM_IO_H
#define ELLIPSOIDSLAM_IO_H

#include <string>
#include <iostream>
#include <vector>

#include <opencv2/opencv.hpp>
#include <Eigen/Core>

#include <map>
#include "Thirdparty/g2o/g2o/types/types_six_dof_expmap.h"

using namespace std;
using namespace cv;
using namespace Eigen;

namespace TUMRGBD
{

enum BBOX_STORAGE
{
    BBOX_STORAGE_TWO_POINT = 0,
    BBOX_STORAGE_YOLO = 1
};

/*
*   All the timestamp is based on RGB images.
*/
class Dataset
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    // Load the next RGB-D frame containing a RGB image, a depth image and a camera pose.
    // only images with groundtruth are valid.
    // pose: x y z qx qy qz qw
    bool readFrame(cv::Mat &rgb, cv::Mat &depth, Eigen::VectorXd &pose);

    // Load only images.
    bool readImageFrame(cv::Mat &rgb, cv::Mat &depth);

    void loadDataset(string &path);
    std::vector<int> generateValidVector();

    // for those depth image storing euclidean distance,  e.g. InteriorNet, ICL-NUIM, please open it.
    // for TUM-RGB-D, ignore it.
    void OpenDepthPreprocess(const Eigen::Matrix3d &K);

    // For those use YOLO origin output: id center.x center.y width height label prob, open this.
    // if bbox saves as id x1 y1 x2 y2 label prob, ignore it
    void SetBboxStorage(int type);

    // load object detections
    bool loadDetectionDir(string &path);
    Eigen::MatrixXd getDetectionMat(int id = -1);

    bool empty();

    int getCurrentID();
    int getTotalNum();

    // load a specified frame using its ID
    bool findFrameUsingID(int id, cv::Mat &rgb, cv::Mat &depth, Eigen::VectorXd &pose);

    // load an odometry data generated from wheels or visual odometry algorithm like ORB-SLAM.
    // the system will automatically calibrate its coordinate by aligning the pose of the first frame to the corresponding ground truth.
    bool SetOdometry(const string &dir_odom, bool calibrate = true);

    bool SetOdometryAndLoadAssociate(const string &dir_odom, const string &dir_associate, bool calibrate = true);

    // jump to a specified frame using its ID
    void SetCurrentID(int id);

    // Get the timestamp of the current frame
    double GetCurrentTimestamp();
    double GetTimestamp(int id);

    bool getPoseFromTimeStamp(string &timestamp, VectorXd &pose);

    MatrixXd GetGroundtruthTrajectory();

    bool findFrameUsingTimestamp(double timestamp, cv::Mat &rgb, cv::Mat &depth, Eigen::VectorXd &pose);
    int findFrameUsingTimestampGetId(double timestamp, cv::Mat &rgb, cv::Mat &depth, Eigen::VectorXd &pose);

    std::string GetDatasetDir();

private:
    void associateRGBWithGroundtruth();

    void loadGroundTruthToMap(string &path);

    // load the associations from the timestamps of rgb images to the timestamps of depth images
    void LoadAssociationRGBToDepth(string &path);
    void LoadAssociationRGBToGroundtruth(const string &path);

    void generateIndexIdToRGBTimeStamp();

    bool judgeValid(int id); // the frame with the id is valid when it contains valid depth and rgb images ...

    bool getPoseFromRGBTimeStamp(string &timestamp, VectorXd &pose);
    VectorXd calibratePose(VectorXd &pose); // calibrate the odom data by aligning to the first frame of the groundtruth

    cv::Mat ReadDepthImage(const string &name);
    cv::Mat preprocessDepth(cv::Mat &depth, double fx, double cx, double cy);

private:
    string msDatasetDir; // the root directory of the dataset

    string msRGBDir;
    string msDepthDir;
    string msGroundtruthPath;

    string msAssociatePath;
    string msAssociateGroundtruthPath;

    string msDetectionDir;

    vector<string> mvRGBFileNames; // store the full paths of all the rgb images

    vector<VectorXd> mvPoses;

    int miCurrentID;
    int miTotalNum;

    vector<string> mvIdToGroundtruthTimeStamp;
    vector<string> mvIdToDepthTimeStamp;
    vector<string> mvIdToDepthImagePath;
    vector<string> mvIdToRGBTimeStamp;

    map<string, VectorXd> mmTimeStampToPose;
    map<string, VectorXd> mmOdomRGBStampToPose;

    bool mbDetectionLoaded;

    bool mbOdomSet;
    string msOdomDir;
    g2o::SE3Quat *mTransGtCalibrate;

    bool mbDepthPreprocess;
    Eigen::Matrix3d mK;

    int mBboxType;
};

} // namespace TUMRGBD

#endif