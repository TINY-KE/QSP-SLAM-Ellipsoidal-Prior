
#ifndef VLAB_H
#define VLAB_H

#include <mutex>
#include <vector>

#include "core/Geometry.h"
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>


using namespace std;

class Labeller
{
public:
    Labeller(const std::string &strSettingPath);

    // Main thread function. Draw points, keyframes, the current camera pose and the last processed
    // frame. Drawing is refreshed according to the camera fps. We use Pangolin.
    void Run();

    // PointCloud
    void ReadPCDFile(const std::string &pcd_filepath);

    // Mesh
    void ReadPLYFile(const std::string &ply_filepath);

    void DrawFrame(float w, float h, float l, Eigen::Matrix4f Two=Eigen::Matrix4f::Identity());
    void DrawCuboid(Eigen::Matrix4f Two, Eigen::Vector3f Scale, bool with_mesh = false);

    // void RequestFinish();

    // void RequestStop();

    // bool isFinished();

    // bool isStopped();

    // void Release();

    // cv::Mat GetFrame();

private:

    // std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> mptrPointCloud;
    std::vector<ORB_SLAM2::PointCloud*> mmPointCloudLists;

    std::vector<pcl::PolygonMesh> mmMeshLists;

    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);


    // bool Stop();

    // 1/fps in ms
    double mT;
    float mImageWidth, mImageHeight;

    float mViewpointX, mViewpointY, mViewpointZ, mViewpointF;

    bool CheckFinish();
    void SetFinish();
    bool mbFinishRequested;
    bool mbFinished;
    std::mutex mMutexFinish;

    bool mbStopped;
    bool mbStopRequested;
    std::mutex mMutexStop;



};


#endif // VLAB_H
	

