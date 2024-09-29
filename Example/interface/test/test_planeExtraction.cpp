// 该文件根据用户的参数变化来重新执行完整的一帧.
// 如何实现这类变化?

// Copy from file rgbd_autoDA.cpp
/*  This file is created on 2020-4-22 by Ziwei Liao.
*   In order to test the automatic probabilistic data association, based on paper:
*   IROS 2017, MIT.
*/

#include "Initializer.h"
#include "core/Geometry.h"
#include "utils/dataprocess_utils.h"
#include "utils/matrix_utils.h"

#include <Eigen/Core>

#include "Viewer.h"
#include "MapDrawer.h"
#include "Map.h"

#include <thread>
#include <string>

#include "include/core/Ellipsoid.h"
#include "src/tum_rgbd/io.h"

#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudPCL;

#include "src/config/Config.h"

#include<opencv2/core/eigen.hpp>

#include <chrono>

using namespace std;
using namespace Eigen;

inline bool diff(double a1, double a2)
{
    return (std::abs(a1-a2)>0.01);
}

int main(int argc,char* argv[]) {
    if( argc != 2)
    {
        std::cout << "usage: " << argv[0] << " path_to_settings" << std::endl;
        return 1;
    }
    string strSettingPath = string(argv[1]);

    ORB_SLAM2::System SLAM(strSettingPath, true);
    SLAM.CloseOptimization();   // close incremental calculation; the full optimization will be processed after the last frame.


    //  path_to_dataset [path_to_map]
    string dataset_path = Config::Get<string>("Dataset.Path.Root");
    string dataset_path_map = Config::Get<string>("Dataset.Path.Map");
    if(dataset_path_map.size()>0)
    {
        if(dataset_path_map[0]=='.')    // 相对路径
            dataset_path_map = dataset_path + "/" + dataset_path_map;
    }
    string dataset_path_savedir = dataset_path + "/result/";

    string strDetectionDir = dataset_path + "/bbox/";

    std::cout << "- settings file: " << strSettingPath << std::endl;
    std::cout << "- dataset_path: " << dataset_path << std::endl;
    std::cout << "- strDetectionDir: " << strDetectionDir << std::endl;
    string dataset_type = Config::Get<string>("Dataset.Type");
    std::cout << "- dataset_type : " << dataset_type << std::endl;

    TUMRGBD::Dataset loader;
    loader.loadDataset(dataset_path);
    loader.loadDetectionDir(strDetectionDir);
    
    // set groundplane mannually
    cv::Mat paramMat = Config::GetMat("Plane.Groundplane.param");
    if(!paramMat.empty())
    {
        Eigen::Vector4d plane_param;
        cv::cv2eigen(paramMat, plane_param);
        SLAM.getTracker()->SetGroundPlaneMannually(plane_param);
    }
    
    double start_time_stamp = Config::Get<double>("Dataset.Timestamp.Start");
    double end_time_stamp = Config::Get<double>("Dataset.Timestamp.End");
    bool config_timestamp_thresh = (start_time_stamp > 0 || end_time_stamp > 0);
    if(config_timestamp_thresh)
    {
        std::cout << "[ start_time_stamp : " << start_time_stamp << std::endl;
        std::cout << "[ end_time_stamp : " << end_time_stamp << std::endl;

        for(int goal_id = 0; goal_id<loader.getTotalNum(); goal_id++){
            double current_time_stamp = loader.GetTimestamp(goal_id);
            if(current_time_stamp>start_time_stamp) 
            {
                loader.SetCurrentID(goal_id);
                std::cout << "Set current ID to : " << goal_id << std::endl;
                break;
            }
        }
    }

    std::chrono::system_clock::time_point a = std::chrono::system_clock::now();
    std::chrono::system_clock::time_point b = std::chrono::system_clock::now();
    cv::Mat rgb,depth;
    VectorXd pose;
    Eigen::MatrixXd detMat;
    double timestamp;


    bool config_one_frame_one_key = true;

    // get to the goal frame
    int jump_frame = Config::Get<int>("Dataset.Jumpframe");
    bool bJumpFrame = (jump_frame > 0);
    int last_frame_id = 0;

    bool bFindFrame = false;
    while(!loader.empty())
    {
        bool valid = loader.readFrame(rgb,depth,pose);
        int current_id = loader.getCurrentID();

        detMat = loader.getDetectionMat();
        timestamp = loader.GetCurrentTimestamp();

        if( config_timestamp_thresh && timestamp > end_time_stamp) break;

        if(valid)
        {
            // jump frame
            if(bJumpFrame){
                if((current_id - last_frame_id) < jump_frame)
                {
                    continue;
                }
                last_frame_id = current_id;
            }

            std::cout << " Find the frame." << std::endl;
            bFindFrame = true;
            break; // get out of here
        }
        std::cout << " -> " << loader.getCurrentID() << "/" << loader.getTotalNum() << std::endl;
    }

    // 添加参数滚动条
    auto pViewer = SLAM.getViewer();
    // Plane.MinSize: 1000
    // Plane.AngleThreshold: 3
    // Plane.DistanceThreshold: 0.05

    // double dMinSizeLast = Config::Get<double>("Plane.MinSize"); 
    // double dAngleLast = Config::Get<double>("Plane.AngleThreshold"); 
    // double dDistanceLast = Config::Get<double>("Plane.DistanceThreshold"); 
    double dMinSizeLast = 0, dAngleLast = 0, dDistanceLast = 0;
    double dMinSizeCurrent, dAngleCurrent, dDistanceCurrent;

    pViewer->addDoubleMenu("----PlaneDebug---",0,1,0);
    int iMenuMinsize = pViewer->addDoubleMenu("MinSize",0,10000,3000);
    int iMenuAngle = pViewer->addDoubleMenu("AngleThreshold",0,20,2.0);
    int iMenuDistance = pViewer->addDoubleMenu("DistanceThreshold",0,0.3,0.1);
    
    std::cout << "Wait for one second..." << std::endl;
    usleep(1000);

    while(1)
    {
        bool bRun = false;
    
        // 判断动态参数.
        pViewer->getValueDoubleMenu(iMenuMinsize, dMinSizeCurrent);
        pViewer->getValueDoubleMenu(iMenuAngle, dAngleCurrent);
        pViewer->getValueDoubleMenu(iMenuDistance, dDistanceCurrent);

        if( diff(dMinSizeLast, dMinSizeCurrent) || diff(dAngleLast, dAngleCurrent) || diff(dDistanceLast, dDistanceCurrent) )
        {
            Config::SetValue<double>("Plane.MinSize", dMinSizeCurrent);    
            Config::SetValue<double>("Plane.AngleThreshold", dAngleCurrent); 
            Config::SetValue<double>("Plane.DistanceThreshold", dDistanceCurrent); 
            
            dMinSizeLast = dMinSizeCurrent;
            dAngleLast = dAngleCurrent;
            dDistanceLast = dDistanceCurrent;
            bRun = true;   
        }

        if(bRun){
            std::cout << "[ Param ]" << std::endl;
            std::cout << " - dMinSizeCurrent : " << dMinSizeLast << std::endl;
            std::cout << " - dAngleCurrent : " << dAngleLast << std::endl;
            std::cout << " - dDistanceCurrent : " << dDistanceLast << std::endl;

            SLAM.getTracker()->OpenGroundPlaneEstimation();
            SLAM.TrackWithObjects(timestamp, pose, detMat, depth, rgb, false);        // Process frame.    
            std::cout << std::endl;
            bRun=false;
        }
        
    }

    
    cout << "Use Ctrl+C to quit." << endl;
    while(1);

    cout << "End." << endl;
    return 0;
}