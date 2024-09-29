// 本项目用于可视化数据集

#include <iostream>
#include "../func/func.h"
#include "src/tum_rgbd/io.h"

#include "src/evo/evo.h"

using namespace std;
using namespace ORB_SLAM2;
using namespace TUMRGBD;

System* initSystem(int argc,char* argv[], Dataset& dataset)
{
    string strSettingPath = string(argv[1]);
    System* mpSystem = new System(strSettingPath);
    auto pMap = mpSystem->getMap();
    auto pDrawer = mpSystem->getFrameDrawer();
    auto pTracker = mpSystem->getTracker();

    string dataset_path = Config::Get<string>("Dataset.Path.Root");
    string dir_detectResult = dataset_path + "/detection_3d_output/";
    string path_odom = dataset_path+"/groundtruth.txt";  // 使用 gt
    string strDetectionDir = dataset_path + "/bbox/";

    dataset.loadDataset(dataset_path);
    dataset.loadDetectionDir(strDetectionDir);

    // 读取gt点云并可视化.
    string dataset_path_map = Config::Get<string>("Dataset.Path.Map");
    if(dataset_path_map.size()>0)
    {
        if(dataset_path_map[0]=='.')    // 相对路径
            dataset_path_map = dataset_path + "/" + dataset_path_map;
    }
    std::cout << "Load background pointcloud from " << dataset_path_map << "..." << std::endl;
    mpSystem->getTracker()->LoadPointcloud(dataset_path_map, "background_world");
    std::cout << "ok." << std::endl;

    // 加载真实物体并可视化
    string path_gt_objects_ground = dataset_path+"/objects_gt_ground.txt"; // 注意 _ground 表示仅包括位于地面的物体!
    MatrixXd refObjMat = readDataFromFile(path_gt_objects_ground.c_str());
    if(refObjMat.rows()==0){    // 如果不存在ground版本则读原始gt
        string path_gt_objects = dataset_path+"/objects_gt.txt"; 
        refObjMat = readDataFromFile(path_gt_objects.c_str());
    }
    evo::VisualizeEllipsoidsInMat(refObjMat, Vector3d(1.0, 0, 0), pMap, true, true);

    return mpSystem;
}

void LoadAndViewDatas(Dataset& loader, System* pSLAM)
{
    bool config_one_frame_one_key = true;
    // ***************************************

    cv::Mat rgb, depth; Eigen::VectorXd pose;
    pSLAM->CloseOptimization();
    while(!loader.empty())
    {
        bool valid = loader.readFrame(rgb,depth,pose);
        int current_id = loader.getCurrentID();

        Eigen::MatrixXd detMat = loader.getDetectionMat();
        double timestamp = loader.GetCurrentTimestamp();

        if(valid)
        {
            if(config_one_frame_one_key)
            {
                std::cout << "*****************************" << std::endl;
                std::cout << "Press [ENTER] to continue ... [y] for automatic mode. " << std::endl;
                std::cout << "*****************************" << std::endl;
                char key = getchar();
                if(key == 'y') config_one_frame_one_key = false;
            }

            pSLAM->TrackWithObjects(timestamp, pose, detMat, depth, rgb, false);        // Process frame.    

        }

        std::cout << " -> " << loader.getCurrentID() << "/" << loader.getTotalNum() << std::endl;
    }

    std::cout << "Finished all data." << std::endl;
}

int main(int argc,char* argv[])
{
    std::cout << "You are what you think you are." << std::endl;

    if( argc != 2 )
    {
        std::cout << "usage: " << argv[0] << " path_to_settings" << std::endl;
        return 1;
    }

    // Init System
    Dataset dataset;
    System* pSLAM = initSystem(argc, argv, dataset);

    // 开始读取帧，并做可视化
    LoadAndViewDatas(dataset, pSLAM);

    return 0;
}