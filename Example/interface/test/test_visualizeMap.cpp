// 该文件用于可视化数据： 包括轨迹真值和地图。不做任何变换检查坐标系。

#include <iostream>
#include <fstream>
#include <Eigen/Core>
#include<algorithm>

#include "include/utils/dataprocess_utils.h"

#include <core/Ellipsoid.h>
#include <core/Geometry.h>
#include <core/System.h>
#include <core/Map.h>

#include "Example/interface/func/func.h"

using namespace std;
using namespace Eigen;
using namespace ORB_SLAM2;

ORB_SLAM2::System* pSLAM;
ORB_SLAM2::Map* pMap;

int main(int argc,char* argv[]) {

    if( argc != 2)
    {
        std::cout << "usage: " << argv[0] << " path_to_settings" << std::endl;
        return 1;
    }

    const string path_setting(argv[1]);
    std::cout << "Load settings from : " << path_setting << std::endl;

    // 初始化 可视化系统
    pSLAM = new ORB_SLAM2::System(path_setting);
    pMap = pSLAM->getMap();

    // LOAD Trajectory
    string dataset_path = Config::Get<string>("Dataset.Path.Root");
    string dataset_path_gt = dataset_path + "/groundtruth.txt";

    std::cout << "Load gt from : " << dataset_path_gt << std::endl;
    MatrixXd gtMat = readDataFromFile(dataset_path_gt.c_str());
    Trajectory gtTraj = MatToTrajectory(gtMat);
    pMap->addOneTrajectory(gtTraj, "AlignedGroundtruth");

    // LOAD MAP
    string dataset_path_map = Config::Get<string>("Dataset.Path.Map");
    if(dataset_path_map.size()>0)
    {
        if(dataset_path_map[0]=='.')    // 相对路径
            dataset_path_map = dataset_path + "/" + dataset_path_map;
    }
    if(dataset_path_map.size() > 0){
        const string path_pcd = dataset_path_map;
        std::cout << "Load background pointcloud from " << path_pcd << "..." << std::endl;
        pSLAM->getTracker()->LoadPointcloud(path_pcd, "Map");
        std::cout << "ok." << std::endl;
    }

    while(1);

    return 0;

}

