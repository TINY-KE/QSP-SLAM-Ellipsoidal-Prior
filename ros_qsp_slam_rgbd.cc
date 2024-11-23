/**
* This file is part of https://github.com/JingwenWang95/DSP-SLAM
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>
*/


#include <iostream>
#include <algorithm>
#include <fstream>
#include <chrono>
#include <iomanip>

#include <opencv2/core/core.hpp>

#include "System.h"

#include "src/config/Config.h"

#include "include/utils/file_operate.h"

#include <pangolin/pangolin.h>


#include <sys/resource.h>
#include <iostream>

#include<ros/ros.h>

using namespace std;

/**
 * @brief 加载图像
 * 
 * @param[in] strAssociationFilename     关联文件的访问路径
 * @param[out] vstrImageFilenamesRGB     彩色图像路径序列
 * @param[out] vstrImageFilenamesD       深度图像路径序列
 * @param[out] vTimestamps               时间戳
 */
void LoadImages(const string &strAssociationFilename, vector<string> &vstrImageFilenamesRGB,
                vector<string> &vstrImageFilenamesD, vector<double> &vTimestamps);

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ROS_ASLAM_RGBD");
    ros::start();
    ros::NodeHandle nh;

    if(argc != 5)
    {
        cerr << endl << "Usage: ./ros_qsp_slam_rgbd path_to_vocabulary path_to_settings path_to_sequence path_to_saved_trajectory" << endl;
        return 1;
    }

    vector<double> vTimestamps;

    cv::FileStorage fSettings(string(argv[2]), cv::FileStorage::READ);

    if(fSettings["CoutToFile"].isNamed()) {
        ofstream fout("logs/log.txt"); //文件输出流对象
        streambuf* pOld =cout.rdbuf(fout.rdbuf());
    }

    auto msensor = ORB_SLAM2::System::RGBD;

    if(fSettings["System.mode"].isNamed()) {
        string system_mode = fSettings["System.mode"];
        if (system_mode=="RGBD") {
            msensor = ORB_SLAM2::System::RGBD;
        }
    }

    ORB_SLAM2::System SLAM(argv[1], argv[2], argv[3], msensor);

    struct rusage rusage;
    long long memory_init, memory_last, memory_curr;
    if (getrusage(RUSAGE_SELF, &rusage) == 0) {
        memory_init = rusage.ru_maxrss;
        memory_last = memory_init;
        std::cout << "\nMemory usage: " << (double)memory_init / 1024. << " MB" << std::endl;
    } else {
        std::cerr << "Failed to get memory usage." << std::endl;
    }

    string strSettingsFile = argv[2];

    // Config::CheckParams("configs/standard_param.yaml");

    //  path_to_dataset [path_to_map]
    string dataset_path = argv[3];
    string dataset_path_map = "./dataset.pcd";
    if(dataset_path_map.size()>0)
    {
        if(dataset_path_map[0]=='.')    // 相对路径
            dataset_path_map = dataset_path + "/" + dataset_path_map.substr(2);
    }
    string dataset_path_savedir = dataset_path + "/result/";


    std::string save_map_dir = std::string(argv[4]);

    assert(CreateDirIfNotExist(save_map_dir));

    std::cout << "- settings file: " << strSettingsFile << std::endl;
    std::cout << "- dataset_path: " << dataset_path << std::endl;


    // Vector for tracking time statistics
    vector<float> vTimesTrack;

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;

    while(1)
    {
        ros::spinOnce();

        std::cout << "\n========================================" << std::endl;
        std::cout << "=> Inputting Image "  << std::endl;

        std::chrono::steady_clock::time_point t1_read = std::chrono::steady_clock::now();

        double tframe = ros::Time::now().toSec();

        SLAM.TrackRosRGBD(dataset_path,tframe);
    }

    SLAM.SaveEntireMap(save_map_dir);

    std::string data_source_dir = std::string(argv[3]);
    string traj_path = data_source_dir  + "KeyFrameTrajectory.txt";

    if(fSettings["System.output"].isNamed()) {
        if(fSettings["System.output_path"].isNamed()) {
            traj_path = std::string(fSettings["System.output_path"]) + std::string("/") + std::string(fSettings["System.output"]);
        }
        else{
            traj_path = std::string(argv[3]) + std::string("/") + fSettings["System.output"];
        }
    }
    
    SLAM.SaveKeyFrameTrajectoryTUM(traj_path);

    // Save pointcloud
    bool mbOpenBuilder = Config::Get<int>("Visualization.Builder.Open") == 1;

    assert(CreateDirIfNotExist(dataset_path_savedir));
    if(mbOpenBuilder)
        SLAM.getTracker()->SavePointCloudMap(dataset_path_savedir+"map.pcd");


    // cv::waitKey(0);
    // cv::destroyAllWindows();

    // // Stop all threads
    SLAM.Shutdown();
    cv::destroyAllWindows();

    cout << "End." << endl;
    return 0;
}

//从关联文件中提取这些需要加载的图像的路径和时间戳
void LoadImages(const string &strAssociationFilename, vector<string> &vstrImageFilenamesRGB,
                vector<string> &vstrImageFilenamesD, vector<double> &vTimestamps)
{
    //输入文件流
    ifstream fAssociation;
    //打开关联文件
    fAssociation.open(strAssociationFilename.c_str());
    //一直读取,知道文件结束
    while(!fAssociation.eof())
    {
        string s;
        //读取一行的内容到字符串s中
        getline(fAssociation,s);
        //如果不是空行就可以分析数据了
        if(!s.empty())
        {
            //字符串流
            stringstream ss;
            ss << s;
            //字符串格式:  时间戳 rgb图像路径 时间戳 深度图像路径
            double t;
            string sRGB, sD;
            ss >> t;
            vTimestamps.push_back(t);
            ss >> sRGB;
            vstrImageFilenamesRGB.push_back(sRGB);
            ss >> t;
            ss >> sD;
            vstrImageFilenamesD.push_back(sD);

        }
    }
}


