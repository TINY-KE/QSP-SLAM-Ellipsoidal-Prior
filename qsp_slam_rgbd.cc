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
    if(argc != 6)
    {
        cerr << endl << "Usage: ./qsp_slam_rgbd path_to_vocabulary path_to_settings path_to_sequence path_to_association path_to_saved_trajectory" << endl;
        return 1;
    }

    vector<string> vstrImageFilenamesRGB;
    vector<string> vstrImageFilenamesD;
    vector<double> vTimestamps;
    string strAssociationFilename = string(argv[4]);

    cv::FileStorage fSettings(string(argv[2]), cv::FileStorage::READ);

    if(fSettings["CoutToFile"].isNamed()) {
        ofstream fout("logs/log.txt"); //文件输出流对象
        streambuf* pOld =cout.rdbuf(fout.rdbuf());
    }

    LoadImages(strAssociationFilename, vstrImageFilenamesRGB, vstrImageFilenamesD, vTimestamps);

    int nImages = vstrImageFilenamesRGB.size();

    if(vstrImageFilenamesRGB.empty()){
        cerr << endl << "No images found in provided path." << endl;
        return 1;
    }
    else if(vstrImageFilenamesD.size()!=vstrImageFilenamesRGB.size()){
        cerr << endl << "Different number of images for rgb and depth." << endl;
        return 1;
    }

    auto msensor = ORB_SLAM2::System::MONOCULAR;

    if(fSettings["System.mode"].isNamed()) {
        string system_mode = fSettings["System.mode"];
        if (system_mode=="RGBD") {
            msensor = ORB_SLAM2::System::RGBD;
        }
    }

    // Create SLAM system. It initializes all system threads and gets ready to process frames.
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
    string dataset_path = Config::Get<string>("Dataset.Path.Root");
    string dataset_path_map = Config::Get<string>("Dataset.Path.Map");
    if(dataset_path_map.size()>0)
    {
        if(dataset_path_map[0]=='.')    // 相对路径
            dataset_path_map = dataset_path + "/" + dataset_path_map.substr(2);
    }
    string dataset_path_savedir = dataset_path + "/result/";

    string strDetectionDir = dataset_path + "/bbox/";

    std::string save_map_dir = std::string(argv[5]);
    std::string data_source_dir = std::string(argv[3]);

    assert(CreateDirIfNotExist(save_map_dir));

    std::cout << "- settings file: " << strSettingsFile << std::endl;
    std::cout << "- dataset_path: " << dataset_path << std::endl;
    std::cout << "- strDetectionDir: " << strDetectionDir << std::endl;
    string dataset_type = Config::Get<string>("Dataset.Type");
    std::cout << "- dataset_type : " << dataset_type << std::endl;

    // 这里的跳步对后续的图像物体检测造成了影响
    int skip_num = Config::Get<int>("Running.skip_num");
    double total_ratio = Config::Get<double>("Running.total_ratio");

    cout << "nImages = " << nImages << std::endl;

    int total_num = std::min(int(total_ratio * (double)nImages + 1), nImages);
    skip_num = std::max(1, skip_num);

    std::cout << "total_num = " << total_num << ", skip_num = " << skip_num << std::endl;

    // jumpResize(vstrImageFilenamesRGB, );
    int new_index = 0, raw_index = 0;
    for(; raw_index < total_num; new_index+=1, raw_index+=skip_num) {
        vstrImageFilenamesRGB[new_index] = vstrImageFilenamesRGB[raw_index];
        vstrImageFilenamesD[new_index] = vstrImageFilenamesD[raw_index];
        vTimestamps[new_index] = vTimestamps[raw_index];
    }
    nImages = new_index;
    vstrImageFilenamesRGB.resize(nImages);
    vstrImageFilenamesD.resize(nImages);
    vTimestamps.resize(nImages);


    SLAM.SetImageNames(vstrImageFilenamesRGB);

    // Vector for tracking time statistics
    vector<float> vTimesTrack;
    vTimesTrack.resize(nImages);

    bool frame_by_frame = fSettings["frame_by_frame"].isNamed();

    cout << endl << "-------" << endl;
    cout << "Start processing sequence ..." << endl;
    cout << "Images in the sequence: " << nImages << endl << endl;

    // Main loop
    cv::Mat imRGB, imD;

    for(int ni = 0; ni < nImages; ni++)
    {
        std::cout << "\n========================================" << std::endl;
        std::cout << "=> Inputting Image " << ni << "/" << nImages << std::endl;

        std::chrono::steady_clock::time_point t1_read = std::chrono::steady_clock::now();

        //! 读取图像
        imRGB = cv::imread(string(argv[3])+"/"+vstrImageFilenamesRGB[ni], CV_LOAD_IMAGE_UNCHANGED);
        imD = cv::imread(string(argv[3])+"/"+vstrImageFilenamesD[ni], CV_LOAD_IMAGE_UNCHANGED);

        std::chrono::steady_clock::time_point t2_read = std::chrono::steady_clock::now();
        double t_read = std::chrono::duration_cast<std::chrono::duration<double> >(t2_read - t1_read).count();

        cout << " Reading Image costs: " << t_read << "s" << endl;

        double tframe = vTimestamps[ni];

        if(imRGB.empty())
        {
            cerr << endl << "Failed to load image at: "
                 << string(argv[3]) << "/" << vstrImageFilenamesRGB[ni] << endl;
            return 1;/*  */
        }

        if(frame_by_frame) {
            std::cout << "*****************************" << std::endl;
            std::cout << "Press [ENTER] to continue ... , [y] to autonomous mode" << std::endl;
            std::cout << "*****************************" << std::endl;
            char key = getchar();
            if (key=='y')
            {
                frame_by_frame = false;
            }
            else if (key=='e'){
                break;
            }
        }

        std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

        // cout << "imRGB.type() = " << imRGB.type() << endl;

        // cout << "imD.type() = " << imD.type() << endl;

        assert(imRGB.type()==16);
        assert(imD.type()==2);

        if (msensor == ORB_SLAM2::System::RGBD)
            SLAM.TrackRGBD(imRGB, imD, tframe);
        else if(msensor == ORB_SLAM2::System::MONOCULAR)
            SLAM.TrackMonocular(imRGB, tframe);
        else
            printf("暂时不支持这个模式\n");


        // FIXME: 如有必要，将内存使用的变化情况存储下来检查一下
        if (getrusage(RUSAGE_SELF, &rusage) == 0) {
            memory_curr = rusage.ru_maxrss;

            double me_ratio = memory_curr / memory_init;
            std::cout << "Memory curr/add/init = "
                      << (double)memory_curr / 1024. << " MB /"
                      << (double)(memory_curr - memory_last) / 1024. << "MB /"
                      << (double)memory_init / 1024. << " MB"
                      << std::endl;
            memory_last = memory_curr;
        } else {
            std::cerr << "Failed to get memory usage." << std::endl;
        }

        std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();

        double ttrack = std::chrono::duration_cast<std::chrono::duration<double> >(t2 - t1).count();

        cout << " - [ total_frame: " << (double) ttrack  << "s ]" << endl;

        vTimesTrack[ni]=ttrack;

        // Wait to load the next frame
        double T = 0.0;
        if(ni<nImages-1)
            T = vTimestamps[ni+1]-tframe;
        else if(ni>0)
            T = tframe-vTimestamps[ni-1];

        if(ttrack<T)
        {
            std::this_thread::sleep_for(std::chrono::microseconds(static_cast<size_t>((T- ttrack)*1e6)));
        }

//        if (SLAM.GetTrackingState() == ORB_SLAM2::Tracking::OK)
//            SLAM.SaveMapCurrentFrame(string(argv[4]), ni);
    }

    SLAM.SaveEntireMap(save_map_dir);

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
    // SLAM.Shutdown();

    // Tracking time statistics
    sort(vTimesTrack.begin(),vTimesTrack.end());
    float totaltime = 0;
    for(int ni=0; ni<nImages; ni++)
    {
        totaltime+=vTimesTrack[ni];
    }
    cout << "-------" << endl << endl;
    cout << "median tracking time: " << vTimesTrack[nImages/2] << endl;
    cout << "mean tracking time: " << totaltime/nImages << endl;

    cout << "Press y to quit." << endl;
    char key;
    key = getchar();

    while(key!='y' && key!='Y'){
        cout << "Press y to quit." << endl;
        key = getchar();
    }

    // SLAM.Shutdown();
    cv::destroyAllWindows();

    // SLAM.Shutdown();

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


