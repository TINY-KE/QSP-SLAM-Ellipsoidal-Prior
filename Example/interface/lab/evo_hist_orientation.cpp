// author : lzw
// created time : 2020-8-21 
// goal : 为审稿人意见添加对 hist-based orientation estimation 方法的验证，具体为：
//          1) 读取每帧检测结果以及 gt pose
//          2) 关联，评估检测结果的 orientation
//          3) 保存并统计结果

#include <iostream>
#include "../func/func.h"
#include "src/config/Config.h"
#include "src/evo/evo.h"

#include <fstream>

struct AnalyzeResult
{
    int number; // 包含的物体数量
                // 物体误差情况
    
};


MatrixXd generateMatFromOnePose(g2o::SE3Quat& campose_wc, int rows)
{
    VectorXd pose_vec = campose_wc.toVector();
    int timestamp_start = 0;

    MatrixXd output; output.resize(rows, pose_vec.size()+1);
    for(int i=0;i<rows;i++){
        VectorXd pose_vec_with_stamp; pose_vec_with_stamp.resize(pose_vec.size()+1);
        pose_vec_with_stamp << timestamp_start+i, pose_vec;

        output.row(i) = pose_vec_with_stamp;
    }

    return output;
}

MatrixXd GenerateObjMat(std::vector<ellipsoid*>& vec_objs)
{
    MatrixXd objMat;objMat.resize(0, 11);
    static int num_partial_ob = 0;

    for(auto e : vec_objs)
    {
        // 注意此处只统计有效的全局约束!
        if(e->bPointModel) {
            num_partial_ob++;
            continue;
        }

        Vector9d vec = e->toMinimalVector();
        VectorXd vec_instance; vec_instance.resize(11);
        vec_instance << e->miInstanceID, vec, e->miLabel;

        addVecToMatirx(objMat, vec_instance);
    }

    std::cout << " num_partial_ob : " << num_partial_ob << std::endl;

    return objMat;
}

std::map<int, evo::COMPARE_RESULT> AnalyzeFrame(Frame* pFrame, MatrixXd& refObjMat, ORB_SLAM2::Map* pMap)
{
    // AnalyzeResult result;
    // if(pFrame==NULL) return result;

    // 1. 获得对应 gt pose
    g2o::SE3Quat campose_wc = pFrame->cam_pose_Twc;

    // 2. 分析每个物体
    std::vector<ellipsoid*> vpLocalObjects = pFrame->mpLocalObjects;
    int obj_num = vpLocalObjects.size();

    // 做帧级别的整体关联
    MatrixXd estObjMat_local = GenerateObjMat(vpLocalObjects);

    // 转换到世界系下!
    MatrixXd estObjMat = evo::transformObjMat(estObjMat_local, campose_wc);

    // 直接当成建立完整的图，直接做完整评估，取其中角度?... 不，并不是完整统计，而是单独统计.
    // 单独统计也都有的!
    // 只需要将各个matrix在这里都读取出来即可。 简单.
    std::map<int, evo::COMPARE_RESULT> output_results;
    evo::StaticResult output;
    MatrixXd gtTrajMat = generateMatFromOnePose(campose_wc, 3);
    MatrixXd estTrajMat = gtTrajMat;

    // std::cout << "[DEBUG CHECK] gtTrajMat : " << std::endl << gtTrajMat << std::endl; // check done.
    evo::Evaluate(refObjMat, estObjMat, gtTrajMat, estTrajMat, output, output_results, pMap);

    std::cout << "output_results.size : " << output_results.size() << std::endl;

    std::cout << "Wait to continue..." << std::endl;
    getchar();

    return output_results;
}

std::vector<ellipsoid*> initEllipsoidsFromMat(MatrixXd &refObjMat)
{
    int num = refObjMat.rows();
    std::vector<ellipsoid*> vEs;
    for( int i=0; i<num; i++)
    {
        auto vec = refObjMat.row(i);
        g2o::ellipsoid* pE = new g2o::ellipsoid();
        pE->fromMinimalVector(vec.block(1, 0, 9, 1));
        vEs.push_back(pE);
    }
    return vEs;
}

class ExcelTool
{

struct InstanceData
{
    std::map<double,evo::COMPARE_RESULT> results;  // 包含timestamp的数据包
};

public:
    void addNew(double timestamp, std::map<int, evo::COMPARE_RESULT>& input)
    {
        for(auto pair:input )
        {
            database[pair.first].results.insert(std::make_pair(timestamp, pair.second));
        }
        std::cout << " Load " << input.size() << " data for timestamp " << timestamp << std::endl;
    };

    void analyze()
    {
        // 保存输出每个Instance的结果Vec, 以及对应的timestamp
        int ins_num = database.size();
        std::cout << "Analyze total " << ins_num << " objects. " << std::endl;

        ofstream value_txt("./value.txt");
        ofstream timestamp_txt("./timestamp.txt");


        for(auto id_data : database)
        {
            int ins_id = id_data.first;
            InstanceData& data = id_data.second;
            
            int result_num = data.results.size();
            VectorXd value_vec, timestamp_vec;
            value_vec.resize(result_num); timestamp_vec.resize(result_num); 

            int result_id = 0;
            for(auto result : data.results )
            {
                double timestamp = result.first;
                evo::COMPARE_RESULT evo_result = result.second;
                double yaw_diff = evo_result.dis_yaw_arbitrary;

                // 生成Vec?
                value_vec[result_id] = yaw_diff;
                timestamp_vec[result_id] = timestamp;
                result_id++;
            }

            // 输出 value_vec, timestamp_vec?
            value_txt << ins_id << " " << value_vec.transpose() << std::endl;
            timestamp_txt << ins_id << " " << timestamp_vec.transpose() << std::endl;
        }

        value_txt.close();
        timestamp_txt.close();
        std::cout << "Save to ./value.txt, ./timestamp.txt" << std::endl;

        return;
    };

private:
    // 按 instance 存放了观测结果
    std::map<int,InstanceData> database;

    // 希望统计: 每一个 Instance 的所有观测情况 ( yaw 角度 ); 为便于检查，可随时输出对应观测的 timestamp.

};


int main(int argc, char* argv[]) {
    if( argc != 2)
    {
        std::cout << "usage: " << argv[0] << " path_to_settings" << std::endl;
        return 1;
    }

    // -------- Load system and config ---------------
    string strSettingPath = string(argv[1]);
    System* mpSystem = new System(strSettingPath);
    ORB_SLAM2::Map* pMap = mpSystem->getMap();
    
    string dataset_path = Config::Get<string>("Dataset.Path.Root");
    string dataset_path_map = Config::Get<string>("Dataset.Path.Map");
    if(dataset_path_map.size()>0)
    {
        if(dataset_path_map[0]=='.')    // 相对路径
            dataset_path_map = dataset_path + "/" + dataset_path_map;
    }
    string dataset_path_savedir = dataset_path + "/result/";
    string strDetectionDir = dataset_path + "/bbox/";
    string dataset_type = Config::Get<string>("Dataset.Type");

    std::cout << "- settings file: " << strSettingPath << std::endl;
    std::cout << "- dataset_path: " << dataset_path << std::endl;
    std::cout << "- strDetectionDir: " << strDetectionDir << std::endl;
    std::cout << "- dataset_type : " << dataset_type << std::endl;

    string dir_detectResult = dataset_path + "/detection_3d/";
    bool bUseDebugDir = true; // default use 
    if(bUseDebugDir){
        dir_detectResult = dataset_path + "/detection_3d_output/";
        std::cout << " [DEBUG] dir_detect Result : " << dir_detectResult << std::endl;
    }
    string path_odom = dataset_path+"/CameraTrajectoryVO.txt";
    const string path_gt = dataset_path +"/groundtruth.txt";
    string path_gt_objects = dataset_path+"/objects_gt.txt";

    MatrixXd gtMat = readDataFromFile(path_gt.c_str()); // 该方法读取的似乎是 Odometry???
    MatrixXd refObjMat = readDataFromFile(path_gt_objects.c_str());
    // -------------- load done --------------

    // ---- 开始做评估了 ----
    // 注意1: 使用 gt 作为 frame的pose
    path_odom = path_gt;
    std::cout << "[Debug] Use gt as odom in the frames. " << std::endl;

    std::vector<Frame*> vpFrames = LoadDataAsFrames(path_odom, dir_detectResult, 1, 1);
    int frame_num = vpFrames.size();

    std::cout << "Load frame num : " << frame_num << std::endl;
    // std::vector<ellipsoid*> gtEllipsoids = initEllipsoidsFromMat(refObjMat);

    ExcelTool excel;
    for(int i=0;i<frame_num;i++)
    {
        std::map<int, evo::COMPARE_RESULT> results_map = AnalyzeFrame(vpFrames[i], refObjMat, pMap);

        // 处理/保存结果
        // 思考一下这个int 啥意思.   -> 检查: 是 refObjs 里的 instanceID.
        excel.addNew(vpFrames[i]->timestamp, results_map);
    }

    excel.analyze();

    return 0;

} // main
