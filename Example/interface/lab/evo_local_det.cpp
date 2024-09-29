// Update on 2021-3-21
// By Lzw
// 基于朝向的旋转评估，扩充为完整参数的评估。为了测试单目推断效果。

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
#include "include/core/PriorInfer.h"
#include "include/core/SemanticLabel.h"

#include<opencv2/core/eigen.hpp>

#include <fstream>

struct AnalyzeResult
{
    int number; // 包含的物体数量
                // 物体误差情况
    
};

// 生成一个Mat, 每一行都是同样的一个位姿
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

g2o::ellipsoid InferObject(Vector4d& bbox, const g2o::SE3Quat& campose_wc)
{
    g2o::ellipsoid e;

    int miImageCols = Config::Get<int>("Camera.width");
    int miImageRows = Config::Get<int>("Camera.height");

    Matrix3d calib;
    float fx = Config::Get<double>("Camera.fx"); 
    float fy = Config::Get<double>("Camera.fy");
    float cx = Config::Get<double>("Camera.cx");
    float cy = Config::Get<double>("Camera.cy");
    calib << fx, 0, cx,
            0, fy, cy,
            0, 0, 1;
    priorInfer priinfer(miImageRows, miImageCols, calib);
    // Plane.Groundplane.param   -> plane_local
    cv::Mat paramMat = Config::GetMat("Plane.Groundplane.param");
    if(paramMat.empty()){
        std::cerr << "Please check param Plane.Groundplane.param!" << std::endl;
        return e;
    }
    Eigen::Vector4d plane_param;
    cv::cv2eigen(paramMat, plane_param);
    g2o::plane pl_world(plane_param);
    g2o::plane pl_local = pl_world; pl_local.transform(campose_wc.inverse());
    e = priinfer.GenerateInitGuess(bbox, pl_local.param); // 只需要一帧!

    // 接着是优化
    Pri pri(1,1);
    double weight = Config::ReadValue<double>("SemanticPrior.Weight");
    g2o::ellipsoid e_opt = priinfer.MonocularInfer(e, pri, weight, pl_local);
    return e_opt;
}

bool IsValidObservation(int label, Vector4d& bbox)
{
    int cols = Config::Get<int>("Camera.width");
    int rows = Config::Get<int>("Camera.height");

    // Check : 确保该物体类型是在地面之上的
    if(!CheckLabelOnGround(label)) return false;

    // Check : 该 bbox 不在边缘
    bool is_border = calibrateMeasurement(bbox, rows, cols, Config::Get<int>("Measurement.Border.Pixels"), Config::Get<int>("Measurement.LengthLimit.Pixels"));
    if(is_border) return false;

    return true;

}

// 分析一帧中的观测，与gt做对比，输出 ob_id 到 gt 的关联关系
// det_id : bbox的序号
// est_id : 有效物体的序号
// ref_id : gt物体列表中的序号
std::map<int, int> AssociateFrame(Frame* pFrame, MatrixXd& refObjMat, ORB_SLAM2::Map* pMap)
{
    std::map<int, int> map_det_ref;

    // 1. 获得对应 gt pose
    g2o::SE3Quat campose_wc = pFrame->cam_pose_Twc;

    // 2. 分析每个物体
    // std::vector<ellipsoid*> vpLocalObjects = pFrame->mpLocalObjects;
    // 从ob中生成局部立方体, 以保留ob顺序?
    std::map<int,int> mapEstToDetId;
    std::vector<ellipsoid*> vpLocalObjects;
    int obj_num = pFrame->mmObservations.rows();

    if(obj_num == 0) return map_det_ref;
    for(int i=0;i<obj_num;i++)
    {
        // Measurement& m = pFrame->meas[i];
        VectorXd bboxVec = pFrame->mmObservations.row(i);
        Vector4d bbox = bboxVec.head(5).tail(4);
        int label = round(bboxVec[5]);

        if(!IsValidObservation(label, bbox)) continue;
        
        g2o::ellipsoid *pObj = new g2o::ellipsoid(InferObject(bbox, campose_wc));
        pObj->miLabel = label;

        // 如果成功了，加入一条
        mapEstToDetId.insert(make_pair(vpLocalObjects.size(), i));
        vpLocalObjects.push_back(pObj);
    }

    // 做帧级别的整体关联
    MatrixXd estObjMat_local = GenerateObjMat(vpLocalObjects);

    // 转换到世界系下!
    MatrixXd estObjMat = evo::transformObjMat(estObjMat_local, campose_wc);

    // 直接当成建立完整的图，直接做完整评估，取其中角度?... 不，并不是完整统计，而是单独统计.
    // 单独统计也都有的!
    // 只需要将各个matrix在这里都读取出来即可。 简单.
    evo::StaticResult output;
    MatrixXd gtTrajMat = generateMatFromOnePose(campose_wc, 3);
    MatrixXd estTrajMat = gtTrajMat;

    // std::cout << "[DEBUG CHECK] gtTrajMat : " << std::endl << gtTrajMat << std::endl; // check done.
    std::map<int, evo::COMPARE_RESULT> output_results;
    evo::Evaluate(refObjMat, estObjMat, gtTrajMat, estTrajMat, output, output_results, pMap, false); // true: 手动检查&可视化
    // std::cout << "output_results.size : " << output_results.size() << std::endl;

    // std::cout << "Associating " << pFrame->timestamp << std::endl;
    // getchar();

    // 此处 estObjMat 的id 与 ob_id 一一对应
    // 注意 output_results 里存储了 ref_id 与 est_id 的对应关系，加上 est_id 与 det_id的关系即可转换了
    for(auto &result : output_results)
    {
        int est_id = result.second.est_id;
        int ref_id = result.second.instanceID;
        int det_id = mapEstToDetId[est_id];
        map_det_ref.insert(make_pair(det_id, ref_id));
    }

    return map_det_ref;
}

std::map<int, evo::COMPARE_RESULT> AnalyzeFrame(Frame* pFrame, MatrixXd& refObjMat, ORB_SLAM2::Map* pMap)
{
    std::map<int, evo::COMPARE_RESULT> output_results;
    // AnalyzeResult result;
    // if(pFrame==NULL) return result;

    // 1. 获得对应 gt pose
    g2o::SE3Quat campose_wc = pFrame->cam_pose_Twc;

    // 2. 分析每个物体
    std::vector<ellipsoid*> vpLocalObjects = pFrame->mpLocalObjects;
    int obj_num = vpLocalObjects.size();
    if(obj_num == 0) return output_results;

    // 做帧级别的整体关联
    MatrixXd estObjMat_local = GenerateObjMat(vpLocalObjects);

    // 转换到世界系下!
    MatrixXd estObjMat = evo::transformObjMat(estObjMat_local, campose_wc);

    // 直接当成建立完整的图，直接做完整评估，取其中角度?... 不，并不是完整统计，而是单独统计.
    // 单独统计也都有的!
    // 只需要将各个matrix在这里都读取出来即可。 简单.
    evo::StaticResult output;
    MatrixXd gtTrajMat = generateMatFromOnePose(campose_wc, 3);
    MatrixXd estTrajMat = gtTrajMat;

    // std::cout << "[DEBUG CHECK] gtTrajMat : " << std::endl << gtTrajMat << std::endl; // check done.
    evo::Evaluate(refObjMat, estObjMat, gtTrajMat, estTrajMat, output, output_results, pMap, false); // true: 手动检查&可视化

    std::cout << "output_results.size : " << output_results.size() << std::endl;

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
    enum VALUE_TYPE
    {
        VALUE_TYPE_IoU = 0,
        VALUE_TYPE_IoU_aligned = 1,
        VALUE_TYPE_dis_trans = 2,
        VALUE_TYPE_dis_yaw = 3,
        VALUE_TYPE_dis_yaw_arbitrary = 4,
        VALUE_TYPE_IoU_aligned_arbitrary = 5
    };

    void addNew(double timestamp, std::map<int, evo::COMPARE_RESULT>& input)
    {
        for(auto pair:input )
        {
            database[pair.first].results.insert(std::make_pair(timestamp, pair.second));
        }
        std::cout << " Load " << input.size() << " data for timestamp " << std::setprecision(16) << timestamp << std::endl;
    };

    void analyze(VALUE_TYPE type)
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
                double value; 
                switch(type)
                {
                    case VALUE_TYPE_dis_yaw_arbitrary: value = evo_result.dis_yaw_arbitrary;
                    case VALUE_TYPE_dis_trans: value = evo_result.dis_trans;
                    case VALUE_TYPE_dis_yaw: value = evo_result.dis_yaw;
                    case VALUE_TYPE_IoU: value = evo_result.IoU;
                    case VALUE_TYPE_IoU_aligned: value = evo_result.IoU_aligned;
                    case VALUE_TYPE_IoU_aligned_arbitrary: value = evo_result.IoU_aligned_arbitrary;
                }
                
                // 生成Vec?
                value_vec[result_id] = value;
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

std::map<int, std::map<double, int>> AssociateDetectionToGt(const std::vector<Frame*> &vpFrames, 
        MatrixXd& refObjMat, ORB_SLAM2::Map* pMap)
{
    // 最后希望生成的 List 为:
    //  Gt -> { [timestamp, ob_id], ...   }
    std::map<int, std::map<double, int>> objObs;    // 应该专门定义一种数据结构处理?

    // 对于每一帧
    for(auto pFrame:vpFrames)
    {
        double timestamp = pFrame->timestamp;
        // 生成的应该是该帧内的 gt -> ob_id 对应关系
        std::map<int, int> detidToRef = AssociateFrame(pFrame, refObjMat, pMap);

        // 将时间戳装载，并且存入更大的数组中
        for(auto &obid_gt:detidToRef)
        {
            objObs[obid_gt.second].insert(std::make_pair(timestamp, obid_gt.first));
        }
    }

    return objObs;
}

void SaveAssociationToFile(const std::map<int, std::map<double, int>>& objObs, const string & path)
{
    ofstream out(path.c_str());
    int gt_num = objObs.size();
    out << gt_num << std::endl;
    for(auto& gt_obs : objObs)
    {
        int gt_id = gt_obs.first;
        int ob_num = gt_obs.second.size();
        out << gt_id << " " << ob_num << std::endl;
        for(auto &ob : gt_obs.second)
        {
            double timestamp = ob.first;
            int ob_id = ob.second;
            out << to_string(timestamp) << " " << ob_id << std::endl;
        }
    }
    return ;
}

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

    std::cout << "Load background pointcloud from " << dataset_path_map << "..." << std::endl;
    mpSystem->getTracker()->LoadPointcloud(dataset_path_map, "background_world");
    std::cout << "ok." << std::endl;

    string dataset_path_savedir = dataset_path + "/result/";
    string strDetectionDir = dataset_path + "/bbox/";
    string dataset_type = Config::Get<string>("Dataset.Type");

    std::cout << "- settings file: " << strSettingPath << std::endl;
    std::cout << "- dataset_path: " << dataset_path << std::endl;
    std::cout << "- strDetectionDir: " << strDetectionDir << std::endl;
    std::cout << "- dataset_type : " << dataset_type << std::endl;

    string dir_detectResult = dataset_path + "/detection_3d/";

    bool bUseCubeSLAM = true;

    bool bUseDebugDir = true; // default use 
    if(bUseDebugDir){
        if(bUseCubeSLAM)
            dir_detectResult = dataset_path + "/detection_3d_output_cubeslam/";
        else 
            dir_detectResult = dataset_path + "/detection_3d_output/";
        std::cout << " [DEBUG] dir_detect Result : " << dir_detectResult << std::endl;
    }
    string path_odom = dataset_path+"/CameraTrajectoryVO.txt";
    const string path_gt = dataset_path +"/groundtruth.txt";
    MatrixXd gtMat = readDataFromFile(path_gt.c_str()); // 该方法读取的似乎是 Odometry???

    string path_gt_objects_ground = dataset_path+"/objects_gt_ground.txt"; // 注意 _ground 表示仅包括位于地面的物体!
    MatrixXd refObjMat = readDataFromFile(path_gt_objects_ground.c_str());
    if(refObjMat.rows()==0){    // 如果不存在ground版本则读原始gt
        string path_gt_objects = dataset_path+"/objects_gt.txt"; 
        refObjMat = readDataFromFile(path_gt_objects.c_str());
    }
    
    // -------------- load done --------------

    // ---- 开始做评估了 ----
    // 注意1: 使用 gt 作为 frame的pose
    path_odom = path_gt;
    std::cout << "[Debug] Use gt as odom in the frames. " << std::endl;

    int iType = 1;
    if(bUseCubeSLAM) iType = 2;
    std::vector<Frame*> vpFrames = LoadDataAsFrames(path_odom, dir_detectResult, iType, 1);
    int frame_num = vpFrames.size();

    std::cout << "Load frame num : " << frame_num << std::endl;
    // std::vector<ellipsoid*> gtEllipsoids = initEllipsoidsFromMat(refObjMat);

    // 关联检测并且存储
    bool bOpenAssociate = false;
    if(bOpenAssociate){
        std::map<int, std::map<double, int>> objObs = AssociateDetectionToGt(vpFrames, refObjMat, pMap);
        SaveAssociationToFile(objObs, "./gt_associate.txt");
    }

    // DEBUG here

    ExcelTool excel;
    int count_success_objects = 0;
    int count_det_objects = 0;

    bool config_one_frame_one_key = true;
    for(int i=0;i<frame_num;i++)
    {
        Frame* pFrame = vpFrames[i];
        std::map<int, evo::COMPARE_RESULT> results_map = AnalyzeFrame(pFrame, refObjMat, pMap);

        // 处理/保存结果
        // 思考一下这个int 啥意思.   -> 检查: 是 refObjs 里的 instanceID.
        excel.addNew(pFrame->timestamp, results_map);

        if(results_map.size()==0) continue;

        // 统计数量
        count_success_objects+=results_map.size();
        count_det_objects+=pFrame->mpLocalObjects.size();

        if(config_one_frame_one_key){
            std::cout << "Wait to continue... [y for automation]" << std::endl;
            char key = getchar();
            if(key == 'y') config_one_frame_one_key = false;
        }
    }

    excel.analyze(ExcelTool::VALUE_TYPE_IoU);   // 单独测试IoU的效果

    std::cout << "COUNT: " << std::endl;
    std::cout << " - Detection Objects: " << count_det_objects << std::endl;
    std::cout << " - Success Objects: " << count_success_objects << std::endl;
    std::cout << " - Success Percent: " << count_success_objects/double(count_det_objects) << std::endl;

    return 0;

} // main
