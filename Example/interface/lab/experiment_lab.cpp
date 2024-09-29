
// Created by LZW on 2020-5-31
// 说明： 本文件实现 MIT 的IROS17论文。
// 由于Matlab开源版本仅仅支持二维，本文件基于 Ellipsoid::Core 实现三维系统。

#include <iostream>
#include "../func/func.h"
#include "src/config/Config.h"
#include "src/evo/evo.h"

using namespace std;
using namespace ORB_SLAM2;

struct EvoResult
{
    Trajectory gtTrajInEst;
    Trajectory estTraj;
    g2o::SE3Quat Tge;
};

string dataset_path_savedir;

class Runner
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    Runner(const string& path_odom, const string& dir_detectResult, System* pSLAM, int iType = 0):mpSLAM(pSLAM),mbInit(false),mbQuadricSLAM(false),mbInitEvo(false)
    {
        mpMap = mpSLAM->getMap();
        mpOptimizer = new Optimizer();  // 注意这里跟 SLAM 系统不是同一个.
        mpTracker = mpSLAM->getTracker();

        miType = iType; // 0 baseline, 1 ours.

        // 读取参数
        // int iJump = Config::Get<int>("Dataset.Jumpframe");
        // std::cout << "Experiment Jump : " << iJump << std::endl;
        int iJump = 1;  // 此处保持为1， 否则内部可能跳过 3d detection 的 timestamp
        int iJumpSet = Config::Get<int>("Dataset.Jumpframe.exp");
        if(iJumpSet > 0) 
        {
            iJump = iJumpSet;
            std::cout << "Jump set to " << iJump << std::endl;
        }
        mvpFrames = LoadDataAsFrames(path_odom, dir_detectResult, miType, iJump);

        // 读取地平面
        LoadGroundPlane(dir_detectResult);

        mCalib = mpTracker->mCalib;
        miRows = mpTracker->mRows;
        miCols = mpTracker->mCols;
    }

// return True if the measure lies on the image border or does not meet the size requirement
bool isBorder(Vector4d &measure , int rows, int cols, int config_boarder){
    int correct_num = 0;
    if(  measure[0]>config_boarder && measure[0]<cols-1-config_boarder )
    {
        correct_num++;
    }
    if(  measure[2]>config_boarder && measure[2]<cols-1-config_boarder )
    {
        correct_num++;
    }
    if(  measure[1]>config_boarder && measure[1]<rows-1-config_boarder )
    {
        correct_num++;
    }
    if(  measure[3]>config_boarder && measure[3]<rows-1-config_boarder )
    {
        correct_num++;
    }

    if( correct_num == 4)
        return false;
    else
        return true;

}

    // 调试局部约束引入的效果： 即对帧内所有的物体观测结果做判断，滤除无效的
    void RejectAllPartialObservations()
    {
        int frame_num = mvpFrames.size();
        int count_reject_3d_num = 0;
        int count_reject_2d_num = 0;
        int count_valid_3d_num = 0;
        int count_valid_2d_num = 0;
        for(int i=0;i<frame_num;i++)
        {
            Frame* pF = mvpFrames[i];
            Measurements& mms = pF->meas;
            // auto pLocalObjs = pF->mpLocalObjects;
            int obj_size = mms.size();
            Measurements mms_filtered;
            for(int n=0;n<obj_size;n++)
            {
                Vector4d bbox = mms[n].ob_2d.bbox;
                ellipsoid* pE = mms[n].ob_3d.pObj;
                bool border = isBorder(bbox, miRows, miCols, Config::Get<int>("Measurement.Border.Pixels"));
                // bool border = isBorder(bbox, miRows, miCols, 10);
                if(border){
                    if(pE->bPointModel)
                        count_reject_2d_num ++;
                    else
                        count_reject_3d_num ++;

                }
                else 
                {
                    mms_filtered.push_back(mms[n]);
                    if(pE->bPointModel)
                        count_valid_2d_num ++;
                    else
                        count_valid_3d_num ++;
                }
            }

            // update mms
            pF->meas = mms_filtered;
        }

        std::cout << "************** REJECT ALL PARTIAL OBSERVATIONS **************" << std::endl;
        std::cout << " - reject_3d_num : " << count_reject_3d_num << std::endl;
        std::cout << " - reject_2d_num : " << count_reject_2d_num << std::endl;
        std::cout << " - valid_3d_num : " << count_valid_3d_num << std::endl;
        std::cout << " - valid_2d_num : " << count_valid_2d_num << std::endl;
        std::cout << "************** **************" << std::endl;
    }

    void LoadGroundPlane(const string& dir_detectResult)
    {
        string groundplane_file = dir_detectResult + "/environment/groundplane.txt";
        MatrixXd planeMat = readDataFromFile(groundplane_file.c_str());
        if(planeMat.rows()!=1)
        {
            std::cout << "Please check groundplane file exist : " << groundplane_file << std::endl;
        }
        Vector4d groundplane = planeMat.row(0).head(4);
        if(groundplane.norm() > 0){
            std::cout << "Load groundplane succeeds : " << groundplane.transpose() << std::endl;
            mpOptimizer->SetGroundPlane(groundplane);
            mpTracker->SetGroundPlaneMannually(groundplane);
        }
        else 
            std::cout << "Load groundplane fail : " << groundplane.transpose() << std::endl;

    }

    void run()
    {
        if(!mbInit){
            Save();
            mbInit = true;
        }
        else
            Load();

        if(miType == 0)
            // 求解数据关联, 完成图优化, baseline
            mpOptimizer->GlobalObjectGraphOptimizationWithPDAPointModel(mvpFrames, mpMap);
        else if(miType == 1)
        {
            // mpTracker->NonparamOptimization();        // Optimize with probabilistic data association
            mpOptimizer->GlobalObjectGraphOptimizationWithPDA(mvpFrames, mpMap, mCalib, miRows, miCols);
        }
    }

    int setType(int type)
    {
        miType = type;
        std::cout << "Change Type to " << miType << std::endl;
    }

    void SaveTrajectory(const string& path)
    {
        SaveTrajectoryTUM(path, mvpFrames);
    }

    void VisualizeEllipsoidsInObjects(Objects& objs, const Vector3d& color)
    {
        int obj_num = objs.size();
        for( int i=0;i<obj_num;i++)
        {
            ORB_SLAM2::Object& obj = objs[i];
            g2o::ellipsoid* pE = obj.pEllipsoid;
            if(pE==NULL) continue;
            g2o::ellipsoid* pEV = new g2o::ellipsoid(*pE);
            pEV->setColor(color);
            int num_measurements = obj.measurementIDs.size();
            pE->prob = MIN(1.0, num_measurements / 10.0);

            mpMap->addEllipsoidVisual(pEV);
        }
    }

    void FilterBoundary(Measurements& mms, int rows, int cols, int pixel_thresh)
    {
        // 处理 mms 中所有bbox
        int border_num = 0;
        int mms_with_border = 0;
        for(int measure_id = 0; measure_id < mms.size(); measure_id++)
        {
            Measurement& mm = mms[measure_id];
            Vector4d& measure = mm.ob_2d.bbox;
        

            Vector4d measure_calibrated(-1,-1,-1,-1);
            Vector4d measure_uncalibrated(-1,-1,-1,-1);

            int correct_num = 0;
            if(  measure[0]>pixel_thresh && measure[0]<cols-1-pixel_thresh )
            {
                measure_calibrated[0] = measure[0];
                correct_num++;
            }
            if(  measure[2]>pixel_thresh && measure[2]<cols-1-pixel_thresh )
            {
                measure_calibrated[2] = measure[2];
                correct_num++;
            }
            if(  measure[1]>pixel_thresh && measure[1]<rows-1-pixel_thresh )
            {
                measure_calibrated[1] = measure[1];
                correct_num++;
            }
            if(  measure[3]>pixel_thresh && measure[3]<rows-1-pixel_thresh )
            {
                measure_calibrated[3] = measure[3];
                correct_num++;
            }

            // 修改为calibrated的部分, 边界部分将保持 -1
            measure = measure_calibrated;

            int measure_border_num = 4-correct_num;
            if(measure_border_num > 0){
                mms_with_border++;
                border_num += measure_border_num;
            }
        }

        std::cout << "Boundary Filter Report: " << std::endl;
        std::cout << " - [Config] Pixels : " << pixel_thresh << std::endl;
        std::cout << " - Measurements : " << mms_with_border << " / " << mms.size() << std::endl;
        std::cout << " - Borders : " << border_num << " / " << mms.size()*4 << std::endl;
        return;
    }

    /*
    *   作为 Baseline 之一，调用 Core 内部已有的函数组成 QuadricSLAM 功能.
    *   数据关联来源: 尽量使用 EllipsoidSLAM 的关联结果( 存储于 objs, mms 中 )
    * 
    * input: calib / rows/ cols.
    */ 
    void QuadricSLAM()
    {
        if(!mbQuadricSLAM)
        {
            std::cout << "Please initialize QuadricSLAM using InitQuadricSLAM() First." << std::endl;
            return;
        }
        std::cout << "Begin QuadricSLAM..." << std::endl;

        Load(); // 先恢复轨迹
        // 获得 measurements, objs.
        Objects objs; Measurements mms;
        mpOptimizer->GetOptimizedResult(objs, mms);

        // 调用 Initializer 获得初始化物体
        Initializer* pIniter = new Initializer(miRows, miCols);
        
        //std::vector<g2o::ellipsoid>..
        int obj_num = objs.size();
        int obj_num_success = 0;

        double config_prob_thresh = Config::ReadValue<double>("Dynamic.Optimizer.EllipsoidProbThresh"); // 对应概率 0.5
        std::cout << " ******* QuadricSLAM CONFIG PROB_THRESH : " << config_prob_thresh << std::endl;
        // std::cout << "******* QuadricSLAM CONFIG MINIM_OBSERVATION : " << config_minimum_observation << "********" << std::endl;
        for(int i=0;i<obj_num;i++){
            ORB_SLAM2::Object& obj = objs[i];
            auto ids = obj.measurementIDs;
            int num_measurements = ids.size();
            if(obj.pEllipsoid->prob<config_prob_thresh){
                obj.pEllipsoid = NULL;  // 同样也要认为该物体初始化失败.
                continue;   // 至少3个观测.
            }
            Observations obs;
            for(int n=0;n<ids.size();n++){
                int measure_id = ids[n];
                Measurement& mm = mms[measure_id];
                obs.push_back(&mm.ob_2d);
            }
            g2o::ellipsoid e_svd = pIniter->initializeQuadric(obs, mCalib);  // 位于世界坐标系下

            if(pIniter->getInitializeResult())
            {
                // 存储该物体?
                e_svd.miInstanceID = obj.pEllipsoid->miInstanceID;
                e_svd.miLabel = obj.pEllipsoid->miLabel;
                e_svd.prob = MIN(1.0, num_measurements / 10.0);   // 作为可视化过滤.
                e_svd.setColor(obj.pEllipsoid->getColor(), 0.4);
                obj.pEllipsoid = new g2o::ellipsoid(e_svd);

                // mpMap->addEllipsoidVisual(obj.pEllipsoid);
                obj_num_success++;
                
            }
            else 
                obj.pEllipsoid = NULL;  // 否则认为该物体初始化失败.
        }

        // 可视化这一批椭球体.  
        std::cout << " Sucess init ellipsoids : " << obj_num_success << " / " << obj_num << "( " << (float)obj_num_success/obj_num << " )" << std::endl;

        // 临时存储物体!
        SaveObjects(dataset_path_savedir+"./objects_svd.txt", objs);

        mpSLAM->getMapDrawer()->SetTransformTge(g2o::SE3Quat()); // 取消变换

        // svd 可视化; 红色.
        VisualizeEllipsoidsInObjects(objs, Vector3d(0,0.7,0));
        // 构造图优化并且优化!
        // objs 存储了初始化物体, 并且该结构还存储了所有观测. pFrames 中包含了所有物体的边. 整个一起做个大优化.
        Trajectory camTraj;

        // ************ 实验开关: 过滤边缘约束 *****************
        bool bFilterBoundary = true;
        if(bFilterBoundary)
            FilterBoundary(mms, miRows, miCols, 30);
        // **************************************************
        mpOptimizer->OptimizeUsingQuadricSLAM(mvpFrames, mms, objs, camTraj, mCalib, miRows, miCols);

        // 可视化优化之后的物体! 并且连接优化前和优化后的物体. 蓝色可视化
        VisualizeEllipsoidsInObjects(objs, Vector3d(0,1.0,0));
        
        SaveObjects(dataset_path_savedir+"./objects_quadricslam.txt", objs);

        MatrixXd estMat = TrajetoryToMat(camTraj);
        saveMatToFile(estMat, (dataset_path_savedir+"./traj_quadricslam.txt").c_str());

        // 评估QuadricSLAM!
        if(mbInitEvo){
            MatrixXd estObjMat = GetObjects(objs);
            this->evo(mmRefObjMat, estObjMat, mmRefTrajMat, estMat, mmOdomTrajMat, true);
            std::cout << " ************* QuadricSLAM Evaluation ************ " << std::endl;
        }
    }

    void InitQuadricSLAM()
    {
        mbQuadricSLAM=true;
        return;
    }

    MatrixXd GetObjects(Objects& objs)
    {
        MatrixXd objMat; objMat.resize(0, 11);
        for(auto& obj : objs)
        {
            g2o::ellipsoid* e = obj.pEllipsoid;
            if(e==NULL) continue;
            Vector9d vec = e->toMinimalVector();

            VectorXd vec_instance(11);
            vec_instance << e->miInstanceID, vec, e->miLabel;
            addVecToMatirx(objMat, vec_instance);
        }
        return objMat;
    }

    void SaveObjects(const string& path, Objects& objs)
    {
        MatrixXd objMat = GetObjects(objs);

        saveMatToFile(objMat, path.c_str());
        std::cout << "Save objects to " << path << std::endl;
        return;
    }

    bool ok()
    {
        return mvpFrames.size() > 0;
    }

    // update 6-27 : 考虑KeyFrames与所有Frame的轨迹精度存在变化.
    EvoResult evo(MatrixXd &refObjMat, MatrixXd &estObjMat, MatrixXd &gtTrajMat, MatrixXd &estTrajMat, MatrixXd &odomTrajMat, bool bVisualize)
    {
        // debug 输出各mat
        std::cout << "*************8 EVO INPUT ************** " << std::endl;
        std::cout << "refobjMat : " << std::endl << refObjMat.topRows(5) << std::endl;
        std::cout << "estObjMat : " << std::endl << estObjMat.topRows(5) << std::endl;
        std::cout << "gtTrajMat : " << std::endl << gtTrajMat.topRows(5) << std::endl;
        std::cout << "estTrajMat : " << std::endl << estTrajMat.topRows(5) << std::endl;
        std::cout << "odomTrajMat : " << std::endl << odomTrajMat.topRows(5) << std::endl;


        EvoResult result;

        // 计算odom rmse
        Trajectory gtTrajInOdom, odomTrajSelected; g2o::SE3Quat Tgo;
        alignTrajectory(odomTrajMat, gtTrajMat, Tgo, gtTrajInOdom, odomTrajSelected);
        // Trajectory odomTraj = MatToTrajectory(odomTrajMat);
        double rmse_odom = CalculateRMSE(odomTrajSelected, gtTrajInOdom);

        // 由于Keyframes只是部分Frames, 根据时间戳找出这部分
        // 该函数用est的timestamp去寻找gt的timestamp, 并且分别输出
        MatrixXd estTrajMatKeyframes, odomTrajMatKeyFrames;
        // MatrixXd odomTrajSelectedMat = TrajetoryToMat(odomTrajSelected);
        GenerateSelectedGtMatAndEstMat(estTrajMat, odomTrajMat, estTrajMatKeyframes, odomTrajMatKeyFrames);
        // Trajectory odomTrajSelectedKeyFramesTraj = MatToTrajectory(odomTrajSelectedKeyFrames);
        Trajectory gtTrajInOdomKey, odomTrajKeyFrames;
        g2o::SE3Quat Tgo_key;
        alignTrajectory(odomTrajMatKeyFrames, gtTrajMat, Tgo_key, gtTrajInOdomKey, odomTrajKeyFrames);
        double rmse_odom_keyframes = CalculateRMSE(odomTrajKeyFrames, gtTrajInOdomKey);

        // 计算est rmse 及其对比.
        Trajectory gtTrajInEst, estTrajSelected; g2o::SE3Quat Tge;
        alignTrajectory(estTrajMat, gtTrajMat, Tge, gtTrajInEst, estTrajSelected);
        Trajectory estTraj = MatToTrajectory(estTrajMat);
        double rmse = CalculateRMSE(estTrajSelected, gtTrajInEst);

        double percent_rmse = (rmse-rmse_odom_keyframes)/rmse_odom_keyframes * 100.0; 
        char char_percent_rmse[100];
        sprintf(char_percent_rmse, "%.2f", percent_rmse);
        std::cout << " [ Odom RMSE: " << rmse_odom_keyframes << " m ]" << std::endl;
        std::cout << " [ Est RMSE: " << rmse << "(" << char_percent_rmse << "%)" << " m ]" << std::endl;

        // EVO 完整评估输出
        
        if(mbInitEvo){
            evo::StaticResult output_staticResult; std::map<int, evo::COMPARE_RESULT> output_results;

            ORB_SLAM2::Map* pMapForVisual = NULL;
            if(bVisualize)
                pMapForVisual = mpMap;

            bool bEvoResult = evo::Evaluate(refObjMat, estObjMat, gtTrajMat, estTrajMat, output_staticResult, output_results, pMapForVisual, false);
            if(bEvoResult){
                evo::OutputResult(std::cout, output_staticResult, output_results, refObjMat);
            }
            else 
            {
                std::cerr << "Fail to evaluate objects." << std::endl;
            }
        }

        std::cout << rmse << "(" << char_percent_rmse << "%)" << std::endl;
        std::cout << " ================ " << std::endl;

        std::cout << "- RMSE: " << rmse << "(" << char_percent_rmse << "%)" << std::endl;

        std::cout << std::endl;
        std::cout << "- rmse_odom_key: " << rmse_odom_keyframes << std::endl;
        std::cout << "- rmse_odom: " << rmse_odom << std::endl;

        // 输出一下 odomTrajMat 看是否是正常的.
        // std::cout << "odomTrajMat : " << std::endl << odomTrajMat.topRows(5) << std::endl;
        // std::cout << "gtTrajMat : " << std::endl << gtTrajMat.topRows(5) << std::endl;

        result.estTraj = estTraj;
        result.gtTrajInEst = gtTrajInEst;
        result.Tge = Tge;
        return result;
    }

    void InitEvaluation(MatrixXd& refTrajMat, MatrixXd& refObjMat, MatrixXd& odomTrajMat)
    {
        mmRefObjMat = refObjMat;
        mmRefTrajMat = refTrajMat;
        mmOdomTrajMat = odomTrajMat;
        mbInitEvo = true;
    }

private:
    void Save()
    {
        mpTracker->Save(mvpFrames);
    }

    void Load()
    {
        mpTracker->Load(mvpFrames);
    }

private:
    ORB_SLAM2::Optimizer* mpOptimizer;
    ORB_SLAM2::System* mpSLAM;
    ORB_SLAM2::Map* mpMap;
    ORB_SLAM2::Tracking* mpTracker;

    std::vector<Frame*> mvpFrames;

    int miType;
    bool mbInit;

    int miRows, miCols;
    bool mbQuadricSLAM;
    Matrix3d mCalib;

    MatrixXd mmRefObjMat, mmRefTrajMat, mmOdomTrajMat;
    bool mbInitEvo;
};

Trajectory TransformTrajectory(Trajectory& gtTraj, const g2o::SE3Quat& trans)
{
    Trajectory trajOut;
    int num = gtTraj.size();
    for( int i=0;i<num;i++)
    {
        auto pSE3In = gtTraj[i];
        ORB_SLAM2::SE3QuatWithStamp* pSE3 = new ORB_SLAM2::SE3QuatWithStamp;
        pSE3->timestamp = pSE3In->timestamp;
        pSE3->pose = trans * pSE3In->pose;
        trajOut.push_back(pSE3);
    }
    return trajOut;
}


int main(int argc, char* argv[]) {
    if( argc != 3 && argc !=4 && argc !=5 && argc !=6 )
    {
        std::cout << "usage: " << argv[0] << " path_to_settings type(0:NP, 1:Ours, 2:QuadricSLAM) evo_visual(1:open) debug_dir(0:default 1:./detection_3d_output) reject_parital(0:no 1: yes)" << std::endl;
        return 1;
    }

    // ===== cout 重定向开始 =====
    // std::cout << "Attention: all outputs have been redirected to ./log.txt" << std::endl;
    // static std::ofstream g_log("log.txt");
    // std::cout.rdbuf(g_log.rdbuf());
    // std::cout.rdbuf(g_log.rdbuf());
    // ===== cout 重定向结束 =====

    string strSettingPath = string(argv[1]);
    System* mpSystem = new System(strSettingPath);
    
    // 装载系统
    int sys_type_input = stoi(string(argv[2]));
    
    int sys_type;
    string sys_type_str;
    if(sys_type_input==1){
        sys_type_str = "OURS";
        sys_type = 1;
    }
    else if(sys_type_input==0)
    {
        sys_type_str = "BASELINE[NP]";
        sys_type = 0;
    }
    else if(sys_type_input==2)
    {
        sys_type_str = "BASELINE[QuadricSLAM]";
        sys_type = 1;
    }
    std::cout << "[ **** SYSTYPE : " << sys_type_str << " **** ]" << std::endl;

    bool bOpenEvoVisual = false;
    if(argc>=4 && (*argv[3])=='1')
        bOpenEvoVisual = true;
    
    bool bUseDebugDir = false;
    if(argc>=5 && (*argv[4])=='1')
        bUseDebugDir = true;

    // const string path_odom(argv[1]);
    // const string dir_detectResult(argv[2]);
    // const string path_settings(argv[3]);
        //  path_to_dataset [path_to_map]
    string dataset_path = Config::Get<string>("Dataset.Path.Root");
    string dataset_path_map = Config::Get<string>("Dataset.Path.Map");
    if(dataset_path_map.size()>0)
    {
        if(dataset_path_map[0]=='.')    // 相对路径
            dataset_path_map = dataset_path + "/" + dataset_path_map;
    }
    dataset_path_savedir = dataset_path + "/result/";

    string strDetectionDir = dataset_path + "/bbox/";

    std::cout << "- settings file: " << strSettingPath << std::endl;
    std::cout << "- dataset_path: " << dataset_path << std::endl;
    std::cout << "- strDetectionDir: " << strDetectionDir << std::endl;
    string dataset_type = Config::Get<string>("Dataset.Type");
    std::cout << "- dataset_type : " << dataset_type << std::endl;

    string dir_detectResult = dataset_path + "/detection_3d/";
    if(bUseDebugDir){
        dir_detectResult = dataset_path + "/detection_3d_output/";
        std::cout << " [DEBUG] dir_detect Result : " << dir_detectResult << std::endl;
    }

    string path_odom = dataset_path+"/CameraTrajectoryVO.txt";
    const string path_gt = dataset_path +"/groundtruth.txt";

    bool bUseGt = Config::Get<double>("Dataset.UseGroundtruth") > 0;
    if(bUseGt){
        path_odom = path_gt;
        std::cout << "[ Attention : Use groundtruth !!! ]" << std::endl;
    }

    string path_gt_objects = dataset_path+"/objects_gt_ground.txt"; // 注意 _ground 表示仅包括位于地面的物体!
    MatrixXd refObjMat = readDataFromFile(path_gt_objects.c_str());
    if(refObjMat.rows()==0){    // 如果不存在ground版本则读原始gt
        path_gt_objects = dataset_path+"/objects_gt.txt"; 
        refObjMat = readDataFromFile(path_gt_objects.c_str());
    }

    Runner* pRunner = new Runner(path_odom, dir_detectResult, mpSystem, sys_type);

    if(!pRunner->ok())
    {
        cout << " Reading frames failed. Please check your local files." << endl;
        abort();
    }

    if(argc>=6 && (*argv[5])=='1')
        pRunner->RejectAllPartialObservations();

    pRunner->setType(sys_type);
    pRunner->InitQuadricSLAM();

    // 此处进入交互式优化调试
    if(argc>=3)
    {
        std::cout << "Enter automatic param testing platform!" << std::endl;
        MatrixXd gtMat = readDataFromFile(path_gt.c_str()); // 该方法读取的似乎是 Odometry???
        MatrixXd odomMat = readDataFromFile(path_odom.c_str());
        MatrixXd refObjMat = readDataFromFile(path_gt_objects.c_str());
        if(refObjMat.size() > 0)
            pRunner->InitEvaluation(gtMat, refObjMat, odomMat);
        else 
            std::cout << "Please check your evo file : " << path_gt_objects << std::endl;

        double last_debug_odom_weight = 0, debug_odom_weight = 0.1;
        double last_debug_plane_dis_sigma = 0, debug_plane_dis_sigma = 0.1;
        double last_debug_plane_weight = 0, debug_plane_weight = 1;
        double last_debug_3DEllipsoidScale = 0, debug_3DEllipsoidScale = 1;
        double last_debug_OptimizeRelationPlane = 0, debug_OptimizeRelationPlane = 1;
        // 添加开关 : QuadricSLAM模式
        bool mode_quadricslam = false; bool last_mode_quadricslam = false;

        // 先刷新一回
        // 此处需要多线程等待初始化完毕
        std::cout << "Wait for param to init in Viewer." << std::endl;
        while(!mpSystem->getViewer()->isParamInit())
        {
            // 等待参数初始化
            usleep(30*1000); // wait 30 ms
        }
        debug_odom_weight = Config::ReadValue<double>("DEBUG.ODOM.WEIGHT");
        debug_plane_dis_sigma = Config::ReadValue<double>("DataAssociation.PlaneError.DisSigma");
        debug_plane_weight = Config::ReadValue<double>("DEBUG.PLANE.WEIGHT");
        debug_3DEllipsoidScale = Config::ReadValue<double>("Optimizer.Edges.3DEllipsoid.Scale");
        debug_OptimizeRelationPlane = Config::ReadValue<double>("Dynamic.Optimizer.OptimizeRelationPlane");
        mode_quadricslam = Config::ReadValue<double>("DEBUG.MODE.QUADRICSLAM") > 0.5;

        std::cout << "Wait for param changing ... " << std::endl;
        while(debug_odom_weight > 0)
        {
            bool flag_changed = std::abs(debug_odom_weight - last_debug_odom_weight)>0.01;
            flag_changed |= std::abs(last_debug_plane_dis_sigma - debug_plane_dis_sigma)>0.01;
            flag_changed |= std::abs(last_debug_plane_weight - debug_plane_weight)>0.01;            
            flag_changed |= std::abs(last_debug_3DEllipsoidScale - debug_3DEllipsoidScale)>10e-7;            
            flag_changed |= std::abs(last_debug_OptimizeRelationPlane - debug_OptimizeRelationPlane)>0.01;            
            
            bool quadricslam_flag_changed = mode_quadricslam ^ last_mode_quadricslam;   // 亦或关系  
            if( flag_changed ){
                pRunner->run();
                last_debug_odom_weight = debug_odom_weight;
                last_debug_plane_dis_sigma = debug_plane_dis_sigma;
                last_debug_plane_weight = debug_plane_weight;
                last_debug_3DEllipsoidScale = debug_3DEllipsoidScale;            
                last_debug_OptimizeRelationPlane = debug_OptimizeRelationPlane; 

                // 获得当前轨迹.
                Trajectory estTraj = mpSystem->getMap()->getTrajectoryWithName("OptimizedTrajectory");

                // 与真实轨迹评估输出结果.
                MatrixXd estMat = TrajetoryToMat(estTraj);

                // 注意此处需要先保存, 否则 evo 过程将gtobjects也写入到了 ellipsoidsvisual 中.
                // save objects 
                string exp_type = sys_type==1?"OURS":"NP";
                string output_path(dataset_path_savedir+"./objects.txt."+exp_type);
                mpSystem->SaveObjectsToFile(output_path);
                // pRunner->SaveTrajectory("./traj.txt");
                saveMatToFile(estMat, (dataset_path_savedir+"./traj.txt."+exp_type).c_str());

                
                // 封装评估本次结果
                MatrixXd estObjMat = mpSystem->GetObjects();
                EvoResult result = pRunner->evo(refObjMat, estObjMat, gtMat, estMat, odomMat, bOpenEvoVisual);
                std::cout << " ************* Exp Evaluation ************ " << std::endl;

                std::cout << "End of this test." << std::endl;

                // 读取gt点云并可视化.
                std::cout << "Load background pointcloud from " << dataset_path_map << "..." << std::endl;
                mpSystem->getTracker()->LoadPointcloud(dataset_path_map, "background_world");
                std::cout << "ok." << std::endl;

                // 为MapDrawer添加底层变换
                Trajectory gtTraj = MatToTrajectory(gtMat);
                if(!bOpenEvoVisual){
                    Trajectory gtTrajInEst = TransformTrajectory(gtTraj, result.Tge.inverse());
                    mpSystem->getMap()->addOneTrajectory(gtTrajInEst, "AlignedGroundtruth");
                    // mpSystem->getMap()->addOneTrajectory(result.gtTrajInEst, "AlignedGroundtruth");
                    mpSystem->getMapDrawer()->SetTransformTge(result.Tge);
                }
                else
                {
                    // update : 在世界系下显示gt
                    mpSystem->getMap()->addOneTrajectory(gtTraj, "AlignedGroundtruth");
                }


                // 该部分处理 QuadricSLAM!!
                if(sys_type_input==2)
                {
                    pRunner->QuadricSLAM();
                }
            }

            // 此时直接可视化结果, 若退出将保存该结果.
            debug_odom_weight = Config::ReadValue<double>("DEBUG.ODOM.WEIGHT");
            debug_plane_dis_sigma = Config::ReadValue<double>("DataAssociation.PlaneError.DisSigma");
            debug_plane_weight = Config::ReadValue<double>("DEBUG.PLANE.WEIGHT");
            debug_3DEllipsoidScale = Config::ReadValue<double>("Optimizer.Edges.3DEllipsoid.Scale");
            debug_OptimizeRelationPlane = Config::ReadValue<double>("Dynamic.Optimizer.OptimizeRelationPlane");

            mode_quadricslam = Config::ReadValue<double>("DEBUG.MODE.QUADRICSLAM") > 0.5;
        }
    }

    std::cout << "Everything done." << std::endl;

    while(1);

    return 1;

}