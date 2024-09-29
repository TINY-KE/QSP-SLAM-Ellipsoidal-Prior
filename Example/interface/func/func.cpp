#include "func.h"

/************From rgbd_odom.cpp************/
MatrixXd TrajetoryToMat(Trajectory& estTraj)
{
    int num = estTraj.size();

    MatrixXd mat; mat.resize(0, 8);
    for(int i= 0 ; i< num;i++)
    {
        VectorXd poseTime; poseTime.resize(8);
        poseTime << estTraj[i]->timestamp, estTraj[i]->pose.toVector();
        addVecToMatirx(mat, poseTime);
    }

    return mat;
}

Trajectory MatToTrajectory(MatrixXd& mat)
{
    int pose_num = mat.rows();
    Trajectory traj;
    for(int i=0;i<pose_num;i++)
    {
        VectorXd pose_vec = mat.row(i);
        SE3QuatWithStamp* pPose = new SE3QuatWithStamp;
        pPose->timestamp = pose_vec[0];
        pPose->pose.fromVector(pose_vec.tail(7));
        traj.push_back(pPose);
    }
    return traj;
}

void GenerateSelectedGtMatAndEstMat(MatrixXd &estMat, MatrixXd &gtMat, MatrixXd &estMatSelected_, MatrixXd& gtMatSelected_)
{
    MatrixXd gtMatSelected; gtMatSelected.resize(0, gtMat.cols());
    MatrixXd estMatSelected; estMatSelected.resize(0, estMat.cols());
    int estNum = estMat.rows();
    int gtNum = gtMat.rows();

    VectorXd gtTimeStampList = gtMat.col(0);
    for(int i=0;i<estNum;i++)
    {
        VectorXd estPose = estMat.row(i);
        double timestamp = estPose[0];

        // 挨个按距离阈值寻找. ( 适合 rgb,depth 时间戳相同的情况. )
        // VectorXd pose_gt;
        // for( int n=0;n<gtNum;n++)
        // {
        //     VectorXd gtPose = gtMat.row(n);
        //     double timestampGT = gtPose[0];
        //     if( std::abs(timestamp - timestampGT) < 0.001 )
        //     {
        //         bFindGT= true;
        //         pose_gt = gtPose;
        //         break;
        //     }
        // }

        // 寻找最小值然后确认阈值的方法
	    MatrixXd::Index minRow, minCol;
        auto gtDiffArray = (gtTimeStampList.array() - timestamp).abs();
        double diff_min = gtDiffArray.minCoeff(&minRow, &minCol);
        if(diff_min < 0.01) // 10ms 内间隔认为有效.
        {
            VectorXd pose_gt = gtMat.row(minRow); // id 
            addVecToMatirx(gtMatSelected, pose_gt);
            addVecToMatirx(estMatSelected, estPose);
        }
    }

    estMatSelected_ = estMatSelected;
    gtMatSelected_ = gtMatSelected;

    // std::cout << " estMatSelected_ Size : " << estMatSelected_.rows() << " x " << estMatSelected_.cols() << std::endl;
    // std::cout << " gtMatSelected_ Size : " << gtMatSelected_.rows() << " x " << gtMatSelected_.cols() << std::endl;

    return;
}

// 估计一个刚体变换使得误差最小. 并输出匹配的变换.
void alignTrajectory(MatrixXd& estMat, MatrixXd& gtMat, g2o::SE3Quat& transform, Trajectory &alignedGt, Trajectory &estTrajSelected)
{
    // 应该是一个闭式解.
    // 生成两个匹配的 Mat; 用 est 去寻找 gt ( 即认为 est <= gt )

    MatrixXd gtMatSelected, estMatSelected;
    GenerateSelectedGtMatAndEstMat(estMat, gtMat, estMatSelected, gtMatSelected);
    MatrixXd gtPointMat = gtMatSelected.block(0,1,gtMatSelected.rows(),3);
    MatrixXd estPointMat = estMatSelected.block(0,1,estMatSelected.rows(),3);

    // std::cout << "gtMatSelected : " << std::endl << gtMatSelected.topRows(5) << std::endl;
    // std::cout << "estMatSelected : " << std::endl << estMatSelected.topRows(5) << std::endl;

    // std::cout << "gtPointMat : " << std::endl << gtPointMat.topRows(5) << std::endl;
    // std::cout << "estPointMat : " << std::endl << estPointMat.topRows(5) << std::endl;

    // 开始求解 : 要求输入的点是一列一个.
    MatrixXd result = Eigen::umeyama(estPointMat.transpose(), gtPointMat.transpose(), false);
    // std::cout << " ----- Umeyama result ---- " << std::endl;
    // std::cout << result << std::endl;

    g2o::SE3Quat trans(result.topLeftCorner(3,3), result.topRightCorner(3,1));

    // 将gt点变换到 est 坐标系下.
    Trajectory tGt;
    int num = gtMatSelected.rows(); // 这里只可视化所有对应帧得了.
    for(int i=0;i<num;i++)
    {
        VectorXd gtPose = gtMatSelected.row(i);
        SE3QuatWithStamp* pGtSE3T = new SE3QuatWithStamp();
        pGtSE3T->pose.fromVector(gtPose.tail(7));
        pGtSE3T->timestamp = gtPose[0];

        // 应用变换
        pGtSE3T->pose = trans.inverse() * pGtSE3T->pose;

        tGt.push_back(pGtSE3T);
    }
    alignedGt = tGt;
    transform = trans;
    estTrajSelected = MatToTrajectory(estMatSelected);

    // 返回RMSE误差
    return;
}

double CalculateRMSE(Trajectory& estTraj, Trajectory& gtTrajInEst)
{
    int num = MIN(estTraj.size(), gtTrajInEst.size());

    double error_sum = 0;
    for( int i=0;i<num;i++)
    {
        Vector3d pos1 = estTraj[i]->pose.translation();
        Vector3d pos2 = gtTrajInEst[i]->pose.translation();

        double error = (pos1-pos2).norm();
        error_sum += error * error;
    }

    double rmse = std::sqrt(error_sum / num);

    return rmse;
}

/************From rgbd_odom.cpp************/


string FindFileUsingTimestamp(std::vector<string>& mvDetFileNames, double timestamp)
{
    for(auto iter = mvDetFileNames.begin(); iter!=mvDetFileNames.end(); iter++)
    {
        string file_name = *iter;
        string bareName = splitFileNameFromFullDir(file_name, true);

        double timestamp_file;
        try {
            timestamp_file = stod(bareName);
        }
        catch(std::out_of_range) {
            std::cout << "out of range : " << bareName << "; " << file_name << std::endl;
            // abort();
            continue;
        }

        if( std::abs(timestamp - timestamp_file) < 0.01)    // 10ms
        {
            return file_name;
        }

    }
    return string("");
}

// 2020-6-1 Update: 可自动判断是否包含 ConstrainPlanes

// detMat格式:
// 每行一个检测
// xxxxx
// xxxxx
//   * 对于每行: [ ------- ] Ellipsoid信息(加VecPlanes)  [ ----- ] bbox 信息(4)
void LoadObjectsObservationsToFrame(MatrixXd& detMat, Frame* pFrame, int iType)
{
    // 开始添加到 localEllipsoid
    int num_objs = detMat.rows();
    for(int n=0;n<num_objs;n++)
    {
        VectorXd detVec = detMat.row(n);    // x,y,z,qx,qy,qz,qw,a,b,c,label,prob
        int label = round(detVec[10]);

        g2o::ellipsoid* pLocalEllipsoid = new g2o::ellipsoid;
        pLocalEllipsoid->LoadFromVector(detVec);
        pFrame->mpLocalObjects.push_back(pLocalEllipsoid);

        if(iType == 0)  // 变成点
        {
            double size_point = 0.1;
            pLocalEllipsoid->scale = Vector3d(size_point,size_point,size_point);
        }
        else if(iType == 2){ // 存储的物体在世界坐标系，所以先转换到局部坐标系来
            // (*pLocalEllipsoid) = pLocalEllipsoid->transform_from(pFrame->cam_pose_Tcw);
            pFrame->cam_pose_Tcw = g2o::SE3Quat();
            pFrame->cam_pose_Twc = g2o::SE3Quat();
            std::cout << "DEBUG, Transform to local " << std::endl;
        }

        // 注意需要存到结构体里面
        Observation ob_2d;
        ob_2d.label = label;
        if(detVec.size() > 15)
            ob_2d.bbox = Vector4d(detVec[12],detVec[13],detVec[14],detVec[15]);     // 此处!  想办法读取一下. 因为QuadricSLAM需要数据
        if(detVec.size() > 11)
            ob_2d.rate = detVec[11];
        else 
            ob_2d.rate = 1.0;
        ob_2d.pFrame = pFrame;

        Observation3D ob_3d;
        ob_3d.pFrame = pFrame;
        ob_3d.pObj = pLocalEllipsoid;

        Measurement m;
        m.measure_id = n;
        m.instance_id = -1; // not associated
        m.ob_2d = ob_2d;
        m.ob_3d = ob_3d;
        pFrame->meas.push_back(m);
    }
    return;
}

void LoadRelationsToFrame(const Eigen::MatrixXd& detMatRelation, Frame* pFrame)
{
    int rows = detMatRelation.rows();
    for(int i=0;i<rows;i++)
    {
        VectorXd vec = detMatRelation.row(i);
        Relation rl; 
        bool valid = rl.InitRelation(vec, pFrame);
        if(valid)
            pFrame->relations.push_back(rl);
    }

    return;
}


// update: 6-23 新增读取关系平面
// iType : 0 baseline, 1 ours, 2 世界坐标系
// iJump: 此处保持为1， 否则内部可能跳过 3d detection 的 timestamp
std::vector<Frame*> LoadDataAsFrames(const string& path_odom, const string& dir_detectResult, int iType, int iJump)
{
    std::vector<Frame*> vpFrames;
    MatrixXd odomMat = readDataFromFile(path_odom.c_str());

    // std::cout << "odomMat : " << std::endl << odomMat << std::endl;

    // 获得目录下所有detection.
    // get all the file names under the directory
    std::cout << "Read detection files in : " << dir_detectResult << std::endl;
    std::vector<string> mvDetFileNames;
    GetFileNamesUnderDir(dir_detectResult, mvDetFileNames); // 注意是完整路径
    sortFileNames(mvDetFileNames, mvDetFileNames);
    int miTotalNum = mvDetFileNames.size();

    //Relations
    std::vector<string> mvRelationsFileNames;
    GetFileNamesUnderDir(dir_detectResult+"/relations/", mvRelationsFileNames); // 注意是完整路径
    sortFileNames(mvRelationsFileNames, mvRelationsFileNames);

    int num_valid_objs = 0;
    int num_valid_relations = 0, num_total_relations = 0;

    // 生成 Frame 结构.
    int frame_num = odomMat.rows();
    for( int i=0;i<frame_num;i+=iJump)
    {
        VectorXd odomVec = odomMat.row(i);
        double timestamp = odomVec[0];
        // 由timestamp搜索文件夹读取文件
        string file_name = FindFileUsingTimestamp(mvDetFileNames, timestamp);
        if(file_name.size() < 1) continue;

        //  HACK行为
        // 从file_name中取末尾的名字
        // 加上bbox的路径
        string dir_bbox = dir_detectResult + "../bbox/";
        string timestamp_txt = splitFileNameFromFullDir(file_name);
        string bbox_file_name = dir_bbox + timestamp_txt;
        MatrixXd bboxMat = readDataFromFile(bbox_file_name.c_str());
        
        Frame* pFrame = new Frame(timestamp, odomVec.tail(7), bboxMat, cv::Mat(), cv::Mat(), false);     // no output
        
        MatrixXd detMat = readDataFromFile(file_name.c_str());  // 存储 vLocalObjects
        LoadObjectsObservationsToFrame(detMat, pFrame, iType);
        
        // 读取Relations.
        string file_name_relation = FindFileUsingTimestamp(mvRelationsFileNames, timestamp);
        MatrixXd detMatRelation = readDataFromFile(file_name_relation.c_str());
        LoadRelationsToFrame(detMatRelation, pFrame);
        num_valid_relations += pFrame->relations.size();
        num_total_relations += detMatRelation.rows();

        vpFrames.push_back(pFrame);

        num_valid_objs+=pFrame->meas.size();
    }
    std::cout << "Read objects : " << num_valid_objs << std::endl;
    std::cout << "Read frame : " << vpFrames.size() << std::endl;

    std::cout << "Read Relations : " << num_valid_relations << " / " << num_total_relations << std::endl;
    return vpFrames;
}

void SaveTrajectoryTUM(const string &filename, std::vector<Frame*>& vpKFs)
{
    cout << endl << "[func.cpp] Saving camera trajectory to " << filename << " ..." << endl;

    ofstream f;
    f.open(filename.c_str());
    f << fixed;

    for( int i=0;i<vpKFs.size();i++)        
    {
        Frame* pF = vpKFs[i];
        Eigen::Vector3d trans = pF->cam_pose_Twc.translation();
        Eigen::Quaterniond q = pF->cam_pose_Twc.rotation();
        f << setprecision(6) << pF->timestamp << " " <<  setprecision(9) << trans[0] << " " << trans[1] << " " << trans[2] << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << endl;
    }
    f.close();
    cout << endl << "trajectory saved!" << endl;
}