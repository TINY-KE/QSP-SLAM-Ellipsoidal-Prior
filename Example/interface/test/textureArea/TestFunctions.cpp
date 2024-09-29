#include "TestFunctions.h"

string histToTxt(const std::map<double,double,std::less<double>>& hist)
{
    string out;
    // 第一列： x
    // 第二列： 数据
    for(auto & pair : hist )
    {
        string line;
        line = to_string(pair.first) + "\t" + to_string(pair.second) + "\n";
        out += line;
    }
    return out;
}

void drawEllipseOnImage(const Vector5d& ellipse, cv::Mat& im, const cv::Scalar& color = cv::Scalar(0,0,255))
{
    // std::cout << "Ellipse from circle : " << ellipse.transpose() << std::endl;
    cv::RotatedRect rotbox2(cv::Point2f(ellipse[0],ellipse[1]), cv::Size2f(ellipse[3]*2,ellipse[4]*2), ellipse[2]/M_PI*180);
    try
    {
        cv::ellipse(im, rotbox2, color);
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
    }
    return;
}

MatrixXd GeneratePoseMat(const std::vector<FrameData>& frameDatas)
{
    MatrixXd poseMat;
    for(auto&dd:frameDatas)
    {
        VectorXd vec = dd.cam_pose_Twc.toVector();
        addVecToMatirx(poseMat, vec);
    }
    return poseMat;
}

MatrixXd GenerateDetectionMat(const std::vector<FrameData>& frameDatas)
{
    MatrixXd detectionMat;
    for(auto&dd:frameDatas)
    {
        VectorXd vec = dd.bbox;
        addVecToMatirx(detectionMat, vec);
    }
    return detectionMat;
}

g2o::ellipsoid InitWithSVD(const std::vector<FrameData>& frameDatas, Matrix3d& calib, int rows, int cols)
{
    std::cout << "Init with SVD!" << std::endl;
    MatrixXd pose_mat = GeneratePoseMat(frameDatas);
    MatrixXd detection_mat = GenerateDetectionMat(frameDatas);
    // 开始初始化
    Initializer initer(rows, cols);
    g2o::ellipsoid e = initer.initializeQuadric(pose_mat, detection_mat, calib);
    return e;
}

g2o::ellipsoid InitWithGroundPlanePri(const std::vector<FrameData>& frameDatas, Matrix3d& calib, int rows, int cols)
{
    std::cout << "Init with groundplane pri!" << std::endl;
    g2o::ellipsoid e;
    if(frameDatas.size() == 0 )
    {
        std::cerr << "Please check frameDatas.size!" << std::endl;
        return e;
    }
    FrameData fd = frameDatas[0];

    priorInfer priinfer(rows, cols, calib);
    // Plane.Groundplane.param   -> plane_local
    cv::Mat paramMat = Config::GetMat("Plane.Groundplane.param");
    if(paramMat.empty()){
        std::cerr << "Please check param Plane.Groundplane.param!" << std::endl;
        return e;
    }
    Eigen::Vector4d plane_param;
    cv::cv2eigen(paramMat, plane_param);
    g2o::plane pl_world(plane_param);
    g2o::plane pl_local = pl_world; pl_local.transform(fd.cam_pose_Twc.inverse());
    e = priinfer.GenerateInitGuess(fd.bbox, pl_local.param); // 只需要一帧!

    // 接着是优化
    // 读取pri
    const string priconfig_path = Config::Get<std::string>("Dataset.Path.PriTable");
    PriFactor prifac;
    bool use_input_pri = (priconfig_path.size()>0);

    Pri pri;
    if(use_input_pri){
        prifac.LoadPriConfigurations(priconfig_path);
        pri = prifac.CreatePri(fd.label);
    }
    else
        pri = Pri(1,1);
    
    double weight = Config::ReadValue<double>("SemanticPrior.Weight");
    g2o::ellipsoid e_opt = priinfer.MonocularInfer(e, pri, weight, pl_local);

    e_opt.miLabel = fd.label;

    // 转换到世界系下
    return e_opt.transform_from(fd.cam_pose_Twc);
}

// METHOD,  0 SVD, 1 Plane
g2o::ellipsoid InitializeQuadrics(const std::vector<FrameData>& frameDatas, Matrix3d& calib, int rows, int cols, int METHOD)
{
    std::cout << "Try to initialize quadrics with " << frameDatas.size() << " datas." << std::endl;

    g2o::ellipsoid e;
    if(METHOD==0) // 方式1： 使用 QuadricSLAM SVD 分解初始化椭球体
        e = InitWithSVD(frameDatas, calib, rows, cols); // TODO : 加入 label!
    else if(METHOD==1) // 方法2： 基于地平面初始化椭球体
    {
        e = InitWithGroundPlanePri(frameDatas, calib, rows, cols);
    }

    // 赋予基本参数
    e.prob = 1;
    e.prob_3d = 1;

    return e;
}

std::vector<Frame *> GenerateFrames(const std::vector<FrameData>& frameDatas)
{
    std::vector<Frame *> pFrames;
    for(auto &dd:frameDatas)  
    {
        Eigen::MatrixXd bboxMat;
        bboxMat.resize(1, 7);
        bboxMat.row(0) << 0, dd.bbox.transpose(), 1, 1;
        Frame* pFrame = new Frame(dd.timestamp, dd.cam_pose_Twc.toVector(), MatrixXd(), cv::Mat(), dd.image, false);
        pFrames.push_back(pFrame);
    }
    return pFrames;
}

Measurements GenerateMeasurements(const std::vector<FrameData>& frameDatas, std::vector<Frame *>& pFrames)
{
    Measurements mms;
    int id = 0;
    for(auto &dd:frameDatas)
    {
        Measurement m;
        m.instance_id = 0;
        m.measure_id = id;
        m.ob_2d.bbox = dd.bbox;
        m.ob_2d.instance = 0;
        m.ob_2d.label = 0;
        m.ob_2d.pFrame = pFrames[id];
        m.ob_2d.rate = 1;

        // 为 Texture 准备的初始化有单独的函数额外赋值
        // cv::Mat rgb = dd.image, gray;
        // cvtColor(rgb, gray, CV_BGR2GRAY);
        // m.ob_2d.gray = gray.clone();
        
        // m.ob_3d -> empty
        id++;
        
        mms.push_back(m);
    }
    return mms;
}

Objects GenerateObjects(const g2o::ellipsoid& e, Measurements& mms)
{
    ORB_SLAM2::Object obj;
    obj.instance_id = 0;
    obj.pEllipsoid = new g2o::ellipsoid(e);
    for(auto &m:mms)
        obj.measurementIDs.push_back(m.measure_id);
    
    ORB_SLAM2::Objects objs; objs.push_back(obj);
    return objs;
}

void InitTextureMeasurements(Measurements& mms)
{
    TextureOptimizer topt;
    for(auto &m:mms)
    {
        // bbox
        // rgb图
        cv::Mat gray = m.ob_2d.pFrame->gray_img;
        Vector4d bbox = m.ob_2d.bbox;
        topt.ExtractKeyPointsFromBbox(gray, bbox, m.ob_2d.keyPoints, m.ob_2d.descripts);
        m.ob_2d.gray = gray;  // 一张image 图

        // 生成dtMat
        topt.GenerateDtMatFromGray(gray, m.ob_2d.dtMat, m.ob_2d.edgesMat);
    }
}

void VisualizeTextureResults(Measurements& mms, Objects& objs, const Matrix3d& calib, int rows, int cols)
{
    // 将每个观测都输出出来。看看纹理点的对称情况，以及附近的 极值 点
    int obj_id = 0;
    for(auto &obj : objs)
    {
        g2o::ellipsoid& e = *obj.pEllipsoid;

        int count_ob_id = -1;
        int MAX_SHOW_NUM = 4;
        int id_per_show = obj.measurementIDs.size() / MAX_SHOW_NUM + 1;
        for(auto id : obj.measurementIDs){
            count_ob_id++;
            bool valid_for_show = ((count_ob_id % id_per_show) == 0);
            if(!valid_for_show) continue;

            auto & m = mms[id];
            auto & keyPoints = m.ob_2d.keyPoints;
            auto & keyPointsDiscri = m.ob_2d.descripts;
            cv::Mat rgb = m.ob_2d.pFrame->rgb_img;
            cv::Mat gray = m.ob_2d.pFrame->gray_img;
            Vector4d bbox = m.ob_2d.bbox;

            // 在图像上生成对称点，并且可视化!
            std::vector<cv::KeyPoint> keyPointsFlected;
            g2o::ellipsoid e_local = e.transform_from(m.ob_2d.pFrame->cam_pose_Tcw);
            double pixelDiff_average = g2o::CalculateEllipsoidTextureCost(e_local, keyPoints, keyPointsDiscri, calib, gray, keyPointsFlected);

            // 这个可视化也要与opt无关，仅仅与某个文件有关。需要再细化地提取。
            // 以后写函数一定要一步步分清楚，各个函数的出口入口等都要很清晰，避免后期再做提取
            TextureOptimizer opt;
            TextureOptimizerResult result;
            result.keyPoints = keyPoints;
            result.keyPointsFlected = keyPointsFlected;

            cv::Mat rgb_show = opt.VisualizeResultOnImage(rgb, e_local, bbox, calib, result);

            char str_window[50];
            sprintf(str_window, "Obj %d Measure %d", obj_id, id);

            char str_error[50];
            sprintf(str_error, "%.2f", pixelDiff_average);
            cv::putText(rgb_show, str_error, cv::Point(0,rgb_show.rows), FONT_HERSHEY_COMPLEX, 1, cv::Scalar(0,255,0));
            cv::imshow(str_window, rgb_show);

            // 可视化edges和dtMat
            bool bShowEdgesMat = true;
            if(bShowEdgesMat){
                char str_window_edges[50];
                sprintf(str_window_edges, "Obj %d Measure %d - edges", obj_id, id);
                cv::imshow(str_window_edges, m.ob_2d.edgesMat);

                char str_window_dtMat[50];
                sprintf(str_window_dtMat, "Obj %d Measure %d - dtMat", obj_id, id);
                // 注意dtMat是需要 normalize 的
                cv::Mat dtMatShow; 
                cv::normalize(m.ob_2d.dtMat, dtMatShow, 0, 1., cv::NORM_MINMAX);
                cv::imshow(str_window_dtMat, dtMatShow);
            }
        }
        obj_id ++;
    }
}

void DebugTestingRunningTime(Measurements& mms, Objects& objs, const Matrix3d& calib, int rows, int cols)
{
    int run_num = 500;
    std::cout << "Running time : " << run_num << std::endl;

    // 将每个观测都输出出来。看看纹理点的对称情况，以及附近的 极值 点  
    auto& obj = objs[0];
    g2o::ellipsoid& e = *obj.pEllipsoid;
    int id = obj.measurementIDs[0];

    auto & m = mms[id];
    auto & keyPoints = m.ob_2d.keyPoints;
    auto & keyPointsDiscri = m.ob_2d.descripts;
    cv::Mat rgb = m.ob_2d.pFrame->rgb_img;
    cv::Mat gray = m.ob_2d.pFrame->gray_img;
    Vector4d bbox = m.ob_2d.bbox;

    // 在图像上生成对称点，并且可视化!
    std::vector<cv::KeyPoint> keyPointsFlected;
    g2o::ellipsoid e_local = e.transform_from(m.ob_2d.pFrame->cam_pose_Tcw);

    // 需要对该部分函数做拆分统计
    for(int i=0;i<run_num;i++)
        g2o::CalculateEllipsoidTextureCost(e_local, keyPoints, keyPointsDiscri, calib, gray, keyPointsFlected);
    
    return;
}

std::vector<g2o::ellipsoid*> graphOptimizeWithAllPossiblePriInit(TextureOptimizer& opt, std::vector<Frame *>& pFrames, Measurements& mms, Objects& objs,
    Matrix3d& calib, int rows, int cols)
{
    // 生成多种可能性
    std::cout << " **** OPTIMIZE WITH ALL POSSIBILITIES!" << std::endl;

    g2o::ellipsoid e = *(objs[0].pEllipsoid);

    Vector3d oriScale = e.scale;
    Vector3d initScale = oriScale;

    // 交换产生6个 分别执行monocularInfer，然后取误差最小的一个
    static const int c33_tables[6][3] = {
        {0,1,2},
        {0,2,1},
        {1,0,2},
        {1,2,0},
        {2,0,1},
        {2,1,0}
    };

    std::vector<double> costList; costList.resize(6);
    std::vector<g2o::ellipsoid> e_list; e_list.resize(6);

    auto originPEllipsoid = objs[0].pEllipsoid;
    for(int i=0;i<6;i++)
    {
        g2o::ellipsoid e_0 = e;
        Vector3d switchScale;
        switchScale << initScale[c33_tables[i][0]], initScale[c33_tables[i][1]], initScale[c33_tables[i][2]];
        e_0.scale = switchScale;

        // 替换掉!
        objs[0].pEllipsoid = new g2o::ellipsoid(e_0);

        auto vEllipsoids = opt.graphOptimize(pFrames, mms, objs, calib, rows, cols);
        g2o::ellipsoid e_0_opt = *vEllipsoids[0];
        auto result = opt.GetResult();
        double cost = result.chi2;

        costList[i] = cost;
        e_list[i] = e_0_opt;
        std::cout << "e_0 for " << i << " : " << e_0.scale.transpose() << std::endl;
    }

    // 开始找出最小的cost
    auto iter_min = std::min_element(costList.begin(), costList.end());
    int pos = std::distance(costList.begin(), iter_min);

    bool bDebug = false;
    if(bDebug)
    {
        // Print all cost
        std::cout << " -- Cost \t Ellipsoid " << std::endl;
        for(int i=0;i<6;i++)
        {
            std::cout << " -- " << costList[i] << "\t" << e_list[i].toMinimalVector().transpose() << std::endl;;
        }
        std::cout << "Final Decision: " << pos << std::endl;
    }

    std::vector<g2o::ellipsoid*> pEs;
    pEs.push_back(new g2o::ellipsoid(e_list[pos]));

    return pEs;
}

g2o::ellipsoid OptimizeFrameDatas(const g2o::ellipsoid& e, std::vector<Frame *>& pFrames, Measurements& mms, Objects& objs,
    Matrix3d& calib, int rows, int cols, int type)
{
    std::cout << "Try to optimize quadrics with " << pFrames.size() << " datas." << std::endl;

    // 构建优化函数，最好还是使用已有的。输入e, 约束，输出e_opt

    TextureOptimizer opt;

    cv::Mat paramMat = Config::GetMat("Plane.Groundplane.param");
    if(paramMat.empty()){
        std::cerr << "Please check param Plane.Groundplane.param!" << std::endl;
        std::cerr << "Please check param Plane.Groundplane.param!" << std::endl;
        std::cerr << "Please check param Plane.Groundplane.param!" << std::endl;
    }
    Eigen::Vector4d plane_param;
    cv::cv2eigen(paramMat, plane_param);

    opt.SetGroundPlaneNormal(plane_param);
    opt.SetGroundplane(type & OPTIMIZE_TYPE_GROUND);  
    opt.SetPri(type & OPTIMIZE_TYPE_PRI);

    bool bTexture = type & OPTIMIZE_TYPE_TEXTURE;
    opt.SetTexture(bTexture);

    bool bTextureDT = type & OPTIMIZE_TYPE_TEXTURE_DT;
    opt.SetTextureDT(bTextureDT);

    if(bTexture || bTextureDT)
    {
        InitTextureMeasurements(mms);
        // std::cout << "Wait to optimize with texture ... " << std::endl;
        // cv::waitKey();
    }

    if(bTexture){
        // 添加临时测试，看看需要耗时多少
        std::cout << "TEMPTEST FEATURE TIME ... " << std::endl;
        DebugTestingRunningTime(mms, objs, calib, rows, cols);
    }

    // ************** TEMP DEBUG: 做边的分析测试 *********
    // if(bTextureDT)
    //     opt.SetEdgeAnalysis(true);

    std::vector<g2o::ellipsoid*> vEllipsoids;
    bool bQuadricSLAM = type & OPTIMIZE_TYPE_QUADRICSLAM;
    if(bQuadricSLAM){
        Trajectory traj;
        Optimizer optzer;
        vEllipsoids = optzer.OptimizeUsingQuadricSLAM(pFrames, mms, objs, traj, calib, rows, cols);
    }
    else if(bTextureDT)
    {
        // 切换成离散采样形式! 没必要做数值优化 能到2度的精度已经顶天了.
        vEllipsoids = opt.SampleRotationWithTexture(pFrames, mms, objs, calib, rows, cols);
    }
    else
    { 
        bool bUsePri = type & OPTIMIZE_TYPE_PRI;
        if(bUsePri)
        {
            vEllipsoids = graphOptimizeWithAllPossiblePriInit(opt, pFrames, mms, objs, calib, rows, cols);
        }
        else 
            vEllipsoids = opt.graphOptimize(pFrames, mms, objs, calib, rows, cols);

        // 特殊情况：带先验的优化
        
    }

    // 在此处基于 TextureOptimizer 重新打造!

    if(bTextureDT)
    {
        // VisualizeTextureResults(mms, objs, calib, rows, cols);
    }

    g2o::ellipsoid e_opt;
    if(vEllipsoids.size() > 0)
        e_opt = *vEllipsoids[0];
    else 
        std::cerr << "ERROR!!! No Valid Optimized Ellipsoids." << std::endl;
    return e_opt;

}


g2o::ellipsoid OptimizeFrameDatasTexture(const g2o::ellipsoid& e, std::vector<Frame *>& pFrames, Measurements& mms, Objects& objs,
    Matrix3d& calib, int rows, int cols)
{
    std::cout << "Try to optimize quadrics with " << pFrames.size() << " datas using textures." << std::endl;

    TextureOptimizer topt;
    /*
        该函数将提取Measurement 2d 里存放的数据。因此需要先对其做初始化.
    */
    InitTextureMeasurements(mms);

    int opt_time = 10;
    for(int i=0;i<opt_time;i++){
        topt.graphOptimize(pFrames, mms, objs, calib, rows, cols);
        VisualizeTextureResults(mms, objs, calib, rows, cols);

        std::cout << "Finish optimization " << i << std::endl;
        cv::waitKey();
    }
    
    g2o::ellipsoid e_opt = *objs[0].pEllipsoid;
    // g2o::ellipsoid e_opt;
    return e_opt;

}

System* initSystem(int argc,char* argv[], TUMRGBD::Dataset& dataset)
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

    return mpSystem;
}

std::vector<FrameData> GenerateFrameDatas(const std::map<double, int>& mapFrameObj, TUMRGBD::Dataset& dataset)
{
    std::vector<FrameData> datas;
    datas.reserve(mapFrameObj.size());
    for(auto &pair : mapFrameObj)
    {
        double timestamp = pair.first;
        int detid = pair.second;
        
        FrameData data;
        cv::Mat rgb, depth; Eigen::VectorXd pose;
        int frame_id = dataset.findFrameUsingTimestampGetId(timestamp, rgb, depth, pose);
        if(frame_id >= 0)
        {
            data.timestamp = timestamp;
            data.cam_pose_Twc.fromVector(pose);
            data.image = rgb.clone();

            // 找到bbox的过程
            Eigen::MatrixXd detMat = dataset.getDetectionMat(frame_id);
            if(detMat.rows() <= detid) continue;
            VectorXd det_vec = detMat.row(detid);  // 应该有一个取bbox的过程, 或者说做原始处理.
            int label = round(det_vec(5));
            double measurement_prob = det_vec(6);
            Eigen::Vector4d bbox = Eigen::Vector4d(det_vec(1), det_vec(2), det_vec(3), det_vec(4));
            data.bbox = bbox;
            data.label = label;

            datas.push_back(data);      // TODO: 注意拷贝是否成功! 检查Image. 如果不行，换成指针.
        }        
    }
    return datas;
}

void drawProjectionOnImage(const g2o::SE3Quat& pose_cw, g2o::ellipsoid e_opt, cv::Mat& im, const Matrix3d& calib)
{
    if(e_opt.isColorSet())
    {
        Vector5d ellipse = e_opt.projectOntoImageEllipse(pose_cw, calib);
        Vector3d color = e_opt.getColor() * 255;
        drawEllipseOnImage(ellipse, im, cv::Scalar(color[2], color[1], color[0]));
    }
    return;
}

void VisualizeOneFrameData(FrameData& data, System* pSLAM)
{
    string window_name = to_string(data.timestamp);
    cv::Mat im = data.image.clone();
    // 绘制bbox
    Vector4d rect = data.bbox;
    cv::rectangle(im, cv::Rect(cv::Point(rect[0],rect[1]),cv::Point(rect[2],rect[3])), cv::Scalar(0,0,255), 4);
    std::cout << "Frame " << data.timestamp << " , bbox : " << data.bbox.transpose() << std::endl;

    cv::imshow(window_name, im);
    
    // 在三维空间可视化各个帧的位置
    pSLAM->getMap()->addCameraStateToTrajectory(&data.cam_pose_Twc);
}

void VisualizeFrameData(std::vector<FrameData>& frameDatas, System* pSLAM)
{
    // 可视化各个图
    std::cout << "Total datas: " << frameDatas.size() << std::endl;

    int total_id = -1;
    for(auto &data : frameDatas)
    {
        total_id++;
        if(total_id%5!=0) continue;

        VisualizeOneFrameData(data, pSLAM);

        std::cout << "Data " << data.timestamp << std::endl;
    }

    cv::waitKey(30);
}

void VisualizeFrameData(std::vector<FrameData>& frameDatas, System* pSLAM, const Matrix3d& calib, 
    const std::vector<g2o::ellipsoid*>& vElips)
{
    // 可视化各个图
    std::cout << "Total datas: " << frameDatas.size() << std::endl;
    int total_id = -1;
    for(auto &data : frameDatas)
    {
        total_id++;
        if(total_id%5!=0) continue;
        string window_name = to_string(data.timestamp);
        cv::Mat im = data.image.clone();
        // 绘制bbox
        Vector4d rect = data.bbox;
        cv::rectangle(im, cv::Rect(cv::Point(rect[0],rect[1]),cv::Point(rect[2],rect[3])), cv::Scalar(0,0,255), 4);

        // 投影
        for(auto &pE:vElips){
            if(pE)
                drawProjectionOnImage(data.cam_pose_Twc.inverse(), *pE, im, calib);
        }
        
        cv::imshow(window_name, im);
        
        // 在三维空间可视化各个帧的位置
        pSLAM->getMap()->addCameraStateToTrajectory(&data.cam_pose_Twc);

        // std::cout << "Data " << data.timestamp << std::endl;
    }

    cv::waitKey(30);

}

void OutPutFrameData(const std::vector<FrameData>& frameDatas)
{
    for(auto &data : frameDatas)
    {
        std::cout << "Timestamp:\t" << data.timestamp << std::endl;
        std::cout << "Pose:\t" << data.cam_pose_Twc.toVector().transpose() << std::endl;
        std::cout << "Bbox:\t" << data.bbox.transpose() << std::endl;
        string str_im = string("out_") + to_string(data.timestamp) + ".png";
        // cv::imwrite(str_im, data.image);
        std::cout << "Image:\t" << str_im << std::endl;
    }
}

std::map<double, int> LoadFrameObjSettingsStatic()
{

    // TEST 1 : Chair
    // std::map<double, int> mapFrameObj = {
    //     {421, 0},
    //     {476, 0},
    //     {492, 0}
    // };

    // TEST 2 : Sofa
    // std::map<double, int> mapFrameObj = {
    //     {264, 0},
    //     {407, 0},
    //     {421, 1}
    // };

    // TEST 3 : Chair2
    std::map<double, int> mapFrameObj = {
        {570, 0},
        {492, 1},
        {551, 0},
        {592, 0},
        {608, 0},
    };

    // --------------------

    // TEST : Sofa
    // std::map<double, int> mapFrameObj = {
    //     {275, 0},
    //     {410, 0}
    // };

    return mapFrameObj;

}

// 准备从已有的文件中读取
std::map<int,std::map<double, int>> LoadFrameObjSettingsFromFile(const string& fileName)
{
    std::map<int,std::map<double, int>> objObs;
    ifstream fin(fileName);
    string line;

    getline(fin, line);  
    if(line.size()==0) return objObs;
    int gt_num = stoi(line);
    for(int i=0;i<gt_num;i++)
    {
        getline(fin, line);
        vector<string> s;
        // edit
        // boost::split(s, line, boost::is_any_of( " \t," ), boost::token_compress_on);
        boost::algorithm::split(s, line, boost::is_any_of( " \t," ), boost::token_compress_on);
        if(s.size()==2)
        {
            int gt_id = stoi(s[0]);
            int ob_num = stoi(s[1]);
            for(int n=0;n<ob_num;n++)
            {
                getline(fin, line);
                boost::split( s, line, boost::is_any_of( " \t," ), boost::token_compress_on );
                if(s.size()==2)
                {
                    double timestamp = stod(s[0]);
                    int ob_id = stoi(s[1]);
                    // 保存
                    objObs[gt_id][timestamp]=ob_id;
                }
            }
        }
        
    }

    return objObs;
}

// std::vector<std::map<double, int>> LoadFrameObjSettingsFromFile(const string& dir)
// {

// }

void OutputObjectObservation(const std::map<double, int>& mapFrameObj)
{
    std::cout << "Observation (timestamp->id):" << std::endl;
    for (auto & pair : mapFrameObj)
    {
        std::cout << " * " << pair.first << " -> " << pair.second << std::endl;
    }
}
