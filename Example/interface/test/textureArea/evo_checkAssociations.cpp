// Update: 2021-3-4
// 该文件用来读取多帧数据，然后构造优化函数完成优化 <想起了一篇类似文章，可以回头看看人家搞了什么>

// 该文件读取粗略物体结果，并测试旋转角优化后的效果
// 之后的计划，自然是既参与前端小优化，又参与后端大优化，核心思想是纹理如何帮助物体估计，这个思路一定是正确的.

// 首先读取封装好的物体效果

#include <iostream>
#include "TextureOptimizer.h"

#include "../../func/func.h"
#include "src/tum_rgbd/io.h"

#include "include/core/Initializer.h"
#include "Optimizer.h"

#include "include/core/PriorInfer.h"

#include "TestFunctions.h"

#include "src/evo/evo.h"

using namespace std;
using namespace ORB_SLAM2;
using namespace TUMRGBD;

void SaveEllipsoidsToFile(const std::map<int, g2o::ellipsoid>& mapInsEllipsoid, const std::string& name)
{
    ofstream out;
    out.open(name);
    for(auto& pair:mapInsEllipsoid)
    {
        int instance = pair.first;
        const g2o::ellipsoid& e = pair.second;
        out << instance << " " << e.toMinimalVector().transpose() << std::endl;
    }
    out.close();
    std::cout << "Save objects to " << name << std::endl;
    return;
}

void ReplaceInitEllipsoid(const g2o::ellipsoid& e, Objects& objs)
{
    // 注意只替换第一个
    if(objs.size() == 1)
    {
        *(objs[0].pEllipsoid) = e;
    }
    return;

}

void ExperiementQuadricSLAM(std::vector<FrameData>& frameDatas,  Matrix3d& calib, int rows, int cols,
    g2o::ellipsoid& e_svd, g2o::ellipsoid& e_quadricslam)
{
    // ******** STEP 1 : Initialize *********
    // 基于frameData做初始化
    g2o::ellipsoid e = InitializeQuadrics(frameDatas, calib, rows, cols, 0);

    // 对frameDatas执行优化过程
    std::vector<Frame *> pFrames = GenerateFrames(frameDatas);
    Measurements mms = GenerateMeasurements(frameDatas, pFrames);
    Objects objs = GenerateObjects(e, mms);

    // QuadricSLAM 全角度优化
    g2o::ellipsoid e_opt = OptimizeFrameDatas(e, pFrames, mms, objs, calib, rows, cols, OPTIMIZE_TYPE_QUADRICSLAM);

    e_svd = e;
    e_quadricslam = e_opt;

    return;
}

void ExperiementStoSLAM(std::vector<FrameData>& frameDatas,  Matrix3d& calib, int rows, int cols,
            g2o::ellipsoid& e_init_, g2o::ellipsoid& e_opt_, g2o::ellipsoid& e_opt_ground_, g2o::ellipsoid& e_opt_ground_pri_, g2o::ellipsoid& e_opt_texture_,
            std::vector<Frame *>& pFrames_, Measurements& mms_, Objects& objs_)
{
    // ******** STEP 1 : Initialize *********
    // 基于frameData做初始化
    g2o::ellipsoid e = InitializeQuadrics(frameDatas, calib, rows, cols, 1);

    // 对frameDatas执行优化过程
    std::vector<Frame *> pFrames = GenerateFrames(frameDatas);
    Measurements mms = GenerateMeasurements(frameDatas, pFrames);
    Objects objs = GenerateObjects(e, mms);

    // ******** STEP 2 : Optimize *********
    // XYZABCYaw版本的优化
    g2o::ellipsoid e_opt = OptimizeFrameDatas(e, pFrames, mms, objs, calib, rows, cols, OPTIMIZE_TYPE_STANDARD);

    // ******** STEP 3 : Optimize with Ground *********
    g2o::ellipsoid e_opt_ground = OptimizeFrameDatas(e, pFrames, mms, objs, calib, rows, cols, OPTIMIZE_TYPE_GROUND);

    // ******** STEP 4 : Optimize with Ground&Pri *********
    g2o::ellipsoid e_opt_ground_pri = OptimizeFrameDatas(e, pFrames, mms, objs, calib, rows, cols, OPTIMIZE_TYPE_GROUND | OPTIMIZE_TYPE_PRI);

    // ******** STEP 5 : Optimize with Ground&Pri&Texture *********
    // 纹理优化比较特殊，要在综合优化好的基础上进行
    ReplaceInitEllipsoid(e_opt_ground_pri, objs);

    // 以Texutre进一步优化椭球体, 每次计算一步!
    g2o::ellipsoid e_opt_texture = OptimizeFrameDatas(e_opt_ground_pri, pFrames, mms, objs, calib, rows, cols, 
            OPTIMIZE_TYPE_GROUND | OPTIMIZE_TYPE_PRI | OPTIMIZE_TYPE_TEXTURE_DT);

    e_init_ = e;
    e_opt_ = e_opt;
    e_opt_ground_ = e_opt_ground;
    e_opt_ground_pri_ = e_opt_ground_pri;
    e_opt_texture_ = e_opt_texture;

    pFrames_ = pFrames;
    mms_ = mms;
    objs_ = objs;

    return;
}

void VisualizeGradientMat(Measurements& mms, Objects& objs, const Matrix3d& calib, int rows, int cols)
{
    if(objs.size() == 0) return;

    ORB_SLAM2::Object& obj = objs[0];
    g2o::ellipsoid e = *(obj.pEllipsoid);

    // TODO: 尝试输出所有的集合, 然后看看是不是凸的

    TextureOptimizer opt;
    for(auto mea_id:obj.measurementIDs)
    {
        Measurement& mm = mms[mea_id];
        Vector4d& bbox = mm.ob_2d.bbox;
        Frame* pFrame = mm.ob_2d.pFrame;
        cv::Mat rgb = pFrame->rgb_img;
        // opt.optimizeWithTexture(e, bbox, calib, rgb);
        // 重新计算keyPoints

        g2o::ellipsoid e_local = e.transform_from(pFrame->cam_pose_Tcw);
        // 输出该角度的cost! 添加到已有的map中去
        auto thetaValueMapNew = opt.GetThetaValueMap(e_local, calib, mm.ob_2d.keyPoints, mm.ob_2d.dtMat);

        // 一起绘制hist
        cv::Mat histMatDynamic = opt.drawXYPlot(thetaValueMapNew, 0, M_PI);
        // 高亮标记当前rot所在的点。
        double current_theta = 0;
        histMatDynamic = opt.highlightThetaOnPlot(histMatDynamic, current_theta, 0, M_PI, 0, 255);
        cv::imshow("plot hist dynamic", histMatDynamic);

        break; // Only one!
    }
    
    return;

}

void VisualizeExperienmentResult(ORB_SLAM2::System* pSLAM, std::vector<FrameData>& frameDatas,  Matrix3d& calib, int rows, int cols,
            std::vector<Frame *>& pFrames, Measurements& mms, Objects& objs,
            g2o::ellipsoid& e_svd, g2o::ellipsoid& e_quadricslam, 
            g2o::ellipsoid& e_init, g2o::ellipsoid& e_opt, g2o::ellipsoid& e_opt_ground, g2o::ellipsoid& e_opt_ground_pri, g2o::ellipsoid& e_opt_texture,
            bool DebugMode = true)
{
    // 挨个对各个椭球体加颜色和注释.
    ORB_SLAM2::Map* pMap = pSLAM->getMap();
    // 清空之前的, 只留下 纹理比较
    pMap->ClearEllipsoidsVisual();

    // --------------
    std::cout << "Begin Visualizing QuadricSLAM" << std::endl;
    e_svd.setColor(Vector3d(1,0,0));
    pMap->addEllipsoidVisual(&e_svd);
    std::cout << "Ellipsoid: SVD. Enter to continue." << std::endl;

    e_quadricslam.setColor(Vector3d(0.5,0,0));
    pMap->addEllipsoidVisual(&e_quadricslam);
    std::cout << "Finish QuadricSLAM Optimizing ... Enter to continue. [y] to automatically.." << std::endl;
    if(DebugMode) {
        char key = waitKey();
        if(key=='y') DebugMode = false;
    }

    // --------------
    std::cout << "Begin Visualizing StoSLAM" << std::endl;
    pMap->ClearEllipsoidsVisual();

    e_init.setColor(Vector3d(1,0,0));
    pMap->addEllipsoidVisual(&e_init);
    std::cout << "Ellipsoid: Init with GroundPlane&Pri. Enter to continue." << std::endl;

    // 可视化优化的椭球体
    e_opt.setColor(Vector3d(0,0.3,0));
    pMap->addEllipsoidVisual(&e_opt);

    std::cout << "Finish optimizing standard... Enter to continue." << std::endl;
    if(DebugMode) waitKey();

    e_opt_ground.setColor(Vector3d(0,0.6,0));
    pMap->addEllipsoidVisual(&e_opt_ground);

    std::cout << "Finish optimizing with Ground... Enter to continue." << std::endl;
    if(DebugMode) waitKey();

    // 清空之前的, 只留下 纹理比较
    pMap->ClearEllipsoidsVisual();

    e_opt_ground_pri.setColor(Vector3d(0,1,0));
    pMap->addEllipsoidVisual(&e_opt_ground_pri);

    std::cout << "Finish optimizing with Ground&Pri... Enter to continue." << std::endl;
    if(DebugMode) waitKey();

    // 可视化纹理优化的椭球体
    e_opt_texture.setColor(Vector3d(0,0,1));
    pMap->addEllipsoidVisual(&e_opt_texture);

    // 再次显示初始化时的物体
    pMap->addEllipsoidVisual(&e_init);

    // ---------------

    // 判断旋转角度的变化
    double diff_yaw = evo::getYawError(e_opt_texture, e_opt_ground_pri);
    std::cout << "Diff Yaw Angle : " << diff_yaw << " deg" << std::endl;

    // 可视化结果
    if(DebugMode) {
        VisualizeTextureResults(mms, objs, calib, rows, cols); // 在 RGB 图像上可视化纹理点

        // 可视化范围梯度图
        VisualizeGradientMat(mms, objs, calib, rows, cols);

        // 可视化投影
        std::vector<g2o::ellipsoid*> vEs; 
        vEs.push_back(&e_init);
        // vEs.push_back(&e_opt);
        vEs.push_back(&e_opt_texture);
        VisualizeFrameData(frameDatas, pSLAM, calib, vEs);

        std::cout << "Please check gradients and texture mat..." << std::endl;

        waitKey();
    }
}

int main(int argc,char* argv[])
{
    std::cout << "You are what you think you are." << std::endl;

    if( argc != 2 )
    {
        std::cout << "usage: " << argv[0] << " path_to_settings" << std::endl;
        return 1;
    }

    // // TEMPTEST
    // Eigen::AngleAxisd x_90(M_PI/2, Vector3d(1,0,0));
    // std::cout << x_90.toRotationMatrix() << std::endl;
    // return 0;

    // Init System
    Dataset dataset;
    System* pSLAM = initSystem(argc, argv, dataset);
    ORB_SLAM2::Map* pMap = pSLAM->getMap();
    
    // PARAM
    Matrix3d calib = pSLAM->getTracker()->mCalib;
    int rows = pSLAM->getTracker()->mRows, cols = pSLAM->getTracker()->mCols;

    // 接下来可以使用的变量: Dataset

    // 此处应该设计一个接口，用来选择“哪些帧的第几个观测”是我要用来做测试的
    // frames,  <Timestamp, obj_id>
    // vecFrames 和 pair
    std::string dir_gtassociation = dataset.GetDatasetDir() + "/gt_associate.txt";
    std::cout << "Load multiple objects associations from : " << dir_gtassociation << std::endl;
    std::map<int,std::map<double, int>> objObs = LoadFrameObjSettingsFromFile(dir_gtassociation);
    std::cout << "Total Objects Num : " << objObs.size() << std::endl;

    // *********** 系统初始化 *********** 
    // 可视化地平面
    cv::Mat paramMat = Config::GetMat("Plane.Groundplane.param");
    if(paramMat.empty()){
        std::cerr << "Please check param Plane.Groundplane.param!" << std::endl;
    }
    Eigen::Vector4d plane_param;
    cv::cv2eigen(paramMat, plane_param);
    g2o::plane pl_ground(plane_param);
    pMap->addPlane(&pl_ground);
    //  ***********  *********** 

    bool DebugMode = true;  // 开启调试模式，每帧将输出更多信息
    int id = 0;
    for(auto& pair : objObs){
        // 清除上一帧的可视化结果
        cv::destroyAllWindows();
        pMap->ClearCameraTrajectory();

        // id++;
        // if(id==1) continue;

        int instance = pair.first;
        auto& mapFrameObj = pair.second;
        std::cout << "Gt " << instance << ", Observation num: " << mapFrameObj.size() << std::endl;

        // 获得上述数据: frame_pose, bbox, image
        std::vector<FrameData> frameDatas = GenerateFrameDatas(mapFrameObj, dataset); // 还要想办法更新图像!
        OutputObjectObservation(mapFrameObj);

        for(int i=0;i<frameDatas.size();i++)
        {
            FrameData& fd = frameDatas[i];
            VisualizeOneFrameData(fd, pSLAM);
            std::cout << "Visualizing frame data ... " << std::endl;
            std::cout << std::endl << std::endl << std::endl;

            // DO SOMETHING!
            

            std::cout << std::endl << "- End of this observation : " << fd.timestamp << std::endl;
            cv::waitKey();
        }

        std::cout << std::endl << "End of this object : " << instance << std::endl;
        cv::waitKey();
    }
    

    return 0;
}