// Update 3-3 
// 该文件从test_textureAreaFrames.cpp 升级，每帧将自动做单帧检测

// 该文件读取粗略物体结果，并测试旋转角优化后的效果
// 之后的计划，自然是既参与前端小优化，又参与后端大优化，核心思想是纹理如何帮助物体估计，这个思路一定是正确的.

// 首先读取封装好的物体效果

#include <iostream>
#include "TextureOptimizer.h"

#include "../../func/func.h"
#include "src/tum_rgbd/io.h"

using namespace std;
using namespace ORB_SLAM2;
using namespace TUMRGBD;

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

int main(int argc,char* argv[])
{
    std::cout << "Nice." << std::endl;

    if( argc != 2 )
    {
        std::cout << "usage: " << argv[0] << " path_to_settings" << std::endl;
        return 1;
    }
    string strSettingPath = string(argv[1]);
    System* mpSystem = new System(strSettingPath);
    auto pMap = mpSystem->getMap();
    auto pDrawer = mpSystem->getFrameDrawer();
    auto pTracker = mpSystem->getTracker();

    string dataset_path = Config::Get<string>("Dataset.Path.Root");
    string dir_detectResult = dataset_path + "/detection_3d_output/";
    string path_odom = dataset_path+"/groundtruth.txt";  // 使用 gt
    string strDetectionDir = dataset_path + "/bbox/";

    Dataset dataset;
    dataset.loadDataset(dataset_path);
    dataset.loadDetectionDir(strDetectionDir);

    auto vecFrames = LoadDataAsFrames(path_odom, dir_detectResult, 1); // 1: Ours

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

    // 开始挨个处理!
    TextureOptimizer opt;
    for(auto pFrame:vecFrames )
    {
        if(!pFrame) continue;
        
        std::cout << "Begin processing ... " << pFrame->timestamp << std::endl;

        // 清除可视化
        pMap->ClearEllipsoidsVisual();

        // 设置当前帧 -> 用于可视化
        pTracker->mCurrFrame = pFrame;
        // 添加mmObservations用于可视化
        // TODO

        // 可视化该帧
        pMap->setCameraState(&pFrame->cam_pose_Twc);
        pMap->addCameraStateToTrajectory(&pFrame->cam_pose_Twc);
        // 读取rgb图像
        // string rgb_name = timestamp_to_image(pFrame->timestamp);
        // string path_to_image = dataset_path + "/rgb/" + rgb_name + ".png";
        // std::cout << "Reading image : " << path_to_image << std::endl;
        // cv::Mat rgb = cv::imread(path_to_image);
        cv:Mat rgb,depth; Eigen::VectorXd pose;
        dataset.findFrameUsingTimestamp(pFrame->timestamp,rgb,depth,pose);
        

        auto pObjs = pFrame->mpLocalObjects;
        cv::Mat rgbShow;
        for(auto pObj:pObjs)
        {
            if(!pObj) continue;
            
            auto bbox = pObj->bbox;
            auto e = *pObj;

            // 绘制bbox
            rgbShow = pDrawer->drawFrameOnImage(rgb);

            // 先可视化再说
            g2o::ellipsoid* pObjw = new g2o::ellipsoid(pObj->transform_from(pFrame->cam_pose_Twc));
            pMap->addEllipsoidVisual(pObjw);

            // 尝试构建小优化，并将结果做比较
            g2o::ellipsoid* pObjw_rotOpt = new g2o::ellipsoid(opt.optimizeWithTexture(*pObj, bbox, pTracker->mCalib, rgb));
            rgbShow = opt.VisualizeResultOnImage(rgb, *pObj, bbox, pTracker->mCalib);
            cv::imshow("rgbShow", rgbShow);

            // 获得Result, 绘制曲线图
            auto result = opt.GetResult();
            cv::Mat histMat = opt.drawXYPlot(result.thetaValueMap, 0, M_PI);
            // 绘制gt
            // TOBEDONE
            cv::imshow("plot hist", histMat);

            // 进入等待，开始运行动态椭球体测试!
            std::cout << "Dynamic Mode, (Please Adjust PLANE.WEIGHT), Press n to continue ... " << std::endl;
            std::map<double,double,std::less<double>> thetaValueMapDynamic;

            double theta_last = -1;
            while(1)
            {
                char key = cv::waitKey(30);
                // std::cout << "key : " << key << std::endl;
                if(key == 'n') break;

                double theta = Config::ReadValue<double>("DEBUG.PLANE.WEIGHT") / 100.0 * M_PI;
                if(theta_last==theta) continue;

                // rotate ellipsoid 
                std::cout << "Theta:" << theta << std::endl;
                g2o::ellipsoid eRot = e.rotate_ellipsoid(theta);
                EllipsoidTex eUpdate(eRot);
                // std::cout << "Theta : " << theta << std::endl;

                // update result
                rgbShow = pDrawer->drawFrameOnImage(rgb);
                opt.optimizeWithTexture(eRot, bbox, pTracker->mCalib, rgb);
                rgbShow = opt.VisualizeResultOnImage(rgbShow, eRot, bbox, pTracker->mCalib);
                cv::imshow("rgbShow", rgbShow);

                // 输出该角度的cost! 添加到已有的map中去
                auto result = opt.GetResult();
                auto thetaValueMapNew = result.thetaValueMap;
                for(auto pair : thetaValueMapNew)
                {
                    thetaValueMapDynamic.insert(std::make_pair(pair.first+theta, pair.second));
                }

                // 一起绘制hist
                cv::Mat histMatDynamic = opt.drawXYPlot(thetaValueMapDynamic, 0, M_PI);
                // 高亮标记当前rot所在的点。
                histMatDynamic = opt.highlightThetaOnPlot(histMatDynamic, theta, 0, M_PI, 0, 255);
                cv::imshow("plot hist dynamic", histMatDynamic);

                // 保存
                ofstream fs("./thetaValueMapDynamic.txt");
                fs << histToTxt(thetaValueMapDynamic) << std::endl;
                fs.close();

                // 可视化旋转了之后的椭球体
                pMap->ClearEllipsoidsVisual();
                pMap->addEllipsoidVisual(pObjw);

                // 添加并可视化旋转之后的椭球体
                g2o::ellipsoid eRotWorld = eRot.transform_from(pFrame->cam_pose_Twc);
                eRotWorld.setColor(Vector3d(1,0,0));
                pMap->addEllipsoidVisual(&eRotWorld);        // TODO: 转换到世界坐标系！！！
                
                theta_last = theta; // 为了循环
            }
            std::cout << "Pressing Next Obj in this frame ... " << std::endl;
        }

        // 处理完一帧
        std::cout << "End Processing." << std::endl;
        if(!rgb.empty())
            cv::imshow("rgb", rgb);
        
        std::cout << "Next frame ... " << std::endl;
        // cv::waitKey();
    }

    return 0;
}