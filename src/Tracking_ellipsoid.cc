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

#include "Tracking.h"
#include "ObjectDetection.h"
#include "ORBmatcher.h"
#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>

using namespace std;

namespace ORB_SLAM2 {

    // ZHJD 移植
    Matrix3Xd generateProjectionMatrix(const SE3Quat &campose_cw, const Matrix3d &Kalib) {
        Matrix3Xd identity_lefttop;
        identity_lefttop.resize(3, 4);
        identity_lefttop.col(3) = Vector3d(0, 0, 0);
        identity_lefttop.topLeftCorner<3, 3>() = Matrix3d::Identity(3, 3);

        Matrix3Xd proj_mat = Kalib * identity_lefttop;

        proj_mat = proj_mat * campose_cw.to_homogeneous_matrix();

        return proj_mat;
    }

    MatrixXd fromDetectionsToLines(Vector4d &detections) {
        bool flag_openFilter = false; // filter those lines lying on the image boundary

        double x1 = detections(0);
        double y1 = detections(1);
        double x2 = detections(2);
        double y2 = detections(3);

        Vector3d line1(1, 0, -x1);
        Vector3d line2(0, 1, -y1);
        Vector3d line3(1, 0, -x2);
        Vector3d line4(0, 1, -y2);

        // those lying on the image boundary have been marked -1
        MatrixXd line_selected(3, 0);
        MatrixXd line_selected_none(3, 0);

        int config_border_pixel = 10;
        int miImageCols = Config::Get<int>("Camera.width");
        int miImageRows = Config::Get<int>("Camera.height");
        if (!flag_openFilter || (x1 > config_border_pixel && x1 < miImageCols - config_border_pixel)) {
            line_selected.conservativeResize(3, line_selected.cols() + 1);
            line_selected.col(line_selected.cols() - 1) = line1;
        }
        if (!flag_openFilter || (y1 > config_border_pixel && y1 < miImageRows - config_border_pixel)) {
            line_selected.conservativeResize(3, line_selected.cols() + 1);
            line_selected.col(line_selected.cols() - 1) = line2;
        }
        if (!flag_openFilter || (x2 > config_border_pixel && x2 < miImageCols - config_border_pixel)) {
            line_selected.conservativeResize(3, line_selected.cols() + 1);
            line_selected.col(line_selected.cols() - 1) = line3;
        }
        if (!flag_openFilter || (y2 > config_border_pixel && y2 < miImageRows - config_border_pixel)) {
            line_selected.conservativeResize(3, line_selected.cols() + 1);
            line_selected.col(line_selected.cols() - 1) = line4;
        }

        return line_selected;
    }

    MatrixXd GenerateBboxPlanes(g2o::SE3Quat &campose_wc, Eigen::Vector4d &bbox, Matrix3d &calib) {
        MatrixXd planes_all(4, 0);
        // std::cout << " [debug] calib : \n " << calib << std::endl;
        // get projection matrix
        MatrixXd P = generateProjectionMatrix(campose_wc.inverse(), calib);

        MatrixXd lines = fromDetectionsToLines(bbox);
        MatrixXd planes = P.transpose() * lines;

        // add to matrix
        for (int m = 0; m < planes.cols(); m++) {
            planes_all.conservativeResize(planes_all.rows(), planes_all.cols() + 1);
            planes_all.col(planes_all.cols() - 1) = planes.col(m);
        }

        return planes_all;
    }

    // 新版本则基于bbox生成，不再与RGBD版本有任何关联
    // replace_detection: 是否将推测物体放入 pFrame 中生效。
    void Tracking::InferObjectsWithSemanticPrior(Frame* pFrame, bool use_input_pri = true, bool replace_detection = false)
    {
        // 要求有地平面估计再启动
        if(miGroundPlaneState != 2)
        {
            std::cout << "Close Infering, as the groundplane is not set." << std::endl;
            return;
        }

        Pri pri = Pri(1,1);
        pri.print();

        // ********* 测试1： 所有先验都是 1:1:1 *********
        // // 读取 pri;  调试模式从全局 Config 中读取
        double weight = Config::ReadValue<double>("SemanticPrior.Weight");
        std::cout << "weight:"<<weight << std::endl;
        std::cout << "Begin infering ... " << std::endl;

        // 对于帧内每个物体，做推断，并可视化新的物体
        auto& meas = pFrame->meas;
        int meas_num = meas.size();

        if(replace_detection) {
            pFrame->mpLocalObjects.clear();
            pFrame->mpLocalObjects.resize(meas_num);
        }
        for(int i=0;i<meas_num;i++)
        {
            Measurement& m = meas[i];
            Vector4d bbox = m.ob_2d.bbox;
            // 检测bbox是否正确
            // int x1 = (int)(bbox(0)), y1 = (int)(bbox(1)), \
            // x2 = (int)(bbox(2)), y2 = (int)(bbox(3));
            // std::cout<< " [zhjd-debug] bbox: " << "x1:"<<x1 
            // << ", y1:" << y1 << ", x2:" << x2 << ", y2:" << y2
            // << std::endl;

            // Check : 确保该物体类型是在地面之上的
            // if(!CheckLabelOnGround(m.ob_2d.label)) continue;

            // Check : 该 bbox 不在边缘
            // bool is_border = calibrateMeasurement(bbox, mRows, mCols, Config::Get<int>("Measurement.Border.Pixels"), Config::Get<int>("Measurement.LengthLimit.Pixels"));
            // if(is_border) continue;

            // 生成Pri
            Pri pri = Pri(1,1);

            std::cout << "Pri for label : " << m.ob_2d.label << std::endl;
            pri.print();

            // RGB_D + Prior
            g2o::plane ground_pl_local = mGroundPlane;   //世界坐标系下的
            ground_pl_local.transform(pFrame->cam_pose_Tcw);  // 相机坐标系下的
            std:;cout<<"[InferObjectsWithSemanticPrior] 0 地平面： world参数:"<< mGroundPlane.param.transpose()
            << " local参数:" << ground_pl_local.param.transpose() <<std::endl;
            
            priorInfer pi(mRows, mCols, mCalib);

            // *********************************
            // 生成一个新的 Initguess
            // *********************************
            std::cout<<"[InferObjectsWithSemanticPrior] 1 准备 初始化一个椭球体"<<std::endl;
            std::cout << "LastCost: " << pi.GetLastCost() << std::endl;
            
            // pi.GenerateInitGuess(bbox, ground_pl_local.param);
            // g2o::ellipsoid e_init_guess = pi.GenerateInitGuess(bbox, ground_pl_local.param);
            int debug_init_guess = Config::ReadValue<double>("OptimizeEllipsoidWithMultiPlanes.debug_init_guess");
            if(debug_init_guess==0){
                g2o::ellipsoid* e_init_guess = new g2o::ellipsoid(); // 获得估计出椭球体的 rpy; 只是将 x,y,z,a,b,c 都设置为0.
                Eigen::Matrix<double, 10, 1>  e_param;
                e_param <<     0, 0, 0,   // x y z
                                0, 0, 0, 0,  // qx qy qz qw
                                0.5, 0.5, 0.5   // length_a  length_b  length_c
                            ;
                std::cout << "e_param: 11:" << e_param.transpose() << std::endl;
                e_init_guess->fromVector(e_param);
                e_init_guess->prob = 1.0;
                e_init_guess->setColor(Vector3d(1,0,0.1));
                std::cout<<"[InferObjectsWithSemanticPrior] 2 结束 初始化一个椭球体,Scale: "<< e_init_guess->scale.transpose() <<std::endl;

                mpMap->addEllipsoidVisual(e_init_guess); // 可视化
            }
            else if(debug_init_guess==1){
                g2o::ellipsoid e_init_guess = pi.GenerateInitGuess(bbox, ground_pl_local.param);
                g2o::ellipsoid* e_init_guess_1 = new g2o::ellipsoid(e_init_guess.transform_from(pFrame->cam_pose_Twc)); 
                e_init_guess_1->prob = 1.0;
                e_init_guess_1->setColor(Vector3d(0.2,0.2,0.2));
                std::cout<<"[InferObjectsWithSemanticPrior] 2 结束 初始化一个椭球体,Scale: "<< e_init_guess_1->scale.transpose() <<std::endl;
                mpMap->ClearEllipsoidsVisual();
                mpMap->addEllipsoidVisual(e_init_guess_1); // 可视化
            
                // ------------------------

                bool bUsePriInit = false;   // 未开发完成的功能
                g2o::ellipsoid e_infer_mono_guess;
                
                e_infer_mono_guess = pi.MonocularInfer(e_init_guess, pri, weight, ground_pl_local);
                // 设置椭球体label, prob
                e_infer_mono_guess.miLabel = m.ob_2d.label;
                e_infer_mono_guess.prob = m.ob_2d.rate; // 暂时设置为 bbox 检测的概率吧
                e_infer_mono_guess.bbox = m.ob_2d.bbox;
                e_infer_mono_guess.prob_3d =  1.0; // 暂定!
                g2o::ellipsoid* pEInfer_mono_guess = new g2o::ellipsoid(e_infer_mono_guess.transform_from(pFrame->cam_pose_Twc));

                Vector3d color_rgb(144,238,144); color_rgb/=255.0;
                if(!use_input_pri) color_rgb = Vector3d(1,0,0); // 默认版本为红色
                pEInfer_mono_guess->setColor(color_rgb);
                // mpMap->addEllipsoidObservation(pEInfer_mono_guess); // 可视化
                mpMap->addEllipsoidVisual(pEInfer_mono_guess); // 可视化

                std::cout << " Before Monocular Infer: " << e_init_guess.toMinimalVector().transpose() << std::endl;
                std::cout << " After Monocular Infer: " << e_infer_mono_guess.toMinimalVector().transpose() << std::endl;

                // DEBUG可视化： 显示一下初始状态的椭球体
                // g2o::ellipsoid* pE_init_guess = new g2o::ellipsoid(e_init_guess.transform_from(pFrame->cam_pose_Twc));
                // pE_init_guess->prob = 1.0;
                // pE_init_guess->setColor(Vector3d(0.1,0,0.1));
                // mpMap->addEllipsoidVisual(pE_init_guess); // 可视化

                // --------- 将结果放到frame中存储
                if(replace_detection)
                    pFrame->mpLocalObjects[i] = new g2o::ellipsoid(e_infer_mono_guess);
            
                // DEBUGING: 调试为何Z轴会发生变化， 先输出在局部坐标系下的两个rotMat
                std::cout << "InitGuess RotMat in Camera: " << std::endl << e_init_guess.pose.rotation().toRotationMatrix() << std::endl;
                std::cout << "Infered RotMat in Camera: " << std::endl << e_infer_mono_guess.pose.rotation().toRotationMatrix() << std::endl;
                std::cout << "GroundPlaneNorma in Camera: " << std::endl << ground_pl_local.normal().head(3).normalized() << std::endl;

                // 可视化bbox的约束平面
                mpMap->clearPlanes();
                // mpTracker->SetGroundPlaneMannually(Vector4d(0,  0,   1,  0));
                mpMap->addPlane(&mGroundPlane);
                VisualizeConstrainPlanes(e_infer_mono_guess, pFrame->cam_pose_Twc, mpMap); // 中点定在全局坐标系
            }
            else if(debug_init_guess==2){
                g2o::SE3Quat local_wc = g2o::SE3Quat();
                MatrixXd mPlanesParamLocal_Col = GenerateBboxPlanes(local_wc, bbox, mCalib);  // attention: store as 列
                MatrixXd mPlanesParamLocal = mPlanesParamLocal_Col.transpose(); 
                std::vector<g2o::plane*> planes_world;
                int num = mPlanesParamLocal.rows();
                // mpMap->clearPlanes();
                int gap = Config::ReadValue<double>("OptimizeEllipsoidWithMultiPlanes.gap");

                if(mCurrentFrame.mnId%gap == 0)
                    for( int i=0;i<num;i++)
                    {
                        VectorXd vec = mPlanesParamLocal.row(i);
                        g2o::plane* plane_new= new g2o::plane(vec.head(4));
                        plane_new->transform(pFrame->cam_pose_Twc);  // 转换到世界坐标系
                        // g2o::ConstrainPlane* pcpl = new g2o::ConstrainPlane(ppl);
                        planes_world.push_back(plane_new);
                        // mpMap->addPlane(plane_new);
                    }

                if (planes_world.size()>=8)
                    continue;
                
                g2o::ellipsoid e_init_guess = pi.GenerateInitGuess(bbox, ground_pl_local.param);
                // if(mpMap->mspEllipsoidsVisual.empty())
                {
                    g2o::ellipsoid* e_init_guess_1 = new g2o::ellipsoid(e_init_guess.transform_from(pFrame->cam_pose_Twc)); 
                    e_init_guess_1->prob = 1.0;
                    e_init_guess_1->setColor(Vector3d(0.2,0.2,0.2));
                    std::cout<<"[InferObjectsWithSemanticPrior] 2 结束 初始化一个椭球体,Scale: "<< e_init_guess_1->scale.transpose() <<std::endl;
                    mpMap->ClearEllipsoidsVisual(); 
                    mpMap->addEllipsoidVisual(e_init_guess_1); // 可视化
                }
            
                // ------------------------
                // 将planes_world转为planes_in_current_camera
                std::vector<g2o::plane> planes_in_current_camera;
                planes_in_current_camera.push_back(ground_pl_local);
                for(auto p:planes_world){
                    g2o::plane* plane_new = new g2o::plane(p->param);
                    plane_new->transform(pFrame->cam_pose_Tcw);    //转到当前相机坐标系
                    planes_in_current_camera.push_back(*plane_new);
                }

                g2o::ellipsoid e_infer_mono_guess;
                e_infer_mono_guess = pi.optimizeEllipsoidWithMultiPlanes(e_init_guess, planes_in_current_camera, pri);   // std::vector<g2o::plane> &planes
                // 设置椭球体label, prob
                e_infer_mono_guess.miLabel = m.ob_2d.label;
                e_infer_mono_guess.prob = m.ob_2d.rate; // 暂时设置为 bbox 检测的概率吧
                e_infer_mono_guess.bbox = m.ob_2d.bbox;
                e_infer_mono_guess.prob_3d =  1.0; // 暂定!
                g2o::ellipsoid* pEInfer_mono_guess = new g2o::ellipsoid(e_infer_mono_guess.transform_from(pFrame->cam_pose_Twc));

                Vector3d color_rgb(144,238,144); color_rgb/=255.0;
                if(!use_input_pri) color_rgb = Vector3d(1,0,0); // 默认版本为红色
                pEInfer_mono_guess->setColor(color_rgb);
                // mpMap->addEllipsoidObservation(pEInfer_mono_guess); // 可视化
                mpMap->addEllipsoidVisual(pEInfer_mono_guess); // 可视化

                std::cout << " Before Monocular Infer: " << e_init_guess.toMinimalVector().transpose() << std::endl;
                std::cout << " After Monocular Infer: " << e_infer_mono_guess.toMinimalVector().transpose() << std::endl;

                // DEBUG可视化： 显示一下初始状态的椭球体
                // g2o::ellipsoid* pE_init_guess = new g2o::ellipsoid(e_init_guess.transform_from(pFrame->cam_pose_Twc));
                // pE_init_guess->prob = 1.0;
                // pE_init_guess->setColor(Vector3d(0.1,0,0.1));
                // mpMap->addEllipsoidVisual(pE_init_guess); // 可视化

                // --------- 将结果放到frame中存储
                if(replace_detection)
                    pFrame->mpLocalObjects[i] = new g2o::ellipsoid(e_infer_mono_guess);
            
                // DEBUGING: 调试为何Z轴会发生变化， 先输出在局部坐标系下的两个rotMat
                std::cout << "InitGuess RotMat in Camera: " << std::endl << e_init_guess.pose.rotation().toRotationMatrix() << std::endl;
                std::cout << "Infered RotMat in Camera: " << std::endl << e_infer_mono_guess.pose.rotation().toRotationMatrix() << std::endl;
                std::cout << "GroundPlaneNorma in Camera: " << std::endl << ground_pl_local.normal().head(3).normalized() << std::endl;

                // 可视化bbox的约束平面
                // mpMap->clearPlanes();
                // mpTracker->SetGroundPlaneMannually(Vector4d(0,  0,   1,  0));
                // mpMap->addPlane(&mGroundPlane);
                VisualizeConstrainPlanes(e_infer_mono_guess, pFrame->cam_pose_Twc, mpMap); // 中点定在全局坐标系
            }
            else if(debug_init_guess==3){
                
                g2o::ellipsoid e_init_guess = pi.GenerateInitGuess(bbox, ground_pl_local.param);
                g2o::ellipsoid* e_init_guess_1 = new g2o::ellipsoid(e_init_guess.transform_from(pFrame->cam_pose_Twc)); 
                e_init_guess_1->prob = 1.0;
                e_init_guess_1->setColor(Vector3d(0.2,0.2,0.2));
                std::cout<<"[InferObjectsWithSemanticPrior] 2 结束 初始化一个椭球体,Scale: "<< e_init_guess_1->scale.transpose() <<std::endl;
                mpMap->ClearEllipsoidsVisual(); 
                mpMap->addEllipsoidVisual(e_init_guess_1); // 可视化
            
                std::vector<g2o::plane*> planes_world;

                // 1.生成 bbox平面  in world
                g2o::SE3Quat local_wc = g2o::SE3Quat();
                MatrixXd mPlanesParamLocal_Col = GenerateBboxPlanes(local_wc, bbox, mCalib);  // attention: store as 列
                MatrixXd mPlanesParamLocal = mPlanesParamLocal_Col.transpose(); 
                
                int num = mPlanesParamLocal.rows();
                
                for( int i=0;i<num;i++)
                    {
                        VectorXd vec = mPlanesParamLocal.row(i);
                        g2o::plane* plane_new= new g2o::plane(vec.head(4));
                        plane_new->transform(pFrame->cam_pose_Twc);  // 转换到世界坐标系
                        // g2o::ConstrainPlane* pcpl = new g2o::ConstrainPlane(ppl);
                        planes_world.push_back(plane_new);
                        // mpMap->addPlane(plane_new);
                    }
                
                // 2.生成 远近平面 in world
                double dis_thresh_near = Config::ReadValue<double>("OptimizeEllipsoidWithMultiPlanes.dis_thresh_near");
                double dis_thresh_far = Config::ReadValue<double>("OptimizeEllipsoidWithMultiPlanes.dis_thresh_far");
                Eigen::Vector3d farest, nearest;
                Eigen::Vector3d normal_far, normal_near;
                double dis_far=0, dis_near=1000;
                cv::Mat Ow = pFrame->GetCameraCenter();
                std::cout<<"[生成远近平面] 1 begin"<<std::endl;
                for(auto pMP:m.mvpObjectPoints){
                    if (!pMP)
                        continue;
                    if (pMP->isBad())
                        continue;
                    if (pMP->isOutlier())
                        continue;

                    auto p_pose = pMP->GetWorldPos();
                    // 计算p和相机的距离
                    cv::Mat normal = p_pose - Ow;
                    // 计算normal的长度
                    double dis = cv::norm(normal);
                    if(dis>dis_thresh_far)
                        continue;
                    if(dis>dis_far){
                        dis_far = dis;
                        farest[0] = p_pose.at<float>(0, 0);  
                        farest[1] = p_pose.at<float>(1, 0);  
                        farest[2] = p_pose.at<float>(2, 0);  
                        normal_far[0] = -1*normal.at<float>(0, 0);
                        normal_far[1] = -1*normal.at<float>(1, 0);
                        normal_far[2] = -1*normal.at<float>(2, 0);
                    }
                    if(dis<dis_thresh_near)
                        continue;
                    if(dis<dis_near){
                        dis_near = dis;
                        nearest[0] = p_pose.at<float>(0, 0);  
                        nearest[1] = p_pose.at<float>(1, 0);  
                        nearest[2] = p_pose.at<float>(2, 0);  
                        normal_near[0] = normal.at<float>(0, 0);
                        normal_near[1] = normal.at<float>(1, 0);
                        normal_near[2] = normal.at<float>(2, 0);
                    }
                }
                std::cout<<"[生成远近平面] 5";
                cerr << "  dis_near:"<<dis_near <<", dis_far:"<<dis_far<< endl;

                if(dis_near<dis_thresh_near || dis_far>dis_thresh_far)
                {
                    cerr << " [Error] 近平面过近，或 远平面过远。"<<dis_far<< endl;
                    exit(-1);
                }
                
                g2o::plane* plane_far = new g2o::plane();
                farest[0] = 0;  
                farest[1] = 0;  
                farest[2] = dis_far;  
                normal_far[0] = 0;
                normal_far[1] = 0;
                normal_far[2] = -1;
                plane_far->fromPointAndNormal(farest, normal_far);
                plane_far->mvPlaneCenter = farest;
                plane_far->color = Vector3d(0,0,1.0);
                plane_far->transform(pFrame->cam_pose_Twc); 
                bool useFar = Config::ReadValue<double>("SemanticPrior.useFar");
                if(useFar)
                    planes_world.push_back(plane_far);
                g2o::plane* plane_near = new g2o::plane();
                nearest[0] = 0;  
                nearest[1] = 0;  
                nearest[2] = dis_near;  
                normal_near[0] = 0;
                normal_near[1] = 0;
                normal_near[2] = 1;
                plane_near->fromPointAndNormal(nearest, normal_near);
                plane_near->mvPlaneCenter = nearest; 
                plane_near->color = Vector3d(0,0,1.0);
                plane_near->transform(pFrame->cam_pose_Twc); 
                bool useNear = Config::ReadValue<double>("SemanticPrior.useNear");
                if(useNear)
                    planes_world.push_back(plane_near);

                mpMap->clearPlanes();
                mpMap->addPlane(&mGroundPlane);
                for(auto p:planes_world){
                    double plane_size=0.5;
                    p->InitFinitePlane(p->mvPlaneCenter, plane_size);
                    mpMap->addPlane(p);
                }

                bool optimizeEllipsoidWithMultiPlanes = Config::ReadValue<double>("optimizeEllipsoidWithMultiPlanes.optimizeEllipsoidWithMultiPlanes");
                if(optimizeEllipsoidWithMultiPlanes) 
                {
                    // 3.转为planes_in_current_camera
                    std::vector<g2o::plane> planes_in_current_camera;
                    // 将地面 转为planes_in_current_camera
                    planes_in_current_camera.push_back(ground_pl_local);
                    // 将bbox planes_world 和 远近平面， 转为planes_in_current_camera
                    for(auto p:planes_world){
                        g2o::plane* plane_new = new g2o::plane(p->param);
                        plane_new->transform(pFrame->cam_pose_Tcw);    //转到当前相机坐标系
                        planes_in_current_camera.push_back(*plane_new);
                    }
                    // 将 远近平面 转为planes_in_current_camera

                    std::cout<<"[生成远近平面] 6 planes_in_current_camera.size()"<< planes_in_current_camera.size() <<std::endl;

                    std::vector<g2o::plane> planesWithNormal;
                    g2o::ellipsoid e_infer_mono_guess;
                    // e_infer_mono_guess = pi.optimizeEllipsoidWithMultiPlanes(e_init_guess, planes_in_current_camera, pri);   // std::vector<g2o::plane> &planes
                    double ground_weight = Config::ReadValue<double>("SemanticPrior.GroundWeight");
                    e_infer_mono_guess = pi.optimizeEllipsoidWithPlanesAndPrior(e_init_guess, planes_in_current_camera, planesWithNormal, pri,weight, ground_weight);
                    // 设置椭球体label, prob
                    e_infer_mono_guess.miLabel = m.ob_2d.label;
                    e_infer_mono_guess.prob = m.ob_2d.rate; // 暂时设置为 bbox 检测的概率吧
                    e_infer_mono_guess.bbox = m.ob_2d.bbox;
                    e_infer_mono_guess.prob_3d =  1.0; // 暂定!
                    g2o::ellipsoid* pEInfer_mono_guess = new g2o::ellipsoid(e_infer_mono_guess.transform_from(pFrame->cam_pose_Twc));

                    Vector3d color_rgb(144,238,144); color_rgb/=255.0;
                    if(!use_input_pri) color_rgb = Vector3d(1,0,0); // 默认版本为红色
                    pEInfer_mono_guess->setColor(color_rgb);
                    // mpMap->addEllipsoidObservation(pEInfer_mono_guess); // 可视化
                    mpMap->addEllipsoidVisual(pEInfer_mono_guess); // 可视化

                    std::cout << " Before Monocular Infer: " << e_init_guess.toMinimalVector().transpose() << std::endl;
                    std::cout << " After Monocular Infer: " << e_infer_mono_guess.toMinimalVector().transpose() << std::endl;


                    // --------- 将结果放到frame中存储
                    if(replace_detection)
                        pFrame->mpLocalObjects[i] = new g2o::ellipsoid(e_infer_mono_guess);
                
                    // DEBUGING: 调试为何Z轴会发生变化， 先输出在局部坐标系下的两个rotMat
                    std::cout << "InitGuess RotMat in Camera: " << std::endl << e_init_guess.pose.rotation().toRotationMatrix() << std::endl;
                    std::cout << "Infered RotMat in Camera: " << std::endl << e_infer_mono_guess.pose.rotation().toRotationMatrix() << std::endl;
                    std::cout << "GroundPlaneNorma in Camera: " << std::endl << ground_pl_local.normal().head(3).normalized() << std::endl;

                    // 可视化bbox的约束平面
                    mpMap->clearPlanes();
                    // mpTracker->SetGroundPlaneMannually(Vector4d(0,  0,   1,  0));
                    mpMap->addPlane(&mGroundPlane);
                    VisualizeConstrainPlanes(e_infer_mono_guess, pFrame->cam_pose_Twc, mpMap); // 中点定在全局坐标系
                }

                // debug
                int frame_by_frame = Config::ReadValue<double>("frame_by_frame_zhjd");
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
            
           }
           else if(debug_init_guess==4){
                
                g2o::ellipsoid e_init_guess = pi.GenerateInitGuess(bbox, ground_pl_local.param);
                g2o::ellipsoid* e_init_guess_1 = new g2o::ellipsoid(e_init_guess.transform_from(pFrame->cam_pose_Twc)); 
                e_init_guess_1->prob = 1.0;
                e_init_guess_1->setColor(Vector3d(0.2,0.2,0.2));
                std::cout<<"[InferObjectsWithSemanticPrior] 2 结束 初始化一个椭球体,Scale: "<< e_init_guess_1->scale.transpose() <<std::endl;
                mpMap->ClearEllipsoidsVisual(); 
                mpMap->addEllipsoidVisual(e_init_guess_1); // 可视化
            
                std::vector<g2o::plane*> planes_world;
                
                // 1.生成 远近平面 in world
                double dis_thresh_near = Config::ReadValue<double>("OptimizeEllipsoidWithMultiPlanes.dis_thresh_near");
                double dis_thresh_far = Config::ReadValue<double>("OptimizeEllipsoidWithMultiPlanes.dis_thresh_far");
                Eigen::Vector3d farest, nearest;
                Eigen::Vector3d normal_far, normal_near;
                double dis_far=0, dis_near=1000;
                cv::Mat Ow = pFrame->GetCameraCenter();
                std::cout<<"[生成远近平面] 1 begin"<<std::endl;
                for(auto pMP:m.mvpObjectPoints){
                    if (!pMP)
                        continue;
                    if (pMP->isBad())
                        continue;
                    if (pMP->isOutlier())
                        continue;

                    auto p_pose = pMP->GetWorldPos();
                    // 计算p和相机的距离
                    cv::Mat normal = p_pose - Ow;
                    // 计算normal的长度
                    double dis = cv::norm(normal);
                    if(dis>dis_thresh_far)
                        continue;
                    if(dis>dis_far){
                        dis_far = dis;
                        farest[0] = p_pose.at<float>(0, 0);  
                        farest[1] = p_pose.at<float>(1, 0);  
                        farest[2] = p_pose.at<float>(2, 0);  
                        normal_far[0] = -1*normal.at<float>(0, 0);
                        normal_far[1] = -1*normal.at<float>(1, 0);
                        normal_far[2] = -1*normal.at<float>(2, 0);
                    }
                    if(dis<dis_thresh_near)
                        continue;
                    if(dis<dis_near){
                        dis_near = dis;
                        nearest[0] = p_pose.at<float>(0, 0);  
                        nearest[1] = p_pose.at<float>(1, 0);  
                        nearest[2] = p_pose.at<float>(2, 0);  
                        normal_near[0] = normal.at<float>(0, 0);
                        normal_near[1] = normal.at<float>(1, 0);
                        normal_near[2] = normal.at<float>(2, 0);
                    }
                }
                std::cout<<"[生成远近平面] 5";
                cerr << "  dis_near:"<<dis_near <<", dis_far:"<<dis_far<< endl;

                if(dis_near<dis_thresh_near || dis_far>dis_thresh_far)
                {
                    cerr << " [Error] 近平面过近，或 远平面过远。"<<dis_far<< endl;
                    exit(-1);
                }
                
                g2o::plane* plane_far = new g2o::plane();
                bool useFar_vertical = Config::ReadValue<double>("SemanticPrior.useFar_vertical");
                if(useFar_vertical){
                    farest[0] = 0;  
                    farest[1] = 0;  
                    farest[2] = dis_far;  
                    normal_far[0] = 0;
                    normal_far[1] = 0;
                    normal_far[2] = -1;
                }
                plane_far->fromPointAndNormal(farest, normal_far);
                plane_far->mvPlaneCenter = farest;
                plane_far->color = Vector3d(0,0,1.0);
                plane_far->transform(pFrame->cam_pose_Twc); 
                bool useFar = Config::ReadValue<double>("SemanticPrior.useFar");
                if(useFar)
                    planes_world.push_back(plane_far);
                g2o::plane* plane_near = new g2o::plane();
                bool useNear_vertical = Config::ReadValue<double>("SemanticPrior.useNear_vertical");
                if(useNear_vertical){
                    nearest[0] = 0;  
                    nearest[1] = 0;  
                    nearest[2] = dis_near;  
                    normal_near[0] = 0;
                    normal_near[1] = 0;
                    normal_near[2] = 1;
                }
                plane_near->fromPointAndNormal(nearest, normal_near);
                plane_near->mvPlaneCenter = nearest; 
                plane_near->color = Vector3d(0,0,1.0);
                plane_near->transform(pFrame->cam_pose_Twc); 
                bool useNear = Config::ReadValue<double>("SemanticPrior.useNear");
                if(useNear)
                    planes_world.push_back(plane_near);

                // mpMap->clearPlanes();
                // mpMap->addPlane(&mGroundPlane);
                // for(auto p:planes_world){
                //     double plane_size=0.5;
                //     p->InitFinitePlane(p->mvPlaneCenter, plane_size);
                //     mpMap->addPlane(p);
                // }

                bool optimizeEllipsoidWithMultiPlanes = Config::ReadValue<double>("optimizeEllipsoidWithMultiPlanes.optimizeEllipsoidWithMultiPlanes");
                std::vector<g2o::plane> planesNearFar;
                if(optimizeEllipsoidWithMultiPlanes) 
                {
                    // 将bbox planes_world 和 远近平面， 转为planes_in_current_camera
                    for(auto p:planes_world){
                        g2o::plane* plane_new = new g2o::plane(p->param);
                        plane_new->transform(pFrame->cam_pose_Tcw);    //转到当前相机坐标系
                        planesNearFar.push_back(*plane_new);
                    }
                    // 将 远近平面 转为planes_in_current_camera

                    g2o::ellipsoid e_infer_mono_guess;
                    double ground_weight = Config::ReadValue<double>("SemanticPrior.GroundWeight");
                    e_infer_mono_guess = pi.MonocularInferWithNearFarPlane(e_init_guess, pri, weight, ground_pl_local, planesNearFar);
                    // 设置椭球体label, prob
                    e_infer_mono_guess.miLabel = m.ob_2d.label;
                    e_infer_mono_guess.prob = m.ob_2d.rate; // 暂时设置为 bbox 检测的概率吧
                    e_infer_mono_guess.bbox = m.ob_2d.bbox;
                    e_infer_mono_guess.prob_3d =  1.0; // 暂定!
                    g2o::ellipsoid* pEInfer_mono_guess = new g2o::ellipsoid(e_infer_mono_guess.transform_from(pFrame->cam_pose_Twc));

                    Vector3d color_rgb(144,238,144); color_rgb/=255.0;
                    if(!use_input_pri) color_rgb = Vector3d(1,0,0); // 默认版本为红色
                    pEInfer_mono_guess->setColor(color_rgb);
                    // mpMap->addEllipsoidObservation(pEInfer_mono_guess); // 可视化
                    mpMap->addEllipsoidVisual(pEInfer_mono_guess); // 可视化

                    std::cout << " Before Monocular Infer: " << e_init_guess.toMinimalVector().transpose() << std::endl;
                    std::cout << " After Monocular Infer: " << e_infer_mono_guess.toMinimalVector().transpose() << std::endl;


                    // --------- 将结果放到frame中存储
                    if(replace_detection)
                        pFrame->mpLocalObjects[i] = new g2o::ellipsoid(e_infer_mono_guess);
                
                    // DEBUGING: 调试为何Z轴会发生变化， 先输出在局部坐标系下的两个rotMat
                    std::cout << "InitGuess RotMat in Camera: " << std::endl << e_init_guess.pose.rotation().toRotationMatrix() << std::endl;
                    std::cout << "Infered RotMat in Camera: " << std::endl << e_infer_mono_guess.pose.rotation().toRotationMatrix() << std::endl;
                    std::cout << "GroundPlaneNorma in Camera: " << std::endl << ground_pl_local.normal().head(3).normalized() << std::endl;

                    // 可视化bbox的约束平面
                    mpMap->clearPlanes();
                    // mpTracker->SetGroundPlaneMannually(Vector4d(0,  0,   1,  0));
                    mpMap->addPlane(&mGroundPlane);
                    VisualizeConstrainPlanes(e_infer_mono_guess, pFrame->cam_pose_Twc, mpMap); // 中点定在全局坐标系
                }

                // debug
                int frame_by_frame = Config::ReadValue<double>("frame_by_frame_zhjd");
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
            
           }
        }       

        std::cout << "Finish infering for " << meas_num << " objects..." << std::endl;
        return;
    }




    // [改进]
    void Tracking::SetGroundPlaneMannually(const Eigen::Vector4d &param)
    {
        std::cout << "[GroundPlane] Set groundplane mannually: " << param.transpose() << std::endl;
        miGroundPlaneState = 3;
        mGroundPlane.param = param;
        mGroundPlane.color = Vector3d(0,1,0);
    }


    void Tracking::SetRealPose(){
        // std::cout << "[Set real pose for the first frame from] : "<< mStrSettingPath << std::endl;
        cv::FileStorage fSettings(mStrSettingPath, cv::FileStorage::READ);
        int ConstraintType = fSettings["ConstraintType"];
        if ( ConstraintType != 1 && ConstraintType != 2 && ConstraintType != 3){
            std::cerr << ">>>>>> [WARRNING] USE NO PARAM CONSTRAINT TYPE!" << std::endl;
            // ConstraintType = 1;
            std::exit(EXIT_FAILURE);  // 或者：std::abort();
        }
        if (ConstraintType == 1){// robot_camera tf
            float qx = fSettings["Tworld_camera.qx"], qy = fSettings["Tworld_camera.qy"], qz = fSettings["Tworld_camera.qz"], qw = fSettings["Tworld_camera.qw"],
                    tx = fSettings["Tworld_camera.tx"], ty = fSettings["Tworld_camera.ty"], tz = fSettings["Tworld_camera.tz"];
            //float qx = fSettings["Tgroud_firstcamera.qx"], qy = fSettings["Tgroud_firstcamera.qy"], qz = fSettings["Tgroud_firstcamera.qz"], qw = fSettings["Tgroud_firstcamera.qw"],
            //       tx = fSettings["Tgroud_firstcamera.tx"], ty = fSettings["Tgroud_firstcamera.ty"], tz = fSettings["Tgroud_firstcamera.tz"];
            mCurrentFrame.mGroundtruthPose_mat = cv::Mat::eye(4, 4, CV_32F);
            Eigen::Quaterniond quaternion(Eigen::Vector4d(qx, qy, qz, qw));
            Eigen::AngleAxisd rotation_vector(quaternion);
            Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
            T.rotate(rotation_vector);
            T.pretranslate(Eigen::Vector3d(tx, ty, tz));
            Eigen::Matrix4d GroundtruthPose_eigen = T.matrix();
            cv::Mat cv_mat_32f;
            cv::eigen2cv(GroundtruthPose_eigen, cv_mat_32f);
            cv_mat_32f.convertTo(mCurrentFrame.mGroundtruthPose_mat, CV_32F);

        } else if(ConstraintType == 2){
            // TODO: IMU
        } else if (ConstraintType == 3){// ros tf
            // tf::TransformListener listener;
            // tf::StampedTransform transform;
            // cv::Mat T_w_camera = cv::Mat::eye(4,4,CV_32F);
            // try
            // {
            //     listener.waitForTransform("/map", "/camera_depth_optical_frame", ros::Time(0), ros::Duration(1.0));
            //     listener.lookupTransform("/map", "/camera_depth_optical_frame", ros::Time(0), transform);
            //     T_w_camera = Converter::Quation2CvMat(
            //                     transform.getRotation().x(),
            //                     transform.getRotation().y(),
            //                     transform.getRotation().z(),
            //                     transform.getRotation().w(),
            //                     transform.getOrigin().x(),
            //                     transform.getOrigin().y(),
            //                     transform.getOrigin().z()
            //             );
            // }
            // catch (tf::TransformException &ex)
            // {
            //     ROS_ERROR("%s -->> lost tf from /map to /base_footprint",ex.what());
            // }

            // mCurrentFrame.mGroundtruthPose_mat = T_w_camera;
        }

        // std::cout << "[Set real pose for the first frame from] : End" << std::endl;
    }

    

}
