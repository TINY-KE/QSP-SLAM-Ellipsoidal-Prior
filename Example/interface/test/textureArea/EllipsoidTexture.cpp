
#include <include/core/Ellipsoid.h>
#include <System.h>
#include <Map.h>
#include <src/config/Config.h>

#include <opencv2/core/core.hpp>

#include "EllipsoidTexture.h"

using namespace ORB_SLAM2;
using namespace Eigen;
using namespace std;

typedef Eigen::Matrix<double, 6, 1> Vector6d;

    VectorXd EllipsoidTex::GetCircle()
    {
        return this->vec_minimal;
    }

// private:

    // theta range: [0,pi]
    Circle3D EllipsoidTex::GenerateCircle(double theta)
    {
        Circle3D e = this->rotate_ellipsoid(theta);
        
        // x为新的x轴长度
        // 椭圆参数方程为 x=acosθ，y=bsinθ
        e.scale[0] = sqrt(pow(e.scale[0]*cos(theta),2) + pow(e.scale[1]*sin(theta),2));

        // y为0.
        e.scale[1] = 0;

        return e;
    }

    Vector3d EllipsoidTex::GenerateNormCenterToPixel(const Vector2d& uv, const Matrix3d& calib) const
    {
        // 注意考虑fx,fy不一样的情况，以及为负数的情况

        // 1. 从平面获得空间中的直线  l = P^T x 

        // auto P = generateProjectionMatrix(g2o::SE3Quat(), calib);
        // Vector3d x; x << uv, 1;
        // Vector4d l = P.transpose() * x;
        // Vector3d norm = l.head(3).normalized();

        double cx = calib(0,2);
        double cy = calib(1,2);
        double fx = calib(0,0);
        double fy = calib(1,1);

        double u = uv[0];
        double v = uv[1];
        
        Vector3d nx(2*(u-cx),0,fx);
        Vector3d ny(0,2*(v-cy),fy);

        if(fx<0) nx = -nx;
        if(fy<0) ny = -ny;

        Vector3d ns = nx + ny;

        Vector3d norm = ns.normalized();

        return norm;
    }

    // near, far
    // 若无解则为空
    MatrixXd EllipsoidTex::Find3DPoint(const Vector2d& uv, const g2o::SE3Quat& camera_wc, const Matrix3d& calib) const
    {
        // 首先全部转换到 camera坐标系，然后将点转回世界坐标系!
        g2o::ellipsoid e_c = this->transform_from(camera_wc.inverse()); // 局部系椭球体

        Vector3d camera_center(0,0,0);

        // // 核心思路，根据投影关系，以参数lambda构造直线上的点，然后代入二次方程直接求解
        // double cx = calib(0,2);
        // double cy = calib(1,2);
        // double f = std::abs(calib(0,0));
        
        // // 参数: a
        // Vector3d norm_cp_3 = Vector3d(uv[0],uv[1],0) - Vector3d(cx, cy, -f);   // camera to pixel ; 方向保证 lambda 小为近
        // Vector4d norm_cp; norm_cp << norm_cp_3,1;

        // 更新：修复fx,fy的正负号不一致时的bug；使得求解过程更一般化
        Vector3d norm_cp_3 = GenerateNormCenterToPixel(uv, calib);

        // 参数: C
        Vector4d C; C << camera_center,1;

        Matrix4d Q = e_c.generateQuadric().inverse();

        // 未考虑齐次坐标的错误推导
        // double param_a = norm_cp.transpose() * Q * norm_cp;
        // double param_c = C.transpose() * Q * C;
        // double param_b = ((C.transpose() * Q * norm_cp) + (norm_cp.transpose() * Q * C))[0];

        MatrixXd Q0 = Q.topLeftCorner(3,3);
        MatrixXd Q1 = Q.topRightCorner(3,1);
        MatrixXd Q2 = Q.bottomLeftCorner(1,3);
        MatrixXd Q3 = Q.bottomRightCorner(1,1);

        MatrixXd mat_param_a = norm_cp_3.transpose() * Q0 * norm_cp_3;
        MatrixXd mat_param_b = (Q2 * norm_cp_3 + norm_cp_3.transpose() * Q1);
        MatrixXd mat_param_c = (Q3);
        double param_a = mat_param_a(0,0);
        double param_c = mat_param_c(0,0);
        double param_b = mat_param_b(0,0);

 
        double delta = param_b*param_b - 4 * param_a * param_c;

        // std::cout << "param_a : " << param_a << std::endl;
        // std::cout << "param_c : " << param_c << std::endl;
        // std::cout << "param_b : " << param_b << std::endl;
        // std::cout << "delta : " << delta << std::endl;

        MatrixXd pointMat;
        if( delta >= 0 )
        {
            // 两个解
            double sqrt_delta = sqrt(delta);
            double lambda_big = (-param_b + sqrt_delta ) / (2 * param_a);
            double lambda_small = (-param_b - sqrt_delta ) / (2 * param_a);

            Vector3d p_near = lambda_small * norm_cp_3;
            Vector3d p_far = lambda_big * norm_cp_3;

            // 转换到世界系里去
            p_near = TransformPoint(p_near, camera_wc.to_homogeneous_matrix());
            p_far = TransformPoint(p_far, camera_wc.to_homogeneous_matrix());

            pointMat.resize(2,3);
            pointMat.row(0) = p_near;
            pointMat.row(1) = p_far;
        }
        
        return pointMat;
    };


    bool EllipsoidTex::FindNearest3DPoint(Vector3d& point, const Vector2d& uv, const g2o::SE3Quat& camera_wc, const Matrix3d& calib) const
    {
        MatrixXd pointMat = Find3DPoint(uv,camera_wc,calib);
        if(pointMat.rows()>0){
            point = pointMat.row(0);
            return true;
        }
        else 
            return false;
    };

    double EllipsoidTex::GetAnglesOfPoint(const Vector3d& point_3d) const
    {
        // 先把点转换到局部系下
        Vector3d const_vec(point_3d);
        Vector3d point_e = TransformPoint(const_vec, pose.inverse().to_homogeneous_matrix());

        // Z轴坐标
        Vector3d norm_z(0,0,1);
        // 原点到3d点向量
        Vector3d norm_op = point_3d;
        // 叉乘
        Vector3d norm_3d = norm_z.cross(norm_op);

        // XZ平面的法向量即Y轴
        Vector3d norm_0(0,1,0);

        // 估计角度，即 norm_3d 与 norm_0 的夹角
        double cos_theta = norm_3d.dot(norm_0) / norm_3d.norm() / norm_0.norm();
        double theta = acos(cos_theta);

        // 分析角度是否有区域限制，是否有方向限制
        // 角度在 0-pi 之间， 有重复。不过没关系。

        return theta;


    };

    cv::Mat EllipsoidTex::GenerateAreaMat(int rows, int cols, const g2o::SE3Quat& campose_wc, const Matrix3d& calib, ORB_SLAM2::Map* pMap){
        // 获得所有需要测试的二维点对应的三维点坐标
        // 1、从椭圆中获得区域
        clock_t time0_start = clock();
        cv::Mat im_3d_valid(rows, cols, CV_8UC1);
        cv::Mat im_3d_points(rows, cols, CV_64FC3);
        im_3d_valid.setTo(0);
        im_3d_points.setTo(cv::Vec3d(0,0,0));

        Matrix3d ellipseMat = this->projectOntoImageEllipseMat(campose_wc.inverse(), calib);
        for(int x=0;x<cols;x+=2)
            for(int y=0;y<rows;y+=2)
            {
                Vector3d uv(x,y,1);
                double value = uv.transpose() * ellipseMat * uv;
                if(value < 0){
                    // 获得对应三维点坐标
                    Vector3d near_p;
                    bool result = this->FindNearest3DPoint(near_p, uv.head(2), campose_wc, calib);
                    // 找最近的三维点
                    if(result)
                    {
                        // 存储三维点到矩阵中
                        im_3d_points.at<cv::Vec3d>(y,x) = cv::Vec3d(near_p[0], near_p[1], near_p[2]);
                        im_3d_valid.at<uchar>(y,x) = 1;
                    }
                    
                }

            }

        // 可视化矩阵
        // cv::imshow("im_3d_points", im_3d_points);
        // cv::waitKey();

        clock_t time1_im_3d_points = clock();

        // 对三维坐标检查其是否位于给定弧面上
        // 给定三维点，求其与Z轴构成的平面，与椭球XZ平面的夹角.
        cv::Mat im_3d_angles(rows, cols, CV_64FC1);
        im_3d_angles.setTo(0);
        for(int x=0;x<cols;x++)
            for(int y=0;y<rows;y++)
            {
                // 遍历有三维位置的矩阵点，计算其角度矩阵
                if(im_3d_valid.at<uchar>(y,x) == 1)
                {
                    cv::Vec3d xyz = im_3d_points.at<cv::Vec3d>(y,x);
                    Vector3d point_3d(xyz[0],xyz[1],xyz[2]);
                    double theta = this->GetAnglesOfPoint(point_3d);
                    
                    im_3d_angles.at<double>(y,x) = theta;
                }
                
                
            }
        

        // 根据是否位于弧面可视化图像
        // 可视化角度?
        // cv::imshow("im_3d_angles", im_3d_angles);
        // cv::waitKey(0);
        clock_t time2_im_3d_angles = clock();

        // 统计
        // cv::Mat meanResult,stddevResult;
        // cv::meanStdDev(im_3d_angles ,meanResult,stddevResult);
        // double meanValue=meanResult.at<double>(0,0);
        // double std=stddevResult.at<double>(0,0);
        // std::cout << "meanValue: " << meanValue << ", std: "  << std << std::endl;

        // 开始划分区域了。
        cv::Mat im_3d_area(rows, cols, CV_8UC1);
        im_3d_area.setTo(0);    // 0 为无效区域

        int CONFIG_N = 20;
        double CONFIG_ANGLE_DELTA = M_PI / CONFIG_N;
        for(int x=0;x<cols;x++)
            for(int y=0;y<rows;y++)
            {
                // 遍历有三维位置的矩阵点，计算其角度矩阵
                if(im_3d_valid.at<uchar>(y,x) == 1)
                {
                    uchar area_id = 0;
                    double theta = im_3d_angles.at<double>(y,x);
                    area_id = theta / CONFIG_ANGLE_DELTA;
                    
                    im_3d_area.at<uchar>(y,x) = area_id;
                }
                
                
            }    
        
        // cv::imshow("im_3d_area", im_3d_area);
        clock_t time3_im_3d_area = clock();

        std::cout << "time1_im_3d_points: " <<(double)(time1_im_3d_points - time0_start) / CLOCKS_PER_SEC << "s" << endl;
        std::cout << "time2_im_3d_angles: " <<(double)(time2_im_3d_angles - time1_im_3d_points) / CLOCKS_PER_SEC << "s" << endl;
        std::cout << "time3_im_3d_area: " <<(double)(time3_im_3d_area - time2_im_3d_angles) / CLOCKS_PER_SEC << "s" << endl;

        return im_3d_area;

    };

    // 历史遗留疑问： 如何让一个matrix具有默认参数，还可以判断是否被外在传入了?

    // 输出的点，若无效，则为 (-1,-1)
    std::vector<cv::Point2i> EllipsoidTex::generateFlectedPoints(const std::vector<cv::Point2i>& pixels, const g2o::SE3Quat& campose_wc, const Matrix3d& calib)
    {
        // 对每个点寻找对称点
        Matrix3d ellipseMat = this->projectOntoImageEllipseMat(campose_wc.inverse(), calib);
        Matrix3Xd  P = generateProjectionMatrix(campose_wc.inverse(), calib);

        std::vector<cv::Point2i> pixels_out;
        for(auto &p: pixels)
        {
            cv::Point2i p_out(-1,-1);

            double x = double(p.x);
            double y = double(p.y);

            Vector3d uv(x,y,1);
            double value = uv.transpose() * ellipseMat * uv;
            if(value < 0){
                // 获得对应三维点坐标
                Vector3d near_p;
                bool result = this->FindNearest3DPoint(near_p, uv.head(2), campose_wc, calib);
                // 找最近的三维点
                if(result)
                {
                    // 存储三维点到矩阵中
                    // 生成对称点
                    Vector3d p_flected = this->flectPoint(near_p);

                    // 投影到图像中!
                    Vector4d center_homo = real_to_homo_coord<double>(p_flected);
                    Vector3d u_homo = P * center_homo;
                    Vector2d u = homo_to_real_coord_vec<double>(u_homo);

                    cv::Point2i uv(u[0],u[1]);
                    p_out = uv;
                }
                else 
                    // 如果属于椭球体，但是没找到三维点，则想办法可视化出来
                    p_out = cv::Point2i(-2,-2);
            }
            else // 甚至不在椭球体上!
                p_out = cv::Point2i(-3,-3);
            pixels_out.push_back(p_out);
        }
        return pixels_out;
    };

    Vector3d EllipsoidTex::flectPoint(const Vector3d& near_p)
    {
        // 判断与0平面的夹角，然后做对称；即相对于XZ平面做对称

        Vector3d center = this->translation();
        Vector3d norm = this->pose.rotation().toRotationMatrix().col(1);  // Y Axis!
        // Vector3d norm = this->pose.rotation().toRotationMatrix().col(0);  // X Axis!
        
        // 从一个平面(点、法向量) 对称一个点
        // Vector3d p1 = near_p - center;
        // double norm_value = p1.dot(norm);
        // Vector3d p2 = p1 - norm_value*norm;
        // Vector3d center_to_flectp = p1 - 2 * p2;
        // Vector3d flectp = center + center_to_flectp;

        // 步骤1： 计算点到平面的距离
        g2o::plane pl; pl.fromPointAndNormal(center, norm);
        double dis = pl.distanceToPoint(near_p, true);
        Vector3d flectp = near_p - 2 * dis * norm;

        return flectp;
    };

    // Vector2d projectPointIntoImagePoint(const Vector3d& point, const SE3Quat& campose_cw, const Matrix3d& Kalib) const
    // {
    //     Matrix3Xd  P = generateProjectionMatrix(campose_cw, Kalib);

    //     Vector3d center_pos = point;

    //     Vector4d center_homo = real_to_homo_coord<double>(center_pos);
    //     Vector3d u_homo = P * center_homo;
    //     Vector2d u = homo_to_real_coord_vec<double>(u_homo);

    //     return u;
    // }


