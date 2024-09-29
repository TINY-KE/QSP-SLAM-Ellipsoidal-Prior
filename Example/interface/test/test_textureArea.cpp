// 新建实验室： TEXT_AREA
// 测试椭球体所构建的三维区域在二维投影上的纹理约束，主要目的在于精确估计转角，并测试其对于综合性能之提升。
// （纹理椭球思路的进化版）

#include <include/core/Ellipsoid.h>
#include <System.h>
#include <Map.h>
#include <src/config/Config.h>

#include <opencv2/core/core.hpp>

using namespace ORB_SLAM2;
using namespace Eigen;
using namespace std;

typedef Eigen::Matrix<double, 6, 1> Vector6d;

// 定义常用函数
vector<cv::Point2i> toPoint2i(const vector<cv::KeyPoint>& in)
{
    vector<cv::Point2i> out; out.reserve(in.size());
    for(auto&p : in)
    {
        cv::Point2i pout(p.pt.x, p.pt.y);
        out.push_back(pout);
    }
    return out;
};

vector<cv::KeyPoint> toKeyPoints(const vector<cv::Point2i>& in)
{
    vector<cv::KeyPoint> out; out.reserve(in.size());
    for(auto&p : in)
    {
        cv::KeyPoint pout;
        pout.pt = cv::Point2f(p.x, p.y);
        out.push_back(pout);
    }
    return out;
};

class Circle3D: public g2o::ellipsoid
{
public:
    Circle3D():g2o::ellipsoid(){}
    Circle3D(const g2o::ellipsoid& e):g2o::ellipsoid(e){}
    
};

class EllipsoidTex: public g2o::ellipsoid
{
public:
    EllipsoidTex(){};
    EllipsoidTex(const g2o::ellipsoid& e):g2o::ellipsoid(e){};

    VectorXd GetCircle()
    {
        return this->vec_minimal;
    }

// private:

    // theta range: [0,pi]
    Circle3D GenerateCircle(double theta)
    {
        Circle3D e = this->rotate_ellipsoid(theta);
        
        // x为新的x轴长度
        // 椭圆参数方程为 x=acosθ，y=bsinθ
        e.scale[0] = sqrt(pow(e.scale[0]*cos(theta),2) + pow(e.scale[1]*sin(theta),2));

        // y为0.
        e.scale[1] = 0;

        return e;
    }

    // near, far
    // 若无解则为空
    MatrixXd Find3DPoint(const Vector2d& uv, const g2o::SE3Quat& camera_wc, const Matrix3d& calib) const
    {
        // 首先全部转换到 camera坐标系，然后将点转回世界坐标系!
        g2o::ellipsoid e_c = this->transform_from(camera_wc.inverse()); // 局部系椭球体

        Vector3d camera_center(0,0,0);

        // 核心思路，根据投影关系，以参数lambda构造直线上的点，然后代入二次方程直接求解
        double cx = calib(0,2);
        double cy = calib(1,2);
        double f = std::abs(calib(0,0));
        
        // 参数: a
        Vector3d norm_cp_3 = Vector3d(uv[0],uv[1],0) - Vector3d(cx, cy, -f);   // camera to pixel ; 方向保证 lambda 小为近
        Vector4d norm_cp; norm_cp << norm_cp_3,1;

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


    bool FindNearest3DPoint(Vector3d& point, const Vector2d& uv, const g2o::SE3Quat& camera_wc, const Matrix3d& calib) const
    {
        MatrixXd pointMat = Find3DPoint(uv,camera_wc,calib);
        if(pointMat.rows()>0){
            point = pointMat.row(0);
            return true;
        }
        else 
            return false;
    };

    double GetAnglesOfPoint(const Vector3d& point_3d) const
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

    cv::Mat GenerateAreaMat(int rows, int cols, const g2o::SE3Quat& campose_wc, const Matrix3d& calib, ORB_SLAM2::Map* pMap){
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
    std::vector<cv::Point2i> generateFlectedPoints(const std::vector<cv::Point2i>& pixels, const g2o::SE3Quat& campose_wc, const Matrix3d& calib)
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
            }
            pixels_out.push_back(p_out);
        }
        return pixels_out;
    };

    Vector3d flectPoint(const Vector3d& near_p)
    {
        // 判断与0平面的夹角，然后做对称；即相对于XZ平面做对称

        Vector3d center = this->translation();
        Vector3d norm = this->pose.rotation().toRotationMatrix().col(1);  // Y Axis!
        
        // 从一个平面(点、法向量) 对称一个点
        Vector3d p1 = near_p - center;
        double norm_value = p1.dot(norm);
        Vector3d p2 = p1 - norm_value*norm;
        Vector3d center_to_flectp = p1 - 2 * p2;
        Vector3d flectp = center + center_to_flectp;

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


};

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

void highlightTextureArea(const Matrix3d& ellipse, const Matrix3d& ellipse2, cv::Mat& im, const cv::Scalar& color = cv::Scalar(0,255,0))
{

    std::cout << "Ellipse Mat 1 : " << std::endl << ellipse << std::endl << std::endl;
    std::cout << "Ellipse Mat 2 : " << std::endl << ellipse2 << std::endl << std::endl;

    cv::Mat valueMat1(im.rows, im.cols, CV_64FC1);
    cv::Mat valueMat2(im.rows, im.cols, CV_64FC1);

    int rows = im.rows;
    int cols = im.cols;
    for(int row_id=0;row_id<rows;row_id++)
        for(int col_id=0;col_id<cols;col_id++)
        {
            int x = col_id;
            int y = row_id;

            Vector3d uv(x,y,1);

            double value1 = uv.transpose() * ellipse * uv;
            double value2 = uv.transpose() * ellipse2 * uv;

            // 记录下来
            valueMat1.at<double>(y,x) = value1;
            valueMat2.at<double>(y,x) = value2;

            if(row_id == 0 && col_id == 0)
            {
                std::cout << "value 1: " << value1 << std::endl;
                std::cout << "value2 1: " << value2 << std::endl;
            }

            if((value1 * value2) < 0)
            {
                // 若符号不一致，则加亮
                im.at<cv::Vec3b>(y,x) = cv::Vec3b(color[0],color[1],color[2]);
            }
        }

    // 可视化！
    // cv::imshow("value1", valueMat1);
    // cv::imshow("value2", valueMat2);
}

void highlightTextureAreaIn3D(cv::Mat& im, const EllipsoidTex& e, const Matrix3d& ellipseMat, const g2o::SE3Quat& camera_wc, const Matrix3d& calib, ORB_SLAM2::Map* pMap)
{
    // 获得所有需要测试的二维点对应的三维点坐标
    // 1、从椭圆中获得区域
    clock_t time0_start = clock();
    cv::Mat im_3d_valid(im.rows, im.cols, CV_8UC1);
    cv::Mat im_3d_points(im.rows, im.cols, CV_64FC3);
    im_3d_valid.setTo(0);
    im_3d_points.setTo(cv::Vec3d(0,0,0));
    for(int x=0;x<im.cols;x+=2)
        for(int y=0;y<im.rows;y+=2)
        {
            Vector3d uv(x,y,1);
            double value = uv.transpose() * ellipseMat * uv;
            if(value < 0){
                im.at<cv::Vec3b>(y,x) = cv::Vec3b(100,100,100); // 标记颜色

                // 获得对应三维点坐标
                Vector3d near_p;
                bool result = e.FindNearest3DPoint(near_p, uv.head(2), camera_wc, calib);
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
    cv::imshow("im_3d_points", im_3d_points);
    // cv::waitKey();

    clock_t time1_im_3d_points = clock();

    // 对三维坐标检查其是否位于给定弧面上
    // 给定三维点，求其与Z轴构成的平面，与椭球XZ平面的夹角.
    cv::Mat im_3d_angles(im.rows, im.cols, CV_64FC1);
    im_3d_angles.setTo(0);
    for(int x=0;x<im.cols;x++)
        for(int y=0;y<im.rows;y++)
        {
            // 遍历有三维位置的矩阵点，计算其角度矩阵
            if(im_3d_valid.at<uchar>(y,x) == 1)
            {
                cv::Vec3d xyz = im_3d_points.at<cv::Vec3d>(y,x);
                Vector3d point_3d(xyz[0],xyz[1],xyz[2]);
                double theta = e.GetAnglesOfPoint(point_3d);
                
                im_3d_angles.at<double>(y,x) = theta;
            }
            
            
        }
    

    // 根据是否位于弧面可视化图像
    // 可视化角度?
    cv::imshow("im_3d_angles", im_3d_angles);
    // cv::waitKey(0);
    clock_t time2_im_3d_angles = clock();

    // 统计
    cv::Mat meanResult,stddevResult;
    cv::meanStdDev(im_3d_angles ,meanResult,stddevResult);
    double meanValue=meanResult.at<double>(0,0);
    double std=stddevResult.at<double>(0,0);
    std::cout << "meanValue: " << meanValue << ", std: "  << std << std::endl;

    // 开始划分区域了。
    cv::Mat im_3d_area(im.rows, im.cols, CV_8UC1);
    im_3d_area.setTo(0);    // 0 为无效区域

    int CONFIG_N = 20;
    double CONFIG_ANGLE_DELTA = M_PI / CONFIG_N;
    for(int x=0;x<im.cols;x++)
        for(int y=0;y<im.rows;y++)
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
    
    cv::imshow("im_3d_area", im_3d_area);
    clock_t time3_im_3d_area = clock();

    cv::Mat im_3d_area_for_show;
    cv::equalizeHist(im_3d_area, im_3d_area_for_show);
    cv::imshow("im_3d_area_for_show", im_3d_area_for_show);
    clock_t time4_im_im_3d_area_for_show = clock();

    std::cout << "time1_im_3d_points: " <<(double)(time1_im_3d_points - time0_start) / CLOCKS_PER_SEC << "s" << endl;
    std::cout << "time2_im_3d_angles: " <<(double)(time2_im_3d_angles - time1_im_3d_points) / CLOCKS_PER_SEC << "s" << endl;
    std::cout << "time3_im_3d_area: " <<(double)(time3_im_3d_area - time2_im_3d_angles) / CLOCKS_PER_SEC << "s" << endl;
    std::cout << "time4_im_im_3d_area_for_show: " <<(double)(time4_im_im_3d_area_for_show - time3_im_3d_area) / CLOCKS_PER_SEC << "s" << endl;

    // cv::waitKey(0);

}

void test3DPoints(const EllipsoidTex& e, const Vector5d& ellipse, const g2o::SE3Quat& camera_wc, const Matrix3d& calib, ORB_SLAM2::Map* pMap)
{
    // 测试中间点
    Vector2d uv_center = ellipse.head(2);

    // 多测试几个点
    MatrixXd test_mat(4,2);
    test_mat << 
        0.2,0.2,
        0.8,0.2,
        0.8,0.8,
        0.2,0.8;
    test_mat.col(0) *= (2*ellipse[3]);  // x axis 
    test_mat.col(1) *= (2*ellipse[4]);
    MatrixXd test_mat_full(5,2);
    test_mat_full.topLeftCorner(4,2) = test_mat;
    test_mat_full.bottomLeftCorner(1,2) << uv_center[0], uv_center[1];

    for(int i=0;i<test_mat_full.rows();i++){
        Vector2d uv = test_mat_full.row(i);
        MatrixXd pMat = e.Find3DPoint(uv, camera_wc, calib);
        if(pMat.rows()>0)
        {
            Vector3d point = pMat.row(0).head(3);
            ORB_SLAM2::PointXYZRGB* pp = new ORB_SLAM2::PointXYZRGB;
            ORB_SLAM2::PointXYZRGB& p = *pp;
            p.x = point[0]; p.y = point[1]; p.z = point[2]; 
            p.r = 0; p.g=0; p.b =255; p.size = 10;
            pMap->addPoint(pp);

            point = pMat.row(1).head(3);
            ORB_SLAM2::PointXYZRGB* pp2 = new ORB_SLAM2::PointXYZRGB;
            ORB_SLAM2::PointXYZRGB& p2 = *pp2;
            p2.x = point[0]; p2.y = point[1]; p2.z = point[2]; 
            p2.r = 255; p2.g=0; p2.b =0; p2.size = 10;
            pMap->addPoint(pp2);
        }
    }

    return;
}

void tool_showHistImage(const cv::Mat& histMat)
{
    double maxVal = 0;
    double minVal = 0;
    cv::minMaxLoc(histMat, &minVal, &maxVal, 0, 0);
    int size = 256;
    cv::Mat histImg(size, size, CV_8U, cv::Scalar(0));
    int hpt = static_cast<int>(0.9*size);

    for (int h = 0; h < 256; h++)
    {
        float binVal = histMat.at<float>(h);
        int intensity = static_cast<int>(binVal * hpt / maxVal);
        cv::line(histImg, cv::Point(h, size-1), cv::Point(h+1, size-1 - intensity), cv::Scalar::all(255));
    }

    static int his_num = 0;
    std::string str_im = std::string("Hist ") + to_string(his_num++);
    cv::imshow(str_im, histImg);
}

void tool_showMaskWithImage(const cv::Mat& mask, const cv::Mat& gray)
{
    static int his_num = 0;
    std::string str_im = std::string("Mask ") + to_string(his_num++);
    cv::Mat grayMask;
    gray.copyTo(grayMask, mask);
    cv::imshow(str_im, grayMask);
}

std::vector<cv::Mat> GenerateMaskFromAreaMat(const cv::Mat& areaMat)
{
    std::vector<cv::Mat> maskVec;
    uchar i = 0;
    while(1){
        cv::Mat mask;
        cv::inRange(areaMat, i, i+1, mask);
        i++;

        // 如何判断是否有有效值???
        cv::Mat meanResult, stddevResult;
        cv::meanStdDev(mask,meanResult,stddevResult);
        double meanValue=meanResult.at<double>(0,0);

        if(meanValue == 0) break;  // 当该数值 i 的区域为空

        maskVec.push_back(mask);  // 复制进去
    };
    return maskVec;
}

std::vector<cv::Mat> calculateHistInAreas(const cv::Mat& rgb, const cv::Mat& areaMat)
{
    // 仅支持灰度运算
    cv::Mat gray;
    cv::cvtColor(rgb, gray, CV_BGR2GRAY);

    // 按区域挨个可视化

    // 从 areaMat 中拆分N个 Mask

    // 从1开始统计直到没有一个数值

    std::vector<cv::Mat> maskVec = GenerateMaskFromAreaMat(areaMat); // 如何储存多个mask?
    std::cout << "Total Mask Num : " << maskVec.size() << std::endl;

    // 计算每个Mask内的 Hist
    std::vector<cv::Mat> distMatVec;
    for(auto& mask : maskVec)
    {
        int dims = 1;
        float hranges[] = { 0,255 };
        const float *ranges[]= { hranges };
        int size = 256;
        int channels = 0;

        cv::Mat dstHist;
        cv::calcHist(&gray, 1, &channels, mask, dstHist, dims, &size, ranges);
        distMatVec.push_back(dstHist);

        // 可视化
        tool_showHistImage(dstHist);
        tool_showMaskWithImage(mask, gray);
    }

    // 输出 Hist
    // 可视化所有Hist
    return distMatVec;
}

cv::Mat GenerateMaskFromBbox(int rows, int cols, const Vector4d& bbox)
{
    int x = round(bbox[0]);
    int y = round(bbox[1]);

    int width = round(std::abs(bbox[2]-bbox[0]));
    int height = round(std::abs(bbox[3]-bbox[1]));

    cv::Mat mask = cv::Mat::zeros(rows, cols, CV_8UC1);
    mask(cv::Rect(x,y,width,height)) = 255; // 将矩形区域设置为255

    return mask;
}

// 请确认第一个和第二个kps一一对应，且数量一致
void drawKeypointsPairs(const cv::Mat& im, cv::Mat& imOut, const std::vector<cv::KeyPoint>& kps1, const std::vector<cv::KeyPoint>& kps2, const cv::Scalar& scalar)
{
    cv::Mat out = im.clone();
    int num1 = kps1.size();    
    int num2 = kps2.size();
    if(num1 == num2)
    {
        for(int i=0;i<num1;i++)
        {
            const cv::KeyPoint& kp1 = kps1[i];
            const cv::KeyPoint& kp2 = kps2[i];

            // 检查有效性
            if(kp1.pt.x>0&&kp1.pt.y>0&&kp2.pt.x>0&&kp2.pt.y>0)
                cv::line(out, kp1.pt, kp2.pt, scalar);
        }
    }
    imOut = out.clone();
    return;
}

double sumPixelDiff(const cv::Mat& im, const vector<cv::KeyPoint>& kps1, const vector<cv::KeyPoint>& kps2)
{
    int diff = 0;
    int num1 = kps1.size();    
    int num2 = kps2.size();

    int valid_num = 0;
    if(num1 == num2)
    {
        for(int i=0;i<num1;i++)
        {
            const cv::KeyPoint& kp1 = kps1[i];
            const cv::KeyPoint& kp2 = kps2[i];

            uchar value1 = im.at<uchar>(kp1.pt);
            uchar value2 = im.at<uchar>(kp2.pt);

            // TODO: 注意对称点要在图像范围内
            if(kp1.pt.x>0&&kp1.pt.y>0&&kp2.pt.x>0&&kp2.pt.y>0)
            {
                diff += int(std::abs(value1-value2));
                valid_num++;
            }

        }
    }

    double average_diff = diff / double(valid_num);
    return average_diff;
}

// data 已经按 x 排序
cv::Mat drawXYPlot(const std::map<double,double>& data, double x_min, double x_max, double y_min, double y_max)
{
    int im_x = 400;
    int im_y = 400;

    // ---
    cv::Mat im = cv::Mat::zeros(400,400,CV_8UC3);
    cv::Point pt_last(0,0);
    for(auto& pt:data)
    {
        double x = pt.first;
        double y = pt.second;

        double x_norm = (x-x_min) / (x_max-x_min);
        double y_norm = (y-y_min) / (y_max-y_min);

        cv::Point pt_im(im_x*x_norm, im_y*y_norm);

        cv::line(im, pt_last, pt_im, cv::Scalar(0,255,0));

        pt_last = pt_im;
    }

    return im;
}

int main(int argc,char* argv[])
{
    std::cout << "Welcome!" << std::endl;
    if( argc != 2)
    {
        std::cout << "usage: " << argv[0] << " path_to_settings" << std::endl;
        return 1;
    }

    const string path_setting(argv[1]);
    std::cout << "Load settings from : " << path_setting << std::endl;

    ORB_SLAM2::System* pSLAM = new ORB_SLAM2::System(path_setting, true);
    ORB_SLAM2::Map* pMap = pSLAM->getMap();

    // 读取一张 rgb 图像
    cv::Mat rgb = cv::imread("/disk/workspace/datasets/ICL-NUIM/living_room_traj2n_frei_png/rgb/0.png"); // 假设rgb是一个彩色图.
    cv::imshow("rgb", rgb);
    cv::Mat gray;
    cv::cvtColor(rgb, gray, CV_BGR2GRAY);

    // 可视化一个标准椭球体
    EllipsoidTex e;
    Vector9d vec;
    vec << 0,0,0,0,0,0,4,5,3;
    e.fromMinimalVector(vec);

    pMap->addEllipsoidVisual(&e);  

    Circle3D circle, circle2;
    pMap->addEllipsoidVisual(&circle);  
    pMap->addEllipsoidVisual(&circle2);  

    // 接下来生成相机测试其投影
    g2o::SE3Quat campose_wc;
    Vector6d cam_vec; cam_vec << 0,-10,0,-M_PI/2.0,0,0;
    campose_wc.fromXYZPRYVector(cam_vec);   // 初始化相机, x顺时针转90度
    pMap->setCameraState(&campose_wc); // 绘制相机位置

    // 创建一个相机
    Matrix3d calib;
    float fx = Config::Get<double>("Camera.fx"); 
    float fy = Config::Get<double>("Camera.fy");
    float cx = Config::Get<double>("Camera.cx");
    float cy = Config::Get<double>("Camera.cy");
    calib << fx, 0, cx,
            0, fy, cy,
            0, 0, 1;
    std::cout << "calib: " << calib << std::endl;

    // 思考 opencv 如何绘制椭圆? 之前完全没有这样做又是什么情况.
    std::cout << "circle param: " << circle.toMinimalVector().transpose() << std::endl;
    Vector5d ellipse_ellipsoid = e.projectOntoImageEllipse(campose_wc.inverse(), calib); // 测试不使用退化椭球体的效果

    // 在图像上绘制该椭圆
    int miImageCols = Config::Get<int>("Camera.width");
    int miImageRows = Config::Get<int>("Camera.height");

    cv::Mat im(miImageRows, miImageCols, CV_8UC3);
    cv::Mat im_3d(miImageRows, miImageCols, CV_8UC3);

    // 初始化一些窗口
    // cv::namedWindow("rgbShow", CV_WINDOW_NORMAL);

    // 交互式测试。
    std::map<double, double, std::less<double>> thetaValueHist;
    while(1){
        // // 刷新图像
        // im.setTo(cv::Scalar(255,255,255));
        // drawEllipseOnImage(ellipse_ellipsoid, im, cv::Scalar(255,0,0));

        double theta = Config::ReadValue<double>("DEBUG.PLANE.WEIGHT") / 100.0 * M_PI * 2;

        // circle = e.GenerateCircle(theta);
        // circle.setColor(Vector3d(1.0,0,0));

        // double delta_theta = M_PI/180.0*30;  // 30度间隔
        // circle2 = e.GenerateCircle(theta+delta_theta);

        // Vector5d ellipse = circle.projectOntoImageEllipse(campose_wc.inverse(), calib); // 测试不使用退化椭球体的效果
        // drawEllipseOnImage(ellipse, im);

        // Vector5d ellipse2 = circle2.projectOntoImageEllipse(campose_wc.inverse(), calib); // 测试不使用退化椭球体的效果
        // drawEllipseOnImage(ellipse2, im);

        // // 染色立体区域
        // Matrix3d ellipseMat = circle.projectOntoImageEllipseMat(campose_wc.inverse(), calib);
        // Matrix3d ellipseMat2 = circle2.projectOntoImageEllipseMat(campose_wc.inverse(), calib);
        // // highlightTextureArea(ellipseMat, ellipseMat2, im);
        
        // cv::imshow("projected", im);
        // cv::waitKey(30);

        // 绘制出对应的三维点
        // test3DPoints(e, ellipse_ellipsoid, campose_wc, calib, pMap);
        // std::cout << "Please check the 3d point of ellipse center!" << std::endl;
        // cv::waitKey();

        // 遍历可视范围内所有点，并输出一张区域图
        // im_3d.setTo(cv::Scalar(255,255,255));
        // Matrix3d ellipseMat_ellipsoid = e.projectOntoImageEllipseMat(campose_wc.inverse(), calib); // 测试不使用退化椭球体的效果
        // highlightTextureAreaIn3D(im_3d, e, ellipseMat_ellipsoid, campose_wc, calib, pMap);

        // cv::Mat areaMat = e.GenerateAreaMat(im_3d.rows, im_3d.cols, campose_wc, calib, pMap);

        // cv::Mat im_3d_area_for_show; // 可视化
        // cv::equalizeHist(areaMat, im_3d_area_for_show);
        // cv::imshow("im_3d_area_for_show", im_3d_area_for_show);

        // 以areaMat为Mask统计区域内的直方图信息
        // calculateHistInAreas(rgb, areaMat);   // [2-7] 并没有调试通过bug，舍弃.

        // 基于 theta 更新椭球体位姿
        g2o::ellipsoid eRot = e.rotate_ellipsoid(theta);
        EllipsoidTex eUpdate(eRot);
        std::cout << "Theta : " << theta << std::endl;
        
        // ***********************************************************
        // *************** 2-7日 提取特征点，并计算其对应点 *************** 
        // ***********************************************************
        Vector4d bbox(200,240,600,440);

        cv::ORB orb;
        vector<cv::KeyPoint> keyPoints;
        cv::Mat bboxMask = GenerateMaskFromBbox(rgb.rows, rgb.cols, bbox);
        orb(rgb, bboxMask, keyPoints);
        std::cout << "Extract orb points: " << keyPoints.size() << std::endl; 
        // 可视化特征点
        cv::Mat rgbShow;
        cv::drawKeypoints(rgb, keyPoints, rgbShow, cv::Scalar(0,0,255));
        cv::rectangle(rgbShow, cv::Rect(cv::Point(bbox(0), bbox(1)), cv::Point(bbox(2), bbox(3))),cv::Scalar(0,255,0));
        
        // 生成keyPoints的三维对称点
        vector<cv::Point2i> points_flected = eUpdate.generateFlectedPoints(toPoint2i(keyPoints), campose_wc, calib);
        // 可视化这些点
        auto keyPointsFlected = toKeyPoints(points_flected);
        cv::drawKeypoints(rgbShow, keyPointsFlected, rgbShow, cv::Scalar(255,0,0));
        // 绘制连线关系
        drawKeypointsPairs(rgbShow, rgbShow, keyPoints, keyPointsFlected, cv::Scalar(0,255,0));
        
        // 可视化椭球体的投影
        Vector5d ellipse_eUpdate = eUpdate.projectOntoImageEllipse(campose_wc.inverse(), calib); // 测试不使用退化椭球体的效果
        drawEllipseOnImage(ellipse_eUpdate, rgbShow, cv::Scalar(255,0,0));

        cv::imshow("rgbShow", rgbShow);

        // 计算点对的像素差, 遍历360度观察其非凸性
        double pixelDiff_average = sumPixelDiff(gray, keyPoints, keyPointsFlected);
        thetaValueHist.insert(std::make_pair(theta, pixelDiff_average));

        // 绘制一个xy散点图
        cv::Mat plot = drawXYPlot(thetaValueHist, 0, M_PI *2, 0, 255);
        cv::imshow("plot", plot);

        // ***********************************************************

        // cv::imshow("projected_3d", im_3d);
        cv::waitKey(30);
    }



    return 0;
}



