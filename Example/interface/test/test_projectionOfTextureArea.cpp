// 新建实验室： TEXT_AREA
// 测试椭球体所构建的三维区域在二维投影上的纹理约束，主要目的在于精确估计转角，并测试其对于综合性能之提升。
// （纹理椭球思路的进化版）

// 该部分为前期功能： 区域的获得，投影，以及在三维、二维可视化。

#include <include/core/Ellipsoid.h>
#include <System.h>
#include <Map.h>
#include <src/config/Config.h>

#include <opencv2/core/core.hpp>

using namespace ORB_SLAM2;
using namespace Eigen;
using namespace std;

typedef Eigen::Matrix<double, 6, 1> Vector6d;


class Circle3D: public g2o::ellipsoid
{
public:
    Circle3D():g2o::ellipsoid(){}
    Circle3D(const g2o::ellipsoid& e):g2o::ellipsoid(e){}
    
};

class EllipsoidTex: public g2o::ellipsoid
{
public:
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

    // 交互式测试。
    while(1){
        // 刷新图像
        im.setTo(cv::Scalar(255,255,255));
        drawEllipseOnImage(ellipse_ellipsoid, im, cv::Scalar(255,0,0));

        double theta = Config::ReadValue<double>("DEBUG.PLANE.WEIGHT") / 100.0 * M_PI * 2;

        circle = e.GenerateCircle(theta);
        circle.setColor(Vector3d(1.0,0,0));

        double delta_theta = M_PI/180.0*30;  // 30度间隔
        circle2 = e.GenerateCircle(theta+delta_theta);

        Vector5d ellipse = circle.projectOntoImageEllipse(campose_wc.inverse(), calib); // 测试不使用退化椭球体的效果
        drawEllipseOnImage(ellipse, im);

        Vector5d ellipse2 = circle2.projectOntoImageEllipse(campose_wc.inverse(), calib); // 测试不使用退化椭球体的效果
        drawEllipseOnImage(ellipse2, im);
        
        cv::imshow("projected", im);
        cv::waitKey(30);
    }



    return 0;
}



