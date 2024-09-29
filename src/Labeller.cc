/**
 * 对点云/网格面片进行物体真值标注
 * 标注的合理顺序：
 * 0. 标注地面真值，获得物体
 * 1. 首先粗略选定物体中心点
 * 2. 根据地面真值能够得到物体的俯仰角与滚转角，仅需对偏航角进行标注
 * 3. 调整物体的长宽高属性
 * 4. 截取出物体的单独点云/网格，用于后续的物体形状准确性评估
*/

#include "Labeller.h"

#include <pybind11/embed.h>
#include <pybind11/eigen.h>

#include <opencv2/core/core.hpp>
#include <pangolin/pangolin.h>
#include <pangolin/geometry/geometry.h>
// #include <pangolin/geometry/geometry_ply.h>

#include <mutex>
#include <unistd.h>

#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>

#include "src/symmetry/PointCloudFilter.h"
#include "Converter.h"
#include "include/utils/matrix_utils.h"

using namespace std;

Labeller::Labeller(const std::string &strSettingPath):
    mbFinishRequested(false), mbFinished(true), mbStopped(true), mbStopRequested(false)
{
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);

    mViewpointX = fSettings["Viewer.ViewpointX"];
    mViewpointY = fSettings["Viewer.ViewpointY"];
    mViewpointZ = fSettings["Viewer.ViewpointZ"];
    mViewpointF = fSettings["Viewer.ViewpointF"];
}


void Labeller::Run()
{
    mbFinished = false;
    mbStopped = false;
    int w = 1024;
    int h = 576;


    // 创建名称为“Main”的GUI窗口，尺寸为640×640
    pangolin::CreateWindowAndBind("Labelling tool",w,h);
    // 启动深度测试
    glEnable(GL_DEPTH_TEST);

    // Issue specific OpenGl we might need
    glEnable (GL_BLEND);
    glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    // pangolin::CreatePanel("menu").SetBounds(0.0,1.0,0.0,pangolin::Attach::Pix(175));
    pangolin::CreatePanel("menu").SetBounds(0.0,0.8,0.0,pangolin::Attach::Pix(175));

    // pangolin::CreatePanel("menu").SetBounds(0.0,0.7,0.4,pangolin::Attach::Pix(300));

    pangolin::Var<std::string> menuInput("Text Input", "Type here...", false);

    // pangolin::Var<bool> menuFollowCamera("menu.Follow Camera",false,true);

    /**
     * 设置调节俯仰、滚转、偏航角，x,y,z坐标和长宽高的滑钮
    */
    
    pangolin::Var<bool> menuFinish("menu.Exit",false,false);
    pangolin::Var<bool> menuResetView("menu.ResetView",false,false);
    
    pangolin::Var<float> SliderRoll("menu.Roll", 0.0, 0.0, M_PI);
    pangolin::Var<float> SliderPitch("menu.Pitch", 0.0, 0.0, M_PI);
    pangolin::Var<float> SliderYaw("menu.Yaw", 0.0, 0.0, M_PI);

    pangolin::Var<float> SliderTvecX("menu.TvecX", 0.0, -10, 10);
    pangolin::Var<float> SliderTvecY("menu.TvecY", 0.0, -10, 10);
    pangolin::Var<float> SliderTvecZ("menu.TvecZ", 0.0, -10, 10);

    pangolin::Var<float> SliderScaleW("menu.ScaleW", 0.8, 0.001, 2);
    pangolin::Var<float> SliderScaleH("menu.ScaleH", 0.8, 0.001, 2);
    pangolin::Var<float> SliderScaleL("menu.ScaleL", 0.8, 0.001, 2);


    // Define Camera Render Object (for view / scene browsing)
    pangolin::OpenGlRenderState s_cam(
                pangolin::ProjectionMatrix(w,h, mViewpointF,mViewpointF,w / 2,h / 2,0.1,5000),
                pangolin::ModelViewLookAt(mViewpointX,mViewpointY,mViewpointZ, 0,0,0,0.0,-1.0, 0.0)
                );

    // Add named OpenGL viewport to window and provide 3D Handler
    pangolin::View& d_cam = pangolin::CreateDisplay()
            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -float(w) / float(h))
            .SetHandler(new pangolin::Handler3D(s_cam));

    /**
    * 读取点云/网格文件
    */

    pangolin::OpenGlMatrix Twc;
    Twc.SetIdentity();

    Eigen::Matrix4f Tec;


    while( !pangolin::ShouldQuit() )
    {
        // 清空颜色和深度缓存
        glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
        d_cam.Activate(s_cam);

        if(menuResetView){
            s_cam.Follow(Twc);
            menuResetView = false;
        }

        glClearColor(1.0f,1.0f,1.0f,1.0f);

    //     // Used for object drawer
    //     Tec = s_cam.GetModelViewMatrix();
    //     Tec.row(1) = -Tec.row(1);
    //     Tec.row(2) = -Tec.row(2);
    //     glClearColor(1.0f,1.0f,1.0f,1.0f);


    //     // cout << "menuFinish = " << menuFinish << endl;

        if(menuFinish) {
            SetFinish();
            menuFinish = false;
            break;
        }

        /**
         * 绘制点云/网格文件
         * 
        */
        glPushMatrix();

        int pointSize = 2;

        for(auto pPoints:mmPointCloudLists){
            if( pPoints == NULL ) continue;
            for(int i=0; i < pPoints->size(); i=i+1)
            {
                ORB_SLAM2::PointXYZRGB &p = (*pPoints)[i];
                glPointSize( pointSize );
                // glPointSize( p.size );
                glBegin(GL_POINTS);
                glColor3d(p.r/255.0, p.g/255.0, p.b/255.0);
                glVertex3d(p.x, p.y, p.z);
                glEnd();

            }
        }
        glPointSize( pointSize );
        glPopMatrix();



        /**
         * 根据设置的物体属性绘制3D外包框
        */
        float roll = SliderRoll, pitch = SliderPitch, yaw = SliderYaw;
        float x = SliderTvecX, y = SliderTvecY, z = SliderTvecZ;
        float w = SliderScaleW, h = SliderScaleH, l = SliderScaleL;
        Eigen::Matrix4f Two;
        Eigen::Vector3f Scale;

        Two = Eigen::Matrix4f::Identity();
        // Eigen::Matrix3f Rwo = euler_zyx_to_rot(roll, pitch, yaw);

        Eigen::Matrix3f rotation_matrix;

        // 计算绕 X 轴的旋转矩阵
        Eigen::Matrix3f rotation_x;
        rotation_x << 1, 0, 0,
                    0, cos(roll), -sin(roll),
                    0, sin(roll), cos(roll);

        // 计算绕 Y 轴的旋转矩阵
        Eigen::Matrix3f rotation_y;
        rotation_y << cos(pitch), 0, sin(pitch),
                    0, 1, 0,
                    -sin(pitch), 0, cos(pitch);

        // 计算绕 Z 轴的旋转矩阵
        Eigen::Matrix3f rotation_z;
        rotation_z << cos(yaw), -sin(yaw), 0,
                    sin(yaw), cos(yaw), 0,
                    0, 0, 1;

        // 合成旋转矩阵
        rotation_matrix = rotation_z * rotation_y * rotation_x;
        

        // // Two.topLeftCorner<3,3>() = euler_zyx_to_rot(roll, pitch, yaw);
        // Two.topLeftCorner<3,3>() = Rwo.inverse();
        // Two.topLeftCorner<3,3>() = rotation_matrix.inverse();

        Two.topLeftCorner<3,3>() = rotation_matrix;



        // 此处应该计算固定角而非欧拉角

        Two.topRightCorner<3,1>() = Eigen::Vector3f(x,y,z);
        Scale = Eigen::Vector3f(w,h,l);


        DrawCuboid(Two, Scale);

    //     // if(CheckFinish())
    //     //     break;

        // // 在原点绘制一个立方体
        // pangolin::glDrawColouredCube();

        Eigen::Matrix4f T_frame = Eigen::Matrix4f::Identity();

        DrawFrame(8,8,8,T_frame);

        // // 绘制网格
        // glColor3f(0.0, 1.0, 1.0); // 设置网格颜色为白色
        // glBegin(GL_TRIANGLES);


        // // glVertex3d(0, 0, 0);
        // // glVertex3d(0, 1, 0);
        // // glVertex3d(0, 0, 1);

        // // glEnd();

        // // cout << "mmMeshLists.size() = " << mmMeshLists.size() << endl;

        // auto mesh = mmMeshLists[0];

        // // for (auto& mesh: mmMeshLists){
        // //     // cout << "mesh.polygons.size() = " << mesh.polygons.size() << endl;
        // //     for (size_t i = 0; i < mesh.polygons.size(); ++i) {
        // //         // polygon 中记录的是face情况
        // //         // glVertex3d(mesh.polygons[i].vertices[0], mesh.polygons[i].vertices[1], mesh.polygons[i].vertices[2]);

        // //         pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        // //         fromPCLPointCloud2(mesh.cloud, *cloud);

        // //         for (size_t j = 0; j < mesh.polygons[i].vertices.size(); ++j) {
        // //             auto pt_id = mesh.polygons[i].vertices[j];

        // //             auto pp = (*cloud)[pt_id];

        // //             glVertex3d((*cloud)[pt_id].x, (*cloud)[pt_id].y, (*cloud)[pt_id].z);

        // //             // std::cout << (*cloud)[pt_id].x << ", " << (*cloud)[pt_id].y << ", " << (*cloud)[pt_id].z << std::endl;
        // //         }
                
        // //         // pcl::PCLPointCloud2

        // //         // for (size_t j = 0; j < mesh.polygons[i].vertices.size(); ++j) {

        // //         //     auto vertice = mesh.polygons[i].vertices[j];
        // //         //     // pcl::PointXYZRGB point = mesh.cloud.points[mesh.polygons[i].vertices[j]];
        // //         //     // pcl::PointXYZRGB point = mesh.cloud.data[mesh.cloud.point_step * mesh.polygons[i].vertices[j]];
                    
        // //         // }
        // //     }
        // // }


        // // glVertex3d(0, 0, 0);
        // // glVertex3d(0, 1, 0);
        // // glVertex3d(0, 0, 1);

        // glEnd();


        // 运行帧循环以推进窗口事件
        pangolin::FinishFrame();

        if (CheckFinish()){
            break;
        }
    }

    std::cout << "Finish" << std::endl;

    SetFinish();
}

// PointCloud
// 注意添加点云的操作要在Run前完成
void Labeller::ReadPCDFile(const std::string &pcd_filepath)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    // PointCloudPCL::Ptr
    if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(pcd_filepath, *cloud) == -1) {
        PCL_ERROR("Couldn't read file rabbit.pcd\n");
        //return(-1);
    }
    else{

        ORB_SLAM2::PointCloud* pCloud = pclToQuadricPointCloudPtr(cloud);
        mmPointCloudLists.push_back(pCloud);
    }
}

// Mesh
void Labeller::ReadPLYFile(const std::string &ply_filepath)
{
    // auto mesh_pangolin = pangolin::LoadGeometry("ply_filepath");

    
    // 读取PLY文件
    pcl::PolygonMesh mesh;
    if (pcl::io::loadPLYFile(ply_filepath, mesh) == -1) {
        PCL_ERROR("Couldn't read file.\n");
    }
    else{
        mmMeshLists.push_back(mesh);
    }
}

void Labeller::DrawFrame(float w, float h, float l, Eigen::Matrix4f Two)
{
    // 绘制坐标系
    glLineWidth(3);
    glBegin ( GL_LINES );
    glColor3f ( 0.8f,0.f,0.f );

    // glVertex3f( -1,-1,-1 );
    // glVertex3f( 0,-1,-1 );

    glVertex3f( 0,0,0 );
    glVertex3f( w,0,0 );

    glColor3f( 0.f,0.8f,0.f);
    glVertex3f( 0,0,0 );
    glVertex3f( 0,h,0 );

    glColor3f( 0.2f,0.2f,1.f);
    glVertex3f( 0,0,0 );
    glVertex3f( 0,0,l );
    glEnd();
}

void Labeller::DrawCuboid(Eigen::Matrix4f Two, Eigen::Vector3f Scale, bool with_mesh)
{
    const float w = Scale(0) / 2;
    const float h = Scale(1) / 2;
    const float l = Scale(2) / 2;

    glPushMatrix();

    pangolin::OpenGlMatrix Two_opengl = ORB_SLAM2::Converter::toMatrixPango(Two);


#ifdef HAVE_GLES
    glMultMatrixf(Two_opengl.m);
#else
    glMultMatrixd(Two_opengl.m);
#endif

    const float mCuboidLineWidth = 3.0;
    glLineWidth(mCuboidLineWidth);
    glColor3f(0.0f,1.0f,0.0f);
    glBegin(GL_LINES);


    vector<vector<float>> pts = {
        {w,h,l}, {w,-h,l}, {-w,-h,l}, {-w,h,l}, {w,h,-l}, {w,-h,-l}, {-w,-h,-l}, {-w,h,-l}
    };

    vector<vector<int>> line_pairs = {
        {0,1},{1,2},{2,3},{3,0},{4,5},{5,6},{6,7},{7,4},{0,4},{1,5},{2,6},{3,7}
    };

    vector<vector<int>> mesh_pairs = {
        // {0,2,1},{0,3,2},{5,4,1},{4,0,1},
        // {6,4,5},{6,7,4},{2,3,6},{3,7,6},
        {0,4,7},{7,3,0},{1,2,6},{1,6,5}
    };

    for(auto &pair: line_pairs){
        vector<float> p1 = pts[pair[0]], p2 = pts[pair[1]];
        glVertex3f(p1[0],p1[1],p1[2]);
        glVertex3f(p2[0],p2[1],p2[2]);
    }

    glEnd();

    // 绘制网格
    glColor3f(0.0, 1.0, 1.0); // 设置网格颜色为青色
    glBegin(GL_TRIANGLES);

    for(auto &pair: mesh_pairs){
        // for (int i = 0; i < 3; i++) {
        for (int i = 2; i >= 0; i--) {
            glVertex3f(pts[pair[i]][0],pts[pair[i]][1],pts[pair[i]][2]);
        }
    }

    // glVertex3d(0, 0, 0);
    // glVertex3d(0, 1, 0);
    // glVertex3d(0, 0, 1);

    glEnd();

    DrawFrame(2*w, 2*h, 2*l);

    glPopMatrix();

}

bool Labeller::CheckFinish(){
    unique_lock<mutex> lock(mMutexFinish);
    return mbFinishRequested;
};

void Labeller::SetFinish(){
    unique_lock<mutex> lock(mMutexFinish);
    mbFinished = true;
};