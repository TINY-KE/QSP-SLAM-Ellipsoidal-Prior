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

#include "ObjectDrawer.h"
// #include <pangolin/pangolin.h>

using namespace Eigen;

namespace ORB_SLAM2
{

// draw axis for ellipsoids
void drawAxis()
{
    float length = 2.0;
    
    // x
    glColor3f(1.0,0.0,0.0); // red x
    glBegin(GL_LINES);
    glVertex3f(0.0, 0.0f, 0.0f);
    glVertex3f(length, 0.0f, 0.0f);
    glEnd();

    // y 
    glColor3f(0.0,1.0,0.0); // green y
    glBegin(GL_LINES);
    glVertex3f(0.0, 0.0f, 0.0f);
    glVertex3f(0.0, length, 0.0f);

    glEnd();

    // z 
    glColor3f(0.0,0.0,1.0); // blue z
    glBegin(GL_LINES);
    glVertex3f(0.0, 0.0f ,0.0f );
    glVertex3f(0.0, 0.0f ,length );

    glEnd();
}

ObjectDrawer::ObjectDrawer(Map *pMap, MapDrawer *pMapDrawer, const string &strSettingPath) : mpMap(pMap), mpMapDrawer(pMapDrawer)
{
    cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
    mViewpointF = fSettings["Viewer.ViewpointF"];
    mvObjectColors.push_back(std::tuple<float, float, float>({230. / 255., 0., 0.}));	 // red  0
    mvObjectColors.push_back(std::tuple<float, float, float>({60. / 255., 180. / 255., 75. / 255.}));   // green  1
    mvObjectColors.push_back(std::tuple<float, float, float>({0., 0., 255. / 255.}));	 // blue  2
    mvObjectColors.push_back(std::tuple<float, float, float>({255. / 255., 0, 255. / 255.}));   // Magenta  3
    mvObjectColors.push_back(std::tuple<float, float, float>({255. / 255., 165. / 255., 0}));   // orange 4
    mvObjectColors.push_back(std::tuple<float, float, float>({128. / 255., 0, 128. / 255.}));   //purple 5
    mvObjectColors.push_back(std::tuple<float, float, float>({0., 255. / 255., 255. / 255.}));   //cyan 6
    mvObjectColors.push_back(std::tuple<float, float, float>({210. / 255., 245. / 255., 60. / 255.}));  //lime  7
    mvObjectColors.push_back(std::tuple<float, float, float>({250. / 255., 190. / 255., 190. / 255.})); //pink  8
    mvObjectColors.push_back(std::tuple<float, float, float>({0., 128. / 255., 128. / 255.}));   //Teal  9
    SE3Tcw = Eigen::Matrix4f::Identity();
    SE3TcwFollow = Eigen::Matrix4f::Identity();

    // cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);

    // mKeyFrameSize = fSettings["Viewer.KeyFrameSize"];
    // mKeyFrameLineWidth = fSettings["Viewer.KeyFrameLineWidth"];
    // mGraphLineWidth = fSettings["Viewer.GraphLineWidth"];
    // mPointSize = fSettings["Viewer.PointSize"];
    // mCameraSize = fSettings["Viewer.CameraSize"];
    mCameraLineWidth = fSettings["Viewer.CameraLineWidth"];

    mbOpenTransform = false;
}

void ObjectDrawer::SetRenderer(ObjectRenderer *pRenderer)
{
    mpRenderer = pRenderer;
}

void ObjectDrawer::AddObject(MapObject *pMO)
{
    unique_lock<mutex> lock(mMutexObjects);
    mlNewMapObjects.push_back(pMO);
}

void ObjectDrawer::ProcessNewObjects()
{
    unique_lock<mutex> lock(mMutexObjects);
    auto pMO = mlNewMapObjects.front();
    // std::cout << "mlNewMapObjects.size() = " << mlNewMapObjects.size() << std::endl;
    // todo: 这里只处理了最新添加的一个物体
    if (pMO)
    {
        int renderId = (int) mpRenderer->AddObject(pMO->vertices, pMO->faces);
        pMO->SetRenderId(renderId);
        mlNewMapObjects.pop_front();
    }
    else {
        // std::cout << "!pMO" << std::endl;
    }
}

void ObjectDrawer::DrawObjects(bool bFollow, const Eigen::Matrix4f &Tec, double prob_thresh, float pointcloudSize)
{
    unique_lock<mutex> lock(mMutexObjects);

    auto mvpMapObjects = mpMap->GetAllMapObjects();

    // std::cout << "mvpMapObjects.size() = " << mvpMapObjects.size() << std::endl;

    for (MapObject *pMO : mvpMapObjects)
    {
        if (!pMO) {
            // std::cout << "!pMO" << std::endl;
            continue;
        }
        if (pMO->isBad()) {
            // std::cout << "pMO->isBad()" << std::endl;
            continue;
        }

        Eigen::Matrix4f Sim3Two = pMO->GetPoseSim3();

        int idx = pMO->GetRenderId();
        
        if (bFollow) {
            SE3TcwFollow = SE3Tcw;
        }
        if (pMO->GetRenderId() >= 0)
        {
            mpRenderer->Render(idx, Tec * SE3TcwFollow * Sim3Two, mvObjectColors[pMO->GetRenderId() % mvObjectColors.size()]);
        }

        DrawCuboid(pMO);

        // 程序的停滞与这里的椭球体获取有关
        // auto obj_ellipsold = pMO->GetEllipsold();

        // if(obj_ellipsold) {
        //     // cout << "obj_ellipsold->prob = " << obj_ellipsold->prob << endl;
        //     if(obj_ellipsold->prob > prob_thresh){
        //         mpMapDrawer->drawEllipsoidInVector(obj_ellipsold);
        //     }
        // }

        // std::shared_ptr<PointCloud> mPointsPtr = pMO->GetPointCloud();
        // PointCloud* rawPtr = mPointsPtr.get();
        // mpMapDrawer->drawPointCloud(rawPtr, pointcloudSize);

    }
}

// // In : Tcw
// // Out: Twc
// void ObjectDrawer::SE3ToOpenGLCameraMatrix(g2o::SE3Quat &matInSe3, pangolin::OpenGlMatrix &M)
// {
//     // eigen to cv
//     Eigen::Matrix4d matEigen = matInSe3.to_homogeneous_matrix();
//     cv::Mat matIn;

//     matIn = Converter::toCvMat(matEigen);

//     // std::cout << "matIn = " << matIn << std::endl;
//     // cv::eigen2cv(matEigen, matIn);

//     if(!matIn.empty())
//     {
//         cv::Mat Rwc(3,3,CV_64F);
//         cv::Mat twc(3,1,CV_64F);

//         {
//             // unique_lock<mutex> lock(mMutexCamera);
//             Rwc = matIn.rowRange(0,3).colRange(0,3).t();
//             twc = -Rwc*matIn.rowRange(0,3).col(3);
//         }

//         // 原来是 double, 发现有问题

//         M.m[0] = Rwc.at<float>(0,0);
//         M.m[1] = Rwc.at<float>(1,0);
//         M.m[2] = Rwc.at<float>(2,0);
//         M.m[3]  = 0.0;

//         M.m[4] = Rwc.at<float>(0,1);
//         M.m[5] = Rwc.at<float>(1,1);
//         M.m[6] = Rwc.at<float>(2,1);
//         M.m[7]  = 0.0;

//         M.m[8] = Rwc.at<float>(0,2);
//         M.m[9] = Rwc.at<float>(1,2);
//         M.m[10] = Rwc.at<float>(2,2);
//         M.m[11]  = 0.0;

//         M.m[12] = twc.at<float>(0);
//         M.m[13] = twc.at<float>(1);
//         M.m[14] = twc.at<float>(2);
//         M.m[15]  = 1.0;
//     }
//     else
//         M.SetIdentity();
// }

// void ObjectDrawer::drawEllipsoidsInVector(ellipsoid* e)
// {
//     // std::cout << "[MapDrawer::drawAllEllipsoidsInVector] " \
//     //     << "ellipsoids.size() = " << ellipsoids.size() << std::endl;
    
//     SE3Quat TmwSE3 = e->pose.inverse();

//     if(mbOpenTransform)
//         TmwSE3 = (mTge * e->pose).inverse(); // Tem

//     Vector3d scale = e->scale;

//     // std::cout << "TmwSE3 = " << TmwSE3.to_homogeneous_matrix().matrix() << std::endl;
//     // std::cout << "Ellipsoid scale = " << scale.transpose().matrix() << std::endl; 

//     glPushMatrix();

//     glLineWidth(mCameraLineWidth/3.0);

//     if(e->isColorSet()){
//         Vector4d color = e->getColorWithAlpha();
//         // std::cout << "color = " << color.matrix() << std::endl;
//         glColor4d(color(0),color(1),color(2),color(3));
//     }
//     else
//         glColor3f(0.0f,0.0f,1.0f);

//     GLUquadricObj *pObj;
//     pObj = gluNewQuadric();
//     gluQuadricDrawStyle(pObj, GLU_LINE);

//     pangolin::OpenGlMatrix Twm;   // model to world

//     // SE3ToOpenGLCameraMatrix(TmwSE3, Twm);

//     glMultMatrixd(Twm.m);  
//     glScaled(scale[0],scale[1],scale[2]);
//     gluSphere(pObj, 1.0, 26, 13); // draw a sphere with radius 1.0, center (0,0,0), slices 26, and stacks 13.

//     drawAxis();
//     glPopMatrix();
//     return;

// }

void ObjectDrawer::DrawCuboid(MapObject *pMO)
{
    const float w = pMO->w / 2;
    const float h = pMO->h / 2;
    const float l = pMO->l / 2;

    glPushMatrix();



    pangolin::OpenGlMatrix Two = Converter::toMatrixPango(pMO->SE3Two);
#ifdef HAVE_GLES
    glMultMatrixf(Two.m);
#else
    glMultMatrixd(Two.m);
#endif

    const float mCuboidLineWidth = 3.0;
    glLineWidth(mCuboidLineWidth);
    glColor3f(0.0f,1.0f,0.0f);
    glBegin(GL_LINES);

    glVertex3f(w,h,l);
    glVertex3f(w,-h,l);

    glVertex3f(-w,h,l);
    glVertex3f(-w,-h,l);

    glVertex3f(-w,h,l);
    glVertex3f(w,h,l);

    glVertex3f(-w,-h,l);
    glVertex3f(w,-h,l);

    glVertex3f(w,h,-l);
    glVertex3f(w,-h,-l);

    glVertex3f(-w,h,-l);
    glVertex3f(-w,-h,-l);

    glVertex3f(-w,h,-l);
    glVertex3f(w,h,-l);

    glVertex3f(-w,-h,-l);
    glVertex3f(w,-h,-l);

    glVertex3f(w,h,-l);
    glVertex3f(w,h,l);

    glVertex3f(-w,h,-l);
    glVertex3f(-w,h,l);

    glVertex3f(-w,-h,-l);
    glVertex3f(-w,-h,l);

    glVertex3f(w,-h,-l);
    glVertex3f(w,-h,l);

    glEnd();

    DrawFrame(w, h, l);

    glPopMatrix();
}

void ObjectDrawer::SetCurrentCameraPose(const Eigen::Matrix4f &Tcw)
{
    unique_lock<mutex> lock(mMutexObjects);
    SE3Tcw = Tcw;
}

void ObjectDrawer::DrawFrame(const float w, const float h, const float l)
{
    glLineWidth(3);
    glBegin(GL_LINES);
    glColor3f(1.0, 0.0, 0.0);
    glVertex3f(0.0, 0.0, 0.0);
    glVertex3f(2*w, 0.0, 0.0);
    glColor3f(0.0, 1.0, 0.0);
    glVertex3f(0.0, 0.0, 0.0);
    glVertex3f(0.0, 2*h, 0.0);
    glColor3f(0.0, 0.0, 1.0);
    glVertex3f(0.0, 0.0, 0.0);
    glVertex3f(0.0, 0.0, 2*l);
    glEnd();
}

}

