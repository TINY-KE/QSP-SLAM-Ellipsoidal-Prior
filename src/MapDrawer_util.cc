/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#include "MapDrawer.h"
#include <pangolin/pangolin.h>
#include <mutex>

namespace ORB_SLAM2
{

    // draw axis for ellipsoids
    void drawAxisNormal()
    {
        float length = 2.0;

        glLineWidth(2);
        
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

void MapDrawer::drawEllipsoid() {
    // SE3Quat TmwSE3 = ellipsoids[i]->pose.inverse();
    // Vector3d scale = ellipsoids[i]->scale;

    glPushMatrix();

    glLineWidth(mCameraLineWidth*3/4.0);

    // if(ellipsoids[i]->isColorSet()){
    //     Vector4d color = ellipsoids[i]->getColorWithAlpha();
    //     glColor4f(color(0),color(1),color(2),color(3));
    // }
    // else
    //     glColor3f(0.0f,0.0f,1.0f);

    GLUquadricObj *pObj;
    pObj = gluNewQuadric();
    gluQuadricDrawStyle(pObj, GLU_LINE);

    pangolin::OpenGlMatrix Twm;   // model to world
    // SE3ToOpenGLCameraMatrix(TmwSE3, Twm);

    Eigen::Vector3d scale(0.5,0.5,0.5);
    Twm.SetIdentity();

    glMultMatrixd(Twm.m);  
    glScaled(scale[0],scale[1],scale[2]);

    gluSphere(pObj, 1.0, 26, 13); // draw a sphere with radius 1.0, center (0,0,0), slices 26, and stacks 13.
    drawAxisNormal();

    glPopMatrix();

    // return true;
}

// draw ellipsoids
bool MapDrawer::drawEllipsoids(double prob_thresh) {
    std::vector<ellipsoid*> ellipsoids = mpMap->GetAllEllipsoids();
    int num_origin = ellipsoids.size();

    std::vector<ellipsoid*> ellipsoidsVisual = mpMap->GetAllEllipsoidsVisual();
    ellipsoids.insert(ellipsoids.end(), ellipsoidsVisual.begin(), ellipsoidsVisual.end());

    // filter those ellipsoids with prob
    std::vector<ellipsoid*> ellipsoids_prob;
    for(auto& pE : ellipsoids)
    {
        if(pE->prob > prob_thresh )
            ellipsoids_prob.push_back(pE);
    }

    // std::cout << "[MapDrawer::drawEllipsoids] "
        // << "ellipsoids.size() = " << ellipsoids.size() << std::endl;
    if(!ellipsoids.empty()){
        auto last_ellipsoid = ellipsoids[ellipsoids.size()-1];
        // std::cout << "[MapDrawer::drawEllipsoids]  Scale of last ellipsoid: " << last_ellipsoid->scale.transpose() << std::endl;
    }
    

    drawAllEllipsoidsInVector(ellipsoids_prob, 4);

    return true;
}

bool MapDrawer::drawEllipsoidsObjects(double prob_thresh){
    std::vector<ellipsoid*> ellipsoids;
    // std::vector<ellipsoid*> ellipsoids = mpMap->GetAllEllipsoidsObjects();
    // int num_origin = ellipsoids.size();

    std::vector<ellipsoid*> ellipsoidsVisual = mpMap->GetAllEllipsoidsObjects();
    ellipsoids.insert(ellipsoids.end(), ellipsoidsVisual.begin(), ellipsoidsVisual.end());

    // filter those ellipsoids with prob
    std::vector<ellipsoid*> ellipsoids_prob;
    for(auto& pE : ellipsoids)
    {
        // if(pE->prob > prob_thresh )
            ellipsoids_prob.push_back(pE);
    }
    
    drawAllEllipsoidsInVector(ellipsoids_prob, 2);

    return true;
}

// 加入了 transform
void MapDrawer::drawAllEllipsoidsInVector(std::vector<ellipsoid*>& ellipsoids, int color_mode)
{
    // std::cout << "[MapDrawer::drawAllEllipsoidsInVector] " \
    //     << "ellipsoids.size() = " << ellipsoids.size() << std::endl;
    
    for( size_t i=0; i<ellipsoids.size(); i++)
    {
        drawEllipsoidInVector(ellipsoids[i], color_mode);
    }
    return;
}

void MapDrawer::drawEllipsoidInVector(ellipsoid* e, int color_mode)
{
    
    SE3Quat TmwSE3 = e->pose.inverse();

    if(mbOpenTransform)
        TmwSE3 = (mTge * e->pose).inverse(); // Tem

    Vector3d scale = e->scale;

    // std::cout << "TmwSE3 = " << TmwSE3.to_homogeneous_matrix().matrix() << std::endl;
    // std::cout << "Ellipsoid scale = " << scale.transpose().matrix() << std::endl; 

    glPushMatrix();

    glLineWidth(mCameraLineWidth/3.0);

    // glColor3f(0.0f,0.0f,1.0f);
    if (color_mode == 0)
        glColor3f(1.0f,0.0f,0.0f);  // BGR
    else if (color_mode == 1)
        glColor3f(0.0f,1.0f,0.0f);
    else if(color_mode == 2)
        glColor3f(0.0f,0.0f,1.0f);
    else if(e->isColorSet()){
        Vector4d color = e->getColorWithAlpha();
        // std::cout << "color = " << color.matrix() << std::endl;
        glColor4d(color(0),color(1),color(2),color(3));
    }
    else
        glColor3f(0.0f,0.0f,1.0f);
    
    GLUquadricObj *pObj;
    pObj = gluNewQuadric();
    gluQuadricDrawStyle(pObj, GLU_LINE);

    pangolin::OpenGlMatrix Twm;   // model to world

    SE3ToOpenGLCameraMatrix(TmwSE3, Twm);

    glMultMatrixd(Twm.m);  
    glScaled(scale[0],scale[1],scale[2]);
    gluSphere(pObj, 1.0, 26, 13); // draw a sphere with radius 1.0, center (0,0,0), slices 26, and stacks 13.

    drawAxisNormal();

    glPopMatrix();
}

bool MapDrawer::drawObservationEllipsoids(double prob_thresh)
{
    std::vector<ellipsoid*> ellipsoidsObservation = mpMap->GetObservationEllipsoids();

    // filter those ellipsoids with prob
    std::vector<ellipsoid*> ellipsoids_prob;
    for(auto& pE : ellipsoidsObservation)
    {
        if(pE->prob > prob_thresh )
            ellipsoids_prob.push_back(pE);
    }

    drawAllEllipsoidsInVector(ellipsoids_prob, 4);
    return true;
}

    // draw all the planes
bool MapDrawer::drawPlanes(int visual_group) {
    std::vector<plane*> planes = mpMap->GetAllPlanes();
    // std::cout << "plane_num = " << planes.size() << std::endl;
    for( size_t i=0; i<planes.size(); i++) {
        g2o::plane* ppl = planes[i];
        if(ppl->miVisualGroup == visual_group) {
            // std::cout << "drawPlaneWithEquation : " << ppl->param.transpose().matrix() << std::endl;
            drawPlaneWithEquation(ppl);
        }
    }

    return true;
}

// In : Tcw
// Out: Twc
void MapDrawer::SE3ToOpenGLCameraMatrix(g2o::SE3Quat &matInSe3, pangolin::OpenGlMatrix &M)
{
    // eigen to cv
    Eigen::Matrix4d matEigen = matInSe3.to_homogeneous_matrix();
    cv::Mat matIn;

    matIn = Converter::toCvMat(matEigen);

    // std::cout << "matIn = " << matIn << std::endl;
    // cv::eigen2cv(matEigen, matIn);

    if(!matIn.empty())
    {
        cv::Mat Rwc(3,3,CV_64F);
        cv::Mat twc(3,1,CV_64F);

        {
            unique_lock<mutex> lock(mMutexCamera);
            Rwc = matIn.rowRange(0,3).colRange(0,3).t();
            twc = -Rwc*matIn.rowRange(0,3).col(3);
        }

        // 原来是 double, 发现有问题

        M.m[0] = Rwc.at<float>(0,0);
        M.m[1] = Rwc.at<float>(1,0);
        M.m[2] = Rwc.at<float>(2,0);
        M.m[3]  = 0.0;

        M.m[4] = Rwc.at<float>(0,1);
        M.m[5] = Rwc.at<float>(1,1);
        M.m[6] = Rwc.at<float>(2,1);
        M.m[7]  = 0.0;

        M.m[8] = Rwc.at<float>(0,2);
        M.m[9] = Rwc.at<float>(1,2);
        M.m[10] = Rwc.at<float>(2,2);
        M.m[11]  = 0.0;

        M.m[12] = twc.at<float>(0);
        M.m[13] = twc.at<float>(1);
        M.m[14] = twc.at<float>(2);
        M.m[15]  = 1.0;
    }
    else
        M.SetIdentity();
}

// from EllipsoidExtractor::calibRotMatAccordingToGroundPlane
Eigen::Matrix3d calibRotMatAccordingToAxis(Matrix3d& rotMat, const Vector3d& normal){
    // in order to apply a small rotation to align the z axis of the object and the normal vector of the groundplane,
    // we need calculate the rotation axis and its angle.

    // first get the rotation axis
    Vector3d ellipsoid_zAxis = rotMat.col(2);
    Vector3d rot_axis = ellipsoid_zAxis.cross(normal); 
    if(rot_axis.norm()>0)
        rot_axis.normalize();

    // then get the angle between the normal of the groundplane and the z axis of the object
    double norm1 = normal.norm();
    double norm2 = ellipsoid_zAxis.norm();
    double vec_dot = normal.transpose() * ellipsoid_zAxis;
    double cos_theta = vec_dot/norm1/norm2;
    double theta = acos(cos_theta);     

    // generate the rotation vector
    AngleAxisd rot_angleAxis(theta,rot_axis);

    Matrix3d rotMat_calibrated = rot_angleAxis * rotMat;

    return rotMat_calibrated;
}

// A sparse version.
void MapDrawer::drawPlaneWithEquation(plane *p) {
    if( p == NULL ) return;
    Vector3d center;            // 平面上一点!!
    double size;
    
    Vector3d color = p->color;
    Vector3d normal = p->normal(); 
    if(normal.norm()>0)
        normal.normalize();
    if(!p->mbLimited)
    {
        // an infinite plane, us default size
        center = p->SampleNearAnotherPoint(Vector3d(0,0,0));
        size = 25;
    }
    else
    {
        // todo: this make only ground drew
        // std::cout << "return because p->mbLimited" << std::endl;
        // return;
        size = p->mdPlaneSize;
        center = p->SampleNearAnotherPoint(p->mvPlaneCenter);
    }

    // draw the plane
    Matrix3d rotMat = Matrix3d::Identity();
    // 将其z轴旋转到 normal 方向.
    Matrix3d rotMatCalib = calibRotMatAccordingToAxis(rotMat, normal);

    Vector3d basis_x = rotMatCalib.col(0);
    Vector3d basis_y = rotMatCalib.col(1);

    // const Vector3d v1(center - (basis_x * size) - (basis_y * size));
    // const Vector3d v2(center + (basis_x * size) - (basis_y * size));
    // const Vector3d v3(center + (basis_x * size) + (basis_y * size));
    // const Vector3d v4(center - (basis_x * size) + (basis_y * size));

    // // Draw wireframe plane quadrilateral:

    // // 外轮廓.??
    // drawLine(v1, v2, color, line_width);
    // drawLine(v2, v3, color, line_width);
    // drawLine(v3, v4, color, line_width);
    // drawLine(v4, v1, color, line_width);

    // 绘制内部线条.
    Vector3d point_ld = center - size/2.0 * basis_x - size/2.0 * basis_y;

    double line_width = 2.0;
    double alpha = 0.8;
    // int sample_num = 15; // 格子数量
    int sample_num = max(15, (int)(size/0.5)); // 格子数量
    double sample_dis = size / sample_num;
    for(int i=0;i<sample_num+1;i++)
    {
        // 从起始到结束, 包含起始和结束的等距离sample
        Vector3d v1(point_ld + i*sample_dis*basis_x);
        Vector3d v2(v1 + size*basis_y);
        drawLine(v1, v2, color, line_width, alpha);
    }
    for(int i=0;i<sample_num+1;i++)
    {
        // 从起始到结束, 包含起始和结束的等距离sample
        Vector3d v1(point_ld + i*sample_dis*basis_y);
        Vector3d v2(v1 + size*basis_x);
        drawLine(v1, v2, color, line_width, alpha);
    }

    bool bDrawDirection = true; // 绘制法向量方向
    double direction_length = size / 3;
    if(bDrawDirection)
    {
        Vector3d end_point = center + normal * direction_length;
        drawLine(center, end_point, color/2.0, line_width/1.5, alpha);

        Vector3d end_point2 = end_point - normal * (direction_length / 4);
        drawLine(end_point, end_point2, color*2.0, line_width*2, alpha);// 绘制末端
    }

    return;
}

void MapDrawer::drawLine(const Vector3d& start, const Vector3d& end, const Vector3d& color, double width, double alpha)
{
    glLineWidth(width);

    glPushMatrix();

    glColor4d(color[0], color[1], color[2], alpha);
    // 先tm画很粗的 Line 吧.
    glBegin(GL_LINES);
    // opengl 画箭头.
    glVertex3d(start[0], start[1], start[2]);
    glVertex3d(end[0], end[1], end[2]);
    glEnd();

    glPopMatrix();
}

void MapDrawer::drawPointCloudLists(float pointSize)
{
    auto pointLists = mpMap->GetPointCloudList();

    // cout << "pointLists.size() = " << pointLists.size() << std::endl;

    glPushMatrix();

    for(auto pair:pointLists){
        auto strpoints = pair.first;
        // std::cout << "strpoints = " << strpoints << std::endl;
        auto pPoints = pair.second;
        if( pPoints == NULL ) continue;
        for(int i=0; i<pPoints->size(); i=i+1)
        {
            PointXYZRGB &p = (*pPoints)[i];
            // std::cout << "pPoints->size() = " << pPoints->size() << std::endl;
            // std::cout << "(&p) == NULL = " << ((&p)==NULL ) << std::endl;
            // std::cout << "p.x = " << p.x << std::endl;

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
}

void MapDrawer::drawPointCloudLists(float pointSize, std::string pcd_name)
{
    auto pointLists = mpMap->GetPointCloudList();

    // cout << "pointLists.size() = " << pointLists.size() << std::endl;

    glPushMatrix();

    for(auto pair:pointLists){
        auto strpoints = pair.first;
        if (strpoints!=pcd_name){
            continue;
        }
        // std::cout << "strpoints = " << strpoints << std::endl;
        auto pPoints = pair.second;
        if( pPoints == NULL ) continue;
        for(int i=0; i<pPoints->size(); i=i+1)
        {
            PointXYZRGB &p = (*pPoints)[i];
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
}


void MapDrawer::drawPointCloud(PointCloud *pPoints, float pointSize)
{
    glPushMatrix();
    if( pPoints == NULL ) return;
    for(int i=0; i<pPoints->size(); i=i+1)
    {
        PointXYZRGB &p = (*pPoints)[i];
        glPointSize( pointSize );
        // glPointSize( p.size );
        glBegin(GL_POINTS);
        glColor3d(p.r/255.0, p.g/255.0, p.b/255.0);
        glVertex3d(p.x, p.y, p.z);
        glEnd();

    }
    glPopMatrix();
}



} //namespace ORB_SLAM
