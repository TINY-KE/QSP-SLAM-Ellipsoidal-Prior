#include "include/core/Ellipsoid.h"

#include "src/Polygon/Polygon.hpp"

#include <Eigen/Core>
#include <Eigen/SVD>
#include <Eigen/Dense>

#include "include/core/Plane.h"
#include "include/core/ConstrainPlane.h"
#include <boost/math/distributions/chi_squared.hpp>

// 卡方分布的分布函数
double chi2cdf(int degree, double chi)
{
    boost::math::chi_squared mydist(degree);
    double p = boost::math::cdf(mydist,chi);
    return p;
}

namespace g2o
{
    ellipsoid::ellipsoid():miInstanceID(-1),mbColor(false),bPointModel(false)
    {
    }

    // xyz roll pitch yaw half_scale
    void ellipsoid::fromMinimalVector(const Vector9d& v){
        Eigen::Quaterniond posequat = zyx_euler_to_quat(v(3),v(4),v(5));
        pose = SE3Quat(posequat, v.head<3>());
        scale = v.tail<3>();

        vec_minimal = v;
    }

    // xyz quaternion, half_scale
    void ellipsoid::fromVector(const Vector10d& v){
        pose.fromVector(v.head<7>());
        scale = v.tail<3>();
        vec_minimal = toMinimalVector();
    }

    const Vector3d& ellipsoid::translation() const {return pose.translation();}
    void ellipsoid::setTranslation(const Vector3d& t_) {pose.setTranslation(t_);}
    void ellipsoid::setRotation(const Quaterniond& r_) {pose.setRotation(r_);}
    void ellipsoid::setRotation(const Matrix3d& R) {pose.setRotation(Quaterniond(R));}
    void ellipsoid::setScale(const Vector3d &scale_) {scale=scale_;}

    // apply update to current ellipsoid. exponential map
    ellipsoid ellipsoid::exp_update(const Vector9d& update)
    {
        ellipsoid res;
        res.pose = this->pose*SE3Quat::exp(update.head<6>());
        res.scale = this->scale + update.tail<3>();

        res.UpdateValueFrom(*this);
        res.vec_minimal = res.toMinimalVector();
        return res;
    }

    // TOBE DELETED.
    ellipsoid ellipsoid::exp_update_XYZABC(const Vector6d& update)
    {
        ellipsoid res;

        // Vector6d pose_vec; pose_vec << 0, 0, 0, update[0], update[1], update[2];
        Vector3d diff_trans = update.head(3);
        Vector3d pose_trans = this->pose.translation();
        res.pose.setTranslation(pose_trans + diff_trans);
        res.pose.setRotation(this->pose.rotation());
        res.scale = this->scale + update.tail<3>();

        res.UpdateValueFrom(*this);
        res.vec_minimal = res.toMinimalVector();
        return res;
    }

    ellipsoid ellipsoid::exp_update_XYZABCYaw(const Vector7d& update)
    {
        // ellipsoid res = exp_update_XYZABC(update.head(6));
        // double diff_yaw = update[6];
        // ellipsoid res_rot = res.rotate_ellipsoid(diff_yaw);

        Vector3d trans = update.head(3);
        Vector3d scale_update = update.head(6).tail(3);
        double yaw = update[6];

        Vector6d update_xyzrpy; update_xyzrpy << trans[0], trans[1], trans[2], 0, 0, yaw;

        // std::cout << "Ellipsoid.cpp : " << "update_xyzrpy:" << update_xyzrpy.transpose() << std::endl;

        g2o::SE3Quat updateSE3; 
        updateSE3.fromXYZPRYVector(update_xyzrpy);   // 亲测有问题
        // updateSE3.setTranslation(trans);
        // Eigen::Quaterniond q_yaw = Eigen::Quaterniond::Identity(); 
        // updateSE3.setRotation(q_yaw);

        ellipsoid res;
        res.pose = this->pose*updateSE3;
        res.scale = this->scale + scale_update;

        res.UpdateValueFrom(*this);
        res.vec_minimal = res.toMinimalVector();

        return res;
    }

    // Update 存储 : X Y Z A B Yaw
    ellipsoid ellipsoid::exp_update_XYZABYaw(const Vector6d& update)
    {
        // 所有更新都在自身坐标系下进行.
        Vector3d update_trans = update.head(3);
        
        Vector3d update_scale; 
        update_scale.head(2) = update.tail(3).head(2);
        update_scale[2] = update[2]; // c 尺度与 z 的变化相同. 
        
        // 注意边界条件: 不能变为 0.

        // 实现在自身坐标系的更新

        // 尺度的更新
        Vector7d xyzabcY; xyzabcY << update_trans, update_scale, update.tail(1);
        return update_XYZABCYaw_Self(xyzabcY);
    }

    ellipsoid ellipsoid::update_XYZABCYaw_Self(const Vector7d& update)
    {
        // 构成一个变换
        Vector6d xyzrpy; 
        xyzrpy.head(3) = update.head(3); 
        xyzrpy.tail(3) << 0,0,update.tail(1);
        g2o::SE3Quat se;
        se.fromMinimalVector(xyzrpy);

        Vector3d scale_update = update.tail(4).head(3);
        
        ellipsoid res;
        res.pose = this->pose*se;
        res.scale = this->scale + scale_update;

        res.UpdateValueFrom(*this);
        res.vec_minimal = res.toMinimalVector();
        return res;
    }

    Vector9d ellipsoid::ellipsoid_log_error_9dof(const ellipsoid& newone) const
    {
        Vector9d res;
        SE3Quat pose_diff = newone.pose.inverse()*this->pose;

        res.head<6>() = pose_diff.log(); 
        res.tail<3>() = this->scale - newone.scale; 
        return res;        
    }

    // change front face by rotate along current body z axis. 
    // another way of representing cuboid. representing same cuboid (IOU always 1)
    ellipsoid ellipsoid::rotate_ellipsoid(double yaw_angle) const // to deal with different front surface of cuboids
    {
        ellipsoid res;
        SE3Quat rot(Eigen::Quaterniond(cos(yaw_angle*0.5),0,0,sin(yaw_angle*0.5)),Vector3d(0,0,0));   // change yaw to rotation.
        res.pose = this->pose*rot;
        res.scale = this->scale;
        
        res.UpdateValueFrom(*this);
        res.vec_minimal = res.toMinimalVector();

        const double eps = 1e-6;
        if ( (std::abs(yaw_angle-M_PI/2.0) < eps) || (std::abs(yaw_angle+M_PI/2.0) < eps) || (std::abs(yaw_angle-3*M_PI/2.0) < eps))
            std::swap(res.scale(0),res.scale(1));   

        return res;
    }

    Vector9d ellipsoid::min_log_error_9dof(const ellipsoid& newone, bool print_details) const
    {
        bool whether_rotate_ellipsoid=true;  // whether rotate cube to find smallest error
        if (!whether_rotate_ellipsoid)
            return ellipsoid_log_error_9dof(newone);

        // NOTE rotating ellipsoid... since we cannot determine the front face consistenly, different front faces indicate different yaw, scale representation.
        // need to rotate all 360 degrees (global cube might be quite different from local cube)
        // this requires the sequential object insertion. In this case, object yaw practically should not change much. If we observe a jump, we can use code
        // here to adjust the yaw.
        Vector4d rotate_errors_norm; Vector4d rotate_angles(-1,0,1,2); // rotate -90 0 90 180
        Eigen::Matrix<double, 9, 4> rotate_errors;
        for (int i=0;i<rotate_errors_norm.rows();i++)
        {
            ellipsoid rotated_cuboid = newone.rotate_ellipsoid(rotate_angles(i)*M_PI/2.0);  // rotate new cuboids
            Vector9d cuboid_error = this->ellipsoid_log_error_9dof(rotated_cuboid);
            rotate_errors_norm(i) = cuboid_error.norm();
            rotate_errors.col(i) = cuboid_error;
        }
        int min_label;
        rotate_errors_norm.minCoeff(&min_label);
        if (print_details)
            if (min_label!=1)
                std::cout<<"Rotate ellipsoid   "<<min_label<<std::endl;
        return rotate_errors.col(min_label);
    }

    // transform a local cuboid to global cuboid  Twc is camera pose. from camera to world
    ellipsoid ellipsoid::transform_from(const SE3Quat& Twc) const{
        ellipsoid res;
        // Twc*Tcq = Twq
        res.pose = Twc*this->pose;
        res.scale = this->scale;
        
        res.UpdateValueFrom(*this);
        res.vec_minimal = res.toMinimalVector();

        return res;
    }

    // transform a global cuboid to local cuboid  Twc is camera pose. from camera to world
    ellipsoid ellipsoid::transform_to(const SE3Quat& Twc) const{
        ellipsoid res;
        res.pose = Twc.inverse()*this->pose;
        res.scale = this->scale;

        res.UpdateValueFrom(*this);
        res.vec_minimal = res.toMinimalVector();
        
        return res;
    }

    // xyz roll pitch yaw half_scale
    Vector9d ellipsoid::toMinimalVector() const{
        Vector9d v;
        v.head<6>() = pose.toXYZPRYVector();
        v.tail<3>() = scale;
        return v;
    }

    // xyz quaternion, half_scale
    Vector10d ellipsoid::toVector() const{
        Vector10d v;
        v.head<7>() = pose.toVector();
        v.tail<3>() = scale;
        return v;
    }

    Matrix4d ellipsoid::similarityTransform() const
    {
        Matrix4d res = pose.to_homogeneous_matrix();    // 4x4 transform matrix
        Matrix3d scale_mat = scale.asDiagonal();
        res.topLeftCorner<3,3>() = res.topLeftCorner<3,3>()*scale_mat;
        return res;
    }


    void ellipsoid::UpdateValueFrom(const g2o::ellipsoid& e){
        this->miLabel = e.miLabel;
        this->mbColor = e.mbColor;
        this->mvColor = e.mvColor;
        this->miInstanceID = e.miInstanceID;

        this->prob = e.prob;
        // this->cplanes = e.cplanes;
        this->mvCPlanesInCamera = e.mvCPlanesInCamera;
        this->mvCPlanesWorld = e.mvCPlanesWorld;

        this->bbox = e.bbox;

        this->prob_3d = e.prob_3d;
        this->bPointModel = e.bPointModel;
    }

    ellipsoid::ellipsoid(const g2o::ellipsoid &e) {
        pose = e.pose;
        scale = e.scale;
        vec_minimal = e.vec_minimal;

        UpdateValueFrom(e);
    }

    const ellipsoid& ellipsoid::operator=(const g2o::ellipsoid &e) {
        pose = e.pose;
        scale = e.scale;
        vec_minimal = e.vec_minimal;

        UpdateValueFrom(e);
        return e;
    }

    // ************* Functions As Ellipsoids ***************
    Vector2d ellipsoid::projectCenterIntoImagePoint(const SE3Quat& campose_cw, const Matrix3d& Kalib) const
    {
        return projectPointIntoImagePoint(translation(), campose_cw, Kalib);
    }

    Vector2d ellipsoid::projectPointIntoImagePoint(const Vector3d& point, const SE3Quat& campose_cw, const Matrix3d& Kalib) const
    {
        Matrix3Xd  P = generateProjectionMatrix(campose_cw, Kalib);

        Vector3d center_pos = point;
        Vector4d center_homo = real_to_homo_coord<double>(center_pos);
        Vector3d u_homo = P * center_homo;
        Vector2d u = homo_to_real_coord_vec<double>(u_homo);

        return u;
    }

    Matrix3d ellipsoid::projectOntoImageEllipseMat(const SE3Quat& campose_cw, const Matrix3d& Kalib) const 
    {
        Matrix4d Q_star = generateQuadric();
        Matrix3Xd  P = generateProjectionMatrix(campose_cw, Kalib);
        Matrix3d C_star = P * Q_star * P.transpose();
        Matrix3d C = C_star.inverse(); 
        C = C / C(2,2); // normalize

        // std::cout << "[Debug Ellipsoid.cpp] C: " << C << std::endl;
        return C;
    }

    // project the ellipsoid into the image plane, and get an ellipse represented by a Vector5d.
    // Ellipse: x_c, y_c, theta, axis1, axis2
    Vector5d ellipsoid::projectOntoImageEllipse(const SE3Quat& campose_cw, const Matrix3d& Kalib) const 
    {
        Matrix3d C = projectOntoImageEllipseMat(campose_cw, Kalib);
        // 计算给定矩阵的特征分解
        SelfAdjointEigenSolver<Matrix3d> es(C);    // ascending sort by default, 默认升序排序
        VectorXd eigens = es.eigenvalues();

        // If it is an ellipse, the sign of eigen values must be :  1 1 -1 
        // Ref book : Multiple View Geometry in Computer Vision
        int num_pos = int(eigens(0)>0) +int(eigens(1)>0) +int(eigens(2)>0);
        int num_neg = int(eigens(0)<0) +int(eigens(1)<0) +int(eigens(2)<0);

        // matrix to equation coefficients: ax^2+bxy+cy^2+dx+ey+f=0
        double a = C(0,0);
        double b = C(0,1)*2;
        double c = C(1,1);
        double d = C(0,2)*2;
        double e = C(2,1)*2;
        double f = C(2,2);

        // get x_c, y_c, theta, axis1, axis2 from coefficients
        double delta = c*c - 4.0*a*b;
        double k = (a*f-e*e/4.0) - pow((2*a*e-c*d),2)/(4*(4*a*b-c*c));
        double theta = 1/2.0*atan2(b,(a-c));
        double x_c = (b*e-2*c*d)/(4*a*c-b*b);
        double y_c = (b*d-2*a*e)/(4*a*c-b*b);
        double a_2 =  2*(a* x_c*x_c+ c * y_c*y_c+ b *x_c*y_c -1) /(a + c + sqrt((a-c)*(a-c)+b*b));
        double b_2 =  2*(a*x_c*x_c+c*y_c*y_c+b*x_c*y_c -1) /( a + c - sqrt((a-c)*(a-c)+b*b));

        double axis1 = sqrt(a_2);
        double axis2 = sqrt(b_2);

        Vector5d output;
        output << x_c, y_c, theta, axis1, axis2;

        return output;
    }

    // Get the bounding box from ellipse in image plane
    Vector4d ellipsoid::getBoundingBoxFromEllipse(Vector5d &ellipse) const
    {
        double a = ellipse[3];
        double b = ellipse[4];
        double theta = ellipse[2];
        double x = ellipse[0];
        double y = ellipse[1];
        
        double cos_theta_2 = cos(theta)*cos(theta);
        double sin_theta_2 = 1- cos_theta_2;

        double x_limit = sqrt(a*a*cos_theta_2+b*b*sin_theta_2);
        double y_limit = sqrt(a*a*sin_theta_2+b*b*cos_theta_2);

        Vector4d output;
        output[0] = x-x_limit; // left up
        output[1] = y-y_limit;
        output[2] = x+x_limit; // right down
        output[3] = y+y_limit;

        return output;
    }

    // Get projection matrix P = K [ R | t ] 
    Matrix3Xd ellipsoid::generateProjectionMatrix(const SE3Quat& campose_cw, const Matrix3d& Kalib) const
    {
        Matrix3Xd identity_lefttop;
        identity_lefttop.resize(3, 4);
        identity_lefttop.col(3)=Vector3d(0,0,0);
        identity_lefttop.topLeftCorner<3,3>() = Matrix3d::Identity(3,3);

        Matrix3Xd proj_mat = Kalib * identity_lefttop;
        proj_mat = proj_mat * campose_cw.to_homogeneous_matrix();

        return proj_mat;
    }

    // Get Q^*
    Matrix4d ellipsoid::generateQuadric() const
    {
        Vector4d axisVec;
        axisVec << (scale[0]*scale[0]), (scale[1]*scale[1]), (scale[2]*scale[2]), -1;
        Matrix4d Q_c_star = axisVec.asDiagonal();  
        // Matrix4d Q_c_star = Q_c.inverse();  
        Matrix4d Q_pose_matrix = pose.to_homogeneous_matrix();   // Twm  model in world,  world to model
        Matrix4d Q_c_star_trans = Q_pose_matrix * Q_c_star * Q_pose_matrix.transpose(); 

        return Q_c_star_trans;
    }

    // Get the projected bounding box in the image plane of the ellipsoid using a camera pose and a calibration matrix.
    Vector4d ellipsoid::getBoundingBoxFromProjection(const SE3Quat& campose_cw, const Matrix3d& Kalib) const
    {
        Vector5d ellipse = projectOntoImageEllipse(campose_cw, Kalib);
        return getBoundingBoxFromEllipse(ellipse);
    }

    Vector3d ellipsoid::getColor(){
        return mvColor.head(3);
    }

    Vector4d ellipsoid::getColorWithAlpha(){
        return mvColor;
    }

    void ellipsoid::setColor(const Vector3d &color_, double alpha){
        mbColor = true;
        mvColor.head<3>() = color_;
        mvColor[3] = alpha;

    }

    bool ellipsoid::isColorSet(){
        return mbColor;
    }

    bool ellipsoid::CheckObservability(const SE3Quat& campose_cw)
    {
        Vector3d ellipsoid_center = toMinimalVector().head(3);    // Pwo
        Vector4d center_homo = real_to_homo_coord_vec<double>(ellipsoid_center);

        Eigen::Matrix4d projMat = campose_cw.to_homogeneous_matrix(); // Tcw
        Vector4d center_inCameraAxis_homo = projMat * center_homo;   // Pco =  Tcw * Pwo
        Vector3d center_inCameraAxis = homo_to_real_coord_vec<double>(center_inCameraAxis_homo);

        if( center_inCameraAxis_homo(2) < 0)    // if the center is behind the camera ; z<0
        {
            return false;
        }
        else
            return true;
    }

    // calculate the IoU Error between two axis-aligned ellipsoid
    double ellipsoid::calculateMIoU(const g2o::ellipsoid& e) const
    {
        return calculateIntersectionError(*this, e);
    }

    double ellipsoid::calculateIntersectionOnZ(const g2o::ellipsoid& e1, const g2o::ellipsoid& e2) const
    {
        g2o::SE3Quat pose_diff = e1.pose.inverse() * e2.pose;
        double z1 = 0; double z2 = pose_diff.translation()[2];
        double scale1 = e1.scale[2];
        double scale2 = e2.scale[2];

        if( z2 < z1 )   // 让 z2为正.
            z2 = -z2;

        double cross = z1 + scale1 - (z2 - scale2);
        double length = MAX(0, MIN( cross , 2*scale2));
        length = MIN( length, 2*scale1);

        // std::cout << "****TEMP: THIS IS THE NEW Intersection Function ... " << std::endl;
        // std::cout << " - z1 : " << z1 << std::endl;
        // std::cout << " - z2 : " << z2 << std::endl;
        // std::cout << " - scale1 : " << scale1 << std::endl;
        // std::cout << " - scale2 : " << scale2 << std::endl;
        // std::cout << " - cross : " << cross << std::endl;
        // std::cout << " - length : " << length << std::endl;

        return length;
    }

    double ellipsoid::calculateArea(const g2o::ellipsoid& e) const
    {
        return e.scale[0]*e.scale[1]*e.scale[2]*8;
    }

    void OutputPolygon(ORB_SLAM2::Polygon& polygon, double resolution)
    {
        int num = polygon.n;
        for( int i=0;i<num;i++)
            std::cout << i << ":" << polygon[i].x << ", " << polygon[i].y << std::endl;
        std::cout << std::endl;
    }

    // Calculate the intersection area after projected the external cubes of two axis-aligned ellipsoids into XY-Plane.
    double ellipsoid::calculateIntersectionOnXY(const g2o::ellipsoid& e1, const g2o::ellipsoid& e2) const
    {
        // First, get the axis-aligned pose error
        g2o::SE3Quat pose_diff = e1.pose.inverse() * e2.pose;
        double x_center1 = 0; double y_center1 = 0;

        double x_center2 = pose_diff.translation()[0];
        double y_center2 = pose_diff.translation()[1];

        double roll,pitch,yaw;
        quat_to_euler_zyx(pose_diff.rotation(),roll,pitch,yaw);

        double a1 = std::abs(e1.scale[0]);
        double b1 = std::abs(e1.scale[1]);

        double a2 = std::abs(e2.scale[0]);
        double b2 = std::abs(e2.scale[1]);

        // *************************
        // 旧版本: 使用 Polygon 库
        // *************************
        // Use polygon to calculate the intersection
        ORB_SLAM2::Polygon polygon1, polygon2;
        double resolution = 0.001;  // m / resolution = pixel

        // polygon1.add(cv::Point(a1/resolution, b1/resolution));    // cvPoint only accepts integer, so use resolution to map meter to pixel ( 0.01 resolution means: 1pixel = 0.01m )
        // polygon1.add(cv::Point(-a1/resolution, b1/resolution)); 
        // polygon1.add(cv::Point(-a1/resolution, -b1/resolution)); 
        // polygon1.add(cv::Point(a1/resolution, -b1/resolution)); 

        polygon1.add(cv::Point(cvRound(a1/resolution), cvRound(b1/resolution)));
        polygon1.add(cv::Point(cvRound(-a1/resolution), cvRound(b1/resolution)));
        polygon1.add(cv::Point(cvRound(-a1/resolution), cvRound(-b1/resolution)));
        polygon1.add(cv::Point(cvRound(a1/resolution), cvRound(-b1/resolution)));

        double c_length = sqrt(a2*a2+b2*b2);

        double init_theta = CV_PI/2.0 - atan2(a2,b2);
        Vector4d angle_plus_vec;
        angle_plus_vec << 0, atan2(a2,b2)*2, CV_PI, CV_PI+atan2(a2,b2)*2;
        for( int n=0;n<4;n++){
            double angle_plus = angle_plus_vec[n];  // rotate 90deg for four times
            double point_x = c_length * cos( init_theta - yaw + angle_plus ) + x_center2;
            double point_y = c_length * sin( init_theta - yaw + angle_plus ) + y_center2;
            polygon2.add(cv::Point(point_x/resolution, point_y/resolution));  
        }

        // calculate the intersection
        ORB_SLAM2::Polygon interPolygon;
        ORB_SLAM2::intersectPolygon(polygon1, polygon2, interPolygon);

        // eliminate resolution.
        double inter_area = interPolygon.area();
        double inter_area_in_m = inter_area * resolution * resolution;
        // *************************

        // *************************
        // 新版本: 使用 cv 库
        // 似乎没有 RotatedRect 的并集!
        // *************************
        // // 与bbox求IoU
        // cv::Rect r1(cv::Point(rect[0], rect[1]), cv::Point(rect[2], rect[3]));
        // cv::Rect r2(cv::Point(bbox[0], bbox[1]), cv::Point(bbox[2], bbox[3]));
        // cv::RotatedRect(); // 检查 是否有交并补 

        // cv::Rect r_and = r1 | r2;
        // cv::Rect r_U = r1 & r2;
        // double iou = r_U.area()*1.0/r_and.area();


        return inter_area_in_m;
    }

    double ellipsoid::calculateIntersectionError(const g2o::ellipsoid& e1, const g2o::ellipsoid& e2) const
    {
        //          AXB          
        // IoU = ----------
        //          AUB
        //   AXB  =  intersection
        //   AUB  =  A+B-intersection

        // Error of IoU : 1 - IoU
        double areaA = std::abs(calculateArea(e1));        
        // std::cout << "areaA : " << areaA << std::endl;

        double areaB = std::abs(calculateArea(e2));
        // std::cout << "areaB : " << areaB << std::endl;

        double proj_inter = calculateIntersectionOnXY(e1,e2);
        double z_inter = calculateIntersectionOnZ(e1,e2);
        // std::cout << "projInter : " << proj_inter << std::endl;
        // std::cout << "z_inter : " << z_inter << std::endl;

        double areaIntersection = proj_inter * z_inter;
        // std::cout << "areaIntersection : " << areaIntersection << std::endl;

        double MIoU = 1 - ((areaIntersection) / (areaA + areaB - areaIntersection));
        // std::cout << "MIoU : " << MIoU << std::endl;
        // std::cout << "e1 : " << e1.toMinimalVector().transpose() << std::endl;
        // std::cout << "e2 : " << e2.toMinimalVector().transpose() << std::endl;

        return MIoU;
    }

    // ***************** Functions as Cubes ******************

    // calculate the external cube of the ellipsoid
    // 8 corners 3*8 matrix, each row is x y z
    Matrix3Xd ellipsoid::compute3D_BoxCorner() const
    {
        Matrix3Xd corners_body;corners_body.resize(3,8);
        corners_body<< 1, 1, -1, -1, 1, 1, -1, -1,
                1, -1, -1, 1, 1, -1, -1, 1,
                -1, -1, -1, -1, 1, 1, 1, 1;
        Matrix3Xd corners_world = homo_to_real_coord<double>(similarityTransform()*real_to_homo_coord<double>(corners_body));
        return corners_world;
    }

    Matrix2Xd ellipsoid::projectOntoImageBoxCorner(const SE3Quat& campose_cw, const Matrix3d& Kalib) const
    {
        Matrix3Xd corners_3d_world = compute3D_BoxCorner();
        Matrix2Xd corner_2d = homo_to_real_coord<double>(Kalib*homo_to_real_coord<double>(campose_cw.to_homogeneous_matrix()*real_to_homo_coord<double>(corners_3d_world)));

        return corner_2d;
    }

    // get rectangles after projection  [topleft, bottomright]
    Vector4d ellipsoid::projectOntoImageRect(const SE3Quat& campose_cw, const Matrix3d& Kalib) const
    {
        Matrix2Xd corner_2d = projectOntoImageBoxCorner(campose_cw, Kalib);
        Vector2d bottomright = corner_2d.rowwise().maxCoeff(); // x y
        Vector2d topleft = corner_2d.rowwise().minCoeff();
        return Vector4d(topleft(0),topleft(1),bottomright(0),bottomright(1));
    }

    // get rectangles after projection  [center, width, height]
    Vector4d ellipsoid::projectOntoImageBbox(const SE3Quat& campose_cw, const Matrix3d& Kalib) const
    {
        Vector4d rect_project = projectOntoImageRect(campose_cw, Kalib);  // top_left, bottom_right  x1 y1 x2 y2
        Vector2d rect_center = (rect_project.tail<2>()+rect_project.head<2>())/2;
        Vector2d widthheight = rect_project.tail<2>()-rect_project.head<2>();
        return Vector4d(rect_center(0),rect_center(1),widthheight(0),widthheight(1));
    }

    std::vector<g2o::plane*> ellipsoid::GetCubePlanes()
    {
        // 根据初始椭球体的pose和scale， 计算 3D 盒子的角点（即顶点）。
        Matrix3Xd mPoints = compute3D_BoxCorner();

        // 每一列表示一组三个点的 ID，这些 ID 将用于定义三角形平面。
        // 这些 ID 将用于从 mPoints 中提取 3 个顶点，以定义每个平面。
        Matrix3Xd mIds; mIds.resize(3, 6);
        // 注意： 该 id 从1开始计数！
        mIds << 1, 5, 1, 3, 6, 8,
                2, 8, 5, 7, 7, 5,
                3, 7, 6, 8, 3, 1;
        
        // 循环构建平面
        std::vector<g2o::plane*> planes;
        for( int i=0;i<6;i++)
        {
            Vector3d p0 = mPoints.col(mIds(0,i)-1);
            Vector3d p1 = mPoints.col(mIds(1,i)-1);
            Vector3d p2 = mPoints.col(mIds(2,i)-1);
            // 这个平面的中心
            Vector3d center = (p0+p2)/2.0;
            // 平面的法向量
            Vector3d normal = (p1-p0).cross(p2-p1);
            g2o::plane* plane = new g2o::plane();
            plane->fromPointAndNormal(center, normal);
            plane->mvPlaneCenter = center;
            plane->color = Vector3d(0,0,1.0);
            planes.push_back(plane);
        }

        //123, 587, 156, 378, 673, 851
        return planes;
    }

    void ellipsoid::addConstrainPlanes(std::vector<ConstrainPlane*>& vCPlanes)
    {
        mvCPlanesInCamera = vCPlanes;
        NormalizeConstrainPlanes();
    }

    void ellipsoid::NormalizeConstrainPlanes()
    {
        int cplane_num = mvCPlanesInCamera.size();
        for(int i=0;i<cplane_num;i++)
        {
            ConstrainPlane* pCPlane = mvCPlanesInCamera[i];
            // 检查椭球体中心的flag.
            bool flag_positive = pCPlane->pPlane->distanceToPoint(this->pose.translation(), true) > 0;
            if(!flag_positive)
                pCPlane->pPlane->param = -pCPlane->pPlane->param; // reverse param
        }
        return;
    }

    // xyz quaternion, half_scale, label, prob [load/save io]
    VectorXd ellipsoid::SaveToVector() const
    {
        VectorXd vec; vec.resize(vectorSize());

        VectorXd basic_vec = this->toVector(); // 10
        double label = this->miLabel;
        double prob = this->prob;

        vec << basic_vec, label, prob, this->bbox, this->prob_3d, this->bPointModel?1:0;
        return vec;
    }

    // 接口: 同时处理有Constrain和无Constrain版本
    // 目前简单做存储内容长度判断，已假设不做恶意修改或纰漏。
    void ellipsoid::LoadFromVector(const VectorXd& vec)
    {
        if(vec.size() > vectorSize()) LoadFromVectorWithVecPlanes(vec);
        else LoadFromVectorWithoutVecPlanes(vec);

        return;
    }

    void ellipsoid::LoadFromVectorWithoutVecPlanes(const VectorXd& vec)
    {
        // x y z qx qy qz qw a b c label  11
        if(vec.size()==vectorSize())
        {
            this->fromVector(vec.head(10));
            this->miLabel = round(vec[10]);
            if(vec.size()>11)
                this->prob = vec[11];
            else 
                this->prob = 1.0;
            
            if(vec.size()>15)
                this->bbox << vec[12],vec[13],vec[14],vec[15];
            
            if(vec.size()>17){
                this->prob_3d = vec[16];
                this->bPointModel = (vec[17] > 0.5);
            }
        }
        else if(vec.size() == 11)
        {
            this->fromVector(vec.head(10));
            this->miLabel = round(vec[10]);
            this->prob = 1.0;
        }
        else 
            std::cerr << "Wrong size to initialize an ellipsoid: " << vec.size() << std::endl;

        return;
    }

    int ellipsoid::vectorSize()
    {
        // 2020.7.6 : 
        // 18: 16 +2 (prob_3d pointmodel)

        // 16: 12   + 4( bbox )
        return 18;
    }

    // 设计单独保存的 PlaneVec. 到底保存到一个txt中还是?
    // 干脆一个txt!
    VectorXd ellipsoid::SaveToVectorWithVecPlanes() const
    {
        VectorXd vec_info = SaveToVector();
        // std::cout << " 1 ) ellipsoid vec : " << vec_info.transpose() << std::endl;
        // 接下来获得平面数量, 并往后排列平面
        int plane_num = mvCPlanesInCamera.size();
        int single_vec_size = ConstrainPlane::vectorSize();

        // 接下来每个平面都变成一个 Vector, 并且叠加到同一个Vector后面
        VectorXd total_cplane_vec; total_cplane_vec.resize(plane_num * single_vec_size);
        for(int i=0; i<plane_num; i++)
        {
            ConstrainPlane* pCPlane = mvCPlanesInCamera[i];
            VectorXd vec_cplane = pCPlane->toVector();
            // std::cout << " 2." << i << ") CPlane vec : " << vec_cplane.transpose() << std::endl;

            // 赋值; TODO: check.
            total_cplane_vec.block(i*single_vec_size, 0, single_vec_size, 1) = vec_cplane;
        }

        // 1: num of the constrain planes ?
        VectorXd vec_objWithCPlane; vec_objWithCPlane.resize(vec_info.size()+total_cplane_vec.size()+1);
        vec_objWithCPlane << vec_info, double(plane_num), total_cplane_vec;
        // std::cout << "info: " << std::endl;
        // std::cout << "vec_info size : " << vec_info.size() << std::endl;
        // std::cout << "total_cplane_vec size : " << total_cplane_vec.size() << std::endl;
        // std::cout << "vec_objWithCPlane size : " << vec_objWithCPlane.size() << std::endl;
        // while(1);//wait for debugging.
        // std::cout << "total_cplane_vec : " << total_cplane_vec.transpose() << std::endl;
        // std::cout << " all. vec : " << vec_objWithCPlane.transpose() << std::endl;
        // while(1);
        return vec_objWithCPlane;
    }

    void ellipsoid::LoadFromVectorWithVecPlanes(const VectorXd& vec)
    {
        int vec_ellipsoid_size = vectorSize();
        LoadFromVectorWithoutVecPlanes(vec.head(vec_ellipsoid_size));        // TODO: check Vector10d->VectorXd 的转换.

        // 处理平面
        int plane_num_pos = vec_ellipsoid_size;  // 位置在物体信息之后
        int plane_num = round(vec[plane_num_pos]);   

        mvCPlanesInCamera.clear();
        mvCPlanesInCamera.resize(plane_num);
        int sngle_vec_size = ConstrainPlane::vectorSize();

        VectorXd vec_cplanes = vec.block(vec_ellipsoid_size+1, 0, plane_num*sngle_vec_size, 1);
        for(int i =0;i<plane_num;i++)
        {
            VectorXd vec_cplane = vec_cplanes.block(i*sngle_vec_size, 0, sngle_vec_size, 1);
            ConstrainPlane* pCPlane = new ConstrainPlane(NULL);
            pCPlane->fromVector(vec_cplane);    // 该过程会判断无plane时初始化
            mvCPlanesInCamera[i] = pCPlane;
        }

        return;
    }

    double ellipsoid::minus(const g2o::ellipsoid& e)
    {
        // dis of center
        // double dis;
        // 检查它们的 Z Axis 是否对齐了
        // Quaterniond quat1 = pose.rotation();
        // Quaterniond quat2 = e.pose.rotation();
        // Quaterniond quat_diff = quat1.inverse() * quat2;


        double dis_center = (translation() - e.translation()).norm();
        
        // double dis_9dof = min_log_error_9dof(e, false);

        return dis_center;
    }

    // 综合考虑椭球体的三维误差: trans, rotation, scale
    double ellipsoid::minus3d(const g2o::ellipsoid& e)
    {
        double config_dis_sigma = 0.1;
        double config_yaw_sigma = M_PI / 180.0 * 10;
        double config_scale_sigma = 0.1;
            
        Vector7d inv_sigma_vec; 
        inv_sigma_vec.head(3).fill(1.0/config_dis_sigma);
        // inv_sigma_vec[3] = 1.0/config_yaw_sigma;
        inv_sigma_vec[3] = 0;
        inv_sigma_vec.tail(3).fill(1.0/config_scale_sigma);
        Matrix<double, 7, 7> info = inv_sigma_vec.cwiseProduct(inv_sigma_vec).asDiagonal();

        // 旋转 yaw 角度获得最小距离差.
        bool bRotate = false;
        double prob_min;
        if(bRotate){
            Vector4d rotate_errors_norm; Vector4d rotate_angles(-1,0,1,2); // rotate -90 0 90 180
            std::vector<double> prob_list; prob_list.resize(rotate_angles.rows());
            for(int i=0;i<rotate_angles.rows();i++){
                g2o::ellipsoid e_rotated = e.rotate_ellipsoid(rotate_angles(i)*M_PI/2.0);
                Vector7d res = ellipsoid_error_3d(e_rotated);
                double chi2 = res.transpose() * info * res;  // 7 个高斯分布相加的分布是什么. 均值不变, 方差 n*Sigma 即n
                double prob = 1 - chi2cdf(7, chi2);
                prob_list[i] = prob;
            }
            prob_min = *std::min_element(prob_list.begin(), prob_list.end());
        }
        else 
        {
            Vector7d res = ellipsoid_error_3d(e);
            double chi2 = res.transpose() * info * res;  // 7 个高斯分布相加的分布是什么. 均值不变, 方差 n*Sigma 即n
            double prob = 1 - chi2cdf(7, chi2);
            prob_min = prob;
        }
        
        // return dis_center;
        // std::cout << std::endl;
        // std::cout << "e1 : " << this->toMinimalVector().transpose() << std::endl;
        // std::cout << "e2 : " << e.toMinimalVector().transpose() << std::endl;
        // // std::cout << "res : " << res.transpose() << std::endl;
        // // std::cout << "chi2 : " << chi2 << std::endl;
        // std::cout << "prob_min : " << prob_min << std::endl;
        // getchar();

        return prob_min;// 未完成!
    }

    // 假定两个椭球体的 Z 轴是 Aligned 的.
    // x,y,z,yaw,a,b,c
    Vector7d ellipsoid::ellipsoid_error_3d(const ellipsoid& e) const
    {
        Vector7d res;
        Vector3d dis_center = (this->translation() - e.translation());
        
        // 计算 yaw 角
        Eigen::Quaterniond quat_dis = this->pose.rotation().inverse() * e.pose.rotation();
        Eigen::AngleAxisd angleAxis(quat_dis);
        double dis_yaw = angleAxis.angle();
        dis_yaw = normalize_to_pi(dis_yaw);
        
        Vector3d scale_diff = this->scale - e.scale;

        res << dis_center, dis_yaw, scale_diff;

        return res;        
    }

    std::vector<plane*> ellipsoid::GetCubePlanesInImages(const SE3Quat& campose_cw, const Matrix3d& Kalib, int rows, int cols, int pixel_thresh)
    {
        // 先获得各个顶点是否在图像内的判断
        Matrix2Xd corner_mat = projectOntoImageBoxCorner(campose_cw, Kalib);
        std::vector<bool> corner_in_image; corner_in_image.resize(8);
        for(int i=0;i<8;i++)
        {
            Vector2d corner = corner_mat.col(i);
            if( corner[0] > pixel_thresh && corner[1] > pixel_thresh && corner[0] < (cols-pixel_thresh) && corner[1] < (rows-pixel_thresh))
                corner_in_image[i] = true;
            else 
                corner_in_image[i] = false;

        }

        // 开始筛选有效平面
        Matrix3Xd mPoints = compute3D_BoxCorner();
        Matrix4Xd mIds; mIds.resize(4, 6);

        // 注意： 该 id 从1开始计数！
        mIds << 1, 5, 1, 3, 6, 8,
                2, 8, 5, 7, 7, 5,
                3, 7, 6, 8, 3, 1,
                4, 6, 2, 4, 2, 4; 
        
        std::vector<g2o::plane*> planes;
        for( int i=0;i<6;i++)
        {
            Vector4d corner_ids = mIds.col(i);
            int valid_count = 0;
            for(int n=0;n<4;n++){
                int id = round(corner_ids[n]) - 1;
                if(corner_in_image[id]) valid_count++;
            }
            if(valid_count < 2) continue; // 4个顶点中必须有两个以上在图像内.

            Vector3d p0 = mPoints.col(mIds(0,i)-1);
            Vector3d p1 = mPoints.col(mIds(1,i)-1);
            Vector3d p2 = mPoints.col(mIds(2,i)-1);

            Vector3d center = (p0+p2)/2.0;

            Vector3d normal = (p1-p0).cross(p2-p1);
            g2o::plane* plane = new g2o::plane();
            plane->fromPointAndNormal(center, normal);
            plane->mvPlaneCenter = center;
            plane->color = Vector3d(0,0,1.0);
            planes.push_back(plane);
        }

        //123, 587, 156, 378, 673, 851
        return planes;
    }

    Matrix3d ellipsoid::projectAxisOnImage(const g2o::SE3Quat& campose_cw, const Matrix3d& calib, int length_scale) const
    {
        // 三个轴的端点
        Matrix3d rotMat = pose.rotation().toRotationMatrix();
        Vector3d center_3d = translation();
        
        Vector3d x_axis_3d = center_3d + rotMat.col(0) * scale[0] * length_scale;
        Vector3d y_axis_3d = center_3d + rotMat.col(1) * scale[1] * length_scale;
        Vector3d z_axis_3d = center_3d + rotMat.col(2) * scale[2] * length_scale;

        Vector2d x_axis_2d = projectPointIntoImagePoint(x_axis_3d, campose_cw, calib);
        Vector2d y_axis_2d = projectPointIntoImagePoint(y_axis_3d, campose_cw, calib);
        Vector2d z_axis_2d = projectPointIntoImagePoint(z_axis_3d, campose_cw, calib);

        Matrix3d proj_axes;
        proj_axes.col(0) << x_axis_2d, 1;
        proj_axes.col(1) << y_axis_2d, 1;
        proj_axes.col(2) << z_axis_2d, 1;
        return proj_axes;
    }

    void ellipsoid::drawEllipseOnImage(const Vector5d& ellipse, cv::Mat& im, const cv::Scalar& color)
    {
        // std::cout << "Ellipse from circle : " << ellipse.transpose() << std::endl;
        // ellipse: center_x, center_y, 
        cv::RotatedRect rotbox2(cv::Point2f(ellipse[0],ellipse[1]), cv::Size2f(ellipse[3]*2,ellipse[4]*2), ellipse[2]/M_PI*180);
        try
        {
            // std::cout << "drawEllipseOnImage success" << std::endl;
            cv::ellipse(im, rotbox2, color);
        }
        catch(const std::exception& e)
        {
            std::cout << "drawEllipseOnImage fail" << std::endl;
            std::cerr << e.what() << '\n';
        }
        return;
    }


    void ellipsoid::addFilteredPlanesInWorld(g2o::plane* plane)
    {
        mpPlanes.push_back(plane);
    }

    std::vector<g2o::plane*> ellipsoid::GetPlanes()
    {
        return mpPlanes;
    }

} // g2o