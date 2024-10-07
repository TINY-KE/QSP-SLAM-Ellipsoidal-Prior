#include "include/core/Plane.h"

namespace g2o
{
    plane::plane() {
        param = Vector4d(1,0,0,0);
        color = Vector3d(1,0,0);
        mdDualDis = 0;

        mbLimited = false;
        miMHType = 0;
    }

    plane::plane(Vector4d param_, Eigen::Vector3d color_) {
        param = param_;
        color = color_;
        mdDualDis = 0;

        mbLimited = false;
        miMHType = 0;
    }

    plane plane::exp_update(const Vector3d& update)
    {
        g2o::plane plane_update;

        plane_update.param[0] = param[0] + update[0]; // A
        plane_update.param[1] = param[1] + update[1]; // B
        plane_update.param[2] = 0; // C 
        plane_update.param[3] = param[3] + update[2]; // D
        plane_update.color = color;

        return plane_update;
    }

    plane plane::exp_update2DOF(const Vector2d& update)
    {
        double k_update = update[0];
        double b_update = update[1];

        double k_current = -param[0]/param[1];
        double b_current = -param[3]/param[1];

        double k = k_current + k_update;
        double b = b_current + b_update;
    
        double B = -1;
        double A = k;
        double C = 0;
        double D = b;

        g2o::plane plane_update;
        plane_update.param[0] = A; // A
        plane_update.param[1] = B; // B
        plane_update.param[2] = C; // C ; z remains unchanged
        plane_update.param[3] = D; // D
        plane_update.color = color;

        return plane_update;
    }

    plane::plane(const plane &p){
        param = p.param;
        color = p.color;

        mbLimited = p.mbLimited;
        mvPlaneCenter = p.mvPlaneCenter;
        mdPlaneSize = p.mdPlaneSize;

        mdDualDis = p.mdDualDis;
        miMHType = p.miMHType;
    }

    const plane& plane::operator=(const plane& p){
        param = p.param;
        color = p.color;

        mbLimited = p.mbLimited;
        mvPlaneCenter = p.mvPlaneCenter;
        mdPlaneSize = p.mdPlaneSize;

        mdDualDis = p.mdDualDis;
        miMHType = p.miMHType;
        return p;
    }

    void plane::fromPointAndNormal(const Vector3d &point, const Vector3d &normal)
    {
        param.head(3) = normal;  // normal : [ a, b, c]^T
        param[3] = -point.transpose() * normal;        // X^T * pi = 0 ; ax0+by0+cz0+d=0
        color = Vector3d(1,0,0);

        mdDualDis = 0;
    }

    void plane::fromDisAndAngle(double dis, double angle)
    {
        fromDisAngleTrans(dis, angle, 0);
    }

    void plane::fromDisAngleTrans(double dis, double angle, double trans)
    {
        param[0] = sin(angle);
        param[1] = -cos(angle);
        param[2] = 0;
        param[3] = -dis;

        mdDualDis = trans;
    }

    // 默认来说，支撑平面的正方向是朝上的
    double plane::distanceToPoint(const Vector3d& point, bool keep_flag) const{
        double fenzi = param(0)*point(0) + param(1)*point(1) +param(2)*point(2) + param(3);
        double fenmu = std::sqrt ( param(0)*param(0)+param(1)*param(1)+param(2)*param(2) );
        double value = fenzi/fenmu;

        if(keep_flag) return value;
        else return std::abs(value);
    }

    // 条件: 平面间角度相差5度以内.
    double plane::distanceToPlane(const g2o::plane& pl)
    {
        Vector3d zero(0,0,0);
        double dis1 = distanceToPoint(zero, true);

        g2o::plane plane_align = pl;
        // 对齐法向量方向.
        const double deg_5 = M_PI/180.0*5;
        if( std::abs(angleToPlane(pl)-0) < deg_5) 
        {
            // nothing
        }
        else if(  std::abs(angleToPlane(pl)-M_PI) < deg_5 )
        {
            // 调转方向
            plane_align.param = -plane_align.param;
        }
        else
        {
            // 不满足条件
            std::cout << "[Plane.cpp] Try to calculate the distance of two non-parallel planes." << std::endl;
            return -1;
        }
        double dis2 = plane_align.distanceToPoint(zero, true);

        double dis = std::abs(dis1 - dis2);
        return dis;
    }
    
    // zhjd： 将平面从w坐标系， 转到c坐标系下
    void plane::transform(const g2o::SE3Quat& Twc){
        Matrix4d matTwc = Twc.to_homogeneous_matrix();
        // std::cout << "matTwc : " << matTwc << std::endl;
        Matrix4d matTwc_trans = matTwc.transpose();
        // std::cout << "matTw_trans : " << matTwc_trans  << std::endl;
        Matrix4d matTwc_trans_inv = matTwc_trans.inverse();
        // std::cout << "matTwc_trans_inv : " << matTwc_trans_inv  << std::endl;
        // 平面的变换公式 Twc*Pw = Pc
        param = matTwc_trans_inv * param;
    }

    // for the visualization of symmetry planes
    void plane::InitFinitePlane(const Vector3d& center, double size)
    {
        mdPlaneSize = size;
        mvPlaneCenter = center;
        mbLimited = true;
    }

    Eigen::Vector4d plane::GeneratePlaneVec()
    {
        return param;
    }

    Eigen::Vector4d plane::GenerateAnotherPlaneVec()
    {
        // azimuth : angle of the normal
        g2o::plane p2;
        p2.fromDisAndAngle(mdDualDis, azimuth());

        return p2.param;
    }

    Vector3d plane::GetLineFromCenterAngle(const Vector2d center, double angle)
    {
        // x = center[0] + t * cos(theta)
        // y = center[1] + t * sin(theta)
        // goal : 
        // AX + BY + C = 0 ;  get A,B,C

        // get rid of t:
        // sin(theta) * x - cos(theta) * y = 
        //                      sin(theta) * center[0] - cos(theta) * center[1]
        
        // so: 
        // sint * x + (- cost) * y  + (cost*c1 - sint*c0) = 0

        Vector3d param;
        param[0] = sin(angle);
        param[1] = -cos(angle);
        param[2] = cos(angle) * center[1] - sin(angle) * center[0];

        return param;
    }

    Vector4d plane::LineToPlane(const Vector3d line)
    {
        Vector4d plane;
        plane << line[0], line[1], 0, line[2];
        return plane;
    }

    double plane::angleToPlane(const g2o::plane& pl)
    {
        Vector3d norm1 = param.head(3);
        Vector3d norm2 = pl.param.head(3);
        
        double cos_angle = norm1.transpose() * norm2;
        cos_angle = cos_angle / norm1.norm() / norm2.norm();
        double angle = acos(cos_angle);

        return angle;
    }

    Vector3d plane::GetThePointNearOrigin()
    {
        // 该点由 法向量 * 到原点的距离 获得.
        // 接着判断其符号, 由距离平面最近给出.
        Vector3d point = normal() * distanceToPoint(Vector3d::Zero());
        double dis_positive = distanceToPoint(point);
        double dis_negative = distanceToPoint(-point);

        Vector3d point_withFlag;
        if(dis_positive > dis_negative)
            point_withFlag = -point;
        else 
            point_withFlag = point;

        // debug output 
        std::cout << "dis_positive : " << dis_positive << std::endl;
        std::cout << "dis_negative : " << dis_negative << std::endl;

        return point_withFlag;
    }

    Vector3d plane::SampleNearAnotherPoint(const Vector3d& point)
    {
        Vector3d normal_norm = normal(); normal_norm.normalize();
        Vector3d point_on_plane1 = point - normal_norm * distanceToPoint(point, false);
        Vector3d point_on_plane2 = point + normal_norm * distanceToPoint(point, false);

        double dis1 = distanceToPoint(point_on_plane1);
        double dis2 = distanceToPoint(point_on_plane2);

        return dis1<dis2?point_on_plane1:point_on_plane2;
    }

} // g2o namespace