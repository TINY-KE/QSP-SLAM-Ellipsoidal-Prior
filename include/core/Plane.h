#pragma  once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>

#include "include/utils/matrix_utils.h"

#include "Ellipsoid.h"

namespace g2o
{
class plane {

public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    plane();

    plane(Vector4d param, Vector3d color=Vector3d(1.0,0.0,0.0));

    // Copy constructor.
    plane(const plane &p);

    const plane& operator=(const plane& p);

    // initialize a plane from a point and a normal vector
    void fromPointAndNormal(const Vector3d &point, const Vector3d &normal);

    void fromDisAndAngle(double dis, double angle);
    void fromDisAngleTrans(double dis, double angle, double trans);

    // update plane parameters in 3 degrees
    plane exp_update(const Vector3d& update);
    
    // update plane parameters in 2 degrees
    plane exp_update2DOF(const Vector2d& update);

    // distance from a point to plane
    double distanceToPoint(const Vector3d& point, bool keep_flag = false) const;
    void transform(const g2o::SE3Quat& Twc);
    double distanceToPlane(const g2o::plane& pl);
    double angleToPlane(const g2o::plane& pl);  // rad from acos

    Vector3d GetThePointNearOrigin();

    // init visualization for a finite plane given a point and a size
    void InitFinitePlane(const Vector3d& center, double size);

    // update function for optimization
    inline void oplus(const Vector3d& v){
      //construct a normal from azimuth and evelation;
      double _azimuth=v[0];
      double _elevation=v[1];
      double s=std::sin(_elevation), c=std::cos(_elevation);
      Vector3d n (c*std::cos(_azimuth), c*std::sin(_azimuth), s) ;
      
      // rotate the normal
      Matrix3d R=rotation(normal());
      double d=distance()+v[2];         // why is plus?
      param.head<3>() = R*n;
      param(3) = -d;
      normalize(param);
    }

    // update function for optimization
    inline void oplus_dual(const Vector3d& v){
      //construct a normal from azimuth and evelation;
      double _azimuth=v[0];
      double _elevation=0;
      double s=std::sin(_elevation), c=std::cos(_elevation);
      Vector3d n (c*std::cos(_azimuth), c*std::sin(_azimuth), s) ;
      
      // rotate the normal
      Matrix3d R=rotation(normal());
      double d=distance()+v[1];         // why is plus?
      param.head<3>() = R*n;
      param(3) = -d;
      normalize(param);

      mdDualDis += v[2];
    }

    inline Vector2d ominus_par(const plane& plane){
        //construct the rotation that would bring the plane normal in (1 0 0)
        Vector3d norm = normal();
        if(plane.normal().dot(norm) < 0)
            norm = -norm;
        Matrix3d R=rotation(norm).transpose();
        Vector3d n=R*plane.normal();  // 即旋转矩阵做差，获得欧拉角形式，各角度完美匹配时应为0

        return Vector2d(azimuth(n), elevation(n));
    }

    inline Vector3d ominus(const plane& plane){
        //construct the rotation that would bring the plane normal in (1 0 0)
        Matrix3d R=rotation(normal()).transpose();
        Vector3d n=R*plane.normal();
        double d=distance()-plane.distance();
        return Vector3d(azimuth(n), elevation(n), d);
    }

    static inline void normalize(Vector4d& coeffs) {
      double n=coeffs.head<3>().norm();
      coeffs = coeffs * (1./n);
    }

    Vector3d normal() const {
      return param.head<3>();
    }

    static Matrix3d rotation(const Vector3d& v)  {
      double _azimuth = azimuth(v);
      double _elevation = elevation(v); 
      return (AngleAxisd(_azimuth,  Vector3d::UnitZ())* AngleAxisd(- _elevation, Vector3d::UnitY())).toRotationMatrix();
    }

    // self
    double azimuth() const {
      return atan2(param[1], param[0]);
    }

    static double azimuth(const Vector3d& v) {
    return std::atan2(v(1),v(0));
    }

    static  double elevation(const Vector3d& v) {
    return std::atan2(v(2), v.head<2>().norm());
    }

    double distance() const {
      return -param(3);
    }

    friend plane operator*(const g2o::SE3Quat& t, const plane& plane);
    
    Eigen::Vector4d GeneratePlaneVec();
    Eigen::Vector4d GenerateAnotherPlaneVec();

    Vector3d SampleNearAnotherPoint(const Vector3d& point);

    // finite plane parameters; treat as a square
    double mdPlaneSize; // side length (meter)
    bool mbLimited;

    // dual plane parameter
    double mdDualDis;

    Vector4d param; // A B C : AX+BY+CZ+D=0
    Vector3d color; // r g b , [0,1.0]
    Vector3d mvPlaneCenter; // the center of the square. roughly defined.    

    int miMHType; // 0: not set; 1: Parallel ; 2: Vertical
    int miVisualGroup; // For Visualization only
private:

    Eigen::Vector3d GetLineFromCenterAngle(const Eigen::Vector2d center, double angle);
    Eigen::Vector4d LineToPlane(const Eigen::Vector3d line);

};

inline plane operator*(const g2o::SE3Quat& t, const plane& plane){
        g2o::plane pl = plane;
        pl.transform(t);
        return pl;
};

} // namespace g2o

