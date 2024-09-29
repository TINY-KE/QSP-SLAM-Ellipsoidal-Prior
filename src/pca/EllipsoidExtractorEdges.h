#pragma once

#include "Thirdparty/g2o/g2o/core/base_multi_edge.h"
#include "Thirdparty/g2o/g2o/types/types_six_dof_expmap.h"
#include "include/core/Ellipsoid.h"
#include <Eigen/Core>
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <core/BasicEllipsoidEdges.h>

namespace g2o {

// 3d plane constrain
class EdgeEllipsoidPlane : public BaseUnaryEdge<1, Vector4d, VertexEllipsoidXYZABC> {
  public:
    virtual bool read(std::istream &is);

    virtual bool write(std::ostream &os) const;

    void computeError();
};

class EdgeEllipsoidXYZABCYawPlane : public BaseUnaryEdge<1, Vector4d, VertexEllipsoidXYZABCYaw> {
  public:
    virtual bool read(std::istream &is);

    virtual bool write(std::ostream &os) const;

    void computeError();
};

// 3d plane constrain, with SE3
class EdgeSE3EllipsoidPlane : public BaseBinaryEdge<1, Vector4d, VertexSE3Expmap, VertexEllipsoidXYZABCYaw> {
  public:
    EdgeSE3EllipsoidPlane() : mbNormalDirection(false){};

    virtual bool read(std::istream &is);

    virtual bool write(std::ostream &os) const;

    void computeError();

    void setNormalDirection(bool bl);

  private:
    bool mbNormalDirection;
};

// 3d plane constrain, with SE3
// With Normal!
class EdgeSE3EllipsoidPlaneWithNormal : public BaseBinaryEdge<2, Vector4d, VertexSE3Expmap, VertexEllipsoidXYZABCYaw> {
  public:
    virtual bool read(std::istream &is);

    virtual bool write(std::ostream &os) const;

    void computeError();

  private:
    double calculateMinAngle(const Vector3d &normal, const Eigen::Quaterniond &quat);
    double calculateMinAngle3D(const Vector3d &normal, const Eigen::Quaterniond &quat);
};

// 3d partial plane constrain, with SE3
// for those planes lying on the borders, contributing to a part area
class EdgeSE3EllipsoidPlanePartial : public BaseBinaryEdge<1, Vector4d, VertexSE3Expmap, VertexEllipsoidXYZABCYaw> {
  public:
    virtual bool read(std::istream &is);

    virtual bool write(std::ostream &os) const;

    void computeError();
};

} // namespace g2o

// 一些几何函数
namespace ORB_SLAM2 {

bool JudgeCross(g2o::plane &pl, g2o::ellipsoid &e);

}
