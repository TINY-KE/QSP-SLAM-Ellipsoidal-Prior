
#ifndef PLANEVERTEXEDGES_H
#define PLANEVERTEXEDGES_H

#include "include/core/Plane.h"
#include "Thirdparty/g2o/g2o/core/base_vertex.h"
#include "Thirdparty/g2o/g2o/core/block_solver.h"
#include "Thirdparty/g2o/g2o/core/optimization_algorithm_levenberg.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_eigen.h"
#include "Thirdparty/g2o/g2o/types/types_six_dof_expmap.h"
#include "Thirdparty/g2o/g2o/core/robust_kernel_impl.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_dense.h"
#include "Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"

namespace g2o
{
    // Vertex 3自由度
    class VertexPlane3DOF:public BaseVertex<3,g2o::plane> 
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        VertexPlane3DOF(){};

        virtual void setToOriginImpl()
        {
            _estimate = g2o::plane();
        };

        virtual void oplusImpl(const double* update_) {
            Eigen::Map<const Vector3d> update(update_);
            _estimate.oplus(update);  
        };

        virtual bool read(std::istream& is) {return true;};
        virtual bool write(std::ostream& os) const {return os.good();};

    };

    // Edges : 3D空间的两平行平面; 角度差应为0.
    class EdgeTwoParPlanes : public BaseBinaryEdge<2, double, VertexPlane3DOF, VertexPlane3DOF> {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        EdgeTwoParPlanes(){};
        void computeError()
        {
            const VertexPlane3DOF* planeVertex2 = static_cast<const VertexPlane3DOF*>(_vertices[1]);
            const VertexPlane3DOF* planeVertex1 = static_cast<const VertexPlane3DOF*>(_vertices[0]);

            plane plane1 = planeVertex1->estimate();
            const plane& plane2 = planeVertex2->estimate();
            // measurement function: remap the plane in global coordinates
            _error = plane1.ominus_par(plane2);
        };

        virtual bool read(std::istream& is) {return true;};
        virtual bool write(std::ostream& os) const {return os.good();};

    };

    // Edges: 局部观测-世界平面; 完整约束, 角度与距离都相同
    class EdgePlaneSE3 : public BaseBinaryEdge<3, plane, VertexPlane3DOF, VertexSE3Expmap> {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        EdgePlaneSE3(){};
        void computeError()
        {
            const VertexSE3Expmap* poseVertex = static_cast<const VertexSE3Expmap*>(_vertices[1]);
            const VertexPlane3DOF* planeVertex = static_cast<const VertexPlane3DOF*>(_vertices[0]);

            const plane& plane = planeVertex->estimate();
            // measurement function: remap the plane in global coordinates
            const g2o::SE3Quat& Tcw = poseVertex->estimate();
            g2o::plane localPlane = Tcw*plane;

            _error = localPlane.ominus(_measurement);
        };

        void setMeasurement(const plane& m){
            _measurement = m;
        };

        virtual bool read(std::istream& is) {return true;};
        virtual bool write(std::ostream& os) const {return os.good();};

    };

} 

#endif // PLANEVERTEXEDGES_H