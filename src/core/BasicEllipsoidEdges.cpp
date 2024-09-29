#include "include/core/BasicEllipsoidEdges.h"
#include "cmath"

namespace g2o
{
// ---- VertexEllipsoid
    void VertexEllipsoid::setToOriginImpl() { _estimate = ellipsoid(); }

    void VertexEllipsoid::oplusImpl(const double* update_) {
        Eigen::Map<const Vector9d> update(update_);
        setEstimate(_estimate.exp_update(update));
    }

    bool VertexEllipsoid::read(std::istream& is) {
        Vector9d est;
        for (int i=0; i<9; i++)
            is  >> est[i];
        ellipsoid Onecube;
        Onecube.fromMinimalVector(est);
        setEstimate(Onecube);
        return true;
    }

    bool VertexEllipsoid::write(std::ostream& os) const {
        Vector9d lv=_estimate.toMinimalVector();
        for (int i=0; i<lv.rows(); i++){
            os << lv[i] << " ";
        }
        return os.good();
    }


// ---- VertexEllipsoidXYZABC
    void VertexEllipsoidXYZABC::setToOriginImpl() { _estimate = ellipsoid(); }

    void VertexEllipsoidXYZABC::oplusImpl(const double* update_) {
        Eigen::Map<const Vector6d> update(update_);
        setEstimate(_estimate.exp_update_XYZABC(update));
    }

    bool VertexEllipsoidXYZABC::read(std::istream& is) {
        Vector9d est;
        for (int i=0; i<9; i++)
            is  >> est[i];
        ellipsoid Onecube;
        Onecube.fromMinimalVector(est);
        setEstimate(Onecube);
        return true;
    }

    bool VertexEllipsoidXYZABC::write(std::ostream& os) const {
        Vector9d lv=_estimate.toMinimalVector();
        for (int i=0; i<lv.rows(); i++){
            os << lv[i] << " ";
        }
        return os.good();
    }

// ---- VertexEllipsoidXYZABCYaw
    void VertexEllipsoidXYZABCYaw::setToOriginImpl() { _estimate = ellipsoid(); }

    void VertexEllipsoidXYZABCYaw::oplusImpl(const double* update_) {
        Eigen::Map<const Vector7d> update(update_);

        Vector7d update_vec = update;
        if(mbRotationOnly){
            Vector7d vec_rot;
            vec_rot << 0, 0, 0, 0, 0, 0, update_vec[6];
            update_vec = vec_rot;
            // for(int i=0;i<6;i++)
                // update.row(i)[0] = 0;
        }
        setEstimate(_estimate.exp_update_XYZABCYaw(update_vec));
    }

    bool VertexEllipsoidXYZABCYaw::read(std::istream& is) {
        Vector9d est;
        for (int i=0; i<9; i++)
            is  >> est[i];
        ellipsoid Onecube;
        Onecube.fromMinimalVector(est);
        setEstimate(Onecube);
        return true;
    }

    bool VertexEllipsoidXYZABCYaw::write(std::ostream& os) const {
        Vector9d lv=_estimate.toMinimalVector();
        for (int i=0; i<lv.rows(); i++){
            os << lv[i] << " ";
        }
        return os.good();
    }

    void VertexEllipsoidXYZABCYaw::SetRotationOnly(bool state)
    {
        mbRotationOnly = state;
    }

// 6 DOF 版本
    void VertexEllipsoidXYABHeightYaw::oplusImpl(const double* update_) {
        Eigen::Map<const Vector7d> update(update_);
        setEstimate(_estimate.exp_update_XYZABYaw(update.head(6)));
    }

// ---- VertexEllipsoidXYZ : For Nonparam Point Model
    void VertexEllipsoidXYZ::setToOriginImpl() { _estimate = ellipsoid(); }

    void VertexEllipsoidXYZ::oplusImpl(const double* update_) {
        Eigen::Map<const Vector3d> update(update_);
        Vector3d trans = _estimate.pose.translation() + update;
        _estimate.pose.setTranslation(trans);
    }

    bool VertexEllipsoidXYZ::read(std::istream& is) {
        Vector9d est;
        for (int i=0; i<9; i++)
            is  >> est[i];
        ellipsoid Onecube;
        Onecube.fromMinimalVector(est);
        setEstimate(Onecube);
        return true;
    }

    bool VertexEllipsoidXYZ::write(std::ostream& os) const {
        Vector9d lv=_estimate.toMinimalVector();
        for (int i=0; i<lv.rows(); i++){
            os << lv[i] << " ";
        }
        return os.good();
    }

    // EdgeEllipsoid9DOF
    bool EdgeSE3Ellipsoid9DOF::read(std::istream& is){
        return true;
    };

    bool EdgeSE3Ellipsoid9DOF::write(std::ostream& os) const
    {
        return os.good();
    };

    void EdgeSE3Ellipsoid9DOF::computeError()
    {
        const VertexSE3Expmap* SE3Vertex   = static_cast<const VertexSE3Expmap*>(_vertices[0]);  //  world to camera pose
        const VertexEllipsoid* cuboidVertex = static_cast<const VertexEllipsoid*>(_vertices[1]);       //  object pose to world

        SE3Quat cam_pose_Twc = SE3Vertex->estimate().inverse();
        ellipsoid global_cube = cuboidVertex->estimate();
        ellipsoid esti_global_cube = _measurement.transform_from(cam_pose_Twc);
        _error = global_cube.min_log_error_9dof(esti_global_cube);
    }


    // EdgeSE3Ellipsoid7DOF
    bool EdgeSE3Ellipsoid7DOF::read(std::istream& is){
        return true;
    };

    bool EdgeSE3Ellipsoid7DOF::write(std::ostream& os) const
    {
        return os.good();
    };

    void EdgeSE3Ellipsoid7DOF::computeError()
    {
        const VertexSE3Expmap* SE3Vertex   = static_cast<const VertexSE3Expmap*>(_vertices[0]);  //  world to camera pose
        const VertexEllipsoidXYZABCYaw* cuboidVertex = static_cast<const VertexEllipsoidXYZABCYaw*>(_vertices[1]);       //  object pose to world

        SE3Quat cam_pose_Twc = SE3Vertex->estimate().inverse();
        ellipsoid global_cube = cuboidVertex->estimate();
        ellipsoid esti_global_cube = _measurement.transform_from(cam_pose_Twc);

        // 测试： 直接使用 3d 完整误差中的一部分.
        
        // res.head<6>() = pose_diff.log();  // rpy xyz
        // res.tail<3>() = this->scale - newone.scale;  abc
        Vector9d vec_9 = global_cube.min_log_error_9dof(esti_global_cube);
        Vector7d vec_7;
        vec_7 << vec_9[3], vec_9[4], vec_9[5], vec_9.head(3).norm(), vec_9.tail(3);

        _error = vec_7;
    }

// EdgeSE3EllipsoidProj
    bool EdgeSE3EllipsoidProj::read(std::istream& is){
        return true;
    };

    bool EdgeSE3EllipsoidProj::write(std::ostream& os) const
    {
        return os.good();
    };

    Vector4d EdgeSE3EllipsoidProj::getProject(){
        const VertexSE3Expmap* SE3Vertex   = static_cast<const VertexSE3Expmap*>(_vertices[0]);  //  world to camera pose
        const VertexEllipsoid* cuboidVertex = static_cast<const VertexEllipsoid*>(_vertices[1]);       //  object pose to world

        SE3Quat cam_pose_Tcw = SE3Vertex->estimate();
        ellipsoid global_cube = cuboidVertex->estimate();

        Vector4d proj = global_cube.getBoundingBoxFromProjection(cam_pose_Tcw, Kalib); // topleft,  rightdown

        return proj;
    }

    void EdgeSE3EllipsoidProj::computeError()
    {
        Vector4d rect_project = getProject();

        _error = Vector4d(0,0,0,0);
        for(int i=0;i<4;i++)
        {
            if( _measurement[i] >= 0 )  // the invalid measurement has been set to -1.
                _error[i] = rect_project[i] - _measurement[i];
        }


        if( std::isnan(chi2()) )
        {
            std::cout << "rect_project : " << rect_project.transpose() << std::endl;
            std::cout << "Kalib : " << Kalib << std::endl;

            const VertexSE3Expmap* SE3Vertex   = static_cast<const VertexSE3Expmap*>(_vertices[0]);  //  world to camera pose
            const VertexEllipsoid* cuboidVertex = static_cast<const VertexEllipsoid*>(_vertices[1]);       //  object pose to world

            SE3Quat cam_pose_Tcw = SE3Vertex->estimate();
            ellipsoid global_cube = cuboidVertex->estimate();

            std::cout << "SE3-pose: " << cam_pose_Tcw.toXYZPRYVector().transpose() << std::endl;
            std::cout << "global_cube: " << global_cube.toMinimalVector().transpose() << std::endl;

            std::cout << "[Error : " << _error.transpose() << "]" << std::endl;

            std::cout << std::endl << std::endl;
        }
        
    }

    void EdgeSE3EllipsoidProj::setKalib(Matrix3d& K)
    {
        Kalib = K;
    }

    // Prior: objects should be vertical to their supporting planes
    bool EdgeEllipsoidGravityPlanePrior::read(std::istream& is){
        return true;
    };

    bool EdgeEllipsoidGravityPlanePrior::write(std::ostream& os) const
    {
        return os.good();
    };

    void EdgeEllipsoidGravityPlanePrior::computeError()
    {
        const VertexEllipsoid* cuboidVertex = static_cast<const VertexEllipsoid*>(_vertices[0]);
        ellipsoid global_cube = cuboidVertex->estimate();

        Eigen::Matrix3d rotMat = global_cube.pose.to_homogeneous_matrix().topLeftCorner(3,3);
        Vector3d zAxis = rotMat.col(2);

        Vector3d goundPlaneNormalAxis = _measurement.head(3);

        double cos_angle = zAxis.dot(goundPlaneNormalAxis)/zAxis.norm()/goundPlaneNormalAxis.norm();
        
        // prevent NaN
        if( cos_angle > 1 )
            cos_angle = cos_angle - 0.0001;
        else if( cos_angle < -1 )
            cos_angle = cos_angle + 0.0001;

        double angle = acos(cos_angle);

        double diff_angle = angle - 0;

        _error = Eigen::Matrix<double, 1, 1>(diff_angle);
    }

    // EdgeSE3EllipsoidCenter
    bool EdgeSE3EllipsoidCenter::read(std::istream& is){
        return true;
    };

    bool EdgeSE3EllipsoidCenter::write(std::ostream& os) const
    {
        return os.good();
    };

    void EdgeSE3EllipsoidCenter::computeError()
    {
        const VertexSE3Expmap* SE3Vertex   = static_cast<const VertexSE3Expmap*>(_vertices[0]);  //  world to camera pose
        const VertexEllipsoidXYZ* cuboidVertex = static_cast<const VertexEllipsoidXYZ*>(_vertices[1]);       //  object pose to world

        SE3Quat cam_pose_Twc = SE3Vertex->estimate().inverse();
        ellipsoid global_cube = cuboidVertex->estimate();
        ellipsoid esti_global_cube = _measurement.transform_from(cam_pose_Twc);

        Vector3d center_est = esti_global_cube.pose.translation();
        Vector3d center_vertex = global_cube.pose.translation();

        double error = (center_est - center_vertex).norm();
        // std::cout << "error center-ellipsoid : " << error << std::endl;
        _error = Eigen::Matrix<double, 1, 1>(error);
    }


    // EdgeSE3EllipsoidCenterFull
    bool EdgeSE3EllipsoidCenterFull::read(std::istream& is){
        return true;
    };

    bool EdgeSE3EllipsoidCenterFull::write(std::ostream& os) const
    {
        return os.good();
    };

    void EdgeSE3EllipsoidCenterFull::computeError()
    {
        const VertexSE3Expmap* SE3Vertex   = static_cast<const VertexSE3Expmap*>(_vertices[0]);  //  world to camera pose
        const VertexEllipsoidXYZABCYaw* cuboidVertex = static_cast<const VertexEllipsoidXYZABCYaw*>(_vertices[1]);       //  object pose to world

        SE3Quat cam_pose_Twc = SE3Vertex->estimate().inverse();
        ellipsoid global_cube = cuboidVertex->estimate();
        ellipsoid esti_global_cube = _measurement.transform_from(cam_pose_Twc);

        Vector3d center_est = esti_global_cube.pose.translation();
        Vector3d center_vertex = global_cube.pose.translation();

        double error = (center_est - center_vertex).norm();
        // std::cout << "error center-ellipsoid : " << error << std::endl;
        _error = Eigen::Matrix<double, 1, 1>(error);
    }


}// g2o