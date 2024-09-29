#include "EllipsoidExtractorEdges.h"
#include "include/core/Plane.h"

// ***************** DEBUG 可视化 ***************
#include "Map.h"
extern ORB_SLAM2::Map *expMap;

bool g_bVisualize = false;
// *********************************************

namespace g2o {
// Prior: objects should be vertical to their supporting planes
bool EdgeEllipsoidPlane::read(std::istream &is) {
    return true;
};

bool EdgeEllipsoidPlane::write(std::ostream &os) const {
    return os.good();
};

// 原始版本: 以 X^TQ*X=0 作为约束
// void EdgeEllipsoidPlane::computeError()
// {
//     const VertexEllipsoidXYZABC* cuboidVertex = static_cast<const VertexEllipsoidXYZABC*>(_vertices[0]);
//     ellipsoid e = cuboidVertex->estimate();

//     Eigen::Matrix4d QStar = e.generateQuadric();
//     Vector4d plane = _measurement;

//     double error = plane.transpose() * QStar * plane;

//     _error = Eigen::Matrix<double, 1, 1>(error);
// }

void GetNearestAndFarthestPointOnEllipsoidToPlane(g2o::plane &pl, g2o::ellipsoid &e,
                                                  Vector3d &nearestP, double &nearestDis, Vector3d &farthestP, double &farthestDis) {
    g2o::SE3Quat Tew = e.pose.inverse();

    g2o::ellipsoid e_e = e.transform_from(Tew);
    g2o::plane ple = pl;
    ple.transform(Tew);

    // AX+BY+CZ+D = 0
    Vector4d plane_e = ple.param;
    double A = plane_e[0];
    double B = plane_e[1];
    double C = plane_e[2];
    double D = plane_e[3];

    // 获得椭球体参数向量形式
    // ax^2+bxy+c^..... = 0;
    Matrix4d Q_star = e_e.generateQuadric();
    Matrix4d Q = Q_star.inverse();    // 系数矩阵

    // 获得系数
    // ax^2+by^2+cz^2+dxy+eyz+fxz+gx+hy+iz+j=0
    // 正椭球体 check
    // x^2/a^2 + y^2/b^2 + z^2/c^2 = 1
    Q = Q / (-Q(3, 3));

    // 等价于求解后面拉格朗日方程的极值： f(x,y,z) = AX+BY+CZ+D - alpha_pos(x^2/a^2 + y^2/b^2 + z^2/c^2 - 1)

    // 对角线前3个 必须大于0, 其他项必须为0
    // std::cout << "Origin Q check : " << std::endl << Q << std::endl;

    double a_2 = 1 / Q(0, 0);
    double b_2 = 1 / Q(1, 1);
    double c_2 = 1 / Q(2, 2);

    double alpha_2 = 4 / (A * A * a_2 + B * B * b_2 + C * C * c_2);
    double alpha_pos = sqrt(alpha_2);

    double x_coeff = A * a_2 / 2;
    double y_coeff = B * b_2 / 2;
    double z_coeff = C * c_2 / 2;
    Vector3d coeff_vec(x_coeff, y_coeff, z_coeff);

    Vector3d extrema_1 = alpha_pos * coeff_vec;
    Vector3d extrema_2 = -extrema_1;

    double dis1 = ple.distanceToPoint(extrema_1);
    double dis2 = ple.distanceToPoint(extrema_2);

    if (std::abs(dis1) > std::abs(dis2)) {
        nearestDis = dis2;
        nearestP = extrema_2;

        farthestDis = dis1;
        farthestP = extrema_1;
    } else {
        nearestDis = dis1;
        nearestP = extrema_1;

        farthestDis = dis2;
        farthestP = extrema_2;
    }

    // std::cout << "nearestP : " << nearestP.transpose() << std::endl;
    // std::cout << "nearestDis : " << nearestDis << std::endl;
    // std::cout << "farthestP : " << farthestP.transpose() << std::endl;
    // std::cout << "farthestDis : " << farthestDis << std::endl;

    return;
}

double GetNearestPointOnEllipsoidToPlane(g2o::plane &pl, g2o::ellipsoid &e, Vector3d &point) {
    double nD, fD;
    Vector3d nP, fP;
    GetNearestAndFarthestPointOnEllipsoidToPlane(pl, e, nP, nD, fP, fD);

    point = nP;

    // *********** 开始可视化部分 ************
    if (g_bVisualize) {
        // 将 点由椭球体的局部系 转换到世界系
        g2o::SE3Quat Twe = e.pose;
        Matrix4d transformMat = Twe.to_homogeneous_matrix();
        Vector3d point_w = homo_to_real_coord_vec<double>(transformMat * real_to_homo_coord_vec<double>(point));

        // 1) 可视化这个最近点.
        ORB_SLAM2::PointCloud *pCloud = new ORB_SLAM2::PointCloud;
        ORB_SLAM2::PointXYZRGB p;
        p.x = point_w[0];
        p.y = point_w[1];
        p.z = point_w[2];
        p.size = 10;

        p.r = 255;
        p.g = 0;
        p.b = 0;
        pCloud->push_back(p);
        expMap->AddPointCloudList("Debug.DistancePoint", pCloud, 1);

        // 2) 连接最近点到平面的点.
        // todo

        // 3) 可视化文字表示距离
        g_bVisualize = false; // 只做一次
    }
    // *************************************

    return nD;
}

double GetFarthestPointOnEllipsoidToPlane(g2o::plane &pl, g2o::ellipsoid &e, Vector3d &point) {
    double nD, fD;
    Vector3d nP, fP;
    GetNearestAndFarthestPointOnEllipsoidToPlane(pl, e, nP, nD, fP, fD);

    point = fP;
    return fD;
}

double distanceFromPlaneToEllipsoid(g2o::plane &pl, g2o::ellipsoid &e) {
    Vector3d point;
    return GetNearestPointOnEllipsoidToPlane(pl, e, point);
}

double FarthestDistanceFromPlaneToEllipsoid(g2o::plane &pl, g2o::ellipsoid &e) {
    Vector3d point;
    return GetFarthestPointOnEllipsoidToPlane(pl, e, point);
}

// 输入: 平面
// !!!! 高等级注意: [需要确保平面法向量方向指向椭球体中心]
double GetDistanceWithDirection(g2o::plane &pl, g2o::ellipsoid &e) {
    // 1) 若当前椭球体中心与法向量方向一致, 直接返回最小距离
    if (pl.distanceToPoint(e.pose.translation(), true) > 0)
        return distanceFromPlaneToEllipsoid(pl, e);
    // 2) 若不一致, 应该返回最大距离
    else
        return FarthestDistanceFromPlaneToEllipsoid(pl, e);
}

// 新版本: 在欧几里得空间，以距离作为约束.
void EdgeEllipsoidPlane::computeError() {
    const VertexEllipsoidXYZABC *cuboidVertex = static_cast<const VertexEllipsoidXYZABC *>(_vertices[0]);
    ellipsoid e = cuboidVertex->estimate();

    Eigen::Matrix4d QStar = e.generateQuadric();
    Vector4d plane = _measurement;
    g2o::plane pl(plane);

    // 此处尝试计算距离
    double dis = distanceFromPlaneToEllipsoid(pl, e);
    //

    _error = Eigen::Matrix<double, 1, 1>(dis);
}

bool EdgeSE3EllipsoidPlane::read(std::istream &is) {
    return true;
};

bool EdgeSE3EllipsoidPlane::write(std::ostream &os) const {
    return os.good();
};

void EdgeSE3EllipsoidPlane::computeError() {
    const VertexSE3Expmap *SE3Vertex = static_cast<const VertexSE3Expmap *>(_vertices[0]);
    const VertexEllipsoidXYZABCYaw *ellipsoidVertex = static_cast<const VertexEllipsoidXYZABCYaw *>(_vertices[1]);
    g2o::SE3Quat Twc = SE3Vertex->estimate().inverse();
    ellipsoid e = ellipsoidVertex->estimate();

    Eigen::Matrix4d QStar = e.generateQuadric();
    Vector4d plane = _measurement;
    g2o::plane pl(plane);
    pl.transform(Twc);

    // 此处尝试计算距离
    double dis;
    if (mbNormalDirection)
        dis = GetDistanceWithDirection(pl, e); // 若开启了则考虑其方向
    else
        dis = distanceFromPlaneToEllipsoid(pl, e);

    // nan check
    bool b_output = true;
    if (std::isnan(dis)) {
        dis = 0;

        if (b_output) {
            std::cout << "[computeError, NaN] pl " << pl.param.transpose() << ", e: " << e.toMinimalVector().transpose() << std::endl;
        }
    }

    _error = Eigen::Matrix<double, 1, 1>(dis);
}

bool EdgeEllipsoidXYZABCYawPlane::read(std::istream &is) {
    return true;
};

bool EdgeEllipsoidXYZABCYawPlane::write(std::ostream &os) const {
    return os.good();
};

void EdgeEllipsoidXYZABCYawPlane::computeError() {
    const VertexEllipsoidXYZABCYaw *cuboidVertex = static_cast<const VertexEllipsoidXYZABCYaw *>(_vertices[0]);
    ellipsoid e = cuboidVertex->estimate();

    Eigen::Matrix4d QStar = e.generateQuadric();
    Vector4d plane = _measurement;
    g2o::plane pl(plane);

    // 此处尝试计算距离
    double dis = distanceFromPlaneToEllipsoid(pl, e);
    //

    _error = Eigen::Matrix<double, 1, 1>(dis);
}

void EdgeSE3EllipsoidPlane::setNormalDirection(bool bl) {
    mbNormalDirection = bl;
}

// 3d plane constrain with normal
bool EdgeSE3EllipsoidPlaneWithNormal::read(std::istream &is) {
    return true;
};

bool EdgeSE3EllipsoidPlaneWithNormal::write(std::ostream &os) const {
    return os.good();
};

// 3D 版本的角度约束
double EdgeSE3EllipsoidPlaneWithNormal::calculateMinAngle3D(const Vector3d &normal, const Eigen::Quaterniond &quat) {
    // 取最小转角使得可以与 quat 的某一主轴方向对齐
    Matrix3d rotMat = quat.toRotationMatrix(); // Rwc

    // normal : Nwn   - >   Ncn
    Vector3d Nc = rotMat.inverse() * normal;

    // 版本1 : 与z轴求三维空间的夹角
    Vector3d ZAxis(0, 0, 1);
    // 获得夹角
    double cos_angle = Nc.transpose() * ZAxis;
    cos_angle = cos_angle / Nc.norm() / ZAxis.norm();
    double angle = acos(cos_angle); // [0,pi]

    // 归一化该angle.
    // 输出 angle 与 0, M_PI/2, M_PI 的最近距离
    double angle_norm = MIN(MIN(angle, std::abs(angle - M_PI / 2)), std::abs(angle - M_PI));

    // // 输出5个debug
    // static int output_count = 0;
    // if(output_count < 5)
    // {
    //     output_count++;
    //     std::cout << "Nc : " << Nc.transpose() << std::endl;
    //     std::cout << "angle : " << angle << std::endl;
    //     std::cout << "angle_norm : " << angle_norm << std::endl;
    //     std::cout << "angle_norm(deg) : " << angle_norm/M_PI*180 << std::endl;
    // }

    return angle_norm;
}

double EdgeSE3EllipsoidPlaneWithNormal::calculateMinAngle(const Vector3d &normal, const Eigen::Quaterniond &quat) {
    // 取最小转角使得可以与 quat 的某一主轴方向对齐
    Matrix3d rotMat = quat.toRotationMatrix(); // Rwc

    // normal : Nwn   - >   Ncn
    Vector3d Nc = rotMat.inverse() * normal;

    // 版本1 : 与z轴求三维空间的夹角
    // Vector3d ZAxis(0,0,1);
    // // 获得夹角
    // double cos_angle = Nc.transpose() * ZAxis;
    // cos_angle = cos_angle / Nc.norm() / ZAxis.norm();
    // double angle = acos(cos_angle);  // [0,pi]

    // 版本2 ： 投影到 xy 平面后，与x轴求二维夹角

    // 首先判断是否法向量与z轴平行，此时不发生角度约束.
    // thresh : 30 deg
    Vector3d ZAxis(0, 0, 1);
    double cos_angle_z = Nc.transpose() * ZAxis;
    cos_angle_z = cos_angle_z / Nc.norm() / ZAxis.norm();
    double angle_z = acos(cos_angle_z); // [0,pi]
    double angle_norm_z = MIN(std::abs(angle_z), std::abs(angle_z - M_PI));
    if (angle_norm_z < M_PI / 180.0 * 30)
        return 0;

    Vector3d XAxis(1, 0, 0);
    Vector3d Nc_xy = Nc;
    Nc_xy[2] = 0; // let z be zero.
    double cos_angle = Nc_xy.transpose() * XAxis;
    cos_angle = cos_angle / Nc_xy.norm() / XAxis.norm();
    double angle = acos(cos_angle); // [0,pi]

    // 归一化该angle.
    // 输出 angle 与 0, M_PI/2, M_PI 的最近距离
    double angle_norm = MIN(MIN(angle, std::abs(angle - M_PI / 2)), std::abs(angle - M_PI));

    // 输出5个debug
    // static int output_count = 0;
    // if(output_count < 5)
    // {
    //     output_count++;
    //     std::cout << "Nc : " << Nc.transpose() << std::endl;
    //     std::cout << "angle : " << angle << std::endl;
    //     std::cout << "angle_norm : " << angle_norm << std::endl;
    //     std::cout << "angle_norm(deg) : " << angle_norm/M_PI*180 << std::endl;
    // }

    // 做一个 NaN 判断
    static int nan_angle_count = 0;
    if (isnan(angle_norm)) {
        std::cout << nan_angle_count << " [ NAN ANGLE ] " << angle_norm << std::endl;
        nan_angle_count++;
        std::cout << "rotMat : " << rotMat << std::endl;
        std::cout << "normal : " << normal.transpose() << std::endl;
        std::cout << "Nc : " << Nc.transpose() << std::endl;
        std::cout << "angle : " << angle << std::endl;
        std::cout << "angle_norm : " << angle_norm << std::endl;
        std::cout << "angle_norm(deg) : " << angle_norm / M_PI * 180 << std::endl;
    }

    return angle_norm;
}

void EdgeSE3EllipsoidPlaneWithNormal::computeError() {
    const VertexSE3Expmap *SE3Vertex = static_cast<const VertexSE3Expmap *>(_vertices[0]);
    const VertexEllipsoidXYZABCYaw *ellipsoidVertex = static_cast<const VertexEllipsoidXYZABCYaw *>(_vertices[1]);
    g2o::SE3Quat Twc = SE3Vertex->estimate().inverse();
    ellipsoid e = ellipsoidVertex->estimate();

    Eigen::Matrix4d QStar = e.generateQuadric();
    Vector4d plane = _measurement;
    g2o::plane pl(plane);
    pl.transform(Twc);

    // 此处尝试计算距离
    double dis = distanceFromPlaneToEllipsoid(pl, e);

    // double dis_angle = calculateMinAngle3D(pl.normal(), e.pose.rotation()); // 计算平面法向量与旋转矩阵的最小夹角
    double dis_angle = calculateMinAngle(pl.normal(), e.pose.rotation()); // 计算平面法向量与旋转矩阵的最小夹角

    _error = Eigen::Matrix<double, 2, 1>(dis, dis_angle);
}

bool EdgeSE3EllipsoidPlanePartial::read(std::istream &is) {
    return true;
};

bool EdgeSE3EllipsoidPlanePartial::write(std::ostream &os) const {
    return os.good();
};

void EdgeSE3EllipsoidPlanePartial::computeError() {
    const VertexSE3Expmap *SE3Vertex = static_cast<const VertexSE3Expmap *>(_vertices[0]);
    const VertexEllipsoidXYZABCYaw *ellipsoidVertex = static_cast<const VertexEllipsoidXYZABCYaw *>(_vertices[1]);
    g2o::SE3Quat Twc = SE3Vertex->estimate().inverse();
    ellipsoid e = ellipsoidVertex->estimate();

    Eigen::Matrix4d QStar = e.generateQuadric();
    Vector4d plane = _measurement;
    g2o::plane pl(plane);
    pl.transform(Twc);

    double error;

    // 1) 判断平面是否与椭球体相交
    bool bCross = ORB_SLAM2::JudgeCross(pl, e);
    // std::cout << "bCross : " << bCross << std::endl;

    // 1.1 若相交, 则error为0
    if (bCross)
        error = 0;
    else {
        // DEBUG VISUALIZE
        g_bVisualize = true;

        // 不相交, 求带符号的距离.
        error = GetDistanceWithDirection(pl, e);
        // std::cout << "dis with direction:" << error << std::endl;
    }

    // 引入作为完整观测的概率
    double error_full = distanceFromPlaneToEllipsoid(pl, e);
    error = 0.9 * error + 0.1 * error_full;

    // 根据距离做截断.
    _error = Eigen::Matrix<double, 1, 1>(error);
}

} // namespace g2o

namespace ORB_SLAM2 {

// 传入坐标系: 都为世界坐标系
bool JudgeCross(g2o::plane &pl, g2o::ellipsoid &e) {
    // 判断方式:
    // 判断最近点与原点与平面关系. 若在一侧, 则外部; 若不在一侧, 则说明相交.
    Vector3d center = e.pose.translation();

    // 获得椭球体上到平面的最近点
    Vector3d pointOnEllipsoidNearPlane; // 椭球体坐标系
    double dis = g2o::GetNearestPointOnEllipsoidToPlane(pl, e, pointOnEllipsoidNearPlane);

    // 转到世界系
    // 将 点由椭球体的局部系 转换到世界系
    g2o::SE3Quat Twe = e.pose;
    Matrix4d transformMat = Twe.to_homogeneous_matrix();
    Vector3d point_w = homo_to_real_coord_vec<double>(transformMat * real_to_homo_coord_vec<double>(pointOnEllipsoidNearPlane));

    bool flag_center = (pl.distanceToPoint(center, true) > 0);
    bool flag_poe = (pl.distanceToPoint(point_w, true) > 0);

    if (flag_center == flag_poe) {
        // 同一侧, 不相交
        return false;
    } else {
        // 不同一侧, 说明相交
        return true;
    }
}

} // namespace ORB_SLAM2