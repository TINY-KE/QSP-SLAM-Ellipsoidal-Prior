#include "EllipsoidExtractor.h"
#include "src/config/Config.h"

#include <pcl/common/centroid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/point_types.h>

#include "include/core/ConstrainPlane.h"
#include "include/utils/dataprocess_utils.h"

#include "include/core/PriorInfer.h"

namespace ORB_SLAM2 {
void VisualizeConstrainPlanes(g2o::ellipsoid &e_local, g2o::SE3Quat &Twc, Map *pMap) {
    std::vector<g2o::ConstrainPlane *> &vCPlanes = e_local.mvCPlanesInCamera;
    g2o::ellipsoid e_global = e_local.transform_from(Twc);
    Vector3d center = e_global.pose.translation();
    int planeNum = vCPlanes.size();

    // 考虑仅仅包含bbox平面的情况
    // bool bOnlyBbox = (planeNum == 4);

    // 顺序 前, 后, 0, 1, 2, 3
    for (int i = 0; i < planeNum; i++) {
        Vector4d planeVec = vCPlanes[i]->pPlane->param.head(4);
        Vector3d color;
        double plane_size = 0.5;
        // if(!bOnlyBbox && (i==0 || i==1)){
        // color = Vector3d(0,1.0,1.0); // 前后平面为蓝色
        // plane_size = 0.3;
        // }
        // else
        {
            // bbox平面
            plane_size = e_local.scale.norm() / 2.0;
            if (vCPlanes[i]->image_border)
                color = Vector3d(0.7, 0, 0); // 边界颜色
            else
                color = Vector3d(0.3, 1, 0); // 非边界
        }
        g2o::plane *pPlane = new g2o::plane(planeVec, color);
        pPlane->transform(Twc);
        pPlane->InitFinitePlane(center, plane_size);
        pMap->addPlane(pPlane);
    }

    return;
}

Matrix3Xd generateProjectionMatrix(const SE3Quat &campose_cw, const Matrix3d &Kalib) {
    Matrix3Xd identity_lefttop;
    identity_lefttop.resize(3, 4);
    identity_lefttop.col(3) = Vector3d(0, 0, 0);
    identity_lefttop.topLeftCorner<3, 3>() = Matrix3d::Identity(3, 3);

    Matrix3Xd proj_mat = Kalib * identity_lefttop;

    proj_mat = proj_mat * campose_cw.to_homogeneous_matrix();

    return proj_mat;
}

MatrixXd fromDetectionsToLines(Vector4d &detections) {
    bool flag_openFilter = false; // filter those lines lying on the image boundary

    double x1 = detections(0);
    double y1 = detections(1);
    double x2 = detections(2);
    double y2 = detections(3);

    Vector3d line1(1, 0, -x1);
    Vector3d line2(0, 1, -y1);
    Vector3d line3(1, 0, -x2);
    Vector3d line4(0, 1, -y2);

    // those lying on the image boundary have been marked -1
    MatrixXd line_selected(3, 0);
    MatrixXd line_selected_none(3, 0);

    int config_border_pixel = 10;
    int miImageCols = Config::Get<int>("Camera.width");
    int miImageRows = Config::Get<int>("Camera.height");
    if (!flag_openFilter || (x1 > config_border_pixel && x1 < miImageCols - config_border_pixel)) {
        line_selected.conservativeResize(3, line_selected.cols() + 1);
        line_selected.col(line_selected.cols() - 1) = line1;
    }
    if (!flag_openFilter || (y1 > config_border_pixel && y1 < miImageRows - config_border_pixel)) {
        line_selected.conservativeResize(3, line_selected.cols() + 1);
        line_selected.col(line_selected.cols() - 1) = line2;
    }
    if (!flag_openFilter || (x2 > config_border_pixel && x2 < miImageCols - config_border_pixel)) {
        line_selected.conservativeResize(3, line_selected.cols() + 1);
        line_selected.col(line_selected.cols() - 1) = line3;
    }
    if (!flag_openFilter || (y2 > config_border_pixel && y2 < miImageRows - config_border_pixel)) {
        line_selected.conservativeResize(3, line_selected.cols() + 1);
        line_selected.col(line_selected.cols() - 1) = line4;
    }

    return line_selected;
}

Matrix3d CameraToCalibMatrix(camera_intrinsic &camera) {
    Matrix3d calib;
    calib << camera.fx, 0, camera.cx,
        0, camera.fy, camera.cy,
        0, 0, 1;
    return calib;
}

MatrixXd GenerateBboxPlanes(g2o::SE3Quat &campose_wc, Eigen::Vector4d &bbox, Matrix3d &calib) {
    MatrixXd planes_all(4, 0);
    // std::cout << " [debug] calib : \n " << calib << std::endl;
    // get projection matrix
    MatrixXd P = generateProjectionMatrix(campose_wc.inverse(), calib);

    MatrixXd lines = fromDetectionsToLines(bbox);
    MatrixXd planes = P.transpose() * lines;

    // add to matrix
    for (int m = 0; m < planes.cols(); m++) {
        planes_all.conservativeResize(planes_all.rows(), planes_all.cols() + 1);
        planes_all.col(planes_all.cols() - 1) = planes.col(m);
    }

    return planes_all;
}

MatrixXd getVectorFromPlanesHomo(MatrixXd &planes) {
    int cols = planes.cols();
    MatrixXd planes_vector(10, 0);

    for (int i = 0; i < cols; i++) {
        VectorXd p = planes.col(i);
        Vector10d v;

        v << p(0) * p(0), 2 * p(0) * p(1), 2 * p(0) * p(2), 2 * p(0) * p(3), p(1) * p(1), 2 * p(1) * p(2), 2 * p(1) * p(3), p(2) * p(2), 2 * p(2) * p(3), p(3) * p(3);

        planes_vector.conservativeResize(planes_vector.rows(), planes_vector.cols() + 1);
        planes_vector.col(planes_vector.cols() - 1) = v;
    }

    return planes_vector;
}

Matrix4d getQStarFromVectors(MatrixXd &planeVecs) {
    MatrixXd A = planeVecs.transpose();

    // svd decompose
    JacobiSVD<Eigen::MatrixXd> svd(A, ComputeThinU | ComputeThinV);
    MatrixXd V = svd.matrixV();

    VectorXd qj_hat = V.col(V.cols() - 1);

    // Get QStar
    Matrix4d QStar;
    QStar << qj_hat(0), qj_hat(1), qj_hat(2), qj_hat(3),
        qj_hat(1), qj_hat(4), qj_hat(5), qj_hat(6),
        qj_hat(2), qj_hat(5), qj_hat(7), qj_hat(8),
        qj_hat(3), qj_hat(6), qj_hat(8), qj_hat(9);

    return QStar;
}

bool getEllipsoidFromQStar(Matrix4d &QStar, g2o::ellipsoid &eOut) {
    g2o::ellipsoid e;
    bool mbResult = false;

    Matrix4d Q = QStar.inverse() * cbrt(QStar.determinant());

    SelfAdjointEigenSolver<Matrix4d> es(Q); // ascending order by default
    MatrixXd D = es.eigenvalues().asDiagonal();
    MatrixXd V = es.eigenvectors();

    VectorXd eigens = es.eigenvalues();

    // For an ellipsoid, the signs of the eigenvalues must be ---+ or +++-
    int num_pos = int(eigens(0) > 0) + int(eigens(1) > 0) + int(eigens(2) > 0) + int(eigens(3) > 0);
    int num_neg = int(eigens(0) < 0) + int(eigens(1) < 0) + int(eigens(2) < 0) + int(eigens(3) < 0);
    if (!(num_pos == 3 && num_neg == 1) && !(num_pos == 1 && num_neg == 3)) {
        cout << " Not Ellipsoid : pos/neg  " << num_pos << " / " << num_neg << endl;
        cout << "eigens :" << eigens.transpose() << endl;
        mbResult = false;
        return mbResult;
    } else
        mbResult = true;

    if (eigens(3) > 0) // normalize to - - - +
    {
        Q = -Q;
        SelfAdjointEigenSolver<Matrix4d> es_2(Q);
        D = es_2.eigenvalues().asDiagonal();
        V = es_2.eigenvectors();

        eigens = es_2.eigenvalues();
    }

    // Solve ellipsoid parameters from matrix Q
    Vector3d lambda_mat = eigens.head(3).array().inverse();

    Matrix3d Q33 = Q.block(0, 0, 3, 3);

    double k = Q.determinant() / Q33.determinant();

    Vector3d value = -k * (lambda_mat);
    Vector3d s = value.array().abs().sqrt();

    Vector4d t = QStar.col(3);
    t = t / t(3);
    Vector3d translation = t.head(3);

    SelfAdjointEigenSolver<Matrix3d> es2(Q33);
    MatrixXd D_Q33 = es2.eigenvalues().asDiagonal();
    MatrixXd rot = es2.eigenvectors();

    double r, p, y;
    rot_to_euler_zyx<double>(rot, r, p, y);
    Vector3d rpy(r, p, y);

    // generate ellipsoid
    Vector9d objectVec;
    objectVec << t(0), t(1), t(2), rpy(0), rpy(1), rpy(2), s(0), s(1), s(2);
    e.fromMinimalVector(objectVec);

    eOut = e;
    return mbResult;
}

g2o::ellipsoid GetEllipsoidFromPlanes(MatrixXd &mPlanesParam) {
    // svd 分解
    // Using SVD to generate a quadric
    MatrixXd planesVector = getVectorFromPlanesHomo(mPlanesParam);
    Matrix4d QStar = getQStarFromVectors(planesVector);

    // generate ellipsoid from the quadric
    g2o::ellipsoid e;
    bool result = getEllipsoidFromQStar(QStar, e);

    // Set color : blue.
    Eigen::Vector3d color(0, 0.3, 1.0);
    e.setColor(color);

    return e;
}

g2o::plane *selectBackPlane(std::vector<g2o::plane *> &vecPlanes) {
    Vector3d axis(0, 0, 1);
    int id = -1;
    double max_angle = -CV_PI;
    int num = vecPlanes.size();
    for (int i = 0; i < num; i++) {
        g2o::plane *pl = vecPlanes[i];
        Vector3d normal_p = pl->param.head(3);
        double cos_angle = normal_p.transpose() * axis;
        cos_angle = cos_angle / axis.norm() / normal_p.norm();
        double angle = acos(cos_angle);
        if (angle > max_angle) {
            id = i;
            max_angle = angle;
        }
    }
    if (id < 0) {
        std::cerr << "error in finding angle diff. please check the normal of the vecPlanes!" << std::endl;
        std::cout << "back id : " << id << std::endl;
        abort();
    }
    return vecPlanes[id];
}

g2o::plane *selectForwardPlane(std::vector<g2o::plane *> &vecPlanes) {
    Vector3d axis(0, 0, 1);
    int id = -1;
    double min_angle = CV_PI;
    int num = vecPlanes.size();
    for (int i = 0; i < num; i++) {
        g2o::plane *pl = vecPlanes[i];
        Vector3d normal_p = pl->param.head(3);
        double cos_angle = normal_p.transpose() * axis;
        cos_angle = cos_angle / axis.norm() / normal_p.norm();
        double angle = acos(cos_angle);
        if (angle < min_angle) {
            id = i;
            min_angle = angle;
        }
    }
    if (id < 0) {
        std::cerr << "error in finding angle diff. please check the normal of the vecPlanes!" << std::endl;
        std::cout << "forward id : " << id << std::endl;
        abort();
    }
    return vecPlanes[id];
}

PCAResult EllipsoidExtractor::PCA(pcl::PointCloud<PointType>::Ptr &pCloudPCL) {
    // process the principle components analysis to get the rotation matrix, scale, and center point of the point cloud
    PCAResult data = ProcessPCA(pCloudPCL);
    // adjust the rotation matrix to be right-handed
    AdjustChirality(data);
    // adjust the x,y,z order
    AlignZAxisToGravity(data);
    // align z axis with the normal of the supporting plane
    ApplyGravityPrior(data);

    return data;
}

void VisualizeNormals(std::vector<cv::Point2d> &vps) {
    // 使用直方图 / cv 图像直接可视化得了.
    cv::Mat im(100, 100, CV_8UC3, cv::Scalar(255, 0, 0));

    int num = vps.size();
    for (int i = 0; i < num; i++) {
        int x = round(vps[i].x * 100);
        int y = round(vps[i].y * 100);
        im.at<cv::Vec3b>(y, x) = cv::Vec3b(0, 255, 0);
    }

    std::cout << "normal num : " << num << std::endl;
    // cv::imshow("hst", im);
    // cv::waitKey();

    // 暂时将该检测保存到文件中去.

    return;
}

// please make sure groundplane has been set.
double EllipsoidExtractor::NormalVoter(pcl::PointCloud<PointType>::Ptr &pCloudPCL) {
    // pCloudPCL 重力坐标系 ( Z 轴为重力方向, 物体为正 )
    // Create the normal estimation class, and pass the input dataset to it
    pcl::NormalEstimation<PointType, pcl::Normal> ne;
    ne.setInputCloud(pCloudPCL);

    // Create an empty kdtree representation, and pass it to the normal estimation object.
    // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
    // 创建一个空的KD树，将它传递给法向量估计对象
    // 将基于输入的数据集在对象内部填充KD树的内容
    pcl::search::KdTree<PointType>::Ptr tree(new pcl::search::KdTree<PointType>());
    ne.setSearchMethod(tree);

    // Output datasets
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);

    // // Use all neighbors in a sphere of radius 3cm
    // ne.setRadiusSearch (0.03);
    int KSearchNum = Config::Get<int>("EllipsoidExtractor.Rotation.NormalVoter.KSearchNum");
    if (KSearchNum <= 0)
        KSearchNum = 30;
    // std::cout << "[debug] KSearchNum: " << KSearchNum << std::endl;
    ne.setKSearch(30);

    // 计算得到法向量结果，全部投影到xy平面上（可以绘制）
    // 对这些xy向量计算角度，在[-pi,+pi]之间进行投票获得直方图（绘制并显示）
    // 最终选出投票最多的，计算对应的偏航角
    ne.compute(*cloud_normals);
    int normal_num = cloud_normals->size();
    std::vector<cv::Point2d> vps;
    vps.resize(normal_num);
    for (int i = 0; i < normal_num; i++) {
        pcl::Normal pn = (*cloud_normals)[i];
        Vector3d pnormal(pn.normal_x, pn.normal_y, pn.normal_z);
        cv::Point2d p_xy(pnormal[0], pnormal[1]);
        vps[i] = p_xy;
    }
    // VisualizeNormals(vps);

    // 从vps中获得直方图.
    int bin_size = 360;
    std::vector<int> bins_num;
    bins_num.resize(bin_size); // 1 deg 1 bin
    fill(bins_num.begin(), bins_num.end(), 0);
    for (int i = 0; i < normal_num; i++) {
        cv::Point2d n_xy = vps[i];
        double yaw = atan2(n_xy.y, n_xy.x); // [-pi,+pi]
        int bin_id = round((yaw + CV_PI) / CV_PI * 180.0);
        bin_id = MAX(bin_id, 0);
        bin_id = MIN(bin_id, bin_size - 1);
        bins_num[bin_id]++;
    }

    // 绘制直方图.
    cv::Mat dstImage(360, 360, CV_8U, cv::Scalar(0));
    int minValue = *min_element(bins_num.begin(), bins_num.end());
    int maxValue = *max_element(bins_num.begin(), bins_num.end());
    int hpt = int(0.9 * bin_size);
    for (int i = 0; i < bin_size; i++) {
        int binValue = bins_num[i]; //   注意hist中是float类型
        // 拉伸到0-max
        int realValue = int(binValue * hpt / maxValue);
        cv::line(dstImage, cv::Point(i, bin_size - 1), cv::Point(i, bin_size - realValue), cv::Scalar(255));
    }
    cv::imshow("Hist", dstImage);
    cv::waitKey(1);

    // 从直方图中获得yaw角 [是否要考虑 +- 90deg 的情况? 暂时不考虑得了. ]

    // 计算最大 yaw 角
    auto maxIter = max_element(bins_num.begin(), bins_num.end());
    int max_id = distance(bins_num.begin(), maxIter);
    double max_yaw = max_id / 180.0 * CV_PI - CV_PI;

    return max_yaw;
}

// Func: 将物体从世界坐标系转化到一个 Normalized 系
// 这个坐标系的z轴即重力方向，yaw角度算好了，且中点在原点(可以不必保证).

// 发现了： 这里的所谓对齐，是强行将坐标系替换成沿着Z轴方向！
g2o::SE3Quat GetNormalizedTransform(const Vector4d &sup_plane, const PCAResult &data) {
    Vector3d rot_vec_z = sup_plane.head(3).normalized();
    Vector3d rot_vec_x = data.rotMat.col(0).normalized(); // or use the origin PCA result.
    Vector3d rot_vec_y = rot_vec_z.cross(rot_vec_x);

    Matrix3d rotMat_wo; // object in world
    rotMat_wo.col(0) = rot_vec_x;
    rotMat_wo.col(1) = rot_vec_y;
    rotMat_wo.col(2) = rot_vec_z;

    // transform to the normalized coordinate
    g2o::SE3Quat *pSE3Two = new g2o::SE3Quat;
    Eigen::Quaterniond quat_wo(rotMat_wo);
    pSE3Two->setRotation(quat_wo);
    pSE3Two->setTranslation(data.center); // it is the center of the old object points; it's better to use the center of the new complete points
    g2o::SE3Quat SE3Tow(pSE3Two->inverse());

    return SE3Tow;
}

MatrixXd TransformPlanes(MatrixXd &planeMat, g2o::SE3Quat &T) {
    //  存储 空间切平面
    //  注意mPlanesParam中的平面是在世界坐标系下, 此处应该转到相机的局部系下再处理.
    MatrixXd pMT;
    pMT.resize(0, 4);
    int pNum = planeMat.rows();
    for (int i = 0; i < pNum; i++) {
        g2o::plane pl(planeMat.row(i).head(4));
        pl.transform(T);
        VectorXd plvt = pl.param;
        addVecToMatirx(pMT, plvt);
    }
    return pMT;
}

// 使用 IoU 来计算
double CalculateProbability(g2o::ellipsoid &e_local, Vector4d &bbox, Matrix3d &calibMat) {
    // 开始写.
    g2o::SE3Quat Tcw;
    // 获得椭球体投影到图像上的矩形包围框：先投影获得一个椭圆，然后计算椭圆外包框
    Vector4d rect = e_local.getBoundingBoxFromProjection(Tcw, calibMat);

    // 与bbox求IoU
    cv::Rect r1_proj(cv::Point(rect[0], rect[1]), cv::Point(rect[2], rect[3]));
    cv::Rect r2_bbox(cv::Point(bbox[0], bbox[1]), cv::Point(bbox[2], bbox[3]));

    cv::Rect r_and = r1_proj | r2_bbox;
    cv::Rect r_U = r1_proj & r2_bbox;
    double iou = r_U.area() * 1.0 / r_and.area();

    // std::cout << "r1_proj : " << r1_proj.x << "," << r1_proj.y << "," << r1_proj.width << "," << r1_proj.height << std::endl;
    // std::cout << "r2_bbox : " << r2_bbox.x << "," << r2_bbox.y << "," << r2_bbox.width << "," << r2_bbox.height << std::endl;
    // std::cout << "iou : " << iou << std::endl;

    // double sigma = 0.5;
    // double value_square = iou * iou;
    // double sigma_square = sigma * sigma;
    // double prob = exp( - value_square / sigma_square);
    // std::cout << "sigma : " << sigma << std::endl;
    // std::cout << "prob : " << prob << std::endl;

    double prob = iou; // 暂时直接以 iou 作为概率
    return prob;
}

// [旧版本，弃用]
// 计算本次单帧椭球体提取的概率;
// 通过将椭球体投影到图像平面，外接矩形与bbox比较获得
double CalculateProbabilityUsingPixelDiff(g2o::ellipsoid &e_local, Vector4d &bbox, Matrix3d &calibMat) {
    // 开始写.
    g2o::SE3Quat Tcw;
    Vector4d rect = e_local.getBoundingBoxFromProjection(Tcw, calibMat);

    // 与bbox求像素差
    double pixel_diff = (rect - bbox).cwiseAbs().sum();
    std::cout << "proj : " << rect.transpose() << std::endl;
    std::cout << "bbox : " << bbox.transpose() << std::endl;
    std::cout << "pixel_diff : " << pixel_diff << std::endl;

    double sigma = 200;

    double value_square = pixel_diff * pixel_diff;
    double sigma_square = sigma * sigma;
    double prob = exp(-value_square / sigma_square);

    std::cout << "sigma : " << sigma << std::endl;
    std::cout << "prob : " << prob << std::endl;

    return prob;
}

std::vector<g2o::ConstrainPlane *> GenerateConstrainPlanesFromMatrix(MatrixXd &mPlanesParamLocal) {
    std::vector<g2o::ConstrainPlane *> vcppl;
    int num = mPlanesParamLocal.rows();
    for (int i = 0; i < num; i++) {
        VectorXd vec = mPlanesParamLocal.row(i);
        g2o::plane *ppl = new g2o::plane(vec.head(4));
        g2o::ConstrainPlane *pcpl = new g2o::ConstrainPlane(ppl);
        vcppl.push_back(pcpl);
    }
    return vcppl;
}

std::vector<bool> GetBorderFlags(Vector4d &measure, int rows, int cols) {
    // CONFIG
    double config_boarder = Config::Get<int>("Measurement.Border.Pixels"); // 可设置一个偏大的值, 保证边界观测必须是 border. 之后还会进行3d检测.
    // -----
    std::vector<bool> flags;
    flags.resize(4);
    fill(flags.begin(), flags.end(), true);
    if (measure[0] > config_boarder && measure[0] < cols - 1 - config_boarder) {
        flags[0] = false;
    }
    if (measure[2] > config_boarder && measure[2] < cols - 1 - config_boarder) {
        flags[2] = false;
    }
    if (measure[1] > config_boarder && measure[1] < rows - 1 - config_boarder) {
        flags[1] = false;
    }
    if (measure[3] > config_boarder && measure[3] < rows - 1 - config_boarder) {
        flags[3] = false;
    }

    return flags;
}

// 方法: 先初始化一个标准坐标系，再将该系Z轴 "Align" 到重力方向. 否则需要指定一个与Z正交的Y轴方向
g2o::SE3Quat EllipsoidExtractor::GenerateGravityCoordinate(const Vector3d &center, const Vector3d &gravity_normal) {
    // 此处构建旋转
    // 旧代码: 将已有的旋转矩阵“对齐”到Z轴; 实际上: 可以直接生成!
    PCAResult gravity_coor;
    gravity_coor.rotMat = Eigen::Matrix3d::Identity(); // 以世界系做初始化
    gravity_coor.center = center;
    // adjust the x,y,z order
    AlignZAxisToGravity(gravity_coor);
    // align z axis with the normal of the supporting plane
    ApplyGravityPrior(gravity_coor);

    // 构建世界系
    g2o::SE3Quat Twg;
    Twg.setTranslation(gravity_coor.center);
    Eigen::Quaterniond gravity_quat(gravity_coor.rotMat); // rotMat转四元数
    Twg.setRotation(gravity_quat);

    return Twg;
}

// x轴将指向所谓的 "前方", 即法向量投票器的峰值
g2o::SE3Quat GenerateTransformNormalToGravity(double yaw) {
    g2o::SE3Quat Tgn; // normal in gravity
    Tgn.setTranslation(Vector3d(0, 0, 0));
    Matrix3d gn_rotMat;
    gn_rotMat.col(0) = Vector3d(cos(yaw), sin(yaw), 0); // 令x轴为yaw
    gn_rotMat.col(2) = Vector3d(0, 0, 1);               // z
    gn_rotMat.col(1) = gn_rotMat.col(2).cross(gn_rotMat.col(0));
    Eigen::Quaterniond gn_quat(gn_rotMat);
    Tgn.setRotation(gn_quat);
    return Tgn;
}

std::vector<g2o::ConstrainPlane *> GenerateConstrainPlanesOfBbox(Vector4d &bbox, Matrix3d &calib, int rows, int cols) {
    g2o::SE3Quat local_wc = g2o::SE3Quat();
    MatrixXd mPlanesParamLocal_Col = GenerateBboxPlanes(local_wc, bbox, calib); // attention: store as 列
    MatrixXd mPlanesParamLocal = mPlanesParamLocal_Col.transpose();

    std::vector<g2o::ConstrainPlane *> vCPlanesWithBorderFlags = GenerateConstrainPlanesFromMatrix(mPlanesParamLocal);

    // 添加flag
    for (auto pCPlane : vCPlanesWithBorderFlags)
        pCPlane->type = 0; // 0, bbox ; 1, cuboids

    // 添加边界flag
    std::vector<bool> bboxIsBorderFlags = GetBorderFlags(bbox, rows, cols);
    for (int i = 0; i < 4; i++) {
        // bbox
        vCPlanesWithBorderFlags[i]->image_border = bboxIsBorderFlags[i];
    }

    // Update: 2020-12-27 保持切平面法向量指向约束物体
    // 方法: 直接以相机中心判断，必须方向为正
    for (auto ppl : vCPlanesWithBorderFlags) {
        // 生成 bbox 中心的向量
        Vector2d center_bbox = (bbox.head(2) + bbox.tail(2)) / 2.0;

        // 定义常量
        double mCamera_cx = calib(0, 2);
        double mCamera_cy = calib(1, 2);
        double mCamera_fx = calib(0, 0);
        double mCamera_fy = calib(1, 1);

        // 这里需要考虑fx,fy的正负性问题，当为负则添加一个负号
        Vector3d center_bbox_3d(center_bbox(0) - mCamera_cx, center_bbox(1) - mCamera_cy, std::abs(mCamera_fx));
        if (mCamera_fx < 0)
            center_bbox_3d[0] = -center_bbox_3d[0];
        if (mCamera_fy < 0)
            center_bbox_3d[1] = -center_bbox_3d[1];
        center_bbox_3d.normalize();
        Vector3d norm_camera_to_obj = center_bbox_3d; // Z 轴正方向
        double dis_sigma = 1.0;
        Vector3d vec_center(dis_sigma * norm_camera_to_obj); // 沿着相机到其中心方向平移一个距离 sigma

        if (ppl) {
            auto pl = ppl->pPlane;
            if (pl) {
                double dis = pl->distanceToPoint(vec_center, true); // 相机前方一个点的距离
                if (dis > 0) {
                    // 反转该平面法向量
                    pl->param = -pl->param;
                }
            }
        }
    }

    return vCPlanesWithBorderFlags;
}

MatrixXd VecPlanesToMat(const std::vector<g2o::plane *> &vecPlanes) {
    MatrixXd outputMat;
    outputMat.resize(0, 4);
    for (auto ppl : vecPlanes) {
        VectorXd param(4);
        param << ppl->param;
        addVecToMatirx(outputMat, param);
    }
    return outputMat;
}

std::vector<g2o::ConstrainPlane *> GenerateConstrainPlanesOfCuboids(g2o::ellipsoid &e_local_normalized, g2o::SE3Quat &campose_wc, Matrix3d &calib, int rows, int cols) {
    // 1、 从立方体中获得前后二面.
    // // 获得所有立方体平面
    // std::vector<g2o::plane*> vecPlanes = e_local_normalized.GetCubePlanes();
    // // 从立方体平面中寻找到得前后平面
    // // 判断所有平面法向量，求与Z轴最小夹角最近的.
    // g2o::plane* pfdPlane = selectForwardPlane(vecPlanes);
    // g2o::plane* pbkPlane = selectBackPlane(vecPlanes);

    // // 局部平面
    // MatrixXd mPlanesParamLocal; mPlanesParamLocal.resize(0,4);

    // VectorXd fdPlaneParam(4); fdPlaneParam << pfdPlane->param;    // 转换到了世界坐标系内.
    // VectorXd bkPlaneParam(4); bkPlaneParam << pbkPlane->param;

    // addVecToMatirx(mPlanesParamLocal, fdPlaneParam);
    // addVecToMatirx(mPlanesParamLocal, bkPlaneParam);

    // 版本2、 获得 Cuboids 的所有有效平面
    std::vector<g2o::plane *> vecPlanes = e_local_normalized.GetCubePlanesInImages(g2o::SE3Quat(), calib, rows, cols, 30);
    MatrixXd mPlanesParamLocal = VecPlanesToMat(vecPlanes);
    std::vector<g2o::ConstrainPlane *> vCPlanes = GenerateConstrainPlanesFromMatrix(mPlanesParamLocal);

    // 添加flag
    for (auto pCPlane : vCPlanes)
        pCPlane->type = 1; // 0, bbox ; 1, cuboids

    return vCPlanes;
}

// update 7-8 : ConstrainPlanes 中将只包含 bbox 的平面.
void GenerateConstrainPlanesToEllipsoid(g2o::ellipsoid &e_local_normalized, Vector4d &bbox, cv::Mat &depth, g2o::SE3Quat &campose_wc, Matrix3d &calib) {
    // [预留功能]: 在局部对椭球体做一次优化. 该部分其实可以转到推理部分. 本质为了与地面结合. 所以此处可以废弃.
    // mPlanesParam 当前6个平面. +2个前后, (如何在svd中固定已有的旋转??)
    // 或者能否用优化方式, 构建一个小优化, 固定 rot 调节scale,center;
    // 这样得到的用于两个目的: 1) 初始化 2) 数据关联.

    // 1) 获得 ellipsoid 外接矩形的 Constrainplanes
    // std::vector<g2o::ConstrainPlane*> CPlanesCuboids = GenerateConstrainPlanesOfCuboids(e_local_normalized, campose_wc, calib, depth.rows, depth.cols);

    // 2) 获得 bbox 产生的 constrainplanes
    std::vector<g2o::ConstrainPlane *> CPlanesBbox = GenerateConstrainPlanesOfBbox(bbox, calib, depth.rows, depth.cols);

    std::vector<g2o::ConstrainPlane *> CPlanesTotal;
    // CPlanesTotal.insert(CPlanesTotal.end(), CPlanesCuboids.begin(), CPlanesCuboids.end());
    CPlanesTotal.insert(CPlanesTotal.end(), CPlanesBbox.begin(), CPlanesBbox.end());

    // mPlanesParam
    MatrixXd mPlanesParam;
    mPlanesParam.resize(CPlanesTotal.size(), 4);
    for (int i = 0; i < CPlanesTotal.size(); i++) {
        // CPlanesTotal[i]->toVector() 返回的是一个九位的向量，平面参数为后四个
        mPlanesParam.row(i) = CPlanesTotal[i]->toVector().segment(5,4).transpose();
    }
    // std::cout << "mPlanesParam = " << mPlanesParam.matrix() << std::endl;
    // std::cout << "Press Enter to continue....." << std::endl;
    // g2o::ellipsoid e_global_multiplanes = OptimizeEllipsoidUsingPlanes(e_global_normalized, mPlanesParam);
    // g2o::ellipsoid e_global_multiplanes = OptimizeEllipsoidUsingPlanes(e_local_normalized, mPlanesParam);


    // 确保切平面的法向量指向椭球体的中心
    e_local_normalized.addConstrainPlanes(CPlanesTotal);
    
    return;
}

// 实际用到的是这个
// 步骤
// 1. 提取点云
// 2. 将点云转换到重力坐标系( Z轴沿重力方向, 中心为物体中心点 )
// 3. ...
g2o::ellipsoid EllipsoidExtractor::EstimateLocalEllipsoidUsingMultiPlanes(\
    cv::Mat &depth, Eigen::Vector4d &bbox, int label, double prob, Eigen::VectorXd &pose, \
    camera_intrinsic &camera, pcl::PointCloud<PointType>::Ptr& pcd_ptr, string suffix)
{
    // std::cout << "In EllipsoidExtractor::EstimateLocalEllipsoidUsingMultiPlanes" << std::endl;
    g2o::ellipsoid e;
    miSystemState = 0;                  // reset the state
    mSymmetryOutputData.result = false; // reset
    mResult = false;

    // 如果没有设置地平面，则退出
    if (!mbSetPlane) {
        std::cerr << " Please set ground plane first." << std::endl;
        return e;
    }

    clock_t time_start = clock();
    // 1. Get the object points after supporting plane filter and euclidean filter in the world coordinate
    // 注意: 该过程由于进行了与世界平面的操作, 所以位于世界坐标系下.

    // 通过降采样、统计滤波、平面筛选、中点欧几里德快速聚类等步骤，提取世界坐标系下的物体点云。
    // 同时也会把物体点云绘制出来
    pcl::PointCloud<PointType>::Ptr pCloudPCL = ExtractPointCloud(depth, bbox, pose, camera, suffix);

    if (pCloudPCL == NULL) {
        cout << "pCloudPCL == NULL" << endl;
        pcd_ptr = NULL;
        return e;
    }
    else{
        *pcd_ptr = *pCloudPCL;
    }
    
    // if (pCloudPCL==NULL) {
    //     cout << "return because pointcloud extraction failed" << endl;
    //     return e;
    // }

    clock_t time_1_ExtractPointCloud = clock();
    if (miSystemState > 0) {
        cout << "return because miSystemState = " << miSystemState << std::endl;
        return e;
    }

    // 搭建世界系描述下的物体重力坐标系
    // gravity 系: 位于物体中心, Z轴与重力方向对齐.
    // 转化之后，点云的正方向即z轴, 即世界系重力方向.
    // get supporting plane

    // 计算点云包围框，计算重力坐标系齐次变换，及该坐标系下的点云
    VectorXd sup_plane = mpPlane->param;
    Eigen::Vector4d centroid;
    pcl::compute3DCentroid(*pCloudPCL, centroid);
    g2o::SE3Quat Twg = GenerateGravityCoordinate(centroid.head(3), sup_plane.head(3));

    // 获得该系下的点云.
    g2o::SE3Quat SE3Tgw = Twg.inverse();
    Eigen::Matrix4d transform_gw = SE3Tgw.to_homogeneous_matrix();
    pcl::PointCloud<PointType>::Ptr pCloudPCLGravity(new pcl::PointCloud<PointType>);
    pcl::transformPointCloud(*pCloudPCL, *pCloudPCLGravity, transform_gw);

    // 可视化: 重力系下的物体
    // ORB_SLAM2::PointCloud *pObjectCloudGravity = pclXYZToQuadricPointCloudPtr(pCloudPCLGravity); // normalized coordinate
    // mpMap->AddPointCloudList("cloud_gravity", pObjectCloudGravity, 0);

    // std::cout << "*****************************" << std::endl;
    // std::cout << "Showing pObjectCloudGravity, press [ENTER] to continue ... " << std::endl;
    // std::cout << "*****************************" << std::endl;
    // getchar();

    // // todo: 测试，有待解除注释
    // delete pObjectCloudGravity;
    // pObjectCloudGravity = NULL;

    // 对点云使用法向量投票器，计算偏航角度
    // std::cout << "Computing cloud_normalized" << std::endl;
    double yaw = NormalVoter(pCloudPCLGravity); // 该函数获得一个位于 XY 平面内的, 三维法向量. 可与 Z轴组完整旋转矩阵.

    // 通过yaw角度将 Gravity - > normalized
    // 进一步计算规范化变换
    g2o::SE3Quat Tgn = GenerateTransformNormalToGravity(yaw);


    Eigen::Matrix4d transform_ng = Tgn.inverse().to_homogeneous_matrix();
    pcl::PointCloud<PointType>::Ptr pCloudPCLNormalized(new pcl::PointCloud<PointType>);
    pcl::transformPointCloud(*pCloudPCLGravity, *pCloudPCLNormalized, transform_ng);
    ORB_SLAM2::PointCloud *pObjectCloudNormalized = pclXYZToQuadricPointCloudPtr(pCloudPCLNormalized); // normalized coordinate

    // // 可视化: 物体重力坐标系下，转角对齐后的点云
    // mpMap->AddPointCloudList("cloud_normalized", pObjectCloudNormalized, 0);

    // std::cout << "*****************************" << std::endl;
    // std::cout << "Showing pObjectCloudNormalized, press [ENTER] to continue ... " << std::endl;
    // std::cout << "*****************************" << std::endl;
    // getchar();

    // 基于PCA结果生成最小包围盒顶点. 位于相机坐标系内.
    // 从规范化的点云中获取椭球体
    g2o::ellipsoid e_zero_normalized = GetEllipsoidFromNomalizedPointCloud(pObjectCloudNormalized);
    // todo: 测试，有待解除注释
    delete pObjectCloudNormalized;
    pObjectCloudNormalized = NULL;

    // 变换回局部坐标系
    g2o::SE3Quat campose_wc;
    campose_wc.fromVector(pose);
    g2o::SE3Quat Twn = Twg * Tgn;
    g2o::SE3Quat Tcn = campose_wc.inverse() * Twn;
    g2o::ellipsoid e_local_normalized = e_zero_normalized.transform_from(Tcn);

    // 可视化: 物体重力坐标系下，转角对齐后的点云
    mpMap->addEllipsoid(&e_local_normalized);

    // std::cout << "*****************************" << std::endl;
    // std::cout << "Showing ellipsoid e_local_normalized, press [ENTER] to continue ... " << std::endl;
    // std::cout << "*****************************" << std::endl;
    // getchar();

    // -------------- 到此已获得相机坐标系下的椭球体!

    // 接下来添加 ConstrainPlanes.
    Matrix3d calib = CameraToCalibMatrix(camera);
    // 生成约束平面(ellipsold, bbox, depth...)，可以在这一过程中增加基于约束平面的优化
    // 存储在 e_local_normalized 中
    GenerateConstrainPlanesToEllipsoid(e_local_normalized, bbox, depth, campose_wc, calib);
    VisualizeConstrainPlanes(e_local_normalized, campose_wc, mpMap); // 中点定在全局坐标系
    // std::cout << "*****************************" << std::endl;
    // std::cout << "Showing Constrain Planes, press [ENTER] to continue ... " << std::endl;
    // std::cout << "*****************************" << std::endl;
    // getchar();


    // 评估本次提取的概率 : 投影回来的矩形与 bbox 的 IoU 作为规律.
    double prob_3d = CalculateProbability(e_local_normalized, bbox, calib);
    std::cout << "prob_3d = " << prob_3d << std::endl;

    // calculate the probability of the single-frame ellipsoid estimation
    e_local_normalized.prob_3d = prob_3d;
    e_local_normalized.prob = prob * prob_3d; // measurement_prob * symmetry_prob
    e_local_normalized.miLabel = label;
    e_local_normalized.bbox = bbox;
    e_local_normalized.scale = e_zero_normalized.scale;
    e_local_normalized.bPointModel = false;
    mResult = true;
    clock_t time_2_fullProcess = clock();


    // output the main running time
    cout << " -- System Time [EllipsoidExtractor.cpp] :" << endl;
    cout << " ---- time_ExtractPointCloud: " << (double)(time_1_ExtractPointCloud - time_start) / CLOCKS_PER_SEC << "s" << endl;
    cout << " ---- total_ellipsoidExtraction: " << (double)(time_2_fullProcess - time_start) / CLOCKS_PER_SEC << "s" << endl;
    cout << endl;

    // // 此处添加一个判断, 若 prob_3d < 0.5 则舍弃
    // if(prob_3d < 0.5)
    // {
    //     mResult = false;
    // }
    // else
    //     mResult = true;

    return e_local_normalized;
}

// // 使用单目相机版本提取椭球体
// g2o::ellipsoid EllipsoidExtractor::EstimateLocalEllipsoidMonocular(Eigen::Vector3d& prior, Eigen::Vector4d& bbox, int label, double prob, Eigen::VectorXd &pose, camera_intrinsic& camera)
// {
//     g2o::ellipsoid e;
//     miSystemState = 0;  // reset the state
//     mSymmetryOutputData.result = false; // reset
//     mResult = false;

//     if(!mbSetPlane)
//     {
//         std::cerr << " Please set ground plane first." << std::endl;
//         return e;
//     }

//     clock_t time_start = clock();

//     VectorXd sup_plane = mpPlane->param;

//     // 以单目形式获得一个初始化椭球体，在后期以semantic prior做精细推理

//     // bbox 生成 切平面

//     // 开始优化得到椭球体

//     // -------------- 到此已获得相机坐标系下的椭球体!

//     // 接下来添加 ConstrainPlanes.
//     Matrix3d calib = CameraToCalibMatrix(camera);
//     GenerateConstrainPlanesToEllipsoid(e_local_normalized, bbox, depth, campose_wc, calib);
//     VisualizeConstrainPlanes(e_local_normalized, campose_wc, mpMap); // 中点定在全局坐标系

//     // 评估本次提取的概率 : 投影回来的矩形与 bbox 的 IoU 作为规律.
//     double prob_3d = CalculateProbability(e_local_normalized, bbox, calib);

//     // calculate the probability of the single-frame ellipsoid estimation
//     e_local_normalized.prob_3d = prob_3d;
//     e_local_normalized.prob = prob * prob_3d;    // measurement_prob * symmetry_prob
//     e_local_normalized.miLabel = label;
//     e_local_normalized.bbox = bbox;
//     e_local_normalized.bPointModel = false;
//     mResult = true;
//     clock_t time_2_fullProcess = clock();

//     // output the main running time
//     cout << " -- System Time [EllipsoidExtractor.cpp] :" << endl ;
//     cout << " ---- time_ExtractPointCloud: " <<(double)(time_1_ExtractPointCloud - time_start) / CLOCKS_PER_SEC << "s" << endl;
//     cout << " ---- total_ellipsoidExtraction: " <<(double)(time_2_fullProcess - time_start) / CLOCKS_PER_SEC << "s" << endl;
//     cout << endl;

//     // // 此处添加一个判断, 若 prob_3d < 0.5 则舍弃
//     // if(prob_3d < 0.5)
//     // {
//     //     mResult = false;
//     // }
//     // else
//     //     mResult = true;

//     return e_local_normalized;
// }

Vector3d Get3DPointFromDepth(int x, int y, const cv::Mat &depth_, const camera_intrinsic &camera) {
    cv::Mat depth = depth_;
    ushort *ptd = depth.ptr<ushort>(y);
    ushort d = ptd[x];
    ORB_SLAM2::PointXYZRGB p;
    p.z = d / camera.scale;

    bool bCenterValid = true;
    if (p.z <= 0.1 || p.z > 10.0) // if the depth is valid
        bCenterValid = false;

    p.x = (x - camera.cx) * p.z / camera.fx;
    p.y = (y - camera.cy) * p.z / camera.fy;

    Vector3d center_3d;
    center_3d << p.x, p.y, p.z;
    return center_3d;
}

ConstrainPlane *GenerateCenterConstrainPlane(const Vector4d &bbox, const cv::Mat &depth, const camera_intrinsic &camera) {
    // 获得中心点
    Vector2d center = (bbox.head(2) + bbox.tail(2)) / 2.0;
    int x = round(center[0]);
    int y = round(center[1]);

    // 恢复出 3d point
    Vector3d center_3d = Get3DPointFromDepth(x, y, depth, camera);

    // 以Radius取3个点，求其法向量.
    int radius = 10;
    int x_delta = radius * 1.73 / 2.0; // sqrt(3)
    int y_delta = radius * 0.5;
    Vector3d center_neighbor1 = Get3DPointFromDepth(x, y + radius, depth, camera);
    Vector3d center_neighbor2 = Get3DPointFromDepth(x - x_delta, y - y_delta, depth, camera);
    Vector3d center_neighbor3 = Get3DPointFromDepth(x + x_delta, y - y_delta, depth, camera);

    // 求附近 normal
    Vector3d normal = (center_neighbor1 - center_neighbor2).cross(center_neighbor3 - center_neighbor2);
    // 取附近点做叉乘恢复出平面

    g2o::plane *ppl = new g2o::plane();
    ppl->fromPointAndNormal(center_3d, normal);

    // 生成 Constrainplane返回.
    ConstrainPlane *pCPlane = new ConstrainPlane(ppl);
    pCPlane->type = 0;
    pCPlane->image_border = false;

    return pCPlane;
}

// *********************
// 来自 Baseline: TrackingNP.h
// *********************
bool EllipsoidExtractor::EstimateLocalEllipsoidUsingPointModel(cv::Mat &depth, Eigen::Vector4d &bbox, int label, double prob, Eigen::VectorXd &pose, camera_intrinsic &camera, g2o::ellipsoid &e_extracted) {
    PointCloud cloud = getPointCloudInRect(depth, bbox, camera);
    if (cloud.size() < 10)
        return false; // 最低要求.

    // 获得中点.
    PointCloudPCL::Ptr pCloud = QuadricPointCloudToPcl(cloud);

    Eigen::Vector4d centroid;
    pcl::compute3DCentroid(*pCloud, centroid);

    // 构造椭球体.
    Vector9d e_vec;
    e_vec << centroid.head(3), 0, 0, 0, 0.05, 0.05, 0.05;
    g2o::ellipsoid e;
    e.fromMinimalVector(e_vec);

    // 设置 label
    e.miLabel = label;
    e.prob = 1;
    e.bbox = bbox;
    e.bPointModel = true;
    e.prob_3d = 0;

    // 添加ConstrainPlanes
    g2o::SE3Quat campose_wc;
    campose_wc.fromVector(pose.tail(7));
    Matrix3d calib = CameraToCalibMatrix(camera);
    auto cplanes = GenerateConstrainPlanesOfBbox(bbox, calib, depth.rows, depth.cols);

    // 生成中心切平面
    // ConstrainPlane* pcenter_cplane = GenerateCenterConstrainPlane(bbox, depth, camera);
    // cplanes.push_back(pcenter_cplane);
    e.addConstrainPlanes(cplanes);

    VisualizeConstrainPlanes(e, campose_wc, mpMap); // 中点定在全局坐标系

    e_extracted = e;
    return true;
}

// 设置曼哈顿平面
void EllipsoidExtractor::SetManhattanPlanes(const std::vector<g2o::plane *> vpPlanes) {
    mvpMHPlanes = vpPlanes;
    if (mvpMHPlanes.size() > 0)
        mbOpenMHPlanesFilter = true;

    std::cout << "MHPlanes set!!! size: " << mvpMHPlanes.size() << std::endl;
}

ORB_SLAM2::PointCloud *EllipsoidExtractor::ApplyMHPlanesFilter(ORB_SLAM2::PointCloud *pCloud, std::vector<g2o::plane *> &vpPlanes) {
    ORB_SLAM2::PointCloud *pCloudFiltered = new ORB_SLAM2::PointCloud;
    int num = pCloud->size();

    int i = 0;
    for (auto p : (*pCloud)) {
        bool ok = true;
        for (auto &pPlane : vpPlanes) {
            double dis = pPlane->distanceToPoint(Vector3d(p.x, p.y, p.z), true); // ture means keeping the flag. The PlaneExtractor has made sure the positive value means the point is above the plane.
            if (dis < 0.03) {
                ok = false;
                break;
            }
        }

        if (ok)
            pCloudFiltered->push_back(p);
    }

    return pCloudFiltered;
}

// 套一个新参数过来生效
// 传入平面: 局部坐标系下的支撑平面
g2o::ellipsoid EllipsoidExtractor::EstimateLocalEllipsoidWithSupportingPlane(cv::Mat &depth, Eigen::Vector4d &bbox, int label, double prob, Eigen::VectorXd &pose, camera_intrinsic &camera, g2o::plane *pSupPlane) {
    // 设置地平面
    g2o::plane *originGroundPlane = mpPlane;

    g2o::SE3Quat Twc;
    Twc.fromVector(pose.tail(7));
    g2o::plane *pSupPlaneWorld = new g2o::plane(*pSupPlane);
    pSupPlaneWorld->transform(Twc);
    // mpPlane = pSupPlaneWorld;

    SetSupportingPlane(pSupPlaneWorld, true);

    pcl::PointCloud<PointType>::Ptr pcd_ptr;

    // g2o::ellipsoid e_extractByFitting_newSym = \
    //     mpEllipsoidExtractor->EstimateLocalEllipsoidUsingMultiPlanes(\
    //         pFrame->frame_img, measurement, label, measurement_prob, pose, mCamera, pcd_ptr, pcd_suffix);

    // det.setPcdPtr(pcd_ptr);

    g2o::ellipsoid e = EstimateLocalEllipsoidUsingMultiPlanes(depth, bbox, label, prob, pose, camera, pcd_ptr);

    // pcd_ptr.delete();

    // 取消地平面
    // mpPlane = originGroundPlane;
    SetSupportingPlane(originGroundPlane, false);

    delete pSupPlaneWorld;
    pSupPlaneWorld = NULL;

    // 返回。
    return e;
}

// // 使用统计方法，更鲁棒地估计其外围立方体
// PCAResult EllipsoidExtractor::ProcessPCANormalizedWithStatics(ORB_SLAM2::PointCloud* pObject)
// {
//     // -------- 参数设置 ---------
//     int config_resolution = 400;   // 格子数量;  一般家具物体 3m长, 则可到 0.0075 cm.

//     // 即考虑噪声的影响，直接截断 0.95 or 0.99%

//     // --------------------------
//     // 对三个方向做统计直方图.
//     PCAResult data;
//     double x,y,z;
//     x=0;y=0;z=0;
//     int num = pObject->size();

//     double max_x = 0, min_x=0;
//     double max_y = 0, min_y=0;
//     double max_z = 0, min_z=0;
//     for( int i=0;i<num;i++ )
//     {
//         double px = (*pObject)[i].x;
//         double py = (*pObject)[i].y;
//         double pz = (*pObject)[i].z;

//         // x += px * px;
//         // y += py * py;
//         // z += pz * pz;

//         if( px > max_x ) max_x = px;
//         if( py > max_y ) max_y = py;
//         if( pz > max_z ) max_z = pz;

//         if( px < min_x ) min_x = px;
//         if( py < min_y ) min_y = py;
//         if( pz < min_z ) min_z = pz;
//     }

//     // 若能基于分布产生一个概率则更好了!!!!
//     // 某种与视角有关的连续假设
// }

} // namespace ORB_SLAM2