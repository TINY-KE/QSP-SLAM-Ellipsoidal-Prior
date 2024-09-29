#include "EllipsoidExtractor.h"

#include <include/utils/matrix_utils.h>

// For Euclidean Cluster Extraction
#include <pcl/ModelCoefficients.h>
#include <pcl/features/normal_3d.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>

// g2o
#include "Thirdparty/g2o/g2o/core/block_solver.h"
#include "Thirdparty/g2o/g2o/core/optimization_algorithm_levenberg.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_dense.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_eigen.h"
#include "Thirdparty/g2o/g2o/types/types_six_dof_expmap.h"

#include "Thirdparty/g2o/g2o/core/robust_kernel.h"
#include "Thirdparty/g2o/g2o/core/robust_kernel_impl.h"

#include <core/Geometry.h>
#include <src/config/Config.h>

namespace ORB_SLAM2 {

bool compare_cloud_with_z(ORB_SLAM2::PointXYZRGB &p1, ORB_SLAM2::PointXYZRGB &p2) {
    return p1.z < p2.z;
}

EllipsoidExtractor::EllipsoidExtractor() {
    mResult = false;
    mbSetPlane = false;

    mDebugCenterCloud = NULL;

    mbOpenVisualization = false;
    miExtractCount = 0;

    mbOpenSymmetry = false;

    SetExtractionMethod(ATLAS_EXTRACTION);

    mbOpenMHPlanesFilter = false;
}

void EllipsoidExtractor::LoadSymmetryPrior() {
    // the semantic label comes from coco names, which is suitable for YOLO.
    // *************************************
    // Object-type LabelID SymmetryType
    // potted plant |   58   |   None
    // bed  |   59   |   Reflection
    // tvmonitor   |   62   |   Reflection
    // sofa |   57   |   Reflection
    // keyboard |   66  |   Dual Reflection
    // laptop   |   63  |   Reflection
    // mouse    |   64  |   Reflection
    //  cup     |   41  |   Reflection
    //  suitcase|   28  |   Dual Reflection

    std::map<int, int> labelSymmetry;
    labelSymmetry.insert(make_pair(58, 0));
    labelSymmetry.insert(make_pair(59, 1));
    labelSymmetry.insert(make_pair(62, 1));
    labelSymmetry.insert(make_pair(57, 1));
    labelSymmetry.insert(make_pair(66, 1));
    labelSymmetry.insert(make_pair(63, 1));
    labelSymmetry.insert(make_pair(64, 1));
    labelSymmetry.insert(make_pair(41, 1));
    labelSymmetry.insert(make_pair(28, 2));

    mmLabelSymmetry = labelSymmetry; // load to the config
}

bool EllipsoidExtractor::GetResult() {
    return mResult;
}

/**
 * 获取bbox内的点云，使用统计滤波滤除率群点
 * 转换到世界坐标系，使用支撑平面/曼哈顿平面进行滤波
 * 计算点云中点，并基于中点使用欧几里德快速聚类筛选物体点
 * 返回的点云为欧几里德快速聚类后的点云
*/
pcl::PointCloud<PointType>::Ptr EllipsoidExtractor::ExtractPointCloud(cv::Mat &depth, Eigen::Vector4d &bbox, Eigen::VectorXd &pose, camera_intrinsic &camera, string suffix) {
    clock_t time_1_start = clock();

    assert(mbSetPlane && "Please set the supporting plane first.");


    double depth_range = Config::ReadValue<double>("EllipsoidExtractor_DEPTH_RANGE", 6);
    g2o::SE3Quat campose_wc;
    campose_wc.fromVector(pose.head(7));

    PointCloud *pPoints_local = new PointCloud(getPointCloudInRect(depth, bbox, camera, depth_range));

    // downsample points with small grid
    // PointCloud* pPoints_local_downsample = new PointCloud;
    // DownSamplePointCloudOnly(*pPoints_local, *pPoints_local_downsample, 0.02);

    // VisualizePointCloud("Points_local", pPoints_local, Vector3d(0, 0.5, 0), 2);
    // std::cout << "*****************************" << std::endl;
    // std::cout << "Showing Points_local, press [ENTER] to continue ... " << std::endl;
    // std::cout << "*****************************" << std::endl;
    // getchar();

    // std::cout << "campose_wc = " << campose_wc.to_homogeneous_matrix().matrix() << std::endl;

    // 产生两组可视化点云 world, world_downsample
    // PointCloud *pPoints_world = transformPointCloud(pPoints_local, &campose_wc);
    // PointCloud *pPoints_world_downsample = transformPointCloud(pPoints_local_downsample, &campose_wc);
    // VisualizePointCloud("Points_world", pPoints_world, Vector3d(0, 0.5, 0), 2);
    // VisualizePointCloud("Points_world_downsample", pPoints_world_downsample, Vector3d(0,0.8,0), 2);

    // std::cout << "*****************************" << std::endl;
    // std::cout << "Showing Points_world, press [ENTER] to continue ... " << std::endl;
    // std::cout << "*****************************" << std::endl;
    // getchar();

    // 在此滤除离群点
    clock_t time_1_1_outliers_filter_start = clock();
    auto inputPclPtr = QuadricPointCloudToPclXYZ(*pPoints_local);

    // /*半径滤波器*/
    // int Config_MinNeighborsInRadius = Config::Get<int>("EllipsoidExtractor.MinNeighborsInRadius");
    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_after_filter(new pcl::PointCloud<pcl::PointXYZ>);
    // pcl::RadiusOutlierRemoval<pcl::PointXYZ> ror;  //创建滤波器
    // ror.setInputCloud(inputPclPtr);    //设置输入点云
    // ror.setRadiusSearch(0.1);     //设置半径为100的范围内找临近点
    // ror.setMinNeighborsInRadius(Config_MinNeighborsInRadius); //设置查询点的邻域点集数小于2的删除
    // ror.filter(*cloud_after_filter);
    // std::cout << "debug: Config_MinNeighborsInRadius: " << Config_MinNeighborsInRadius << std::endl;

    /* 统计滤波器 (慢?) */
    // Create the filtering object
    int Config_MeanK = Config::Get<int>("EllipsoidExtractor.StatisticalOutlierRemoval.MeanK");
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_after_filter(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud(inputPclPtr);
    sor.setMeanK(Config_MeanK);
    sor.setStddevMulThresh(1.0);
    sor.filter(*cloud_after_filter);

    // std::cout << "filter before/after points: " << inputPclPtr->points.size() << " / " << cloud_after_filter->points.size() << std::endl;
    ORB_SLAM2::PointCloud *pCloudFiltered = pclXYZToQuadricPointCloudPtr(cloud_after_filter);
    clock_t time_1_1_outliers_filter_end = clock();

    // transform to the world coordinate.
    // 经过滤波的世界坐标系点云
    PointCloud *pPoints_global = transformPointCloud(pCloudFiltered, &campose_wc);
    delete pPoints_local;
    pPoints_local = NULL;
    // delete pPoints_local_downsample; pPoints_local_downsample = NULL;

    // 新添加的可视化: 滤波后
    PointCloud *pCloudFilteredWorld = transformPointCloud(pCloudFiltered, &campose_wc);
    // VisualizePointCloud("CloudFiltered", pCloudFilteredWorld, Vector3d(0, 1.0, 0), 2);

    // std::cout << "*****************************" << std::endl;
    // std::cout << "Showing CloudFiltered, press [ENTER] to continue ... " << std::endl;
    // std::cout << "*****************************" << std::endl;
    // getchar();

    delete pCloudFiltered;
    pCloudFiltered = NULL;

    mpPointsDebug = pPoints_global;
    clock_t time_2_getPointsDownsampleTransToWorld = clock();

    PointCloud *pPoints_planeFiltered;
    if (!mbOpenMHPlanesFilter){    // 使用支撑平面进行滤波
        // std::cout << "ApplySupportingPlaneFilter" << std::endl;
        pPoints_planeFiltered = ApplySupportingPlaneFilter(pPoints_global);
    }
    else {
        // std::cout << "ApplyMHPlanesFilter" << std::endl;    // 使用曼哈顿平面进行滤波
        std::vector<g2o::plane *> vMHPlanes = mvpMHPlanes;
        vMHPlanes.push_back(mpPlane);
        pPoints_planeFiltered = ApplyMHPlanesFilter(pPoints_global, vMHPlanes);
    }
    clock_t time_3_SupportingPlaneFilter = clock();

    // VisualizePointCloud("planeFiltered", pPoints_planeFiltered, Vector3d(1.0, 0, 0), 2);
    // std::cout << "*****************************" << std::endl;
    // std::cout << "Showing CloudCloud planeFiltered, press [ENTER] to continue ... " << std::endl;
    // std::cout << "*****************************" << std::endl;
    // getchar();

    clock_t time_4_VisualizePointCloud = clock();

    if (pPoints_planeFiltered->size() < 1) {
        std::cout << "No point left." << std::endl;
        std::cout << "pPoints_global points: " << pPoints_global->size() << std::endl;
        std::cout << "No enough point cloud after Filtering. Num:  " << pPoints_planeFiltered->size() << std::endl;
        miSystemState = 4;
        return NULL;
    }

    // // 防止内部都是 NaN
    // std::vector<int> indices; //保存去除的点的索引
    // pcl::PointCloud<PointType>::Ptr pPoints_planeFiltered_pcl = QuadricPointCloudToPclXYZ(*pPoints_planeFiltered);
    // pcl::removeNaNFromPointCloud(*pPoints_planeFiltered_pcl,*pPoints_planeFiltered_pcl, indices); //去除点云中的NaN点，（m是个结构体对象，参数１是输入，参数二是输出。indices一般不用）
    // if( pPoints_planeFiltered_pcl->size() < 1 )
    // {
    //     std::cout << "No point left AFTER REMOVING NAN." << std::endl;
    //     miSystemState = 4;
    //     return NULL;
    // }

    PointCloud *pPoints_sampled = pPoints_planeFiltered;

    if (pPoints_sampled->size() < 1) {
        std::cout << "No enough point cloud after sampling. Num:  " << pPoints_sampled->size() << std::endl;
        miSystemState = 3;
        return NULL;
    }

    pcl::PointCloud<PointType>::Ptr clear_cloud_ptr(new pcl::PointCloud<PointType>);;
    // 计算中点
    Vector3d center;
    bool bCenter = GetCenter(depth, bbox, pose, camera, center);
    if (!bCenter) {
        miSystemState = 1;
        std::cout << "[Pointcloud Extraction] Can't Find Center. Bbox: " << bbox.transpose() << std::endl;
        cout << "Press Enter to continue" << endl;
        // getchar();
        return clear_cloud_ptr;
    }
    clock_t time_5_GetCenter = clock();
    // std::cout << "center = " << center.transpose().matrix() << std::endl;

    // 使用快速欧几里德聚类进行滤波
    mDebugCenter = center;
    PointCloud *pPointsEuFiltered = ApplyEuclideanFilter(pPoints_sampled, center);
    // delete pPoints_sampled; pPoints_sampled = NULL;

    if (miEuclideanFilterState > 0) {
        std::cout << "Fail to filter by EuclideanFilter." << std::endl;
        miSystemState = 2; // fail to filter
        return NULL;
    }
    clock_t time_6_ApplyEuclideanFilter = clock();

    std::cout << "pPointsEuFiltered.size() = " << pPointsEuFiltered->size() << std::endl;

    // we have gotten the object points in the world coordinate
    // pcl::PointCloud<PointType>::Ptr clear_cloud_ptr = QuadricPointCloudToPclXYZ(*pPointsEuFiltered);
    clear_cloud_ptr = QuadricPointCloudToPclXYZ(*pPointsEuFiltered);

    mpPoints = pPointsEuFiltered;


    VisualizePointCloud("EuclideanFiltered" + suffix, mpPoints, Vector3d(0.4, 0, 1.0), 2);

    clock_t time_7_VisualizePointCloud = clock();

    // output: time efficiency
    // cout << "****** System Time [ExtractPoints.cpp] ******" << endl ;
    // cout << "time_2_getPointsDownsampleTransToWorld: " <<(double)(time_2_getPointsDownsampleTransToWorld - time_1_start) / CLOCKS_PER_SEC << "s" << endl;
    // cout << "time_3_SupportingPlaneFilter: " <<(double)(time_3_SupportingPlaneFilter - time_2_getPointsDownsampleTransToWorld) / CLOCKS_PER_SEC << "s" << endl;
    // cout << "time_4_VisualizePointCloud: " <<(double)(time_4_VisualizePointCloud - time_3_SupportingPlaneFilter) / CLOCKS_PER_SEC << "s" << endl;
    // cout << "time_5_GetCenter: " <<(double)(time_5_GetCenter - time_4_VisualizePointCloud) / CLOCKS_PER_SEC << "s" << endl;
    // cout << "time_6_ApplyEuclideanFilter: " <<(double)(time_6_ApplyEuclideanFilter - time_5_GetCenter) / CLOCKS_PER_SEC << "s" << endl;
    // cout << "time_7_VisualizePointCloud: " <<(double)(time_7_VisualizePointCloud - time_6_ApplyEuclideanFilter) / CLOCKS_PER_SEC << "s" << endl;
    cout << "time_1_1_outliers_filter: " << (double)(time_1_1_outliers_filter_end - time_1_1_outliers_filter_start) / CLOCKS_PER_SEC << "s" << endl;
    return clear_cloud_ptr;
}

PCAResult EllipsoidExtractor::ProcessPCA(pcl::PointCloud<PointType>::Ptr &pCloudPCL) {
    pcl::PointCloud<PointType>::Ptr cloud = pCloudPCL;
    pcl::PointCloud<NormalType>::Ptr cloud_normal(new pcl::PointCloud<NormalType>());

    Eigen::Vector4d pcaCentroid;
    pcl::compute3DCentroid(*cloud, pcaCentroid);
    Eigen::Matrix3d covariance;
    pcl::computeCovarianceMatrixNormalized(*cloud, pcaCentroid, covariance);
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> eigen_solver(covariance, Eigen::ComputeEigenvectors);
    Eigen::Matrix3d eigenVectorsPCA = eigen_solver.eigenvectors();
    Eigen::Vector3d eigenValuesPCA = eigen_solver.eigenvalues();

    // center point
    PointType c;
    c.x = pcaCentroid(0);
    c.y = pcaCentroid(1);
    c.z = pcaCentroid(2);

    PCAResult output;
    output.result = true;
    output.center = Eigen::Vector3d(c.x, c.y, c.z);
    output.rotMat = eigenVectorsPCA;                                                              // Rotation matrix(in world coordinate)
    output.covariance = Eigen::Vector3d(eigenValuesPCA(0), eigenValuesPCA(1), eigenValuesPCA(2)); // covariance used for scale estimation of ellipsoid

    return output;
}

void EllipsoidExtractor::ApplyGravityPrior(PCAResult &data) {
    assert(mbSetPlane && "Please set the supporting plane first.");

    Eigen::Matrix3d matRot = calibRotMatAccordingToGroundPlane(data.rotMat, mpPlane->param.head(3));
    data.rotMat = matRot;
    return;
}

void EllipsoidExtractor::AlignZAxisToGravity(PCAResult &data) {
    // the goal for aligning Z axis :
    // make sure the z axis of the ellipsoid is along the direction of the groundplane normal,
    // so that when calling computeError() in the edge, we could rotate 90 deg along Z axis three times and use the minimum angle difference as the error

    // First, get the z axis
    double max_cos_theta_abs = 0;
    bool max_flag_pos;
    int max_id = -1;

    Vector3d z_axis;
    if (mbSetPlane)
        z_axis = mpPlane->param.head(3).normalized();
    else
        z_axis = Vector3d(0, 0, 1);

    // find which axis in rotMat has minimum angle difference with z_axis
    for (int i = 0; i < 3; i++) {
        Vector3d axis = data.rotMat.col(i);
        double cos_theta = axis.dot(z_axis); // a*b = |a||b|cos(theta)

        bool flag_pos = cos_theta > 0;
        double cos_theta_abs = std::abs(cos_theta);

        if (cos_theta_abs > max_cos_theta_abs) {
            max_cos_theta_abs = cos_theta_abs;
            max_flag_pos = flag_pos;
            max_id = i;
        }
    }

    assert(max_id >= 0 && "Must find a biggest one.");

    // swap the rotMat to get the correct z axis
    Matrix3d rotMatSwap;
    Vector3d covarianceSwap;
    Vector3d z_axis_vec;

    // invert the direction
    if (max_flag_pos)
        z_axis_vec = data.rotMat.col(max_id);
    else
        z_axis_vec = -data.rotMat.col(max_id);
    rotMatSwap.col(2) = z_axis_vec;
    covarianceSwap(2) = data.covariance[max_id];

    // get other two axes.
    int x_axis_id = (max_id + 1) % 3; // next axis.
    rotMatSwap.col(0) = data.rotMat.col(x_axis_id);
    covarianceSwap(0) = data.covariance[x_axis_id];

    int y_axis_id = (max_id + 2) % 3;
    rotMatSwap.col(1) = rotMatSwap.col(2).cross(rotMatSwap.col(0));
    covarianceSwap(1) = data.covariance[y_axis_id];

    data.rotMat = rotMatSwap;
    data.covariance = covarianceSwap;

    return;
}

// This function is a simple implementation for constructing an ellipsoid from rgb-d data, which differs from the paper.
g2o::ellipsoid EllipsoidExtractor::ConstructEllipsoid(PCAResult &data) {
    g2o::ellipsoid e;

    // 1) use the calibrated rotation matrix from PCA as the orientation of the ellipsoid.
    //  the z axis has been adjusted to be along the normal of the groundplane.
    Eigen::Quaterniond quat(data.rotMat);

    // 2) the scale will be decided using an approximation: the max x,y,z value of the object points.
    Eigen::Vector3d scale = data.scale;

    // x y z qx qy qz qw a b c
    Vector10d ellip_vec;
    ellip_vec << data.center[0], data.center[1], data.center[2],
        (quat.x()), (quat.y()), (quat.z()), (quat.w()),
        scale[0], scale[1], scale[2];
    e.fromVector(ellip_vec);

    return e;
}

g2o::ellipsoid EllipsoidExtractor::EstimateLocalEllipsoid(cv::Mat &depth, Eigen::Vector4d &bbox, int label, double prob, Eigen::VectorXd &pose, camera_intrinsic &camera) {
    miExtractCount++; // the total extraction times for naming point clouds.

    g2o::ellipsoid e;
    miSystemState = 0;                  // reset the state
    mSymmetryOutputData.result = false; // reset
    mResult = false;

    clock_t time_start = clock();
    // 1. Get the object points after supporting plane filter 
    //      and euclidean filter in the world coordinate
    //  获得支撑平面过滤和快速欧几里德聚类后的（世界坐标系）物体点
    pcl::PointCloud<PointType>::Ptr pCloudPCL = ExtractPointCloud(depth, bbox, pose, camera);
    if (miSystemState > 0)
        return e;
    clock_t time_1_ExtractPointCloud = clock();

    // process the principle components analysis to get the rotation matrix, 
    // scale, and center point of the point cloud
    // 使用主成分分析获得点云的旋转矩阵、尺度和中心点
    PCAResult data = ProcessPCA(pCloudPCL);

    // adjust the rotation matrix to be right-handed
    // 将旋转矩阵调整到右手系
    AdjustChirality(data);
    // adjust the x,y,z order
    AlignZAxisToGravity(data);
    // align z axis with the normal of the supporting plane
    ApplyGravityPrior(data);

    Vector3d center = data.center; // center point of the object points from PCA.
    ORB_SLAM2::PointCloud *pObjectClearCloud = pclXYZToQuadricPointCloudPtr(pCloudPCL);
    // ORB_SLAM2::PointCloud* pObjectCloud = pObjectClearCloud;    // world coordinate

    // downsample to estimate symmetry.
    // 降采样以估计对称性
    double grid_size_for_symmetry = Config::ReadValue<double>("EllipsoidExtraction.Symmetry.GridSize");
    ORB_SLAM2::PointCloud *pObjectCloud = new ORB_SLAM2::PointCloud;
    DownSamplePointCloudOnly(*pObjectClearCloud, *pObjectCloud, grid_size_for_symmetry);
    VisualizePointCloud("Points For Sym", pObjectCloud, Vector3d(0.5, 0.5, 0.0), 6);

    // construct a normalized rotation matrix using the normal 
    // of the supporting plane and the normal of the symmetry plane.
    // 使用支撑平面的法向量和对称平面的法向量构建一个归一化的旋转矩阵
    Vector3d rot_vec_z;
    if (mbSetPlane)
        rot_vec_z = mpPlane->param.head(3).normalized();
    else
        rot_vec_z = Vector3d(0, 0, 1);
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
    pSE3Two->setTranslation(center); // it is the center of the old object points;  
                                    // it's better to use the center of the new complete points
    g2o::SE3Quat SE3Tow(pSE3Two->inverse());
    ORB_SLAM2::PointCloud *pObjectCloudNormalized = transformPointCloud(pObjectCloud, &SE3Tow); // normalized coordinate
    // VisualizePointCloud("normalizedPoints", pObjectCloudNormalized, Vector3d(0,0.4,0), 2);

    clock_t time_2_partPCA = clock();

    // begin symmetry plane estimation.
    // 开始对称平面估计
    SymmetryOutputData dataSymOutput;
    dataSymOutput.result = false;
    bool runSymmetry = false;
    if (mbOpenSymmetry) {
        // 1. Check symmetry type 检查对称类型
        bool hasSymmetry = (mmLabelSymmetry.find(label) != mmLabelSymmetry.end());
        int symmetryType = -1;
        if (hasSymmetry) {
            symmetryType = mmLabelSymmetry[label];
            if (symmetryType > 0)
                runSymmetry = true; // have valid symmetry type
        }
        if (runSymmetry) {
            // 3. initialize the symmetry solver 初始化对称求解器
            Symmetry ext;

            // Get a depth map whose values store the straight distances between the 3d points and camera center
            // 获得一个深度图，它的值存储了3D点和相机中心的直线距离
            cv::Mat projDepth = ext.getProjDepthMat(depth, camera);
            g2o::SE3Quat campose_wc;
            campose_wc.fromVector(pose.tail(7));
            g2o::SE3Quat campose_oc = SE3Tow * campose_wc;
            Eigen::VectorXd poseNormalized = campose_oc.toVector();

            SymmetrySolverData result = ext.estimateSymmetry(bbox, pObjectCloudNormalized, poseNormalized, projDepth, camera, symmetryType);

            // store the output
            dataSymOutput.pCloud = pObjectCloudNormalized;
            dataSymOutput.prob = result.prob;
            dataSymOutput.result = result.result;
            dataSymOutput.center = center;
            dataSymOutput.symmetryType = symmetryType;

            // store the sym plane
            g2o::plane symPlaneWorld(*result.pPlane);
            symPlaneWorld.transform(*pSE3Two);
            dataSymOutput.planeVec = symPlaneWorld.param; // store the symmetry plane in the world coordinate

            if (symmetryType == 2) // another symmetry plane for dual reflection
            {
                g2o::plane symPlaneWorld2(*result.pPlane2);
                symPlaneWorld2.transform(*pSE3Two);
                dataSymOutput.planeVec2 = symPlaneWorld2.param;
            }

            // complete the pointcloud using the symmetry plane if the estimation is successful
            if (dataSymOutput.result) {
                PointCloud *pSymCloudNormalized = SymmetrySolver::GetSymmetryPointCloud(pObjectCloudNormalized, *result.pPlane); // the plane in result is in normalized coordinate.

                if (symmetryType == 2) {
                    PointCloud *pSymCloudNormalized2_1 = SymmetrySolver::GetSymmetryPointCloud(pObjectCloudNormalized, *result.pPlane2);
                    PointCloud *pSymCloudNormalized2_2 = SymmetrySolver::GetSymmetryPointCloud(pSymCloudNormalized, *result.pPlane2);
                    CombinePointCloud(pSymCloudNormalized, pSymCloudNormalized2_1);
                    CombinePointCloud(pSymCloudNormalized, pSymCloudNormalized2_2);
                }

                // add the mirrored points to the object points
                int sym_num = pSymCloudNormalized->size();
                for (int i = 0; i < sym_num; i++)
                    pObjectCloudNormalized->push_back((*pSymCloudNormalized)[i]);

                // here, mirrwoed points have changed the center and rotation of the object points,
                // so we need a new transformation to move back the object points to the normalized coordinate

                // get the new center of the objects with mirrored points
                // 
                Vector3d centerCombined = GetPointcloudCenter(pObjectCloudNormalized);

                // to world coordinate
                Vector3d centerCombinedWorld = TransformPoint(centerCombined, pSE3Two->to_homogeneous_matrix());
                dataSymOutput.center = centerCombinedWorld;

                g2o::SE3Quat Tom; // mirror objects in normalized coordinate
                Tom.setTranslation(centerCombined);
                Matrix3d rotMat_om;                                              // object in world
                rotMat_om.col(0) = result.pPlane->param.head(3);                 // x: the normal of symmetry plane 1
                rotMat_om.col(0) = rotMat_om.col(0) / (rotMat_om.col(0).norm()); // normalize
                rotMat_om.col(2) = Vector3d(0, 0, 1);                            // z: the normal of the groundplane, which is Z(0,0,1)
                rotMat_om.col(1) = rotMat_om.col(2).cross(rotMat_om.col(0));     // y:  z cross x

                Eigen::Quaterniond quat_om(rotMat_om);
                Tom.setRotation(quat_om);

                // transform points
                g2o::SE3Quat Tmo = Tom.inverse();
                transformPointCloudSelf(pObjectCloudNormalized, &Tmo); // po->pm

                // change T
                g2o::SE3Quat *pSE3Twm = new g2o::SE3Quat();
                (*pSE3Twm) = (*pSE3Two) * Tom;
                pSE3Two = pSE3Twm;
            }
            mSymmetryOutputData = dataSymOutput;

            // VisualizePointCloud("WithSymNormalized", pObjectCloudNormalized, Vector3d(0,0,0.4), 2);

            // transform to the world
            ORB_SLAM2::PointCloud *pObjectCloudWithSymWorld = ORB_SLAM2::transformPointCloud(pObjectCloudNormalized, pSE3Two);
            VisualizePointCloud("Mirrored Points", pObjectCloudWithSymWorld, Vector3d(0, 1.0, 0.2), 8);

        } // end of symmetry type
    }     // end of the symmetry.
    clock_t time_3_symmetryEstimation = clock();

    // Estimate an ellipsoid from the complete object points
    // calculate the covariance along three main axes
    g2o::ellipsoid e_zero_normalized = GetEllipsoidFromNomalizedPointCloud(pObjectCloudNormalized);

    // transform back to the world coordinate
    g2o::ellipsoid e_global_normalized = e_zero_normalized.transform_from(*pSE3Two);

    // transform to the local coordinate.
    g2o::SE3Quat campose_wc;
    campose_wc.fromVector(pose);
    g2o::ellipsoid e_local_normalized = e_global_normalized.transform_from(campose_wc.inverse());
    clock_t time_5_zeroPCA = clock();

    // output the main running time
    // cout << "****** System Time [EllipsoidExtractor.cpp] ******" << endl ;
    // cout << "time_ExtractPointCloud: " <<(double)(time_1_ExtractPointCloud - time_start) / CLOCKS_PER_SEC << "s" << endl;
    // // cout << "time_partPCA: " <<(double)(time_2_partPCA - time_1_ExtractPointCloud) / CLOCKS_PER_SEC << "s" << endl;      // too small
    // cout << "time_symmetryEstimation: " <<(double)(time_3_symmetryEstimation - time_2_partPCA) / CLOCKS_PER_SEC << "s" << endl;
    // // cout << "time_zeroPCA: " <<(double)(time_5_zeroPCA - time_3_symmetryEstimation) / CLOCKS_PER_SEC << "s" << endl << endl;     // too small
    // cout << "total_ellipsoidExtraction: " <<(double)(time_5_zeroPCA - time_start) / CLOCKS_PER_SEC << "s" << endl;

    g2o::ellipsoid e_local = e_local_normalized;

    // calculate the probability of the single-frame ellipsoid estimation
    double prob_symmetry;
    if (runSymmetry)
        prob_symmetry = dataSymOutput.prob; // when the symmetry is opened
    else
        prob_symmetry = 1.0; // when the symmetry is closed, set it to a constant

    e_local.prob = prob_symmetry * prob; // measurement_prob * symmetry_prob
    e_local.miLabel = label;

    mResult = true;
    return e_local;
}

PCAResult EllipsoidExtractor::ProcessPCANormalized(ORB_SLAM2::PointCloud *pObject) {
    PCAResult data;
    double x, y, z;
    x = 0;
    y = 0;
    z = 0;
    int num = pObject->size();

    double max_x = 0, min_x = 0;
    double max_y = 0, min_y = 0;
    double max_z = 0, min_z = 0;
    for (int i = 0; i < num; i++) {
        double px = (*pObject)[i].x;
        double py = (*pObject)[i].y;
        double pz = (*pObject)[i].z;

        // x += px * px;
        // y += py * py;
        // z += pz * pz;

        if (px > max_x)
            max_x = px;
        if (py > max_y)
            max_y = py;
        if (pz > max_z)
            max_z = pz;

        if (px < min_x)
            min_x = px;
        if (py < min_y)
            min_y = py;
        if (pz < min_z)
            min_z = pz;
    }

    // x /= num;
    // y /= num;
    // z /= num;
    double center_x = (max_x + min_x) / 2.0;
    double center_y = (max_y + min_y) / 2.0;
    double center_z = (max_z + min_z) / 2.0;

    double scale_x = (max_x - min_x) / 2.0;
    double scale_y = (max_y - min_y) / 2.0;
    double scale_z = (max_z - min_z) / 2.0;

    // data.covariance = Vector3d(x,y,z);
    data.center = Vector3d(center_x, center_y, center_z); // a new center is also needed
    data.rotMat = Matrix3d::Identity();
    data.scale << scale_x, scale_y, scale_z;

    return data;
}

SymmetryOutputData EllipsoidExtractor::GetSymmetryOutputData() {
    return mSymmetryOutputData;
}

ORB_SLAM2::PointCloud *EllipsoidExtractor::GetPointCloudInProcess() {
    return mpPoints;
}

ORB_SLAM2::PointCloud *EllipsoidExtractor::GetPointCloudDebug() {
    return mpPointsDebug;
}

// pointcloud process
ORB_SLAM2::PointCloud *EllipsoidExtractor::ApplyPlaneFilter(ORB_SLAM2::PointCloud *pCloud, double z) {
    ORB_SLAM2::PointCloud *pCloudFiltered = new ORB_SLAM2::PointCloud;
    int num = pCloud->size();
    for (auto p : (*pCloud)) {
        if (p.z > z)
            pCloudFiltered->push_back(p);
    }

    return pCloudFiltered;
}

ORB_SLAM2::PointCloud *EllipsoidExtractor::ApplySupportingPlaneFilter(ORB_SLAM2::PointCloud *pCloud) {
    ORB_SLAM2::PointCloud *pCloudFiltered = new ORB_SLAM2::PointCloud;
    int num = pCloud->size();

    double config_dis_thresh = Config::Get<double>("EllipsoidExtractor.SupportingPlaneFilter.DisThresh");
    // if(mbLocalSupportingPlane)
    //     config_dis_thresh = 0.01;
    // else
    //     config_dis_thresh = 0.03;

    // // DEBUG: 完全取消支撑平面分割!
    // config_dis_thresh = -50;

    int i = 0;
    for (auto p : (*pCloud)) {
        double dis = mpPlane->distanceToPoint(Vector3d(p.x, p.y, p.z), true); // ture means keeping the flag. The PlaneExtractor has made sure the positive value means the point is above the plane.

        bool ok = dis > config_dis_thresh;

        if (ok)
            pCloudFiltered->push_back(p);
    }

    return pCloudFiltered;
}

// The function will generate the 3D center point of the object from the bounding box and the depth image, which will help select the object cluster among all the clusters from Euclidean filter.
// Considering the robustness, several points will be sampled around the center point of the bounding box,
//  and their average 3D positions will be taken as the output.
bool EllipsoidExtractor::GetCenter(cv::Mat &depth, Eigen::Vector4d &bbox, Eigen::VectorXd &pose, camera_intrinsic &camera, Vector3d &center) {
    double depth_range = Config::ReadValue<double>("EllipsoidExtractor_DEPTH_RANGE");
    // cout << "[EllipsoidExtractor::GetCenter] depth_range = " << depth_range << std::endl;
    // get the center of the bounding box
    int x = int((bbox(0) + bbox(2)) / 2.0);
    int y = int((bbox(1) + bbox(3)) / 2.0);

    // std::cout << "bbox = " << bbox.transpose().matrix() << std::endl;

    int point_num = 10; // sample 10 * 10 points
    int x_delta = std::abs(bbox(0) - bbox(2)) / 4.0 / point_num;
    int y_delta = std::abs(bbox(1) - bbox(3)) / 4.0 / point_num;

    PointCloudPCL::Ptr pCloud(new PointCloudPCL);
    PointCloudPCL &cloud = *pCloud;
    for (int x_id = -point_num / 2; x_id < point_num / 2; x_id++) {
        for (int y_id = -point_num / 2; y_id < point_num / 2; y_id++) {
            int x_ = x + x_id * x_delta;
            int y_ = y + y_id * y_delta;
            // cout << "x_ = " << x_ << ", y_ = " << y_ << std::endl; 
            ushort *ptd = depth.ptr<ushort>(y_);
            ushort d = ptd[x_];

            PointT p;
            p.z = d / camera.scale;
            // if the depth value is invalid, ignore this point
            if (p.z <= 0.1 || p.z > depth_range)
                continue;
            // cout << "p.z = " << p.z << std::endl;
            p.x = (x_ - camera.cx) * p.z / camera.fx;
            p.y = (y_ - camera.cy) * p.z / camera.fy;
            cloud.points.push_back(p);
        }
    }
    mDebugCenterCloud = pCloud; // store for visualization

    if (cloud.size() < 2){
        cout << "Validcloud.size() < 2" << endl;
        return false; // we need at least 2 valid points
    }

    Eigen::Vector4d centroid;
    pcl::compute3DCentroid(cloud, centroid); // get their centroid

    ORB_SLAM2::PointXYZRGB p;
    p.x = centroid[0];
    p.y = centroid[1];
    p.z = centroid[2];

    // transform to the world coordintate
    g2o::SE3Quat campose_wc;
    campose_wc.fromVector(pose);
    Eigen::Matrix4d Twc = campose_wc.to_homogeneous_matrix();

    Eigen::Vector3d xyz(p.x, p.y, p.z);
    Eigen::Vector4d Xc = real_to_homo_coord<double>(xyz);

    Eigen::Vector4d Xw = Twc * Xc;
    Eigen::Vector3d xyz_w = homo_to_real_coord<double>(Xw);

    center = xyz_w;

    return true;
}

ORB_SLAM2::PointCloud *EllipsoidExtractor::ApplyEuclideanFilter(ORB_SLAM2::PointCloud *pCloud, Vector3d &center) {
    clock_t time_1_start = clock();

    // load config parameters
    int CONFIG_MinClusterSize = Config::Get<int>("EllipsoidExtraction.Euclidean.MinClusterSize");
    double CONFIG_ClusterTolerance = Config::Get<double>("EllipsoidExtraction.Euclidean.ClusterTolerance");
    double CONFIG_CENTER_DIS = Config::Get<double>("EllipsoidExtraction.Euclidean.CenterDis");

    assert(CONFIG_MinClusterSize > 0 && CONFIG_ClusterTolerance > 0 && CONFIG_CENTER_DIS > 0 && "Forge to set param. ");

    pcl::PointCloud<pcl::PointXYZ>::Ptr pCloudPCL = QuadricPointCloudToPclXYZ(*pCloud);

    // Creating the KdTree object for the search method of the extraction
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>); // use kdTree to speed up
    tree->setInputCloud(pCloudPCL);

    int point_num = pCloudPCL->size();

    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(CONFIG_ClusterTolerance); // an important parameter. it must be larger than the grid size in the down sampling process
    ec.setMinClusterSize(CONFIG_MinClusterSize);
    ec.setMaxClusterSize(point_num);

    ec.setSearchMethod(tree);
    ec.setInputCloud(pCloudPCL);
    ec.extract(cluster_indices);

    bool bFindCluster = false;
    pcl::PointCloud<pcl::PointXYZ>::Ptr pFinalPoints;

    clock_t time_2_extractClusters = clock();

    int cluster_size = cluster_indices.size();

    // store the point clouds
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloud_cluster_vector;
    for (auto it = cluster_indices.begin(); it != cluster_indices.end(); it++) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
        for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); ++pit)
            cloud_cluster->points.push_back(pCloudPCL->points[*pit]);
        cloud_cluster_vector.push_back(cloud_cluster);
    }
    mDebugEuclideanFilterClouds = cloud_cluster_vector; // store for debugging

    clock_t time_3_getClusterPoints = clock();

    // select the cluster that has a distance below a threshold with the 3D center point projected near the center of the 2D bounding box
    for (int i = 0; i < cluster_size; i++) {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster = cloud_cluster_vector[i];

        if (cluster_size == 1) {
            pFinalPoints = cloud_cluster;
            bFindCluster = true;
        }

        double dis = getDistanceFromPointToCloud(center, cloud_cluster);
        bool c2 = false;
        if (dis < CONFIG_CENTER_DIS)
            c2 = true; // the distance should be small enough
        if (c2) {
            pFinalPoints = cloud_cluster;
            bFindCluster = true;
            break;
        }
    }

    ORB_SLAM2::PointCloud *pCloudFiltered;
    if (!bFindCluster) {
        miEuclideanFilterState = 3;
        return pCloudFiltered;
    }

    clock_t time_4_selectTheBestCluster = clock();

    pCloudFiltered = new ORB_SLAM2::PointCloud(pclXYZToQuadricPointCloud(pFinalPoints));
    miEuclideanFilterState = 0;

    clock_t time_5_copyNewCloud = clock();

    // cout << "****** System Time [Euclidean Filter] ******" << endl ;
    // cout << "time_2_extractClusters: " <<(double)(time_2_extractClusters - time_1_start) / CLOCKS_PER_SEC << "s" << endl;
    // cout << "time_3_getClusterPoints: " <<(double)(time_3_getClusterPoints - time_2_extractClusters) / CLOCKS_PER_SEC << "s" << endl;
    // cout << "time_4_selectTheBestCluster: " <<(double)(time_4_selectTheBestCluster - time_3_getClusterPoints) / CLOCKS_PER_SEC << "s" << endl;
    // cout << "time_5_copyNewCloud: " <<(double)(time_5_copyNewCloud - time_4_selectTheBestCluster) / CLOCKS_PER_SEC << "s" << endl;

    return pCloudFiltered;
}

double EllipsoidExtractor::getDistanceFromPointToCloud(Vector3d &point, pcl::PointCloud<pcl::PointXYZ>::Ptr pCloud) {
    double mini_dis = 999999;
    if (pCloud->size() < 1)
        return -1;

    for (auto p : *pCloud) {
        Vector3d p_(p.x, p.y, p.z);
        double dis = (point - p_).norm();
        if (dis < mini_dis)
            mini_dis = dis;
    }
    return mini_dis;
}

void EllipsoidExtractor::SetSupportingPlane(g2o::plane *pPlane, bool local) {
    mpPlane = pPlane;
    mbSetPlane = true;

    mbLocalSupportingPlane = local;
}

void EllipsoidExtractor::AdjustChirality(PCAResult &data) {
    // using cross operation to generate a new axis to make the coordinate right-handed
    data.rotMat.col(2) = data.rotMat.col(0).cross(data.rotMat.col(1));
    return;
}

Eigen::Matrix3d EllipsoidExtractor::calibRotMatAccordingToGroundPlane(Matrix3d &rotMat, const Vector3d &normal) {
    // in order to apply a small rotation to align the z axis of the object and the normal vector of the groundplane,
    // we need calculate the rotation axis and its angle.

    // first get the rotation axis
    Vector3d ellipsoid_zAxis = rotMat.col(2);
    Vector3d rot_axis = ellipsoid_zAxis.cross(normal);
    if (rot_axis.norm() > 0) // if rot_axis is 0,0,0; there will be an error
        rot_axis.normalize();

    // then get the angle between the normal of the groundplane and the z axis of the object
    double norm1 = normal.norm();
    double norm2 = ellipsoid_zAxis.norm();
    double vec_dot = normal.transpose() * ellipsoid_zAxis;
    double cos_theta = vec_dot / norm1 / norm2;
    double theta = acos(cos_theta);

    // generate the rotation vector
    AngleAxisd rot_angleAxis(theta, rot_axis);

    Matrix3d rotMat_calibrated = rot_angleAxis * rotMat;

    return rotMat_calibrated;
}

void EllipsoidExtractor::OpenVisualization(Map *pMap) {
    mbOpenVisualization = true;
    mpMap = pMap;
}

void EllipsoidExtractor::ClearPointCloudList() {
    if (mbOpenVisualization) {
        mpMap->DeletePointCloudList("EllipsoidExtractor", 1); // partial martching
    }
}

void EllipsoidExtractor::VisualizePointCloud(const string &name, ORB_SLAM2::PointCloud *pCloud, const Vector3d &color, int point_size) {
    if (mbOpenVisualization) {
        // if the color is not set, use random color
        uchar r, g, b;
        if (color[0] < -0.01 || color[1] < -0.01 || color[2] < -0.01) {
            srand(time(0));
            r = rand() % 155;
            g = rand() % 155;
            b = rand() % 155;
        } else {
            r = 255 * color[0];
            g = 255 * color[1];
            b = 255 * color[2];
        }

        SetPointCloudProperty(pCloud, r, g, b, point_size);

        string full_name = string("EllipsoidExtractor.") + name;

        mpMap->AddPointCloudList(full_name, pCloud, 1);
    }
}

void EllipsoidExtractor::VisualizeEllipsoid(const string &name, g2o::ellipsoid *pObj) {
    if (mbOpenVisualization) {
        mpMap->addEllipsoidVisual(pObj);
    }
}

void EllipsoidExtractor::OpenSymmetry() {
    std::cout << std::endl;
    std::cout << " * Open Symmetry Estimation. " << std::endl;

    LoadSymmetryPrior();

    mbOpenSymmetry = true;
}

g2o::ellipsoid EllipsoidExtractor::GetEllipsoidFromNomalizedPointCloud(ORB_SLAM2::PointCloud *pCloud) {
    g2o::ellipsoid e_zero_normalized;
    // 注意，这里是以规范化PCA的模式进行处理的
    PCAResult dataCenterPCA = ProcessPCANormalized(pCloud); // find the max value along x,y,z axis
    e_zero_normalized = ConstructEllipsoid(dataCenterPCA);
    return e_zero_normalized;
}

void EllipsoidExtractor::SetExtractionMethod(int method) {
    miMethod = method;
}

} // namespace ORB_SLAM2
