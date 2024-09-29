// 将评估工具正式封装到函数内部.

#pragma once

#include <iostream>
#include <fstream>
#include <Eigen/Core>
#include <algorithm>

#include "include/utils/dataprocess_utils.h"

#include <core/Ellipsoid.h>
#include <core/Geometry.h>
// #include <System.h>
#include <Map.h>

#include "Example/interface/func/func.h"

#include "Hungarian.h"

#include <boost/algorithm/string.hpp>

// #include <pybind11/embed.h>
// #include <pybind11/eigen.h>

// namespace py = pybind11;

using namespace std;
using namespace Eigen;
using namespace ORB_SLAM2;

// ORB_SLAM2::System *pSLAM;
ORB_SLAM2::Map *pMap;

namespace evo
{
    struct COMPARE_RESULT
    {
        int instanceID;

        double IoU;         // 原始IoU
        double IoU_aligned; // 对齐中心后的IoU

        double dis_trans;
        double dis_yaw;

        // 新函数结果测试
        double dis_yaw_arbitrary;
        double IoU_aligned_arbitrary;

        // 记录 IoU 的计算方式
        std::vector<int> iou_order;
        int est_id; // 记录est在mat中的行数
        int ref_id;

        // 记录角度是否应该生效, 对称物体将不评估角度
        bool valid_angle;

        // 记录 est_id_origin
        int est_id_origin;
    };

    struct StaticResult
    {
        double aver_IoU;
        double aver_IoU_aligned;
        double aver_dis_trans;
        double aver_dis_yaw;
        double aver_dis_yaw_arb;
        double aver_dis_yaw_arb_valid;
        double aver_IoU_aligned_arb;
        double mid_iou;
        double mid_iou_aligned;
        double mid_dis_trans;
        double mid_dis_yaw;
        double mid_dis_yaw_arb;
        double mid_dis_yaw_arb_valid;
        double mid_iou_aligned_arb;
        double rmse;
        double precision;
        double recall;
        double F1;

        std::set<int> list_noAlgin;
    };

    bool Evaluate(MatrixXd &refObjMat, MatrixXd &estObjMat, MatrixXd &gtTrajMat, MatrixXd &estTrajMat, StaticResult &output,
                std::map<int, COMPARE_RESULT>& output_results, ORB_SLAM2::Map *pMap = NULL, bool manual_check = false, bool filter_sym = false,
                bool use_id_to_associate = false);

    void OutputResult(ostream &fio, StaticResult &staticResult, std::map<int, COMPARE_RESULT> &results, MatrixXd &refObjMat);

    // Tools Function
    MatrixXd transformObjMat(MatrixXd &objMat, g2o::SE3Quat &T);

    double getYawError(g2o::ellipsoid &e1, g2o::ellipsoid &e2);

    void VisualizeEllipsoidsInMat(MatrixXd &mat, const Vector3d &color, ORB_SLAM2::Map *pMap, bool show_instance, bool show_in_observation = false);


    std::map<int, std::map<double, int>> VisualizeAndRevisedAssociations(const std::map<int, std::map<double, int>>& objObs, 
        const std::vector<Frame*> &vpFrames, MatrixXd& refObjMat, ORB_SLAM2::Map* pMap);
}