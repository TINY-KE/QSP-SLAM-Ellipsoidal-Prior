// Update 2020-6-27
// 将坐标系统一对齐到gt下.

// Update 2020-6-18
// 转移封装.

// Update 2020-6-9
// 添加 HungarianAlgorithm 做最佳关联解算。

// Update 2020-6-3
// 添加关联结果的可视化，以做肉眼确认.

// Update 2020-5-29:
// 1) 适应新版本 Ellipsoid::Core.
// 2) 添加轨迹对齐和自动关联.

// Update 2020-2-22:
// 假定椭球体z轴已对齐(支持粗略对齐)，计算 IoU.

// Update : 2020-1-4 完善椭球体评估功能
// 评估物体重建和真实位置之间的差值
#include "evo.h"

namespace evo
{
    bool compare_pair_int_double(std::pair<int, double> &p1, std::pair<int, double> &p2)
    {
        return p1.second < p2.second;
    }

    bool JudgeInsideRec(Vector3d& X, g2o::ellipsoid& e)
    {
        g2o::SE3Quat Two = e.pose;
        g2o::SE3Quat Tow = Two.inverse();
        Vector3d X_local = TransformPoint(X, Tow.to_homogeneous_matrix());

        double x = std::abs(X_local[0]);
        double y = std::abs(X_local[1]);
        double z = std::abs(X_local[2]);

        Vector3d scale = e.scale.cwiseAbs();
        if(x < scale[0] && y < scale[1] && z < scale[2])
            return true;
        else 
            return false;
    }

    // obj : instance x y z r p yaw a b c
    //           0    1 2 3 4 5  6  7 8 9
    // Resolution: 点的分辨率 m. 总共Sample的点为 (Volumn/Resolution^3)
    // Shape: 0 Quadrics, 1 Rectangle
    double MonteCarloIoU(VectorXd &ob1, VectorXd &ob2, int sample_num = 1000, int shape = 1)
    {
        bool bDebug = false;

        // 获得立方体.
        // 取 x,y,z最大值
        double radius1 = MAX(MAX(ob1[7], ob1[8]), ob1[9]);
        double radius2 = MAX(MAX(ob2[7], ob2[8]), ob2[9]);

        double x_max = MAX(ob1[1] + radius1, ob2[1] + radius2);
        double x_min = MIN(ob1[1] - radius1, ob2[1] - radius2);
        double y_max = MAX(ob1[2] + radius1, ob2[2] + radius2);
        double y_min = MIN(ob1[2] - radius1, ob2[2] - radius2);
        double z_max = MAX(ob1[3] + radius1, ob2[3] + radius2);
        double z_min = MIN(ob1[3] - radius1, ob2[3] - radius2);

        double length_x = x_max - x_min;
        double length_y = y_max - y_min;
        double length_z = z_max - z_min;

        double total_volumn = length_x * length_y * length_z;
        double resolution = pow(total_volumn / sample_num, 1.0/3.0);

        // 随机 Sample 点云
        int total_num = 0;

        int ob1_num = 0;
        int ob2_num = 0;
        int both_num = 0;

        g2o::ellipsoid e1;
        e1.fromMinimalVector(ob1.head(10).tail(9));
        Eigen::Matrix4d Q1_star = e1.generateQuadric();
        Eigen::Matrix4d Q1 = Q1_star.inverse();

        g2o::ellipsoid e2;
        e2.fromMinimalVector(ob2.head(10).tail(9));
        Eigen::Matrix4d Q2_star = e2.generateQuadric();
        Eigen::Matrix4d Q2 = Q2_star.inverse();

        ORB_SLAM2::PointCloud *pCloud = new ORB_SLAM2::PointCloud;
        for (double x = x_min; x < x_max; x += resolution)
        {
            for (double y = y_min; y < y_max; y += resolution)
            {
                for (double z = z_min; z < z_max; z += resolution)
                {
                    Vector3d point_vec(x, y, z);
                    Eigen::Vector4d X = real_to_homo_coord_vec<double>(point_vec);

                    bool isInside_1, isInside_2;
                    bool bUseQuadric = (shape==0);

                    // For Debugging
                    // // if(bUseQuadric){
                    //     isInside_1 = (X.transpose() * Q1 * X) < 0;
                    //     isInside_2 = (X.transpose() * Q2 * X) < 0;


                    //     if(isInside_1 && isInside_2)
                    //         std::cout << "OK Break!" << std::endl;
                    // // }
                    // // else 
                    // // {
                    //     bool rec_isInside_1, rec_isInside_2;
                    //     rec_isInside_1 = JudgeInsideRec(point_vec, e1);
                    //     rec_isInside_1 = JudgeInsideRec(point_vec, e2);

                    // // }

                    if(bUseQuadric)
                    {
                        isInside_1 = (X.transpose() * Q1 * X) < 0;
                        isInside_2 = (X.transpose() * Q2 * X) < 0;
                    }
                    else 
                    {
                        isInside_1 = JudgeInsideRec(point_vec, e1);
                        isInside_2 = JudgeInsideRec(point_vec, e2);
                    }
                    
                    
                    if (isInside_1)
                        ob1_num++;
                    if (isInside_2)
                        ob2_num++;
                    if (isInside_1 && isInside_2)
                        both_num++;

                    total_num++;

                    ORB_SLAM2::PointXYZRGB p;
                    p.x = x;
                    p.y = y;
                    p.z = z;

                    // 默认颜色
                    p.r = 0;
                    p.g = 0;
                    p.b = 0;

                    if (isInside_1)
                        p.r = 255;
                    if (isInside_2)
                        p.b = 255;
                    if (isInside_1 && isInside_2)
                        p.g = 128;
                    p.size = 5;
                    pCloud->push_back(p);
                }
            }
        }

        // 统计
        if(bDebug){
            std::cout << "ob1/ob2/both : " << ob1_num << "/" << ob2_num << "/" << both_num << std::endl;
            std::cout << "Total Num : " << total_num << std::endl;

            // 可视化部分. 显示椭球体、所有数据点, 并按类别标记颜色
            g2o::ellipsoid *pE1 = new g2o::ellipsoid(e1);
            pE1->setColor(Vector3d(0, 0, 1));
            g2o::ellipsoid *pE2 = new g2o::ellipsoid(e2);
            pE2->setColor(Vector3d(1, 0, 0));
            pMap->ClearEllipsoidsVisual();
            pMap->addEllipsoidVisual(pE1);
            pMap->addEllipsoidVisual(pE2);

            pMap->clearPointCloud();
            pMap->addPointCloud(pCloud);

            // 显示大区域
            Vector9d area_vec;
            double x_center = (x_min + x_max) / 2;
            double y_center = (y_min + y_max) / 2;
            double z_center = (z_min + z_max) / 2;
            double x_halfSize = (x_max - x_min) / 2;
            double y_halfSize = (y_max - y_min) / 2;
            double z_halfSize = (z_max - z_min) / 2;
            area_vec << x_center, y_center, z_center, 0, 0, 0, x_halfSize, y_halfSize, z_halfSize;
            g2o::ellipsoid *pE_Area = new g2o::ellipsoid;
            pE_Area->fromMinimalVector(area_vec);
            pE_Area->setColor(Vector3d(0, 1, 0));
            pMap->addEllipsoidVisual(pE_Area);
            
            std::cout << "Show result of sampling for IoU ... Push any key to continue ... " << std::endl;
            getchar();
        }

        // 输出结果.
        double IoU = both_num / double(ob1_num + ob2_num);
        return IoU;
    }

    std::vector<Matrix3d, Eigen::aligned_allocator<Matrix3d>> GenerateAllPotentialMat(Matrix3d &rotMat)
    {
        std::vector<Matrix3d, Eigen::aligned_allocator<Matrix3d>> matLists;
        // 6
        // axis(x 0, y 1, z 2, flag+-)
        for (int i = 0; i < 6; i++)
        {
            int axis_id = i / 2;
            bool positive_flag = i % 2;

            Vector3d z_ = rotMat.col(axis_id) * (positive_flag ? 1 : -1);

            for (int n = 0; n < 4; n++)
            {
                int axis_id_delta = n / 2 + 1;
                int axis_id_x = (axis_id + axis_id_delta) % 3;

                bool positive_flag_x = n % 2;

                Vector3d x_ = rotMat.col(axis_id_x) * (positive_flag_x ? 1 : -1);
                Vector3d y_ = z_.cross(x_);

                Matrix3d rotMat_;
                rotMat_.col(0) = x_;
                rotMat_.col(1) = y_;
                rotMat_.col(2) = z_;

                matLists.push_back(rotMat_);
            }
        }

        return matLists;
    }

    // 2020-6-9 任意两个椭球体的最小旋转 Rotation angle
    double GetMinAngleArbitrary(g2o::ellipsoid &e1, g2o::ellipsoid &e2)
    {

        Matrix3d rotMat1(e1.pose.rotation());
        Matrix3d rotMat2(e2.pose.rotation());

        std::vector<Matrix3d, Eigen::aligned_allocator<Matrix3d>> rotMat2_allpos = GenerateAllPotentialMat(rotMat2);
        std::vector<double> angle_vec;
        for (int i = 0; i < 24; i++)
        {
            Matrix3d diffMat = rotMat1.inverse() * rotMat2_allpos[i];
            // 其实就是绕 x,y,z 轴旋转, 遍历所有可能性 ( 一共 6 个? )
            AngleAxisd axis(diffMat);
            double angle = axis.angle();

            double abs_angle = std::abs(angle);
            angle_vec.push_back(abs_angle);
        }
        // 思考如何生成 24种 所有可能性吧...
        // TOBEDONE.

        auto iter_min = std::min_element(angle_vec.begin(), angle_vec.end());

        double min_angle_deg = *iter_min / M_PI * 180;
        return min_angle_deg;
    }

    // 2020-6-9: 对任意两个椭球体，无需对齐，计算 Aligned IoU.
    // obj : instance x y z r p yaw a b c
    //           0    1 2 3 4 5  6  7 8 9
    double GetAlignedMIoUArbitrary(VectorXd &ob1, VectorXd &ob2, std::vector<int> &iou_order)
    {
        // align
        Vector9d vec1;
        vec1 << 0, 0, 0, 0, 0, 0, ob1[7], ob1[8], ob1[9];
        g2o::ellipsoid e1;
        e1.fromMinimalVector(vec1);

        std::vector<std::vector<int>> vec_combinations = {
            {0, 1, 2}, {0, 2, 1}, {1, 0, 2}, {1, 2, 0}, {2, 0, 1}, {2, 1, 0}};

        std::vector<double> vIous;
        vIous.resize(6);
        for (int i = 0; i < 6; i++) // 一共六种可能性.
        {
            Vector9d vec2;
            int first = vec_combinations[i][0] + 7;
            int second = vec_combinations[i][1] + 7;
            int third = vec_combinations[i][2] + 7;
            vec2 << 0, 0, 0, 0, 0, 0, ob2[first], ob2[second], ob2[third];
            g2o::ellipsoid e2;
            e2.fromMinimalVector(vec2);
            double iou_aligned = e1.calculateMIoU(e2);

            vIous[i] = iou_aligned;
        }
        auto iter = std::min_element(vIous.begin(), vIous.end());
        int pos = iter - vIous.begin();
        iou_order = vec_combinations[pos];
        return *iter;
    }

    // 2020-2-22: 假定椭球体z轴已对齐(支持粗略对齐)，计算 MIoU.
    // obj : instance x y z r p yaw a b c
    //           0    1 2 3 4 5  6  7 8 9
    double GetAxisAlignedMIoU(VectorXd &ob1, VectorXd &ob2, bool align = false)
    {
        if (!align)
        {
            g2o::ellipsoid e1;
            e1.fromMinimalVector(ob1.block(1, 0, 9, 1));
            g2o::ellipsoid e2;
            e2.fromMinimalVector(ob2.block(1, 0, 9, 1));

            return e1.calculateMIoU(e2);
        }
        else
        {
            // align
            Vector9d vec1;
            vec1 << 0, 0, 0, 0, 0, 0, ob1[7], ob1[8], ob1[9];

            Vector9d vec2_a;
            vec2_a << 0, 0, 0, 0, 0, 0, ob2[7], ob2[8], ob2[9];
            Vector9d vec2_b;
            vec2_b << 0, 0, 0, 0, 0, 0, ob2[8], ob2[7], ob2[9];

            g2o::ellipsoid e1;
            e1.fromMinimalVector(vec1);
            g2o::ellipsoid e2_a;
            e2_a.fromMinimalVector(vec2_a);
            g2o::ellipsoid e2_b;
            e2_b.fromMinimalVector(vec2_b);

            double iou_aligned1 = e1.calculateMIoU(e2_a);
            double iou_aligned2 = e1.calculateMIoU(e2_b);

            g2o::ellipsoid *pE1 = new g2o::ellipsoid(e1);
            pE1->setColor(Vector3d(1, 0, 0));

            g2o::ellipsoid *pE2_a = new g2o::ellipsoid(e2_a);
            pE2_a->setColor(Vector3d(0, 0, 1));
            g2o::ellipsoid *pE2_b = new g2o::ellipsoid(e2_b);
            pE2_b->setColor(Vector3d(0, 0, 1));

            double iou_out;
            if (iou_aligned1 < iou_aligned2)
            {
                iou_out = iou_aligned1;
                pE2_a->setColor(Vector3d(0, 1, 0));
                // std::cout << "iouE_wrong : " << iou_aligned2 << std::endl;
            }
            else
            {
                iou_out = iou_aligned2;
                pE2_b->setColor(Vector3d(0, 1, 0));
                // std::cout << "iouE_wrong : " << iou_aligned1 << std::endl;
            }

            // pMap->ClearEllipsoidsVisual();
            // pMap->addEllipsoidVisual(pE1);
            // pMap->addEllipsoidVisual(pE2_a);
            // pMap->addEllipsoidVisual(pE2_b);

            // std::cout << "iouE_selected : " << iou_out << std::endl;

            return iou_out;
        }
    }

    double normalizeAngle(double angle)
    {
        double angle_360 = atan2(sin(angle), cos(angle)); // -pi ~ pi

        double angle_180; // 0 ~ pi
        if (angle_360 < 0)
            angle_180 = angle_360 + M_PI;
        else
            angle_180 = angle_360;
        return angle_180;
    }

    double getYawError(g2o::ellipsoid &e1, g2o::ellipsoid &e2)
    {
        Eigen::Quaterniond q1 = e1.pose.rotation();
        Eigen::Quaterniond q2 = e2.pose.rotation();
        Eigen::Quaterniond q_diff = q1.inverse() * q2;
        Eigen::AngleAxisd angleAxis(q_diff);
        double dis_yaw = std::abs(angleAxis.angle());
        // dis_yaw = normalizeAngle(dis_yaw);
        // dis_yaw = std::abs(normalize_to_pi<double>(dis_yaw));

        double mini_dis_yaw = dis_yaw; // [ 0, 90deg ]

        double yaw_deg = mini_dis_yaw / M_PI * 180; // 单位 deg

        return yaw_deg;
    }

    double GetMinYawAngle(g2o::ellipsoid &e1, g2o::ellipsoid &e2)
    {

        Vector4d rotate_errors_norm;
        Vector4d rotate_angles(-1, 0, 1, 2); // rotate -90 0 90 180
        for (int i = 0; i < rotate_errors_norm.rows(); i++)
        {
            ellipsoid e2_rotated = e2.rotate_ellipsoid(rotate_angles(i) * M_PI / 2.0); // rotate new cuboids

            double yaw_error = getYawError(e1, e2_rotated);
            rotate_errors_norm(i) = yaw_error;
        }
        int min_label;
        rotate_errors_norm.minCoeff(&min_label);
        return rotate_errors_norm[min_label];
    }

    bool IsAxisAligned(VectorXd &ob1,VectorXd &ob2)
    {
        g2o::ellipsoid e1;
        e1.fromMinimalVector(ob1.block(1, 0, 9, 1));
        g2o::ellipsoid e2;
        e2.fromMinimalVector(ob2.block(1, 0, 9, 1));

        // 如何判断两个椭球体的轴是否对齐? 检查其Z轴角度差
        Eigen::Vector3d Z1 = e1.pose.rotation().toRotationMatrix().col(2);
        Eigen::Vector3d Z2 = e2.pose.rotation().toRotationMatrix().col(2);
        
        double cc = Z1.transpose() * Z2;

        double cos_angle_diff = cc / Z1.norm() / Z2.norm();
        double angle_diff = std::abs(acos(cos_angle_diff));

        static double ANGLE_THRESH = M_PI / 180.0 * 5;
        if(angle_diff > ANGLE_THRESH) return false;
        else return true;
    }

    // ob1: ref
    // ob2: est
    COMPARE_RESULT compareObjectWithVector(VectorXd &ob1, VectorXd &ob2)
    {
        // 综合评价: 直接比较IoU

        // 位置: 原点差距
        Vector3d ob1_ori = ob1.block(1, 0, 3, 1);
        Vector3d ob2_ori = ob2.block(1, 0, 3, 1);
        double origin_distance = (ob1_ori - ob2_ori).norm();

        // 原始IoU

        // 首先判断Z轴是否对齐了!
        double IoU_ori;
        if(IsAxisAligned(ob1,ob2))
            IoU_ori = 1 - GetAxisAlignedMIoU(ob1, ob2, false);
        else 
            IoU_ori = MonteCarloIoU(ob1, ob2, 1000);

        // 放到原点比较IoU
        double IoU_aligned = 1 - GetAxisAlignedMIoU(ob1, ob2, true);

        // getchar();
        // 构造旋转矩阵比较 angle.
        g2o::ellipsoid e1;
        e1.fromMinimalVector(ob1.block(1, 0, 9, 1));
        g2o::ellipsoid e2;
        e2.fromMinimalVector(ob2.block(1, 0, 9, 1));

        double yaw_deg = GetMinYawAngle(e1, e2);

        // ---- 测试新函数
        std::vector<int> iou_order;
        double IoU_aligned_arb = 1 - GetAlignedMIoUArbitrary(ob1, ob2, iou_order);
        double yaw_deg_arb = GetMinAngleArbitrary(e1, e2);

        COMPARE_RESULT output;
        output.instanceID = int(ob2[0]);

        output.dis_trans = origin_distance;
        output.dis_yaw = yaw_deg;
        output.IoU = IoU_ori;
        output.IoU_aligned = IoU_aligned;

        output.dis_yaw_arbitrary = yaw_deg_arb;
        output.IoU_aligned_arbitrary = IoU_aligned_arb;
        output.iou_order = iou_order;

        return output;
    }

    // **************** 处理物体对齐 ***********************

    // edit
    // bool deleteElement(std::vector<Vector4d, Eigen::aligned_allocator<Vector3d>> &vec, int id)
    bool deleteElement(std::vector<Vector4d, Eigen::aligned_allocator<Vector4d>> &vec, int id)
    {
        for (auto iter = vec.begin(); iter != vec.end(); iter++)
        {
            double id_d = (*iter)[0];
            if ((round(id_d) == id))
            {
                // 删除
                vec.erase(iter);
                return true;
            }
        }
        return false;
    }

    double compareTwoEllipsoid(const g2o::ellipsoid &e_est, const g2o::ellipsoid &e_ref)
    {
        // 先试试中心.
        Vector3d center_est = e_est.pose.translation();
        Vector3d center_ref = e_ref.pose.translation();

        return (center_est - center_ref).norm();
    }

    std::vector<int> AlignEllipsoidsUseID(MatrixXd &estObjMat, MatrixXd &refObjMat)
    {
        int est_obj_num = estObjMat.rows();
        int ref_obj_num = refObjMat.rows();

        std::vector<int> result_vec;
        result_vec.resize(est_obj_num);
        fill(result_vec.begin(), result_vec.end(), -1);
        for (int i = 0; i < est_obj_num; i++)
        {
            VectorXd estVec = estObjMat.row(i);
            int estid = round(estVec[0]);
            for(int n=0;n<ref_obj_num;n++)
            {
                VectorXd refVec = refObjMat.row(n);
                int refid = round(refVec[0]);
                if(estid==refid){
                    result_vec[i] = n;
                    continue;
                }
            }
        }
        return result_vec;  // 第i个关联到了第 value 的 ref
    }

    // 注意： 该版本使用少数的 ref 去关联 est, 更加合理.
    // 使用 Hungarian 算法求解全局最小误差关联
    std::vector<int> AlignEllipsoidsHungarian(MatrixXd &estObjMat, MatrixXd &refObjMat)
    {
        // std::cout << "estObjMat : " << std::endl
        //           << estObjMat << std::endl;
        // std::cout << "refObjMat : " << std::endl
        //           << refObjMat << std::endl;

        double PARAM_MIN_DIS = 1.0;  // 单位为m
        if(Config::Get<std::string>("Dataset.Type")=="TUM") PARAM_MIN_DIS = 0.5;
        std::cout << "- Align with param dis : " << PARAM_MIN_DIS << std::endl;

        int est_obj_num = estObjMat.rows();
        int ref_obj_num = refObjMat.rows();

        MatrixXd costMat(ref_obj_num, est_obj_num);
        for (int ref_id = 0; ref_id < ref_obj_num; ref_id++)
        {
            VectorXd vec_ref_whole = refObjMat.row(ref_id);
            VectorXd vec_ref = vec_ref_whole.block(1, 0, 9, 1);
            int label_ref = round(vec_ref_whole[10]);
            g2o::ellipsoid e_ref;
            e_ref.fromMinimalVector(vec_ref);
            for (int est_id = 0; est_id < est_obj_num; est_id++)
            {
                VectorXd vec_est_whole = estObjMat.row(est_id);
                VectorXd vec_est = vec_est_whole.block(1, 0, 9, 1);
                g2o::ellipsoid e_est;
                e_est.fromMinimalVector(vec_est);
                // if (ref_id == 0 && est_id == 0)
                // {
                //     std::cout << "[DEBUG] est_vec : " << vec_est.transpose() << std::endl;
                //     std::cout << "[DEBUG] ref_vec : " << vec_ref.transpose() << std::endl;

                //     std::cout << "[DEBUG] center in est" << e_est.translation().transpose() << std::endl;
                //     std::cout << "[DEBUG] center in ref" << e_ref.translation().transpose() << std::endl;
                //     std::cout << "[DEBUG] center dis" << (e_ref.translation() - e_est.translation()).norm() << std::endl;
                // }

                // ************ 定义数据关联时的Cost计算方式 ************ 
                // Cost 0 : Label 的比较
                int label_est = round(vec_est_whole[10]);
                bool bSameLabel = (label_est == label_ref);
                double cost_label = 0;
                if(!bSameLabel) cost_label = 999;

                // Cost 1 : 中心差距
                double cost_dis = e_est.minus(e_ref); // 椭球体相减函数
                if( cost_dis > PARAM_MIN_DIS ) cost_dis = 999; // 如果距离太大，则认为一定不能关联.

                // Cost 2 : 尺度、中心差距
                // discenter 3, yaw 1, disscale 3
                // Vector7d error = e_est.ellipsoid_error_3d(e_ref);
                // double error_dis = error.head(3).norm();
                // double error_yaw = error[3];
                // double error_scale = error.tail(3).cwiseAbs().sum();
                // Vector3d error_vec; error_vec << error_dis, error_yaw, error_scale;
                // Vector3d sigma_vec; sigma_vec << 0.5, M_PI/180.0*20.0, 0.2;
                // Vector3d inv_sigma = sigma_vec.cwiseInverse();
                // Matrix3d info = inv_sigma.cwiseProduct(inv_sigma).asDiagonal();
                // double cost = error_vec.transpose() * info * error_vec;
                // ************ END ************ 

                double cost = cost_label + cost_dis;
                bool bIgnoreLabel = Config::Get<int>("Evo.IgnoreLabel") == 1;
                if(bIgnoreLabel)
                    cost = cost_dis;

                costMat(ref_id, est_id) = cost;
            }
        }

        MatrixXd costMatCube;
        // if the dimensions are equal (square matrix), we're done
        // else, have to figure out larger dimension and pad with matrix max
        if (est_obj_num == ref_obj_num)
        {
            costMatCube = costMat;
        }
        else
        {
            float max_elem = costMat.maxCoeff();     // find the max element
            int dim = max(est_obj_num, ref_obj_num); // find the dimension for the new, square matrix
            costMatCube.resize(dim, dim);
            // fill the matrix with the elements from temp and pad with max element
            for (int i = 0; i < dim; i++)
            {
                for (int j = 0; j < dim; j++)
                {
                    if (i >= ref_obj_num || j >= est_obj_num)
                        costMatCube(i, j) = max_elem;
                    else
                        costMatCube(i, j) = costMat(i, j);
                }
            }
        }

        // std::cout << " costMat : \n"
        //           << costMat << std::endl;
        // std::cout << " rows,cols : \n"
        //           << costMat.rows() << ", " << costMat.cols() << std::endl;
        // std::cout << " costMatCube : \n"
        //           << costMatCube << std::endl;
        // std::cout << " rows,cols : \n"
        //           << costMatCube.rows() << ", " << costMatCube.cols() << std::endl;

        std::cout << "Begin Finding the result . " << std::endl;
        MatrixXd result(costMatCube.rows(), costMatCube.cols());
        result.fill(0);
        evo::findMatching(costMatCube, result, evo::MATCH_MIN);

        // std::cout << " Hungarian Result : \n"
        //           << result << std::endl;

        // 将 result 转换成vector
        // vector ,  第n个 estObj 关联的物体, 若没有则为 -1.
        std::vector<int> result_vec;
        result_vec.resize(est_obj_num);
        fill(result_vec.begin(), result_vec.end(), -1);
        for (int i = 0; i < est_obj_num; i++)
        {
            VectorXd row_vec = result.col(i);
            MatrixXd::Index maxRow, maxCol;
            double max = row_vec.maxCoeff(&maxRow, &maxCol);
            if (maxRow < ref_obj_num) // max =1
            {
                // 检查是否距离太远
                double cost = costMatCube(maxRow, i);
                
                // 注意该判断不能删除，最后总会选择一个很大的作为最优结果
                if (cost > PARAM_MIN_DIS)
                    result_vec[i] = -1; // 仍然认为无效
                else
                    result_vec[i] = maxRow;
                
                // result_vec[i] = maxRow;
            }
        }
        return result_vec;
    }

    // 本函数为1个接1个去匹配最小值，很可能得不到全局最优
    std::vector<int> AlignEllipsoids(MatrixXd &estObjMat, MatrixXd &refObjMat)
    {
        // 都生成x,y,z Vec
        MatrixXd estPointMat = estObjMat.block(0, 1, estObjMat.rows(), 3);
        MatrixXd refPointMat = refObjMat.block(0, 1, refObjMat.rows(), 3);

        // cout << "estPointMat : \n " << estPointMat << endl;
        // cout << "refPointMat : \n " << refPointMat << endl;

        // 将 refPoint 放到一个 vector中，以实现动态删除
        int ref_num = refPointMat.rows();
        // edit
        // std::vector<Vector4d, Eigen::aligned_allocator<Vector3d>> vRefPointsWithID;
        std::vector<Vector4d, Eigen::aligned_allocator<Vector4d>> vRefPointsWithID;
        for (int i = 0; i < ref_num; i++)
        {
            VectorXd ref_vec = refPointMat.row(i);
            Vector3d center_ref = ref_vec;
            Vector4d center_ref_withID;
            center_ref_withID << i, center_ref;

            vRefPointsWithID.push_back(center_ref_withID);
        }

        // Easy版本: 通过椭球体之间xyz距离做 alignment.
        int est_num = estPointMat.rows();
        std::vector<int> associations;
        associations.resize(est_num);
        fill(associations.begin(), associations.end(), -1);
        for (int i = 0; i < est_num; i++)
        {
            VectorXd est_vec = estObjMat.row(i);
            Vector3d center_est = est_vec.block(1, 0, 3, 1);

            // 寻找距离最小的拿走.
            int ref_num_left = vRefPointsWithID.size();

            if (ref_num_left == 0)
                break; // 已经没有剩余的ref objects了, 结束.

            std::vector<std::pair<int, double>> vIdDis;
            for (int m = 0; m < ref_num_left; m++)
            {
                double dis = (center_est - vRefPointsWithID[m].tail(3)).norm();
                int id = round(vRefPointsWithID[m][0]);
                vIdDis.push_back(std::make_pair(id, dis));
            }
            std::sort(vIdDis.begin(), vIdDis.end(), compare_pair_int_double);

            double min_dis = vIdDis[0].second;
            if (min_dis < 1.0) // 要求 1m 以内
            {
                int id = vIdDis[0].first;
                associations[i] = vIdDis[0].first;
                // 动态删除
                bool result = deleteElement(vRefPointsWithID, id);
                if (!result)
                {
                    std::cout << " Possible BUG: delte element fails." << std::endl;
                }
            }
        }

        return associations;
    }

    // objMat : id x y z r p y a b c [label]
    // 该函数变换xyzrpy, 与其他维度无关.
    MatrixXd transformObjMat(MatrixXd &objMat, g2o::SE3Quat &T)
    {
        int num = objMat.rows();
        MatrixXd objMatTrans;
        objMatTrans.resize(0, objMat.cols());
        for (int i = 0; i < num; i++)
        {
            VectorXd vec = objMat.row(i);
            g2o::ellipsoid e;
            e.fromMinimalVector(vec.head(10).tail(9));
            g2o::ellipsoid e_trans = e.transform_from(T);
            Vector9d e_miniVec = e_trans.toMinimalVector();

            VectorXd vec_trans = vec;
            vec_trans.head(10).tail(9) = e_miniVec;     // TOBE CHECK: 该类替换是否生效. 是否是该部分的引用.

            // 变换到世界系
            addVecToMatirx(objMatTrans, vec_trans);
        }
        return objMatTrans;
    }

    // ***************************************************

    Trajectory MatToTraj(MatrixXd &mat, g2o::SE3Quat Trans = g2o::SE3Quat())
    {
        Trajectory tGt;
        int num = mat.rows(); // 这里只可视化所有对应帧得了.
        for (int i = 0; i < num; i++)
        {
            VectorXd gtPose = mat.row(i);
            SE3QuatWithStamp *pGtSE3T = new SE3QuatWithStamp();
            pGtSE3T->pose.fromVector(gtPose.tail(7));
            pGtSE3T->timestamp = gtPose[0];

            // 应用变换
            pGtSE3T->pose = Trans * pGtSE3T->pose;

            tGt.push_back(pGtSE3T);
        }
        return tGt;
    }

    void VisualizeEllipsoidsInMat(MatrixXd &mat, const Vector3d &color, ORB_SLAM2::Map *pMap, bool show_instance, bool show_in_observation)
    {
        int num = mat.rows();
        for (int i = 0; i < num; i++)
        {
            VectorXd mat_vec = mat.row(i);
            VectorXd obj_vec = mat_vec.block(1, 0, 9, 1);
            g2o::ellipsoid *pE = new g2o::ellipsoid;
            pE->fromMinimalVector(obj_vec);
            pE->setColor(color);
            pE->prob = 1.0;
            if(mat_vec.size()>10)
                pE->miLabel = round(mat_vec[10]);
            if (show_instance)
                pE->miInstanceID = round(mat_vec[0]);

            // 定义颜色等信息
            if (show_in_observation)
            {
                pE->miInstanceID = round(mat_vec[0]);
                pMap->addEllipsoidObservation(pE);
            }
            else
                pMap->addEllipsoidVisual(pE);
        }

        return;
    }

    void DrawAlignLines(MatrixXd &estObjMat, MatrixXd &refObjMat, std::vector<int> &vAlignEstToRef, ORB_SLAM2::Map *pMap) // 绘制关联线.
    {
        // std::cout << "ref obj mat: " << endl << refObjMat << endl;
        for (int i = 0; i < estObjMat.rows(); i++)
        {
            VectorXd vec = estObjMat.row(i);
            int ref_id = vAlignEstToRef[i];
            Vector3d center_est = vec.block(1, 0, 3, 1);
            if (ref_id != -1)
            {
                VectorXd ref_vec = refObjMat.row(ref_id);
                // cout << "ref_vec : " << ref_vec.transpose() << endl;
                Vector3d center_ref = ref_vec.block(1, 0, 3, 1);
                // cout << "center_ref : " << center_ref << endl;
                // 绘制它们二者中心的连线.
                Vector3d color = Vector3d(0, 0, 0); // black
                Vector3d norm = center_ref - center_est;
                pMap->addArrow(center_est, norm, color);
            }
            else
            {
                // 没有观测的.!
                // pMap->addArrow(center_est, Vector3d(0,0,1), Vector3d(0,0,0.8));
            }
        }
    }

    // 都在同一个坐标系下 : ref 坐标系
    void VisualizeEllipsoidsWithAssociations(MatrixXd &estObjMat, MatrixXd &refObjMat, std::vector<int> &vAlignEstToRef, ORB_SLAM2::Map *pMap)
    {
        if (pMap == NULL)
            return;
        pMap->ClearEllipsoidsVisual();
        pMap->ClearEllipsoidsObservation();
        pMap->clearArrows();
        // VisualizeEllipsoidsInMat(estObjMat, Vector3d(0, 0, 1.0), pMap, true, vAlignEstToRef);    // 仅仅关联上的才可视化
        VisualizeEllipsoidsInMat(estObjMat, Vector3d(0, 1.0, 0), pMap, true, true);    // 仅仅关联上的才可视化
        VisualizeEllipsoidsInMat(refObjMat, Vector3d(1.0, 0, 0), pMap, true);
        DrawAlignLines(estObjMat, refObjMat, vAlignEstToRef, pMap);
    }

    int findRowIDWithInstance(int ins_input, std::vector<int> &vAlignEstToRef, MatrixXd &refObjectMat)
    {
        // 先检查谁关联到了 ins1
        for (int i = 0; i < vAlignEstToRef.size(); i++)
        {
            int ref_id = vAlignEstToRef[i];
            if (ref_id == -1)
                continue;
            int ins = round(refObjectMat.row(ref_id)[0]);
            if (ins == ins_input)
                return i;
        }
        return -1;
    }

    // 查找物体mat中，ins为 ref_ins 所在行数
    int findObjectRowIDWithInstance(int ref_ins, MatrixXd &refObjectMat)
    {
        int num = refObjectMat.rows();
        for (int i = 0; i < num; i++)
        {
            if (round(refObjectMat.row(i)[0]) == ref_ins)
                return i;
        }
        return -1;
    }

    // 一个人工用户界面.
    std::vector<int> RunCommand(std::vector<int> &vAlignEstToRef, MatrixXd &refObjectMat, MatrixXd &estObjMatInRef, char a, int ins1, int ins2)
    {
        std::vector<int> outputAlign = vAlignEstToRef;
        // exchange
        if (a == 'c')
        {
            // vAlignEstToRef  存储了第 n 行
            int id1 = findRowIDWithInstance(ins1, outputAlign, refObjectMat);
            int id2 = findRowIDWithInstance(ins2, outputAlign, refObjectMat);
            std::cout << " id1 : " << id1 << ", id2 : " << id2 << std::endl;
            if (id1 > 0 && id2 > 0)
            {
                // 执行修改
                // outputAlign[]
                // 找到哪个 est_id -> ref_id
                // auto iter_id1 = find(outputAlign.begin(), outputAlign.end(), id1);
                // auto iter_id2 = find(outputAlign.begin(), outputAlign.end(), id2);

                // std::cout << "Change : " << *iter_id1 << " -> " << id2 << std::endl;
                // std::cout << "Change : " << *iter_id2 << " -> " << id1 << std::endl;
                // *iter_id1 = id2;
                // *iter_id2 = id1;

                int id2_align = outputAlign[id2];
                outputAlign[id2] = outputAlign[id1];
                outputAlign[id1] = id2_align;
            }
        }
        else if (a == 's')
        {
            // 将某 est_instance 设置为 ref_instance.

            // 先找到 EstIns 的 id. (即 vAlign 的位置)
            int est_ins = ins1;
            int est_pos = findObjectRowIDWithInstance(est_ins, estObjMatInRef);

            // 再找到 ref_ins 的 id, 即值的结果.
            int ref_ins = ins2;
            int ref_pos = findObjectRowIDWithInstance(ref_ins, refObjectMat);

            // 找到原来指向该物体的 est id，将其 align 位置设置为 -1.
            int est_pos_who_refer_to_ref_pos = findRowIDWithInstance(ins2, outputAlign, refObjectMat);

            outputAlign[est_pos] = ref_pos;
            outputAlign[est_pos_who_refer_to_ref_pos] = -1;

            std::cout << "Change : " << std::endl
                      << est_pos << " -> " << ref_pos << std::endl;
            std::cout << est_pos_who_refer_to_ref_pos << " -> -1" << std::endl;
        }
        else
            std::cout << "Wrong command type : " << a << std::endl;
        return outputAlign;
    }

    void GetCommand(char &a, int &ins1, int &ins2)
    {
        std::cout << "Please input: command(c,q) ins1 ins2" << std::endl;
        string str;
        getline(cin, str); //不阻塞

        vector<string> s;
        // edit
        // boost::split(s, str, boost::is_any_of(" \t,"), boost::token_compress_on);
        boost::algorithm::split(s, str, boost::is_any_of(" \t,"), boost::token_compress_on);

        if (s.size() == 3){
            a = s[0][0];
            ins1 = stoi(s[1]);
            ins2 = stoi(s[2]);
        }
        else if (s.size() > 0)
            a = s[0][0];
        return;
    }

    void OutputResult(ostream &fio, StaticResult &staticResult, std::map<int, COMPARE_RESULT> &results, MatrixXd &refObjMat)
    {
        fio << " ----------- AVERAGE ---------" << endl;
        fio << "aver_IoU: " << staticResult.aver_IoU << endl;
        fio << "aver_IoU_aligned: " << staticResult.aver_IoU_aligned << endl;
        fio << "aver_dis_trans: " << staticResult.aver_dis_trans << endl;
        fio << "aver_dis_yaw: " << staticResult.aver_dis_yaw << endl;
        fio << "aver_dis_yaw_arb: " << staticResult.aver_dis_yaw_arb << endl;
        fio << "aver_dis_yaw_arb_valid: " << staticResult.aver_dis_yaw_arb_valid << endl;
        fio << "aver_IoU_aligned_arb: " << staticResult.aver_IoU_aligned_arb << endl;

        fio << " ----------- Mid Value ---------" << endl;
        fio << "mid_IoU: " << staticResult.mid_iou << endl;
        fio << "mid_IoU_aligned: " << staticResult.mid_iou_aligned << endl;
        fio << "mid_dis_trans: " << staticResult.mid_dis_trans << endl;
        fio << "mid_dis_yaw: " << staticResult.mid_dis_yaw << endl;
        fio << "mid_dis_yaw_arb: " << staticResult.mid_dis_yaw_arb << endl;
        fio << "mid_dis_yaw_arb_valid: " << staticResult.mid_dis_yaw_arb_valid << endl;
        fio << "mid_IoU_aligned_arb: " << staticResult.mid_iou_aligned_arb << endl;

        fio << " ----------- SINGLE OBJECTS COMPARE ---------" << endl;
        fio << "instanceID\t"
            << "IoU\t"
            << "IoU_aligned\t"
            << "dis_trans\t"
            << "dis_yaw\t"
            << "dis_yaw_arb\t"
            << "IoU_aligned_arb\t" << endl;

        int refNum = refObjMat.rows();
        for (int n = 0; n < refNum; n++)
        {
            int ins_id_ref = round(refObjMat.row(n)[0]);
            auto iter = results.find(ins_id_ref);
            if (iter != results.end())
            {
                fio << iter->first << "\t" << iter->second.IoU << "\t" << iter->second.IoU_aligned << "\t" << iter->second.dis_trans << "\t" << iter->second.dis_yaw
                    << "\t" << iter->second.dis_yaw_arbitrary << "\t" << iter->second.IoU_aligned_arbitrary << "\t" << iter->second.est_id_origin << "(est)" << endl;
            }
            else
            {
                fio << ins_id_ref << "\t"
                    << "Empty." << endl;
            }
        }

        // 输出未找到关联对象的观测结果.
        std::set<int> list_noAlign = staticResult.list_noAlgin;
        fio << "List_noAlign( est_id ) : " << endl;
        for (auto iter = list_noAlign.begin(); iter != list_noAlign.end(); iter++)
            fio << *iter << " ";
        fio << endl;

        // 计算odom的RMSE做比较
        fio << " [ RMSE : " << staticResult.rmse << " ] " << endl;

        // 评估 Association
        fio << "- precision : " << staticResult.precision << std::endl;
        fio << "- recall : " << staticResult.recall << std::endl;
        fio << "- F1 : " << staticResult.F1 << std::endl;

        // 快速输出
        fio << "--- Simple version : " << std::endl;
        fio << "- rmse : " << staticResult.rmse << std::endl;
        fio << "- Tr/Ro/Sh : " << staticResult.aver_dis_trans << "/" << staticResult.aver_dis_yaw_arb << "/" << staticResult.aver_IoU_aligned_arb << std::endl;
        fio << "- P/R/F : " << staticResult.precision << "/" << staticResult.recall << "/" << staticResult.F1 << std::endl;

        // 超 Simple
        fio << " ======= All Rotations =======" << std::endl;
        // fio << staticResult.rmse << std::endl;
        fio << staticResult.aver_dis_trans << "/" << staticResult.aver_dis_yaw_arb << "/" << staticResult.aver_IoU_aligned_arb << std::endl;
        fio << staticResult.precision << "/" << staticResult.recall << "/" << staticResult.F1 << std::endl;

        fio << " ======= Valid Rotations =======" << std::endl;
        fio << staticResult.aver_dis_trans << "/" << staticResult.aver_dis_yaw_arb_valid << "/" << staticResult.aver_IoU_aligned_arb << std::endl;
        fio << staticResult.precision << "/" << staticResult.recall << "/" << staticResult.F1 << std::endl;

    }

    StaticResult CalculateResult(std::map<int, COMPARE_RESULT> &results)
    {
        // 计算整体值.    // TODO 写整体计算，然后调试.
        double aver_IoU = 0;         // 原始IoU
        double aver_IoU_aligned = 0; // 对齐中心后的IoU
        double aver_dis_trans = 0;
        double aver_dis_yaw = 0;
        double aver_dis_yaw_arb = 0;
        double aver_IoU_aligned_arb = 0;
        double aver_dis_yaw_arb_valid = 0;


        // 计算中位数
        std::vector<double> vector_iou;
        std::vector<double> vector_iou_aligned;
        std::vector<double> vector_dis_trans;
        std::vector<double> vector_dis_yaw;
        std::vector<double> vector_dis_yaw_arb;
        std::vector<double> vector_iou_aligned_arb;
        std::vector<double> vector_dis_yaw_arb_valid;

        int count_valid_angle = 0;
        for (auto iter : results)
        {
            aver_IoU += iter.second.IoU;
            aver_IoU_aligned += iter.second.IoU_aligned;
            aver_dis_trans += iter.second.dis_trans;
            aver_IoU_aligned_arb += iter.second.IoU_aligned_arbitrary;

            vector_iou.push_back(iter.second.IoU);
            vector_iou_aligned.push_back(iter.second.IoU_aligned);
            vector_dis_trans.push_back(iter.second.dis_trans);
            vector_iou_aligned_arb.push_back(iter.second.IoU_aligned_arbitrary);

            aver_dis_yaw += iter.second.dis_yaw;
            aver_dis_yaw_arb += iter.second.dis_yaw_arbitrary;
            vector_dis_yaw.push_back(iter.second.dis_yaw);
            vector_dis_yaw_arb.push_back(iter.second.dis_yaw_arbitrary);

            if(iter.second.valid_angle){
                aver_dis_yaw_arb_valid += iter.second.dis_yaw_arbitrary;
                vector_dis_yaw_arb_valid.push_back(iter.second.dis_yaw_arbitrary);
                count_valid_angle++;
            }
        }

        StaticResult staticResult;
        int total_valid_num = results.size();
        if(total_valid_num == 0) 
        {
            std::cout << "EVO : No Valid associated Estimation Objects!!!" << std::endl;
            return staticResult;
        }
        aver_IoU /= total_valid_num;
        aver_IoU_aligned /= total_valid_num;
        aver_dis_trans /= total_valid_num;
        aver_dis_yaw /= total_valid_num;
        aver_dis_yaw_arb /= total_valid_num;
        aver_IoU_aligned_arb /= total_valid_num;
        if(count_valid_angle > 0)
            aver_dis_yaw_arb_valid /= count_valid_angle;
        else 
            aver_dis_yaw_arb_valid = -1;
        staticResult.aver_IoU = aver_IoU;
        staticResult.aver_IoU_aligned = aver_IoU_aligned;
        staticResult.aver_dis_trans = aver_dis_trans;
        staticResult.aver_dis_yaw = aver_dis_yaw;
        staticResult.aver_dis_yaw_arb = aver_dis_yaw_arb;
        staticResult.aver_IoU_aligned_arb = aver_IoU_aligned_arb;
        staticResult.aver_dis_yaw_arb_valid = aver_dis_yaw_arb_valid;

        sort(vector_iou.begin(), vector_iou.end());
        sort(vector_iou_aligned.begin(), vector_iou_aligned.end());
        sort(vector_dis_trans.begin(), vector_dis_trans.end());
        sort(vector_dis_yaw.begin(), vector_dis_yaw.end());
        sort(vector_dis_yaw_arb.begin(), vector_dis_yaw_arb.end());
        sort(vector_iou_aligned_arb.begin(), vector_iou_aligned_arb.end());
        sort(vector_dis_yaw_arb_valid.begin(), vector_dis_yaw_arb_valid.end());
        staticResult.mid_iou = vector_iou[total_valid_num / 2];
        staticResult.mid_iou_aligned = vector_iou_aligned[total_valid_num / 2];
        staticResult.mid_dis_trans = vector_dis_trans[total_valid_num / 2];
        staticResult.mid_dis_yaw = vector_dis_yaw[total_valid_num / 2];
        staticResult.mid_dis_yaw_arb = vector_dis_yaw_arb[total_valid_num / 2];
        staticResult.mid_iou_aligned_arb = vector_iou_aligned_arb[total_valid_num / 2];
        if(count_valid_angle > 0)
            staticResult.mid_dis_yaw_arb_valid = vector_dis_yaw_arb_valid[count_valid_angle / 2];
        else 
            staticResult.mid_dis_yaw_arb_valid = -1;

        return staticResult;
    }

    const char* GetLabelText_Optimizer(int id)
    {
        static const char *coco_classes[] = {"person","bicycle","car","motorcycle","airplane","bus","train",
        "truck","boat","traffic light","fire hydrant","stop sign","parking meter","bench","bird",
        "cat","dog","horse","sheep","cow","elephant","bear","zebra","giraffe","backpack","umbrella",
        "handbag","tie","suitcase","frisbee","skis","snowboard","sports ball","kite","baseball bat",
        "baseball glove","skateboard","surfboard","tennis racket","bottle","wine glass","cup","fork",
        "knife","spoon","bowl","banana","apple","sandwich","orange","broccoli","carrot","hot dog",
        "pizza","donut","cake","chair","couch","potted plant","bed","dining table","toilet","monitor",
        "laptop","mouse","remote","keyboard","cell phone","microwave","oven","toaster","sink",
        "refrigerator","book","clock","vase","scissors","teddy bear","hair drier","toothbrush"};
        if(id >= 0)
            return coco_classes[id];
        else 
            return "Unknown";
    }

    // 根据语义标签，若该物体是对称的，则赋予对称性质. 
    // 目前该性质仅仅用于：
    // 1) 判断旋转约束是否生效 
    // 2) evo中是否评估其与gt的旋转error.
    bool IsSymmetry(int label)
    {
        static const std::set<std::string> symLabels = 
        {
            "vase", "bowl", "cup", "potted plant", "bottle"
        };
        const char* txtLabel = GetLabelText_Optimizer(label);
        if( symLabels.find(std::string(txtLabel)) != symLabels.end() )
            return true;
        else 
            return false;
    }

    // 添加变量 use_id_to_associate: 若为真，则不调用自动关联，而以首位id直接关联
    bool Evaluate(MatrixXd &refObjMat, MatrixXd &estObjMat, MatrixXd &gtTrajMat, MatrixXd &estTrajMat, 
                StaticResult &output_staticResult, std::map<int, COMPARE_RESULT>& output_results,
                  ORB_SLAM2::Map *pMap, bool manual_check, bool filter_sym, bool use_id_to_associate)
    {
        int refNum = refObjMat.rows();
        int estNum_ori = estObjMat.rows();
        int estNum = estNum_ori;

        if(refNum < 1 || estNum < 1)
        {
            std::cout << "Error in RefMat or EstMat, Please check your file exsist." << std::endl;
            return false;
        }

        Trajectory gtTrajInEst, estTrajSelected;
        g2o::SE3Quat Tre;
        alignTrajectory(estTrajMat, gtTrajMat, Tre, gtTrajInEst, estTrajSelected);

        // std::cout << "Umeda Align Result [x y z roll pitch yaw] : " << std::endl;
        std::cout << Tre.toMinimalVector().transpose() << std::endl;

        // TODO: 可视化轨迹对齐情况
        Trajectory gtTraj = MatToTraj(gtTrajMat);
        Trajectory estTrajInGt = MatToTraj(estTrajMat, Tre);

        bool bVisualize = (pMap != NULL);
        if (bVisualize)
        {
            pMap->addOneTrajectory(estTrajInGt, "OptimizedTrajectory");
            pMap->addOneTrajectory(gtTraj, "AlignedGroundtruth");
        }

        MatrixXd estObjMatInRef = transformObjMat(estObjMat, Tre);
        // std::vector<int> vAlignEstToRef = AlignEllipsoids(estObjMatInRef, refObjMat);

        // testing...
        std::vector<int> vAlignEstToRef;
        if(use_id_to_associate)
            vAlignEstToRef = AlignEllipsoidsUseID(estObjMatInRef, refObjMat);
        else 
            vAlignEstToRef = AlignEllipsoidsHungarian(estObjMatInRef, refObjMat);

        // 在此可视化所有椭球体及它们的关系. 并使用互动界面做检查
        if (manual_check)
        {
            bool bStart = false;
            bool bFirst = true;
            while (!bStart)
            {
                if (!bFirst)
                {
                    char a;
                    int ins1, ins2; // change ins1 and ins2.
                    GetCommand(a, ins1, ins2);
                    if (a == 'q')
                        break;
                    else
                    {
                        std::cout << "Command : " << a << ", " << ins1 << ", " << ins2 << std::endl;
                        vAlignEstToRef = RunCommand(vAlignEstToRef, refObjMat, estObjMatInRef, a, ins1, ins2);
                    }
                }
                std::cout << "*************" << std::endl;
                for (int i = 0; i < vAlignEstToRef.size(); i++)
                    std::cout << i << " -> " << vAlignEstToRef[i] << std::endl;
                std::cout << "*************" << std::endl;

                VisualizeEllipsoidsWithAssociations(estObjMatInRef, refObjMat, vAlignEstToRef, pMap);
                bFirst = false;
            }
        }
        else 
            VisualizeEllipsoidsWithAssociations(estObjMatInRef, refObjMat, vAlignEstToRef, pMap);

        std::map<int, COMPARE_RESULT> results;

        std::set<int> list_noAlign;
        for (int i = 0; i < estNum_ori; i++)
        {
            // 计算单个物体差别, 再输出整体平均差别.
            VectorXd estOb = estObjMatInRef.row(i);
            int ref_id = vAlignEstToRef[i];

            int ins_id_est = round(estOb[0]);
            if (ref_id == -1)
            {
                // no align
                list_noAlign.insert(ins_id_est);
                estNum--;
                continue;
            }

            // VectorXd refOb = refObjMat.row(ref_id).head(10); // 去掉最后一个没用的id
            VectorXd refOb = refObjMat.row(ref_id); // 去掉最后一个没用的id

            // 开始比较各项指标。
            COMPARE_RESULT result = compareObjectWithVector(refOb, estOb);

            result.instanceID = round(refOb[0]); // 此处ins记录的是reference id 的值
            result.est_id = i;
            result.ref_id = ref_id;
            result.est_id_origin = ins_id_est;

            // 评估是否是对称物体
            if(estOb.size()>=11){
                int label = round(estOb[10]);
                bool is_sym = IsSymmetry(label);
                result.valid_angle = !is_sym; // 不对称的才评估
            }
            else result.valid_angle = true; // 默认是评估的

            results.insert(make_pair(result.instanceID, result));
        }

        // 补充可视化
        bool bSupVisualization = false;
        if (bSupVisualization)
        {
            // 1) 可视化aligned_iou_arb的最佳顺序
            int result_num = results.size();
            int result_id = 0;
            for (auto iter = results.begin(); iter != results.end(); iter++)
            {
                std::cout << "Visualize Result ID : " << result_id << "..." << std::endl;

                // 在原点可视化一下?
                auto order = iter->second.iou_order;
                int ref_id = iter->second.ref_id;
                int est_id = iter->second.est_id;

                VectorXd refOb = refObjMat.row(ref_id).head(10);
                Vector9d vec1;
                vec1 << 0, 0, 0, 0, 0, 0, refOb.tail(3);
                g2o::ellipsoid e1;
                e1.fromMinimalVector(vec1);
                e1.setColor(Vector3d(1.0, 0, 0));

                VectorXd estOb = estObjMatInRef.row(est_id).head(10);
                Vector9d vec2;
                int first = order[0] + 7;
                int second = order[1] + 7;
                int third = order[2] + 7;
                vec2 << 0, 0, 0, 0, 0, 0, estOb[first], estOb[second], estOb[third];
                g2o::ellipsoid e2;
                e2.fromMinimalVector(vec2);
                e1.setColor(Vector3d(0, 0, 1.0));

                // 先清理槽, 然后可视化 e1, e2
                pMap->ClearEllipsoidsObservation();
                pMap->addEllipsoidObservation(&e1);
                pMap->addEllipsoidObservation(&e2);

                getchar();
                result_id++;
            }
        }

        // -------------------- 综合评估过程 -----------------------
        StaticResult staticResult = CalculateResult(results);
        staticResult.list_noAlgin = list_noAlign;

        // 计算RMSE
        // Trajectory estTraj = MatToTrajectory(estTrajMat);
        double rmse = CalculateRMSE(estTrajSelected, gtTrajInEst);
        staticResult.rmse = rmse;

        // 评估 Association
        int num_correct_estimation = estNum;
        int num_total_estimation = estNum_ori;
        int num_real_objects = refNum; // 注意理应根据数据集的变化做修改
        double precision = double(estNum) / estNum_ori;
        double recall = double(estNum) / num_real_objects;
        double F1 = 2 * precision * recall / (precision + recall);
        staticResult.precision = precision;
        staticResult.recall = recall;
        staticResult.F1 = F1;

        output_staticResult = staticResult;
        output_results = results;
        return true;
    }

// std::map<int, std::map<double, int>> VisualizeAndRevisedAssociations(const std::map<int, std::map<double, int>>& objObs, 
//         const std::vector<Frame*> &vpFrames, MatrixXd& refObjMat, ORB_SLAM2::Map* pMap)
// {
//     int obj_num = objObs.size();

//     for(int obj_id=0;obj_id<obj_num;obj_id++){
//         pMap->ClearEllipsoidsVisual();
//         auto ins_to_obs = 
//         pMap->

//     // 首先可视化该物体
    


//     // 然后挨个可视化该物体关联的观测
//     //      提供指令： 1. 切换成另一个物体 2. 保存并继续

//     }

//  // ---

//     bool bStart = false;
//     bool bFirst = true;
//     while (!bStart)
//     {
//         if (!bFirst)
//         {
//             char a;
//             int ins1, ins2; // change ins1 and ins2.
//             GetCommand(a, ins1, ins2);
//             if (a == 'q')
//                 break;
//             else
//             {
//                 std::cout << "Command : " << a << ", " << ins1 << ", " << ins2 << std::endl;
//                 vAlignEstToRef = RunCommand(vAlignEstToRef, refObjMat, estObjMatInRef, a, ins1, ins2);
//             }
//         }
//         std::cout << "*************" << std::endl;
//         for (int i = 0; i < vAlignEstToRef.size(); i++)
//             std::cout << i << " -> " << vAlignEstToRef[i] << std::endl;
//         std::cout << "*************" << std::endl;

//         VisualizeEllipsoidsWithAssociations(estObjMatInRef, refObjMat, vAlignEstToRef, pMap);
//         bFirst = false;
//     }
// }

} // namespace evo