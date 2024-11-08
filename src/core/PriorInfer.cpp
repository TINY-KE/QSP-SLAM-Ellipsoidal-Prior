// 该文件基于先验表对椭球体做优化

#include "Thirdparty/g2o/g2o/core/block_solver.h"
#include "Thirdparty/g2o/g2o/core/optimization_algorithm_levenberg.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_eigen.h"
#include "Thirdparty/g2o/g2o/types/types_six_dof_expmap.h"
#include "Thirdparty/g2o/g2o/core/robust_kernel_impl.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_dense.h"
#include "Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"

#include "src/pca/EllipsoidExtractorEdges.h"

#include "src/config/Config.h"
#include "include/core/Plane.h"
#include "include/core/ConstrainPlane.h"
#include "include/core/PriorInfer.h"

#include "include/utils/dataprocess_utils.h"

#include <Eigen/Core>
#include "src/pca/EllipsoidExtractor.h"

using namespace g2o;
using namespace Eigen;

namespace ORB_SLAM2
{

    Pri::Pri(double a_, double b_, double c_)
    {
        double d0 = b_ / a_;
        double d1 = c_ / a_;
        
        if (a_ > b_)
        {
            d = b_;
            e = a_;
        }
        else
        {
            d = a_;
            e = b_;
        }
    }

    Pri::Pri(double a_, double b_)
    {
        if (a_ > b_)
        {
            d = b_;
            e = a_;
        }
        else
        {
            d = a_;
            e = b_;
        }
    }

    Pri::Pri(const g2o::ellipsoid &e) // 直接从椭球体读取 pri
    {
        // std::cout << "Before : " << e.scale.transpose() << std::endl;
        // print();
        Vector3d scale_abs = e.scale.cwiseAbs();
        std::vector<double> scale_vec;
        scale_vec.push_back(scale_abs[0]);
        scale_vec.push_back(scale_abs[1]);
        scale_vec.push_back(scale_abs[2]);
        sort(scale_vec.begin(), scale_vec.end()); // 从小到大

        double min = scale_vec[0];
        double mid = scale_vec[1];
        double max = scale_vec[2];
        
        this->d = mid/min;
        this->e = max/min;
        // std::cout << "After: " << std::endl;
        // print();
    }

    void Pri::print()
    {
        std::cout << "Pri : " << 1 << " : " << d << " : " << e << std::endl;
    }

    double Pri::GetD() const
    {
        return this->d;
    }

    double Pri::GetE() const
    {
        return this->e;
    }

    priorInfer::priorInfer(int rows, int cols, const Matrix3d& calib)
    {
        mRows = rows;
        mCols = cols;
        mCalib = calib;
        std::cout << "Load rows/cols: " << rows << "/" << cols << ", calib:" << mCalib << std::endl;

        mdCost = 0;
    }

    double priorInfer::GetLastCost()
    {
        return mdCost;
    }

    g2o::ellipsoid priorInfer::GenerateInitGuess(Vector4d& bbox, const Vector4d& plane_local)
    {
        double dis_sigma = Config::ReadValue<double>("MonocularInfer.Init.Dis"); // 0.5m
        double size_sigma = 0.1;

        ellipsoid e_init_guess; // 获得估计出椭球体的 rpy; 只是将 x,y,z,a,b,c 都设置为0.
        // e_init_guess 中要包含平面
        auto cplanes = GenerateConstrainPlanesOfBbox(bbox, mCalib, mRows, mCols);
        e_init_guess.addConstrainPlanes(cplanes);

        Vector2d center_bbox = (bbox.head(2) + bbox.tail(2)) / 2.0;

        // 这里需要考虑fx,fy的正负性问题，当为负则添加一个负号
        double cx = mCalib(0,2);
        double cy = mCalib(1,2);
        double fx = mCalib(0,0);
        double fy = mCalib(1,1);
        
        Vector3d center_bbox_3d(center_bbox(0)-cx, center_bbox(1)-cy, std::abs(fx));
        if(fx<0) center_bbox_3d[0] = -center_bbox_3d[0];
        if(fy<0) center_bbox_3d[1] = -center_bbox_3d[1];
        center_bbox_3d.normalize();
        Vector3d norm_camera_to_obj = center_bbox_3d;  // Z 轴正方向
        e_init_guess.pose.setTranslation(dis_sigma*norm_camera_to_obj);  // 沿着相机到其中心方向平移一个距离 sigma

        // 生成一个朝向相机、且与地平面方向垂直的旋转矩阵
        Vector3d zaxis = plane_local.head(3); zaxis.normalize();
        Vector3d xaxis = norm_camera_to_obj- norm_camera_to_obj.dot(zaxis) * zaxis; // 相机z轴在地平面上的投影方向 即法方向上的垂直分量
        xaxis.normalize();
        Vector3d yaxis = zaxis.cross(xaxis); 
        if(yaxis.norm()>0)
            yaxis.normalize();
        Matrix3d rotMat; rotMat.col(0) = xaxis; rotMat.col(1) = yaxis; rotMat.col(2) = zaxis; 
        Eigen::Quaterniond rotMatQuat = Eigen::Quaterniond(rotMat);
        e_init_guess.pose.setRotation(rotMatQuat);
        e_init_guess.scale = Vector3d(size_sigma,size_sigma,size_sigma);
        e_init_guess.vec_minimal = e_init_guess.toMinimalVector();

        // 直接输出rotMat
        // std::cout << "---CHECK--- RotMat: " << std::endl << rotMat << std::endl;
        // std::cout << "---CHECK--- RotMatFromEllipsoid: " << std::endl << e_init_guess.pose.rotation().toRotationMatrix() << std::endl;
        // std::cout << "---CHECK--- RotMatQuat: " << rotMatQuat.coeffs().transpose() << std::endl;
        // std::cout << "---CHECK--- RotMatFromEllipsoidQuat: " << e_init_guess.pose.rotation().coeffs().transpose() << std::endl;

        e_init_guess.bbox = bbox;
        return e_init_guess;
    }

    g2o::ellipsoid priorInfer::infer(g2o::ellipsoid &e, const Pri &pri, double weight, g2o::plane& plane_ground)
    {
        std::vector<g2o::plane> planes;
        std::vector<g2o::plane> planesWithNormal;

        // 构建约束平面!
        // 0) 地平面约束
        planesWithNormal.push_back(plane_ground);

        // 1) 椭球体的六个平面都作为约束平面
        // std::vector<g2o::plane*> vecPlanes = e.GetCubePlanesInImages(g2o::SE3Quat(), mCalib, mRows, mCols, 30);
        std::vector<g2o::plane*> vecPlanes = e.GetCubePlanes();

        // vec planes to planes
        double angle_thresh = M_PI / 180 * 45; // 10 deg
        std::vector<g2o::plane*> vecPlanes_potential_invalid;  // 存储夹角太小的平面。按道理是一组平面
        for(int i=0;i<vecPlanes.size();i++)
        {
            // 检查平面有效性。即并非被遮挡的平面。
            // 1) 选出法向量与z轴夹角接近为0的一组
            // 2） 仅仅保留较近的一个
            g2o::plane* ppl = vecPlanes[i];
            Vector3d normVec = ppl->param.head(3);
            Vector3d cam_to_objcenter = e.translation();
            double cos_angle_to_Z = normVec.dot(cam_to_objcenter) / normVec.norm() / cam_to_objcenter.norm();
            double angle_to_Z = acos(cos_angle_to_Z);   // TO BE CHECK RANGE
            std::cout << i << " - angle : " << angle_to_Z << std::endl;
            if(std::abs(angle_to_Z) < angle_thresh || std::abs(M_PI-angle_to_Z) < angle_thresh)
            {
                vecPlanes_potential_invalid.push_back(ppl);
            }
            else 
                planesWithNormal.push_back(*vecPlanes[i]);
        }
        // TODO : 去掉不可见平面
        std::cout << "obj scale : " << e.scale.transpose() << std::endl;
        if(vecPlanes_potential_invalid.size()==2)
        {
            double dis1 = vecPlanes_potential_invalid[0]->distanceToPoint(Vector3d(0,0,0));
            double dis2 = vecPlanes_potential_invalid[1]->distanceToPoint(Vector3d(0,0,0));

            std::cout << "Erase an invalid plane. Dis : " << dis1 << ", " << dis2 << std::endl;

            if(dis1<dis2)   // 将较近的放入有效平面
                planesWithNormal.push_back(*vecPlanes_potential_invalid[0]);
            else
                planesWithNormal.push_back(*vecPlanes_potential_invalid[1]);

        }
        else 
            std::cout << " Error! Size of Potential Planes : " << vecPlanes_potential_invalid.size() << std::endl;

        // 2) 物体检测的约束平面        

        // 构建一个约束函数
        double ground_weight = Config::ReadValue<double>("SemanticPrior.GroundWeight");
        std::cout << "Ground Plane Weight : " << ground_weight << std::endl;
        return optimizeEllipsoidWithPlanesAndPrior(e, planes, planesWithNormal, pri, weight, ground_weight);
    }

    // 其中地面的约束法向量，已经隐藏在了 ellipsoid 里面
    // 提取该椭球体时，已经固定了朝向为地面法向量
    // e: 初始推断
    g2o::ellipsoid priorInfer::MonocularInfer(g2o::ellipsoid &e, const Pri &pri, double weight, g2o::plane& plane_ground)
    {
        std::vector<g2o::plane> planes;
        std::vector<g2o::plane> planesWithNormal;

        // 已有地平面旋转约束： 2

        // 地平面相切约束： 1
        planes.push_back(plane_ground);

        // 还需要传入 bbox. x4
        int bbox_planes_num = e.mvCPlanesInCamera.size();
        std::vector<g2o::plane> planes_bbox; planes_bbox.resize(bbox_planes_num);
        for(int i=0;i<bbox_planes_num;i++){
            auto pCP = e.mvCPlanesInCamera[i];
            if(pCP)
                if(pCP->pPlane)
                    planes_bbox[i] = *(pCP->pPlane);
        }
        planes.insert(planes.end(), planes_bbox.begin(), planes_bbox.end());

        // 比例约束 x2 in Pri
        std::cout << " Monocular Plane Num : " << planes.size() << std::endl;

        double ground_weight = Config::ReadValue<double>("SemanticPrior.GroundWeight");
        std::cout << "Ground Plane Weight : " << ground_weight << std::endl;
        return optimizeEllipsoidWithPlanesAndPrior(e, planes, planesWithNormal, pri, weight, ground_weight);
    }

    g2o::ellipsoid priorInfer::MonocularInferWithNearFarPlane(g2o::ellipsoid &e, const Pri &pri, double weight, g2o::plane& plane_ground, std::vector<g2o::plane> &planesNearFar)
    {
        std::vector<g2o::plane> planes;
        std::vector<g2o::plane> planesWithNormal;

        // 已有地平面旋转约束： 2

        // 地平面相切约束： 1
        planes.push_back(plane_ground);

        // 还需要传入 bbox. x4
        int bbox_planes_num = e.mvCPlanesInCamera.size();
        std::vector<g2o::plane> planes_bbox; planes_bbox.resize(bbox_planes_num);
        for(int i=0;i<bbox_planes_num;i++){
            auto pCP = e.mvCPlanesInCamera[i];
            if(pCP)
                if(pCP->pPlane)
                    planes_bbox[i] = *(pCP->pPlane);
        }
        planes.insert(planes.end(), planes_bbox.begin(), planes_bbox.end());
        planes.insert(planes.end(), planesNearFar.begin(), planesNearFar.end());

        // 比例约束 x2 in Pri
        std::cout << " Monocular Plane Num : " << planes.size() << std::endl;

        double ground_weight = Config::ReadValue<double>("SemanticPrior.GroundWeight");
        std::cout << "Ground Plane Weight : " << ground_weight << std::endl;
        return optimizeEllipsoidWithPlanesAndPrior(e, planes, planesWithNormal, pri, weight, ground_weight);
    }

    // Expand: 考虑多种可能性做初始化
    g2o::ellipsoid priorInfer::MonocularInferExpand(g2o::ellipsoid &e, const Pri &pri, double weight, g2o::plane& plane_ground)
    {
        std::cout << " **** INFER WITH ALL POSSIBILITIES!" << std::endl;
        double pri_a = 1;
        double pri_d = pri.GetD();
        double pri_e = pri.GetE();
        Vector3d oriScale = e.scale;
        Vector3d initScale;
        initScale << oriScale[0] * pri_a, oriScale[1] * pri_d, oriScale[2] * pri_e;

        // 交换产生6个 分别执行monocularInfer，然后取误差最小的一个
        static const int c33_tables[6][3] = {
            {0,1,2},
            {0,2,1},
            {1,0,2},
            {1,2,0},
            {2,0,1},
            {2,1,0}
        };

        std::vector<double> costList; costList.resize(6);
        std::vector<g2o::ellipsoid> e_list; e_list.resize(6);
        for(int i=0;i<6;i++)
        {
            g2o::ellipsoid e_0 = e;
            Vector3d switchScale;
            switchScale << initScale[c33_tables[i][0]], initScale[c33_tables[i][1]], initScale[c33_tables[i][2]];
            e_0.scale = switchScale;
            g2o::ellipsoid e_0_infer = this->MonocularInfer(e_0, pri, weight, plane_ground);
            double cost = this->GetLastCost();
            costList[i] = cost;
            e_list[i] = e_0_infer;

            std::cout << "e_0 for " << i << " : " << e_0.scale.transpose() << std::endl;
        }

        // 开始找出最小的cost
        auto iter_min = std::min_element(costList.begin(), costList.end());
        int pos = std::distance(costList.begin(), iter_min);

        bool bDebug = false;
        if(bDebug)
        {
            // Print all cost
            std::cout << " -- Cost \t Ellipsoid " << std::endl;
            for(int i=0;i<6;i++)
            {
                std::cout << " -- " << costList[i] << "\t" << e_list[i].toMinimalVector().transpose() << std::endl;;
            }
            std::cout << "Final Decision: " << pos << std::endl;
        }

        // Save
        mvePossibleEllipsoids = e_list;

        // 最佳
        return e_list[pos];
    }

    std::vector<g2o::ellipsoid> priorInfer::GetAllPossibleEllipsoids()
    {
        return mvePossibleEllipsoids;
    }

    g2o::ellipsoid priorInfer::optimizeEllipsoidWithPlanesAndPrior(const g2o::ellipsoid &init_guess, std::vector<g2o::plane> &planes,
                                                       std::vector<g2o::plane> &planesWithNormal, const Pri &pri, double weight, double ground_plane_weight)
    {
        // 基本参数的读取
        double config_plane_angle_sigma = Config::Get<double>("Optimizer.Edges.3DConstrain.PlaneAngle.Sigma");
        bool bUseGroundPlaneWeight = true;  // 请将地平面放在平面约束的第一个！

        // 基本配置： 是否使用的是单目版本
        // Debug: 取消单目版本.
        // bool bMonocularVersion = (ground_plane_weight > 0);
        // bool bMonocularVersion = false;

        // 做g2o优化，直到 cost function 最小

        // initialize graph optimization.
        g2o::SparseOptimizer graph;
        g2o::BlockSolverX::LinearSolverType *linearSolver;
        linearSolver = new g2o::LinearSolverDense<g2o::BlockSolverX::PoseMatrixType>();
        g2o::BlockSolverX *solver_ptr = new g2o::BlockSolverX(linearSolver);
        g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
        graph.setAlgorithm(solver);
        graph.setVerbose(false); // Set output.

        // 添加椭球体
        // Add objects vertices
        g2o::VertexEllipsoidXYZABCYaw *vEllipsoid;

        // 注意单目版本和非单目版本区别
        // if(bMonocularVersion)
        //     vEllipsoid = new g2o::VertexEllipsoidXYABHeightYaw();
        // else 
        //     vEllipsoid = new g2o::VertexEllipsoidXYZABCYaw();
        vEllipsoid = new g2o::VertexEllipsoidXYZABCYaw();
        vEllipsoid->setEstimate(init_guess);
        vEllipsoid->setId(graph.vertices().size());
        vEllipsoid->setFixed(false);
        graph.addVertex(vEllipsoid);

        // 添加约束
        // 这里的平面与椭球体已经位于同一个坐标系，所以创建一个单位变换作为SE3
        g2o::VertexSE3Expmap *vSE3 = new g2o::VertexSE3Expmap();
        vSE3->setId(graph.vertices().size());
        vSE3->setEstimate(g2o::SE3Quat()); // Identity
        vSE3->setFixed(true);
        graph.addVertex(vSE3);

        // 1) 边约束 : 注意包括无normal边 和 normal 边 ( 约束其朝向 ). 等会儿，是否对朝向做估计？ 要不只估计 x,y,z,a,b,c

        // 关闭边约束的朝向!
        int flag_valid_angle = 1;
        for (int i = 0; i < planesWithNormal.size(); i++)
        {
            g2o::EdgeSE3EllipsoidPlaneWithNormal* pEdge = new g2o::EdgeSE3EllipsoidPlaneWithNormal;
            pEdge->setId(graph.edges().size());
            pEdge->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>( vSE3 ));
            pEdge->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>( vEllipsoid ));
            pEdge->setMeasurement(planesWithNormal[i].param);

            Matrix<double,2,1> inv_sigma;
            inv_sigma << 1, 1/(config_plane_angle_sigma * 1 / 180.0 * M_PI) * flag_valid_angle;   // 距离, 角度标准差 ; 暂时不管

            double pl_weight = 1;
            if(bUseGroundPlaneWeight && i==0) pl_weight = ground_plane_weight;
            inv_sigma = inv_sigma * pl_weight;
            MatrixXd info = inv_sigma.cwiseProduct(inv_sigma).asDiagonal();
            pEdge->setInformation(info);
            pEdge->setRobustKernel( new g2o::RobustKernelHuber() );

            graph.addEdge(pEdge);
        }

        // 2) 无normal 边约束
        for (int i = 0; i < planes.size(); i++)
        {
            g2o::EdgeSE3EllipsoidPlane *pEdge = new g2o::EdgeSE3EllipsoidPlane;
            pEdge->setId(graph.edges().size());
            pEdge->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(vSE3));
            pEdge->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(vEllipsoid));
            pEdge->setMeasurement(planes[i].param);

            pEdge->setNormalDirection(true);   // 要求被约束的椭球体在平面的法向量方向，防止其出现在地面下方或相机后方。

            double pl_weight = 1;
            if(bUseGroundPlaneWeight && i==0) pl_weight = ground_plane_weight;
            Matrix<double, 1, 1> inv_sigma;
            inv_sigma << 1 * pl_weight;
            MatrixXd info = inv_sigma.cwiseProduct(inv_sigma).asDiagonal();
            pEdge->setInformation(info);
            pEdge->setRobustKernel(new g2o::RobustKernelHuber());

            graph.addEdge(pEdge);
        }

        // 添加先验约束
        EdgePri *pEdgePri = new EdgePri;
        pEdgePri->setId(graph.edges().size());
        pEdgePri->setVertex(0, vEllipsoid);
        pEdgePri->setMeasurement(pri);
        Vector2d inv_sigma;
        inv_sigma << 1, 1;
        inv_sigma = inv_sigma * weight;
        MatrixXd info = inv_sigma.cwiseProduct(inv_sigma).asDiagonal();
        pEdgePri->setInformation(info);
        graph.addEdge(pEdgePri);

        // 开始优化
        int num_opt = 10;
        std::cout << "Begin Optimization of ellipsoid with prior... x " << num_opt << std::endl;
        std::cout << " - Plane Num : " << planes.size() << std::endl;
        std::cout << " - PlaneWithNormal Num : " << planesWithNormal.size() << std::endl;
        graph.initializeOptimization();
        graph.optimize( num_opt );  //optimization step
        std::cout << "Optimization done." << std::endl;

        // 保存最终 cost 
        mdCost = graph.chi2();

        return vEllipsoid->estimate();
    }


    g2o::ellipsoid priorInfer::optimizeEllipsoidWithMultiPlanes(const g2o::ellipsoid &init_guess, std::vector<g2o::plane> &planes, const Pri &pri)
    {
        double ground_plane_weight = Config::ReadValue<double>("SemanticPrior.GroundWeight");
        std::cout << "Ground Plane Weight : " << ground_plane_weight << std::endl;
        
        // 基本参数的读取
        double config_plane_angle_sigma = Config::Get<double>("Optimizer.Edges.3DConstrain.PlaneAngle.Sigma");
        bool bUseGroundPlaneWeight = true;  // 请将地平面放在平面约束的第一个！

        // 基本配置： 是否使用的是单目版本
        // Debug: 取消单目版本.
        // bool bMonocularVersion = (ground_plane_weight > 0);
        // bool bMonocularVersion = false;

        // 做g2o优化，直到 cost function 最小

        // initialize graph optimization.
        g2o::SparseOptimizer graph;
        g2o::BlockSolverX::LinearSolverType *linearSolver;
        linearSolver = new g2o::LinearSolverDense<g2o::BlockSolverX::PoseMatrixType>();
        g2o::BlockSolverX *solver_ptr = new g2o::BlockSolverX(linearSolver);
        g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
        graph.setAlgorithm(solver);
        graph.setVerbose(false); // Set output.

        // 添加椭球体
        // Add objects vertices
        g2o::VertexEllipsoidXYZABCYaw *vEllipsoid;

        // 注意单目版本和非单目版本区别
        // if(bMonocularVersion)
        //     vEllipsoid = new g2o::VertexEllipsoidXYABHeightYaw();
        // else 
        //     vEllipsoid = new g2o::VertexEllipsoidXYZABCYaw();
        vEllipsoid = new g2o::VertexEllipsoidXYZABCYaw();
        vEllipsoid->setEstimate(init_guess);
        vEllipsoid->setId(graph.vertices().size());
        vEllipsoid->setFixed(false);
        graph.addVertex(vEllipsoid);

        // 添加约束
        // 这里的平面与椭球体已经位于同一个坐标系，所以创建一个单位变换作为SE3
        g2o::VertexSE3Expmap *vSE3 = new g2o::VertexSE3Expmap();
        vSE3->setId(graph.vertices().size());
        vSE3->setEstimate(g2o::SE3Quat()); // Identity
        vSE3->setFixed(true);
        graph.addVertex(vSE3);

        //  无normal 边约束, 包括地面
        for (int i = 0; i <    planes.size(); i++)
        {
            g2o::EdgeSE3EllipsoidPlane *pEdge = new g2o::EdgeSE3EllipsoidPlane;
            pEdge->setId(graph.edges().size());
            pEdge->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(vSE3));
            pEdge->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(vEllipsoid));
            pEdge->setMeasurement(planes[i].param);

            pEdge->setNormalDirection(true);   // 要求被约束的椭球体在平面的法向量方向，防止其出现在地面下方或相机后方。

            double pl_weight = 1;
            if(bUseGroundPlaneWeight && i==0) pl_weight = ground_plane_weight;
            Matrix<double, 1, 1> inv_sigma;
            inv_sigma << 1 * pl_weight;
            MatrixXd info = inv_sigma.cwiseProduct(inv_sigma).asDiagonal();
            pEdge->setInformation(info);
            pEdge->setRobustKernel(new g2o::RobustKernelHuber());

            graph.addEdge(pEdge);
        }
         // 添加先验约束
        bool usePrior = Config::ReadValue<double>("SemanticPrior.usePrior");
        double weight = Config::ReadValue<double>("SemanticPrior.Weight");
        if(usePrior){
            EdgePri *pEdgePri = new EdgePri;
            pEdgePri->setId(graph.edges().size());
            pEdgePri->setVertex(0, vEllipsoid);
            pEdgePri->setMeasurement(pri);
            Vector2d inv_sigma;
            inv_sigma << 1, 1;
            inv_sigma = inv_sigma * weight;
            MatrixXd info = inv_sigma.cwiseProduct(inv_sigma).asDiagonal();
            pEdgePri->setInformation(info);
            graph.addEdge(pEdgePri);
        }

        // 开始优化
        int num_opt = 10;
        std::cout << "Begin Optimization of ellipsoid with prior... x " << num_opt << std::endl;
        std::cout << " - Plane Num : " << planes.size() << std::endl;
        graph.initializeOptimization();
        graph.optimize( num_opt );  //optimization step
        std::cout << "Optimization done." << std::endl;

        // 保存最终 cost 
        mdCost = graph.chi2();

        return vEllipsoid->estimate();
    }


    // 边的计算
    void EdgePri::computeError()
    {
        const VertexEllipsoidXYZABCYaw *eVertex = static_cast<const VertexEllipsoidXYZABCYaw *>(_vertices[0]); //  object pose to world
        ellipsoid global_e = eVertex->estimate();

        // 开始计算 pri

        Pri p(global_e);
        _error = p - _measurement;

        // std::cout << "p" << std::endl; p.print();
        // std::cout << "_measurement: " << std::endl; _measurement.print();
        // std::cout << "Error of Pri : " << _error.transpose() << std::endl;
    }

    EdgePri::EdgePri()
    {

    }

    bool EdgePri::read(std::istream& is){
        return true;
    };

    bool EdgePri::write(std::ostream& os) const
    {
        return os.good();
    };

    bool PriFactor::LoadPriConfigurations(const string& str_path)
    {
        Eigen::MatrixXd mat = readDataFromFile(str_path.c_str());
        std::cout << std::endl;
        std::cout << "Pri Table : " << std::endl;
        std::cout << "Label \t s0 \t s1 " << std::endl;
        std::cout << mat << std::endl;
        std::cout << std::endl;
        int rows = mat.rows();
        for(int i=0;i<rows;i++)
        {
            Eigen::VectorXd rowData = mat.row(i);  // label d e 

            // std::cout << "row " << i << " : " << rowData.transpose() << std::endl;

            double value_d = rowData[1];
            double value_e = rowData[2];

            int label = round(rowData[0]);
            std::string key_d = PREFIX_D + to_string(label);
            std::string key_e = PREFIX_E + to_string(label);
            Config::SetValue(key_d, value_d);
            Config::SetValue(key_e, value_e);
        }
        
        std::cout << "Read pri config from " << str_path << std::endl;
        std::cout << "Total Num : " << rows << std::endl;
    }

    Pri PriFactor::CreatePri(int label)
    {
        // 从 Config 中读取参数
        std::string key_d = PREFIX_D + to_string(label);
        std::string key_e = PREFIX_E + to_string(label);

        double d = Config::ReadValue<double>(key_d);
        double e = Config::ReadValue<double>(key_e);

        if( d>0 && e>0)
            return Pri(d,e);
        else 
            return Pri(1,1);
    }

} // namespace EllipsoidSLAM