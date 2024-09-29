#include "EllipsoidExtractor.h"
#include "EllipsoidExtractorEdges.h"

#include "Thirdparty/g2o/g2o/core/block_solver.h"
#include "Thirdparty/g2o/g2o/core/optimization_algorithm_levenberg.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_eigen.h"
#include "Thirdparty/g2o/g2o/types/types_six_dof_expmap.h"
#include "Thirdparty/g2o/g2o/core/robust_kernel_impl.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_dense.h"
#include "Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"


namespace ORB_SLAM2
{

g2o::ellipsoid EllipsoidExtractor::OptimizeEllipsoidUsingPlanes(g2o::ellipsoid &e_in, MatrixXd& mPlanesParam)
{
    // 此处使用图优化, 与各个平面做优化.
    // 迭代过程中，固定其中一个转角轴? 即垂直方向解耦合.?
    // 先不管细节优化，先实现功能. 即，重力方向约束 + 各平面的联合优化.
    // 立方体也直接转为平面. - > 部分平面是选择性的: 遮挡部分去除.
    
    // initialize graph optimization.
    // 创建一个图、线性求解器、块求解器、LB求解器、
    g2o::SparseOptimizer graph;
    g2o::BlockSolverX::LinearSolverType* linearSolver;
    linearSolver = new g2o::LinearSolverDense<g2o::BlockSolverX::PoseMatrixType>();
    g2o::BlockSolverX * solver_ptr = new g2o::BlockSolverX(linearSolver);
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    graph.setAlgorithm(solver);
    graph.setVerbose(false);        // Set output.

    // Add objects vertices
    // 添加物体顶点
    g2o::VertexEllipsoidXYZABC *vEllipsoid = new g2o::VertexEllipsoidXYZABC();
    vEllipsoid->setEstimate(e_in);
    vEllipsoid->setId(graph.vertices().size());
    vEllipsoid->setFixed(false);
    graph.addVertex(vEllipsoid);    

    // // Add gravity prior
    // if(mbGroundPlaneSet && mbSetGravityPrior ){
    //     g2o::EdgeEllipsoidGravityPlanePrior *vGravityPriorEdge = new g2o::EdgeEllipsoidGravityPlanePrior;
    //     vGravityPriorEdge->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>( vEllipsoid ));

    //     vGravityPriorEdge->setMeasurement(mGroundPlaneNormal);  
    //     Matrix<double,1,1> inv_sigma;
    //     inv_sigma << 1 * dGravityPriorScale;
    //     MatrixXd info = inv_sigma.cwiseProduct(inv_sigma).asDiagonal();
    //     vGravityPriorEdge->setInformation(info);
        
    //     graph.addEdge(vGravityPriorEdge);
    //     // edgesEllipsoidGravityPlanePrior.push_back(vGravityPriorEdge);
        
    // }

    // Add edges from planes

    // 需要创建新的边. 即 3d 约束 plane-ellipsoid
    int plane_num = mPlanesParam.rows();
    for( int i=0;i<plane_num;i++)
    {
        Vector4d planeVec = mPlanesParam.row(i).head(4);
        g2o::EdgeEllipsoidPlane* pEdge = new g2o::EdgeEllipsoidPlane;
        pEdge->setId(graph.edges().size());
        pEdge->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>( vEllipsoid ));
        pEdge->setMeasurement(planeVec);

        Matrix<double,1,1> inv_sigma;
        inv_sigma << 1;
        MatrixXd info = inv_sigma.cwiseProduct(inv_sigma).asDiagonal();
        pEdge->setInformation(info);

        graph.addEdge(pEdge);
    }

    graph.initializeOptimization();
    graph.optimize( 10 );  //optimization step
    // DEBUG: CLOSE IT!!!!
    std::cout << "[ATTENTION LOCALOPTIMIZATION] CLOSED." << std::endl;

    g2o::ellipsoid e_optimized = vEllipsoid->estimate();

    return e_optimized;
}

}