#include "TextureOptimizer.h"

#include <iostream>
#include "../../func/func.h"
#include "Optimizer.h"

#include "Thirdparty/g2o/g2o/core/block_solver.h"
#include "Thirdparty/g2o/g2o/core/optimization_algorithm_levenberg.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_eigen.h"
#include "Thirdparty/g2o/g2o/types/types_six_dof_expmap.h"
#include "Thirdparty/g2o/g2o/core/robust_kernel_impl.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_dense.h"
#include "Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"

#include "TextureEdges.h"
#include "src/pca/EllipsoidExtractorEdges.h"
#include "include/core/ConstrainPlane.h"

#include "include/core/PriorInfer.h"

using namespace std;
using namespace ORB_SLAM2;

// 定义拷贝构造函数
KeyPointSon::KeyPointSon(const cv::KeyPoint &kp)
{
    this->pt = kp.pt;
}

// 定义 == 运算符
bool KeyPointSon::operator==(const KeyPointSon &kp) const
{
    auto diff = kp.pt - this->pt;
    return ((std::abs(diff.x) + std::abs(diff.y)) < 1e-3);
}

vector<KeyPointSon> KeyPointSon::Generate(const vector<cv::KeyPoint> &kps)
{
    vector<KeyPointSon> kpsout;
    kpsout.reserve(kps.size());
    for (auto &kp : kps)
        kpsout.push_back(KeyPointSon(kp));
    return kpsout;
}

g2o::EdgeSE3EllipsoidTexture* GenerateTextureEdge(g2o::VertexSE3Expmap *vSE3, g2o::VertexEllipsoidXYZABCYaw *vEllipsoid, 
        const Observation& ob_2d, const Eigen::Matrix3d& calib, int edge_id)
{
    g2o::EdgeSE3EllipsoidTexture* pEdge = new g2o::EdgeSE3EllipsoidTexture;
    pEdge->setId(edge_id);
    pEdge->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>( vSE3 ));
    pEdge->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>( vEllipsoid ));
    Matrix<double,1,1> inv_sigma;
    // inv_sigma << 1 / 128.0;
    inv_sigma << 1;
    MatrixXd info = inv_sigma.cwiseProduct(inv_sigma).asDiagonal();
    pEdge->setInformation(info);
    pEdge->setParam(ob_2d.gray, ob_2d.bbox, ob_2d.keyPoints, ob_2d.descripts, calib);
    pEdge->setRobustKernel( new g2o::RobustKernelHuber() );
    
    return pEdge;
}

g2o::EdgeSE3EllipsoidTextureDT* GenerateTextureEdgeDT(g2o::VertexSE3Expmap *vSE3, g2o::VertexEllipsoidXYZABCYaw *vEllipsoid, 
        const Observation& ob_2d, const Eigen::Matrix3d& calib, int edge_id)
{
    g2o::EdgeSE3EllipsoidTextureDT* pEdge = new g2o::EdgeSE3EllipsoidTextureDT;
    pEdge->setId(edge_id);
    pEdge->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>( vSE3 ));
    pEdge->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>( vEllipsoid ));
    Matrix<double,1,1> inv_sigma;
    // inv_sigma << 1 / 128.0;
    double sigma = 50;
    inv_sigma << 1 / sigma;
    MatrixXd info = inv_sigma.cwiseProduct(inv_sigma).asDiagonal();
    pEdge->setInformation(info);
    pEdge->setParam(ob_2d.dtMat, ob_2d.bbox, ob_2d.keyPoints, calib);
    pEdge->setRobustKernel( new g2o::RobustKernelHuber() );
    
    return pEdge;
}

// // 新: 该版本使用 RGB-D 平面约束，优化在“支撑平面”之上放着的物体
// void TextureOptimizer::graphOptimizeWithPlanes(std::vector<Frame *> &pFrames,
//                 Measurements& mms, Objects& objs, Trajectory& camTraj, const Matrix3d& calib, int iRows, int iCols,
//                 const Vector4d& ground_plane) {

//     // 注意 ： 开启了3d 平面的局部过滤. 30 pixel
//     // 注意 ： 开启了2d 平面的局部过滤. param设置 30pixel. 只有非局部平面才添加约束.
//     // *************************************

//     // ************************ LOAD CONFIGURATION ************************
//     double config_ellipsoid_3d_scale = Config::ReadValue<double>("Optimizer.Edges.3DEllipsoid.Scale");
//     double config_ellipsoid_2d_scale = Config::ReadValue<double>("Optimizer.Edges.2D.Scale");
//     // if(config_ellipsoid_2d_scale <= 0.01)
//     //     config_ellipsoid_2d_scale = 1.0;
//     double dGravityPriorScale = Config::Get<double>("Optimizer.Edges.GravityPrior.Scale");
//     bool mbOpenPartialObservation = Config::Get<int>("Optimizer.PartialObservation.Open") == 1;
//     bool mbOpen3DProb = true;  

//     std::cout << " -- Image : " << iCols << " x " << iRows << std::endl;

//     // OUTPUT
//     std::cout << " -- Optimization parameters : " << std::endl;
    
//     cout<<" * Scale_3dedge: " << config_ellipsoid_3d_scale << endl;
//     cout<<" * Scale_2dedge: " << config_ellipsoid_2d_scale << endl;
//     cout<<" * Scale_GravityPrior: " << dGravityPriorScale << endl;

//     double config_odometry_weight = Config::ReadValue<double>("DEBUG.ODOM.WEIGHT");
//     double exp_config_odometry_weight = config_odometry_weight; // normal
//     cout << " * DEBUG-Odometry weight : " << exp_config_odometry_weight << endl;

//     // 添加一个关闭 2D 约束的开关.
//     bool mbClose2DConstrain = Config::Get<int>("Optimizer.Edges.2DConstrain.Close") > 0;
//     if(mbClose2DConstrain)
//         cout << "********** CLOSE 2D Constrain. **********" << endl;
//     bool bOpen3DEllipsoidEdges = !(Config::Get<int>("Optimizer.Edges.3DConstrain.Close") > 0);
//     if(!bOpen3DEllipsoidEdges)
//         cout << "********** CLOSE 3D Constrain. **********" << endl;
//     // ************************************************************************

//     // Initialize variables.
//     int total_frame_number = int(pFrames.size());
//     int objects_num = int(objs.size());

//     // initialize graph optimization.
//     g2o::SparseOptimizer graph;
//     g2o::BlockSolverX::LinearSolverType* linearSolver;
//     linearSolver = new g2o::LinearSolverDense<g2o::BlockSolverX::PoseMatrixType>();
//     g2o::BlockSolverX * solver_ptr = new g2o::BlockSolverX(linearSolver);
//     g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
//     graph.setAlgorithm(solver);
//     graph.setVerbose(true);        // Set output.

//     std::map<int, g2o::VertexEllipsoidXYZABCYaw*> vEllipsoidVertexMaps;
//     std::vector<g2o::EdgeSE3EllipsoidProj*> edges, edgesValid, edgesInValid;
//     std::vector<bool> validVec; validVec.resize(total_frame_number);
//     std::vector<g2o::EdgeEllipsoidGravityPlanePrior *> edgesEllipsoidGravityPlanePrior;     // Gravity prior
//     std::vector<g2o::VertexSE3Expmap*> vSE3Vertex;

//     std::vector<g2o::EdgeSE3Ellipsoid7DOF*> vEllipsoid3DEdges;
//     std::vector<g2o::EdgeSE3Expmap*> vEdgesOdom;
//     std::vector<g2o::EdgeSE3EllipsoidPlane*> vEdgeEllipsoidPlane;
//     std::vector<g2o::EdgeSE3EllipsoidPlane*> vEdgeEllipsoidPlane3D;
//     std::vector<g2o::EdgeSE3EllipsoidPlaneWithNormal*> vEdgeEllipsoidPlane3DWithNormal;
//     std::vector<g2o::EdgeSE3EllipsoidPlanePartial*> vEdgeEllipsoidPlanePartial;
//     std::vector<g2o::EdgeSE3EllipsoidCenterFull*> vEdgeSE3EllipsoidCenterFull;

//     std::map<int, std::vector<g2o::EdgeSE3EllipsoidPlane*>> mapInsEdgePlanes;       // 8-27 check : 该部分统计并没有考虑新的带 normal 的法向量
//     std::map<int, std::vector<g2o::EdgeSE3EllipsoidPlanePartial*>> mapInsEdgePlanesPartial; 

//     // Add SE3 vertices for camera poses
//     // bool bSLAM_mode = (Config::Get<int>("Optimizer.SLAM.mode") == 1);   // Mapping Mode : Fix camera poses and mapping ellipsoids only
//     bool bSLAM_mode = false;
//     std::cout << " [ SLAM Mode : " << bSLAM_mode << " ] " << std::endl;
//     for( int frame_index=0; frame_index< total_frame_number ; frame_index++) {
//         g2o::SE3Quat curr_cam_pose_Twc = pFrames[frame_index]->cam_pose_Twc;

//         g2o::VertexSE3Expmap *vSE3 = new g2o::VertexSE3Expmap();
//         vSE3->setId(graph.vertices().size());
//         graph.addVertex(vSE3);
//         vSE3->setEstimate(pFrames[frame_index]->cam_pose_Tcw); // Tcw
//         if(!bSLAM_mode)
//             vSE3->setFixed(true);       // Fix all the poses in mapping mode.
//         else 
//             vSE3->setFixed(frame_index == 0);   
//         vSE3Vertex.push_back(vSE3);

//         // Add odom edges if in SLAM Mode
//         if(bSLAM_mode && frame_index > 0){
//             g2o::SE3Quat prev_cam_pose_Tcw = pFrames[frame_index-1]->cam_pose_Twc.inverse();
//             g2o::SE3Quat curr_cam_pose_Tcw = curr_cam_pose_Twc.inverse();

//             // 最后的 inverse: 将 wc 转为 cw
//             // g2o::SE3Quat odom_val = (curr_cam_pose_Tcw*prev_cam_pose_Tcw.inverse()).inverse();; //  odom_wc * To = T1
//             g2o::SE3Quat odom_val = curr_cam_pose_Tcw*prev_cam_pose_Tcw.inverse(); 

//             g2o::EdgeSE3Expmap* e = new g2o::EdgeSE3Expmap();
//             e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>( vSE3Vertex[frame_index-1] ));
//             e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>( vSE3Vertex[frame_index] ));
//             e->setMeasurement(odom_val);

//             e->setId(graph.edges().size());
//             Vector6d inv_sigma;inv_sigma<<1,1,1,1,1,1;
//             inv_sigma = inv_sigma*exp_config_odometry_weight;
//             Matrix6d info = inv_sigma.cwiseProduct(inv_sigma).asDiagonal();
//             e->setInformation(info);
//             graph.addEdge(e);

//             vEdgesOdom.push_back(e);
//         }
//     }

//     // 接下来： 以 mms, objs 构造图.
//     // Initialize objects vertices and add edges of camera-objects 2d observations 
//     int count_partial_3d_planes = 0;
//     int count_pointmodel_num = 0;
//     std::vector<std::vector<g2o::ConstrainPlane*>> vwcPlanes;
//     std::map<int,int> mmInstanceObservationNum;
//     for(int object_id=0; object_id<objects_num; object_id++ )        
//     {
//         Object &obj = objs[object_id];
//         int instance = obj.instance_id;
//         int label = obj.pEllipsoid->miLabel;
//         // 查语义表格判断该物体是否具有对称性质
//         bool bSemanticSymmetry = IsSymmetry(label);

//         // Add objects vertices
//         g2o::VertexEllipsoidXYZABCYaw *vEllipsoid = new g2o::VertexEllipsoidXYZABCYaw();
//         vEllipsoid->setEstimate(*obj.pEllipsoid);
//         vEllipsoid->setId(graph.vertices().size());
//         vEllipsoid->setFixed(false);
//         graph.addVertex(vEllipsoid);
//         vEllipsoidVertexMaps.insert(make_pair(instance, vEllipsoid)); 

//         // std::vector<g2o::ConstrainPlane*> vCPlanesWorld;
//         // MatrixXd world_constrain_planes; world_constrain_planes.resize(0,4);
//         // 添加 3d 约束 : 来自三维立方体构造时所用的三维平面.
//         for( int i=0; i<obj.measurementIDs.size(); i++ )
//         {
//             Measurement& measurement = mms[obj.measurementIDs[i]];
//             int instance = measurement.instance_id;
            
//             Observation3D& ob_3d = measurement.ob_3d;
//             g2o::ellipsoid* pObj_ob = ob_3d.pObj;
            
//             if( pObj_ob == NULL ) continue;

//             // if( i < 3 ) std::cout << "pObj_ob->prob : " << pObj_ob->prob << std::endl;

//             int frame_index = measurement.ob_3d.pFrame->frame_seq_id;
//             auto vEllipsoid = vEllipsoidVertexMaps[instance];  
//             auto vSE3 = vSE3Vertex[frame_index];

//             std::vector<g2o::ConstrainPlane*> vCPlanes = pObj_ob->mvCPlanes;
//             // MatrixXd mPlanesParam = pObj_ob->cplanes;
//             int plane_num = vCPlanes.size();

//             g2o::SE3Quat &Twc = measurement.ob_2d.pFrame->cam_pose_Twc;

//             if(!mbClose2DConstrain){
//                 for( int i=0;i<plane_num;i++)
//                 {
//                     g2o::ConstrainPlane* pCPlane = vCPlanes[i];
//                     Vector4d planeVec = pCPlane->pPlane->param.head(4); // local coordinate

//                     // 为边缘平面添加特制的约束
//                     if(pCPlane->valid){
//                         g2o::EdgeSE3EllipsoidPlane* pEdge = new g2o::EdgeSE3EllipsoidPlane;
//                         pEdge->setId(graph.edges().size());
//                         pEdge->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>( vSE3 ));
//                         pEdge->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>( vEllipsoid ));
//                         pEdge->setMeasurement(planeVec);

//                         Matrix<double,1,1> inv_sigma;
//                         inv_sigma << 1 * config_ellipsoid_2d_scale;
//                         MatrixXd info = inv_sigma.cwiseProduct(inv_sigma).asDiagonal();
//                         pEdge->setInformation(info);
//                         pEdge->setRobustKernel( new g2o::RobustKernelHuber() );

//                         graph.addEdge(pEdge);
//                         vEdgeEllipsoidPlane.push_back(pEdge);
//                         mapInsEdgePlanes[instance].push_back(pEdge);
                        
//                         mmInstanceObservationNum[instance]++;
//                     }
//                 }
//         } // 观测遍历

//     } // objs 遍历

//     // output 
//     int valid_plane_num = vEdgeEllipsoidPlane.size();
//     int partial_plane_num = vEdgeEllipsoidPlanePartial.size();
//     int total_plane_num = valid_plane_num + partial_plane_num;
//     std::cout << std::endl << " ---- GRAPH INFORMATION ---- : " << std::endl;
//     cout << " * Object Num : " << objects_num << endl;
//     cout<<  " * Vertices: "<<graph.vertices().size()<<endl;
//     cout << "   - SE3: " << vSE3Vertex.size() << endl;
//     cout << "   - Object: " << vEllipsoidVertexMaps.size() << endl;
//     cout << " * Edges: " << graph.edges().size() << endl;

//     if(!mbOpenPartialObservation)
//         cout << "   - Ellipsoid-Plane: " << total_plane_num << "( partial " << partial_plane_num << " [CLOSED])" << endl;
//     else
//         cout << "   - Ellipsoid-Plane: " << total_plane_num << "( partial " << partial_plane_num << " )" << endl;
//     if(bOpen3DEllipsoidEdges){
//         cout << "   - Ellipsoid-center: " << vEdgeSE3EllipsoidCenterFull.size() << std::endl;
//         cout << "   - Ellipsoid-3D: " << vEllipsoid3DEdges.size() << std::endl;
//         cout << "   - Ellipsoid-Plane-3D: " << vEdgeEllipsoidPlane3D.size() << "(filter partial:" << count_partial_3d_planes << ")" << std::endl;
//         cout << "   - Ellipsoid-Plane-3DWithNormal: " << vEdgeEllipsoidPlane3DWithNormal.size() << "(filter partial:" << count_partial_3d_planes << ")" << std::endl;
//     }
//     cout << "   - Odom: " << vEdgesOdom.size() << endl;
//     cout<<  "   - Gravity: " << edgesEllipsoidGravityPlanePrior.size() << endl;
//     cout << " * Object Measurements : " << mms.size() << endl;
//     cout << "   -- Point-Model: " << count_pointmodel_num << endl;
//     cout << endl;

//     // int num_optimize = Config::Get<double>("Optimizer.Optimize.Num");
//     int num_optimize = 1;
//     if(graph.edges().size()>0){
//         std::cout << "Begin Optimization..." << std::endl;
//         graph.initializeOptimization();
//         graph.optimize( num_optimize );  //optimization step
//         std::cout << "Optimization done." << std::endl;

//         // Update estimated ellispoids to the map
//         // 更新结果到 objs
//         for( int i = 0 ; i< objs.size() ; i++)
//         {
//             g2o::ellipsoid* pEllipsoid = objs[i].pEllipsoid;
//             if(pEllipsoid!=NULL)
//             {
//                 int instance = objs[i].instance_id;
//                 if(vEllipsoidVertexMaps.find(instance) == vEllipsoidVertexMaps.end()) continue;
//                 auto pEVertex = vEllipsoidVertexMaps[instance];
                
//                 bool colorSet = pEllipsoid->isColorSet();
//                 Vector4d old_color;
//                 if(colorSet) old_color = pEllipsoid->getColorWithAlpha();
//                 (*pEllipsoid) = pEVertex->estimate();
//                 pEllipsoid->miInstanceID = instance;
//                 if(colorSet) pEllipsoid->setColor(old_color.head(3), old_color[3]);   // 保持颜色不变.
//             }
//         }
//     }
//     else 
//     {
//         std::cout << " No observations valid." << std::endl;
//     }

//     return;
// }

std::vector<g2o::EdgeSE3EllipsoidPlane*> GeneratePlaneConstrainEdges(Vector4d& bbox, const g2o::SE3Quat& Twc, 
    g2o::VertexSE3Expmap *vSE3, g2o::VertexEllipsoidXYZABCYaw *vEllipsoid, Matrix3d& mCalib, int rows, int cols, int graph_edges)
{
    std::vector<g2o::ConstrainPlane*> vCPlanes = GenerateConstrainPlanesOfBbox(bbox, mCalib, rows, cols);
    // MatrixXd mPlanesParam = pObj_ob->cplanes;
    int plane_num = vCPlanes.size();

    std::vector<g2o::EdgeSE3EllipsoidPlane*> vEdgeEllipsoidPlane;
    vEdgeEllipsoidPlane.reserve(plane_num);
    for( int i=0;i<plane_num;i++)
    {
        g2o::ConstrainPlane* pCPlane = vCPlanes[i];
        Vector4d planeVec = pCPlane->pPlane->param.head(4); // local coordinate

        g2o::EdgeSE3EllipsoidPlane* pEdge = new g2o::EdgeSE3EllipsoidPlane;
        pEdge->setId(graph_edges+i);
        pEdge->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>( vSE3 ));
        pEdge->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>( vEllipsoid ));
        pEdge->setMeasurement(planeVec);

        Matrix<double,1,1> inv_sigma;
        inv_sigma << 1;
        MatrixXd info = inv_sigma.cwiseProduct(inv_sigma).asDiagonal();
        pEdge->setInformation(info);
        pEdge->setRobustKernel( new g2o::RobustKernelHuber() );

        vEdgeEllipsoidPlane.push_back(pEdge);
    }

    return vEdgeEllipsoidPlane;
}

// 构造比例先验约束
EdgePri* GeneratePriEdge(g2o::VertexEllipsoidXYZABCYaw *vEllipsoid, int label, double weight, int edge_id)
{
    // Pri pri(1,1);
    const string priconfig_path = Config::Get<std::string>("Dataset.Path.PriTable");
    PriFactor prifac;
    bool use_input_pri = (priconfig_path.size()>0);

    Pri pri;
    if(use_input_pri){
        prifac.LoadPriConfigurations(priconfig_path);
        pri = prifac.CreatePri(label);
    }
    else
        pri = Pri(1,1);

    // 添加先验约束
    EdgePri *pEdgePri = new EdgePri;
    pEdgePri->setId(edge_id);
    pEdgePri->setVertex(0, vEllipsoid);
    pEdgePri->setMeasurement(pri);
    Vector2d inv_sigma;
    inv_sigma << 1, 1;
    inv_sigma = inv_sigma * weight;
    MatrixXd info = inv_sigma.cwiseProduct(inv_sigma).asDiagonal();
    pEdgePri->setInformation(info);
    
    return pEdgePri;
}

    // std::vector<g2o::EdgeSE3EllipsoidPlane*> vEdgesSE3EllipsoidPlane;
    // std::vector<g2o::EdgeSE3EllipsoidTexture*> edgesTexture;
    // std::vector<EdgePri*> edgesPri;

void OutputEdgesErrors(std::vector<g2o::EdgeSE3EllipsoidPlane*> vEdges, std::string& file_name)
{
    std::string dir = "./logs/";
    // 设置一种打开后不会清空的形式
    ofstream out;
    out.open((dir+file_name).c_str(), ios::app);
    for(auto pEdge:vEdges)
    {
        out << "EdgeSE3EllipsoidPlane" << "\t"  // 输出类别
            << pEdge->id() << "\t"     // 输出边的ID
            << pEdge->error().transpose() << "\t"  // 输出误差
            << "\n";
    }
    out.close();
    return;
}

void OutputEdgesErrors(std::vector<g2o::EdgeSE3EllipsoidTexture*> vEdges, std::string& file_name)
{
    std::string dir = "./logs/";
    // 设置一种打开后不会清空的形式
    ofstream out;
    out.open((dir+file_name).c_str(), ios::app);
    for(auto pEdge:vEdges)
    {
        out << "EdgeSE3EllipsoidTexture" << "\t"  // 输出类别
            << pEdge->id() << "\t"     // 输出边的ID
            << pEdge->error().transpose() << "\t"  // 输出误差
            << "\n";
    }
    out.close();
    return;
}

void OutputEdgesErrors(std::vector<EdgePri*> vEdges, std::string& file_name)
{
    std::string dir = "./logs/";
    // 设置一种打开后不会清空的形式
    ofstream out;
    out.open((dir+file_name).c_str(), ios::app);
    for(auto pEdge:vEdges)
    {
        out << "EdgePri" << "\t"  // 输出类别
            << pEdge->id() << "\t"     // 输出边的ID
            << pEdge->error().transpose() << "\t"  // 输出误差
            << "\n";
    }
    out.close();
    return;
}

void SaveValuesMapToFile(const std::map<std::string, std::vector<double>>& valueMap, 
        const std::string& file_name)
{
    ofstream out;
    out.open(file_name.c_str());
    for(auto& pair:valueMap)
    {
        auto& name = pair.first;
        auto& valueVec = pair.second;
        out << name << " ";
        for(auto value:valueVec)
            out << value << " ";
        out << std::endl;
    }
    out.close();
}

void RunEdgeAnalysis(const std::map<int, g2o::VertexEllipsoidXYZABCYaw*>& vEllipsoidVertexMaps, 
            const std::vector<EdgePri*>& edgesPri, 
            const std::vector<g2o::EdgeEllipsoidXYZABCYawPlane *>& edgesGroundplane, 
            const std::vector<g2o::EdgeSE3EllipsoidPlane*>& vEdgesSE3EllipsoidPlane, 
            const std::vector<g2o::EdgeSE3EllipsoidTextureDT*>& edgesTextureDT)
{
    // 存储所有边的值的结构
    std::map<std::string, std::vector<double>> mapNameToValues;

    // *****
    g2o::VertexEllipsoidXYZABCYaw* pVertex = vEllipsoidVertexMaps.begin()->second;
    g2o::ellipsoid e = pVertex->estimate();

    int TOTAL_NUM = 180;
    std::vector<double> thetaLists;
    for (int i = 0; i < TOTAL_NUM; i++)
        thetaLists.push_back(double(i * (M_PI / double(TOTAL_NUM))));

    int current_id = 0;
    for (auto theta : thetaLists)
    {
        g2o::ellipsoid e_rot = e.rotate_ellipsoid(theta);
        pVertex->setEstimate(e_rot);

        // 开始计算、保存所有边
        for(int id=0;id<edgesPri.size();id++)
        {
            std::string name = "edgesPri" + to_string(id);
            auto edge = edgesPri[id];

            edge->computeError();
            double chi2 = edge->chi2();
            mapNameToValues[name].push_back(chi2);
        }

        for(int id=0;id<edgesGroundplane.size();id++)
        {
            std::string name = "edgesGroundplane" + to_string(id);
            auto edge = edgesGroundplane[id];
            
            edge->computeError();
            double chi2 = edge->chi2();
            mapNameToValues[name].push_back(chi2);
        }

        for(int id=0;id<vEdgesSE3EllipsoidPlane.size();id++)
        {
            std::string name = "vEdgesSE3EllipsoidPlane" + to_string(id);
            auto edge = vEdgesSE3EllipsoidPlane[id];
            
            edge->computeError();
            double chi2 = edge->chi2();
            mapNameToValues[name].push_back(chi2);
        }

        for(int id=0;id<edgesTextureDT.size();id++)
        {
            std::string name = "edgesTextureDT" + to_string(id);
            auto edge = edgesTextureDT[id];
            
            edge->computeError();
            double chi2 = edge->chi2();
            mapNameToValues[name].push_back(chi2);
        }

        current_id ++ ;
        if(current_id % 10 == 0)
        std::cout << "Finish rotation : " << theta << ", " << current_id << "/" << TOTAL_NUM << std::endl;
    }

    std::cout << "Save Value Map ... " << std::endl;
    // 输出Values
    SaveValuesMapToFile(mapNameToValues, "./edge_analysis_valueMap.txt");
    std::cout << "Save Value Map ... FINISHED" << std::endl;

    return;

}

g2o::EdgeEllipsoidXYZABCYawPlane * GenerateGroundplaneEdge(g2o::VertexEllipsoidXYZABCYaw *vEllipsoid, const Vector4d& normal, double weight, int edge_id)
{
    g2o::VertexSE3Expmap *vSE3 = new g2o::VertexSE3Expmap();      
    vSE3->setEstimate(g2o::SE3Quat()); // Tcw
    vSE3->setFixed(true);

    g2o::EdgeEllipsoidXYZABCYawPlane* pEdge = new g2o::EdgeEllipsoidXYZABCYawPlane;
    pEdge->setId(edge_id);
    pEdge->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>( vEllipsoid ));
    pEdge->setMeasurement(normal);

    Matrix<double,1,1> inv_sigma;
    inv_sigma << 1 * weight;
    MatrixXd info = inv_sigma.cwiseProduct(inv_sigma).asDiagonal();
    pEdge->setInformation(info);
    // pEdge->setRobustKernel( new g2o::RobustKernelHuber() );
   
    return pEdge;
}

TextureOptimizer::TextureOptimizer():mbOpenTexture(false),mbOpenPri(false),mbOpenGroundplane(false),mbOpenEdgeAnalysis(false)
{
    
}

std::map<double, double, std::less<double>> TextureOptimizer::GetThetaValueMap(g2o::ellipsoid& e_local, const Matrix3d& calib, std::vector<cv::KeyPoint>& keyPoints, cv::Mat& dtMat, int sample_num)
{
    int TOTAL_NUM = sample_num;
    std::vector<double> thetaLists;
    for (int i = 0; i < TOTAL_NUM; i++)
        thetaLists.push_back(double(i * (M_PI / double(TOTAL_NUM))));
    std::map<double, double, std::less<double>> thetaValueHist;
    vector<cv::KeyPoint> keyPointsFlected;

    for (auto theta : thetaLists)
    {
        g2o::ellipsoid e_local_rot = e_local.rotate_ellipsoid(theta);
        // double pixelDiff_average = g2o::CalculateEllipsoidTextureCost(e_rot, keyPoints, keyPointsDiscri, calib, gray, keyPointsFlected);
        double pixelDiff_average = g2o::CalculateEllipsoidTextureCostEdges(e_local_rot, keyPoints, calib, dtMat, keyPointsFlected);
        thetaValueHist.insert(std::make_pair(theta, pixelDiff_average));
    }

    return thetaValueHist;
}

inline bool compare_valueMap(const std::pair<double, double>&p1, const std::pair<double, double>&p2)
{
    return p1.second < p2.second;
}

std::vector<g2o::ellipsoid*> TextureOptimizer::SampleRotationWithTexture(std::vector<Frame *> &pFrames, Measurements& mms, 
    Objects& objs, Matrix3d& mCalib, int rows, int cols)
{
    std::vector<g2o::ellipsoid*> vpEs;
    if(objs.size() == 0) return vpEs;

    ORB_SLAM2::Object& obj = objs[0];
    g2o::ellipsoid e = *(obj.pEllipsoid);

    // TODO: 尝试输出所有的集合, 然后看看是不是凸的

    std::map<double, double, std::less<double>> thetaValueMapTotal;
    for(auto mea_id:obj.measurementIDs)
    {
        Measurement& mm = mms[mea_id];
        Vector4d& bbox = mm.ob_2d.bbox;
        Frame* pFrame = mm.ob_2d.pFrame;
        cv::Mat rgb = pFrame->rgb_img;
        // opt.optimizeWithTexture(e, bbox, calib, rgb);
        // 重新计算keyPoints

        g2o::ellipsoid e_local = e.transform_from(pFrame->cam_pose_Tcw);
        // 输出该角度的cost! 添加到已有的map中去
        std::map<double, double, std::less<double>> thetaValueMapNew = GetThetaValueMap(e_local, mCalib, mm.ob_2d.keyPoints, mm.ob_2d.dtMat, 90);
        for(auto pair : thetaValueMapNew)
        {
            if(thetaValueMapTotal.find(pair.first)!=thetaValueMapTotal.end())
                thetaValueMapTotal[pair.first] += pair.second;
            else 
                thetaValueMapTotal[pair.first] = pair.second;
        }
    }

    // 已经排列好了，自然直接输出最小的. 但并没有指向具体的角度？ 横坐标就是!
    g2o::ellipsoid e_opt;
    if(thetaValueMapTotal.size()>0)
    {
        // 从小到大排列一下

        auto iter = std::min_element(thetaValueMapTotal.begin(), thetaValueMapTotal.end(), compare_valueMap);

        double theta = iter->first;
        double cost = iter->second;

        e_opt = e.rotate_ellipsoid(theta);

        // 在这里可视化一下结果
        // 一起绘制hist
        cv::Mat histMatTotal = drawXYPlot(thetaValueMapTotal, 0, M_PI);
        // 高亮标记当前rot所在的点。
        histMatTotal = highlightThetaOnPlot(histMatTotal, theta, 0, M_PI, 0, 255);
        cv::imshow("plot TOTAL hist mat", histMatTotal);

        std::cout << "Please check the hist mat total..." << std::endl;
        cv::waitKey();

    }

    vpEs.push_back(new g2o::ellipsoid(e_opt));

    // 保存结果
    mResult.thetaValueMap = thetaValueMapTotal;

    return vpEs;
}

// 该版本使用QuadricSLAM的约束，优化完整自由度的椭球体。
// 默认开关设置： 
//  OpenGround,  CLOSE 
//  OpenPri,    CLOSE
//  OpenTexture,    CLOSE
std::vector<g2o::ellipsoid*> TextureOptimizer::graphOptimize(std::vector<Frame *> &pFrames, Measurements& mms, 
    Objects& objs, Matrix3d& mCalib, int rows, int cols)
{
    // ************************ LOAD CONFIGURATION ************************
    double config_odometry_weight = Config::ReadValue<double>("DEBUG.ODOM.WEIGHT");
    double exp_config_odometry_weight = config_odometry_weight; // normal
    cout << " * DEBUG-Odometry weight : " << exp_config_odometry_weight << " ( 10^" << config_odometry_weight << ")" << endl;

    bool bUseProbThresh = Config::ReadValue<double>("Dynamic.Optimizer.UseProbThresh") > 0.01;
    double config_prob_thresh = Config::ReadValue<double>("Dynamic.Optimizer.EllipsoidProbThresh");

    double config_ellipsoid_2d_scale = Config::ReadValue<double>("Optimizer.Edges.2D.Scale");
    // ************************************************************************
    // Initialize variables.
    int total_frame_number = int(pFrames.size());
    int objects_num = int(objs.size());

    // initialize graph optimization.
    g2o::SparseOptimizer graph;
    g2o::BlockSolverX::LinearSolverType* linearSolver;
    linearSolver = new g2o::LinearSolverDense<g2o::BlockSolverX::PoseMatrixType>();
    g2o::BlockSolverX * solver_ptr = new g2o::BlockSolverX(linearSolver);
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    graph.setAlgorithm(solver);
    graph.setVerbose(true);        // Set output.

    std::map<int, g2o::VertexEllipsoidXYZABCYaw*> vEllipsoidVertexMaps;
    std::map<int, g2o::VertexSE3Expmap*> mSE3Vertex;
    std::vector<g2o::EdgeSE3EllipsoidProj*> edgesValid, edgesInValid, vEdgeSE3Ellipsoid;
    std::vector<g2o::EdgeSE3EllipsoidPlane*> vEdgesSE3EllipsoidPlane;
    std::vector<g2o::EdgeSE3EllipsoidTexture*> edgesTexture;
    std::vector<g2o::EdgeSE3EllipsoidTextureDT*> edgesTextureDT;
    std::vector<EdgePri*> edgesPri;
    std::vector<g2o::EdgeEllipsoidXYZABCYawPlane *> edgesGroundplane;

    // 加入轨迹边
    bool bSLAM_mode = false;
    for( int frame_index=0; frame_index< total_frame_number ; frame_index++) {
        auto pFrame = pFrames[frame_index];
        g2o::SE3Quat curr_cam_pose_Twc = pFrame->cam_pose_Twc;

        g2o::VertexSE3Expmap *vSE3 = new g2o::VertexSE3Expmap();
        vSE3->setId(graph.vertices().size());
        graph.addVertex(vSE3);
        vSE3->setEstimate(pFrame->cam_pose_Tcw); // Tcw
        if(!bSLAM_mode)
            vSE3->setFixed(true);       // Fix all the poses in mapping mode.
        else 
            vSE3->setFixed(frame_index == 0);   
        mSE3Vertex.insert(make_pair(pFrame->frame_seq_id,vSE3));
    }

    // 加入图优化边!! 这些应该与 QuadricSLAM 约束保持一致的
    for(int object_id=0; object_id<objects_num; object_id++ )        
    {
        ORB_SLAM2::Object &obj = objs[object_id];
        int instance = obj.instance_id;
        int label = obj.pEllipsoid->miLabel;

        if(obj.pEllipsoid==NULL)  continue;     // 初始化失败的物体直接跳过去.

        // Add objects vertices
        g2o::VertexEllipsoidXYZABCYaw *vEllipsoid = new g2o::VertexEllipsoidXYZABCYaw();
        vEllipsoid->setEstimate(*obj.pEllipsoid);
        vEllipsoid->setId(graph.vertices().size());
        vEllipsoid->setFixed(false);
        graph.addVertex(vEllipsoid);
        vEllipsoidVertexMaps.insert(make_pair(instance, vEllipsoid)); 

        if(mbOpenTextureDT)
            vEllipsoid->SetRotationOnly(true);

        // 给每一个物体添加地平面约束(注意该Veretex为XYZABCYaw约束)
        // 给每一个物体添加比例约束
        // double weight = Config::ReadValue<double>("SemanticPrior.Weight");
        if(mbOpenPri && !mbOpenTextureDT){
            double weight = 1;
            EdgePri* pPri = GeneratePriEdge(vEllipsoid, label, weight, graph.edges().size());
            graph.addEdge(pPri);
            edgesPri.push_back(pPri);
        }

        // 添加地平面相切约束.
        // 注意: 由于这里是XYZABCYaw的椭球体，角度约束已经内涵
        if(mbOpenGroundplane && !mbOpenTextureDT)
        {
            double weight = 1e4;
            Vector4d ground_normal = mGroundNormal;
            g2o::EdgeEllipsoidXYZABCYawPlane *pEdge = GenerateGroundplaneEdge(vEllipsoid, ground_normal, weight, graph.edges().size());
            graph.addEdge(pEdge);
            edgesGroundplane.push_back(pEdge);
        }

        // 添加 3d 约束 : 来自三维立方体构造时所用的三维平面.
        for( int i=0; i<obj.measurementIDs.size(); i++ )
        {
            Measurement& measurement = mms[obj.measurementIDs[i]];
            int instance = measurement.instance_id;
            
            Observation& ob_2d = measurement.ob_2d;

            Vector4d bbox = ob_2d.bbox;

            int frame_index = ob_2d.pFrame->frame_seq_id;
            auto vEllipsoid = vEllipsoidVertexMaps[instance];  
            auto vSE3 = mSE3Vertex[frame_index];

            g2o::SE3Quat &Twc = measurement.ob_2d.pFrame->cam_pose_Twc;
            
            // DEBUG: 纹理模式时不考虑相切平面!
            if(!mbOpenTextureDT){
                std::vector<g2o::EdgeSE3EllipsoidPlane*> vEdgesPlane = GeneratePlaneConstrainEdges(bbox, Twc, vSE3, vEllipsoid, 
                                mCalib, rows, cols, graph.edges().size());
                for(auto pEdge:vEdgesPlane){
                    graph.addEdge(pEdge);
                    vEdgesSE3EllipsoidPlane.push_back(pEdge);
                }
            }

            // 考虑添加约束边 : ellipsoid, measurement, cv::Image
            //TODO: 如何让每一帧做预处理，包括获得 Gray图像，提取区域内KeyPoints，计算描述子; 需要放到Frame的初始化函数里面.
            // 应该存到Measurement里面!
            if(mbOpenTexture){
                g2o::EdgeSE3EllipsoidTexture* pEdgeTexture = GenerateTextureEdge(vSE3, vEllipsoid, ob_2d, mCalib, graph.edges().size());
                graph.addEdge(pEdgeTexture);
                edgesTexture.push_back(pEdgeTexture);
            }

            if(mbOpenTextureDT){
                g2o::EdgeSE3EllipsoidTextureDT* pEdgeTexture = GenerateTextureEdgeDT(vSE3, vEllipsoid, ob_2d, mCalib, graph.edges().size());
                if(edgesTextureDT.size() < 1){
                    graph.addEdge(pEdgeTexture);
                    edgesTextureDT.push_back(pEdgeTexture);
                }
            }
        }

    } // objs 遍历

    int num_optimize = Config::Get<double>("Optimizer.Optimize.Num");
    // int num_optimize = 5;   // 只优化一次!
    
    // output 
    std::cout << std::endl << " ---- Optimize Quadrics with Texture ---- : " << std::endl;
    cout << " * Object Num : " << objects_num << endl;
    cout<<  " * Vertices: "<<graph.vertices().size()<<endl;
    cout << "   - SE3: " << mSE3Vertex.size() << endl;
    cout << "   - Object: " << vEllipsoidVertexMaps.size() << endl;
    cout << " * Edges: " << graph.edges().size() << endl;

    cout << "   - Ellipsoid-SE3-Planes: " << vEdgesSE3EllipsoidPlane.size() << endl;
    cout << "   - Ellipsoid-Texture: " << edgesTexture.size() << "[" << (mbOpenTexture?"OPEN":"CLOSE") << "]" << endl;
    cout << "   - Ellipsoid-Texture(DT): " << edgesTextureDT.size() << "[" << (mbOpenTextureDT?"OPEN":"CLOSE") << "]" << endl;
    cout << "   --- Valid / Invalid : " << edgesValid.size() << " / " << edgesInValid.size() << endl;

    cout << "   - ObjectPri: " << edgesPri.size() << "[" << (mbOpenPri?"OPEN":"CLOSE") << "]" << endl;
    cout << "   - Groundplane: " << edgesGroundplane.size() << "[" << (mbOpenGroundplane?"OPEN":"CLOSE") << "]" << endl;

    cout << " * Optimization Num: " << num_optimize << endl;
    
    cout << endl;

    // 特殊模式, 做一次误差梯度分析
    if(mbOpenEdgeAnalysis)
    {
        // 考虑第一个物体，并将所有边的信息输出
        RunEdgeAnalysis(vEllipsoidVertexMaps, edgesPri, edgesGroundplane, vEdgesSE3EllipsoidPlane, edgesTextureDT);
    }
    else    // 正常执行优化流程
    {
        // 构造图优化!
        if(graph.edges().size()>0){
            graph.initializeOptimization();
            graph.optimize( num_optimize );  //optimization step

            cout << "Optimization done." << endl;

            // 更新结果到 objs
            bool bUpdateInput = false;

            bUpdateInput = mbOpenTextureDT; // DEBUG: 纹理优化则更新
            if(bUpdateInput){
                for( int i = 0 ; i< objs.size() ; i++)
                {
                    g2o::ellipsoid* pEllipsoid = objs[i].pEllipsoid;
                    if(pEllipsoid!=NULL)
                    {
                        int instance = objs[i].instance_id;
                        if(vEllipsoidVertexMaps.find(instance) == vEllipsoidVertexMaps.end()) continue;
                        auto pEVertex = vEllipsoidVertexMaps[instance];
                        
                        bool colorSet = pEllipsoid->isColorSet();
                        Vector3d old_color;
                        if(colorSet) old_color = pEllipsoid->getColor();
                        (*pEllipsoid) = pEVertex->estimate();
                        pEllipsoid->miInstanceID = instance;
                        if(colorSet) pEllipsoid->setColor(old_color);   // 保持颜色不变.
                    }
                }
            }
        }    
        else 
        {
            std::cout << " No observations valid." << std::endl;
        }

        // 输出优化之后的边的数值
        // 边的类型
        // vEdgesSE3EllipsoidPlane：物体平面观测
        // edgesTexture： 物体纹理约束
        // edgesPri： 物体先验大小
        bool bOpenLogs = false;
        if(bOpenLogs){
            time_t now = time(0);
            tm *ltm = localtime(&now);
            std::string str_time = to_string(ltm->tm_hour) + "_" + to_string(ltm->tm_min) + "_" + to_string(ltm->tm_sec);
            std::string text_name = std::string("edges_erros_") + str_time;
            OutputEdgesErrors(vEdgesSE3EllipsoidPlane, text_name);
            OutputEdgesErrors(edgesTexture, text_name);
            OutputEdgesErrors(edgesPri, text_name);
        }

    }

    std::vector<g2o::ellipsoid*> vpObjs; vpObjs.reserve(vEllipsoidVertexMaps.size());
    for(auto& pair : vEllipsoidVertexMaps)
    {
        g2o::ellipsoid* pE = new g2o::ellipsoid(pair.second->estimate());
        vpObjs.push_back(pE);
    }

    //DEBUG: 测试地平面误差
    if(edgesGroundplane.size()>0)
        std::cout << "Groundplane error : " << edgesGroundplane[0]->error() << std::endl;

    // 存储到 result
    mResult.chi2 = graph.chi2();

    return vpObjs;

}

void TextureOptimizer::ExtractKeyPointsFromBbox(const cv::Mat& gray, const Vector4d& bbox, std::vector<cv::KeyPoint>& keypoints_points, cv::Mat& descri)
{
    // 遍历所有角度，输出一个hist，然后输出最佳角度
    int nfeatures = 50;
    cv::ORB orb(nfeatures);
    vector<cv::KeyPoint> keyPoints;
    cv::Mat bboxMask = GenerateMaskFromBbox(gray.rows, gray.cols, bbox);
    orb(gray, bboxMask, keyPoints);
    // std::cout << "Extract orb points: " << keyPoints.size() << std::endl;

    // 计算原始特征点的描述子
    cv::Mat keyPointsDiscri;
    orb.compute(gray, keyPoints, keyPointsDiscri);

    // 输出
    descri = keyPointsDiscri.clone();
    keypoints_points = keyPoints;

    return;
}

void drawAxesOnImage(cv::Mat& rgbShow, const g2o::ellipsoid& e, const g2o::SE3Quat& campose_cw, const Matrix3d& calib)
{
    Vector2d center = e.projectCenterIntoImagePoint(campose_cw, calib);
    Matrix3d proj_axes = e.projectAxisOnImage(campose_cw, calib, 2);

    for(int i=0;i<proj_axes.cols();i++)
    {
        Vector3d p = proj_axes.col(i);  // u,v,1
        cv::line(rgbShow, cv::Point(center[0],center[1]), cv::Point(p[0],p[1]),cv::Scalar(i==2?255:0,i==1?255:0,i==0?255:0),2);
    }
}

cv::Mat TextureOptimizer::VisualizeResultOnImage(const cv::Mat& im, const g2o::ellipsoid& e,
    const Vector4d& bbox, const Matrix3d& calib, const TextureOptimizerResult& result_)
{
    // 输入 result_ 缺省时，使用上一次的结果
    TextureOptimizerResult result;
    if(result_.empty())
        result = this->GetResult();
    else 
        result = result_;

    cv::Mat rgbShow = im.clone();
    // ************** 可视化部分 **********
    // 绘制一个xy散点图 [ 已经移动到外部 ]
    // cv::Mat plot = drawXYPlot(thetaValueHist, 0, M_PI *2, 0, 255);
    // cv::imshow("plot", plot);

    // 仅仅最后一次才可视化 优化对象
        // ************** 可视化部分 **********
    // 绘制一个xy散点图 [ 已经移动到外部 ]
    // cv::Mat plot = drawXYPlot(thetaValueHist, 0, M_PI *2, 0, 255);
    // cv::imshow("plot", plot);

    // 仅仅最后一次才可视化 优化对象
    // 可视化对称点
    cv::drawKeypoints(rgbShow, result.keyPointsFlected, rgbShow, cv::Scalar(255, 0, 0));
    // 绘制连线关系
    drawKeypointsPairs(rgbShow, rgbShow, result.keyPoints, result.keyPointsFlected, cv::Scalar(0, 255, 0));
    // 可视化椭球体的投影
    Vector5d ellipse = e.projectOntoImageEllipse(g2o::SE3Quat(), calib); // 测试不使用退化椭球体的效果
    drawEllipseOnImage(ellipse, rgbShow, cv::Scalar(255, 0, 0));
    drawAxesOnImage(rgbShow, e, g2o::SE3Quat(), calib);

    // 可视化特征点
    cv::drawKeypoints(rgbShow, result.keyPoints, rgbShow, cv::Scalar(0, 0, 255)); // 详细绘制参数: , cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS
    cv::rectangle(rgbShow, cv::Rect(cv::Point(bbox(0), bbox(1)), cv::Point(bbox(2), bbox(3))), cv::Scalar(0, 255, 0));

    return rgbShow;
}

// 一个函数，传入bbox椭球体图像，输出一个优化之后的更好的结果
// 注意转换到局部坐标系来!!!
g2o::ellipsoid TextureOptimizer::optimizeWithTexture(const g2o::ellipsoid &e_local, const Vector4d &bbox, const Matrix3d &calib, const cv::Mat &im)
{
    cv::Mat gray;
    cv::Mat rgbShow = im.clone();
    cvtColor(im, gray, CV_BGR2GRAY);

    cv::Mat keyPointsDiscri;
    std::vector<cv::KeyPoint> keyPoints;
    ExtractKeyPointsFromBbox(gray, bbox, keyPoints, keyPointsDiscri);

    // DEBUG: 将图像平滑
    // cv::GaussianBlur(gray, gray, cv::Size(29,29), 0, 0);
    cv::Mat dtMat, edges;
    GenerateDtMatFromGray(gray, dtMat, edges);

    cv::Mat dtMatShow;
    cv::normalize(dtMat, dtMatShow, 0, 1., cv::NORM_MINMAX);
    cv::imshow("dtMatShow", dtMatShow);

    int TOTAL_NUM = 90;
    std::vector<double> thetaLists;
    for (int i = 0; i < TOTAL_NUM; i++)
        thetaLists.push_back(double(i * (M_PI / double(TOTAL_NUM))));
    std::map<double, double, std::less<double>> thetaValueHist;
    vector<cv::KeyPoint> keyPointsFlected;
    EllipsoidTex eUpdate;

    for (auto theta : thetaLists)
    {
        g2o::ellipsoid e_local_rot = e_local.rotate_ellipsoid(theta);
        // double pixelDiff_average = g2o::CalculateEllipsoidTextureCost(e_local_rot, keyPoints, keyPointsDiscri, calib, gray, keyPointsFlected);
        double pixelDiff_average = g2o::CalculateEllipsoidTextureCostEdges(e_local_rot, keyPoints, calib, dtMat, keyPointsFlected);
        thetaValueHist.insert(std::make_pair(theta, pixelDiff_average));

        eUpdate = EllipsoidTex(e_local_rot);  // 纯粹为了后面的可视化
    }

    // 存储结果
    TextureOptimizerResult result;
    result.thetaValueMap = thetaValueHist;
    result.keyPoints = keyPoints;
    result.keyPointsFlected = keyPointsFlected;
    mResult = result;

    g2o::ellipsoid eBest; // Tobe filled
    return eBest;
};

// 获得优化过程中的数据
// thetaValueHist: 每个角度对应的Cost值
TextureOptimizerResult TextureOptimizer::GetResult()
{
    return mResult;
}

// data 已经按 x 排序
cv::Mat TextureOptimizer::drawXYPlot(const std::map<double, double> &data, 
        double x_min, double x_max, double y_min, double y_max)
{

    // ---
    cv::Mat im = cv::Mat::zeros(400, 400, CV_8UC3);
    cv::Point pt_last(0, 0);

    // --- 自动获得最大最小值
    static double y_max_param = 0;
    static double y_min_param = INFINITY;
    if (std::abs(y_min - y_max) < 1e-5)
    {
        double value_max = 0;
        double value_min = INFINITY;
        for (auto &pt : data)
        {
            if (pt.second > value_max)
                value_max = pt.second;
            else if (pt.second < value_min)
                value_min = pt.second;
        }

        if (value_max > y_max_param)
            y_max_param = value_max;
        if (value_min < y_min_param)
            y_min_param = value_min;

        y_min_param = 0; // 强制最低为0
    }
    else
    {
        y_max_param = y_max;
        y_min_param = y_min;
    }

    for (auto &pt : data)
    {
        double x = pt.first;
        double y = pt.second;

        double x_norm = (x - x_min) / (x_max - x_min);
        double y_norm = 1 - (y - y_min_param) / (y_max_param - y_min_param);

        cv::Point pt_im(PARAM_HIST_X * x_norm, PARAM_HIST_Y * y_norm);

        cv::line(im, pt_last, pt_im, cv::Scalar(0, 255, 0));

        pt_last = pt_im;
    }

    return im;
};

cv::Mat TextureOptimizer::highlightThetaOnPlot(cv::Mat &in, double theta, 
        double x_min, double x_max, double y_min, double y_max)
{
    // 标记一根竖线吧

    // theta to x
    double x_norm = (theta - x_min) / (x_max - x_min);
    double x = PARAM_HIST_X * x_norm;

    cv::Mat out = in.clone();
    cv::line(out, cv::Point(x, 0), cv::Point(x, PARAM_HIST_Y), cv::Scalar(255, 255, 255));
    return out;
}

void TextureOptimizer::SaveDescriptor(const cv::Mat &mat, const string &name)
{
    if (mat.rows == 0)
        return;
    const static string str_dir = "./logs/";
    Eigen::MatrixXd mat_eigen;
    cv::cv2eigen(mat, mat_eigen);
    saveMatToFile(mat_eigen, (str_dir + name).c_str());
}

void TextureOptimizer::drawEllipseOnImage(const Vector5d &ellipse, 
        cv::Mat &im, const cv::Scalar &color)
{
    // std::cout << "Ellipse from circle : " << ellipse.transpose() << std::endl;
    cv::RotatedRect rotbox2(cv::Point2f(ellipse[0], ellipse[1]), cv::Size2f(ellipse[3] * 2, ellipse[4] * 2), ellipse[2] / M_PI * 180);
    try
    {
        cv::ellipse(im, rotbox2, color);
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << '\n';
    }
    return;
}

cv::Mat TextureOptimizer::GenerateMaskFromBbox(int rows, int cols, const Vector4d &bbox)
{
    int x = round(bbox[0]);
    int y = round(bbox[1]);

    int width = round(std::abs(bbox[2] - bbox[0]));
    int height = round(std::abs(bbox[3] - bbox[1]));

    cv::Mat mask = cv::Mat::zeros(rows, cols, CV_8UC1);
    mask(cv::Rect(x, y, width, height)) = 255; // 将矩形区域设置为255

    return mask;
};

// double TextureOptimizer::sumPixelDiff(const cv::Mat &im, const vector<cv::KeyPoint> &kps1, 
//     const vector<cv::KeyPoint> &kps2)
// {
//     int diff = 0;
//     int num1 = kps1.size();
//     int num2 = kps2.size();

//     int x_max = im.cols;
//     int y_max = im.rows;

//     int valid_num = 0;
//     if (num1 == num2)
//     {
//         for (int i = 0; i < num1; i++)
//         {
//             const cv::KeyPoint &kp1 = kps1[i];
//             const cv::KeyPoint &kp2 = kps2[i];

//             uchar value1 = im.at<uchar>(kp1.pt);
//             uchar value2 = im.at<uchar>(kp2.pt);

//             // TODO: 注意对称点要在图像范围内
//             if (kp1.pt.x > 0 && kp1.pt.x < x_max && kp1.pt.y > 0 && kp1.pt.y < y_max && kp2.pt.x > 0 && kp2.pt.x < x_max && kp2.pt.y > 0 && kp2.pt.y < y_max)
//             {
//                 diff += int(std::abs(value1 - value2));
//                 valid_num++;
//             }
//         }
//     }

//     double average_diff = diff / double(valid_num);
//     return average_diff;
// };

// 请确认第一个和第二个kps一一对应，且数量一致
void TextureOptimizer::drawKeypointsPairs(const cv::Mat &im, cv::Mat &imOut, 
    const std::vector<cv::KeyPoint> &kps1, const std::vector<cv::KeyPoint> &kps2, 
    const cv::Scalar &scalar)
{
    cv::Mat out = im.clone();
    int num1 = kps1.size();
    int num2 = kps2.size();
    if (num1 == num2)
    {
        for (int i = 0; i < num1; i++)
        {
            const cv::KeyPoint &kp1 = kps1[i];
            const cv::KeyPoint &kp2 = kps2[i];

            // 检查有效性
            if (kp1.pt.x > 0 && kp1.pt.y > 0 && kp2.pt.x > 0 && kp2.pt.y > 0)
                cv::line(out, kp1.pt, kp2.pt, scalar);
            else if (kp2.pt.x == -2 && kp2.pt.y == -2) // 发现 -2,-2; 在椭圆内部，但没找到
                cv::circle(out, kp1.pt, 5, cv::Scalar(0, 255, 0));
            else if (kp2.pt.x == -3 && kp2.pt.y == -3) // 发现 -2,-2; 不在椭圆内部，但没找到
                cv::circle(out, kp1.pt, 5, cv::Scalar(255, 255, 255));
        }
    }
    imOut = out.clone();
    return;
};

void TextureOptimizer::SetPri(bool state)
{
    mbOpenPri = state;
}

void TextureOptimizer::SetTexture(bool state)
{
    mbOpenTexture = state;
}

void TextureOptimizer::SetTextureDT(bool state)
{
    mbOpenTextureDT = state;
}

void TextureOptimizer::SetGroundplane(bool state)
{
    mbOpenGroundplane = state;
}

void TextureOptimizer::SetGroundPlaneNormal(const Vector4d& normal)
{
    mGroundNormal = normal;
}

void TextureOptimizer::SetEdgeAnalysis(bool state)
{
    mbOpenEdgeAnalysis = state;
}


void TextureOptimizer::GenerateDtMatFromGray(const cv::Mat& gray, cv::Mat& dtMap, cv::Mat& edges)
{
    int lowThreshold = 20;
    int ratio = 3;
    int kernel_size = 3;
    // ----------
    cv::Mat detected_edges;
    /// 使用 3x3内核降噪
    blur( gray, detected_edges, cv::Size(3,3) );

    /// 运行Canny算子
    cv::Mat edges_not;
    cv::Canny( detected_edges, detected_edges, lowThreshold, lowThreshold*ratio, kernel_size );
    cv::bitwise_not(detected_edges, edges_not);

    cv::Mat dtmap_;
    cv::distanceTransform(edges_not, dtmap_, CV_DIST_L1, 3);
    
    dtMap = dtmap_.clone();
    edges = detected_edges.clone();
}