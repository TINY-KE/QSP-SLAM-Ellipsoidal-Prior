// 说明： 本cpp文件处理 Nonparametric Pose Graph 的数据关联和图优化，
// 在代码结构上，实际是 Optimizer的扩展；函数的声明发生在Optimizer.h文件中.
// 2020-6-3 by LZW

#include "include/core/NonparamDA.h"
#include "Optimizer.h"
#include "include/core/Ellipsoid.h"
#include "include/core/SupportingPlane.h"
#include "include/core/ConstrainPlane.h"

#include "src/config/Config.h"
#include "src/pca/EllipsoidExtractorEdges.h"        // DEBUG : 为了计算平面-物体的 cross.

#include <boost/math/distributions/chi_squared.hpp>

static int g_instance_total_id = 0;       // 宏观记录物体的instance该到哪了.

const double sqrt_2pi_inv = 1 / std::sqrt( 2 * M_PI );

namespace ORB_SLAM2
{

class OneDAResult
{
public:
//            double posterior = label_prob * prob_dis * dp_prior;
    double prob_label;
    double prob_dis;
    double dp_prior;

    double posterior;

    int measure_id;
    int object_id;
};

class DAResult
{
public:
    int measure_id;
    int object_id;
    
    int result_id;
    std::vector<OneDAResult> results;
};

template<typename T>
int findInstanceIndex(T& objs, int instance)
{
    for( int i = 0; i< objs.size() ; i++)
    {
        if (objs[i].instance_id == instance )
            return i;
    }

    return -1;
}

template<typename T>
bool deleteIndex(T& objs, int index)
{
    auto it = objs.begin() + index;
    objs.erase(it);
    return true;
}

// 从 Tracking 当前的存储方式，组合一个 Measurements 出来. 
// 最好是 Tracking 在计算过程中，直接组合出 Measurements. 这意味着对已有算法做重构了.
void InitMeasurements(Measurements &mms, std::vector<Frame *> &pFrames)
{
    // 将 frame 中存储的measure一一连接到 mms 中去.
    // TODO: 考虑同一帧的观测不能关联到相同的物体上.  ( 隐含，同一帧bbox不会出现很近的物体 )

    // CONFIG
    const double config_prob_thresh = 0;    // 暂时不引入这么一个变量，全部都考虑!

    mms.clear();
    for(auto& pF : pFrames){
        Measurements& meas = pF->meas;
        int mea_num = meas.size();

        for( int i = 0; i < mea_num; i++){
            auto &m = meas[i];
            bool c1 = m.ob_3d.pObj!=NULL;
            if(!c1) continue;
            bool c2 = m.ob_3d.pObj->prob > config_prob_thresh;
            if(!c2) continue;
            mms.push_back(m);
        }
    }
    return;
}

// 注意：本函数也会修改measurements
// 1) 设定关联的物体id
// 2) 更新measure_id 为全局索引 ( 原本是帧内的索引 )
void InitObjectsWithMeasurements(Measurements &mms, Objects &objectObservations)
{
    Objects objs;
    for( int k = 0; k < mms.size(); k++)
    {
        Measurement& m = mms[k];
        int instance = m.instance_id;
        int label = m.ob_2d.label;

        if( m.ob_3d.pObj == NULL ) continue;
        // 要求不能为局部观测
        if( m.ob_3d.pObj->bPointModel == true ) continue;

        // 无可关联物体, 初始化一个新物体
        int instance_id = g_instance_total_id++;
        EllipObject ob_new;
        ob_new.instance_id = instance_id;
        ob_new.classVoter[label] = 1;
        ob_new.measurementIDs.push_back(k);

        // 分配新的椭球体内存空间.
        // 此处应该归给地图.
        g2o::SE3Quat frame_pos_Twc = m.ob_2d.pFrame->cam_pose_Twc;
        ellipsoid* pe = new g2o::ellipsoid(m.ob_3d.pObj->transform_from(frame_pos_Twc)); 
        ob_new.pEllipsoid = pe;

        objs.push_back(ob_new);

        m.instance_id = instance_id;
        m.measure_id = k;
    }

    objectObservations = objs;
}

void OutputDataAssociation(Measurements& mms, Objects& objs)
{
    string file_name("data_association.txt");
    ofstream out_obj(file_name.c_str());

    for(int i=0;i<objs.size();i++)
    {
        EllipObject& ob = objs[i]; 
        int ob_num = ob.measurementIDs.size();
        out_obj << "[EllipObject] Instance " << ob.instance_id << ", " << "Observation Num: " << ob_num << std::endl;
        out_obj << " -> MeasurementIDs : ";
        for(int ob_id = 0; ob_id < ob_num; ob_id++)
        {
            int measure_id = ob.measurementIDs[ob_id];
            Measurement& m = mms[measure_id];
            out_obj << measure_id << " ";
        }
        out_obj << std::endl;

        // 保存classVoter
        out_obj << " -> Dominant Label : " << ob.pEllipsoid->miLabel << std::endl;
        out_obj << " -> classVoter( label : num ): ";
        for(auto iter : ob.classVoter )
        {
            out_obj << iter.first << " : " << iter.second << ", ";
        }
        out_obj << std::endl << std::endl;
    }

    out_obj.close();
    std::cout << "Save data association results to " << file_name << std::endl;
}

void UpdateSupportingPlanesInMap(Relations &rls, SupportingPlanes &spls, Map* pMap)
{
    // 其实相当简单. 但是Map 需要留出接口.
    // 即往该地方添加的平面，需要某个选项的同意. 如此无论可视化任何东西，都可以选择了.

    // 添加所有优化后的面
    srand(time(0));
    int spl_num = spls.size();
    for( int i=0;i<spl_num;i++)
    {
        g2o::plane* ppl = spls[i].pPlane;

        // 设置颜色
        double r = rand()%155 / 255.0;
        double g = rand()%155 / 255.0;
        double b = rand()%155 / 255.0;
        if(ppl->color.norm() < 0.01){
            ppl->color = Vector3d(r,g,b);
        }
        pMap->addPlane(ppl, 1);

        // 同时添加所有 relation plane.
        std::set<int>& rl_set = spls[i].vRelation;
        for(auto iter=rl_set.begin();iter!=rl_set.end();iter++)
        {
            int rl_id = *iter;
            g2o::plane* pl_local = rls[rl_id].pPlane;
            g2o::SE3Quat& Twc = rls[rl_id].pFrame->cam_pose_Twc;
            g2o::plane* pl_world = new g2o::plane(Twc*(*pl_local));
            pl_world->color = Vector3d(r,g,b);
            pMap->addPlane(pl_world, 2);
        }
    }


    // 参考 id:
            //     if(menuShowOptimizedSupPlanes)
            //     mpMapDrawer->drawPlanes(1);
            // if(menuShowSupPlanesObservation)
            //     mpMapDrawer->drawPlanes(2);

}

// "book", 73
// "laptop", 63
// "keyboard", 66
// "mouse", 64
// "backpack", 24
// "bottle", 39
// "monitor" 62
Vector3d GenerateColorFromLabel(int label)
{
    // 常见label 手动制定颜色
    switch (label)
    {
    case 73:
        return Vector3d(1,0,0);
    case 63:
        return Vector3d(0.8,0.2,0);
    case 66:
        return Vector3d(0.5,0.5,0);
    case 64:
        return Vector3d(0.2,0.8,0);
    case 24:
        return Vector3d(0,1,0);
    case 39:
        return Vector3d(0,0.8,0.2);
    case 62:
        return Vector3d(1,0.5,0.5);   
    default:
        return Vector3d(0,0,1);
    }
}

void UpdateObjectInMap(Measurements& mms, Objects& objs, Map *pMap)
{
    pMap->ClearEllipsoidsVisual();
    // pMap->ClearEllipsoidsObservation();
    // 按颜色光谱生成每个instance的颜色.
    // Vector3d color1(102,204,0);
    // Vector3d color2(102,204,255);
    srand(time(0));

    int num_obj = objs.size();
    std::cout << "[ Debug : num_obj : " << num_obj << " ]" << std::endl;
    // 可视化最后优化得到的物体
    for(int i=0;i<num_obj;i++)
    {
        EllipObject& ob = objs[i]; 

        // Vector3d color = color1 + double(i)/double(num_obj)*(color2-color1);

        // 若未设置颜色, 则设置随机颜色
        if(!ob.pEllipsoid->isColorSet()){
            // double r = rand()%150 / 255.0;
            // double g = rand()%150 / 255.0;
            // double b = rand()%150 / 255.0;
            // Vector3d color(r,g,b);

            Vector3d color;
            color << 0,0,1.0; // 蓝色

            // Vector3d color = GenerateColorFromLabel(ob.pEllipsoid->miLabel);
            ob.pEllipsoid->setColor(color, 1.0);
        }

        // ----------- 可视化版本１: 所有实例　以及　>0.9　概率的实例 -----------
        // pMap->addEllipsoidObservation(ob.pEllipsoid);

        // if( ob.pEllipsoid->prob > 0.9 ) // 论文中该参数为 10%, 即超过10个有效观测即可.
        //     pMap->addEllipsoidVisual(ob.pEllipsoid);
        // ---------------------------------------------------------------

        // ----------- 可视化版本２：　所有实例　以及　所有观测 -----------------
        pMap->addEllipsoidVisual(ob.pEllipsoid);
        // 绘制观测结果
        // int num_measurements = ob.measurementIDs.size();
        // for( int m =0; m< num_measurements; m++)
        // {
        //     Measurement& mea = mms[ob.measurementIDs[m]];
        //     g2o::ellipsoid* pE_mea = mea.ob_3d.pObj;    // 局部坐标系
        //     g2o::SE3Quat Twc = mea.ob_2d.pFrame->cam_pose_Twc;
        //     g2o::ellipsoid* pE_mea_global = new g2o::ellipsoid(pE_mea->transform_from(Twc));
        //     Vector3d color = ob.pEllipsoid->getColor();
        //     pE_mea_global->setColor(color, 0.3);
        //     pE_mea_global->miInstanceID = mea.measure_id;   // 将measure_id通过MapDrawer可视化出来 
        //     pMap->addEllipsoidObservation(pE_mea_global);
        // }
        // ---------------------------------------------------------------
    }

}

typedef std::pair<int,int> PAIR;
bool cmp_by_value(const PAIR& lhs, const PAIR& rhs) {  
  return lhs.second > rhs.second;  
}  

// 更新 label, prob
void UpdateObjectInformation(Objects& objs)
{
    for(int i=0;i<objs.size();i++)
    {
        EllipObject& ob = objs[i]; 
        if(ob.measurementIDs.size()<=0) continue;

        std::map<int,int>& voter = ob.classVoter;
        std::vector<PAIR> voter_vec(voter.begin(), voter.end());  
        sort(voter_vec.begin(), voter_vec.end(), cmp_by_value);  
        int label = voter_vec[0].first;
        ob.pEllipsoid->miLabel = label;

        // 定义概率
        int num_measurements = ob.measurementIDs.size();
        double obj_prob = MIN(1.0, num_measurements / 10.0);
        ob.pEllipsoid->prob = obj_prob;
    }
}

// 可视化优化的轨迹
void VisualizeOptimizedTraj(Trajectory &camTraj, Map *pMap)
{
    pMap->clearTrajectoryWithName("OptimizedTrajectory");
    for(int i=0;i<camTraj.size();i++){
        pMap->addToTrajectoryWithName(camTraj[i], "OptimizedTrajectory");
    }
}

// 初始化关系
void InitRelations(Relations &rls,  std::vector<Frame *> &pFrames)
{
    Relations rlsOut;
    for(auto pF : pFrames)
    {
        Relations& crls = pF->relations;
        rlsOut.insert(rlsOut.end(), crls.begin(), crls.end());
    }

    rls = rlsOut;

    return;
}

// 更新关联的(支撑)平面
void UpdateAssociationPlanes(Relations &rls, SupportingPlanes &spls)
{
    int rl_num = rls.size();

    // 开始聚类, 仿照 Nonparam 流程. 一一取出再关联.

    // 理应对 Nonparam 做抽象?
    // 做一个求解器.
    for( int k=rl_num-1; k>=0; k--)
    {
        Relation& rl = rls[k];
        int sp_id = findInstanceIndex<SupportingPlanes>(spls, rl.plane_instance_id);

        if(sp_id==-1)
        {
            // 该 relation 尚未被关联.
            std::cout << "Un associated relation, rl_id : " << k << ",  instance id : " << rl.plane_instance_id << std::endl;
            std::cout << "This is imppossible, please check." << std::endl;
            continue;
        }
        SupportingPlane& spl = spls[sp_id];

        // 取出观测
        spl.deleteObservation(rl, k);
        if(spl.empty())
        {
            // 删除该 spl
            deleteIndex<SupportingPlanes>(spls, sp_id);
        }

        // 遍历寻找最近关联
        double prob; int id;
        bool result = SupportingPlane::FindNearestDis(rl, spls, prob, id);
        if(result)
        {
            // std::cout << " Prob : " << prob << std::endl;
            if( prob > 0.0)
                spls[id].addObservation(rl, k);    // 关联完成.
            else 
            {
                SupportingPlane sp(rl, k);
                spls.push_back(sp);
            }
        }
    }

    return;
}

// 初始化有关联的支撑平面
void InitSupportingPlanesWithRelations(Relations& rls, SupportingPlanes& spls_)
{
    SupportingPlanes spls;
    // g2o::plane plGd(groundplaneNormal);
    // 将 Relations 中的所有观测平面. ( 不同的 ) 初始化为一个新平面.
    int rl_num = rls.size();

    for( int i=0;i<rl_num;i++)
    {
        Relation& rl = rls[i];
        // 每个都初始化为新的 spls
        SupportingPlane sp(rl, i);
        spls.push_back(sp);
    }    

    spls_ = spls;
}

// 输出支撑平面
void OutputSupportingPlanes(SupportingPlanes &spls)
{
    ofstream out_obj("./supportingplanes.txt");

    int num = spls.size();
    out_obj << "Total Num : " << num << std::endl;
    for(int i=0;i<num;i++)
    {
        SupportingPlane& sp = spls[i];
        
        // int instance_id;    // 平面的唯一 instance 
        // g2o::plane* pPlane;  // 世界坐标系下的平面表达
        // double height; //  平面相对地面的高度, 用于聚类
        // std::set<int> vRelation;   // 该平面的观测, 以 relation 做索引
        
        // std::set<int> vInstance;       // 位于该平面上的物体 id. 索引 objects
        out_obj << "- instance: " << sp.instance_id << ", planeVec: " << sp.pPlane->param.transpose() << std::endl;
        out_obj << "    height : " << sp.height << std::endl;
        out_obj << "    vRelation: " << sp.vRelation.size() << std::endl;
        out_obj << "    vInstance: " << sp.vInstance.size() << std::endl;
        out_obj << "        [";
        for(auto iter=sp.vInstance.begin();iter!=sp.vInstance.end();iter++)
            out_obj << " " << *iter;
        out_obj << " ] " << std::endl;
        out_obj << std::endl;
    }

    out_obj.close();
    return;
}

// 该函数专门获取平面上支撑的物体
void GetObjectInstanceOnSupportingPlanes(SupportingPlanes& spls, Relations& rls, Measurements& mms, std::vector<Frame *> &pFrames)
{
    int spl_num = spls.size();
    for(int i=0;i<spl_num;i++)
    {
        SupportingPlane& sp = spls[i];
        sp.vInstance.clear();
        for(auto iter=sp.vRelation.begin();iter!=sp.vRelation.end();iter++)
        {
            Relation& rl = rls[*iter];
            int mm_id = GetTotalObjectIndex(pFrames, rl.pFrame->frame_seq_id, rl.obj_id);
            Measurement& m = mms[mm_id];
            int obj_instance = m.instance_id;
            sp.vInstance.insert(obj_instance);
        }
    }
    return;
}

// 更新高度
void UpdateHeight(SupportingPlanes& spls, Vector4d& mGroundPlaneNormal)
{
    g2o::plane pl_gd(mGroundPlaneNormal);
    for(auto& sp: spls)
    {
        
        g2o::plane* ppl = sp.pPlane;
        double dis = ppl->distanceToPlane(pl_gd);
        sp.height = dis;
    }

    return;
}

// 注意目前所有的 ConstrainPlane 都已经存储在了 obj 下的 ellipsoid 里面.
// 这些平面在世界坐标系下. 直接计算即可 已经与 measurement 无关!
void UpdateConstrainPlaneState(Measurements& mms, Objects& objs)
{
    int count_cross = 0;
    int count_not_cross =0;

    int obj_num = objs.size();
    for(int i=0;i<obj_num;i++)
    {
        EllipObject &obj = objs[i];
        int mea_num = obj.measurementIDs.size();

        g2o::ellipsoid& e_w = *obj.pEllipsoid;
        std::vector<ConstrainPlane*> cplanes = e_w.mvCPlanesWorld;
        // 一一判断平面与 objs 之间的关系
        for(auto iter_plane = cplanes.begin();iter_plane!=cplanes.end();iter_plane++)
        {
            ConstrainPlane* pCPlane = *iter_plane;
            if(pCPlane->valid) continue;

            g2o::plane pl(pCPlane->pPlane->param);

            // Vector4d param = pCPlane->pPlane->param; // 局部系
            // g2o::plane pl(param); pl.transform(Twc);

            // 世界系平面参数param_w 与世界系物体e_w 的关系判断
            bool bCross = JudgeCross(pl, e_w);

            if(bCross == true) {
                pCPlane->state = 1;
                count_cross++;
            }
            else {
                pCPlane->state = 2;
                count_not_cross++;
            }
        }
        
    }

    std::cout << " ==> cross : " << count_cross << ", not cross " << count_not_cross << std::endl;
}

void UpdateWorldConstrainPlanesForObjects(Objects& objs, Measurements& mms)
{
    // ************************************************
    // [可视化专用] 基于优化后的新位姿更新世界 vwcPlanes
    // ************************************************
    for( int i = 0 ; i< objs.size() ; i++)
    {
        EllipObject& obj = objs[i];
        g2o::ellipsoid* pEllipsoid = obj.pEllipsoid;
        if(pEllipsoid==NULL) continue;
        std::vector<ConstrainPlane*> vCPlanesWorld;
        for( int i=0; i<obj.measurementIDs.size(); i++ )
        {
            Measurement& measurement = mms[obj.measurementIDs[i]];
            
            Observation3D& ob_3d = measurement.ob_3d;
            g2o::ellipsoid* pObj_ob = ob_3d.pObj;
                            
            std::vector<g2o::ConstrainPlane*> vCPlanes = pObj_ob->mvCPlanes;
            // MatrixXd mPlanesParam = pObj_ob->cplanes;
            int plane_num = vCPlanes.size();

            g2o::SE3Quat &Twc = measurement.ob_2d.pFrame->cam_pose_Twc;
            for( int i=0;i<plane_num;i++)
            {
                g2o::ConstrainPlane* pCPlane = vCPlanes[i];
                Vector4d planeVec = pCPlane->pPlane->param.head(4); // local coordinate
                {
                    g2o::plane* ppl = new g2o::plane(planeVec); ppl->transform(Twc);
                    g2o::ConstrainPlane* pcpl = new g2o::ConstrainPlane(ppl);
                    pcpl->valid = pCPlane->valid;   // 保留 valid 信息，仅仅在 Mapdrawer中用做可视化。仅仅在 Mapdrawer中用做可视化。
                    pcpl->type = pCPlane->type; // 保留 type
                    vCPlanesWorld.push_back(pcpl);
                }
            }
        }
        pEllipsoid->mvCPlanesWorld = vCPlanesWorld; // 配置世界系下的约束平面; 
    }

    UpdateConstrainPlaneState(mms, objs);   // 添加状态以做可视化

    std::cout << "Finish updating mvCPlanesWorld using newest camera poses." << std::endl;
    return;
    // ************************************************
    // ************************************************
}

// // 该函数参照论文 IROS17, 使用基本的点模型作为物体模型.
// void Optimizer::GlobalObjectGraphOptimizationWithPDAPointModel(std::vector<Frame *> &pFrames, Map *pMap)
// {
//     int config_iter_num = Config::Get<double>("Optimizer.NonparametricDA.Num");
//     std::cout << "Run iterations : " << config_iter_num << std::endl;

//     // 初始化 mms, objs
//     Measurements mms; Objects objs; 
    
//     // 将Frames中满足条件的measurements全部提取出来组成一个大measurements
//     // 1) 3d检测非NULL   2) 观测满足概率阈值 (当前为0)
//     InitMeasurements(mms, pFrames);  
//     InitObjectsWithMeasurements(mms, objs); // 给每个measurement 初始化一个 obj

//     // 优化得到的轨迹
//     Trajectory camTraj; camTraj.resize(pFrames.size());
//     for( int iter_id = 0; iter_id < config_iter_num; iter_id ++ )
//     {
//         // 临时debug
//         pMap->DeletePointCloudList("Debug.DistancePoint", 1);

//         // step 1: estimate data association
//         // 存储方式: objectObservations   按 instance->obs 的方式存储了各观测
//         // 修改数据关联后，应该重新排列组成 objectObservations 结构.

//         // 基于当前里程计数据，帧内观测数据，为各物体配置数据关联.
//         std::cout << " [Optimizer.cpp] Begin calculating the data association." << std::endl;
//         UpdateDataAssociation(mms, objs, 1);   // 直接以新关联方式覆盖旧的.

//         // step 2: optimize as usual
//         // 基于新计算的数据关联结果，优化获得位姿估计，地图中路标.
//         std::cout << " [Optimizer.cpp] Begin optimizing the trajectory and objects." << std::endl;

//         // 新版本优化: 使用多平面相切约束
//         OptimizeWithDataAssociationUsingMultiplanesPointModel(pFrames, mms, objs, camTraj);
//     }
    
//     // 根据 ClassVoters 更新物体的Label.
//     UpdateObjectInformation(objs);

//     // 输出数据关联情况
//     OutputDataAssociation(mms, objs);

//     // 将优化结果得到的物体存放入地图中可视化.
//     UpdateObjectInMap(mms, objs, pMap);

//     // 可视化轨迹
//     VisualizeOptimizedTraj(camTraj, pMap);

//     // 保存本次结果
//     mObjects = objs;
//     mMeasurements = mms;
// }


void StaticOutput(Measurements& mms)
{
    int count_total = mms.size();
    int count_valid = 0;

    int mm_num = mms.size();
    for( int i=0;i<mm_num;i++)
    {
        Measurement& mm = mms[i];
        double prob_3d = mm.ob_3d.pObj->prob_3d;
        if(prob_3d>0.5)
            count_valid++;
    }
    double percent = count_valid/double(count_total) * 100;
    std::cout << " ******** Valid Count IoU > 0.5 ********" << std::endl;
    std::cout << " Valid / Total : " << count_valid << " / " << count_total << "(" << percent << "%)" << std::endl;
}

// [ Copy from : EllipsoidExtractor::calibRotMatAccordingToGroundPlane ]
Eigen::Matrix3d calibRotMatAccordingToGroundPlane(Matrix3d& rotMat, const Vector3d& normal){
    // in order to apply a small rotation to align the z axis of the object and the normal vector of the groundplane,
    // we need calculate the rotation axis and its angle.

    // first get the rotation axis
    Vector3d ellipsoid_zAxis = rotMat.col(2);
    Vector3d rot_axis = ellipsoid_zAxis.cross(normal); 
    if(rot_axis.norm()>0)
        rot_axis.normalize();

    // then get the angle between the normal of the groundplane and the z axis of the object
    double norm1 = normal.norm();
    double norm2 = ellipsoid_zAxis.norm();
    double vec_dot = normal.transpose() * ellipsoid_zAxis;
    double cos_theta = vec_dot/norm1/norm2;

    // 防止过界:
    cos_theta = cos_theta>1?1:cos_theta;
    cos_theta = cos_theta<-1?-1:cos_theta;

    double theta = acos(cos_theta);     

    // generate the rotation vector
    AngleAxisd rot_angleAxis(theta,rot_axis);

    Matrix3d rotMat_calibrated = rot_angleAxis * rotMat;

    return rotMat_calibrated;
}


void AlignObjectsToSupportingPlane(Measurements& mms, Objects& objs, Vector4d& groundplane, Map* pMap)
{
    std::cout << "Align Objects to Supporting plane ... " << std::endl;
    // 1) 全部初始化时的可视化

    // 根据 ClassVoters 更新物体的Label,概率
    UpdateObjectInformation(objs);
    // 将优化结果得到的物体存放入地图中可视化.
    // 即更新 Visual(优化后的物体), Observation(所有观测物体)
    // UpdateObjectInMap(mms, objs, pMap);

    // std::cout << "visualizing ... press key to continue. " << std::endl;
    // getchar();

    // 2) 对齐
    int obj_num = objs.size();
    std::cout << "GroundPlane Normal : " << groundplane.head(3).transpose() << std::endl;
    for(int i=0;i<obj_num;i++)
    {
        EllipObject& obj = objs[i];
        g2o::ellipsoid * pE = obj.pEllipsoid;
        Eigen::Quaterniond quat = pE->pose.rotation();    // 世界系的旋转
        Matrix3d rotMat = quat.toRotationMatrix();

        // ground_normal: 世界系的重力 (Z轴) 方向
        Matrix3d rotMatCalib = calibRotMatAccordingToGroundPlane(rotMat, groundplane.head(3));

        // 如何做一个右乘旋转，使其z轴对齐?
        Eigen::Quaterniond quatCalib(rotMatCalib);
        // std::cout << "--------------> old rotMat: " << rotMat << std::endl;
        // std::cout << "--------------> new rot: " << quatCalib.coeffs().transpose() << std::endl;
        pE->pose.setRotation(quatCalib);
    }
    // std::cout << "Calib done, press any key to see the visualization ... " << std::endl;
    // getchar();

    // 3) 再一次可视化
    // UpdateObjectInMap(mms, objs, pMap);
    // std::cout << "visualizing ... press key to continue. " << std::endl;
    // getchar();
}

// KEY: [Optimization]使用 概率数据关联 进行 全局图优化
void Optimizer::GlobalObjectGraphOptimizationWithPDA(std::vector<Frame*> &pFrames, Map *pMap, const Matrix3d& calib, int iRows, int iCols)
{
    int config_iter_num = Config::Get<double>("Optimizer.NonparametricDA.Num");
    std::cout << "================ Run iterations : " << config_iter_num << " ================" << std::endl;

    // 初始化 mms, objs
    Measurements mms; Objects objs;
    
    InitMeasurements(mms, pFrames);  // 旧存储方式的转换

    InitObjectsWithMeasurements(mms, objs); // 给每个measurement 初始化一个 obj

    // 考虑到局部初始化的椭球体，z轴不一定与世界平面对齐，该函数将其对齐.
    if(mbGroundPlaneSet)
        AlignObjectsToSupportingPlane(mms, objs, mGroundPlaneNormal, pMap);

    // 对Relations完成数据关联
    // 注意 Relations/SupportingPlanes 的关系与 Measurements/Objects 正是一样的.
    Relations rls; SupportingPlanes spls;
    InitRelations(rls, pFrames);
    InitSupportingPlanesWithRelations(rls, spls); 

    // 优化得到的轨迹
    Trajectory camTraj; camTraj.resize(pFrames.size());

    for( int iter_id = 0; iter_id < config_iter_num; iter_id ++ )
    {
        std::cout << "================================" << std::endl;
        std::cout << " Nonparam DA iterations : " << iter_id << std::endl;
        std::cout << "================================" << std::endl;

        // 临时debug. done. 不再需要.
        // pMap->DeletePointCloudList("Debug.DistancePoint", 1);

        // Step 1: estimate data association
        // 基于当前轨迹数据，和所有观测数据，为各物体配置数据关联. (三维空间的聚类)
        std::cout << "\n [Optimizer.cpp] Begin calculating the data association." << std::endl;
        clock_t da_begin = clock();
        UpdateDataAssociation(mms, objs);   // 直接以新关联方式覆盖旧的.
        clock_t da_end = clock();
        std::cout << " - DA Time : " << double(da_end-da_begin)/CLOCKS_PER_SEC << std::endl;

        // Step 2: optimize as usual
        // 基于新计算的数据关联结果，优化获得位姿估计，地图中路标.
        std::cout << " [Optimizer.cpp] Begin optimizing the trajectory and objects." << std::endl;

        // 全局关联支撑关系, 获得聚类的平面
        UpdateAssociationPlanes(rls, spls);
        // 更新关联后的变量 sp.vInstance 
        GetObjectInstanceOnSupportingPlanes(spls, rls, mms, pFrames);
        this->LoadRelations(rls, spls); // 将关系加载到优化器, 在优化时将载入.

        

        // 新版本优化: 使用多平面相切约束
        OptimizeWithDataAssociationUsingMultiplanes(pFrames, mms, objs, camTraj, calib, iRows, iCols);

        // 包含平面关系的优化
        // OptimizeWithDataAssociationUsingMultiplanesWithRelations(pFrames, rows, cols, mCalib, mms, objs, camTraj, rls);
        
        // 测试： 每次迭代都可视化

        // 根据 ClassVoters 更新物体的Label,概率
        UpdateObjectInformation(objs);
        // 将优化结果得到的物体存放入地图中可视化.
        // 即更新 Visual(优化后的物体), Observation(所有观测物体)
        UpdateObjectInMap(mms, objs, pMap);

        // std::cout << "iter : " << iter_id << " / " << config_iter_num << std::endl;
        // std::cout << "Wait for Enter Key to next iteration ... " << std::endl;
        // getchar();

    }

    // 调试信息输出    

    // 关系平面的信息输出
    // debug :: 输出 plane relations
    UpdateHeight(spls, mGroundPlaneNormal);
    // OutputSupportingPlanes(spls);
    UpdateSupportingPlanesInMap(rls, spls, pMap);

    // 输出数据关联情况
    // OutputDataAssociation(mms, objs);

    // 可视化轨迹
    VisualizeOptimizedTraj(camTraj, pMap);

    // 更新状态关系:  仅仅用作可视化目的.
    // 将世界切平面的位置按优化结果做更新； 同时还更新世界切平面的状态 (局部/内部...)
    UpdateWorldConstrainPlanesForObjects(objs, mms);

    // 统计输出观测中 prob_3d的情况
    // StaticOutput(mms);

    // 保存本次结果
    mObjects = objs;
    mMeasurements = mms;
}

MatrixXd TransformPlanes(MatrixXd& planeMat, g2o::SE3Quat& T)
{
    //  存储 空间切平面
    //  注意mPlanesParam中的平面是在世界坐标系下, 此处应该转到相机的局部系下再处理.
    MatrixXd pMT; pMT.resize(0, 4);
    int pNum = planeMat.rows();
    for( int i=0; i<pNum; i++)
    {
        g2o::plane pl(planeMat.row(i).head(4));
        pl.transform(T);
        VectorXd plvt = pl.param;
        addVecToMatirx(pMT, plvt);
    }
    return pMT;
}

/* *****
    拷贝版本: EllipsoidExtractorEdges.cpp
* *****/
double distanceFromPlaneToEllipsoid(g2o::plane& pl, g2o::ellipsoid& e)
{
    g2o::SE3Quat Tew = e.pose.inverse();

    g2o::ellipsoid e_e = e.transform_from(Tew);
    g2o::plane ple = pl; ple.transform(Tew);

    // AX+BY+CZ+D = 0
    Vector4d plane_e = ple.param;
    double A = plane_e[0];
    double B = plane_e[1];
    double C = plane_e[2];
    double D = plane_e[3];

    // 获得椭球体参数向量形式
    // ax^2+bxy+c^..... = 0;
    Matrix4d Q_star = e_e.generateQuadric();
    Matrix4d Q = Q_star.inverse();
    
    // 获得系数
    // ax^2+by^2+cz^2+dxy+eyz+fxz+gx+hy+iz+j=0
    // 正椭球体 check
    // x^2/a^2 + y^2/b^2 + z^2/c^2 = 1
    Q = Q / (-Q(3,3));

    // 对角线前3个 必须大于0, 其他项必须为0
    // std::cout << "Origin Q check : " << std::endl << Q << std::endl;

    double a_2 = 1/Q(0,0);
    double b_2 = 1/Q(1,1);
    double c_2 = 1/Q(2,2);

    double alpha_2 = 4 / (A*A*a_2+B*B*b_2+C*C*c_2);
    double alpha_pos = sqrt(alpha_2);

    double x_coeff = A*a_2/2;
    double y_coeff = B*b_2/2;
    double z_coeff = C*c_2/2;
    Vector3d coeff_vec(x_coeff, y_coeff, z_coeff);

    Vector3d extrema_1 = alpha_pos * coeff_vec;
    Vector3d extrema_2 = -extrema_1;

    double dis1 = ple.distanceToPoint(extrema_1);
    double dis2 = ple.distanceToPoint(extrema_2);

    double min_dis = std::min( std::abs(dis1), std::abs(dis2) );

    return min_dis;
}

// 该数据关联将考虑局部观测的包含关系，引入 EIoU.
// 即 observation 可以包含在 Measure 内部.( 即局部观测 )

// // iou 的计算显然应该先对齐. 然后去调用已有库. (注意库似乎是有些问题的)
// double calculateAssociationProbabilityUsingEllipsoid(g2o::ellipsoid& e_mea, g2o::ellipsoid& e_ob)
// {
//     // 1: IoU
//     // double IoU = e_mea.calculateMIoU(e_ob);
//     // double error_iou = 1 - IoU;

//     // 2: center dis
//     // 计算与已有物体三维空间的距离 sq_dist
//     // g2o::SE3Quat obj_global_ellipsoid_Twe = e_mea.pose;
//     // double dis = (e_ob.pose.inverse() * obj_global_ellipsoid_Twe).translation().norm();
//     // double prob = exp(-4*dis);

//     // 3: 切平面 : 已经转换到世界坐标系下.
//     std::vector<g2o::ConstrainPlane*> vCPlanes = e_mea.mvCPlanes;
//     // MatrixXd cplanes = e_mea.cplanes; 
//     int num = vCPlanes.size();
//     // Matrix4d Q_star = e_ob.generateQuadric();

//     double total_plane_error = 0;
//     for( int i=0; i<num; i++)
//     {
//         Vector4d param = vCPlanes[i]->pPlane->param.head(4);
//         // double error = param.transpose() * Q_star * param;
//         g2o::plane pl(param);
//         double dis = distanceFromPlaneToEllipsoid(pl, e_ob);

//         // error_sigma = 1% size.
//         // double center_dis = pl.distanceToPoint(e_ob.pose.translation());
//         // double size_length = center_dis - dis;

//         total_plane_error += dis;
//     }
//     double average_plane_error = total_plane_error / num;
//     // 误差转概率
//     double sigma_dis = Config::Get<double>("DataAssociation.PlaneError.DisSigma");    // 1cm
//     double prob = exp( - 1.0/sigma_dis/sigma_dis/2.0 * average_plane_error * average_plane_error);
//     return prob;
// }

// // 数据关联的核心函数2： 使用 ellipsoid 的 3d error 作为 cost.
// // 注意输出 : prob 概率
// double calculateAssociationProb3DError(g2o::ellipsoid& e_mea, g2o::ellipsoid& e_ob)
// {
//     // Vector9d vec_9dof = e_mea.min_log_error_9dof(e_ob);
//     // Vector9d inv_sigma; inv_sigma.fill(1);
//     // Matrix9d info = inv_sigma.asDiagonal();
//     // double chi2 = vec_9dof.dot(info * vec_9dof);    // 卡方统计量..?
//     // 如何将 chi2 转化为概率???????
//     // return chi2;    // 先tm试试看
//     double prob = e_mea.minus3d(e_ob);
//     return prob;
// }

// 数据关联的核心函数1： 使用切平面计算 cost.
// 新版本函数, 使用单个切平面做判例
// 最终还会判断该切平面是否"真的"属于边缘平面
// [注意这里传入的世界系下的ConstrainPlane是拷贝获得的，不影响原始数据]
double calculateAssociationProbabilityUsingEllipsoid(g2o::ellipsoid& e_mea, g2o::ellipsoid& e_ob)
{
    // 切平面 : 已经转换到世界坐标系下.
    std::vector<g2o::ConstrainPlane*> vCPlanes = e_mea.mvCPlanes;
    int num = vCPlanes.size();
    int valid_num = 0;

    double total_plane_error = 0;
    for( int i=0; i<num; i++)
    {
        // 依次判断每个切平面的距离
        ConstrainPlane* pCPlane = vCPlanes[i];
        Vector4d param = pCPlane->pPlane->param.head(4);

        g2o::plane pl(param);
        double dis = distanceFromPlaneToEllipsoid(pl, e_ob);

        // 对属于图像边界的平面做进一步阈值判断.
        bool bImageBorder = pCPlane->image_border;
        bool bValid = true;
        if(bImageBorder)
        {
            if( dis > 0.1)
            {
                bValid = false;
            }
        }

        if(bValid)
        {
            // 考虑非边界平面
            total_plane_error += dis;
            valid_num++;
        }

        // 该距离理应跟物体大小有关系. 由易到难先设置固定值吧. 1cm
    }

    // 判断未在边界的数量, 至少4个.
    // 即 2 depth bbox + 2 image bbox
    // 这里不应该加判断，而是在前面，若观测有效，则边界不能超过两个。
    // if(valid_num >= 4)
    {
        double average_plane_error = total_plane_error / double(valid_num);

        // 误差转概率
        double sigma_dis = Config::ReadValue<double>("DataAssociation.PlaneError.DisSigma");    // 1cm
        double prob = sqrt_2pi_inv * exp( - 1.0/sigma_dis/sigma_dis/2.0 * average_plane_error * average_plane_error);

        return prob;
    }

}

std::vector<g2o::ConstrainPlane*> TransformAndGenerateConstrainPlanes(std::vector<g2o::ConstrainPlane*> &vCPlanes, g2o::SE3Quat& Twc)
{
    int num = vCPlanes.size();
    std::vector<g2o::ConstrainPlane*> vCPlanesOut;
    vCPlanesOut.resize(num);
    for( int i=0;i<num;i++)
    {
        g2o::ConstrainPlane* pCPlane = vCPlanes[i];
        g2o::plane* pPlane = pCPlane->pPlane;
        g2o::plane* pPlaneWorld = new g2o::plane(*pPlane); pPlaneWorld->transform(Twc);
        g2o::ConstrainPlane* pCPlaneWorld = new g2o::ConstrainPlane(pPlaneWorld);
        vCPlanesOut[i] = pCPlaneWorld;
    }
    return vCPlanesOut;
}

double calculateAssociationProbabilityUsingPoint(Measurement& m, EllipObject& ob){
    
    g2o::SE3Quat& Twc = m.ob_2d.pFrame->cam_pose_Twc;
    g2o::ellipsoid e_measure_global = m.ob_3d.pObj->transform_from(Twc);

    double dis = (e_measure_global.pose.translation() - ob.pEllipsoid->pose.translation()).norm();
    double sq_dis = dis * dis;

    double sigma_dis = Config::ReadValue<double>("DataAssociation.PlaneError.DisSigma");    // 与平面方差共用
    // default : sigma_dis = 0.353   
    double prob = exp(-sq_dis/sigma_dis/sigma_dis/2.0);
    return prob;
}

// 卡方分布的分布函数
double chi2cdf_(int degree, double chi)
{
    boost::math::chi_squared mydist(degree);
    double p = boost::math::cdf(mydist,chi);
    return p;
}

// 椭球体
double EllipsoidDiffProb(g2o::ellipsoid& e_mea, g2o::ellipsoid& e_ob)
{
    Vector9d vec_9 = e_mea.min_log_error_9dof(e_ob);
    Vector7d vec_7;
    vec_7 << vec_9[3], vec_9[4], vec_9[5], vec_9.head(3).norm(), vec_9.tail(3);

    Vector7d inv_sigma;
    inv_sigma << 1,1,1,1,1,1,1;
    Matrix<double,7,7> info = inv_sigma.cwiseProduct(inv_sigma).asDiagonal();

    double chi2 = vec_7.dot(info*vec_7);

    // 7 自由度的卡方分布.
    double prob = chi2cdf_(7, chi2);

    return prob;
}

double calculateAssociationProbability(Measurement& m, EllipObject& ob){
    
    g2o::SE3Quat& Twc = m.ob_2d.pFrame->cam_pose_Twc;

    g2o::ellipsoid e_measure_global = m.ob_3d.pObj->transform_from(Twc);

    // 将切平面转换到世界坐标系下
    std::vector<g2o::ConstrainPlane*> vCPlanes = e_measure_global.mvCPlanes;
    std::vector<g2o::ConstrainPlane*> vCPlanesWorld = TransformAndGenerateConstrainPlanes(vCPlanes, Twc);
    // MatrixXd pMT = e_measure_global.cplanes;
    // MatrixXd pMTWorld = TransformPlanes(pMT, Twc);

    e_measure_global.mvCPlanes = vCPlanesWorld;

    // 注意: 平面方差与三维距离方差是一样的. 
    // double prob_cplanes = calculateAssociationProbabilityUsingEllipsoid(e_measure_global, *ob.pEllipsoid);

    // 测试 3d 中心的数据关联.
    double prob_center = calculateAssociationProbabilityUsingPoint(m, ob);
    // double prob = prob_cplanes * prob_center;

    // 测试使用 3d error 的数据关联
    // double prob = EllipsoidDiffProb(e_measure_global, *ob.pEllipsoid);

    double prob = prob_center;
    
    return prob;
}

double dirichlet(std::map<int,int>& classVoter, int total_num, int label)
{
    auto iter = classVoter.find(label);

    int label_num = 0;
    if( iter != classVoter.end() ){
        label_num = classVoter[label];
    }

    label_num += 1; // at least one ob
    double prob = double(label_num) / (total_num+1);

    return prob;
}

void OutputComplexDAResult(std::vector<DAResult>& daResults, Objects& objs)
{
    ofstream out_obj("./complex_daresult.txt");

    int result_num = daResults.size();

    for( int i=0; i< result_num; i++)
    {
        DAResult& r = daResults[i];

        int num_result = r.results.size();
        OneDAResult& bestR = r.results[r.result_id];
            // int measure_id;
            // int object_id;
            // int result_id;
        int best_obj_id = bestR.object_id;
        int best_ins = objs[best_obj_id].instance_id;

        char str[1000];
        sprintf(str, "[Mesure %d : Result %d , EllipObject %d] [ResultNum %d] \n", r.measure_id, r.result_id, best_ins, num_result);
        out_obj << str;
        sprintf(str, "  -> Highest (OBins %d) : P(%.2f) = DP(%.2f) * Label(%.2f) * Dis(%.8f) \n", best_ins, bestR.posterior, bestR.dp_prior, bestR.prob_label, bestR.prob_dis);
        out_obj << str;
        for( int n = 0; n<num_result; n ++)
        {
            OneDAResult& cR = r.results[n];
            EllipObject& ob = objs[cR.object_id];
            int ins = ob.instance_id;
            sprintf(str, "    --> (OBins %d) : P(%.2f) = DP(%.2f) * Label(%.2f) * Dis(%.8f) \n", ins, cR.posterior, cR.dp_prior, cR.prob_label, cR.prob_dis);
            out_obj << str;
        }
    }
    out_obj.close();

    return;
}

void CheckValidBorder(EllipObject& ob, Measurement& m)
{
    g2o::ellipsoid& e_mea = *m.ob_3d.pObj;
    g2o::ellipsoid& e_ob = *ob.pEllipsoid;
    std::vector<g2o::ConstrainPlane*> vCPlanes = e_mea.mvCPlanes;   // 局部坐标系
    
    int num = vCPlanes.size();
    for( int i=0; i<num; i++)
    {
        ConstrainPlane* pCPlane = vCPlanes[i];
        bool bImageBorder = pCPlane->image_border;
        if(!bImageBorder) continue; // 只检查位于边界的

        // 依次判断每个切平面的距离
        g2o::plane pl_w = *pCPlane->pPlane;
        pl_w.transform(m.ob_2d.pFrame->cam_pose_Twc);
        double dis = distanceFromPlaneToEllipsoid(pl_w, e_ob);

        // 对属于图像边界的平面做进一步阈值判断.
        if( dis > 0.1)  // 距离太大.
        {
            // 标记
            pCPlane->association_border = false;
            pCPlane->valid = false;
        }
        else
        {
            // 之前无效的是否也可以变得有效??? 当然是可以的.
            // if(!pCPlane->valid)
            // {
            //     std::cout << " One observation WANTS to become valid!" << std::endl;
            // }
            pCPlane->association_border = true;
            pCPlane->valid = true;
        }
    }    
}

// 该函数即 Matlab 中 DPSample 函数
// Input: measurements    // 所有的观测，不会删除，以 index 索引.
// Input: objs              // 初始的时候，每个 mms 生成了一个objs.
// Input: model, 0 Ellipsoid, 1 Point
// Output: instanceObs
void Optimizer::UpdateDataAssociation(Measurements& mms, Objects& objs, int model)
{
    double CONFIG_DP_alpha = Config::Get<double>("DataAssociation.DPAlpha");   // 默认值
    // ********************************

    std::map<int, Observations> output;  // 函数最后的计算结果，注意该存储方式本身就以 instance 为中心了. 
    int total_meas_num = mms.size();
    std::cout << "total_meas_num:" << total_meas_num << std::endl;

    // ********************************
    // 该部分变量生存周期： 每次迭代都进行访问
    // ********************************

    // 一些规则
    // 1) instance 即决定了在 objs 中的存放顺序. 由于该instance可能动态增删，实际会变动，所以不能直接这样.
    
    // 将 mms 从后往前遍历
    std::vector<DAResult> daResults;
    for( int k = mms.size()-1; k >= 0; k--)
    {
        Measurement& m = mms[k];
        DAResult daResult;

        if(m.ob_3d.pObj==NULL) continue;

        int instance = m.instance_id;
        int label = m.ob_2d.label;
        bool bPointModel = m.ob_3d.pObj->bPointModel;

        // 从对应物体列表取出该观测
        int index = findInstanceIndex<Objects>(objs, instance); 
        // assert( index >= 0 && "Can't find the instance in Objects.");

        // 若与已有物体已经关联上，先取消该关联.
        if(index >= 0 ) {
            EllipObject& ob = objs[index];
            ob.classVoter[label]--; // 该类别投票器减一
            if((ob.measurementIDs.size()-1)<=0)   // 若减去该观测后，没有有效观测了
            {
                bool result = deleteIndex<Objects>(objs, index);    // 删除该 instance, 以 index 为索引
                assert( result && "Delete an unexisted index.");
            }
            else 
            {
                // 否则，在ob的存储中去除该观测id
                std::vector<int>::iterator iter=std::find(ob.measurementIDs.begin(),ob.measurementIDs.end(),k);
                ob.measurementIDs.erase(iter);
            }
        }

        // ************************
        // 基于观测内容计算其关联概率： 椭球体参数(位置/大小/旋转)，二维检测框，label标签
        // ************************
        // 计算该 measurement 的空间位置
        g2o::SE3Quat frame_pos_Twc = m.ob_3d.pFrame->cam_pose_Twc;   // 此处 frame_pos 将随着优化而变化

        // 依次计算与现有objects的关联概率.
        std::vector<double> vec_posterior;
        for(int obj_id = 0; obj_id < objs.size(); obj_id++)
        {
            EllipObject& ob_rh = objs[obj_id];
            // 首先获得，每个物体，在观测label上投票器内的已有观测数量  dp_prior

            int ob_num = ob_rh.measurementIDs.size();

            // dp_prior :   m_i / (m_total + alpha),
            //      由于分母部分所有物体都是一样，可以省去，该先验只与 m_i 即观测数量有关.
            // double dp_prior = double(ob_num+1) / (double(total_meas_num+1) + config_real_dp_alpha); 
            // 5-27更新: 分母部分对于所有比较对象都是常数，可以不予考虑。于是将其去掉。
            // double dp_prior = double(ob_num+1);

            // label_prob : 似然, 物体检测的标签部分.
            // double label_prob = dirichlet(ob_rh.classVoter, ob_num, label);

            // update 6-2日更新:
            // 获得该 label 的观测数量
            
            // ************* 原始代码 [Matlab] ****************
            // dp_prior = p(obj.measurements.label(k),:);
            // sq_dist = sum((obj_poses - repmat(pos,1,N)).^2,1);
            // posterior = exp(-4*sq_dist).*dp_prior;
            // ************************************************

            auto iter = ob_rh.classVoter.find(label);
            int label_num = 0;
            if( iter != ob_rh.classVoter.end() ){
                label_num = iter->second;
            }
            label_num += 1; // at least one ob
            // 将分母简化后， dp_prior 只与该label的观测数量有关
            double dp_prior = double(label_num);

            double prob_dis = 0;
            if(model == 0)  // Ellipsoid
                prob_dis = calculateAssociationProbability(m, ob_rh);
                // prob_dis = calculateAssociationProbabilityUsingPoint(m, ob_rh); // 测试, 也使用点模型.
            else if(model == 1) // Point
                prob_dis = calculateAssociationProbabilityUsingPoint(m, ob_rh);
            else
            {
                std::cout << "Error model value : " << model << ", must be 0: ellipsoid, 1: point." << std::endl;
            }
            double posterior = prob_dis * dp_prior;

            // 接着取最大概率做处理.
            vec_posterior.push_back(posterior);

            // 保存结果
            OneDAResult result;
            result.measure_id = k;
            result.object_id = obj_id;
            result.posterior = posterior;
            result.prob_dis = prob_dis;
            result.prob_label = 1.0;    // useless , wait to be deleted.
            result.dp_prior = dp_prior;
            daResult.results.push_back(result);
        }
        // 找到最大的 posterior 及其 id
        if(vec_posterior.size() != 0) {
            auto biggest = std::max_element(std::begin(vec_posterior), std::end(vec_posterior));
            int maxpiror_obj_id = std::distance(std::begin(vec_posterior), biggest);
            double maxprior = *biggest;

            daResult.measure_id = k;
            daResult.result_id = maxpiror_obj_id;
            // 最大后验是否满足配置要求
            if(maxprior > CONFIG_DP_alpha)
            {
                EllipObject& ob = objs[maxpiror_obj_id];
                // add measurement
                if( ob.classVoter.find(label) == ob.classVoter.end())
                    ob.classVoter[label] = 1;  // 注意初始化时 label 一定要覆盖所有label?
                else 
                    ob.classVoter[label]++;

                ob.measurementIDs.push_back(k);

                // 观测关联物体
                m.instance_id = ob.instance_id;

                daResult.object_id = maxpiror_obj_id;

                // 针对该物体标记“边缘无效边"
                // std::cout << "Begin check valid border." << std::endl;
                CheckValidBorder(ob, m);
            }
            else
            {
                // 注意这类初始化下面还有一个
                if(!bPointModel){
                    // 无可关联物体, 初始化一个新物体
                    int instance_id = g_instance_total_id++;      // 这个必须是唯一 id.
                    EllipObject ob_new;
                    ob_new.instance_id = instance_id;
                    ob_new.classVoter[label] = 1;
                    ob_new.measurementIDs.push_back(k);

                    // 分配新的椭球体内存空间.
                    // 此处应该归给地图.
                    ellipsoid* pe = new g2o::ellipsoid(m.ob_3d.pObj->transform_from(frame_pos_Twc));
                    ob_new.pEllipsoid = pe;

                    objs.push_back(ob_new);

                    m.instance_id = instance_id;

                    daResult.object_id = instance_id;
                }
                else
                {
                    // 关联失败
                    // 维持现状
                    m.instance_id = -1; // 一定要恢复..
                }
                
            }
        }
        else 
        {                
            // 无可关联物体, 初始化一个新物体
            if(!bPointModel){
                int instance_id = g_instance_total_id++;      // 这个必须是唯一 id.
                EllipObject ob_new;
                ob_new.instance_id = instance_id;
                ob_new.classVoter[label] = 1;
                ob_new.measurementIDs.push_back(k);

                // 分配新的椭球体内存空间.
                // 此处应该归给地图.
                ellipsoid* pe = new g2o::ellipsoid(m.ob_3d.pObj->transform_from(frame_pos_Twc));

                // 1) 此处要对齐地面

                ob_new.pEllipsoid = pe;

                objs.push_back(ob_new);

                m.instance_id = instance_id;

                daResult.object_id = instance_id;
            }
            else
            {
                m.instance_id = -1; // 一定要恢复..
            }
        }

        daResults.push_back(daResult);
        
    } // for : measurements

    // output da Result
    if(daResults.size() > 1)
        OutputComplexDAResult(daResults, objs);
    return;
}

void Optimizer::LoadRelations(Relations& rls, SupportingPlanes& spls)
{
    mRelations = rls;
    mSupportingPlanes = spls;
    mbRelationLoaded = true;
    std::cout << "Relations and Supportingplanes have been Loaded." << std::endl;
    return;
}

void Optimizer::GetOptimizedResult(Objects& objs, Measurements& mms)
{
    objs = mObjects;
    mms = mMeasurements;
}

int GetTotalObjectIndex(std::vector<Frame *> &pFrames, int frame_index, int index_in_frame)
{
    int total_id = 0;
    for(int i=0;i<frame_index;i++)
    {
        total_id+= pFrames[i]->mpLocalObjects.size();
    }
    return total_id+index_in_frame;
}

} // namespace: EllipsoidSLAM