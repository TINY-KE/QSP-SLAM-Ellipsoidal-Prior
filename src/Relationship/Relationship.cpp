#include "Relationship.h"
// #include "Frame.h"

// #include <pybind11/embed.h>
// #include <pybind11/eigen.h>
// #include <Python.h>

// #include <pybind11/embed.h>
// #include <pybind11/eigen.h>

// namespace py = pybind11;

using namespace std;
namespace ORB_SLAM2
{
    bool sort_plane_dis(pair<double,g2o::plane*>&p1, pair<double,g2o::plane*>&p2)
    {
        return p1.first < p2.first;
    }

    int JudgeRelation(std::vector<g2o::plane*> &objPlanes, g2o::plane *pPlane)
    {
        // 判断平面类别.
        int relationType = 0;
        if (pPlane->miMHType == 1) // parallel
        {
            // 判断支撑关系.

            // 取得最下方的平面
            // 经检查 1,2,3,4点为底部点, 对应id为0
            g2o::plane &bottom_object_plane = *(objPlanes[0]);

            // 判断二者距离
            double dis = bottom_object_plane.distanceToPlane(*pPlane);

            if (dis < 0.2) // 5cm dis
                relationType = 1;
            else 
                std::cout << " == dis : " << dis << std::endl;
        }

        // --------------------------
        //   处理倚靠关系, 暂时不考虑.
        // ---------------------------
        // else if (pPlane->miMHType == 2) // vertical
        // {
        //     // 判断倚靠关系

        //     // 不如直接获得周围四个平面, 计算与该平面朝向&&距离.
        //     // 二者皆小于一定条件则认为倚靠成立.
        //     // 经过规律检查, 2,3,4,5 id属于侧面一圈
        //     for (int i = 2; i < 6; i++)
        //     {
        //         g2o::plane &side_object_plane = *(objPlanes[i]);

        //         // 先判断转角
        //         double angle_diff = side_object_plane.angleToPlane(*pPlane);
        //         if (std::abs(angle_diff) < M_PI / 180.0 * 5) // 容忍 5 度
        //         {
        //             double dis = side_object_plane.distanceToPlane(*pPlane);
        //             if (dis < 0.2)
        //             {
        //                 // 角度与距离皆满足要求, 认为关联成功
        //                 relationType = 2;
        //             }
        //         }
        //     }
        // }

        return relationType;
    }

    Relations RelationExtractor::ExtractRelations(std::vector<g2o::ellipsoid *> &vpEllips, std::vector<g2o::plane *> &vpPlanes)
    {
        Relations relations;
        int obj_num = vpEllips.size();
        for (int obj_id = 0; obj_id < obj_num; obj_id++)
        {
            g2o::ellipsoid *pEllip = vpEllips[obj_id];
            if(pEllip == NULL ) {
                // std::cout << "[Relation] NULL ellipsoid." << std::endl;
                continue;
            }
            std::vector<g2o::plane*> obj_planes = pEllip->GetCubePlanes();  // 椭球体所在的坐标系

            int plane_num = vpPlanes.size();
            for (int plane_id = 0; plane_id < plane_num; plane_id++)
            {
                g2o::plane *pPlane = vpPlanes[plane_id];

                // 0: no Relation ;  1 : Supporting ;  2 : LeanOn
                int type = JudgeRelation(obj_planes, pPlane);

                if (type > 0)
                {
                    // 保存该组关系
                    Relation rl;
                    rl.obj_id = obj_id;
                    rl.plane_id = plane_id;
                    rl.type = type;
                    rl.pPlane = pPlane;
                    rl.pEllipsoid = pEllip;
                    relations.push_back(rl);
                }
            }
        }
        return relations;
    }

    // 新函数，仅仅提取支撑关系
    // 要求传入平面: 满足曼哈顿假设, 且与地平面平行.
    Relations RelationExtractor::ExtractSupporttingRelations(std::vector<g2o::ellipsoid *> &vpEllips, std::vector<g2o::plane *> &vpPlanes, Frame* pFrame, int model)
    {
        Relations relations;
        int obj_num = vpEllips.size();
        for (int obj_id = 0; obj_id < obj_num; obj_id++)
        {
            g2o::ellipsoid *pEllip = vpEllips[obj_id];
            if(pEllip == NULL ) {
                // std::cout << "[Relation] NULL ellipsoid." << std::endl;
                continue;
            }

            bool success = false;
            int sup_plane_id=-1;
            g2o::plane* pPlane_best = NULL;
            if(model == 1){
                std::vector<g2o::plane*> obj_planes = pEllip->GetCubePlanes();  // 椭球体所在的坐标系
                g2o::plane* pObj_bottom_plane = obj_planes[0];

                int plane_num = vpPlanes.size();
                
                // 计算所有平面与某物体距离, 取最近小于阈值平面.
                std::vector<std::pair<double, g2o::plane*>> planeDisVec;
                for (int plane_id = 0; plane_id < plane_num; plane_id++)
                {
                    g2o::plane *pPlane = vpPlanes[plane_id];
                    if(pPlane->miMHType==1){
                        double dis = pObj_bottom_plane->distanceToPlane(*pPlane);
                        if(dis>0)   // 有效dis. 即属于平行平面
                            planeDisVec.push_back(make_pair(dis, pPlane));
                    }
                }
                if(planeDisVec.size()<1) continue;
                // 获得最近距离平面.
                sort(planeDisVec.begin(), planeDisVec.end(), sort_plane_dis);
                
                double dis_min = planeDisVec[0].first;
                pPlane_best = planeDisVec[0].second;
                
                if( dis_min < 0.1 ) success = true;
            }
            else if( model == 0 )
            {
                // ***************** 尚未完成 *****************
                // ******************************************
                //        获得到中心点最近的平面, 且要求沿着重力方向. //
                // 配置
                double config_pointmodel_dis_thresh = 2;    // 暂时设2m, 形同虚设; 其实应该跟物体体积有关.

                // 若属于点模型，使用中点判断距离
                int plane_num = vpPlanes.size();
                Eigen::Vector3d center = pEllip->pose.translation();
                
                // 计算所有平面与某物体[中心]距离 (带方向), 取最近小于阈值平面.
                std::vector<std::pair<double, g2o::plane*>> planeDisVec;
                for (int plane_id = 0; plane_id < plane_num; plane_id++)
                {
                    g2o::plane *pPlane = vpPlanes[plane_id];
                    if(pPlane->miMHType==1){
                        double dis = pPlane->distanceToPoint(center, true);
                        if(dis>0 && dis < config_pointmodel_dis_thresh)   // 位于平面上方
                            planeDisVec.push_back(make_pair(dis, pPlane));
                    }
                }
                if(planeDisVec.size()<1) continue;
                // 获得最近距离平面.
                sort(planeDisVec.begin(), planeDisVec.end(), sort_plane_dis);
                success = true;
                
                double dis_min = planeDisVec[0].first;
                pPlane_best = planeDisVec[0].second;
            }

            if(success)
            {
                // 寻找 plane_id
                for(int i=0;i<vpPlanes.size();i++)
                {
                    if(vpPlanes[i] == pPlane_best)
                        sup_plane_id = i;
                }
                
                // 保存该组关系
                Relation rl;
                rl.obj_id = obj_id;
                rl.plane_id = sup_plane_id;
                rl.type = 1;     // Type id : {0: no Relation ;  1 : Supporting ;  2 : LeanOn}
                rl.pPlane = pPlane_best;
                rl.pEllipsoid = pEllip;
                rl.pFrame = pFrame;
                relations.push_back(rl);
            }
        }
        return relations;        
    }

        // int obj_id; // 索引: 与localEllipsoids, measurements同索引.
        // int plane_instance_id;  // 关联 SupportingPlanes 的 id
        // int plane_id;
        // g2o::plane* pPlane;
        // g2o::ellipsoid* pEllipsoid;
        // Frame* pFrame;
        // int type; // 无效关系0, 支撑关系 1, 倚靠关系 2.

    Eigen::VectorXd Relation::SaveToVec()
    {
        Eigen::VectorXd vec;
        vec.resize(GetDataNum());

        vec << double(obj_id), double(plane_instance_id), double(plane_id), double(type), 
                pPlane->param;

        // 保存平面!

        return vec;
    }

    bool Relation::InitRelation(const Eigen::VectorXd& vec, ORB_SLAM2::Frame* pFrameIn)
    {
        // 根据 obj_id, plane_id 初始化 pPlane, pEllipsoid 的值.

        this->pFrame = pFrameIn;
        LoadFromVec(vec);

        if(pFrameIn->mpLocalObjects.size() <= obj_id)
        {
            std::cout << "Wrong size of mpLocalObjects in pFrameIn." << std::endl;
            std::cout << "Objects in frame : " << pFrameIn->mpLocalObjects.size() << std::endl;
            std::cout << "obj_id : " << obj_id << std::endl;
            std::cout << "vec : " << vec.transpose() << std::endl;
            // std::cout << "timestamp of frame : " << std::to_string(pFrameIn->timestamp) << std::endl;
            std::cout << "timestamp of frame : " << std::to_string(pFrameIn->mTimeStamp) << std::endl;
            return false;
        }

        this->pEllipsoid = pFrameIn->mpLocalObjects[obj_id];

        return true;
    }

    void Relation::LoadFromVec(const Eigen::VectorXd& vec)
    {
        if(vec.size()!=GetDataNum()){
            std::cerr << "Wrong size of vec to load." << std::endl;
        }
        obj_id = round(vec[0]);
        plane_instance_id = round(vec[1]);
        plane_id = round(vec[2]);
        type = round(vec[3]);

        Vector4d planeVec = vec.head(8).tail(4);    // 4-8
        g2o::plane* pPlaneConstruct = new g2o::plane(planeVec);
        pPlane = pPlaneConstruct;

        return;
    }

    int Relation::GetDataNum()
    {
        // planevec : 4
        return 4 + 4;
    }

} // namespace EllipsoidSLAM