#pragma  once

#include "include/core/Plane.h"
#include "src/Relationship/Relationship.h"
#include <vector>
#include <set>

namespace ORB_SLAM2
{

class Relation;

class SupportingPlane
{
public:
    SupportingPlane(Relation& rl, int id);
    bool addObservation(Relation& rl, int id);
    bool deleteObservation(Relation& rl, int id);
    bool empty();

    static bool FindNearestDis(Relation& rl, std::vector<SupportingPlane> &spls, double& dis, int& id);

public:
    int static total_sup_plane;

    int instance_id;    // 平面的唯一 instance 
    g2o::plane* pPlane;  // 世界坐标系下的平面表达
    double height; //  平面相对地面的高度, 用于聚类
    std::set<int> vRelation;   // 该平面的观测, 以 relation 做索引
    
    std::set<int> vInstance;       // 位于该平面上的物体 id. 索引 objects

private:
    void refreshInstance();

};

typedef std::vector<SupportingPlane> SupportingPlanes;

}