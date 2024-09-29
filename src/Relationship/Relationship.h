// 2020-5-13, by lzw
// 该文件用于处理平面与物体的“支撑关系”， 包括判断、约束、关联等内容。

#ifndef RELATIONSHIP_H
#define RELATIONSHIP_H

#include <iostream>
#include <vector>

#include <include/core/Ellipsoid.h>
#include <include/core/Plane.h>

#include <Eigen/Core>

#include "Frame.h"

namespace ORB_SLAM2
{
// 该类在局部坐标系下计算, 匹配局部物体与局部平面之间的潜在约束关系。
// 提取结果保存为  obj_id -> plane_id 的映射. 以及结构体.
class Frame;

// 关联关系，包含物体索引、支撑平面索引、平面索引、
class Relation
{
public:
    int obj_id; // 索引: 与localEllipsoids, measurements同索引.
    int plane_instance_id;  // 关联 SupportingPlanes 的 id
    int plane_id;
    g2o::plane* pPlane;
    g2o::ellipsoid* pEllipsoid;
    Frame* pFrame;
    int type; // 无效关系0, 支撑关系 1, 倚靠关系 2.

    Eigen::VectorXd SaveToVec();
    void LoadFromVec(const Eigen::VectorXd& vec);
    bool InitRelation(const Eigen::VectorXd& vec, Frame* pFrameIn);

private:
    int GetDataNum();
};
typedef std::vector<Relation> Relations;

class RelationExtractor
{
public:
    Relations ExtractRelations(std::vector<g2o::ellipsoid *> &vpEllips, std::vector<g2o::plane *> &vpPlanes);

    Relations ExtractSupporttingRelations(std::vector<g2o::ellipsoid *> &vpEllips, std::vector<g2o::plane *> &vpPlanes, Frame* pFrame, int model = 1);

};

} // namespace EllipsoidSLAM

#endif
