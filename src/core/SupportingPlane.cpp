#include "include/core/SupportingPlane.h"
#include "Frame.h"

#include <Eigen/Core>
using namespace Eigen;

namespace ORB_SLAM2
{
int SupportingPlane::total_sup_plane=0;

bool compare_dis(std::pair<double,int>&p1, std::pair<double,int>&p2)
{
    return p1.first < p2.first;
}

// 注意： 并非寻找最近距离, 而是考虑 dp_prior 的聚类.
bool SupportingPlane::FindNearestDis(Relation& rl, SupportingPlanes &spls, double& prob, int& id)
{
    int spls_num = spls.size();
    if(spls_num <= 0 ) 
    {
        std::cout << "Error: No supporting planes!" << std::endl;
        return false;
    }

    std::vector<std::pair<double,int>> pvDisId;
    std::vector<double> disVec;
    for( int i=0;i<spls_num;i++)
    {
        g2o::SE3Quat& Twc = rl.pFrame->cam_pose_Twc;
        g2o::plane* pPlane = rl.pPlane;
        g2o::plane* pl_w = new g2o::plane(*pPlane); pl_w->transform(Twc);
 
        SupportingPlane& spl = spls[i];
        g2o::plane* pl_spl = spl.pPlane;

        // 新版本 在 3D 计算旋转和平移误差.
        Vector3d diff_vec = pl_w->ominus(*pl_spl); // azimuth, elevation, d

        const double deg = M_PI / 180.0 * 10;
        Vector3d inv_sigma; inv_sigma << 1.0/deg, 1.0/deg, 1.0/0.1; 
        Matrix3d info_mat = inv_sigma.cwiseProduct(inv_sigma).asDiagonal();
        double chi2 = diff_vec.dot(info_mat*diff_vec);

        double prob = exp(-chi2); // 求方差后综合计算一个prob概率.

        // *********** 旧版本: 假定这些平面必然平行 ***********
        // double dis = pl_w->distanceToPlane(*pl_spl);
        // double sq_dist = dis * dis;

        // double dp_prior = spl.vRelation.size(); // 即已有的数量.
        // double prob = exp(-sq_dist/0.1/0.1)*dp_prior;
        // *************************************************

        // 计算距离.
        pvDisId.push_back(std::make_pair(prob, i));
        disVec.push_back(chi2);
    }

    // 排序
    sort(pvDisId.begin(), pvDisId.end(), compare_dis);
    // 输出 id
    id = pvDisId[0].second;
    prob = pvDisId[0].first;

    return true;
}

SupportingPlane::SupportingPlane(Relation& rl, int id)
{
    instance_id = total_sup_plane++;

    g2o::SE3Quat& Twc = rl.pFrame->cam_pose_Twc;
    g2o::plane* pPlane = rl.pPlane;
    g2o::plane* pl_w = new g2o::plane(*pPlane); pl_w->transform(Twc);

    this->pPlane = pl_w;
    addObservation(rl, id);
}

// 传入 relations 的 id
bool SupportingPlane::addObservation(Relation& rl, int i)
{
    vRelation.insert(i);

    // 关联该relation到sp
    rl.plane_instance_id = instance_id;

    return true;
}

bool SupportingPlane::deleteObservation(Relation& rl, int k)
{
    auto iter = vRelation.find(k);
    if(iter!=vRelation.end()){
        vRelation.erase(iter);
        rl.plane_instance_id = -1;  // 归位
        return true;
    }
    else return false;
}

bool SupportingPlane::empty()
{
    return (vRelation.size()==0);
}

} // namespace EllipsoidSLAM