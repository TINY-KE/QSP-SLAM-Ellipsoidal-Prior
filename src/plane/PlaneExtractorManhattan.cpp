// 本文件基于曼哈顿假设提取平面

#include "PlaneExtractorManhattan.h"
#include <src/config/Config.h>

namespace ORB_SLAM2
{

void PlaneExtractorManhattan::SetGroundPlane(g2o::plane* gplane)
{
    mpGroundplane = gplane;
}

bool PlaneExtractorManhattan::extractManhattanPlanes(const cv::Mat &depth, Eigen::Vector3d& local_gt, g2o::SE3Quat &Twc)
{
    // ************************
    // INIT
    // ************************
    mParam.RangeOpen = false;
    mvPotentialGroundPlanePoints.clear();
    mvPotentialMHPlanesPoints.clear();
    mvpPotentialMHPlanes.clear();   // 满足了曼哈顿假设的都放进去
    mvpPotentialStructuralMHPlanes.clear();
    mbResult = false;

    // 直接提取!
    // 注意: 该部分结果完全可以与地平面共用
    extractPlanes(depth);
    if( mvPlaneCoefficients.size() < 1)     // there should be more than 1 potential planes 
        return false;

    // *******************************************
    //     筛选曼哈顿！  需要参考平面: 地平面(重力方向)
    // the Manhattan should meet the criteria:
    // vertical to or parrel to the gravity direction.
    // *******************************************
    std::vector<g2o::plane*> vpPlanes;
    std::vector<std::pair<g2o::plane*, int>> mapPlaneSize;

    double config_angle_delta = 5 / 180.0 * M_PI;   // 5 deg.
    int MH_plane_size = depth.rows * depth.cols / 6;   // 要求具有屏幕 1/4的点!
    MH_plane_size = MH_plane_size / 9;
    for( int i=0; i<mvPlaneCoefficients.size(); i++ )
    {
        Eigen::Vector4d vec;
        auto& coeff = mvPlaneCoefficients[i];
        vec << double(coeff.at<float>(0)), double(coeff.at<float>(1)), double(coeff.at<float>(2)), double(coeff.at<float>(3));

        Eigen::Vector3d axis_gravity = local_gt;       
        Eigen::Vector3d axisNorm = vec.head(3);
        double cos_theta = axisNorm.transpose() * axis_gravity;
        cos_theta = cos_theta / axisNorm.norm() / axis_gravity.norm();

        double angle = acos( cos_theta );      // acos : [0,pi]

        int iMHType = 0;
        if( std::abs(angle - 0)<config_angle_delta || 
                std::abs(angle- M_PI) < config_angle_delta ) 
        {
            iMHType = 1;    // parallel

        }

        // ---- DEBUG: 暂时取消垂直倚靠关系. 只考虑普遍存在的支撑关系
        else if( std::abs(angle - M_PI/2.0)<config_angle_delta )
        {
            iMHType = 2; // Vertical
        }

        if( iMHType > 0 )
        {
            g2o::plane* pPlane = new g2o::plane();
            pPlane->param= vec;
            pPlane->miMHType = iMHType;

            mvpPotentialMHPlanes.push_back(pPlane);   // 满足了曼哈顿假设的都放进去
            mvPotentialMHPlanesPoints.push_back(mvPlanePoints[i]);

            int num_size = mvPlanePoints[i].size();
            if( num_size > MH_plane_size){
                vpPlanes.push_back(pPlane);
                mvPotentialGroundPlanePoints.push_back(mvPlanePoints[i]);
            }
            else
            {
                // 若属于支撑平面, 大小要求放宽!
                if( iMHType == 1 && num_size > 200)
                {
                    vpPlanes.push_back(pPlane);
                    mvPotentialGroundPlanePoints.push_back(mvPlanePoints[i]);
                }
            }
            
        }  
    }

    std::cout << "Potential MHPlanes size: " << vpPlanes.size() << std::endl;
    if( vpPlanes.size() < 1) {      // there should be more than 1 valid planes 
        return false;
    }

    // 保存结果！
    mvpPotentialStructuralMHPlanes = vpPlanes;

    // 针对此次结果更新存储的曼哈顿平面
    UpdateMHPlanes(Twc);

    return true;
}

void PlaneExtractorManhattan::AddNewDominantMHPlane(g2o::plane* vP)
{
    mvpDominantStructuralMHPlanes.push_back(vP);
    std::cout << "[Manhattan] Add new MH Plane. Total : " << mvpDominantStructuralMHPlanes.size() << std::endl;

    if(mvpDominantStructuralMHPlanes.size() == 5)
        mbDominantResult = true;
}

std::vector<g2o::plane*> PlaneExtractorManhattan::GetDominantMHPlanes()
{
    return mvpDominantStructuralMHPlanes;
}


PlaneExtractorManhattan::PlaneExtractorManhattan():mbResult(false),mbDominantResult(false),mpGroundplane(NULL)
{}

void PlaneExtractorManhattan::UpdateMHPlanes(g2o::SE3Quat &Twc)
{
    if(!mbDominantResult)
    {
        // 一一判断 mvpPotentialStructuralMHPlanes 是否为新的曼哈顿帧, 条件:
        // 0). 平面之大小: 已经过滤完毕.
        // 1). 不能与已有的距离太近 ( 2m )
        // 2). 与已有平面法向量相同的, 不能超过2个 

        for( auto& vP:mvpPotentialStructuralMHPlanes)
        {
            if( mvpDominantStructuralMHPlanes.size() >= 5) break; // 已经满了

            // 对每个检查所有已确定 MH Plane
            int state = 0;
            int parallel_count = 0;
            int vertical_count = 0;

            g2o::plane* pPlanesGlobal = new g2o::plane(*vP); pPlanesGlobal->transform(Twc);
            Vector4d param_plane = pPlanesGlobal->param; 
            Eigen::Vector3d norm_plane = param_plane.head(3);

            std::vector<g2o::plane*> MHPlanes = mvpDominantStructuralMHPlanes;
            MHPlanes.push_back(mpGroundplane);   // 加入地平面
            for( auto& vMHP : MHPlanes ) // 注意可能添加新的 mvpD 进去! 无所谓，是新的循环了
            {
                Vector4d param_MHplane = vMHP->param; 
                Eigen::Vector3d norm_MHplane = param_MHplane.head(3);

                // 判断法向量角度
                double cos_theta = norm_plane.transpose() * norm_MHplane;
                cos_theta = cos_theta / norm_plane.norm() / norm_MHplane.norm();
                double angle = acos( cos_theta );      // acos : [0,pi]
                
                double angle_tolerance = 5 * M_PI / 180.0;
                if( std::abs(angle-0) < angle_tolerance || std::abs(angle-M_PI) < angle_tolerance )
                {
                    // 与该平面平行
                    parallel_count++;

                    // 将两平面的 ABC 归一化.
                    // double D_plane = param_plane[3] / param_plane.head(3).norm();
                    // double D_MH = param_MHplane[3] / param_MHplane.head(3).norm();
                    // double distance = std::abs(D_MH-D_plane);

                    double distance = pPlanesGlobal->distanceToPlane(*vMHP);
                    if( distance < 3.0) // TODO: 输出调试这个值.
                    {
                        state = 1;      // 1) 发现距离太近平面
                        std::cout << "distance : " << distance << std::endl;
                    }
                }
                else if (std::abs(angle-M_PI/2) < angle_tolerance )
                {
                    // 与该平面垂直
                    vertical_count++;
                    // 无距离 check.
                }
                else 
                {
                    // 与已有平面，不垂直也不平行； 不满足曼哈顿假设.
                    state = 3;
                }

                // double angle = ;    // 若是垂直则不需要管. 
                //                     // 平行 则粗略计算距离.

                //                     // 注意这里假定了 MH, 第一个平面根据大小过滤必然是墙面.
            }

            // 检查count
            if(parallel_count >= 2)
                state = 2; // 2) 该方向的 MH平面已经有了                    
            

            if(state == 0){
                // 做符号检查, 要求在平面内部的距离 > 0
                Eigen::Vector3d center_wc = Twc.translation();
                if (pPlanesGlobal->distanceToPoint(center_wc, true) < 0)
                    pPlanesGlobal->param = - pPlanesGlobal->param;

                // 可视化
                pPlanesGlobal->InitFinitePlane(center_wc, 10);

                // 通过检查, 添加进 DMHP
                AddNewDominantMHPlane(pPlanesGlobal);
                mbResult = true;    // 添加了新的，本次成功.

            }
            else 
                std::cout << " [Manhattan] MH Plane Fails. State: " << state << std::endl;

        }

    }
}

std::vector<g2o::plane*> PlaneExtractorManhattan::GetPotentialStructuralMHPlanes()
{
    return mvpPotentialStructuralMHPlanes;
}

std::vector<g2o::plane*> PlaneExtractorManhattan::GetPotentialMHPlanes()
{
    return mvpPotentialMHPlanes;
}


bool PlaneExtractorManhattan::GetMHResult()
{
    return mbResult;
}

std::vector<PointCloudPCL> PlaneExtractorManhattan::GetPotentialMHPlanesPoints()
{
    return mvPotentialMHPlanesPoints;
}

}