#include "include/core/ConstrainPlane.h"
#include "include/core/Plane.h"

#include <Eigen/Core>
using namespace Eigen;
namespace g2o
{

ConstrainPlane::ConstrainPlane(plane* ppl):valid(true), image_border(false), association_border(false), pPlane(ppl), state(0), type(-1)
{
}

VectorXd ConstrainPlane::toVector()
{
    // 一些状态
    Vector5d stateVec;
    stateVec << double(valid), double(image_border), double(association_border), double(state), double(type);

    // 还要包含内部的平面结构
    Vector4d vec_plane(-1,-1,-1,-1);
    if(pPlane!=NULL)
        vec_plane = pPlane->param;

    // 注意平面内部的其他信息不存
    VectorXd vec_cplane; vec_cplane.resize(vectorSize());
    vec_cplane << stateVec, vec_plane;

    return vec_cplane;
}

void ConstrainPlane::fromVector(VectorXd& vec){
    if(vec.size()!=vectorSize()) {std::cerr << "Wrong vec size : " << vec.size() << std::endl; return;}
    Vector4d stateVec = vec.head(4);

    // TODO: check 这样的转换是否有效
    valid = round(vec[0]) > 0;
    image_border = round(vec[1]) > 0;
    association_border = round(vec[2]) > 0;
    state = round(vec[3]);
    type = round(vec[4]);

    // 读取平面
    Vector4d vec_plane = vec.tail(4);
    if(pPlane==NULL) pPlane = new g2o::plane(vec_plane);
    else pPlane->param = vec_plane;

    return;
}

// 7-8:  8 + 1: type
int ConstrainPlane::vectorSize()
{
    return 9;
}


} // g2o
