/**
* This file is part of https://github.com/JingwenWang95/DSP-SLAM
*
* This program is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program.  If not, see <http://www.gnu.org/licenses/>
*/

#include "MapObject.h"
#include "Converter.h"
#include "src/symmetry/PointCloudFilter.h"

#include <Eigen/Dense>

#include <pcl/io/pcd_io.h>

namespace ORB_SLAM2
{

int MapObject::nNextId = 0;

MapObject::MapObject(const Eigen::Matrix4f &T, const Eigen::Matrix<float, 64, 1> &vCode, KeyFrame *pRefKF, Map *pMap, int class_id):
        mpRefKF(pRefKF), mpNewestKF(pRefKF), mnBALocalForKF(0), mnAssoRefID(0), mnFirstKFid(pRefKF->mnId),
        mnCorrectedByKF(0), mnCorrectedReference(0), mnLoopObjectForKF(0), mnBAGlobalForKF(0),
        w(0.4), h(0.4), l(0.4), mbBad(false), mbDynamic(false), mpMap(pMap), nObs(0), mRenderId(-1)
{
    // Transformation Matrix in Sim3
    Sim3Two = T;
    Sim3Tow = Sim3Two.inverse();

    // Decompose T into Rotation, translation and scale
    Rwo = T.topLeftCorner<3, 3>();
    // scale is fixed once the object is initialized
    scale = pow(Rwo.determinant(), 1./3.);
    invScale = 1. / scale;
    Rwo /= scale;
    two = T.topRightCorner<3, 1>();

    // Transformation Matrix in SE3
    SE3Two = Eigen::Matrix4f::Identity();
    SE3Two.topLeftCorner<3, 3>() = Rwo;
    SE3Two.topRightCorner<3, 1>() = two;
    SE3Tow = SE3Two.inverse();

    vShapeCode = vCode;
    velocity = Eigen::Vector3f::Zero();
    mnId = nNextId++;
    findGoodOrientation = false;
    mbValidEllipsoldFlag = false;
    mbValidPointCloudFlag = false;
    label = class_id;
}

MapObject::MapObject(KeyFrame *pRefKF, Map *pMap, int class_id) :
        mpRefKF(pRefKF), mpNewestKF(pRefKF), mnBALocalForKF(0), mnAssoRefID(0), mnFirstKFid(pRefKF->mnId),
        mnCorrectedByKF(0), mnCorrectedReference(0), mnLoopObjectForKF(0), mnBAGlobalForKF(0),
        reconstructed(false), w(0.4), h(0.4), l(0.4), mbBad(false), mbDynamic(false), mpMap(pMap), nObs(0), mRenderId(-1)
{
    mnId = nNextId++;
    // scale = 1.;
    scale = 0.4;
    // invScale = 1.;
    invScale = 1. / scale;

    vShapeCode = Eigen::Matrix<float, 64, 1>::Zero();
    findGoodOrientation = false;
    mbValidEllipsoldFlag = false;
    mbValidPointCloudFlag = false;
    label = class_id;
    // hasSetEllipsold = false;

}

void MapObject::AddObservation(KeyFrame *pKF, int idx)
{
    unique_lock<mutex> lock(mMutexObject);
    if(!mObservations.count(pKF))
        nObs++;
    mObservations[pKF]=idx;
    mpNewestKF = pKF;
}

std::map<KeyFrame*, size_t> MapObject::GetObservations()
{
    unique_lock<mutex> lock(mMutexObject);
    return mObservations;
}

void MapObject::EraseObservation(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexObject);
    if(mObservations.count(pKF))
    {
        nObs--;
        mObservations.erase(pKF);

        if(mpRefKF==pKF)
            mpRefKF=mObservations.begin()->first;

        if(mpNewestKF == pKF)
        {
            int mnLargestKFId = 0;
            KeyFrame *pNewestKF = static_cast<KeyFrame*>(nullptr);
            for(std::map<KeyFrame *, size_t>::iterator mit = mObservations.begin(), mend = mObservations.end(); mit != mend; mit++)
            {
                KeyFrame* plKF = mit->first;
                if (plKF->mnId > mnLargestKFId)
                {
                    mnLargestKFId = plKF->mnId;
                    pNewestKF = plKF;
                }
            }
            mpNewestKF = pNewestKF;
        }

    }
}

int MapObject::Observations()
{
    unique_lock<mutex> lock(mMutexObject);
    return nObs;
}

void MapObject::SetBadFlag()
{
    map<KeyFrame*,size_t> obs;
    {
        unique_lock<mutex> lock(mMutexObject);
        mbBad = true;
        obs = mObservations;
        mObservations.clear();
    }
    for(map<KeyFrame*,size_t>::iterator mit=obs.begin(), mend=obs.end(); mit!=mend; mit++)
    {
        KeyFrame* pKF = mit->first;
        pKF->EraseMapObjectMatch(mit->second);
    }
}

bool MapObject::isBad()
{
    unique_lock<mutex> lock(mMutexObject);
    return mbBad;
}

bool MapObject::hasValidEllipsold()
{
    unique_lock<mutex> lock(mMutexObject);
    return mbValidEllipsoldFlag;
}

bool MapObject::hasValidPointCloud()
{
    unique_lock<mutex> lock(mMutexObject);
    return mbValidPointCloudFlag;
}

int MapObject::GetIndexInKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexObject);
    if (mObservations.count(pKF))
        return mObservations[pKF];
    else
        return -1;
}

bool MapObject::IsInKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexObject);
    return (mObservations.count(pKF));
}

void MapObject::Replace(MapObject *pMO)
{
    if(pMO->mnId==this->mnId)
        return;

    map<KeyFrame*,size_t> obs;
    {
        unique_lock<mutex> lock1(mMutexObject);
        obs = mObservations;
        mObservations.clear();
        mbBad = true;
        mpReplaced = pMO;
    }

    for(map<KeyFrame*,size_t>::iterator mit=obs.begin(), mend=obs.end(); mit!=mend; mit++)
    {
        // Replace measurement in keyframe
        KeyFrame* pKF = mit->first;

        if(!pMO->IsInKeyFrame(pKF))
        {
            pKF->ReplaceMapObjectMatch(mit->second, pMO);
            pMO->AddObservation(pKF, mit->second);
        }
        else
        {
            pKF->EraseMapObjectMatch(mit->second);
        }
    }

    this->SetBadFlag();
}

KeyFrame* MapObject::GetReferenceKeyFrame()
{
    unique_lock<mutex> lock(mMutexObject);
    return mpRefKF;
}

void MapObject::SetObjectPoseSim3(const Eigen::Matrix4f &Two)
{
    unique_lock<mutex> lock(mMutexObject);
    Sim3Two = Two;
    Sim3Tow = Sim3Two.inverse();

    // Decompose T into Rotation, translation and scale
    Rwo = Two.topLeftCorner<3, 3>();
    // scale is fixed once the object is initialized
    scale = pow(Rwo.determinant(), 1./3.);
    invScale = 1. / scale;
    Rwo /= scale;
    two = Two.topRightCorner<3, 1>();

    // Transformation Matrix in SE3
    SE3Two = Eigen::Matrix4f::Identity();
    SE3Two.topLeftCorner<3, 3>() = Rwo;
    SE3Two.topRightCorner<3, 1>() = two;
    SE3Tow = SE3Two.inverse();
    // cout << "SE3Two = \n" << SE3Two.matrix() << endl;
}

void MapObject::SetObjectPoseSE3(const Eigen::Matrix4f &Two)
{
    unique_lock<mutex> lock(mMutexObject);
    SE3Two = Two;
    SE3Tow = SE3Two.inverse();
    Rwo = SE3Two.topLeftCorner<3, 3>();
    two = SE3Two.topRightCorner<3, 1>();
    Sim3Two.topLeftCorner<3, 3>() = Rwo * scale;
    Sim3Two.topRightCorner<3, 1>() = two;
    Sim3Tow = Sim3Two.inverse();
}

void MapObject::SetShapeCode(const Eigen::Matrix<float, 64, 1> &code)
{
    unique_lock<mutex> lock(mMutexObject);
    vShapeCode = code;
}

void MapObject::UpdateReconstruction(const Eigen::Matrix4f &T, const Eigen::Matrix<float, 64, 1> &vCode)
{
    SetObjectPoseSim3(T);
    SetShapeCode(vCode);
}

std::vector<MapPoint*> MapObject::GetMapPointsOnObject()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return vector<MapPoint *>(map_points.begin(), map_points.end());
}

// 对物体地图点进行简单过滤，如果
void MapObject::RemoveOutliersSimple()
{
    // First pass, remove all the outliers marked by ORB-SLAM
    // 首先去除所有被ORB-SLAM标注的外点

    // 统计计算所有内点的平均值
    int n_pts = 0;
    Eigen::Vector3f x3D_mean = Eigen::Vector3f::Zero();
    for (auto pMP : GetMapPointsOnObject())
    {
        if (!pMP)
            continue;
        if (pMP->isBad())
            this->EraseMapPoint(pMP);
        else
        {
            x3D_mean += Converter::toVector3f(pMP->GetWorldPos());
            n_pts++;
        }
    }

    if (n_pts == 0)
    {
        this->SetBadFlag();
        return;
    }

    // Second pass, remove obvious outliers
    // 距离大于1m则直接过滤
    double outlierDistance = Config::ReadValue<double>("ComputeCuboidPCA.outlierDistance");
    x3D_mean /= n_pts;
    for (auto pMP : GetMapPointsOnObject())
    {
        Eigen::Vector3f x3Dw = Converter::toVector3f(pMP->GetWorldPos());
        if ((x3Dw - x3D_mean).norm() > outlierDistance)
        {
            this->EraseMapPoint(pMP);
        }
    }
}

// 使用网格模型的顶点（添加一定系数）来过滤外点
void MapObject::RemoveOutliersModel()
{
    // sanity check: too few number of vertices
    if (vertices.rows() <= 10)
        return;

    float xmin = vertices.col(0).minCoeff();
    float xmax = vertices.col(0).maxCoeff();
    float ymin = vertices.col(1).minCoeff();
    float ymax = vertices.col(1).maxCoeff();
    float zmin = vertices.col(2).minCoeff();
    float zmax = vertices.col(2).maxCoeff();

    w = (xmax - xmin) * scale;
    h = (ymax - ymin) * scale;
    l = (zmax - zmin) * scale;
    float sx = 1.2;
    float sy = 1.5;
    float sz = 1.2;

    auto mvpMapPoints = GetMapPointsOnObject();
    for (auto pMP : mvpMapPoints)
    {
        if (!pMP)
            continue;

        if (pMP->isBad())
        {
            this->EraseMapPoint(pMP);
        }
        else
        {
            auto x3Dw = Converter::toVector3f(pMP->GetWorldPos());
            auto x3Do = invScale * Rwo.inverse() * x3Dw - invScale * Rwo.inverse() * two;
            if (x3Do(0) > sx * xmax || x3Do(0) < sx * xmin ||
                x3Do(1) > sy * ymax || x3Do(1) < sy * ymin ||
                x3Do(2) > sz * zmax || x3Do(2) < sz * zmin)
            {
                pMP->SetOutlierFlag();
            }
        }
    }
}


void GetRPYFromRotationMatrix(const Eigen::Matrix3f& R)
{
    // 提取旋转矩阵的元素
    float r11 = R(0, 0);
    float r12 = R(0, 1);
    float r13 = R(0, 2);
    float r21 = R(1, 0);
    float r31 = R(2, 0);
    float r32 = R(2, 1);
    float r33 = R(2, 2);
    
    // 计算yaw (绕Z轴的旋转)
    float yaw = std::atan2(r21, r11);
    
    // 计算pitch (绕Y轴的旋转)
    float pitch = std::atan2(-r31, std::sqrt(r32 * r32 + r33 * r33));
    
    // 计算roll (绕X轴的旋转)
    float roll = std::atan2(r32, r33);
    
    // 输出结果
    std::cout << "Roll: "  << roll  << " radians (" << roll * 180.0 / M_PI << " degrees)" << std::endl;
    std::cout << "Pitch: " << pitch << " radians (" << pitch * 180.0 / M_PI << " degrees)" << std::endl;
    std::cout << "Yaw: "   << yaw   << " radians (" << yaw * 180.0 / M_PI << " degrees)" << std::endl;
}

void MapObject::ComputeCuboidPCA(bool updatePose)
{
    RemoveOutliersSimple();
    auto mvpMapPoints = GetMapPointsOnObject();
    int N = mvpMapPoints.size();

    if (N == 0)
    {
        this->SetBadFlag();
        return;
    }

    Eigen::Vector3f x3D_mean = Eigen::Vector3f::Zero();
    Eigen::MatrixXf Xpts = Eigen::MatrixXf::Zero(N, 3);
    Eigen::MatrixXf Xpts_shifted = Eigen::MatrixXf::Zero(N, 3);
    for (int i = 0; i < N; i++)
    {
        auto pMP = mvpMapPoints[i];
        cv::Mat x3Dw = pMP->GetWorldPos();
        Xpts(i, 0) = x3Dw.at<float>(0);
        Xpts(i, 1) = x3Dw.at<float>(1);
        Xpts(i, 2) = x3Dw.at<float>(2);
        x3D_mean += Converter::toVector3f(pMP->GetWorldPos());
    }

    x3D_mean /= N;
    for (int i = 0; i < N; i++)
    {
        Xpts_shifted.row(i) = Xpts.row(i) - x3D_mean.transpose();
    }

    auto covX = Xpts_shifted.transpose() * Xpts_shifted;
    // cout << covX << endl;
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3f> eigensolver(covX);

    auto eigenvectors = eigensolver.eigenvectors();

    // Get rotation matrix, following ShapeNet definition
    // x : right, y: up, z: back
    Eigen::Matrix3f R;
    // Assume the order of principal axis: y, x, -z
    R.col(0) = eigenvectors.col(1);
    R.col(1) = eigenvectors.col(0);
    R.col(2) = -eigenvectors.col(2);

    // Check if det(R) = -1
    if (R.determinant() < 0)
        R.col(0) = -R.col(0);

    // Check if y direction is pointing upward by comparing its angle between camera
    auto neg_y = Eigen::Vector3f(0.f, -1.f, 0.f);
    if (neg_y.dot(R.col(1)) < 0)
    {
        // R.col(0) = -R.col(0);
        // R.col(1) = -R.col(1);
        Eigen::Matrix3f Rx;
        float angle = M_PI;  
        Rx << 1, 0, 0,
            0, std::cos(angle), -std::sin(angle),
            0, std::sin(angle), std::cos(angle);
        R = R*Rx;
    }

    int lo = int (0.05 * N);  // percentile threshold
    int hi = int (0.95 * N);
    auto Xpts_o = R.inverse() * Xpts.transpose(); // 3 x N
    Eigen::VectorXf x, y, z;
    x = Xpts_o.row(0);  // x corresponds to w
    y = Xpts_o.row(1);  // y corresponds to h
    z = Xpts_o.row(2);  // z corresponds to l
    // Sort the vectors
    std::sort(x.data(),x.data() + x.size());
    std::sort(y.data(),y.data() + y.size());
    std::sort(z.data(),z.data() + z.size());

    // PCA box dims
    w = (x(hi) - x(lo));
    h = (y(hi) - y(lo));
    l = (z(hi) - z(lo));
    Eigen::Vector3f cuboid_centre_o((x(hi) + x(lo)) / 2., (y(hi) + y(lo)) / 2., (z(hi) + z(lo)) / 2.);
    Eigen::Vector3f cuboid_centre_w = R * cuboid_centre_o;

    // Remove outliers using computed PCA box
    // 使用PCA包围盒过滤外点
    int num_outliers = 0;
    // float s = 1.2;
    double s = Config::ReadValue<double>("ComputeCuboidPCA.scale");
    for (auto pMP : mvpMapPoints)
    {
        if (!pMP)
            continue;

        if (pMP->isBad())
        {
            this->EraseMapPoint(pMP);
        }
        else
        {
            auto x3Dw = Converter::toVector3f(pMP->GetWorldPos());
            auto x3Do = R.inverse() * x3Dw - R.inverse() * cuboid_centre_w;
            if (x3Do(0) > s * w / 2 || x3Do(0) < -s * w / 2 ||
                x3Do(1) > s * h / 2 || x3Do(1) < -s * h / 2 ||
                x3Do(2) > s * l / 2 || x3Do(2) < -s * l / 2)
            {
                pMP->SetOutlierFlag();
                num_outliers++;
            }
        }
    }
    // Update object pose with pose computed by PCA, only for the very first few frames
    if (updatePose)
    {
        Eigen::Matrix4f Two = Eigen::Matrix4f::Identity();
        // GetRPYFromRotationMatrix(R);
        Two.topLeftCorner(3, 3) = 0.40 * l * R;
        // cout << R.determinant() << " " << endl;
        // cout << pow(T.topLeftCorner(3, 3).determinant(), 1./3) << endl;
        Two.topRightCorner(3, 1) = cuboid_centre_w;
        SetObjectPoseSim3(Two);

        // 使用PCA结果创建和保存一个关联的椭球体
    }
}

g2o::ellipsoid* MapObject::GetEllipsold()
{
    unique_lock<mutex> lock(mMutexObject);
    // TODO: 这里待解开，为何返回未定义的mpEllipsold会报错
    if (mpEllipsold == NULL) {
        // cout << "mpEllipsold == NULL" << endl;
        // cout << "This MapObject' mpEllipsold == NULL" << endl;
        return NULL;
    }
    else{
        return mpEllipsold;
    }
    // cout << "MapObject::GetEllipsold, Object_id = " << mnId << endl;
    // cout << "In MapObject, mpEllipsold->prob = " << mpEllipsold->prob << endl;
    // auto prob = mpEllipsold->prob;
    // return mpEllipsold;
}

void MapObject::SetPoseByEllipsold(g2o::ellipsoid* e)
{
    std::cout<<"[zhjd-debug] SetPoseByEllipsold"<<std::endl;
    Eigen::Matrix4f Two;

    {

    
    // cout << "In SetPoseByEllipsold, e->prob = " << e->prob << endl;
    if(mpEllipsold == NULL) {
        {
            // 这里遇到了一个死锁的问题
            unique_lock<mutex> lock(mMutexObject);
            mpEllipsold = new g2o::ellipsoid(*(e));
        }
        
        // mpEllipsold = e;
        // this->SetBadFlag();
        cout << "mpEllipsold->prob = " << mpEllipsold->prob << endl;
        
    }
    // 这里遇到了一个死锁的问题
    unique_lock<mutex> lock(mMutexObject);

    mbValidEllipsoldFlag = true;
    // else  
    cout << "MapObject::SetPoseByEllipsold, Object_id = " << mnId << endl;
    cout << "MapObject::SetPoseByEllipsold, mpEllipsold->prob = " << mpEllipsold->prob << endl;

    // SE3Quat pose;  // rigid body transformation, object in world coordinate
    // Vector3d scale; // a,b,c : half length of axis x,y,z

    // world -> object
    Two = Converter::toMatrix4f(e->pose);
    // cout << "Ellipsold->pose, Two = \n" << Two.matrix() << endl;

    Vector3d& scale = e->scale;
    float s = scale.norm() * 2;

    // Rx(90)*Ry(-90) 
    Eigen::Matrix3f Ron = Eigen::AngleAxisf(M_PI/2, Eigen::Vector3f(1,0,0)).matrix()
        * Eigen::AngleAxisf(-M_PI/2, Eigen::Vector3f(0,1,0)).matrix();
    Two.topLeftCorner(3, 3) = Two.topLeftCorner(3, 3) * Ron;

    // cout << "Two from ellipsold = " << Two.matrix() << endl;

    Two.topLeftCorner(3, 3) = 0.40 * s * Two.topLeftCorner(3, 3);


    w = e->scale(1) * 2;  // x
    h = e->scale(2) * 2;  // y
    l = e->scale(0) * 2;  // z

    // std::cout << "Setting scale = " << e->scale.transpose().matrix() << std::endl;
    // cout << "in setPoseByEllipsold: Two = \n" << Two.matrix() << endl;
    }
    SetObjectPoseSim3(Two); // Two
}

bool MapObject::AddDepthPointCloudFromObjectDetection(ObjectDetection* det)
{
    // // 这里可能还需要一次降采样操作
    unique_lock<mutex> lock(mMutexPointCloud);

    // std::cout << "mnId = " << mnId << std::endl;
    
    if (pcd_ptr == nullptr) {
        // std::cout << "pcd_ptr = nullptr" << std::endl;
        pcd_ptr = pcl::PointCloud<PointType>::Ptr(new pcl::PointCloud<PointType>);
        *pcd_ptr = *(det->pcd_ptr);
        mbValidPointCloudFlag = true;
    }
    else{
        // std::cout << "Merging .. " << std::endl;
        pcl::PointCloud<PointType>::Ptr mergedCloud(new pcl::PointCloud<PointType>);
        pcl::concatenate(*(det->pcd_ptr), *pcd_ptr, *mergedCloud);
        pcd_ptr->clear();
        det->pcd_ptr->clear();
        *pcd_ptr = *mergedCloud;
    }
    // 打印合并后的点云的大小
    std::cout << "Merged Cloud Size: " << pcd_ptr->size() << std::endl;
    // std::cout << "mnId = " << mnId << ", Merged Cloud Size: " << std::endl;

    mPoints = std::make_shared<PointCloud>(pclXYZToQuadricPointCloud(pcd_ptr));

    return true;
}

std::shared_ptr<PointCloud> MapObject::GetPointCloud()
{
    unique_lock<mutex> lock(mMutexPointCloud);
    return mPoints;
}

pcl::PointCloud<PointType>::Ptr MapObject::GetPointCloudPCL()
{
    return pcd_ptr;
}


void MapObject::AddMapPoints(MapPoint *pMP)
{
    unique_lock<mutex> lock(mMutexFeatures);
    map_points.insert(pMP);
}

void MapObject::EraseMapPoint(MapPoint *pMP)
{
    unique_lock<mutex> lock(mMutexFeatures);
    map_points.erase(pMP);
    pMP->SetBadFlag();
}


void MapObject::SetVelocity(const Eigen::Vector3f &v)
{
    unique_lock<mutex> lock(mMutexObject);
    velocity = v;
}

Eigen::Matrix4f MapObject::GetPoseSim3()
{
    unique_lock<mutex> lock(mMutexObject);
    return Sim3Two;
}

Eigen::Matrix4f MapObject::GetPoseSE3()
{
    unique_lock<mutex> lock(mMutexObject);
    return SE3Two;
}

Eigen::Matrix<float, 64, 1> MapObject::GetShapeCode()
{
    unique_lock<mutex> lock(mMutexObject);
    return vShapeCode;
}

int MapObject::GetRenderId()
{
    unique_lock<mutex> lock(mMutexObject);
    return mRenderId;
}

void MapObject::SetRenderId(int id)
{
    unique_lock<mutex> lock(mMutexObject);
    mRenderId = id;
}

void MapObject::SetDynamicFlag()
{
    unique_lock<mutex> lock(mMutexObject);
    mbDynamic = true;
}

bool MapObject::isDynamic()
{
    unique_lock<mutex> lock(mMutexObject);
    return mbDynamic;
}

}

