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

# include "Optimizer.h"
# include "Thirdparty/g2o/g2o/core/block_solver.h"
# include "Thirdparty/g2o/g2o/core/optimization_algorithm_levenberg.h"
# include "Thirdparty/g2o/g2o/solvers/linear_solver_eigen.h"
// # include "Thirdparty/g2o/g2o/types/types_six_dof_expmap.h"
# include "Thirdparty/g2o/g2o/core/robust_kernel_impl.h"
# include "Thirdparty/g2o/g2o/solvers/linear_solver_dense.h"
# include "Thirdparty/g2o/g2o/core/factory.h"
# include "Thirdparty/g2o/g2o/types/types_six_dof_expmap.h"


# include "ObjectPoseGraph.h"

#include "src/pca/EllipsoidExtractorEdges.h"
#include "include/core/ConstrainPlane.h"
// #include "include/core/SupportingPlane.h"
#include "src/plane/PlaneVertexEdges.h"

namespace ORB_SLAM2
{

G2O_REGISTER_TYPE(VERTEX_SE3:OBJ, VertexSE3Object);
G2O_REGISTER_TYPE(EDGE_SE3:LIE_ALGEBRA, EdgeSE3LieAlgebra);

int Optimizer::nBAdone = 0;

void Optimizer::GlobalJointBundleAdjustemnt(Map *pMap, int nIterations, bool *pbStopFlag, const unsigned long nLoopKF,
                                       const bool bRobust) {
    vector < KeyFrame * > vpKFs = pMap->GetAllKeyFrames();
    vector < MapPoint * > vpMP = pMap->GetAllMapPoints();
    vector < MapObject * > vpMO = pMap->GetAllMapObjects();
    JointBundleAdjustment(vpKFs, vpMP, vpMO, nIterations, pbStopFlag, nLoopKF, bRobust);
}

void Optimizer::JointBundleAdjustment(const vector<KeyFrame *> &vpKFs, const vector<MapPoint *> &vpMP,
                                      const std::vector<MapObject *> &vpMO,
                                      int nIterations, bool *pbStopFlag, const unsigned long nLoopKF,
                                      const bool bRobust) {
    vector<bool> vbNotIncludedMP;
    vbNotIncludedMP.resize(vpMP.size());
    vector<bool> vbNotIncludedMO;
    vbNotIncludedMO.resize(vpMO.size());

    g2o::SparseOptimizer optimizer;
    g2o::BlockSolver_6_3::LinearSolverType *linearSolver;
    linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>();
    g2o::BlockSolver_6_3 *solver_ptr = new g2o::BlockSolver_6_3(linearSolver);
    g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    optimizer.setAlgorithm(solver);

    if (pbStopFlag)
        optimizer.setForceStopFlag(pbStopFlag);

    long unsigned int maxKFid = 0;
    long unsigned int maxMPid = 0;

    // Set KeyFrame vertices
    for (size_t i = 0; i < vpKFs.size(); i++) {
        KeyFrame *pKF = vpKFs[i];
        if (pKF->isBad())
            continue;
        g2o::VertexSE3Expmap *vSE3 = new g2o::VertexSE3Expmap();
        vSE3->setEstimate(Converter::toSE3Quat(pKF->GetPose()));
        vSE3->setId(pKF->mnId);
        vSE3->setFixed(pKF->mnId == 0);
        optimizer.addVertex(vSE3);
        if (pKF->mnId > maxKFid)
            maxKFid = pKF->mnId;
    }

    const float thHuber2D = sqrt(5.99);
    const float thHuber3D = sqrt(7.815);
    const float invSigmaObject = 1e3;
    const float thHuberObject = sqrt(0.10 * invSigmaObject);
    const float thHuberObjectSquare = pow(thHuberObject, 2);

    // Set MapPoint vertices
    for (size_t i = 0; i < vpMP.size(); i++) {
        MapPoint *pMP = vpMP[i];
        if (pMP->isBad())
            continue;
        g2o::VertexSBAPointXYZ *vPoint = new g2o::VertexSBAPointXYZ();
        vPoint->setEstimate(Converter::toVector3d(pMP->GetWorldPos()));
        const int id = pMP->mnId + maxKFid + 1;
        vPoint->setId(id);
        vPoint->setMarginalized(true);
        optimizer.addVertex(vPoint);
        if (pMP->mnId > maxMPid)
            maxMPid = pMP->mnId;

        const map<KeyFrame *, size_t> observations = pMP->GetObservations();

        int nEdges = 0;
        //SET EDGES
        for (map<KeyFrame *, size_t>::const_iterator mit = observations.begin(); mit != observations.end(); mit++) {

            KeyFrame *pKF = mit->first;
            if (pKF->isBad() || pKF->mnId > maxKFid)
                continue;

            nEdges++;

            const cv::KeyPoint &kpUn = pKF->mvKeysUn[mit->second];

            if (pKF->mvuRight[mit->second] < 0) {
                Eigen::Matrix<double, 2, 1> obs;
                obs << kpUn.pt.x, kpUn.pt.y;

                g2o::EdgeSE3ProjectXYZ *e = new g2o::EdgeSE3ProjectXYZ();

                e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(id)));
                e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(pKF->mnId)));
                e->setMeasurement(obs);
                const float &invSigma2 = pKF->mvInvLevelSigma2[kpUn.octave];
                e->setInformation(Eigen::Matrix2d::Identity() * invSigma2);

                if (bRobust) {
                    g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(thHuber2D);
                }

                e->fx = pKF->fx;
                e->fy = pKF->fy;
                e->cx = pKF->cx;
                e->cy = pKF->cy;

                optimizer.addEdge(e);
            } else {
                Eigen::Matrix<double, 3, 1> obs;
                const float kp_ur = pKF->mvuRight[mit->second];
                obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

                g2o::EdgeStereoSE3ProjectXYZ *e = new g2o::EdgeStereoSE3ProjectXYZ();

                e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(id)));
                e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>(optimizer.vertex(pKF->mnId)));
                e->setMeasurement(obs);
                const float &invSigma2 = pKF->mvInvLevelSigma2[kpUn.octave];
                Eigen::Matrix3d Info = Eigen::Matrix3d::Identity() * invSigma2;
                e->setInformation(Info);

                if (bRobust) {
                    g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(thHuber3D);
                }

                e->fx = pKF->fx;
                e->fy = pKF->fy;
                e->cx = pKF->cx;
                e->cy = pKF->cy;
                e->bf = pKF->mbf;

                optimizer.addEdge(e);
            }
        }

        if (nEdges == 0) {
            optimizer.removeVertex(vPoint);
            vbNotIncludedMP[i] = true;
        } else {
            vbNotIncludedMP[i] = false;
        }
    }

    // Set MapObject Vertices
    for (size_t i = 0; i < vpMO.size(); i++) {
        auto pMO = vpMO[i];

        if (!pMO) {
            vbNotIncludedMO[i] = true;
            continue;
        }
        if (pMO->isDynamic() || pMO->isBad()) {
            vbNotIncludedMO[i] = true;
            continue;
        }


        g2o::VertexSE3Expmap *vSE3Obj = new g2o::VertexSE3Expmap();
        vSE3Obj->setEstimate(Converter::toSE3Quat(pMO->SE3Tow));
        int id = pMO->mnId + maxKFid + maxMPid + 2;
        vSE3Obj->setId(id);
        optimizer.addVertex(vSE3Obj);

        const map<KeyFrame *, size_t> observations = pMO->GetObservations();

        // Set Edges
        int nEdges = 0;
        for (auto observation : observations) {
            KeyFrame *pKFi = observation.first;

            // reject those frames after requesting stop
            if (pKFi->isBad() || pKFi->mnId > maxKFid)
                continue;
            // Get detections
            auto mvpObjectDetections = pKFi->GetObjectDetections();

            // cout << "EllipObject KF ID: " << pKFi->mnId << endl;
            EdgeSE3LieAlgebra *e = new EdgeSE3LieAlgebra();
            e->setVertex(0, optimizer.vertex(pKFi->mnId));
            e->setVertex(1, optimizer.vertex(id));
            auto det = mvpObjectDetections[observation.second];
            e->setMeasurement(Converter::toSE3Quat(det->SE3Tco));
            Eigen::Matrix<double, 6, 6> Info = Eigen::Matrix<double, 6, 6>::Identity();
            Info *= invSigmaObject;
            e->setInformation(Info);

            if (bRobust) {
                g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber;
                e->setRobustKernel(rk);
                rk->setDelta(thHuberObject);
            }

            optimizer.addEdge(e);
            nEdges++;
        }

        if (nEdges == 0) {
            optimizer.removeVertex(vSE3Obj);
            vbNotIncludedMO[i] = true;
        } else {
            vbNotIncludedMO[i] = false;
        }

    }

    // Optimize!
    optimizer.setVerbose(false);
    optimizer.initializeOptimization();
    optimizer.optimize(nIterations);

    // Recover optimized data

    //Keyframes
    for (size_t i = 0; i < vpKFs.size(); i++) {
        KeyFrame *pKF = vpKFs[i];
        if (pKF->isBad())
            continue;
        g2o::VertexSE3Expmap *vSE3 = static_cast<g2o::VertexSE3Expmap *>(optimizer.vertex(pKF->mnId));
        g2o::SE3Quat SE3quat = vSE3->estimate();
        if (nLoopKF == 0) {
            pKF->SetPose(Converter::toCvMat(SE3quat));
        } else {
            pKF->mTcwGBA.create(4, 4, CV_32F);
            Converter::toCvMat(SE3quat).copyTo(pKF->mTcwGBA);
            pKF->mnBAGlobalForKF = nLoopKF;
        }
    }

    //Points
    for (size_t i = 0; i < vpMP.size(); i++) {
        if (vbNotIncludedMP[i])
            continue;

        MapPoint *pMP = vpMP[i];

        if (pMP->isBad())
            continue;
        g2o::VertexSBAPointXYZ *vPoint = static_cast<g2o::VertexSBAPointXYZ *>(optimizer.vertex(
                pMP->mnId + maxKFid + 1));

        if (nLoopKF == 0) {
            pMP->SetWorldPos(Converter::toCvMat(vPoint->estimate()));
            pMP->UpdateNormalAndDepth();
        } else {
            pMP->mPosGBA.create(3, 1, CV_32F);
            Converter::toCvMat(vPoint->estimate()).copyTo(pMP->mPosGBA);
            pMP->mnBAGlobalForKF = nLoopKF;
        }
    }

    // Objects
    for (size_t i = 0; i < vpMO.size(); i++) {
        if (vbNotIncludedMO[i])
            continue;

        MapObject *pMO = vpMO[i];

        if (pMO->isBad())
            continue;
        if (pMO->isDynamic())
            continue;

        g2o::VertexSE3Expmap *vSE3Obj = static_cast<g2o::VertexSE3Expmap *>(optimizer.vertex(
                pMO->mnId + maxKFid + maxMPid + 2));
        g2o::SE3Quat SE3Tow = vSE3Obj->estimate();

        if (nLoopKF == 0) {
            Eigen::Matrix4f SE3Two = Converter::toMatrix4f(SE3Tow).inverse();
            pMO->SetObjectPoseSE3(SE3Two);
        } else {
            pMO->mTwoGBA = Converter::toMatrix4f(SE3Tow).inverse();
            pMO->mnBAGlobalForKF = nLoopKF;
        }
    }
}

void Optimizer::LocalJointBundleAdjustment(KeyFrame *pKF, bool *pbStopFlag, Map *pMap)
{
    // Local KeyFrames: First Breath Search from Current Keyframe
    list<KeyFrame*> lLocalKeyFrames;

    lLocalKeyFrames.push_back(pKF);
    pKF->mnBALocalForKF = pKF->mnId;

    const vector<KeyFrame*> vNeighKFs = pKF->GetVectorCovisibleKeyFrames();
    for(int i=0, iend=vNeighKFs.size(); i<iend; i++)
    {
        KeyFrame* pKFi = vNeighKFs[i];
        pKFi->mnBALocalForKF = pKF->mnId;
        if(!pKFi->isBad())
            lLocalKeyFrames.push_back(pKFi);
    }

//    cout << "No. Local Keyframes: " << lLocalKeyFrames.size() << endl;

    // Local MapPoints and MapObjects seen in Local KeyFrames
    list<MapPoint*> lLocalMapPoints;
    list<MapObject*> lLocalMapObjects;
    for(list<KeyFrame*>::iterator lit=lLocalKeyFrames.begin() , lend=lLocalKeyFrames.end(); lit!=lend; lit++)
    {
        vector<MapPoint*> vpMPs = (*lit)->GetMapPointMatches();
        for(vector<MapPoint*>::iterator vit=vpMPs.begin(), vend=vpMPs.end(); vit!=vend; vit++)
        {
            MapPoint* pMP = *vit;
            if(pMP)
            {
                if(!pMP->isBad())
                {
                    if (pMP->mnBALocalForKF != pKF->mnId)
                    {
                        lLocalMapPoints.push_back(pMP);
                        pMP->mnBALocalForKF = pKF->mnId;
                    }
                }
            }
        }

        vector<MapObject*> vpMOs = (*lit)->GetMapObjectMatches();
        for (auto pMO : vpMOs)
        {
            if (pMO)
            {
                if (pMO->mnBALocalForKF != pKF->mnId)
                {
                    lLocalMapObjects.push_back(pMO);
                    pMO->mnBALocalForKF = pKF->mnId;
                }
            }
        }
    }

    // Fixed Keyframes. Keyframes that see Local MapPoints but that are not Local Keyframes
    list<KeyFrame*> lFixedCameras;
    for(list<MapPoint*>::iterator lit=lLocalMapPoints.begin(), lend=lLocalMapPoints.end(); lit!=lend; lit++)
    {
        map<KeyFrame*,size_t> observations = (*lit)->GetObservations();
        for(map<KeyFrame*,size_t>::iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
        {
            KeyFrame* pKFi = mit->first;

            if(pKFi->mnBALocalForKF!=pKF->mnId && pKFi->mnBAFixedForKF!=pKF->mnId)
            {
                pKFi->mnBAFixedForKF=pKF->mnId;
                if(!pKFi->isBad())
                    lFixedCameras.push_back(pKFi);
            }
        }
    }

    // Setup optimizer
    g2o::SparseOptimizer optimizer;
    g2o::BlockSolver_6_3::LinearSolverType * linearSolver;
    linearSolver = new g2o::LinearSolverEigen<g2o::BlockSolver_6_3::PoseMatrixType>();
    g2o::BlockSolver_6_3 * solver_ptr = new g2o::BlockSolver_6_3(linearSolver);
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    optimizer.setAlgorithm(solver);

    if(pbStopFlag)
        optimizer.setForceStopFlag(pbStopFlag);

    unsigned long maxKFid = 0;
    std::set<long unsigned int> msKeyframeIDs;

    // Set Local KeyFrame vertices
    for(list<KeyFrame*>::iterator lit=lLocalKeyFrames.begin(), lend=lLocalKeyFrames.end(); lit!=lend; lit++)
    {
        KeyFrame* pKFi = *lit;
        g2o::VertexSE3Expmap * vSE3 = new g2o::VertexSE3Expmap();
        vSE3->setEstimate(Converter::toSE3Quat(pKFi->GetPose()));
        vSE3->setId(pKFi->mnId);
        msKeyframeIDs.insert(pKFi->mnId);
        // cout << "KF ID: " << pKFi->mnId << endl;
        vSE3->setFixed(pKFi->mnId==0);
        optimizer.addVertex(vSE3);
        if(pKFi->mnId>maxKFid)
            maxKFid=pKFi->mnId;
    }

    // Set Fixed KeyFrame vertices
    for(list<KeyFrame*>::iterator lit=lFixedCameras.begin(), lend=lFixedCameras.end(); lit!=lend; lit++)
    {
        KeyFrame* pKFi = *lit;
        g2o::VertexSE3Expmap * vSE3 = new g2o::VertexSE3Expmap();
        vSE3->setEstimate(Converter::toSE3Quat(pKFi->GetPose()));
        vSE3->setId(pKFi->mnId);
        msKeyframeIDs.insert(pKFi->mnId);
        vSE3->setFixed(true);
        optimizer.addVertex(vSE3);
        if(pKFi->mnId>maxKFid)
            maxKFid=pKFi->mnId;
    }

    // Set MapPoint vertices and edges
    const int nExpectedSize = (lLocalKeyFrames.size()+lFixedCameras.size())*lLocalMapPoints.size();

    vector<g2o::EdgeSE3ProjectXYZ*> vpEdgesMono;
    vpEdgesMono.reserve(nExpectedSize);
    vector<KeyFrame*> vpEdgeKFMono;
    vpEdgeKFMono.reserve(nExpectedSize);
    vector<MapPoint*> vpMapPointEdgeMono;
    vpMapPointEdgeMono.reserve(nExpectedSize);

    vector<g2o::EdgeStereoSE3ProjectXYZ*> vpEdgesStereo;
    vpEdgesStereo.reserve(nExpectedSize);
    vector<KeyFrame*> vpEdgeKFStereo;
    vpEdgeKFStereo.reserve(nExpectedSize);
    vector<MapPoint*> vpMapPointEdgeStereo;
    vpMapPointEdgeStereo.reserve(nExpectedSize);

    vector<EdgeSE3LieAlgebra*> vpEdgesCamObj;
    vector<KeyFrame*> vpEdgeKFCamObj;
    vector<MapObject*> vpMapObjectEdgeCamObj;

    const float thHuberMono = sqrt(5.991);
    const float thHuberStereo = sqrt(7.815);
    const float invSigmaObject = 1e3;
    const float thHuberObjectSquare = 1e3;
    const float thHuberObject = sqrt(thHuberObjectSquare);

    unsigned long maxMPid = 0;

    for(list<MapPoint*>::iterator lit=lLocalMapPoints.begin(), lend=lLocalMapPoints.end(); lit!=lend; lit++)
    {
        MapPoint* pMP = *lit;
        g2o::VertexSBAPointXYZ* vPoint = new g2o::VertexSBAPointXYZ();
        vPoint->setEstimate(Converter::toVector3d(pMP->GetWorldPos()));
        int id = pMP->mnId+maxKFid+1;
        vPoint->setId(id);
        vPoint->setMarginalized(true);
        optimizer.addVertex(vPoint);

        const map<KeyFrame*,size_t> observations = pMP->GetObservations();

        //Set edges
        for(map<KeyFrame*,size_t>::const_iterator mit=observations.begin(), mend=observations.end(); mit!=mend; mit++)
        {
            KeyFrame* pKFi = mit->first;

            if(!pKFi->isBad())
            {
                const cv::KeyPoint &kpUn = pKFi->mvKeysUn[mit->second];

                // Monocular observation
                if(pKFi->mvuRight[mit->second]<0)
                {
                    Eigen::Matrix<double,2,1> obs;
                    obs << kpUn.pt.x, kpUn.pt.y;

                    g2o::EdgeSE3ProjectXYZ* e = new g2o::EdgeSE3ProjectXYZ();

                    e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
                    e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKFi->mnId)));
                    e->setMeasurement(obs);
                    const float &invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave];
                    e->setInformation(Eigen::Matrix2d::Identity()*invSigma2);

                    g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(thHuberMono);

                    e->fx = pKFi->fx;
                    e->fy = pKFi->fy;
                    e->cx = pKFi->cx;
                    e->cy = pKFi->cy;

                    optimizer.addEdge(e);
                    vpEdgesMono.push_back(e);
                    vpEdgeKFMono.push_back(pKFi);
                    vpMapPointEdgeMono.push_back(pMP);

                    if(pMP->mnId > maxMPid)
                        maxMPid = pMP->mnId;
                }
                else // Stereo observation
                {
                    Eigen::Matrix<double,3,1> obs;
                    const float kp_ur = pKFi->mvuRight[mit->second];
                    obs << kpUn.pt.x, kpUn.pt.y, kp_ur;

                    g2o::EdgeStereoSE3ProjectXYZ* e = new g2o::EdgeStereoSE3ProjectXYZ();

                    e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(id)));
                    e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>(optimizer.vertex(pKFi->mnId)));
                    e->setMeasurement(obs);
                    const float &invSigma2 = pKFi->mvInvLevelSigma2[kpUn.octave];
                    Eigen::Matrix3d Info = Eigen::Matrix3d::Identity()*invSigma2;
                    e->setInformation(Info);

                    g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(thHuberStereo);

                    e->fx = pKFi->fx;
                    e->fy = pKFi->fy;
                    e->cx = pKFi->cx;
                    e->cy = pKFi->cy;
                    e->bf = pKFi->mbf;

                    optimizer.addEdge(e);
                    vpEdgesStereo.push_back(e);
                    vpEdgeKFStereo.push_back(pKFi);
                    vpMapPointEdgeStereo.push_back(pMP);

                    if(pMP->mnId > maxMPid)
                        maxMPid = pMP->mnId;
                }
            }
        }
    }

    // Set map object vertices and edges
    for (auto pMO : lLocalMapObjects)
    {
        if (!pMO->isDynamic())
        {
            g2o::VertexSE3Expmap *vSE3Obj = new g2o::VertexSE3Expmap();
            vSE3Obj->setEstimate(Converter::toSE3Quat(pMO->SE3Tow));
            int id = pMO->mnId + maxKFid + maxMPid + 2;
            vSE3Obj->setId(id);
            optimizer.addVertex(vSE3Obj);

            const map<KeyFrame*, size_t> observations = pMO->GetObservations();

            for (auto observation : observations)
            {
                KeyFrame* pKFi = observation.first;
                if (msKeyframeIDs.count(pKFi->mnId) == 0)
                    continue;

                if(!pKFi->isBad())
                {
                    auto mvpObjectDetections = pKFi->GetObjectDetections();
                    // cout << "EllipObject KF ID: " << pKFi->mnId << endl;
                    EdgeSE3LieAlgebra* e = new EdgeSE3LieAlgebra();
                    e->setVertex(0, optimizer.vertex(pKFi->mnId));
                    e->setVertex(1, optimizer.vertex(id));
                    auto det = mvpObjectDetections[observation.second];
                    e->setMeasurement(Converter::toSE3Quat(det->SE3Tco));
                    Eigen::Matrix<double, 6, 6> Info = Eigen::Matrix<double, 6, 6>::Identity();
                    Info*= invSigmaObject;
                    e->setInformation(Info);

                    g2o::RobustKernelHuber* rk = new g2o::RobustKernelHuber;
                    e->setRobustKernel(rk);
                    rk->setDelta(thHuberObject);

                    optimizer.addEdge(e);
                    vpEdgesCamObj.push_back(e);
                    vpEdgeKFCamObj.push_back(pKFi);
                    vpMapObjectEdgeCamObj.push_back(pMO);
                }
            }
        }
    }

    if(pbStopFlag)
    {
        if(*pbStopFlag)
        {
            // cout << "Local BA hasn't finished, but abort signal triggered!!!!!!!!!!!!!!!!!" << endl;
            return;
        }
    }

    optimizer.setVerbose(false);
    optimizer.initializeOptimization();
    optimizer.optimize(5);

    bool bDoMore= true;

    if(pbStopFlag)
    {
        if(*pbStopFlag)
        {
            // cout << "Local BA hasn't finished, but abort signal triggered!!!!!!!!!!!!!!!!!" << endl;
            return;
        }
    }

    if(bDoMore)
    {
        // Check inlier observations
        for(size_t i=0, iend=vpEdgesMono.size(); i<iend;i++)
        {
            g2o::EdgeSE3ProjectXYZ* e = vpEdgesMono[i];
            MapPoint* pMP = vpMapPointEdgeMono[i];

            if(pMP->isBad())
                continue;

            if(e->chi2()>5.991 || !e->isDepthPositive())
            {
                e->setLevel(1);
            }

            e->setRobustKernel(0);
        }

        for(size_t i=0, iend=vpEdgesStereo.size(); i<iend;i++)
        {
            g2o::EdgeStereoSE3ProjectXYZ* e = vpEdgesStereo[i];
            MapPoint* pMP = vpMapPointEdgeStereo[i];

            if(pMP->isBad())
                continue;

            if(e->chi2()>7.815 || !e->isDepthPositive())
            {
                e->setLevel(1);
            }

            e->setRobustKernel(0);
        }

        for(size_t i=0, iend=vpEdgesCamObj.size(); i<iend;i++)
        {
            auto e = vpEdgesCamObj[i];

            if(e->chi2() > thHuberObjectSquare)
            {
                e->setLevel(1);
            }
            e->setRobustKernel(0);
            // cout << "Edge " << vpEdgeKFCamObj[i]->mnId << "-" << vpMapObjectEdgeCamObj[i]->mnId << " Loss: " << e->chi2() << endl;
        }

        // Optimize again without the outliers
        optimizer.initializeOptimization(0);
        optimizer.optimize(10);

    }

    vector<pair<KeyFrame*, MapPoint*>> vToEraseCamPoints;
    vector<pair<KeyFrame*, MapObject*>> vToEraseCamObjects;
    vToEraseCamPoints.reserve(vpEdgesMono.size() + vpEdgesStereo.size());
    vToEraseCamObjects.reserve(vpEdgesCamObj.size());

    // Check inlier observations
    for(size_t i=0, iend=vpEdgesMono.size(); i<iend;i++)
    {
        g2o::EdgeSE3ProjectXYZ* e = vpEdgesMono[i];
        MapPoint* pMP = vpMapPointEdgeMono[i];

        if(pMP->isBad())
            continue;

        if(e->chi2()>5.991 || !e->isDepthPositive())
        {
            KeyFrame* pKFi = vpEdgeKFMono[i];
            vToEraseCamPoints.push_back(make_pair(pKFi,pMP));
        }
    }

    for(size_t i=0, iend=vpEdgesStereo.size(); i<iend;i++)
    {
        g2o::EdgeStereoSE3ProjectXYZ* e = vpEdgesStereo[i];
        MapPoint* pMP = vpMapPointEdgeStereo[i];

        if(pMP->isBad())
            continue;

        if(e->chi2()>7.815 || !e->isDepthPositive())
        {
            KeyFrame* pKFi = vpEdgeKFStereo[i];
            vToEraseCamPoints.push_back(make_pair(pKFi,pMP));
        }
    }

    for(size_t i=0, iend=vpEdgesCamObj.size(); i<iend;i++)
    {
        auto e = vpEdgesCamObj[i];
        MapObject* pMO = vpMapObjectEdgeCamObj[i];

        if(e->chi2() > thHuberObjectSquare)
        {
            KeyFrame* pKFi = vpEdgeKFCamObj[i];
            vToEraseCamObjects.push_back(make_pair(pKFi, pMO));
        }
    }

    // Get Map Mutex
    unique_lock<mutex> lock(pMap->mMutexMapUpdate);

    if(!vToEraseCamPoints.empty())
    {
        for(size_t i=0; i<vToEraseCamPoints.size(); i++)
        {
            KeyFrame* pKFi = vToEraseCamPoints[i].first;
            MapPoint* pMPi = vToEraseCamPoints[i].second;
            pKFi->EraseMapPointMatch(pMPi);
            pMPi->EraseObservation(pKFi);
        }
    }

    if (!vToEraseCamObjects.empty())
    {
        for(size_t i=0; i<vToEraseCamObjects.size(); i++)
        {
            KeyFrame* pKFi = vToEraseCamObjects[i].first;
            MapObject* pMO = vToEraseCamObjects[i].second;
            pKFi->EraseMapObjectMatch(pMO);
            pMO->EraseObservation(pKFi);
        }
    }

    // Recover optimized data

    //Keyframes
    for(list<KeyFrame*>::iterator lit=lLocalKeyFrames.begin(), lend=lLocalKeyFrames.end(); lit!=lend; lit++)
    {
        KeyFrame* pKF = *lit;
        g2o::VertexSE3Expmap* vSE3 = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(pKF->mnId));
        g2o::SE3Quat SE3quat = vSE3->estimate();
        pKF->SetPose(Converter::toCvMat(SE3quat));
    }

    //Points
    for(list<MapPoint*>::iterator lit=lLocalMapPoints.begin(), lend=lLocalMapPoints.end(); lit!=lend; lit++)
    {
        MapPoint* pMP = *lit;
        g2o::VertexSBAPointXYZ* vPoint = static_cast<g2o::VertexSBAPointXYZ*>(optimizer.vertex(pMP->mnId+maxKFid+1));
        pMP->SetWorldPos(Converter::toCvMat(vPoint->estimate()));
        pMP->UpdateNormalAndDepth();
    }

    //Objects
    for (auto pMO : lLocalMapObjects)
    {
        if (!pMO->isDynamic() && !pMO->isBad())
        {
            g2o::VertexSE3Expmap* vSE3Obj = static_cast<g2o::VertexSE3Expmap*>(optimizer.vertex(pMO->mnId + maxKFid + maxMPid + 2));
            g2o::SE3Quat SE3Tow = vSE3Obj->estimate();
            Eigen::Matrix4f SE3Two = Converter::toMatrix4f(SE3Tow).inverse();
            pMO->SetObjectPoseSE3(SE3Two);
        }
    }
    Optimizer::nBAdone++;

}

/**
 *   [关系加载]
 * 将 Relations / SupportingPlanes 加载到图中并返回边的指针数组.
 * 输入参数：
*/

// typedef std::vector<SupportingPlane> SupportingPlanes;

void LoadRelationsToGraph(Relations& relations, SupportingPlanes& supportingPlanes, 
            g2o::SparseOptimizer& graph, std::vector<g2o::VertexSE3Expmap*>& vSE3Vertex, std::vector<g2o::EdgePlaneSE3*>& vEdgePlaneSE3_)
{
    double config_plane_weight = Config::ReadValue<double>("DEBUG.PLANE.WEIGHT");
    cout << " * DEBUG-Plane weight : " << config_plane_weight << endl;

    // 仿造 mms, objs 的加载方式.
    int spl_num = supportingPlanes.size();
    std::map<int, g2o::VertexPlane3DOF *> mapPlaneVertices;
    std::vector<g2o::EdgePlaneSE3*> vEdgePlaneSE3;

    // 遍历支撑平面
    for(int sp_id=0; sp_id<spl_num; sp_id++ )        
    {
        SupportingPlane &spl = supportingPlanes[sp_id];
        int instance = spl.instance_id;
        g2o::plane* pCPlane = spl.pPlane;

        // 创建 plane 顶点，设置初始估计，设置不固定，设置顶点，并且使用一个map记录进行优化的平面
        g2o::VertexPlane3DOF *vPlane = new g2o::VertexPlane3DOF();
        vPlane->setEstimate(*pCPlane);
        vPlane->setId(graph.vertices().size());
        vPlane->setFixed(false);

        graph.addVertex(vPlane);
        mapPlaneVertices.insert(make_pair(instance, vPlane)); 

        // 开始添加平面的观测
        std::set<int>& relation_ids = spl.vRelation;
        for( auto iter=relation_ids.begin(); iter!=relation_ids.end(); iter++ )
        {
            int rl_id = *iter;
            Relation& rl = relations[rl_id];
            g2o::plane* plane_local = rl.pPlane;

            // 寻找 frame vertex
            int frame_index = rl.pFrame->frame_seq_id;
            auto vSE3 = vSE3Vertex[frame_index];

            if( plane_local == NULL ) continue;

            g2o::EdgePlaneSE3* pEdge = new g2o::EdgePlaneSE3;
            pEdge->setId(graph.edges().size());
            pEdge->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>( vPlane ));
            pEdge->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>( vSE3 ));
            pEdge->setMeasurement(*plane_local);

            Vector3d inv_sigma;
            inv_sigma << 1,1,1;
            inv_sigma = inv_sigma * config_plane_weight;
            MatrixXd info = inv_sigma.cwiseProduct(inv_sigma).asDiagonal();
            pEdge->setInformation(info);

            graph.addEdge(pEdge);
            vEdgePlaneSE3.push_back(pEdge);
               
        } // plane的观测: relations 遍历
    } // supporting planes 遍历

    // 保存输出
    vEdgePlaneSE3_ = vEdgePlaneSE3;
    return;
}

void Optimizer::SetGroundPlane(Vector4d& normal){
    mbGroundPlaneSet = true;
    mGroundPlaneNormal = normal;
}

const char* GetLabelText_Optimizer(int id)
{
    static const char *coco_classes[] = {"person","bicycle","car","motorcycle","airplane","bus","train",
    "truck","boat","traffic light","fire hydrant","stop sign","parking meter","bench","bird",
    "cat","dog","horse","sheep","cow","elephant","bear","zebra","giraffe","backpack","umbrella",
    "handbag","tie","suitcase","frisbee","skis","snowboard","sports ball","kite","baseball bat",
    "baseball glove","skateboard","surfboard","tennis racket","bottle","wine glass","cup","fork",
    "knife","spoon","bowl","banana","apple","sandwich","orange","broccoli","carrot","hot dog",
    "pizza","donut","cake","chair","couch","potted plant","bed","dining table","toilet","monitor",
    "laptop","mouse","remote","keyboard","cell phone","microwave","oven","toaster","sink",
    "refrigerator","book","clock","vase","scissors","teddy bear","hair drier","toothbrush"};
    if(id >= 0)
        return coco_classes[id];
    else 
        return "Unknown";
}

// TODO: 根据语义标签，若该物体是对称的，则赋予对称性质. 
// 目前该性质仅仅用于：
// 1) 判断旋转约束是否生效 
// 2) evo中是否评估其与gt的旋转error.
bool IsSymmetry(int label)
{
    static const std::set<std::string> symLabels = 
    {
        "vase", "bowl", "cup", "potted plant", "bottle"
    };
    const char* txtLabel = GetLabelText_Optimizer(label);
    if( symLabels.find(std::string(txtLabel)) != symLabels.end() )
        return true;
    else 
        return false;
}

// 输出函数
void OutputOptimizationEdgeErrors(Objects& objs, std::map<int, std::vector<g2o::EdgeSE3EllipsoidPlane*>>& mapInsEdgePlanes, 
    std::map<int, std::vector<g2o::EdgeSE3EllipsoidPlanePartial*>>& mapInsEdgePlanesPartial)
{
    ofstream out_obj("./optimization_edges.txt");
    for( int i=0;i<objs.size();i++)
    {
        EllipObject& obj = objs[i];
        int instance = obj.instance_id;

        std::vector<g2o::EdgeSE3EllipsoidPlane*>& vEdgesValid = mapInsEdgePlanes[instance];
        std::vector<g2o::EdgeSE3EllipsoidPlanePartial*>& vEdgesPartial = mapInsEdgePlanesPartial[instance];

        int num_valid = vEdgesValid.size();
        int num_partial = vEdgesPartial.size();
        int num_total = num_valid + num_partial;

        out_obj << "EllipObject " << instance << ", PlaneEdges " << num_total << "( V " << num_valid << ", P " << num_partial << " )" << std::endl;
        out_obj << " -> Valid edges : " << std::endl;
        for( int n =0;n<num_valid; n++)
        {
            g2o::EdgeSE3EllipsoidPlane* e = vEdgesValid[n];
            double error = e->error()(0,0);
            out_obj << "   v " << n << " : " << error << std::endl;
        }

        out_obj << std::endl;
        out_obj << " -> Partial edges : " << std::endl;
        for( int n =0;n<num_partial; n++)
        {
            g2o::EdgeSE3EllipsoidPlanePartial* e = vEdgesPartial[n];
            double error = e->error()(0,0);
            out_obj << "   p " << n << " : " << error << std::endl;
        }
        out_obj << std::endl;
    }

    out_obj.close();
}

void OutputInstanceObservationNum(std::map<int,int>& map)
{
    ofstream out_obj("./instance_observation_num.txt");

    for(auto pair : map)
    {
        out_obj << pair.first << " " << pair.second << std::endl;
    }
    out_obj.close();

    return ;
}


/*
*   [新版本] 基于切平面完成椭球体的全局优化!
*   10-4 Ours 论文发表使用的版本。
*/
void Optimizer::OptimizeWithDataAssociationUsingMultiplanes(std::vector<Frame *> &pFrames,
                Measurements& mms, Objects& objs, Trajectory& camTraj, const Matrix3d& calib, int iRows, int iCols) {
    // ************* 系统调试 ***************
    // 是否使用平面作为3D约束、是否使用法向约束、3D约束？
    bool bUsePlanesAs3DConstrains = true;
    bool bUseNormalConstrain = true; // 是否使用三维切平面的法向量约束

    double config_plane_angle_sigma = Config::Get<double>("Optimizer.Edges.3DConstrain.PlaneAngle.Sigma");
    
    // 注意 ： 开启了3d 平面的局部过滤. 30 pixel
    // 注意 ： 开启了2d 平面的局部过滤. param设置 30pixel. 只有非局部平面才添加约束.
    // *************************************

    // ************************ 加载优化相关的配置参数 ************************
    /**
     * 3D约束：使用单帧物体观测中椭球体包络立方体的平面进行约束
     * 2D约束：使用2D bbox 边线构成的平面进行约束
    */

    /**
     * - 椭球体3d scale, 2d scale
     * - 是否设置重力先验、重力先验scale、是否开启部分观测、是否使用3D概率
     * - 里程计权重
     * - 3D 2D 约束开关
    */

    double config_ellipsoid_3d_scale = Config::ReadValue<double>("Optimizer.Edges.3DEllipsoid.Scale");
    double config_ellipsoid_2d_scale = Config::ReadValue<double>("Optimizer.Edges.2D.Scale");
    // if(config_ellipsoid_2d_scale <= 0.01)
    //     config_ellipsoid_2d_scale = 1.0;

    bool mbSetGravityPrior = Config::Get<int>("Optimizer.Edges.GravityPrior.Open") == 1;  
    double dGravityPriorScale = Config::Get<double>("Optimizer.Edges.GravityPrior.Scale");
    bool mbOpenPartialObservation = Config::Get<int>("Optimizer.PartialObservation.Open") == 1;
    bool mbOpen3DProb = true;

    // 输出图片大小、优化参数
    std::cout << " -- Image : " << iCols << " x " << iRows << std::endl;
    std::cout << " -- Optimization parameters : " << std::endl;
    if(mbGroundPlaneSet)
        std::cout << " [ Using Ground Plane: " << mGroundPlaneNormal.transpose() << " ] " << std::endl;
    if(!mbGroundPlaneSet || !mbSetGravityPrior )   
        std::cout << " * Gravity Prior : closed." << std::endl;
    else
        std::cout << " * Gravity Prior : Open." << std::endl;
    cout<<" * Scale_3dedge: " << config_ellipsoid_3d_scale << endl;
    cout<<" * Scale_2dedge: " << config_ellipsoid_2d_scale << endl;
    cout<<" * Scale_GravityPrior: " << dGravityPriorScale << endl;
    cout<<" * config_plane_angle_sigma: " << config_plane_angle_sigma << endl;

    double config_odometry_weight = Config::ReadValue<double>("DEBUG.ODOM.WEIGHT");
    // double exp_config_odometry_weight = pow(10, config_odometry_weight);    //exp
    double exp_config_odometry_weight = config_odometry_weight; // normal
    cout << " * DEBUG-Odometry weight : " << exp_config_odometry_weight << endl;

    // 添加一个关闭 2D 约束的开关.
    bool mbClose2DConstrain = Config::Get<int>("Optimizer.Edges.2DConstrain.Close") > 0;
    if(mbClose2DConstrain)
        cout << "********** CLOSE 2D Constrain. **********" << endl;
    bool bOpen3DEllipsoidEdges = !(Config::Get<int>("Optimizer.Edges.3DConstrain.Close") > 0);
    if(!bOpen3DEllipsoidEdges)
        cout << "********** CLOSE 3D Constrain. **********" << endl;
    // ************************************************************************


    // Initialize variables.
    // 总共帧数、物体数量
    int total_frame_number = int(pFrames.size());
    int objects_num = int(objs.size());

    cout << total_frame_number << " frames, " << objects_num << " objects." << endl;

    // initialize graph optimization.
    // 创建g2o稀疏优化器、线性求解器、块求解器、LB算法，设置优化过程不输出
    g2o::SparseOptimizer graph;
    g2o::BlockSolverX::LinearSolverType* linearSolver;
    linearSolver = new g2o::LinearSolverDense<g2o::BlockSolverX::PoseMatrixType>();
    g2o::BlockSolverX * solver_ptr = new g2o::BlockSolverX(linearSolver);
    g2o::OptimizationAlgorithmLevenberg* solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
    graph.setAlgorithm(solver);
    graph.setVerbose(false);        // Set output.

    // 创建7Dof椭球体顶点、椭球投影边、顶点valid记录、椭球体重力平面先验、
    std::map<int, g2o::VertexEllipsoidXYZABCYaw*> vEllipsoidVertexMaps;
    std::vector<g2o::EdgeSE3EllipsoidProj*> edges, edgesValid, edgesInValid;
    std::vector<bool> validVec; validVec.resize(total_frame_number);
    std::vector<g2o::EdgeEllipsoidGravityPlanePrior *> edgesEllipsoidGravityPlanePrior;     // Gravity prior
    // 表示欧几里德空间中刚体变换的一种方式，该顶点在内部使用一个变换矩阵表示刚体变换，外部使用它的指数映射表示
    std::vector<g2o::VertexSE3Expmap*> vSE3Vertex;

    std::vector<g2o::EdgeSE3Ellipsoid7DOF*> vEllipsoid3DEdges;
    std::vector<g2o::EdgeSE3Expmap*> vEdgesOdom;

    std::vector<g2o::EdgeSE3EllipsoidPlane*> vEdgeEllipsoidPlane;
    std::vector<g2o::EdgeSE3EllipsoidPlane*> vEdgeEllipsoidPlane3D;
    std::vector<g2o::EdgeSE3EllipsoidPlaneWithNormal*> vEdgeEllipsoidPlane3DWithNormal;
    std::vector<g2o::EdgeSE3EllipsoidPlanePartial*> vEdgeEllipsoidPlanePartial;
    std::vector<g2o::EdgeSE3EllipsoidCenterFull*> vEdgeSE3EllipsoidCenterFull;

    std::map<int, std::vector<g2o::EdgeSE3EllipsoidPlane*>> mapInsEdgePlanes;       // 8-27 check : 该部分统计并没有考虑新的带 normal 的法向量
    std::map<int, std::vector<g2o::EdgeSE3EllipsoidPlanePartial*>> mapInsEdgePlanesPartial; 

    // Add SE3 vertices for camera poses
    // 为相机位姿添加SE3顶点
    bool bSLAM_mode = (Config::Get<int>("Optimizer.SLAM.mode") == 1);   // Mapping Mode : Fix camera poses and mapping ellipsoids only
    std::cout << " [ SLAM Mode : " << bSLAM_mode << " ] " << std::endl;
    
    std::map<int, int> frameId2FrameIndex;

    for( int frame_index = 0; frame_index < total_frame_number ; frame_index++) {

        int frame_id = pFrames[frame_index]->frame_seq_id;
        frameId2FrameIndex[frame_id] = frame_index;


        g2o::SE3Quat curr_cam_pose_Twc = pFrames[frame_index]->cam_pose_Twc;

        g2o::VertexSE3Expmap *vSE3 = new g2o::VertexSE3Expmap();
        vSE3->setId(graph.vertices().size());
        graph.addVertex(vSE3);
        vSE3->setEstimate(pFrames[frame_index]->cam_pose_Tcw); // Tcw
        if(!bSLAM_mode)
            vSE3->setFixed(true);       // Fix all the poses in mapping mode.
        else 
            vSE3->setFixed(frame_index == 0);
        
        vSE3Vertex.push_back(vSE3);

        // cout << "Add vSE3, frame_index = " << frame_index << endl;
        // cout << "vSE3Vertex.size() = " << vSE3Vertex.size() << endl;

        // if (vSE3 == nullptr) {
        //     cout << "vSE3 == nullptr" << endl;
        // }
        // else{
        //     cout << "vSE3 != nullptr" << endl;
        // }

        // cout << "pFrames[frame_index]->cam_pose_Tcw = "
        //      << pFrames[frame_index]->cam_pose_Tcw.toVector().transpose().matrix() << endl;


        // Add odom edges if in SLAM Mode
        if(bSLAM_mode && frame_index > 0){
            g2o::SE3Quat prev_cam_pose_Tcw = pFrames[frame_index-1]->cam_pose_Twc.inverse();
            g2o::SE3Quat curr_cam_pose_Tcw = curr_cam_pose_Twc.inverse();

            // 最后的 inverse: 将 wc 转为 cw
            // g2o::SE3Quat odom_val = (curr_cam_pose_Tcw*prev_cam_pose_Tcw.inverse()).inverse();; //  odom_wc * To = T1
            g2o::SE3Quat odom_val = curr_cam_pose_Tcw*prev_cam_pose_Tcw.inverse(); 

            g2o::EdgeSE3Expmap *e = new g2o::EdgeSE3Expmap();

            e->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex*>( vSE3Vertex[frame_index-1] ));
            e->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex*>( vSE3Vertex[frame_index] ));
            e->setMeasurement(odom_val);

            e->setId(graph.edges().size());
            Vector6d inv_sigma; inv_sigma << 1,1,1,1,1,1;
            inv_sigma = inv_sigma*exp_config_odometry_weight;

            Matrix6d info = inv_sigma.cwiseProduct(inv_sigma).asDiagonal();
            e->setInformation(info);
            graph.addEdge(e);

            vEdgesOdom.push_back(e);
        }
    }

    // 接下来： 以 mms, objs 构造图.
    // Initialize objects vertices and add edges of camera-objects 2d observations
    // 初始化物体顶点，添加相机-物体2D观测之间的边

    // 物体id，当前物体id，对称平面id，是否使用概率阈值，
    int objectid_in_edge = 0;
    int current_ob_id = 0;  
    int symplaneid_in_edge = 0;

    bool bUseProbThresh = Config::ReadValue<double>("Dynamic.Optimizer.UseProbThresh") > 0.01;
    double config_prob_thresh = Config::ReadValue<double>("Dynamic.Optimizer.EllipsoidProbThresh");
    int count_jump_prob_thresh = 0;

    if(bUseProbThresh)
        std::cout << "Use prob threshold, please make sure this is at least the second time running optimization!!" << std::endl;

    int count_partial_3d_planes = 0;
    int count_pointmodel_num = 0;
    std::vector<std::vector<g2o::ConstrainPlane*>> vwcPlanes;
    std::map<int,int> mmInstanceObservationNum;

    // 遍历物体，
    for(int object_id = 0; object_id < objects_num; object_id++ )        
    {
        EllipObject &obj = objs[object_id];
        int instance = obj.instance_id;
        int label = obj.pEllipsoid->miLabel;

        // 查语义表格判断该物体是否具有对称性质
        bool bSemanticSymmetry = IsSymmetry(label);

        if(bUseProbThresh)
        {
            // 因为obj中的概率需要在优化完之后做更新.
            if(obj.pEllipsoid->prob < config_prob_thresh) {
                count_jump_prob_thresh++;
                continue;
            }
        }

        // Add objects vertices
        // 添加物体顶点
        g2o::VertexEllipsoidXYZABCYaw *vEllipsoid = new g2o::VertexEllipsoidXYZABCYaw();
        vEllipsoid->setEstimate(*obj.pEllipsoid);
        vEllipsoid->setId(graph.vertices().size());
        vEllipsoid->setFixed(false);
        graph.addVertex(vEllipsoid);
        vEllipsoidVertexMaps.insert(make_pair(instance, vEllipsoid)); 

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
        //     edgesEllipsoidGravityPlanePrior.push_back(vGravityPriorEdge);
        // }
        // std::vector<g2o::ConstrainPlane*> vCPlanesWorld;
        // MatrixXd world_constrain_planes; world_constrain_planes.resize(0,4);


        // cout << "obj.measurementIDs.size() = " << obj.measurementIDs.size() << endl;
        
        // 添加 2d 和 3d 约束 : 来自三维立方体构造时所用的三维平面.
        for(int i = 0; i < obj.measurementIDs.size(); i++)
        {
            Measurement& measurement = mms[obj.measurementIDs[i]];
            instance = measurement.instance_id;
            
            Observation3D& ob_3d = measurement.ob_3d;
            g2o::ellipsoid* pObj_ob = ob_3d.pObj;
            
            if( pObj_ob == NULL ) continue;

            // if( i < 3 ) std::cout << "pObj_ob->prob : " << pObj_ob->prob << std::endl;

            int frame_id = measurement.ob_3d.pFrame->frame_seq_id;
            int frame_index = frameId2FrameIndex[frame_id];

            auto vEllipsoid = vEllipsoidVertexMaps[instance];
            auto vSE3 = vSE3Vertex[frame_index];

            std::vector<g2o::ConstrainPlane*> vCPlanes = pObj_ob->mvCPlanesInCamera;
            // MatrixXd mPlanesParam = pObj_ob->cplanes;
            int plane_num = vCPlanes.size();

            g2o::SE3Quat &Twc = measurement.ob_2d.pFrame->cam_pose_Twc;

            if(!mbClose2DConstrain && plane_num > 0){
                // 检查是否已经初始化过
                if (mapInsEdgePlanes.find(instance) == mapInsEdgePlanes.end()) {
                    mapInsEdgePlanes[instance] = std::vector<g2o::EdgeSE3EllipsoidPlane*>();
                }
                for( int i = 0; i < plane_num; i++)
                {
                    g2o::ConstrainPlane* pCPlane = vCPlanes[i];
                    Vector4d planeVec = pCPlane->pPlane->param.head(4); // local coordinate

                    // 为边缘平面添加特制的约束
                    if(pCPlane->valid){
                        g2o::EdgeSE3EllipsoidPlane* pEdge = new g2o::EdgeSE3EllipsoidPlane();
                        pEdge->setId(graph.edges().size());
                        pEdge->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>( vSE3 ));
                        pEdge->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>( vEllipsoid ));
                        pEdge->setMeasurement(planeVec);

                        Matrix<double,1,1> inv_sigma;
                        inv_sigma << 1 * config_ellipsoid_2d_scale;
                        MatrixXd info = inv_sigma.cwiseProduct(inv_sigma).asDiagonal();
                        pEdge->setInformation(info);
                        
                        // 设置鲁棒核函数，添加边到图中，记录边，
                        pEdge->setRobustKernel( new g2o::RobustKernelHuber() );

                        graph.addEdge(pEdge);

                        vEdgeEllipsoidPlane.push_back(pEdge);
                        mapInsEdgePlanes[instance].push_back(pEdge);
                        mmInstanceObservationNum[instance]++;
                    }

                    // update : 2020-7-3 
                    // 二维约束中的局部平面统一全部忽略.
                    // else
                    // {
                    //     // 局部平面
                    //     g2o::EdgeSE3EllipsoidPlanePartial* pEdge = new g2o::EdgeSE3EllipsoidPlanePartial;
                    //     pEdge->setId(graph.edges().size());
                    //     pEdge->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>( vSE3 ));
                    //     pEdge->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>( vEllipsoid ));
                    //     pEdge->setMeasurement(planeVec);
                    //     Matrix<double,1,1> inv_sigma;
                    //     inv_sigma << 1 * pObj_ob->prob * config_ellipsoid_2d_scale;
                    //     MatrixXd info = inv_sigma.cwiseProduct(inv_sigma).asDiagonal();
                    //     pEdge->setInformation(info);
                    //     pEdge->setRobustKernel( new g2o::RobustKernelHuber() );
                    //     if(mbOpenPartialObservation)    // 注意需要开放才生效
                    //         graph.addEdge(pEdge);
                    //     vEdgeEllipsoidPlanePartial.push_back(pEdge);
                    //     mapInsEdgePlanesPartial[instance].push_back(pEdge);
                    // } // valid plane
                } // plane 遍历

            } // 2D Constrain flag

            // 更新: 2020-6-17日, 添加3d-ellipsoid之间的约束.
            // if(bOpen3DEllipsoidEdges)
            // {
            //     // 局部3d观测:pObj_ob
            //     g2o::EdgeSE3EllipsoidCenterFull* pEdge = new g2o::EdgeSE3EllipsoidCenterFull;
            //     pEdge->setId(graph.edges().size());
            //     pEdge->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>( vSE3 ));
            //     pEdge->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>( vEllipsoid ));
            //     pEdge->setMeasurement(*pObj_ob);

            //     Matrix<double,1,1> inv_sigma;
            //     inv_sigma << 1 * pObj_ob->prob;
            //     MatrixXd info = inv_sigma.cwiseProduct(inv_sigma).asDiagonal();
            //     pEdge->setInformation(info);

            //     graph.addEdge(pEdge);
            //     vEdgeSE3EllipsoidCenterFull.push_back(pEdge);
            //     // mapInsEdgePlanes[instance].push_back(pEdge);
            // }

            // 更新： 切换成完整约束
            bool bPointModel = pObj_ob->bPointModel;    // 参与不能是点模型.
            if(bPointModel) count_pointmodel_num++; // 统计
            
            if(bOpen3DEllipsoidEdges && !bPointModel){
                // bool bUsePlanesAs3DConstrains = Config::Get<double>("Optimizer.Edges.3DEllipsoid.PlanesAsConstrains") > 0;

                // 使用切平面的3d约束
                // ****************** 7-3 更新代码: 添加未被遮挡的平面 ********************
                // 要不先测试简单的，所有平面都放进来. 取消三维约束.
                if(bUsePlanesAs3DConstrains)
                {
                    // std::vector<g2o::plane*> vecPlanes = pObj_ob->GetCubePlanes();
                    // 开始判断是否在界外
                    
                    std::vector<g2o::plane*> vecPlanes = pObj_ob->GetCubePlanesInImages(g2o::SE3Quat(),calib,iRows, iCols, 30);
                    // 看怎么封装好一点，把边缘的都给我去掉.

                    count_partial_3d_planes += (6 - vecPlanes.size());

                    // 将界内的边构建并加入到图优化中
                    // 先尝试普通平面边，之后再尝试带法向量约束的边.

                    for(int i=0;i<vecPlanes.size();i++)
                    {
                        // 取一个平面
                        g2o::plane* ppl = vecPlanes[i];
                        Vector4d planeVec = ppl->param;

                        if(bUseNormalConstrain)
                        {
                            int flag_valid_angle =  !bSemanticSymmetry ? 1 : 0;
                            g2o::EdgeSE3EllipsoidPlaneWithNormal* pEdge = new g2o::EdgeSE3EllipsoidPlaneWithNormal;
                            pEdge->setId(graph.edges().size());
                            pEdge->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>( vSE3 ));
                            pEdge->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>( vEllipsoid ));
                            pEdge->setMeasurement(planeVec);

                            Matrix<double,2,1> inv_sigma;
                            inv_sigma << 1, 1/(config_plane_angle_sigma * 1 / 180.0 * M_PI) * flag_valid_angle;   // 距离, 角度标准差 ; 暂时不管
                            inv_sigma = inv_sigma * config_ellipsoid_3d_scale;
                            MatrixXd info = inv_sigma.cwiseProduct(inv_sigma).asDiagonal();
                            pEdge->setInformation(info);
                            pEdge->setRobustKernel( new g2o::RobustKernelHuber() );

                            graph.addEdge(pEdge);
                            vEdgeEllipsoidPlane3DWithNormal.push_back(pEdge);

                            mmInstanceObservationNum[instance]++;
                        }
                        else // 不使用法向量约束
                        {
                            g2o::EdgeSE3EllipsoidPlane* pEdge = new g2o::EdgeSE3EllipsoidPlane;
                            pEdge->setId(graph.edges().size());
                            pEdge->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>( vSE3 ));
                            pEdge->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>( vEllipsoid ));
                            pEdge->setMeasurement(planeVec);

                            Matrix<double,1,1> inv_sigma;
                            inv_sigma << 1 * config_ellipsoid_3d_scale;
                            MatrixXd info = inv_sigma.cwiseProduct(inv_sigma).asDiagonal();
                            pEdge->setInformation(info);
                            pEdge->setRobustKernel( new g2o::RobustKernelHuber() );

                            graph.addEdge(pEdge);
                            vEdgeEllipsoidPlane3D.push_back(pEdge);
                            // mapInsEdgePlanes[instance].push_back(pEdge);
                        }
                    }
                }
                else 
                {
                    // 完整3d约束
                    g2o::EdgeSE3Ellipsoid7DOF* vEllipsoid3D = new g2o::EdgeSE3Ellipsoid7DOF; 
                    vEllipsoid3D->setId(graph.edges().size()); 
                    vEllipsoid3D->setVertex(0, dynamic_cast<g2o::OptimizableGraph::Vertex *>( vSE3 ));
                    vEllipsoid3D->setVertex(1, dynamic_cast<g2o::OptimizableGraph::Vertex *>( vEllipsoid ));                
                    vEllipsoid3D->setMeasurement(*pObj_ob); 

                    Vector7d inv_sigma;
                    inv_sigma << 1,1,1,1,1,1,1;
                    // if(mbOpen3DProb)
                    //     inv_sigma = inv_sigma * sqrt(pObj_ob->prob);
                    // Matrix7d info = inv_sigma.cwiseProduct(inv_sigma).asDiagonal() * config_ellipsoid_3d_scale * pObj_ob->prob;
                    Matrix7d info = inv_sigma.cwiseProduct(inv_sigma).asDiagonal() * config_ellipsoid_3d_scale * pObj_ob->prob;
                    vEllipsoid3D->setInformation(info);
                    vEllipsoid3D->setRobustKernel( new g2o::RobustKernelHuber() );

                    graph.addEdge(vEllipsoid3D);
                    vEllipsoid3DEdges.push_back(vEllipsoid3D);
                }
            }

            bool bOpenYawConstrain = false;
            // 旋转角度观测约束.

        
        } // 观测遍历

    } // objs 遍历

    /**
     * 是否对相关的平面进行优化
    */
    bool bOpenPlaneOptimization = Config::ReadValue<double>("Dynamic.Optimizer.OptimizeRelationPlane") > 0;
    if(bOpenPlaneOptimization){
        std::vector<g2o::EdgePlaneSE3*> vEdgeRelations;
        if(mbRelationLoaded){
            // 该函数加载所有平面观测到图优化中
            // cout << "CLOSE RELATION OPTIMIZATION." << endl;
            LoadRelationsToGraph(mRelations, mSupportingPlanes, graph, vSE3Vertex, vEdgeRelations);
            mbRelationLoaded = false;

            // Output Information of Relation Planes
            cout << " ----- Relation Planes -----" << endl;
            cout<<  "   - SupportingPlanes: " << mSupportingPlanes.size() << endl;
            cout<<  "   - Relations: " << mRelations.size() << endl;
            cout<<  "   - Edge Splane-Plane: " << vEdgeRelations.size() << endl;
            cout << endl << endl;
        }
    }

    /**
     * 输出优化器的相关信息
     * 
     * */ 
    int valid_plane_num = vEdgeEllipsoidPlane.size();
    int partial_plane_num = vEdgeEllipsoidPlanePartial.size();
    int total_plane_num = valid_plane_num + partial_plane_num;
    std::cout << std::endl << " ---- GRAPH INFORMATION ---- : " << std::endl;
    cout << " * EllipObject Num : " << objects_num << endl;
    cout<<  " * Vertices: "<<graph.vertices().size()<<endl;
    cout << "   - SE3: " << vSE3Vertex.size() << endl;
    cout << "   - EllipObject: " << vEllipsoidVertexMaps.size() << endl;
    cout << " * Edges: " << graph.edges().size() << endl;

    if(!mbOpenPartialObservation)
        cout << "   - Ellipsoid-Plane: " << total_plane_num << "( partial " << partial_plane_num << " [CLOSED])" << endl;
    else
        cout << "   - Ellipsoid-Plane: " << total_plane_num << "( partial " << partial_plane_num << " )" << endl;
    if(bOpen3DEllipsoidEdges){
        cout << "   - Ellipsoid-center: " << vEdgeSE3EllipsoidCenterFull.size() << std::endl;
        cout << "   - Ellipsoid-3D: " << vEllipsoid3DEdges.size() << std::endl;
        cout << "   - Ellipsoid-Plane-3D: " << vEdgeEllipsoidPlane3D.size() << "(filter partial:" << count_partial_3d_planes << ")" << std::endl;
        cout << "   - Ellipsoid-Plane-3DWithNormal: " << vEdgeEllipsoidPlane3DWithNormal.size() << "(filter partial:" << count_partial_3d_planes << ")" << std::endl;
    }
    cout << "   - Odom: " << vEdgesOdom.size() << endl;
    cout<<  "   - Gravity: " << edgesEllipsoidGravityPlanePrior.size() << endl;
    cout << " * EllipObject Measurements : " << mms.size() << endl;
    cout << "   -- Point-Model: " << count_pointmodel_num << endl;

    cout << endl;

    if(bUseProbThresh){
        std::cout << " * ProbThresh : " << config_prob_thresh << std::endl;
        std::cout << " * Jump objects : " << count_jump_prob_thresh << std::endl;
    }

    // nan check
    bool bOpenNanCheck = false;
    if(bOpenNanCheck)
    {
        std::cout << "Begin NaN checking ... " << std::endl;
        // odom  vEdgesOdom
        // 2d vEdgeEllipsoidPlane
        // 3d vEdgeEllipsoidPlane3DWithNormal

        int nan_count = 0;
        for(auto edge : vEdgesOdom)
        {
            edge->computeError();
            double chi2 = edge->chi2();
            if(std::isnan(chi2))
            {
                // 输出
                nan_count++;
            }
        }
        std::cout << "odom nan : " << nan_count << std::endl;

        nan_count = 0;
        for(auto edge : vEdgeEllipsoidPlane)
        {
            edge->computeError();
            double chi2 = edge->chi2();
            if(std::isnan(chi2))
            {
                // 输出
                nan_count++;
            }
        }
        std::cout << "2d plane nan : " << nan_count << std::endl;

        nan_count = 0;
        for(auto edge : vEdgeEllipsoidPlane3DWithNormal)
        {
            edge->computeError();
            double chi2 = edge->chi2();
            if(std::isnan(chi2))
            {
                // 输出
                nan_count++;
            }
        }
        std::cout << "3d plane with normal nan : " << nan_count << std::endl;
        
        std::cout << "[NAN_CHECK]Press any key to continue .. " << std::endl;
        getchar();
    }
    
    // 进行迭代优化
    int num_optimize = Config::Get<double>("Optimizer.Optimize.Num");
    if(graph.edges().size()>0){
        std::cout << "Begin Optimization..." << std::endl;
        graph.initializeOptimization();
        graph.optimize( num_optimize );  //optimization step
        std::cout << "Optimization done." << std::endl;

        // Update estimated ellispoids to the map
        // 更新结果到 objs
        for( int i = 0 ; i< objs.size() ; i++)
        {
            g2o::ellipsoid* pEllipsoid = objs[i].pEllipsoid;
            if(pEllipsoid!=NULL)
            {
                int instance = objs[i].instance_id;
                if(vEllipsoidVertexMaps.find(instance) == vEllipsoidVertexMaps.end()) continue;
                auto pEVertex = vEllipsoidVertexMaps[instance];
                
                bool colorSet = pEllipsoid->isColorSet();
                Vector4d old_color;
                if(colorSet) old_color = pEllipsoid->getColorWithAlpha();
                (*pEllipsoid) = pEVertex->estimate();
                pEllipsoid->miInstanceID = instance;
                if(colorSet) pEllipsoid->setColor(old_color.head(3), old_color[3]);   // 保持颜色不变.
            }
        }

        // 更新 Frames 轨迹; 也应该用临时结构体，而非直接改frame
        camTraj.resize(total_frame_number);
        for( int i=0; i< total_frame_number; i++ )
        {
            //  直接修改到帧中
            // 2020-6-2 让其生效. 引入 Save/Load 实现 debugging
            // Frame* pF = pFrames[i];
            // pF->cam_pose_Tcw = vSE3Vertex[i]->estimate();
            // pF->cam_pose_Twc = pF->cam_pose_Tcw.inverse();

            // 暂存版本
            SE3QuatWithStamp* pPoseTimestamp = new SE3QuatWithStamp;
            pPoseTimestamp->pose = vSE3Vertex[i]->estimate().inverse();  // Tcw -> Twc
            pPoseTimestamp->timestamp = pFrames[i]->mTimeStamp;
            camTraj[i] = pPoseTimestamp;
        }
    }
    else 
    {
        std::cout << " No observations valid." << std::endl;
    }

    // Output : 以物体为核心, 输出所有约束边对应的error
    OutputOptimizationEdgeErrors(objs, mapInsEdgePlanes, mapInsEdgePlanesPartial);
    OutputInstanceObservationNum(mmInstanceObservationNum);

    // Output optimization information.
    // object list
    ofstream out_obj("./object_list_nonparam.txt");
    auto iter = vEllipsoidVertexMaps.begin();
    for(;iter!=vEllipsoidVertexMaps.end();iter++)
    {
        out_obj << iter->first << "\t" << iter->second->estimate().toMinimalVector().transpose() 
            << "\t" << iter->second->estimate().miLabel << std::endl;
    }
    out_obj.close();

    return;
}

// 该函数即 Matlab 中 DPSample 函数
// Input: measurements    // 所有的观测，不会删除，以 index 索引.
// Input: objs              // 初始的时候，每个 mms 生成了一个objs.
// Input: model, 0 Ellipsoid, 1 Point
// Output: instanceObs
// void Optimizer::UpdateDataAssociation(Measurements& mms, Objects& objs, int model)
// {
//     double CONFIG_DP_alpha = Config::Get<double>("DataAssociation.DPAlpha");   // 默认值
//     // ********************************

//     std::map<int, Observations> output;  // 函数最后的计算结果，注意该存储方式本身就以 instance 为中心了. 
//     int total_meas_num = mms.size();
//     std::cout << "total_meas_num:" << total_meas_num << std::endl;

//     // ********************************
//     // 该部分变量生存周期： 每次迭代都进行访问
//     // ********************************

//     // 一些规则
//     // 1) instance 即决定了在 objs 中的存放顺序. 由于该instance可能动态增删，实际会变动，所以不能直接这样.
    
//     // 将 mms 从后往前遍历
//     std::vector<DAResult> daResults;
//     for( int k = mms.size()-1; k >= 0; k--)
//     {
//         Measurement& m = mms[k];
//         DAResult daResult;

//         if(m.ob_3d.pObj==NULL) continue;

//         int instance = m.instance_id;
//         int label = m.ob_2d.label;
//         bool bPointModel = m.ob_3d.pObj->bPointModel;

//         // 从对应物体列表取出该观测
//         int index = findInstanceIndex<Objects>(objs, instance); 
//         // assert( index >= 0 && "Can't find the instance in Objects.");

//         // 若与已有物体已经关联上，先取消该关联.
//         if(index >= 0 ) {
//             EllipObject& ob = objs[index];
//             ob.classVoter[label]--; // 该类别投票器减一
//             if((ob.measurementIDs.size()-1)<=0)   // 若减去该观测后，没有有效观测了
//             {
//                 bool result = deleteIndex<Objects>(objs, index);    // 删除该 instance, 以 index 为索引
//                 assert( result && "Delete an unexisted index.");
//             }
//             else 
//             {
//                 // 否则，在ob的存储中去除该观测id
//                 std::vector<int>::iterator iter=std::find(ob.measurementIDs.begin(),ob.measurementIDs.end(),k);
//                 ob.measurementIDs.erase(iter);
//             }
//         }

//         // ************************
//         // 基于观测内容计算其关联概率： 椭球体参数(位置/大小/旋转)，二维检测框，label标签
//         // ************************
//         // 计算该 measurement 的空间位置
//         g2o::SE3Quat frame_pos_Twc = m.ob_3d.pFrame->cam_pose_Twc;   // 此处 frame_pos 将随着优化而变化

//         // 依次计算与现有objects的关联概率.
//         std::vector<double> vec_posterior;
//         for(int obj_id = 0; obj_id < objs.size(); obj_id++)
//         {
//             EllipObject& ob_rh = objs[obj_id];
//             // 首先获得，每个物体，在观测label上投票器内的已有观测数量  dp_prior

//             int ob_num = ob_rh.measurementIDs.size();

//             // dp_prior :   m_i / (m_total + alpha),
//             //      由于分母部分所有物体都是一样，可以省去，该先验只与 m_i 即观测数量有关.
//             // double dp_prior = double(ob_num+1) / (double(total_meas_num+1) + config_real_dp_alpha); 
//             // 5-27更新: 分母部分对于所有比较对象都是常数，可以不予考虑。于是将其去掉。
//             // double dp_prior = double(ob_num+1);

//             // label_prob : 似然, 物体检测的标签部分.
//             // double label_prob = dirichlet(ob_rh.classVoter, ob_num, label);

//             // update 6-2日更新:
//             // 获得该 label 的观测数量
            
//             // ************* 原始代码 [Matlab] ****************
//             // dp_prior = p(obj.measurements.label(k),:);
//             // sq_dist = sum((obj_poses - repmat(pos,1,N)).^2,1);
//             // posterior = exp(-4*sq_dist).*dp_prior;
//             // ************************************************

//             auto iter = ob_rh.classVoter.find(label);
//             int label_num = 0;
//             if( iter != ob_rh.classVoter.end() ){
//                 label_num = iter->second;
//             }
//             label_num += 1; // at least one ob
//             // 将分母简化后， dp_prior 只与该label的观测数量有关
//             double dp_prior = double(label_num);

//             double prob_dis = 0;
//             if(model == 0)  // Ellipsoid
//                 prob_dis = calculateAssociationProbability(m, ob_rh);
//                 // prob_dis = calculateAssociationProbabilityUsingPoint(m, ob_rh); // 测试, 也使用点模型.
//             else if(model == 1) // Point
//                 prob_dis = calculateAssociationProbabilityUsingPoint(m, ob_rh);
//             else
//             {
//                 std::cout << "Error model value : " << model << ", must be 0: ellipsoid, 1: point." << std::endl;
//             }
//             double posterior = prob_dis * dp_prior;

//             // 接着取最大概率做处理.
//             vec_posterior.push_back(posterior);

//             // 保存结果
//             OneDAResult result;
//             result.measure_id = k;
//             result.object_id = obj_id;
//             result.posterior = posterior;
//             result.prob_dis = prob_dis;
//             result.prob_label = 1.0;    // useless , wait to be deleted.
//             result.dp_prior = dp_prior;
//             daResult.results.push_back(result);
//         }
//         // 找到最大的 posterior 及其 id
//         if(vec_posterior.size() != 0) {
//             auto biggest = std::max_element(std::begin(vec_posterior), std::end(vec_posterior));
//             int maxpiror_obj_id = std::distance(std::begin(vec_posterior), biggest);
//             double maxprior = *biggest;

//             daResult.measure_id = k;
//             daResult.result_id = maxpiror_obj_id;
//             // 最大后验是否满足配置要求
//             if(maxprior > CONFIG_DP_alpha)
//             {
//                 EllipObject& ob = objs[maxpiror_obj_id];
//                 // add measurement
//                 if( ob.classVoter.find(label) == ob.classVoter.end())
//                     ob.classVoter[label] = 1;  // 注意初始化时 label 一定要覆盖所有label?
//                 else 
//                     ob.classVoter[label]++;

//                 ob.measurementIDs.push_back(k);

//                 // 观测关联物体
//                 m.instance_id = ob.instance_id;

//                 daResult.object_id = maxpiror_obj_id;

//                 // 针对该物体标记“边缘无效边"
//                 // std::cout << "Begin check valid border." << std::endl;
//                 CheckValidBorder(ob, m);
//             }
//             else
//             {
//                 // 注意这类初始化下面还有一个
//                 if(!bPointModel){
//                     // 无可关联物体, 初始化一个新物体
//                     int instance_id = g_instance_total_id++;      // 这个必须是唯一 id.
//                     EllipObject ob_new;
//                     ob_new.instance_id = instance_id;
//                     ob_new.classVoter[label] = 1;
//                     ob_new.measurementIDs.push_back(k);

//                     // 分配新的椭球体内存空间.
//                     // 此处应该归给地图.
//                     ellipsoid* pe = new g2o::ellipsoid(m.ob_3d.pObj->transform_from(frame_pos_Twc));
//                     ob_new.pEllipsoid = pe;

//                     objs.push_back(ob_new);

//                     m.instance_id = instance_id;

//                     daResult.object_id = instance_id;
//                 }
//                 else
//                 {
//                     // 关联失败
//                     // 维持现状
//                     m.instance_id = -1; // 一定要恢复..
//                 }
                
//             }
//         }
//         else 
//         {                
//             // 无可关联物体, 初始化一个新物体
//             if(!bPointModel){
//                 int instance_id = g_instance_total_id++;      // 这个必须是唯一 id.
//                 EllipObject ob_new;
//                 ob_new.instance_id = instance_id;
//                 ob_new.classVoter[label] = 1;
//                 ob_new.measurementIDs.push_back(k);

//                 // 分配新的椭球体内存空间.
//                 // 此处应该归给地图.
//                 ellipsoid* pe = new g2o::ellipsoid(m.ob_3d.pObj->transform_from(frame_pos_Twc));

//                 // 1) 此处要对齐地面

//                 ob_new.pEllipsoid = pe;

//                 objs.push_back(ob_new);

//                 m.instance_id = instance_id;

//                 daResult.object_id = instance_id;
//             }
//             else
//             {
//                 m.instance_id = -1; // 一定要恢复..
//             }
//         }

//         daResults.push_back(daResult);
        
//     } // for : measurements

//     // output da Result
//     if(daResults.size() > 1)
//         OutputComplexDAResult(daResults, objs);
//     return;
// }

}