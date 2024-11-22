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

#include <LocalMapping.h>
#include <ORBmatcher.h>

using namespace std;

namespace ORB_SLAM2
{

/*
 * Tracking utils for stereo+lidar on KITTI
 */
void LocalMapping::MapObjectCulling()
{
    // Check Recent Added MapObjects
    /**
     * 检查最新添加的地图物体
    */

    list<MapObject*>::iterator lit = mlpRecentAddedMapObjects.begin();
    const unsigned long int nCurrentKFid = mpCurrentKeyFrame->mnId;

    const int cnThObs = 2;

    // Treat static and dynamic objects differently
    /**
     * 对静态和动态物体采用不同的处理：
     * （1) 动态物体：当超过两帧未观测到，则设置物体 Bad，从最近添加物体列表中剔除
     * （2) 物体isBad，剔除
     * （3）超过2帧未观测到,且物体观测数量大于等于cnThObs，剔除
     * （4) 超过3帧未观测到
    */
    while(lit != mlpRecentAddedMapObjects.end())
    {
        MapObject* pMO = *lit;
        if (pMO->isDynamic())
        {
            if ((int) nCurrentKFid - (int) pMO->mpNewestKF->mnId  >= 2)
            {
                pMO->SetBadFlag();
                lit = mlpRecentAddedMapObjects.erase(lit);
                mpMap->mnDynamicObj--;
            }
        }

        if(pMO->isBad())
        {
            lit = mlpRecentAddedMapObjects.erase(lit);
        }
        else if(((int)nCurrentKFid-(int)pMO->mnFirstKFid) >= 2 && pMO->Observations() <= cnThObs)
        {
            pMO->SetBadFlag();
            lit = mlpRecentAddedMapObjects.erase(lit);
        }
        else if(((int)nCurrentKFid-(int)pMO->mnFirstKFid) >= 3)
            lit = mlpRecentAddedMapObjects.erase(lit);
        else
            lit++;
    }

    // Dynamic objects that aren't recently added
    if (mpMap->mnDynamicObj > 0)
    {
        std::vector<MapObject*> pMOs = mpMap->GetAllMapObjects();
        for (MapObject *pMO : pMOs)
        {
            if (pMO->isDynamic())
            {
                if ((int) nCurrentKFid - (int) pMO->mpNewestKF->mnId  >= 2)
                {
                    pMO->SetBadFlag();
                    mpMap->mnDynamicObj--;
                }
            }
        }
    }
}

// KEY：这里有进行距离上的判断
void LocalMapping::GetNewObservations()
{
    PyThreadStateLock PyThreadLock;

    // cout << "LocalMapping: Estimating new poses for associated objects" << endl;

    auto Tcw = Converter::toMatrix4f(mpCurrentKeyFrame->GetPose());
    auto mvpAssociatedObjects = mpCurrentKeyFrame->GetMapObjectMatches();
    auto mvpObjectDetections = mpCurrentKeyFrame->GetObjectDetections();

    for (int i = 0; i < mvpObjectDetections.size(); i++)
    {
        auto det = mvpObjectDetections[i];
        if (det->isNew)
            continue;
        if (!det->isGood)
            continue;

        auto pMO = mvpAssociatedObjects[i];
        if (pMO)
        {
            // Tco obtained by transforming Two to camera frame
            Eigen::Matrix4f iniSE3Tco = Tcw * pMO->GetPoseSE3();
            g2o::SE3Quat Tco = Converter::toSE3Quat(iniSE3Tco);

            // Tco after running ICP, use Tco provided by detector
            // Eigen::Matrix4f SE3Tco = pyOptimizer.attr("estimate_pose_cam_obj")
            //         (det->SE3Tco, pMO->scale, det->SurfacePoints, pMO->GetShapeCode()).cast<Eigen::Matrix4f>();

            int class_id = pMO->label;

            py::object* optimizer_ptr;

            if(mmPyOptimizers.count(class_id) > 0) {
                py::object* optimizer_ptr_local = &(mmPyOptimizers[class_id]);
                optimizer_ptr = optimizer_ptr_local;
            }
            else{
                cout << "class " << class_id << " is not in yolo_classes" << endl;
                py::object* optimizer_ptr_local = &pyOptimizer;
                optimizer_ptr = optimizer_ptr_local;
            }
            
            Eigen::Matrix4f SE3Tco = optimizer_ptr->attr("estimate_pose_cam_obj")
                    (det->SE3Tco, pMO->scale, det->SurfacePoints, pMO->GetShapeCode()).cast<Eigen::Matrix4f>();
            g2o::SE3Quat Zco = Converter::toSE3Quat(SE3Tco);
            // error
            Eigen::Vector3f dist3D = SE3Tco.topRightCorner<3, 1>() - iniSE3Tco.topRightCorner<3, 1>();
            Eigen::Vector2f dist2D; dist2D << dist3D[0], dist3D[2];
            Eigen::Matrix<double , 6, 1> e = (Tco.inverse() * Zco).log();

            /**
             * 如果是动态物体，计算物体运动速度
             * 更新物体位姿、速度
            */
            if (pMO->isDynamic()) // if associated with a dynamic object
            {
                auto motion = pMO->SE3Tow * Tcw.inverse() * SE3Tco;
                float deltaT = (float)(mpCurrentKeyFrame->mnFrameId - mpLastKeyFrame->mnFrameId);
                auto speed = motion.topRightCorner<3, 1>() / deltaT;
                pMO->SetObjectPoseSE3(Tcw.inverse() * SE3Tco);
                pMO->SetVelocity(speed);
            }
            else // associated with a static object
            {
                /**
                 * 如果是静态物体，计算距离差值和位姿差值的范数是否在阈值范围内
                 * （1）是： 保持静止属性
                 * （2）否：如果变化很大，可能是动态物体/错误关联
                 *       - 如果观测数量小于等于2,设置为动态
                 *       - 否则设置该帧观测为新，去除该关键帧与该物体之间的关联
                */
                if (dist2D.norm() < 1.0 && e.norm() < 1.5) // if the change of translation is very small, then it really is a static object
                {
                    det->SetPoseMeasurementSE3(SE3Tco);
                }
                else // if change is large, it could be dynamic object or false association
                {
                    // If just observed, assume it is dynamic
                    if (pMO->Observations() <= 2)
                    {
                        pMO->SetDynamicFlag();
                        auto motion = pMO->SE3Tow * Tcw.inverse() * SE3Tco;
                        float deltaT = (float)(mpCurrentKeyFrame->mnFrameId - mpLastKeyFrame->mnFrameId);
                        auto speed = motion.topRightCorner<3, 1>() / deltaT;
                        pMO->SetObjectPoseSE3(Tcw.inverse() * SE3Tco);
                        pMO->SetVelocity(speed);
                        mpMap->mnDynamicObj++;
                    }
                    else
                    {
                        det->isNew = true;
                        mpCurrentKeyFrame->EraseMapObjectMatch(i);
                        pMO->EraseObservation(mpCurrentKeyFrame);
                    }
                }
            }
        }
    }
}

// 对 未关联上的观测进行物体创建
void LocalMapping::CreateNewMapObjects_stereo()
{
    PyThreadStateLock PyThreadLock;

    cout << "[zhjd-debug] LocalMapping: Started new objects creation" << endl;

    /**
     * 获取关键帧位姿、物体观测，遍历物体观测
     * 对 isNew 且 isGood 的观测进行新物体的创建
    */

    auto SE3Twc = Converter::toMatrix4f(mpCurrentKeyFrame->GetPoseInverse());
    auto mvpObjectDetections = mpCurrentKeyFrame->GetObjectDetections();
    cout << "[zhjd-debug] LocalMapping: mvpObjectDetections.size() = " << mvpObjectDetections.size() << endl;

    for (int i = 0; i < mvpObjectDetections.size(); i++)
    {
        // This might happen when a new KF is created in Tracking thread
        if (mbAbortBA)
            return;

        auto det = mvpObjectDetections[i];

        if (det->nRays == 0)
            continue;
        if (!det->isNew)
            continue;
        if (!det->isGood)
            continue;
        // auto pyMapObject = pyOptimizer.attr(">>>>reconstruct_object")
        //         (det->Sim3Tco, det->SurfacePoints, det->RayDirections, det->DepthObs);

        int class_id = det->label;

        py::object* optimizer_ptr;

        if(mmPyOptimizers.count(class_id) > 0) {
            py::object* optimizer_ptr_local = &(mmPyOptimizers[class_id]);
            optimizer_ptr = optimizer_ptr_local;
        }
        else{
            cout << "class " << class_id << " is not in yolo_classes" << endl;
            py::object* optimizer_ptr_local = &pyOptimizer;
            optimizer_ptr = optimizer_ptr_local;
        }

        auto pyMapObject = optimizer_ptr->attr("reconstruct_object")
                (det->Sim3Tco, det->SurfacePoints, det->RayDirections, det->DepthObs);

        if (!pyMapObject.attr("is_good").cast<bool>())
            continue;

        if (mbAbortBA)
            return;

        auto Sim3Tco = pyMapObject.attr("t_cam_obj").cast<Eigen::Matrix4f>();
        det->SetPoseMeasurementSim3(Sim3Tco);
        // Sim3, SE3, Sim3
        Eigen::Matrix4f Sim3Two = SE3Twc * Sim3Tco;
        auto code = pyMapObject.attr("code").cast<Eigen::Matrix<float, 64, 1>>();
        auto pNewObj = new MapObject(Sim3Two, code, mpCurrentKeyFrame, mpMap, class_id);

        // auto pyMesh = pyMeshExtractor.attr("extract_mesh_from_code")(code);

        py::object* mesh_extracter_ptr;

        if(mmPyOptimizers.count(class_id) > 0) {
            py::object* mesh_extracter_ptr_local = &(mmPyMeshExtractors[class_id]);
            mesh_extracter_ptr = mesh_extracter_ptr_local;
        }
        else{
            cout << " [LocalMapping_util.cc 265] class " << class_id << " is not in yolo_classes" << endl;
            py::object* mesh_extracter_ptr_local = &pyMeshExtractor;
            mesh_extracter_ptr = mesh_extracter_ptr_local;
        }

        auto pyMesh = mesh_extracter_ptr->attr("extract_mesh_from_code")(code);


        pNewObj->vertices = pyMesh.attr("vertices").cast<Eigen::MatrixXf>();
        pNewObj->faces = pyMesh.attr("faces").cast<Eigen::MatrixXi>();

        pNewObj->AddObservation(mpCurrentKeyFrame, i);
        mpCurrentKeyFrame->AddMapObject(pNewObj, i);
        mpMap->AddMapObject(pNewObj);
        mpObjectDrawer->AddObject(pNewObj);
        cout << "mpObjectDrawer->AddObject" << endl;
        mlpRecentAddedMapObjects.push_back(pNewObj);
    }
    cout << "[zhjd-debug] LocalMapping: Finished new objects creation" << endl;
}

/*
 * Tracking utils for monocular input on Freiburg Cars and Redwood OS
 */
void LocalMapping::CreateNewObjectsFromDetections()
{
    /*这一帧中进行新物体的创建*/

    cout << "\n[ LocalMapping - CreateNewObjectsFromDetections ]" << endl;

    // Step 1: 获取当前帧的旋转、平移和物体检测
    // cv::Mat Rcw = mpCurrentKeyFrame->GetRotation();
    // cv::Mat tcw = mpCurrentKeyFrame->GetTranslation();
    auto mvpObjectDetections = mpCurrentKeyFrame->GetObjectDetections();
    // std::cout << " => " << "KF" << mpCurrentKeyFrame->mnId << ", mvpObjectDetections.size() = " << mvpObjectDetections.size() << std::endl;

    // Step 2: 遍历检测
    // Create new objects first, otherwise data association might fail
    for (int det_i = 0; det_i < mvpObjectDetections.size(); det_i++)
    {
        // std::cout << "\n=> det_i : " << det_i << std::endl;
        // Step 2.1: 如果该检测如果已经与物体关联/关键点过少，则continue
        auto det = mvpObjectDetections[det_i];
        int class_id = det->label;

        // If the detection is a new object, create a new map object.
        // todo: 这里原本根据点云数量进行判断是否创建物体

        if (!det->isNew) {
            std::cout << "continue because !det->isNew" << std::endl;
            continue;
        }
        
        if (!det->isGood && !add_depth_pcd_to_map_object) {
            std::cout << "continue because !det->isGood && !add_depth_pcd_to_map_object" << std::endl;
            continue;
        }

        // Step 2.2: 创建地图物体，向当前帧、地图中进行添加，向该物体添加关联地图点
        auto pNewObj = new MapObject(mpCurrentKeyFrame, mpMap, class_id);
        mpCurrentKeyFrame->AddMapObject(pNewObj, det_i);
        mpMap->AddMapObject(pNewObj);

        // New add
        det->isNew = false;
        auto mvpMapPoints = mpCurrentKeyFrame->GetMapPointMatches();
        int n_valid_points = 0;
        for (int k_i : det->GetFeaturePoints())
        {
            auto pMP = mvpMapPoints[k_i];
            if (!pMP)
                continue;
            if (pMP->isBad())
                continue;
            pMP->in_any_object = true;
            pMP->object_id = pNewObj->mnId;
            pMP->keyframe_id_added_to_object = int(mpCurrentKeyFrame->mnId);
            pNewObj->AddMapPoints(pMP);
            n_valid_points++;
        }
        // std::cout << "n_valid_points = " << n_valid_points << std::endl;
        // std::cout << "det->isNew" << det->isNew << std::endl;

        if (create_single_object)
        {
            // todo: 这里只处理了一个结果
            return;  // for mono sequences, we only focus on the single object in the middle
        }
    }
}


void LocalMapping::ProcessDetectedObjects_origin()
{
    auto SE3Twc = Converter::toMatrix4f(mpCurrentKeyFrame->GetPoseInverse());
    auto SE3Tcw = Converter::toMatrix4f(mpCurrentKeyFrame->GetPose());
    cv::Mat Rcw = mpCurrentKeyFrame->GetRotation();
    cv::Mat tcw = mpCurrentKeyFrame->GetTranslation();
    auto mvpObjectDetections = mpCurrentKeyFrame->GetObjectDetections();
    auto mvpAssociatedObjects = mpCurrentKeyFrame->GetMapObjectMatches();

    for (int det_i = 0; det_i < mvpObjectDetections.size(); det_i++)
    {
        auto det = mvpObjectDetections[det_i];

        // If the detection is associated with an existing map object, we consider 2 different situations:
        // 1. the object has been reconstructed: update observations 2. the object has not been reconstructed:
        // check if it's ready for reconstruction, reconstruct if it's got enough points
        if (det->isNew)
            continue;
        if (!det->isGood)
            continue;

        MapObject *pMO = mvpAssociatedObjects[det_i];
        if (!pMO)
            continue;
        // We only consider the object in the middle
        if (pMO->mnId != 0)
            continue;

        int numKFsPassedSinceInit = int(mpCurrentKeyFrame->mnId - pMO->mpRefKF->mnId);

        if (numKFsPassedSinceInit < 50)
            pMO->ComputeCuboidPCA(numKFsPassedSinceInit < 15);
        else  // when we have relative good object shape
            pMO->RemoveOutliersModel();
        // only begin to reconstruct the object if it is observed for enough amoubt of time (15 KFs)
        if(numKFsPassedSinceInit < 15)
            continue;

        if ((numKFsPassedSinceInit - 15) % 5 != 0)
            continue;

//        int numKFsPassedSinceLastRecon = int(mpCurrentKeyFrame->mnId) - nLastReconKFID;
//        if (numKFsPassedSinceLastRecon  < 8)
//            continue;

        std::vector<MapPoint*> points_on_object = pMO->GetMapPointsOnObject();
        int n_points = points_on_object.size();
        int n_valid_points = 0;
        for (auto pMP : points_on_object)
        {
            if (!pMP)
                continue;
            if (pMP->isBad())
                continue;
            if (pMP->isOutlier())
                continue;
            n_valid_points++;
        }

        int n_rays = 0;
        auto map_points_vector = mpCurrentKeyFrame->GetMapPointMatches();
        for (auto idx : det->GetFeaturePoints())
        {
            auto pMP = map_points_vector[idx];
            if (!pMP)
                continue;
            if (pMP->isBad())
                continue;
            if (pMP->object_id != pMO->mnId)
                continue;
            if (pMP->isOutlier())
                continue;
            n_rays++;
        }
        // cout << "Object " << pMO->mnId << ": " << n_points << " points observed, " << "with " << n_valid_points << " valid points, and " << n_rays << " rays" << endl;

        // Surface points
        if (n_valid_points >= 50 && n_rays > 20)
        {
            Eigen::MatrixXf surface_points_cam = Eigen::MatrixXf::Zero(n_valid_points, 3);
            int p_i = 0;
            for (auto pMP : points_on_object)
            {
                if (!pMP)
                    continue;
                if (pMP->isBad())
                    continue;
                if (pMP->isOutlier())
                    continue;

                cv::Mat x3Dw = pMP->GetWorldPos();
                cv::Mat x3Dc = Rcw * x3Dw + tcw;
                float xc = x3Dc.at<float>(0);
                float yc = x3Dc.at<float>(1);
                float zc = x3Dc.at<float>(2);
                surface_points_cam(p_i, 0) = xc;
                surface_points_cam(p_i, 1) = yc;
                surface_points_cam(p_i, 2) = zc;
                p_i++;
            }

            // Rays
            Eigen::MatrixXf ray_pixels = Eigen::MatrixXf::Zero(n_rays, 2);
            Eigen::VectorXf depth_obs = Eigen::VectorXf::Zero(n_rays);
            int k_i = 0;
            for (auto point_idx : det->GetFeaturePoints())
            {
                auto pMP = map_points_vector[point_idx];
                if (!pMP)
                    continue;
                if(pMP->isBad())
                    continue;
                if(pMP->object_id != pMO->mnId)
                    continue;
                if (pMP->isOutlier())
                    continue;

                cv::Mat x3Dw = pMP->GetWorldPos();
                cv::Mat x3Dc = Rcw * x3Dw + tcw;
                depth_obs(k_i) = x3Dc.at<float>(2);
                ray_pixels(k_i, 0) = mpCurrentKeyFrame->mvKeysUn[point_idx].pt.x;
                ray_pixels(k_i, 1 ) = mpCurrentKeyFrame->mvKeysUn[point_idx].pt.y;
                k_i++;
            }

            Eigen::MatrixXf u_hom(n_rays, 3);
            u_hom << ray_pixels, Eigen::MatrixXf::Ones(n_rays, 1);
            Eigen::MatrixXf fg_rays(n_rays, 3);
            Eigen::Matrix3f invK = Converter::toMatrix3f(mpTracker->GetCameraIntrinsics()).inverse();
            for (int i = 0; i  < n_rays; i++)
            {
                auto x = u_hom.row(i).transpose();
                fg_rays.row(i) = (invK * x).transpose();
            }
            Eigen::MatrixXf rays(fg_rays.rows() + det->background_rays.rows(), 3);
            rays << fg_rays, det->background_rays;

            PyThreadStateLock PyThreadLock;
            auto pyMapObject = pyOptimizer.attr("reconstruct_object")
                    (SE3Tcw * pMO->Sim3Two, surface_points_cam, rays, depth_obs, pMO->vShapeCode);

            // cout << "Number of KF passed: " << numKFsPassedSinceInit << endl;

            // If not initialized, duplicate optimization to resolve orientation ambiguity
            if (!pMO->reconstructed)
            {
                auto flipped_Two = pMO->Sim3Two;
                flipped_Two.col(0) *= -1;
                flipped_Two.col(2) *= -1;
                auto pyMapObjectFlipped = pyOptimizer.attr("reconstruct_object")
                        (SE3Tcw * flipped_Two, surface_points_cam, rays, depth_obs, pMO->vShapeCode);

                if (pyMapObject.attr("loss").cast<float>() > pyMapObjectFlipped.attr("loss").cast<float>())
                    pyMapObject = pyMapObjectFlipped;
            }

            auto Sim3Tco = pyMapObject.attr("t_cam_obj").cast<Eigen::Matrix4f>();
            det->SetPoseMeasurementSim3(Sim3Tco);
            // Sim3, SE3, Sim3
            Eigen::Matrix4f Sim3Two = SE3Twc * Sim3Tco;
            int code_len = pyOptimizer.attr("code_len").cast<int>();
            Eigen::Matrix<float, 64, 1> code = Eigen::VectorXf::Zero(64);
            if (code_len == 32)
            {
                auto code_32 = pyMapObject.attr("code").cast<Eigen::Matrix<float, 32, 1>>();
                code.head(32) = code_32;
            }
            else
            {
                code = pyMapObject.attr("code").cast<Eigen::Matrix<float, 64, 1>>();
            }

            pMO->UpdateReconstruction(Sim3Two, code);
            auto pyMesh = pyMeshExtractor.attr("extract_mesh_from_code")(code);
            pMO->vertices = pyMesh.attr("vertices").cast<Eigen::MatrixXf>();
            pMO->faces = pyMesh.attr("faces").cast<Eigen::MatrixXi>();
            pMO->reconstructed = true;
            pMO->AddObservation(mpCurrentKeyFrame, det_i);
            mpCurrentKeyFrame->AddMapObject(pMO, det_i);
            mpObjectDrawer->AddObject(pMO);
            mlpRecentAddedMapObjects.push_back(pMO);

            nLastReconKFID = int(mpCurrentKeyFrame->mnId);
        }
    }
}


void LocalMapping::ProcessDetectedObjects()
{
    std::cout << "\n[zhjd-debug] [ LocalMapping - ProcessDetectedObjects ]" << std::endl;
    char key;
    // std::cout << "Ready to reconstruct_object" << std::endl;
    // std::cout << "Press [ENTER] to continue ... " << std::endl;
    // key = getchar();
    
    /** 获取当前关键帧的位姿信息、物体检测和地图物体 */
    auto SE3Twc = Converter::toMatrix4f(mpCurrentKeyFrame->GetPoseInverse());
    auto SE3Tcw = Converter::toMatrix4f(mpCurrentKeyFrame->GetPose());
    cv::Mat Rcw = mpCurrentKeyFrame->GetRotation();
    cv::Mat tcw = mpCurrentKeyFrame->GetTranslation();
    auto mvpObjectDetections = mpCurrentKeyFrame->GetObjectDetections();
    auto mvpAssociatedObjects = mpCurrentKeyFrame->GetMapObjectMatches();

    auto mvpGlobalEllipsolds = mpCurrentKeyFrame->GetEllipsoldsGlobal();

    std::cout << " => " << "KF" << mpCurrentKeyFrame->mnId << ", mvpObjectDetections.size() = " << mvpObjectDetections.size() << std::endl;

    // 处理当前关键帧的所有detection
    for (int det_i = 0; det_i < mvpObjectDetections.size(); det_i++)
    {
        std::cout << "\n=> det_i " << det_i+1 << "/" << mvpObjectDetections.size() << std::endl;
        // std::cout << "Press [ENTER] to continue ... " << std::endl;
        // key = getchar();

        auto det = mvpObjectDetections[det_i];
        MapObject *pMO = mvpAssociatedObjects[det_i];
        

        // If the detection is associated with an existing map object, we consider 2 different situations:
        // 1. 物体已经被重建： 更新观测
        // 2. 物体尚未被重建： 检查是否准备好重建，如果有足够多点则进行重建
        
        /**
         * 如果:
         *   (1) 该检测尚未与地图物体关联 
         *   (2) 该检测包含的特征点数量较少
         *   (3) 该检测关联的物体为NULL
         * 则 continue
        */

        if (show_ellipsold_process && det_i)
        {
            MapObject *pMO_last = mvpAssociatedObjects[det_i - 1];
            if (!pMO_last)
                cout << "pMO == null" << endl;
            else
                cout << "Object " << pMO_last->mnId << ": \n" << pMO_last->Sim3Two.matrix() << endl;


            auto det_vec = mvpObjectDetections[det_i-1]->bbox;
            int x1 = (int)det_vec(0), y1 = (int)det_vec(1), x2 = (int)det_vec(2), y2 = (int)det_vec(3);
            // cv::Mat img_show = pFrame->rgb_img.clone();
            cv::Mat img_show(cam_height, cam_width, CV_8UC3, cv::Scalar(255, 255, 255));
            cv::rectangle(img_show, cv::Point(x1, y1), cv::Point(x2, y2), cv::Scalar(255, 0, 0), 2);  // Scalar(255, 0, 0) is for blue color, 2 is the thickness
            cv::imshow("Image with Bbox", img_show);
            // cv::waitKey(0);
            cv::waitKey(10);
            cout << "Press any key to continue" << endl;
            char key = getchar();
        }

        if (det->isNew) {
            std::cout << "  Conitinue because det->isNew" << std::endl;
            continue;
        }

        // 这里的中间判据有待改进
        if (!det->isGood && !add_depth_pcd_to_map_object ) {
            std::cout << "  Conitinue because !det->isGood && !add_depth_pcd_to_map_object" << std::endl;
            continue;
        }

        if (!det->isValidPcd && add_depth_pcd_to_map_object) {
            std::cout << "  Conitinue because !det->isValidPcd && add_depth_pcd_to_map_object" << std::endl;
            continue;
        }

        if (!pMO) {
            std::cout << "  Conitinue because !pMO" << std::endl;
            continue;
        }

        /** 这里人为规定了只考虑编号为0（中间的）的物体 */
        if (pMO->mnId != 0 && create_single_object) {
            std::cout << "  Conitinue because pMO->mnId != 0" << std::endl;
            continue;
        }

        std::cout << "pMO->mnId = " << pMO->mnId << std::endl;

        // 把深度点云加到地图物体中
        if (add_depth_pcd_to_map_object) {
            pMO->AddDepthPointCloudFromObjectDetection(det);
            // cout << "AddDepthPointCloudFromObjectDetection" << endl;
        }

        // std::cout << "use_ellipsold_pose_for_shape_optimization = " << \
        //     use_ellipsold_pose_for_shape_optimization << std::endl;

        int numKFsPassedSinceInit = int(mpCurrentKeyFrame->mnId - pMO->mpRefKF->mnId);

        std::cout << "pMO's mpRefKF / curKF / passedKF = " \
                  << pMO->mpRefKF->mnId << " / " \
                  << mpCurrentKeyFrame->mnId << " / " << numKFsPassedSinceInit << std::endl;

        // bool isShortInternal = numKFsPassedSinceInit < 15;
        // std::string condition_str = "numKFsPassedSinceInit < 15";

        bool isShortInternal = numKFsPassedSinceInit < 15 and (mpCurrentKeyFrame->mnId >= 3);
        std::string condition_str = "numKFsPassedSinceInit < 15 and (mpCurrentKeyFrame->mnId >= 3)";

        /**
         * 在物体被初始化的时间间隔的过程中：
         * （1) 少于 50 帧（尚未获得较好形状）：持续进行初始位姿设置
         * （2) 大于 50 帧（有较好形状）：仅进行外点剔除
         * 
         *  
         * 需要增加逻辑：
         * （1) 第一次设置位姿时，直接使用椭球体位姿
         *       物体在隐式形状优化过程中选择了最佳朝向。
         * （2）在非首次的处理过程中，如何使用椭球体位姿？
         *       
         */
        

        // FIXME: 这里可以选择，在使用椭球体模式下，如果没有成功估计椭球体，就跳过物体优化
        if (numKFsPassedSinceInit < 50 && !pMO->reconstructed) {
            if (!use_ellipsold_pose_for_shape_optimization) {
                std::cout << "ComputeCuboidPCA" << std::endl;
                pMO->ComputeCuboidPCA(numKFsPassedSinceInit < 15);
            }
            else{
                if (mvpGlobalEllipsolds[det_i] == NULL) {
                    cout << "mvpGlobalEllipsolds[" << det_i << "] == NULL" << endl;
                    pMO->SetBadFlag();
                }
                else{
                    // Method 2: 使用来自椭球体的位姿信息
                    std::cout << "SetPoseByEllipsold" << std::endl;
                    pMO->SetPoseByEllipsold(mvpGlobalEllipsolds[det_i]);
                }
            }

        }
        else { // when we have relative good object shape
            std::cout << "RemoveOutliersModel" << std::endl;
            pMO->RemoveOutliersModel();
        }
        
        // // only begin to reconstruct the object if it is observed for enough amoubt of time (15 KFs)
        // // 只在观测间隔了足够数量
        // if(isShortInternal) {
        //     std::cout << "  Conitinue because " << condition_str << std::endl;
        //     continue;
        // }

        /**
         * 为了减少优化次数，只在 passedKF 为5的倍数时处理
        */
        if ((numKFsPassedSinceInit - 15) % 5 != 0) {
            std::cout << "  Conitinue because (numKFsPassedSinceInit - 15) % 5 != 0" << std::endl;
            continue;
        }

        std::vector<MapPoint*> points_on_object = pMO->GetMapPointsOnObject();
        int n_points = points_on_object.size();
        int n_valid_points = 0;

        if (add_depth_pcd_to_map_object && use_depth_pcd_to_reconstruct){

            std::shared_ptr<PointCloud> mPointsPtr = pMO->GetPointCloud();
            PointCloud* pPoints = mPointsPtr.get();

            // auto points = pMO->GetPointCloud();
            n_valid_points = pPoints->size();
        }
        else{
            for (auto pMP : points_on_object)
            {
                if (!pMP)
                    continue;
                if (pMP->isBad())
                    continue;
                if (pMP->isOutlier())
                    continue;
                n_valid_points++;
            }
        }

        // 记录物体上的关键点的数量
        int n_rays = 0;
        auto map_points_vector = mpCurrentKeyFrame->GetMapPointMatches();

        for (auto idx : det->GetFeaturePoints())
        {
            auto pMP = map_points_vector[idx];
            if (!pMP)
                continue;
            if (pMP->isBad())
                continue;
            if (pMP->object_id != pMO->mnId)
                continue;
            if (pMP->isOutlier())
                continue;
            n_rays++;
        }

        int n_background_ray = det->background_rays.rows();

        cout << "\n==> Object " << pMO->mnId << ": " \
             << n_points << " points observed, " << "with " \
             << n_valid_points << " valid points, " \
             << n_rays << " rays, and " \
             << n_background_ray << " back_ground_rays" \
             << endl;

        // Surface points
        // std::cout << " =>" << "n_valid_points: " << n_valid_points << ", n_rays: " << n_rays << std::endl;
        if (n_valid_points >= min_valid_points && n_rays > min_valid_rays)
        {
            //！获取surface_points_cam
            Eigen::MatrixXf surface_points_cam = Eigen::MatrixXf::Zero(n_valid_points, 3);
            int p_i = 0;


            if (add_depth_pcd_to_map_object && use_depth_pcd_to_reconstruct)
            {
                std::shared_ptr<PointCloud> mPointsPtr = pMO->GetPointCloud();
                PointCloud* pPoints = mPointsPtr.get();

                for(int i=0; i<pPoints->size(); i=i+1)
                {
                    PointXYZRGB &p = (*pPoints)[i];
                    cv::Mat x3Dw = (cv::Mat_<float>(3,1) << p.x, p.y, p.z);
                    cv::Mat x3Dc = Rcw * x3Dw + tcw;
                    float xc = x3Dc.at<float>(0);
                    float yc = x3Dc.at<float>(1);
                    float zc = x3Dc.at<float>(2);
                    surface_points_cam(p_i, 0) = xc;
                    surface_points_cam(p_i, 1) = yc;
                    surface_points_cam(p_i, 2) = zc;
                    p_i++;
                }

            }
            else{
                for (auto pMP : points_on_object)
                {
                    if (!pMP)
                        continue;
                    if (pMP->isBad())
                        continue;
                    if (pMP->isOutlier())
                        continue;

                    cv::Mat x3Dw = pMP->GetWorldPos();
                    cv::Mat x3Dc = Rcw * x3Dw + tcw;
                    float xc = x3Dc.at<float>(0);
                    float yc = x3Dc.at<float>(1);
                    float zc = x3Dc.at<float>(2);
                    surface_points_cam(p_i, 0) = xc;
                    surface_points_cam(p_i, 1) = yc;
                    surface_points_cam(p_i, 2) = zc;
                    p_i++;
                }
            }

            // 椭球体本身能否用于深度渲染约束？

            //！ 获取ray_pixels和depth_obs
            Eigen::MatrixXf ray_pixels = Eigen::MatrixXf::Zero(n_rays, 2);
            Eigen::VectorXf depth_obs = Eigen::VectorXf::Zero(n_rays);
            int k_i = 0;
            for (auto point_idx : det->GetFeaturePoints())
            {
                auto pMP = map_points_vector[point_idx];
                if (!pMP)
                    continue;
                if(pMP->isBad())
                    continue;
                if(pMP->object_id != pMO->mnId)
                    continue;
                if (pMP->isOutlier())
                    continue;

                cv::Mat x3Dw = pMP->GetWorldPos();
                cv::Mat x3Dc = Rcw * x3Dw + tcw;
                depth_obs(k_i) = x3Dc.at<float>(2);
                ray_pixels(k_i, 0) = mpCurrentKeyFrame->mvKeysUn[point_idx].pt.x;
                ray_pixels(k_i, 1 ) = mpCurrentKeyFrame->mvKeysUn[point_idx].pt.y;
                k_i++;
            }

            // 像素点的归一化的向量 [[x,y,1], ... ]
            Eigen::MatrixXf u_hom(n_rays, 3);
            u_hom << ray_pixels, Eigen::MatrixXf::Ones(n_rays, 1);

            // 转换到相机坐标系的射线向量
            Eigen::MatrixXf fg_rays(n_rays, 3);
            Eigen::Matrix3f invK = Converter::toMatrix3f(mpTracker->GetCameraIntrinsics()).inverse();

            for (int i = 0; i  < n_rays; i++)
            {
                auto x = u_hom.row(i).transpose();
                fg_rays.row(i) = (invK * x).transpose();
            }

            Eigen::MatrixXf rays(fg_rays.rows() + det->background_rays.rows(), 3);
            rays << fg_rays, det->background_rays;

            /**
             * 表面点与射线数据准备完毕，下面进行物体重建
             * 
            */
            std::cout << "!!! Ready to reconstruct_object !!!" << std::endl;

            PyThreadStateLock PyThreadLock;

            // std::cout << "mpCurrentKeyFrame->pose = \n" << SE3Tcw.matrix() << std::endl;
            // std::cout << "pMO->Sim3Two = \n" << pMO->Sim3Two.matrix() << std::endl;

            auto Sim3Two_pMO = pMO->Sim3Two;

            Eigen::Matrix4f Sim3Two_raw = Sim3Two_pMO;

            int class_id = det->label;

            py::object* optimizer_ptr;

            if(mmPyOptimizers.count(class_id) > 0) {
                py::object* optimizer_ptr_local = &(mmPyOptimizers[class_id]);
                optimizer_ptr = optimizer_ptr_local;
            }
            else{
                cout << " [ProcessDetectedObjects] class " << class_id << " is not in yolo_classes" << endl;
                py::object* optimizer_ptr_local = &pyOptimizer;
                optimizer_ptr = optimizer_ptr_local;
            }

            cout << "Before reconstruct_object" << std::endl;

            auto pyMapObject = optimizer_ptr->attr("reconstruct_object")
                    (SE3Tcw * Sim3Two_pMO, surface_points_cam, rays, depth_obs, pMO->vShapeCode);

            // std::cout << "0 rad, is_good: " << pyMapObject.attr("is_good").cast<bool>()\
            //           << ", loss: " << pyMapObject.attr("loss").cast<float>() << std::endl;

            auto& pyMapObjectLeastLoss = pyMapObject;

            if (!pMO->findGoodOrientation)
            {
                // 绕y轴进行采样，可以直接附加到Two上
                std::cout << "Has not found good orientation yet." << std::endl;
                std::vector<float> losses(flip_sample_num);
                losses[0] = pyMapObject.attr("loss").cast<float>();
                
                for (int i_rot = 1; i_rot < flip_sample_num; i_rot++) {

                    auto flipped_Two = Sim3Two_pMO;
                    Eigen::Matrix3f Ry = Eigen::AngleAxisf(double(i_rot) * flip_sample_angle, \
                                                Eigen::Vector3f(0,1,0)).matrix();

                    flipped_Two.topLeftCorner(3,3) = flipped_Two.topLeftCorner(3,3) * Ry;
                    // cout << " => flipped_Two = " << flipped_Two.matrix() << std::endl;

                    // auto pyMapObjectFlipped = pyOptimizer.attr(">>>reconstruct_object")
                    //         (SE3Tcw * flipped_Two, surface_points_cam, rays, depth_obs, pMO->vShapeCode);

                    auto pyMapObjectFlipped = optimizer_ptr->attr("reconstruct_object")
                            (SE3Tcw * flipped_Two, surface_points_cam, rays, depth_obs, pMO->vShapeCode);
                    
                    // std::cout << "Loss: " << pyMapObjectFlipped.attr("loss").cast<float>() << std::endl;
                    losses[i_rot] = pyMapObjectFlipped.attr("loss").cast<float>();

                    bool recon_state = pyMapObjectLeastLoss.attr("is_good").cast<bool>();
                    bool recon_state_flipped = pyMapObjectFlipped.attr("is_good").cast<bool>();
                    float loss_least = pyMapObjectLeastLoss.attr("loss").cast<float>();
                    float loss_flipped = pyMapObjectFlipped.attr("loss").cast<float>();

                    std::cout << double(i_rot) * flip_sample_angle
                            <<  " rad, is_good: " << pyMapObjectFlipped.attr("is_good").cast<bool>()\
                            << ", loss: " << pyMapObjectFlipped.attr("loss").cast<float>() << std::endl;

                    // 如果先前的重建失败 / 当前重建成功且loss更小
                    if (!recon_state || \
                        (loss_least > loss_flipped && recon_state_flipped)) {
                        std::cout << "# Rotate y axis by " << double(i_rot) * flip_sample_angle << " rad" << std::endl;
                        pyMapObjectLeastLoss = pyMapObjectFlipped;
                    }
                }
                // todo: to Judge whether good orientation has been found
                std::cout << "# Losses: ";
                for (auto &loss: losses) {
                    std::cout << loss << ",";
                }
                std::cout << std::endl;
            }

            auto t_cam_obj = pyMapObjectLeastLoss.attr("t_cam_obj");

            if (t_cam_obj.is_none()) {
                std::cout << "!!!! Output t_cam_obj == None, reconstruction failed." << std::endl;
                std::cout << "Sim3Two_raw = " << Sim3Two_raw.matrix() << std::endl;
                std::cout << "pMO->SE3Two = " << pMO->SE3Two.matrix() << std::endl;
                std::cout << "pMO->scale = " << pMO->scale << std::endl;
                continue;
            }
            auto Sim3Tco = t_cam_obj.cast<Eigen::Matrix4f>();

            // std::cout << "Sim3Tco = " << Sim3Tco.matrix() << std::endl;

            // auto Sim3Tco = SE3Tcw * pMO->Sim3Two;

            std::cout << "Reconstruction successed" << std::endl;
            
            // 设置检测结果的位姿测量
            det->SetPoseMeasurementSim3(Sim3Tco);
            // Sim3, SE3, Sim3
            // Sim3 可以乘在前面但不能乘在后面?
            Eigen::Matrix4f Sim3Two = SE3Twc * Sim3Tco;

            // std::cout << "Sim3Two = " << Sim3Two.matrix() << std::endl;

            // int code_len = pyOptimizer.attr("code_len").cast<int>();

            int code_len = optimizer_ptr->attr("code_len").cast<int>();

            Eigen::Matrix<float, 64, 1> code = Eigen::VectorXf::Zero(64);
            if (code_len == 32)
            {
                auto code_32 = pyMapObjectLeastLoss.attr("code").cast<Eigen::Matrix<float, 32, 1>>();
                code.head(32) = code_32;
            }
            else
            {
                code = pyMapObjectLeastLoss.attr("code").cast<Eigen::Matrix<float, 64, 1>>();
            }

            /**
             * 更新物体属性： 
             * - 位姿、编码、网格顶点、网格面片、
             * - 是否已重建、（关键帧，检测索引）
             * 向 当前关键帧、物体绘制器 添加物体
            */
            
            if (keep_raw_pose) {
                cout << "Draw Sim3Two_raw " << endl;
                pMO->UpdateReconstruction(Sim3Two_raw, code);
            }
            else {
                pMO->UpdateReconstruction(Sim3Two, code);
                
            }

            // auto pyMesh = pyMeshExtractor.attr("extract_mesh_from_code")(code);

            py::object* mesh_extracter_ptr;
            if(mmPyOptimizers.count(class_id) > 0) {
                // mesh_extracter_ptr = mmPyMeshExtractors[class_id];
                py::object* mesh_extracter_ptr_local = &(mmPyMeshExtractors[class_id]);
                mesh_extracter_ptr = mesh_extracter_ptr_local;
            }
            else{
                cout << " [LocalMapping_util.cc 265] class " << class_id << "is not in yolo_classes" << endl;
                py::object* mesh_extracter_ptr_local = &pyMeshExtractor;
                mesh_extracter_ptr = mesh_extracter_ptr_local;
            }

            auto pyMesh = mesh_extracter_ptr->attr("extract_mesh_from_code")(code);

            pMO->vertices = pyMesh.attr("vertices").cast<Eigen::MatrixXf>();
            pMO->faces = pyMesh.attr("faces").cast<Eigen::MatrixXi>();
            pMO->reconstructed = true;
            pMO->AddObservation(mpCurrentKeyFrame, det_i);
            mpCurrentKeyFrame->AddMapObject(pMO, det_i);
            mpObjectDrawer->AddObject(pMO);

            mlpRecentAddedMapObjects.push_back(pMO);

            nLastReconKFID = int(mpCurrentKeyFrame->mnId);

            // std::cout << "Finish reconstructing one object" << std::endl;

            // std::cout << "Press [ENTER] to continue ... " << std::endl;
            // key = getchar();
        }
    }

    if (show_ellipsold_process && mvpObjectDetections.size())
    {
        auto det_vec = mvpObjectDetections.back()->bbox;
        int x1 = (int)det_vec(0), y1 = (int)det_vec(1), x2 = (int)det_vec(2), y2 = (int)det_vec(3);
        cv::Mat img_show(cam_height, cam_width, CV_8UC3, cv::Scalar(255, 255, 255));
        cv::rectangle(img_show, cv::Point(x1, y1), cv::Point(x2, y2), cv::Scalar(255, 0, 0), 2);  // Scalar(255, 0, 0) is for blue color, 2 is the thickness
        cv::imshow("Image with Bbox", img_show);
        cv::waitKey(10);
        cout << "Press any key to continue" << endl;
        char key = getchar();

        cv::destroyWindow("Image with Bbox");
    }


    
    // if (show_ellipsold_process && mvpObjectDetections.size()){
    //     cv::destroyWindow("Image with Bbox");
    // }
}

void LocalMapping::UpdateObjectsToMap()
{
    cout << "\n[LocalMapping::UpdateObjectsToMap]" << endl;
    auto mapObjects = mpMap->GetAllMapObjects();
    vector<int> valid_obj_ids;
    for (auto &pMO: mapObjects){
        if (!pMO->isBad()) {
            valid_obj_ids.push_back(pMO->mnId);
        }
    }
    int obj_num_valid = valid_obj_ids.size();
    int obj_num_dropped = mapObjects.size() - obj_num_valid;

    cout << " - Get Objects: " << obj_num_valid << " valid, "
         << obj_num_dropped << " dropped" << endl;
    cout << " - Valid objects's Ids: ";
    for (auto &obj_id: valid_obj_ids) {
        cout << obj_id <<  ", ";
    }
    cout << endl;
    
    // 每次都会重新更新一遍地图中的物体椭球体
    mpMap->ClearEllipsoidsObjects();
    // cout << "ClearEllipsoidsObjects Finished" << endl;
    mpMap->DeletePointCloudList("MapObject PointCloud", 0);

    for (auto &pMO: mapObjects){

        // if (pMO->isBad()) {
        //     continue;
        // }

        // // Default type = 0: replace when exist , 1: add when exist
        // bool Map::AddPointCloudList(const string &name, PointCloud *pCloud, int type) {
        
        // TODO: 下一步考虑如何将所有物体的点云都添加到地图中，并且可以在每个新帧进行更新
        if (pMO->hasValidPointCloud()){
            // std::shared_ptr<PointCloud> mPointsPtr = pMO->GetPointCloud();
            // PointCloud* pPoints = mPointsPtr.get();
            auto pcl_ptr= pMO->GetPointCloudPCL();
            auto pPoints = pclXYZToQuadricPointCloudPtr(pcl_ptr);
            // auto pcd = pMO->GetPointCloud();

            mpMap->AddPointCloudList("MapObject PointCloud", pPoints, 1);

            // int n_valid_points = pPoints->size();
            // cout << "MapObject PointCloud size = " << pPoints->size() << endl;
        }


        if (pMO->isBad()) {
            continue;
        }


        // FIXME：这里添加了一个没有初始化的ellipsold导致显示错误，暂时通过判断e的概率小于0.001
        auto e = pMO->GetEllipsold();
        if (e != NULL) {
            mpMap->addEllipsoidObjects(e);
        }
    }
}

void LocalMapping::SetOptimizer(Optimizer* optimizer)
{
    mpOptimizer = optimizer;
}

void LocalMapping::AssociateObjects3D()
{
    cout << "\n [LocalMapping_utils.cc] AssociateObjects3D" << endl;
    auto allMapObjects = mpMap->GetAllMapObjects();
    int obj_num = allMapObjects.size();

    for (int i = 0; i < obj_num-1; i++) {
        for (int j = obj_num - 1; j > i; j--) {
            // FIXME: 这里只能暂且不考虑同一个物体被检测出不同类别的情况了，根据情况考虑是否只对上次新添加的物体进行处理
            // if (same_class && close && others), 
            //  merge obj_j into obj_i, set obj_j bad
            auto pMO_i = allMapObjects[i], pMO_j = allMapObjects[j];
            if (!pMO_i->hasValidEllipsold() || !pMO_j->hasValidEllipsold()) {
                continue;
            }
            bool c0 = (pMO_i->label == pMO_j->label);
            auto SE3Two_i = pMO_i->GetPoseSE3();
            auto SE3Two_j = pMO_j->GetPoseSE3();

            auto scale_i = pMO_i->GetEllipsold()->scale; // 长，宽，高
            auto scale_j = pMO_j->GetEllipsold()->scale;

            float dist_limit = scale_i.head<2>().norm() + scale_j.head<2>().norm();

            // TODO：这个距离是在世界坐标系中的，但是世界坐标系并不与地面对齐
            Eigen::Vector3f dist3D = SE3Two_i.topRightCorner<3, 1>() - SE3Two_j.topRightCorner<3, 1>();
            float dist3D_norm = dist3D.norm();

            // FIXME: 这里的0.5有待改成配置文件中进行设置
            bool c1 = (dist3D_norm < dist_filt_param * dist_limit);
            // if (c0 && c1) {
            if (c1) {
                // 这里应该把 pMO_j 合并到 pMO_i 中
                cout << "MergeMapObject " << endl;
                MergeMapObject(pMO_i, pMO_j);
            }
        }
    }
}

void LocalMapping::GlobalOptimization()
{
    // mpOptimizer->OptimizeWithDataAssociationUsingMultiplanes(pFrames, mms, objs, camTraj, calib, iRows, iCols);
    // auto mvpFrames = 
    mpOptimizer->GlobalObjectGraphOptimizationWithPDA(mvpFrames, mpMap, mpTracker->mCalib, mpTracker->mRows, mpTracker->mCols);
}

void LocalMapping::MergeMapObject(MapObject* pMO_i, MapObject* pMO_j)
{   
    // TODO：这里可能需要同时获取到两个物体的锁，或者转移到MapObject的函数中
    // 需要考虑如果两个物体的尺寸差异很大怎么办

    // 点云合并


    // 观测合并: 需要考虑如果两个物体都在某同一关键帧有观测怎么办
    // auto objs_i = pMO_i->GetObservations();
    // auto objs_j = pMO_j->GetObservations();

    // for 

    // 椭球体合并

    // 合并完之后的隐式编码应该如何处理

    // 设置 pMO_j 为 bad
    pMO_j->SetBadFlag();
}

}
