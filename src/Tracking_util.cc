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

#include "Tracking.h"
#include "ObjectDetection.h"
#include "ORBmatcher.h"

#include "utils/dataprocess_utils.h"

#include <Eigen/Dense>
#include <opencv2/core/eigen.hpp>


using namespace std;

namespace ORB_SLAM2 {

void VisualizeRelations(Relations& rls, Map* pMap, g2o::SE3Quat &Twc, std::vector<PointCloudPCL>& vPlanePoints);

/**
 * Tracking utils for stereo+lidar on KITTI
 * 获得双目+激光雷达点云的检测结果，创建对应的物体观测和地图物体，添加到相应关键帧中
 */
void Tracking::GetObjectDetectionsLiDAR(KeyFrame *pKF) {

    PyThreadStateLock PyThreadLock;

    std::string frame_name = mvstrImageFilenamesRGB[pKF->mnFrameId];

    std::cout << "frame_name = " << frame_name << std::endl;

    // py::list detections = mpSystem->pySequence.attr("get_frame_by_id")(pKF->mnFrameId);

    // 这里的检测结果中会直接包含 T_cam_obj, scale, surface_points, rays, depth, mask, depth (激光雷达点云在python程序中处理)

    py::list detections = mpSystem->pySequence.attr("get_frame_by_name")(pKF->mnFrameId, frame_name);

    for (auto det : detections) {
        auto pts = det.attr("surface_points").cast<Eigen::MatrixXf>();
        auto Sim3Tco = det.attr("T_cam_obj").cast<Eigen::Matrix4f>();
        auto rays = det.attr("rays");
        Eigen::MatrixXf rays_mat;
        Eigen::VectorXf depth;

        if (rays.is_none()) {
            // std::cout << "No 2D masks associated!" << std::endl;
            rays_mat = Eigen::Matrix<float, 0, 0>::Zero();
            depth = Eigen::Matrix<float, 0, 1>::Zero();
        } else {
            rays_mat = rays.cast<Eigen::MatrixXf>();
            depth = det.attr("depth").cast<Eigen::VectorXf>();
        }

        // Create C++ detection instance
        auto o = new ObjectDetection(Sim3Tco, pts, rays_mat, depth);
        pKF->mvpDetectedObjects.push_back(o);
    }
    pKF->nObj = pKF->mvpDetectedObjects.size();
    pKF->mvpMapObjects = vector<MapObject *>(pKF->nObj, static_cast<MapObject *>(NULL));
}


/**
 * （双目+Lidar）物体数据关联
 *  建立观测与地图物体之间的关联
 *  zhjd: 不是只能创建一个新物体？
*/
void Tracking::ObjectDataAssociation(KeyFrame *pKF)
{

    /**
     * 首先获取局部（近邻）关键帧中的地图物体，通过修改 mnAssoRefID 的方式避免多次添加同一物体
    */
    vector<MapObject *> vpLocalMapObjects;
    // Loop over all the local frames to find matches
    for (KeyFrame *plKF : mvpLocalKeyFrames)
    {
        vector<MapObject *> vpMOs = plKF->GetMapObjectMatches();
        for (MapObject *pMO : vpMOs)
        {
            if (pMO)
            {
                // Prevent multiple association to the same object
                if (pMO->mnAssoRefID != pKF->mnId)
                {
                    vpLocalMapObjects.push_back(pMO);
                    pMO->mnAssoRefID = pKF->mnId;
                }
            }
        }
    }

    if (vpLocalMapObjects.empty())
        return;

    /**
     * 获取这一帧的Tcw, Detections
     * 遍历这些 Detections, 对于每个Detection 遍历局部物体：
     *  - 如果为空/isbad： dist设置为1000
     *  - 如果为静态物体： 通过计算3D距离、2D距离，保存到dist中
     * 找到dist中最近距离的物体
    */
    Eigen::Matrix4f Tcw = Converter::toMatrix4f(mCurrentFrame.mTcw);
    Eigen::Matrix3f Rcw = Tcw.topLeftCorner<3, 3>();
    Eigen::Vector3f tcw = Tcw.topRightCorner<3, 1>();
    auto vDetections = pKF->mvpDetectedObjects;


    // loop over all the detections.
    for (int i = 0; i < pKF->nObj; i++)
    {
        auto det = vDetections[i];
        Eigen::Vector3f transDet = det->tco;
        // 记录每个地图物体与 det 之间的距离
        vector<float> dist;

        for (auto pObj : vpLocalMapObjects)
        {
            if (!pObj || pObj->isBad())
            {
                dist.push_back(1000.0);
                continue;
            }

            if (!pObj->isDynamic()) {
                Eigen::Vector3f dist3D = Rcw * pObj->two + tcw - transDet;
                Eigen::Vector2f dist2D;
                dist2D << dist3D[0], dist3D[2];
                dist.push_back((dist2D).norm());
            }
            else
            {
                float deltaT = (float) (mCurrentFrame.mnId - mpLastKeyFrame->mnFrameId);
                auto twoPredicted = pObj->two + pObj->velocity * deltaT;
                Eigen::Vector3f dist3D = Rcw * twoPredicted + tcw - transDet;
                Eigen::Vector2f dist2D;
                dist2D << dist3D[0], dist3D[2];
                dist.push_back((dist2D).norm());
            }
        }
        float minDist = *min_element(dist.begin(), dist.end());

        /**
         * 如果距离小于5,认为关联上，进一步判断：
         * - 观测的点数是否达到一定数量
         * - 该关键帧是否已经关联过该物体
         *   - 否： 设置距离，向关键帧添加物体，向物体添加观测
         *   - 是： 比较距离，如果距离更小，则更改关联的观测，原来与物体关联的观测重新设置 is_new
         * 
         * 如果距离大于5,认为is_new，关联地图点数大于50认为isGood
        */

        // Start with a loose threshold
        if (minDist < 5.0)
        {
            det->isNew = false;
            if (det->nPts < 25)
                det->isGood = false;

            // 局部地图物体idx
            int idx = min_element(dist.begin(), dist.end()) - dist.begin();
            MapObject *pMO = vpLocalMapObjects[idx];

            if (!pKF->mdAssociatedObjects.count(pMO)) {
                pKF->mdAssociatedObjects[pMO] = minDist;
                pKF->AddMapObject(pMO, i);
                pMO->AddObservation(pKF, i);
            } else // Another detection is associated with pMO, compare distance
            {
                if (minDist < pKF->mdAssociatedObjects[pMO]) {
                    // cout << "Associated to: " << pMO->mnId << ", Distance: " << minDist << endl;
                    pKF->mdAssociatedObjects[pMO] = minDist;
                    int detId = pMO->GetObservations()[pKF];
                    pKF->EraseMapObjectMatch(detId);
                    vDetections[detId]->isNew = true;
                    pKF->AddMapObject(pMO, i);
                    pMO->AddObservation(pKF, i);
                    // 这里无需移除PMO原先设置的观测，AddObservation中每个物体仅会与一个 detection id 建立联系
                }
            }
        }
        else
        {
            det->isNew = true;
            if (det->nPts < 50)
                det->isGood = false;
        }
    }
}

/*
 * Tracking utils for monocular input on Freiburg Cars and Redwood OS
 */
cv::Mat Tracking::GetCameraIntrinsics()
{
    return mK;
}

void Tracking::GetObjectDetectionsMono(KeyFrame *pKF)
{
    std::cout << "[ Tracking - GetObjectDetectionsMono ]" << std::endl;
    // => Step 1: 准备Python线程锁，清空物体 mask 的 vector
    PyThreadStateLock PyThreadLock;
    mvImObjectMasks.clear();
    mvImObjectBboxs.clear();

    // => Step 2: 获取帧号/帧名称，调用物体检测
    // std::string frame_name = mvstrImageFilenamesRGB[pKF->mnFrameId];
    // std::cout << "frame_name = " << frame_name << std::endl;
    // py::list detections = mpSystem->pySequence.attr("get_frame_by_name")(pKF->mnFrameId, frame_name);

    py::list detections = mpSystem->pySequence.attr("get_frame_by_id")(pKF->mnFrameId);

    std::cout << " => " << "detections.size() = " << detections.size() << std::endl;

    if (detections.size() == 0) // No detections, return immediately 
    {
        std::cout << "Return because detections.size() == 0" << std::endl;
        return;
    }

    // => Step 3: 遍历这些检测结果
    for (int detected_idx = 0; detected_idx < detections.size(); detected_idx++)
    {
        // => Step 3.1: 创建物体检测类，暂时保存mask，
        //              并将ray、bbox和mask内的关键点保存到检测实例中
        auto det = new ObjectDetection();
        auto py_det = detections[detected_idx];
        det->background_rays = py_det.attr("background_rays").cast<Eigen::MatrixXf>();
        auto mask = py_det.attr("mask").cast<Eigen::MatrixXf>();
        det->bbox = py_det.attr("bbox").cast<Eigen::Vector4d>();
        det->label = py_det.attr("label").cast<int>();
        det->prob = py_det.attr("prob").cast<double>();

        cv::Mat mask_cv;
        cv::eigen2cv(mask, mask_cv);
        // cv::imwrite("mask.png", mask_cv);
        cv::Mat mask_erro = mask_cv.clone();
        cv::Mat kernel = getStructuringElement(cv::MORPH_ELLIPSE,
                                               cv::Size(2 * maskErrosion + 1, 2 * maskErrosion + 1),
                                               cv::Point(maskErrosion, maskErrosion));
        cv::erode(mask_cv, mask_erro, kernel);

        int x1 = (int)(det->bbox(0)), y1 = (int)(det->bbox(1)), \
            x2 = (int)(det->bbox(2)), y2 = (int)(det->bbox(3));

        mvImObjectMasks.push_back(std::move(mask_cv));
        mvImObjectBboxs.push_back({x1,y1,x2,y2});
        // std::cout<< " [zhjd-debug] bbox: " << "x1:"<<x1 
        //     << ", y1:" << y1 << ", x2:" << x2 << ", y2:" << y2
        //     << std::endl;
            
        // get 2D feature points inside mask
        for (int i = 0; i < pKF->mvKeys.size(); i++)
        {
            int val = (int) mask_erro.at<float>(pKF->mvKeys[i].pt.y, pKF->mvKeys[i].pt.x);
            if (val > 0)  // inside the mask
            {
                det->AddFeaturePoint(i);
            }
        }

        // Step 3.2: 将少于20个关键点的det定义为isGood = false，
        //           将det加入当前帧物体检测vector中

        // Reject the detection if too few keypoints are extracted

        if (det->NumberOfPoints() < 20)
        {
            std::cout << " => det" << detected_idx << "->NumberOfPoints() < 20" << std::endl;
            det->isGood = false;
        }
        pKF->mvpDetectedObjects.push_back(det);
        std::cout << " => " << "pKF->mvpDetectedObjects.push_back(det)" << std::endl;
    }

    auto mvpObjectDetections = pKF->GetObjectDetections();

    std::cout << " => " << "KF" << pKF->mnId << ", mvpObjectDetections.size() = " << mvpObjectDetections.size() << std::endl;

    // => Step 4: 记录当前帧的物体检测数量，预留同样数量的物体Vector
    pKF->nObj = pKF->mvpDetectedObjects.size();
    pKF->mvpMapObjects = vector<MapObject *>(pKF->nObj, static_cast<MapObject *>(NULL));
}


void Tracking::GetObjectDetectionsRGBD(KeyFrame *pKF)
{
    std::cout << "\n [Tracking::GetObjectDetectionsRGBD] " << std::endl;

    // printMemoryUsage();

    PyThreadStateLock PyThreadLock;

    mvImObjectMasks.clear();
    mvImObjectBboxs.clear();

    std::string frame_name = mvstrImageFilenamesRGB[pKF->mnFrameId];

    // Get a series of object detections
    bool use_for_ros = Config::Get<int>("use_for_ros");
    py::list detections;
    if (use_for_ros) {
        string file_name = "ros.png";
        string file_path = Config::Get<string>("Dataset.Path.Root");;
        cv::imwrite(file_path+"/"+file_name, pKF->color_img);
        detections= mpSystem->pySequence.attr("get_frame_by_name")(pKF->mnFrameId, file_name);

    }
    else
        detections= mpSystem->pySequence.attr("get_frame_by_name")(pKF->mnFrameId, frame_name);

    int num_dets = detections.size();

    std::cout << "  - Detects " << num_dets << " objects Observations." << std::endl;

    // No detections, return immediately
    if (num_dets == 0)
        return;

    for (int detected_idx = 0; detected_idx < num_dets; detected_idx++)
    {
        auto det = new ObjectDetection();
        auto py_det = detections[detected_idx];
        det->background_rays = py_det.attr("background_rays").cast<Eigen::MatrixXf>();
        auto mask = py_det.attr("mask").cast<Eigen::MatrixXf>();
        det->bbox = py_det.attr("bbox").cast<Eigen::Vector4d>();
        det->label = py_det.attr("label").cast<int>();
        det->prob = py_det.attr("prob").cast<double>();

        cv::Mat mask_cv;
        cv::eigen2cv(mask, mask_cv);
        // cv::imwrite("mask.png", mask_cv);
        cv::Mat mask_erro = mask_cv.clone();
        // std::cout << "maskErrosion = " << maskErrosion << std::endl;
        cv::Mat kernel = getStructuringElement(cv::MORPH_ELLIPSE,
                                               cv::Size(2 * maskErrosion + 1, 2 * maskErrosion + 1),
                                               cv::Point(maskErrosion, maskErrosion));
        
        cv::erode(mask_cv, mask_erro, kernel);

        int x1 = (int)(det->bbox(0)), y1 = (int)(det->bbox(1)), \
            x2 = (int)(det->bbox(2)), y2 = (int)(det->bbox(3));

        mvImObjectMasks.push_back(std::move(mask_cv));
        mvImObjectBboxs.push_back({x1,y1,x2,y2});
        
        // get 2D feature points inside mask
        for (int i = 0; i < pKF->mvKeys.size(); i++)
        {
            int val = (int)mask_erro.at<float>(pKF->mvKeys[i].pt.y, pKF->mvKeys[i].pt.x);
            if (val > 0)  // inside the mask
            {
                det->AddFeaturePoint(i);
            }
        }

        // Reject the detection if too few keypoints are extracted
        // todo: 设置点数量数量而不是isGood
        if (det->NumberOfPoints() < minimux_points_to_judge_good)
        {
            det->isGood = false;
        }

        pKF->mvpDetectedObjects.push_back(det);

        // cout << "Finish detection " << detected_idx << endl;
        // printMemoryUsage();
    }

    // 将det结果保存到矩阵mmObservations中
    pKF->nObj = pKF->mvpDetectedObjects.size();
    pKF->mvpMapObjects = vector<MapObject *>(pKF->nObj, static_cast<MapObject *>(NULL));
}

/**
 * 通过地图点建立观测与地图物体之间的联系，并补充地图点
 * 
 * => Step 1: 获取该关键帧关联的地图点、物体检测
 * => Step 2: 遍历所有检测结果
 *           - 遍历检测关联的地图点，判断其是否合法且为内点，记录这些地图点中已经关联的物体编号
 *           - 根据关键点数量寻找与哪个object编号更匹配，获取对应的物体指针，将物体、检测添加到关键帧，
 *             设置该 det->isNew = false，将其他的检测关联地图点设置与物体关联
 * 
*/
void Tracking::AssociateObjectsByProjection(ORB_SLAM2::KeyFrame *pKF)
{
    
    std::cout << "\n >> [ Tracking - AssociateObjectsByProjection ] << " << std::endl;
    // => Step 1: 获取该关键帧关联的地图点、物体检测
    auto mvpMapPoints = pKF->GetMapPointMatches();
    // Try to match and triangulate key-points with last key-frame
    auto detectionsKF1 = pKF->mvpDetectedObjects;
    
    // => Step 2: 遍历所有检测结果
    for (int d_i = 0; d_i < detectionsKF1.size(); d_i++)
    {
        // Step 2.1: 遍历检测关联的地图点，判断其是否合法且为内点
        //           记录这些地图点中已经关联的物体编号
        cout << "Detection: " << d_i + 1 << endl;
        auto detKF1 = detectionsKF1[d_i];
        vector<double> iou_stats;
        bool has_associate = false;

        auto bbox_det = detKF1->bbox;
        auto label_bbox = detKF1->label;
        // int x1 = (int)bbox_det(1), y1 = (int)bbox_det(2), x2 = (int)bbox_det(3), y2 = (int)bbox_det(4);
        cv::Rect r2_bbox(cv::Point(bbox_det[0], bbox_det[1]), cv::Point(bbox_det[2], bbox_det[3]));

        // 如果使用椭球体投影的椭圆进行关联，则不进行后续的使用地图点的关联
        if (associate_object_with_ellipsold) {
            // cout << "Tracking::AssociateObjectsByProjection" << endl;
            auto mapObjects = mpMap->GetAllMapObjects();
            for (auto pMO: mapObjects){
                // FIXME：这里暂时对于 e 为 NULL 的情况跳过处理
                cv::Mat img_show = mCurrentFrame.color_img.clone();
                auto e = pMO->GetEllipsold();

                if (e==NULL){
                    continue;
                }

                auto label_obj = pMO->label;
                auto campose_cw = mCurrentFrame.cam_pose_Tcw;
                auto ellipse = e->projectOntoImageEllipse(campose_cw, mCalib);
                e->drawEllipseOnImage(ellipse, img_show);
                
                // draw bbox of object
                Vector4d rect = e->getBoundingBoxFromProjection(campose_cw, mCalib); 

                // 与bbox求IoU
                // 获取椭球体投影后的边界框
                cv::Rect r1_proj(cv::Point(rect[0], rect[1]), cv::Point(rect[2], rect[3]));
                cv::rectangle(img_show, r1_proj, cv::Scalar(0, 0, 255), 2);

                // draw bbox of det
                // cv::rectangle(img_show, cv::Point(x1, y1), cv::Point(x2, y2), cv::Scalar(255, 0, 0), 2);  // Scalar(255, 0, 0) is for blue color, 2 is the thickness
                cv::rectangle(img_show, r2_bbox, cv::Scalar(255, 0, 0), 2);  // Scalar(255, 0, 0) is for blue color, 2 is the thickness

                cv::Rect r_and = r1_proj | r2_bbox;
                cv::Rect r_U = r1_proj & r2_bbox;
                double iou = r_U.area()*1.0/r_and.area();

                iou_stats.push_back(iou);

                //TODO: 没关联上可能是因为椭球体的参数没有及时更新
                if (associate_debug)
                {
                    if (iou > associate_IoU_thresold && label_bbox==label_obj){
                        std::cout << "!! Associated !!" << std::endl;
                    }
                    // else {
                    //     std::cout << "Not Associated" << std::endl;
                    // }
                    std::cout << "class(bbox/obj)/IoU: " << label_bbox << "/" \
                            << label_obj << "/" << iou << std::endl;

                    cv::imshow("Ellipse Projection", img_show);
                    cv::waitKey(10);

                    std::cout << "Press any key to continue" << endl;
                    char key = getchar();
                }

                if (iou > associate_IoU_thresold && label_bbox==label_obj){
                    // cout << "Associate" << std::endl;
                    // 这里有一个问题，被关联过的物体可能在下一个det再次被遍历到
                    has_associate = true;
                    associateDetWithObject(pKF, pMO, d_i, detKF1, mvpMapPoints);
                    break;
                }
            }
            
            std::cout << "Detection " << d_i << ", class " << label_bbox ;
            
            if (has_associate) std::cout << ", associated successfully: ";
            else std::cout << " associated failed: ";
            for (auto &iou : iou_stats) {
                cout << iou << ", ";
            }
            cout << endl;
        }
        else {
            map<int, int> observed_object_id;
            int nOutliers = 0;

            for (int k_i : detKF1->GetFeaturePoints()) {
                auto pMP = mvpMapPoints[k_i];
                if (!pMP)
                    continue;
                if (pMP->isOutlier())
                {
                    nOutliers++;
                    continue;
                }

                // 在初始观测时都不加入具体的object_id，即为-1
                if (pMP->object_id < 0)
                    continue;

                if (observed_object_id.count(pMP->object_id))
                    observed_object_id[pMP->object_id] += 1;
                else
                    observed_object_id[pMP->object_id] = 1;
            }

            // Step 2.2: 根据关键点数量寻找与哪个object编号更匹配
            //           获取对应的物体指针，将物体、检测添加到关键帧，
            //           设置该 det->isNew = false
            //           将其他的检测关联地图点设置与物体关联

            if (!observed_object_id.empty())
            {
                // Find object that has the most matches
                int object_id_max_matches = 0;  // global object id
                int max_matches = 0;
                for (auto it = observed_object_id.begin(); it != observed_object_id.end(); it++) {
                    if (it->second > max_matches) {
                        max_matches = it->second;
                        object_id_max_matches = it->first;
                    }
                }
                // associated object
                // todo: 这里的关联方式过于粗糙，只要有相同地图点就关联到一起了
                // 增加判断条件： 除了地图点验证，再增加椭球体的IOU验证
                // bool match = CheckIoU();
                if (max_matches > minimum_match_to_associate)
                {
                    // std::cout << ""
                    auto pMO = mpMap->GetMapObject(object_id_max_matches);
                    int newly_matched_points = associateDetWithObject(pKF, pMO, d_i, detKF1, mvpMapPoints);
                    cout <<  "Matches: " << max_matches << ", New points: " << newly_matched_points << ", Keypoints: " <<
                        detKF1->mvKeysIndices.size() << ", Associated to object by projection " << object_id_max_matches
                        << endl << endl;

                }
                else
                {
                    std::cout << "No association because of few MP shared" << std::endl;
                }
            }
        }
    }
}

// KEY: 将一个观测关联到某个物体之后的操作：
/**
 * 1. 对关键帧和物体进行分别的设置
 * 2. （Done）地图点融合
*/
int Tracking::associateDetWithObject(ORB_SLAM2::KeyFrame *pKF, MapObject* pMO, int d_i, ObjectDetection* detKF1, vector<MapPoint*>& mvpMapPoints)
{
    // 设置该帧的某个观测对应的物体
    pKF->AddMapObject(pMO, d_i);
    pMO->AddObservation(pKF, d_i);

    // 设置物体所包含的观测
    detKF1->isNew = false;

    int associate_object_id = pMO->mnId;
    // pMO

    // add newly detected feature points to object
    int newly_matched_points = 0;
    for (int k_i : detKF1->GetFeaturePoints()) {
        auto pMP = mvpMapPoints[k_i];
        if (pMP && !pMP->isBad())
        {
            // new map points
            if (pMP->object_id < 0)
            {
                pMP->in_any_object = true;
                pMP->object_id = associate_object_id;
                pMO->AddMapPoints(pMP);
                newly_matched_points++;
            }
            else
            {
                // if pMP is already associate to a different object, set bad flag
                // 一个特征点在不同帧可以在不同物体的mask内
                if (pMP->object_id != associate_object_id)
                    pMP->SetBadFlag();
            }
        }
    }

    return newly_matched_points;

    // cout <<  "Matches: " << max_matches << ", New points: " << newly_matched_points << ", Keypoints: " <<
    //     detKF1->mvKeysIndices.size() << ", Associated to object by projection " << object_id_max_matches
    //     << endl << endl;
    /*cout <<  "Matches: " << max_matches << ", New points: " << newly_matched_points << ", Keypoints: " <<
        detKF1->mvKeysIndices.size() << ", Associated to object by projection " << object_id_max_matches
        << endl << endl;*/
}

void Tracking::SetImageNames(vector<string>& vstrImageFilenamesRGB)
{
    mvstrImageFilenamesRGB.resize(vstrImageFilenamesRGB.size());
    mvstrImageFilenamesRGB = std::vector<string>(vstrImageFilenamesRGB.begin(), vstrImageFilenamesRGB.end());
}

void Tracking::OpenGroundPlaneEstimation(){
    miGroundPlaneState = 1;
    PlaneExtractorParam param;
    param.fx = mK.at<float>(0,0);
    param.fy = mK.at<float>(1,1);
    param.cx = mK.at<float>(0,2);
    param.cy = mK.at<float>(1,2);
    param.scale = Config::Get<double>("Camera.scale");
    pPlaneExtractor = new PlaneExtractor;
    pPlaneExtractor->SetParam(param);

    // Manhattan Task
    miMHPlanesState = 1;
    pPlaneExtractorManhattan = new PlaneExtractorManhattan;
    pPlaneExtractorManhattan->SetParam(param);

    std::cout << " * Open Groundplane Estimation" << std::endl;
    std::cout << std::endl;
}

// TODO: 更新物体观测
void Tracking::UpdateObjectObservation_GenerateEllipsoid(ORB_SLAM2::Frame *pFrame, KeyFrame* pKF, bool withAssociation) {
    clock_t time_0_start = clock();
    // 
    bool use_infer_detection = Config::Get<int>("System.MonocularInfer.Open") > 0;

    // [2] process single-frame ellipsoid estimation
    if(!use_infer_detection){
        // 使用深度图像估计物体椭球体
        UpdateDepthEllipsoidEstimation(pFrame, pKF, withAssociation);

        // [3] Extract Relationship
        // 构建椭球体与曼哈顿平面之间的关联关系
        TaskRelationship(pFrame);

        // // [4] Use Relationship To Refine Ellipsoids
        // // 注意: Refine时必然在第一步可以初始化出有效的物体.
        RefineObjectsWithRelations(pFrame);

        // // [5] 对于第一次提取，Refine提取都失败的，使用点模型
        // UpdateDepthEllipsoidUsingPointModel(pFrame);

        GenerateObservationStructure(pFrame);
    }
    // RGBD模式，可以对多个物体建模，并且同时使用了point椭球和infer椭球
    else if(mSensor==System::RGBD){
        // 使用深度图像估计物体椭球体
        UpdateDepthAndInferEllipsoidEstimation(pFrame, pKF, withAssociation);

        // [3] Extract Relationship
        // 构建椭球体与曼哈顿平面之间的关联关系
        TaskRelationship(pFrame);

        // // [4] Use Relationship To Refine Ellipsoids
        // // 注意: Refine时必然在第一步可以初始化出有效的物体.
        RefineObjectsWithRelations(pFrame);

        // // [5] 对于第一次提取，Refine提取都失败的，使用点模型
        // UpdateDepthEllipsoidUsingPointModel(pFrame);

        GenerateObservationStructure(pFrame);
    }
    // 单目模式，只能处理一个物体，无法使用point椭球体
    else if(mSensor==System::MONOCULAR)
    {

        pFrame->SetObservations(pKF);  

        // const string priconfig_path = Config::Get<std::string>("Dataset.Path.PriTable");
        // bool bUseInputPri = (priconfig_path.size() > 0);
        // InferObjectsWithSemanticPrior(pFrame, bUseInputPri, use_infer_detection); // 使用PriTable先验初始化，然而存在问题，尚未调试完毕

        InferObjectsWithSemanticPrior(pFrame, false, use_infer_detection);   // 使用 1:1:1 的比例初始化

    }

    // // Output running time
    // // cout << " -- UpdateObjectObservation Time: " << endl;
    // // cout << " --- time_1_TaskGroundPlane: " <<(double)(time_1_TaskGroundPlane - time_0_start) / CLOCKS_PER_SEC << "s" << endl;        
    // // cout << " --- time_2_TaskManhattanPlanes: " <<(double)(time_2_TaskManhattanPlanes - time_1_TaskGroundPlane) / CLOCKS_PER_SEC << "s" << endl;        
    // // cout << " --- time_3_UpdateDepthEllipsoidEstimation: " <<(double)(time_3_UpdateDepthEllipsoidEstimation - time_2_TaskManhattanPlanes) / CLOCKS_PER_SEC << "s" << endl;        
    // // cout << " --- time_4_TaskRelationship: " <<(double)(time_4_TaskRelationship - time_3_UpdateDepthEllipsoidEstimation) / CLOCKS_PER_SEC << "s" << endl;        
    // // cout << " --- time_5_RefineObjectsWithRelations: " <<(double)(time_5_RefineObjectsWithRelations - time_4_TaskRelationship) / CLOCKS_PER_SEC << "s" << endl;        

}

void Tracking::TaskGroundPlane()
{
    // // if (地面估计没有成功时进行)
    // ProcessGroundPlaneEstimation();
    // // int miGroundPlaneState; // 0: Closed  1: estimating 2: estimated 3: set by mannual
    if(miGroundPlaneState == 1) // State 1: Groundplane estimation opened, and not done yet.
        ProcessGroundPlaneEstimation();
    else if(miGroundPlaneState == 3) // State : Set by mannual
        ActivateGroundPlane(mGroundPlane);
}


void Tracking::ProcessGroundPlaneEstimation()
{
    // cv::Mat depth = mCurrentFrame.mImDepth;
    cv::Mat depth = mImDepth;
    g2o::plane groundPlane;
    bool result = pPlaneExtractor->extractGroundPlane(depth, groundPlane);
    g2o::SE3Quat& Twc = mCurrentFrame.cam_pose_Twc;

    // // Camera pose.
    // // cv::Mat Tcw_mat = mCurrentFrame.mTcw;
    
    // // g2o::SE3Quat& Twc = mCurrentFrame.cam_pose_Twc;
    // cv::Mat Twc_mat;
    // cv::invert(mCurrentFrame.mTcw, Twc_mat);
    // // g2o::SE3Quat& Twc = Converter::toSE3Quat(Twc_mat);
    // g2o::SE3Quat Twc = Converter::toSE3Quat(Twc_mat);

    // 可视化[所有平面]结果 : 放这里为了让Mannual Check 看见
    auto vPotentialPlanePoints = pPlaneExtractor->GetPoints();
    mpMap->AddPointCloudList("pPlaneExtractor.PlanePoints", vPotentialPlanePoints, Twc, REPLACE_POINT_CLOUD);
    std::cout << " Extract Plane Num : " << vPotentialPlanePoints.size() << std::endl;

    if( result )
    {        
        // 设置世界地平面
        std::cout << " * [Local] Ground plane : " << groundPlane.param.transpose() << std::endl;
        groundPlane.transform(Twc);   // transform to the world coordinate.
        mGroundPlane = groundPlane;
        std::cout << " * Estimate Ground Plane Succeeds: " << mGroundPlane.param.transpose() << std::endl;

        // 可视化该平面 : 为了 Mannual Check.
        mGroundPlane.color = Eigen::Vector3d(0.0,0.8,0.0); 
        mGroundPlane.InitFinitePlane(Twc.translation(), 10);
        mpMap->addPlane(&mGroundPlane);

        // Active the mannual check of groundplane estimation.
        int active_mannual_groundplane_check = Config::Get<int>("Plane.MannualCheck.Open");
        int key = -1;
        bool open_mannual_check = active_mannual_groundplane_check==1;
        bool result_mannual_check = false;
        if(open_mannual_check)
        {
            std::cout << "Estimate Groundplane Done." << std::endl;
            std::cout << "As Groundplane estimation is a simple implementation, please mannually check its correctness." << std::endl;
            std::cout << "Enter Key \'Y\' to confirm, and any other key to cancel this estimation: " << std::endl;

            key = getchar();
        }

        result_mannual_check = (key == 'Y' || key == 'y');            

        if( !open_mannual_check || (open_mannual_check &&  result_mannual_check) )
        {
            ActivateGroundPlane(mGroundPlane);
        }
        else
        {
            std::cout << " * Cancel this Estimation. " << std::endl;
            miGroundPlaneState = 1;
        }

        // 可视化 : [所有潜在地平面], 从中选择了距离最近的一个
        auto vPotentialGroundplanePoints = pPlaneExtractor->GetPotentialGroundPlanePoints();
        mpMap->AddPointCloudList("pPlaneExtractor.PlanePoints", vPotentialGroundplanePoints, Twc, REPLACE_POINT_CLOUD);
    }
    else
    {
        std::cout << " * Estimate Ground Plane Fails " << std::endl;
    }
}

void Tracking::ActivateGroundPlane(g2o::plane &groundplane)
{
    // Set groundplane to EllipsoidExtractor
    
    if( mbOpenDepthEllipsoid ){
        std::cout << " * Add supporting plane to Ellipsoid Extractor." << std::endl;
        mpEllipsoidExtractor->SetSupportingPlane(&groundplane, false);
    }

    // Set groundplane to Optimizer
    std::cout << " * Add supporting plane to optimizer. " << std::endl;
    mpOptimizer->SetGroundPlane(groundplane.param);

    std::cout << " * Add supporting plane to Manhattan Plane Extractor. " << std::endl;
    pPlaneExtractorManhattan->SetGroundPlane(&groundplane);

    // Change state
    miGroundPlaneState = 2;
}

void Tracking::OpenDepthEllipsoid(){
    mbOpenDepthEllipsoid = true;

    mpEllipsoidExtractor = new EllipsoidExtractor;
    
    // Open visualization during the estimation process
    mpEllipsoidExtractor->OpenVisualization(mpMap);

    // Open symmetry
    if(Config::Get<int>("EllipsoidExtraction.Symmetry.Open") == 1)
        mpEllipsoidExtractor->OpenSymmetry();

    std::cout << std::endl;
    cout << " * Open Single-Frame Ellipsoid Estimation. " << std::endl;
    std::cout << std::endl;
}

// Debug函数用于可视化
void VisualizeCuboidsPlanesInImages(g2o::ellipsoid& e, const g2o::SE3Quat& campose_wc, const Matrix3d& calib, int rows, int cols, Map* pMap)
{
    g2o::ellipsoid e_global = e.transform_from(campose_wc);
    Vector3d center = e_global.pose.translation();


    std::vector<plane*> pPlanes = e.GetCubePlanesInImages(g2o::SE3Quat(), calib, rows, cols, 30);
    int planeNum = pPlanes.size();
    // std::cout << "[Tracking::VisualizeCuboidsPlanesInImages] planeNum = " << planeNum << std::endl;
    for( int i = 0; i < planeNum; i++){
        Vector4d planeVec = pPlanes[i]->param.head(4);
        Vector3d color(0,0,1.0);  
        double plane_size = e.scale.norm()/2.0;

        g2o::plane *pPlane = new g2o::plane(planeVec, color);
        pPlane->transform(campose_wc);
        pPlane->InitFinitePlane(center, plane_size);
        pMap->addPlane(pPlane);
    }
}



void Tracking::UpdateDepthAndInferEllipsoidEstimation(ORB_SLAM2::Frame* pFrame, KeyFrame* pKF, bool withAssociation)
{
    if( !mbOpenDepthEllipsoid ) return;

    auto mvpObjectDetections = pKF->GetObjectDetections();

    Eigen::MatrixXd &obs_mat = pFrame->mmObservations;
    int rows = obs_mat.rows();

    Eigen::VectorXd pose = pFrame->cam_pose_Twc.toVector();

    // 每次清除一下椭球体提取器的点云
    // mpEllipsoidExtractor->ClearPointCloudList();    // clear point cloud visualization

    bool bPlaneNotClear = true;

    // 每次更新深度观测的时候都清除
    bool bEllipsoidNotClear = true;
    std::cout << "[Tracking::UpdateDepthEllipsoidEstimation] " << std::endl;
    std::cout << "共有 " << rows << " 个检测结果" << std::endl;
    std::string pcd_suffix;

    for(int i = 0; i < rows; i++){

        auto det = mvpObjectDetections[i];

        if (add_suffix_to_pcd){
            pcd_suffix = to_string(i);
        }
        else {
            pcd_suffix = "";
        }

        Eigen::VectorXd det_vec = obs_mat.row(i);  // id x1 y1 x2 y2 label rate instanceID

        std::cout << "\n=> Det " << i << ": " << det_vec.transpose().matrix() << std::endl;

        int label = round(det_vec(5));
        double measurement_prob = det_vec(6);

        Eigen::Vector4d measurement = Eigen::Vector4d(det_vec(1), det_vec(2), det_vec(3), det_vec(4));

        // Filter those detections lying on the border.
        bool is_border = calibrateMeasurement(measurement, mRows, mCols, Config::Get<int>("Measurement.Border.Pixels"), Config::Get<int>("Measurement.LengthLimit.Pixels"));

        // FIXME: 这里涉及到对观测框靠近边界的物体观测如何处理的问题：暂时在python检测中去除靠近边界的检测

        if (is_border) {
            cout << ">> [is_border] "
                 << "measure: " << measurement.transpose().matrix()
                 << ", row: " << mRows << ", col: " << mCols
                 << ", length_limit: " << Config::Get<int>("Measurement.Border.Pixels")
                 << endl;
            cout << "Press enter to continue: " << endl;
            getchar();
        }

        double prob_thresh = Config::Get<double>("Measurement.Probability.Thresh");

        bool prob_check = (measurement_prob > prob_thresh);

        g2o::ellipsoid* pLocalEllipsoidThisObservation = NULL;
        g2o::ellipsoid* pGlobalEllipsoidThisObservation = NULL;
        // 2 conditions must meet to start ellipsoid extraction:
        // C1 : the bounding box is not on border
        // C1 : 包围框是否不在边界上
        bool c1 = !is_border;

        // C2 : the groundplane has been estimated successfully
        // C2 : 地面是否被成功估计
        bool c2 = miGroundPlaneState == 2;

        // in condition 3, it will not start
        // C3 : under with association mode, and the association is invalid, no need to extract ellipsoids again.
        // C3 : 在关联模式下，但是关联关系非法，则不再对其进行椭球体提取
        bool c3 = false;
        if( withAssociation )  // 使用预定义的关联关系
        {
            int instance = round(det_vec(7));
            if ( instance < 0 ) c3 = true;  // invalid instance
        }

        // C4 : 物体过滤
        // 部分动态物体，如人类的label=0，将被过滤不考虑
        bool c4 = true;
        std::set<int> viIgnoreLabelLists = {
            0 // Human
        };
        if(viIgnoreLabelLists.find(label) != viIgnoreLabelLists.end())
            c4 = false;

        cout << "prob|NotBorder|HasGround|NotAssociation|NotFiltered:" \
             << prob_check << "," << c1 << "," << c2 << "," << !c3 << "," << c4 << std::endl;

        // 对观测进行椭球体提取的几大条件
        if( prob_check && c1 && c2 && !c3 && c4 ){
            // std::cout << "bPlaneNotClear = " << bPlaneNotClear << std::endl;
            if(bPlaneNotClear){
                std::cout << "mpMap->clearPlanes()" << std::endl;
                mpMap->clearPlanes();
                if(miGroundPlaneState == 2) // if the groundplane has been estimated
                    mpMap->addPlane(&mGroundPlane);

                VisualizeManhattanPlanes();
                bPlaneNotClear = false;
            }
            // std::cout << "*****************************" << std::endl;
            // std::cout << "Ready to EstimateLocalEllipsoidUsingMultiPlanes, press [ENTER] to continue ... " << std::endl;
            // std::cout << "*****************************" << std::endl;
            // getchar();
            // 使用多平面估计局部椭球体 (depth, label, bbox, prob, mCamera)
            // TODO： 这里有待将物体对应的深度点云添加给MapObject，可以先通过椭球体进行关联
            // 得到的椭球体模型表示在相机坐标系中

            // TODO: 这里要将物体点云添加给观测
            pcl::PointCloud<PointType>::Ptr pcd_ptr(new pcl::PointCloud<PointType>);

            // 提取椭球体
            // FIXME: 这里需要对不能提取出椭球体的情况进行一个分类处理
            g2o::ellipsoid e_extractByFitting_newSym = \
                mpEllipsoidExtractor->EstimateLocalEllipsoidUsingMultiPlanes(\
                    pFrame->frame_img, measurement, label, measurement_prob, pose, mCamera, pcd_ptr, pcd_suffix);

            cout << "e_extractByFitting_newSym.prob = " << e_extractByFitting_newSym.prob << endl;
            // FIXME: 如果点云提取不成功，则设置该帧观测为不好, 因为 mpGlobalEllipsolds 需要添加，因此不能 continue

            // 无非两个特殊情况需要考虑： 椭球体提取不成功，深度点云提取不成功
            if (pcd_ptr==NULL){
                det->isValidPcd = false;
                // continue;
            }

            if (add_depth_pcd_to_map_object && pcd_ptr!=NULL) {
                det->setPcdPtr(pcd_ptr);
            }

            bool c0 = mpEllipsoidExtractor->GetResult();
            std::cout << "mpEllipsoidExtractor->GetResult() = " << c0 << std::endl;

            // 可视化部分
            if( c0 )
            {
                // Visualize estimated ellipsoid
                // 将相机坐标系的椭球体转换到世界坐标系内
                g2o::ellipsoid* pObjByFitting = new g2o::ellipsoid(e_extractByFitting_newSym.transform_from(pFrame->cam_pose_Twc));

                if(pObjByFitting->prob_3d > 0.5)
                    pObjByFitting->setColor(Vector3d(0.8,0.0,0.0), 1); // Set green color
                else{
                    // prob_3d
                    // FIXME： 如果 prob_3d < 0.5, 使用bbox边界对ellipsold进行再次refine
                    pObjByFitting->setColor(Vector3d(0.8,0,0), 0.5); // 透明颜色
                }

                // 临时更新： 此处显示的是 3d prob
                // pObjByFitting->prob = pObjByFitting->prob_3d;

                // 第一次添加时清除上一次观测!
                if(bEllipsoidNotClear)
                {
                    mpMap->ClearEllipsoidsVisual(); // Clear the Visual Ellipsoids in the map
                    mpMap->ClearBoundingboxes();
                    bEllipsoidNotClear = false;
                }


                mpMap->addEllipsoidVisual(pObjByFitting);

                // std::cout << "Add Ellipsold" << std::endl;

                // cout << "detection " << i << " = " << pObjByFitting->pose << endl;

                // std::cout << "*****************************" << std::endl;
                // std::cout << "Show EllipsoidVisual, press [ENTER] to continue ... " << std::endl;
                // std::cout << "*****************************" << std::endl;
                // getchar();
                // 添加debug, 测试筛选图像平面内的bbox平面
                VisualizeCuboidsPlanesInImages(e_extractByFitting_newSym, pFrame->cam_pose_Twc, mCalib, mRows, mCols, mpMap);

                g2o::ellipsoid *pE_extractByFitting = new g2o::ellipsoid(e_extractByFitting_newSym);
                pLocalEllipsoidThisObservation = pE_extractByFitting;   // Store result to pE_extracted.

                g2o::ellipsoid *pE_extractByFittingGlobal = new g2o::ellipsoid(*(pObjByFitting));
                pGlobalEllipsoidThisObservation = pE_extractByFittingGlobal;
            }

        }

        if (show_ellipsold_process)
        {
            int x1 = (int)det_vec(1), y1 = (int)det_vec(2), x2 = (int)det_vec(3), y2 = (int)det_vec(4);
            cv::Mat img_show = pFrame->color_img.clone();
            cv::rectangle(img_show, cv::Point(x1, y1), cv::Point(x2, y2), cv::Scalar(255, 0, 0), 2);  // Scalar(255, 0, 0) is for blue color, 2 is the thickness
            cv::imshow("Image with Bbox", img_show);
            // cv::waitKey(0);
            cv::waitKey(10);
            cout << "Press any key to continue" << endl;
            char key = getchar();
        }

        // 若不成功保持为NULL
        // cout << "ellipsold add 1" << endl;
        pFrame->mpLocalObjects.push_back(pLocalEllipsoidThisObservation);
        // cout << "ellipsold add 2" << endl;
        pKF->mpLocalEllipsolds.push_back(pLocalEllipsoidThisObservation);
        // cout << "ellipsold add 3" << endl;

        if (pGlobalEllipsoidThisObservation==NULL){
            cout << "pGlobalEllipsoidThisObservation==NULL" << endl;
        }
        else{
            cout << "pGlobalEllipsoidThisObservation->prob = " << pGlobalEllipsoidThisObservation->prob << endl;
        }

        // 这里要注意创建一个新的
        pKF->mpGlobalEllipsolds.push_back(pGlobalEllipsoidThisObservation);

    }

    if (show_ellipsold_process){
        cv::destroyWindow("Image with Bbox");
    }
    return;
}

// Process Ellipsoid Estimation for every boundingboxes in current frame.
// Finally, store 3d Ellipsoids into the member variable mpLocalObjects of pFrame.
// 为当前帧中的每个包围框处理椭球体估计
// 最后，将3D椭球体存储到每一帧的成员变量mpLocalObjects中
void Tracking::UpdateDepthEllipsoidEstimation(ORB_SLAM2::Frame* pFrame, KeyFrame* pKF, bool withAssociation)
{
    if( !mbOpenDepthEllipsoid ) return;

    auto mvpObjectDetections = pKF->GetObjectDetections();

    Eigen::MatrixXd &obs_mat = pFrame->mmObservations;
    int rows = obs_mat.rows();

    Eigen::VectorXd pose = pFrame->cam_pose_Twc.toVector();

    // 每次清除一下椭球体提取器的点云
    // mpEllipsoidExtractor->ClearPointCloudList();    // clear point cloud visualization

    bool bPlaneNotClear = true;

    // 每次更新深度观测的时候都清除
    bool bEllipsoidNotClear = true;
    std::cout << "[Tracking::UpdateDepthEllipsoidEstimation] " << std::endl;
    std::cout << "共有 " << rows << " 个检测结果" << std::endl;
    std::string pcd_suffix;

    for(int i = 0; i < rows; i++){

        auto det = mvpObjectDetections[i];

        if (add_suffix_to_pcd){
            pcd_suffix = to_string(i);
        }
        else {
            pcd_suffix = "";
        }
        
        Eigen::VectorXd det_vec = obs_mat.row(i);  // id x1 y1 x2 y2 label rate instanceID

        std::cout << "\n=> Det " << i << ": " << det_vec.transpose().matrix() << std::endl;

        int label = round(det_vec(5));
        double measurement_prob = det_vec(6);

        Eigen::Vector4d measurement = Eigen::Vector4d(det_vec(1), det_vec(2), det_vec(3), det_vec(4));

        // Filter those detections lying on the border.
        bool is_border = calibrateMeasurement(measurement, mRows, mCols, Config::Get<int>("Measurement.Border.Pixels"), Config::Get<int>("Measurement.LengthLimit.Pixels"));

        // FIXME: 这里涉及到对观测框靠近边界的物体观测如何处理的问题：暂时在python检测中去除靠近边界的检测

        if (is_border) {
            cout << ">> [is_border] " 
                 << "measure: " << measurement.transpose().matrix()
                 << ", row: " << mRows << ", col: " << mCols 
                 << ", length_limit: " << Config::Get<int>("Measurement.Border.Pixels")
                 << endl;
            cout << "Press enter to continue: " << endl;
            getchar();
        }
        
        double prob_thresh = Config::Get<double>("Measurement.Probability.Thresh");

        bool prob_check = (measurement_prob > prob_thresh);

        g2o::ellipsoid* pLocalEllipsoidThisObservation = NULL;
        g2o::ellipsoid* pGlobalEllipsoidThisObservation = NULL;
        // 2 conditions must meet to start ellipsoid extraction:
        // C1 : the bounding box is not on border
        // C1 : 包围框是否不在边界上
        bool c1 = !is_border;

        // C2 : the groundplane has been estimated successfully
        // C2 : 地面是否被成功估计
        bool c2 = miGroundPlaneState == 2;
        
        // in condition 3, it will not start
        // C3 : under with association mode, and the association is invalid, no need to extract ellipsoids again.
        // C3 : 在关联模式下，但是关联关系非法，则不再对其进行椭球体提取
        bool c3 = false;
        if( withAssociation )
        {
            int instance = round(det_vec(7));
            if ( instance < 0 ) c3 = true;  // invalid instance
        }

        // C4 : 物体过滤
        // 部分动态物体，如人类， label=0，将被过滤不考虑
        bool c4 = true;
        std::set<int> viIgnoreLabelLists = {
            0 // Human
        };

        if(viIgnoreLabelLists.find(label) != viIgnoreLabelLists.end())
            c4 = false;

        cout << "prob|NotBorder|HasGround|NotAssociation|NotFiltered:" \
             << prob_check << "," << c1 << "," << c2 << "," << !c3 << "," << c4 << std::endl;

        // 对观测进行椭球体提取的几大条件
        if( prob_check && c1 && c2 && !c3 && c4 ){
            // std::cout << "bPlaneNotClear = " << bPlaneNotClear << std::endl;
            if(bPlaneNotClear){
                std::cout << "mpMap->clearPlanes()" << std::endl;
                mpMap->clearPlanes();
                if(miGroundPlaneState == 2) // if the groundplane has been estimated
                    mpMap->addPlane(&mGroundPlane);

                VisualizeManhattanPlanes();
                bPlaneNotClear = false;
            }
            // std::cout << "*****************************" << std::endl;
            // std::cout << "Ready to EstimateLocalEllipsoidUsingMultiPlanes, press [ENTER] to continue ... " << std::endl;
            // std::cout << "*****************************" << std::endl;
            // getchar();
            // 使用多平面估计局部椭球体 (depth, label, bbox, prob, mCamera)
            // TODO： 这里有待将物体对应的深度点云添加给MapObject，可以先通过椭球体进行关联
            // 得到的椭球体模型表示在相机坐标系中

            // TODO: 这里要将物体点云添加给观测
            pcl::PointCloud<PointType>::Ptr pcd_ptr(new pcl::PointCloud<PointType>);

            // FIXME: 这里需要对不能提取出椭球体的情况进行一个分类处理
            g2o::ellipsoid e_extractByFitting_newSym = \
                mpEllipsoidExtractor->EstimateLocalEllipsoidUsingMultiPlanes(\
                    pFrame->frame_img, measurement, label, measurement_prob, pose, mCamera, pcd_ptr, pcd_suffix);

            cout << "e_extractByFitting_newSym.prob = " << e_extractByFitting_newSym.prob << endl;
            // FIXME: 如果点云提取不成功，则设置该帧观测为不好, 因为 mpGlobalEllipsolds 需要添加，因此不能 continue

            // 无非两个特殊情况需要考虑： 椭球体提取不成功，深度点云提取不成功
            if (pcd_ptr==NULL){
                det->isValidPcd = false;
                // continue;
            }

            if (add_depth_pcd_to_map_object && pcd_ptr!=NULL) {
                det->setPcdPtr(pcd_ptr);
            }
            
            bool c0 = mpEllipsoidExtractor->GetResult();
            std::cout << "mpEllipsoidExtractor->GetResult() = " << c0 << std::endl;
            
            // 可视化部分
            if( c0 )
            {
                // Visualize estimated ellipsoid
                // 将相机坐标系的椭球体转换到世界坐标系内
                g2o::ellipsoid* pObjByFitting = new g2o::ellipsoid(e_extractByFitting_newSym.transform_from(pFrame->cam_pose_Twc));
                
                if(pObjByFitting->prob_3d > 0.5)
                    pObjByFitting->setColor(Vector3d(0.8,0.0,0.0), 1); // Set green color
                else{
                    // prob_3d
                    // FIXME： 如果 prob_3d < 0.5, 使用bbox边界对ellipsold进行再次refine
                    pObjByFitting->setColor(Vector3d(0.8,0,0), 0.5); // 透明颜色
                }

                // 临时更新： 此处显示的是 3d prob
                // pObjByFitting->prob = pObjByFitting->prob_3d;

                // 第一次添加时清除上一次观测!
                if(bEllipsoidNotClear)
                {
                    mpMap->ClearEllipsoidsVisual(); // Clear the Visual Ellipsoids in the map
                    mpMap->ClearBoundingboxes();
                    bEllipsoidNotClear = false;
                }


                mpMap->addEllipsoidVisual(pObjByFitting);

                // std::cout << "Add Ellipsold" << std::endl;
                
                // cout << "detection " << i << " = " << pObjByFitting->pose << endl;
                
                // std::cout << "*****************************" << std::endl;
                // std::cout << "Show EllipsoidVisual, press [ENTER] to continue ... " << std::endl;
                // std::cout << "*****************************" << std::endl;
                // getchar();
                // 添加debug, 测试筛选图像平面内的bbox平面
                VisualizeCuboidsPlanesInImages(e_extractByFitting_newSym, pFrame->cam_pose_Twc, mCalib, mRows, mCols, mpMap);


            // }   // successful estimation.

            // // 存储条件1: 该检测 3d_prob > 0.5
            // // 最终决定使用的估计结果
            // if( c0 ){

                g2o::ellipsoid *pE_extractByFitting = new g2o::ellipsoid(e_extractByFitting_newSym);
                pLocalEllipsoidThisObservation = pE_extractByFitting;   // Store result to pE_extracted.

                g2o::ellipsoid *pE_extractByFittingGlobal = new g2o::ellipsoid(*(pObjByFitting));
                pGlobalEllipsoidThisObservation = pE_extractByFittingGlobal;
            }

        }

        if (show_ellipsold_process) 
        {
            int x1 = (int)det_vec(1), y1 = (int)det_vec(2), x2 = (int)det_vec(3), y2 = (int)det_vec(4);
            cv::Mat img_show = pFrame->color_img.clone();
            cv::rectangle(img_show, cv::Point(x1, y1), cv::Point(x2, y2), cv::Scalar(255, 0, 0), 2);  // Scalar(255, 0, 0) is for blue color, 2 is the thickness
            cv::imshow("Image with Bbox", img_show);
            // cv::waitKey(0);
            cv::waitKey(10);
            cout << "Press any key to continue" << endl;
            char key = getchar();
        }

        // 若不成功保持为NULL
        // cout << "ellipsold add 1" << endl;
        pFrame->mpLocalObjects.push_back(pLocalEllipsoidThisObservation);
        // cout << "ellipsold add 2" << endl;
        pKF->mpLocalEllipsolds.push_back(pLocalEllipsoidThisObservation);
        // cout << "ellipsold add 3" << endl;

        if (pGlobalEllipsoidThisObservation==NULL){
            cout << "pGlobalEllipsoidThisObservation==NULL" << endl;
        }
        else{
            cout << "pGlobalEllipsoidThisObservation->prob = " << pGlobalEllipsoidThisObservation->prob << endl;
        }

        // 这里要注意创建一个新的
        pKF->mpGlobalEllipsolds.push_back(pGlobalEllipsoidThisObservation);

    }

    if (show_ellipsold_process){
        cv::destroyWindow("Image with Bbox");
    }
    return;
}

// 构建椭球体与曼哈顿平面之间的关联关系
void Tracking::TaskRelationship(ORB_SLAM2::Frame *pFrame)
{
    std::vector<g2o::ellipsoid*>& vpEllipsoids = pFrame->mpLocalObjects;
    // 获得局部 planes.
    std::vector<g2o::plane*> vpPlanes = pPlaneExtractorManhattan->GetPotentialMHPlanes();

    Relations rls = mpRelationExtractor->ExtractSupporttingRelations(vpEllipsoids, vpPlanes, pFrame, QUADRIC_MODEL);

    if(rls.size()>0)
    {
        // 将结果存储到 frame 中
        pFrame->mbSetRelation = true;
        pFrame->relations = rls;
    }

    // ****************************
    //          可视化部分
    // ****************************
    g2o::SE3Quat Twc = pFrame->cam_pose_Twc;
    std::vector<PointCloudPCL> vPlanePoints = pPlaneExtractorManhattan->GetPotentialMHPlanesPoints();
    mpMap->AddPointCloudList("Relationship.Relation Planes", vPlanePoints, Twc, REPLACE_POINT_CLOUD);

    // 可视化该关系
    VisualizeRelations(rls, mpMap, Twc, vPlanePoints); // 放到地图中去显示?

    // std::cout << "Objects: " << vpEllipsoids.size() << std::endl;
    // std::cout << "Relation Planes : " << vpPlanes.size() << std::endl;
    // std::cout << "Relations : " << rls.size() << std::endl;
}

// *******
// 
// 1) 基于局部提取的平面，做一次分割以及椭球体提取
// 2) 若该椭球体满足 IoU >0.5, 则替换掉之前的
// 3) 若不满足，则使用点云中心+bbox产生点模型椭球体
void Tracking::RefineObjectsWithRelations(ORB_SLAM2::Frame *pFrame)
{
    // 获取该帧
    Relations& rls = pFrame->relations;
    int num = rls.size();

    Eigen::VectorXd pose = pFrame->cam_pose_Twc.toVector();

    int success_num = 0;
    for(int i=0;i<num;i++){
        // 对于支撑关系, 且平面非地平面
        // 将该新平面加入到 MHPlanes 中，重新计算一遍提取.
        Relation& rl = rls[i];
        if(rl.type == 1){   // 支撑关系
            g2o::plane* pSupPlane = rl.pPlane;  // 局部坐标系的平面位置. TODO: 检查符号
            int obj_id = rl.obj_id;
            // 此处需要bbox位置.
            // cv::Mat& depth, Eigen::Vector4d& bbox, int label, double prob, Eigen::VectorXd &pose, camera_intrinsic& camera
            Eigen::VectorXd det_vec = pFrame->mmObservations.row(obj_id);  // id x1 y1 x2 y2 label rate imageID
            int label = round(det_vec(5));
            Eigen::Vector4d bbox = Eigen::Vector4d(det_vec(1), det_vec(2), det_vec(3), det_vec(4));
            double prob = det_vec(6);

            g2o::ellipsoid e = mpEllipsoidExtractor->EstimateLocalEllipsoidWithSupportingPlane(pFrame->frame_img, bbox, label, prob, pose, mCamera, pSupPlane); // 取消
            // 该提取不再放入 world? 不, world MHPlanes 还是需要考虑的.

            // 可视化该 Refined Object
            bool c0 = mpEllipsoidExtractor->GetResult();
            std::cout << "Refined mpEllipsoidExtractor->GetResult()" << c0 << std::endl;
            if( c0 )
            {
                // Visualize estimated ellipsoid
                g2o::ellipsoid* pObjRefined = new g2o::ellipsoid(e.transform_from(pFrame->cam_pose_Twc));
                pObjRefined->setColor(Vector3d(0,0.8,0), 1); 
                mpMap->addEllipsoidVisual(pObjRefined);

                // 存储条件1: 该检测 3d_prob > 0.5
                // bool c1 = (e.prob_3d > 0.5);
                // 最终决定使用的估计结果
                // if( c0 && c1 ){
                    // (*pFrame->mpLocalObjects[obj_id]) = e;                    
                    // success_num++;

                // }

                // 此处设定 Refine 一定优先.
                (*pFrame->mpLocalObjects[obj_id]) = e;                    
                success_num++;
            }
        }
    }
    std::cout << "Refine result : " << success_num << " objs." << std::endl;
}

// void Tracking::UpdateDepthEllipsoidUsingPointModel(ORB_SLAM2::Frame* pFrame)
// {
//     if( !mbDepthEllipsoidOpened ) return;
    
//     Eigen::MatrixXd &obs_mat = pFrame->mmObservations;
//     int rows = obs_mat.rows();
//     Eigen::VectorXd pose = pFrame->cam_pose_Twc.toVector();

//     if(pFrame->mpLocalObjects.size() < rows) return;

//     int count_point_model = 0;
//     int count_ellipsoid_model = 0;
//     for(int i=0;i<rows;i++){
//         // 下列情况不再进行点模型提取
//         // 1) 在前面的过程中，已经产生了有效的椭球体， not NULL, prob_3d > 0.5
//         bool c0 = pFrame->mpLocalObjects[i] != NULL && pFrame->mpLocalObjects[i]->prob_3d > 0.5;
//         if(c0) 
//         {
//             count_ellipsoid_model ++;
//             continue;
//         }

//         // ***************
//         //  此处执行前有大量条件判断，搬照UpdateDepthEllipsoidEstimation
//         // ***************
//         Eigen::VectorXd det_vec = obs_mat.row(i);  // id x1 y1 x2 y2 label rate instanceID
//         int label = round(det_vec(5));
//         double measurement_prob = det_vec(6);

//         Eigen::Vector4d measurement = Eigen::Vector4d(det_vec(1), det_vec(2), det_vec(3), det_vec(4));

//         // Filter those detections lying on the border.
//         bool is_border = calibrateMeasurement(measurement, mRows, mCols, Config::Get<int>("Measurement.Border.Pixels"), Config::Get<int>("Measurement.LengthLimit.Pixels"));
//         double prob_thresh = Config::Get<double>("Measurement.Probability.Thresh");
//         bool prob_check = (measurement_prob > prob_thresh);

//         // 2 conditions must meet to start ellipsoid extraction:
//         // C1 : the bounding box is not on border
//         bool c1 = !is_border;

//         // C2 : the groundplane has been estimated successfully
//         bool c2 = miGroundPlaneState == 2;
        
//         // in condition 3, it will not start
//         // C3 : under with association mode, and the association is invalid, no need to extract ellipsoids again.
//         bool c3 = false;

//         // C4 : 物体过滤
//         // 部分动态物体，如人类， label=0，将被过滤不考虑
//         bool c4 = true;
//         std::set<int> viIgnoreLabelLists = {
//             0 // Human
//         };
//         if(viIgnoreLabelLists.find(label) != viIgnoreLabelLists.end())
//             c4 = false;

//         if( prob_check && c1 && c2 && !c3 && c4 ){   
//             // 单帧椭球体估计失败，或 prob_3d 概率太低，激活中心点估计
//             // 为考虑三维分割效果，将其投影到图像平面，与bbox做对比；另一种方法：将概率变得更为显著.
//             g2o::ellipsoid e_extracted;
//             bool result = mpEllipsoidExtractor->EstimateLocalEllipsoidUsingPointModel(pFrame->frame_img, measurement, label, measurement_prob, pose, mCamera, e_extracted);
//             if(result){
//                 // 可视化
//                 // Visualize estimated ellipsoid
//                 g2o::ellipsoid* pObj_world = new g2o::ellipsoid(e_extracted.transform_from(pFrame->cam_pose_Twc));
//                 pObj_world->setColor(Vector3d(0,1.0,0.0), 1); // Set green color

//                 mpMap->addEllipsoidVisual(pObj_world);

//                 g2o::ellipsoid* pEllipsoid = new g2o::ellipsoid(e_extracted);
//                 pFrame->mpLocalObjects[i] = pEllipsoid;
//                 count_point_model++;
//             } 
//         }
//     }

//     std::cout << "[Observations of frame] Total Num : " << rows << std::endl;
//     std::cout << " - Ellipsoid Model: " << count_ellipsoid_model << std::endl;
//     std::cout << " - Point Model : " << count_point_model << std::endl;
//     std::cout << " - Invalid : " << rows-count_ellipsoid_model-count_point_model << std::endl;

//     return;

// }

// void Tracking::GenerateObservationStructure(ORB_SLAM2::Frame* pFrame)
// {
//     // 本存储结构以物体本身观测为索引.
//     // pFrame->meas;
    
//     Eigen::MatrixXd &obs_mat = pFrame->mmObservations;
//     int ob_num = obs_mat.rows();

//     // 3d ob
//     std::vector<g2o::ellipsoid*> pLocalObjects = pFrame->mpLocalObjects;

//     // if(ob_num != pLocalObjects.size()) 
//     // {
//     //     std::cout << " [Error] 2d observations and 3d observations should have the same size." << std::endl;
//     //     return;
//     // }

//     for( int i = 0; i < ob_num; i++)
//     {
//         Eigen::VectorXd det_vec = obs_mat.row(i);  // id x1 y1 x2 y2 label rate imageID
//         int label = round(det_vec(5));
//         Eigen::Vector4d bbox = Eigen::Vector4d(det_vec(1), det_vec(2), det_vec(3), det_vec(4));

//         Observation ob_2d;
//         ob_2d.label = label;
//         ob_2d.bbox = bbox;
//         ob_2d.rate = det_vec(6);
//         ob_2d.pFrame = pFrame;

//         Observation3D ob_3d;
//         ob_3d.pFrame = pFrame;
//         if(pLocalObjects.size() == ob_num)
//             ob_3d.pObj = pLocalObjects[i];
//         else 
//             ob_3d.pObj = NULL;

//         Measurement m;
//         m.measure_id = i;
//         m.instance_id = -1; // not associated
//         m.ob_2d = ob_2d;
//         m.ob_3d = ob_3d;
//         pFrame->meas.push_back(m);
//     }
// }

void Tracking::VisualizeManhattanPlanes()
{
    // 可视化提取结果.
    std::vector<g2o::plane*> vDominantMHPlanes = pPlaneExtractorManhattan->GetDominantMHPlanes();
    for(auto& vP:vDominantMHPlanes ) {
        mpMap->addPlane(vP);
    }
}

void VisualizeRelations(Relations& rls, Map* pMap, g2o::SE3Quat &Twc, std::vector<PointCloudPCL>& vPlanePoints)
{
    int num = rls.size();
    // std::cout << "Relation Num: " << num << std::endl;
    
    int mode = 0;   //clear

    pMap->clearArrows();
    for(int i=0;i<num;i++)
    {
        Relation &rl = rls[i];
        g2o::ellipsoid* pEllip = rl.pEllipsoid;
        g2o::plane* pPlane = rl.pPlane;
        if(pEllip==NULL || pPlane==NULL) {
            std::cout << "[Relation] NULL relation : " << rl.obj_id << ", " << rl.plane_id << std::endl;
            continue;
        }
        g2o::ellipsoid e_world = pEllip->transform_from(Twc);
        g2o::plane* plane_world = new g2o::plane(*pPlane); plane_world->transform(Twc);
        Vector3d obj_center = e_world.pose.translation();
        Vector3d norm = plane_world->param.head(3); norm.normalize();
        double length = 0.5; norm = norm * length;

        if(rl.type == 1)    // 支撑
        {
            // 即在物体底端产生一个向上大竖直箭头.
            // 以物体为中心.
            // 以平面法向量为方向.
            pMap->addArrow(obj_center, norm, Vector3d(0,1.0,0));
        }
        else if(rl.type == 2) // 倚靠
        {
            // 同上
            pMap->addArrow(obj_center, norm, Vector3d(0,0,1.0));
        }

        // 同时高亮平面.
        plane_world->InitFinitePlane(obj_center, 0.7);
        plane_world->color = Vector3d(0, 1, 1);    // 黄色显示关系面
        pMap->addPlane(plane_world);

        // 高亮对应平面的点云
        int plane_id = rl.plane_id;
        if(plane_id >= 0 && plane_id < vPlanePoints.size())
        {
            PointCloudPCL::Ptr pCloudPCL(new PointCloudPCL(vPlanePoints[rl.plane_id]));
            ORB_SLAM2::PointCloud cloudQuadri = pclToQuadricPointCloud(pCloudPCL);
            ORB_SLAM2::PointCloud* pCloudGlobal = transformPointCloud(&cloudQuadri, &Twc);
            
            int r = 0;
            int g = 255;
            int b = 255;
            SetPointCloudProperty(pCloudGlobal, r, g, b, 4);
            pMap->AddPointCloudList(string("Relationship.Activiate Sup-Planes"), pCloudGlobal, mode);
            if(mode == 1){
                delete pCloudGlobal;    // 该指针对应的点云已被拷贝到另一个指针点云,清除多余的一个
                pCloudGlobal = NULL;
            }

            mode = 1;   // 仅仅第一次清除.
        }
        else 
        {
            std::cout << "Invalid plane_id : " << plane_id << std::endl;
        }
        
    }
}

void Tracking::ManageMemory()
{
    cout << "ManageMemory" << endl;
    if(mvpFrames.size() > 1){
        Frame* pLastFrame = mvpFrames[mvpFrames.size()-2];

        // std::cout << "Going to release image..." << std::endl;
        // getchar();
        pLastFrame->frame_img.release();
        pLastFrame->color_img.release();
        // std::cout << "Released rgb and depth images." << std::endl;
    }
}

bool Tracking::SavePointCloudMap(const string& path)
{
    std::cout << "Save pointcloud Map to : " << path << std::endl;
    mpBuilder->saveMap(path);

    return true;
}

Builder* Tracking::GetBuilder()
{
    return mpBuilder;
}

void Tracking::SetFrameByFrame()
{
    unique_lock<mutex> (mMutexFrameByFrame);
    frame_by_frame = true;
}


void Tracking::GenerateObservationStructure(ORB_SLAM2::Frame* pFrame)
{
    // 本存储结构以物体本身观测为索引.
    // pFrame->meas;
    
    Eigen::MatrixXd &obs_mat = pFrame->mmObservations;
    int ob_num = obs_mat.rows();

    // 3d ob
    std::vector<g2o::ellipsoid*> pLocalObjects = pFrame->mpLocalObjects;

    // if(ob_num != pLocalObjects.size()) 
    // {
    //     std::cout << " [Error] 2d observations and 3d observations should have the same size." << std::endl;
    //     return;
    // }

    for( int i = 0; i < ob_num; i++)
    {
        Eigen::VectorXd det_vec = obs_mat.row(i);  // id x1 y1 x2 y2 label rate imageID
        int label = round(det_vec(5));
        Eigen::Vector4d bbox = Eigen::Vector4d(det_vec(1), det_vec(2), det_vec(3), det_vec(4));

        Observation ob_2d;
        ob_2d.label = label;
        ob_2d.bbox = bbox;
        ob_2d.rate = det_vec(6);
        ob_2d.pFrame = pFrame;

        Observation3D ob_3d;
        ob_3d.pFrame = pFrame;
        if(pLocalObjects.size() == ob_num)
            ob_3d.pObj = pLocalObjects[i];
        else 
            ob_3d.pObj = NULL;

        Measurement m;
        m.measure_id = i;
        m.instance_id = -1; // not associated
        m.ob_2d = ob_2d;
        m.ob_3d = ob_3d;
        pFrame->meas.push_back(m);
    }

    
}

std::vector<Frame*> Tracking::GetAllFramesWithKeyframe()
{
    return mvpFrames;
}



void Tracking::AssociateObjectsByProjection_mono_nouse(ORB_SLAM2::KeyFrame *pKF)
{
    auto mvpMapPoints = pKF->GetMapPointMatches();
    // Try to match and triangulate key-points with last key-frame
    auto detectionsKF1 = pKF->mvpDetectedObjects;
    for (int d_i = 0; d_i < detectionsKF1.size(); d_i++)
    {
        // cout << "Detection: " << d_i + 1 << endl;
        auto detKF1 = detectionsKF1[d_i];
        map<int, int> observed_object_id;
        int nOutliers = 0;
        for (int k_i : detKF1->GetFeaturePoints()) {
            auto pMP = mvpMapPoints[k_i];
            if (!pMP)
                continue;
            if (pMP->isOutlier())
            {
                nOutliers++;
                continue;
            }

            if (pMP->object_id < 0)
                continue;

            if (observed_object_id.count(pMP->object_id))
                observed_object_id[pMP->object_id] += 1;
            else
                observed_object_id[pMP->object_id] = 1;
        }

        // If associated with an object
        if (!observed_object_id.empty())
        {
            // Find object that has the most matches
            int object_id_max_matches = 0;  // global object id
            int max_matches = 0;
            for (auto it = observed_object_id.begin(); it != observed_object_id.end(); it++) {
                if (it->second > max_matches) {
                    max_matches = it->second;
                    object_id_max_matches = it->first;
                }
            }

            // associated object
            auto pMO = mpMap->GetMapObject(object_id_max_matches);
            pKF->AddMapObject(pMO, d_i);
            detKF1->isNew = false;

            // add newly detected feature points to object
            int newly_matched_points = 0;
            for (int k_i : detKF1->GetFeaturePoints()) {
                auto pMP = mvpMapPoints[k_i];
                if (pMP)
                {
                    if (pMP->isBad())
                        continue;
                    // new map points
                    if (pMP->object_id < 0)
                    {
                        pMP->in_any_object = true;
                        pMP->object_id = object_id_max_matches;
                        pMO->AddMapPoints(pMP);
                        newly_matched_points++;
                    }
                    else
                    {
                        // if pMP is already associate to a different object, set bad flag
                        if (pMP->object_id != object_id_max_matches)
                            pMP->SetBadFlag();
                    }
                }
            }
            /*cout <<  "Matches: " << max_matches << ", New points: " << newly_matched_points << ", Keypoints: " <<
                 detKF1->mvKeysIndices.size() << ", Associated to object by projection " << object_id_max_matches
                 << endl << endl;*/
        }

    }
}


}