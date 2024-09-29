#include "include/core/Tracking.h"
#include "src/symmetry/PointCloudFilter.h"

// 必须要重载
namespace EllipsoidSLAM{
class TrackingNP : public Tracking
{
public:
    TrackingNP(System* pSys, FrameDrawer* pFrameDrawer, MapDrawer* pMapDrawer, Map* pMap,
        const string &strSettingPath):Tracking(pSys, pFrameDrawer, pMapDrawer, pMap,strSettingPath){};

    bool EstimateLocalEllipsoidUsingPointModel(cv::Mat& depth, Eigen::Vector4d& bbox, int label, double prob, Eigen::VectorXd &pose, camera_intrinsic& camera, g2o::ellipsoid& e_extracted)
    {
        PointCloud cloud = getPointCloudInRect(depth, bbox, camera);
        if(cloud.size() < 10) return false;  // 最低要求.

        // 获得中点.
        PointCloudPCL::Ptr pCloud = QuadricPointCloudToPcl(cloud);

        Eigen::Vector4d centroid;
        pcl::compute3DCentroid(*pCloud, centroid);

        // 构造椭球体.
        Vector9d e_vec;
        e_vec << centroid.head(3), 0, 0, 0, 0.05, 0.05, 0.05;
        g2o::ellipsoid e; e.fromMinimalVector(e_vec);

        // 设置 label
        e.miLabel = label;
        e.prob = 1;
        e.bbox = bbox;

        e_extracted = e;
        return true;
    }

    void UpdateObjectObservationWithPointModel(Frame* pFrame, bool withAssociation)
    {
        if( !mbDepthEllipsoidOpened ) {
            std::cout << "Ellipsoid estimation closed." << std::endl;
            return;
        }

        Eigen::MatrixXd &obs_mat = pFrame->mmObservations;
        int rows = obs_mat.rows();

        Eigen::VectorXd pose = pFrame->cam_pose_Twc.toVector();

        bool bPlaneNotClear = true;
        bool bEllipsoidNotClear = true;
        for(int i=0;i<rows;i++){
            Eigen::VectorXd det_vec = obs_mat.row(i);  // id x1 y1 x2 y2 label rate instanceID
            int label = round(det_vec(5));
            double measurement_prob = det_vec(6);

            Eigen::Vector4d measurement = Eigen::Vector4d(det_vec(1), det_vec(2), det_vec(3), det_vec(4));

            // Filter those detections lying on the border.
            bool is_border = calibrateMeasurement(measurement, mRows, mCols, Config::Get<int>("Measurement.Border.Pixels"), Config::Get<int>("Measurement.LengthLimit.Pixels"));
            double prob_thresh = Config::Get<double>("Measurement.Probability.Thresh");
            bool prob_check = (measurement_prob > prob_thresh);

            g2o::ellipsoid* pLocalEllipsoidThisObservation = NULL;
            // 2 conditions must meet to start ellipsoid extraction:
            // C1 : the bounding box is not on border
            bool c1 = !is_border;

            // C2 : the groundplane has been estimated successfully
            bool c2 = true; // 对于点模型，一直为真.
            
            // in condition 3, it will not start
            // C3 : under with association mode, and the association is invalid, no need to extract ellipsoids again.
            bool c3 = false;
            if( withAssociation )
            {
                int instance = round(det_vec(7));
                if ( instance < 0 ) c3 = true;  // invalid instance
            }
            
            if( prob_check && c1 && c2 && !c3 ){   
                // 此处提取点云中心即可!
                g2o::ellipsoid e_extracted;
                bool result = EstimateLocalEllipsoidUsingPointModel(pFrame->frame_img, measurement, label, measurement_prob, pose, mCamera, e_extracted);

                if(result){
                    // 若提取成功
                    pLocalEllipsoidThisObservation = new g2o::ellipsoid(e_extracted);

                    // 可视化
                    // Visualize estimated ellipsoid
                    g2o::ellipsoid* pObj_world = new g2o::ellipsoid(e_extracted.transform_from(pFrame->cam_pose_Twc));
                    pObj_world->setColor(Vector3d(0.8,0.0,0.0), 1); // Set green color

                    // 第一次添加时清除上一次观测!
                    if(bEllipsoidNotClear)
                    {
                        mpMap->ClearEllipsoidsVisual(); // Clear the Visual Ellipsoids in the map
                        mpMap->ClearBoundingboxes();
                        bEllipsoidNotClear = false;
                    }
                    mpMap->addEllipsoidVisual(pObj_world); 
                }
            }

            // 若不成功保持为NULL
            pFrame->mpLocalObjects.push_back(pLocalEllipsoidThisObservation);
        }

        GenerateObservationStructure(pFrame);
        return;
    }

    // 重载掉该函数.
    bool GrabPoseAndObjects(double timestamp, const Eigen::VectorXd &pose, const Eigen::MatrixXd &bboxMap,
        const cv::Mat &imDepth, const cv::Mat &imRGB, bool withAssociation) {

        std::cout << "Run Baseline Tracker with Point Model..." << std::endl;
        clock_t time_start = clock();        
        
        AddNewFrame(timestamp, pose, bboxMap, imDepth, imRGB);
        clock_t time_AddNewFrame = clock();

        UpdateObjectObservationWithPointModel(mCurrFrame, withAssociation);   // Store object observation in a specific data structure.
        clock_t time_UpdateObjectObservation = clock();
        
        if(mbOpenOptimization){
            mpOptimizer->GlobalObjectGraphOptimizationWithPDAPointModel(mvpFrames, mpMap);
            
            // 该函数以 instance 为索引记录物体历史, 此时Optimization过程id不断变化无效
            RefreshObjectHistory();  

            // 输出所有帧, 所有局部椭球体的提取结果.
            SaveFrameAndDetectionResult(mvpFrames, true);   // true: save constrain planes.
        }
        clock_t time_NonparamOptimization = clock();

        // Visualization
        ProcessVisualization();
        clock_t time_Visualization = clock();

        // Memory Management
        // 释放掉两帧前的 cv::Mat , rgb/depth.
        ManageMemory();

        // Output running time
        cout << " - System Time: " << endl;
        cout << " -- time_AddNewFrame: " <<(double)(time_AddNewFrame - time_start) / CLOCKS_PER_SEC << "s" << endl;        
        cout << " -- time_UpdateObjectObservation: " <<(double)(time_UpdateObjectObservation - time_AddNewFrame) / CLOCKS_PER_SEC << "s" << endl;
        cout << " -- time_NonparamOptimization: " <<(double)(time_NonparamOptimization - time_UpdateObjectObservation) / CLOCKS_PER_SEC << "s" << endl;
        cout << " -- time_Visualization: " <<(double)(time_Visualization - time_NonparamOptimization) / CLOCKS_PER_SEC << "s" << endl;
        cout << " - [ total_frame: " <<(double)(time_Visualization - time_start) / CLOCKS_PER_SEC << "s ]" << endl;

        return true;
    }
};
}