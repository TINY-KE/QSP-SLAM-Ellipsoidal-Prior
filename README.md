# 编译回忆
    + eigen用的是ubuntu20自带的 337版本
    + DBow2  
        cmake -DOpenCV_DIR="/home/robotlab/thirdparty/for_dspslam/opencv/build" ..
    + 

# 恢复单目,但物体检测结果待筛选 commit d4ec5517bcfad64dd372450c6f1653790c337862
    + 编译qsp_slam_mono
    + 补充freiburg_001.yaml和config_freiburg_001.json
    + 将get_frame_by_name 改回 get_frame_by_id

# 显示原点大球 commit 5c090653542062dd0c397afa6db99ab93de8ab66
    + 目前用的汽车检测结果，用的是左上角第一个，改为 同类型的正中心的。修改yaml和json中的图片宽度和高度。
    + 修改yaml中的物体yoloclass和DecoderPaths
    + sdf模型创建失败： 通过在localmapping中根据.attr("reconstruct_object")寻找，可知是数据关联失败，当前使用的是李建的拖球体数据关联，改回dsp的数据关联。修改方法——修改yaml中的参数：
        System.UseEllipsoldPoseForDSP.Open: 0
        CreateSingleObject: 1
        Tracking.AssociateObjectWithEllipsold: 0
    + 添加setrealpose和[改进] [位姿真值] ， 将相机和point转移到真实世界坐标系下。
    + 修改InferObjectsWithSemanticPrior， 在pangolin中显示原点大球

# 实现了MonocularInfer，但是在车头观测时不够长 commit a90770336d0c0051818f5265f85c6824f936d1a4
    + 添加地平面 mpTracker->SetGroundPlaneMannually(Vector4d(0,  0,   1,  0));
    + 修改InferObjectsWithSemanticPrior， 生成地面上的椭球
    + GenerateInitGuess用的是世界坐标系下的地面

# 实现了单帧的椭球体粗略估计，可用于物体预测。 commit 4e60e11ccc56c74a8a3dc8215ad056237c1c1b6a 
    + 编写MonocularInferWithNearFarPlane，
    + 将Tracking::GenerateObservationStructure(ORB_SLAM2::Frame* pFrame)， 统一到SetObservations(pKF); 并将获取的物体内3D点，存到measure中
    + 提取远近平面，并用于椭球体粗略估计
    + 通过useFar_vertical, useNear_vertical 等选项，控制远近平面的生成方法


# 提高点云聚类的阈值，从而使得dsp效果变差    commit 6bb976c8ec1537d8ab63be61adf86c160fad7590
    + 恢复单目中原本的物体reconstruct函数 ProcessDetectedObjects_origin()
    + 提高聚类的严格标准1：  修改ComputeCuboidPCA.scale 为0.8
    + 提高聚类的严格标准2：  RemoveOutliersSimple();
    + 
    + reconstruct_object中用到的 Sim3Two 怎么计算的
        auto Sim3Two_pMO = pMO->Sim3Two;
        理解：
        Eigen::Matrix4f Sim3Two = SE3Twc * Sim3Tco;

    + PCA 方法没变？
        ComputeCuboidPCA(numKFsPassedSinceInit < 15);
    + 提高聚类阈值1
    std::cout << "RemoveOutliersModel" << std::endl;
            pMO->RemoveOutliersModel();
    + 提高阈值2
        RemoveOutliersSimple();
    + 待： 汽车目前是底朝天，原因应该是在聚类时，修改了算法。


    + 待： 往一个拖球体中不断添加平面。
    通过GenerateInitGuess(bbox, ground_pl_local.param);


# 将点云和相机rviz显示   commit caf390440bf989784b882ab95838f8bab63d08fe
+ 通过向map中的椭球体，添加多帧平面，从而联合优化椭球体。
+ 引入MapPublisher.cpp
+ 解决多线程无法中断的问题

# dsp物体通过rviz显示
+  通过Marching Cubes 算法生成SDF的网格顶点矩阵， 转换为点和面片
    + vertices, faces = convert_sdf_voxels_to_mesh(sdf_tensor.view(self.voxels_dim, self.voxels_dim, self.voxels_dim))
    + pMO->vertices = pyMesh.attr("vertices").cast<Eigen::MatrixXf>();
    + pMO->faces = pyMesh.attr("faces").cast<Eigen::MatrixXi>();
+ rviz显示dsp物体

# 迁移到RGBD模式
+ 使用self数据集
+ 在初始的时候，添加相机真值
+ 更改物体关联的方式?
  + AssociateObjects(pKF) 和 AssociateObjectsByProjection(pKF) 的区别
  + 结论：暂时先用point椭球体关联
+ 关联过程中，同时存储infer椭球体和point椭球体，并对比两者，当infer椭球体中点云相对充盈的时候，再用infer椭球体以生成最终物体
  + UpdateObjectObservation_GenerateEllipsoid在图像帧中存储的椭球体，是如何用在数据关联AssociateObjectsByProjection中使用的？
    + pFrame->mpLocalObjects
    + pKF->mpLocalEllipsolds
    + pKF->mpGlobalEllipsolds
    + 图像帧中存储的椭圆在AssociateObjectsByProjection没用到，使用的是地图中的物体对应的椭球，
    + 因此，图像帧中存储的椭圆应该是在localmap中使用
+ 关联在一起的椭球体是怎么融合的？？
+ 待：在深度模式中添加infer
+ 

+ 待：dsp物体生成时，旋转椭球体，与地面进行对比
+ 待：修改，提升slam速度。System.LocalMappingInSameThread: 1


判断椅子和沙发，在预估椭球体之后，四个侧面。
旋转轴z轴旋转。
可能的位置：





# 利用地平面和多帧bbox平面，生成椭球体
    + 将单帧bbox平面从相机坐标系转移到世界坐标系，并通过可视化，验证了准确性
    + 将平面从世界坐标系转移到当前帧的相机坐标系
    + 编写optimizeEllipsoidWithMultiPlanes，实现利用地平面和多帧bbox平面，生成椭球体

    + 问题： 物体问题很多，是不是可以通过PlaneNormal的方式 解决这个问题
    
    + 今晚（1）检查bbox生成的平面有没有normal
            （2）给平面添加 角度限制，可以先通过50帧添加一次bbox平面，试试。

    + 
        /*
        *   [新版本] 基于切平面完成椭球体的全局优化!
        *   10-4 Ours 论文发表使用的版本。
        */
        void Optimizer::OptimizeWithDataAssociationUsingMultiplanes(std::vector<Frame *> &pFrames,
    

# 平面法向量的计算
    Vector3d normal = (p1-p0).cross(p2-p1);



# 平面的提取与转换
    //相机坐标系下的地面
    pPlaneExtractor->extractGroundPlane(depth, groundPlane);

    // 设置世界地平面
    groundPlane.transform(Twc);   // transform to the world coordinate.
    mGroundPlane = groundPlane;
    mGroundPlane.color = Eigen::Vector3d(0.0,0.8,0.0); 
    mGroundPlane.InitFinitePlane(Twc.translation(), 10);
    mpMap->addPlane(&mGroundPlane);

# 椭球体生成中 平面的转换
    g2o::plane ground_pl_local = mGroundPlane;   //世界坐标系下的
    ground_pl_local.transform(pFrame->cam_pose_Tcw);   //相机坐标系下的

# bbox转mvCPlanes
    + GenerateConstrainPlanesOfBbox(bbox, mCalib, mRows, mCols)
    + 

# Ellipsoids 和 Ellipsoids Objects 的区别
    + GetAllEllipsoids() GetAllEllipsoidsVisual()  <--->  GetAllEllipsoidsObjects()
    + mspEllipsoids <--->  mspEllipsoidsObjects
    + addEllipsoid(  addEllipsoidVisual( <--->  addEllipsoidObjects(ellipsoid *pObj)
    + 

# plane::transform中，为什么matTwc_trans_inv 和 matTwc 不相同
    matTwc :    -0.043747   0.231869  -0.971763  0.0254108
                0.998925  0.0250846 -0.0389844  0.0779903
                0.0153371  -0.972424  -0.232717   0.398303
                        0          0          0          1
    matTw_trans :  -0.043747   0.998925  0.0153371          0
                    0.231869  0.0250846  -0.972424          0
                    -0.971763 -0.0389844  -0.232717          0
                    0.0254108  0.0779903   0.398303          1
    matTwc_trans_inv :  -0.043747   0.231869  -0.971763         -0
                        0.998925  0.0250846 -0.0389844         -0
                        0.0153371  -0.972424  -0.232717         -0
                        -0.0829036   0.379471   0.120425          1


为什答案会是这样

# 整合1
    + 取消地面估计
    + 取消对称性
    + 设置相机初始位置

# 待针对Fre数据集的分类
    + py::list detections = mpSystem->pySequence.attr("get_frame_by_id")(pKF->mnFrameId);
    + 


# WARNNING 1 
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
