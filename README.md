# 恢复单目,但物体检测结果待筛选 commit d4ec5517bcfad64dd372450c6f1653790c337862
    + 编译qsp_slam_mono
    + 补充freiburg_001.yaml和config_freiburg_001.json
    + 将get_frame_by_name 改回 get_frame_by_id

# 显示原点大球 commit 962c1b35539d0e164a0c9efd979f18741a8ca28e
    + 目前用的汽车检测结果，用的是左上角第一个，改为 同类型的正中心的。修改yaml和json中的图片宽度和高度。
    + 修改yaml中的物体yoloclass和DecoderPaths
    + sdf模型创建失败： 通过在localmapping中根据.attr("reconstruct_object")寻找，可知是数据关联失败，当前使用的是李建的拖球体数据关联，改回dsp的数据关联。修改方法——修改yaml中的参数：
        System.UseEllipsoldPoseForDSP.Open: 0
        CreateSingleObject: 1
        Tracking.AssociateObjectWithEllipsold: 0
    + 添加setrealpose和[改进] [位姿真值] ， 将相机和point转移到真实世界坐标系下。
    + 修改InferObjectsWithSemanticPrior， 在pangolin中显示原点大球

# sa
    + 添加地平面 mpTracker->SetGroundPlaneMannually(Vector4d(0,  0,   1,  0));
    + 修改InferObjectsWithSemanticPrior， 生成地面上的椭球
    + 

    + 待： 汽车目前是底朝天，原因应该是在聚类时，修改了算法。

# Ellipsoids 和 Ellipsoids Objects 的区别
    + GetAllEllipsoids() GetAllEllipsoidsVisual()  <--->  GetAllEllipsoidsObjects()
    + mspEllipsoids <--->  mspEllipsoidsObjects
    + addEllipsoid(  addEllipsoidVisual( <--->  addEllipsoidObjects(ellipsoid *pObj)
    + 

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
