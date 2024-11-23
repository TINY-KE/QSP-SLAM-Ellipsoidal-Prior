/**
* This file is part of ORB-SLAM.
*
* Copyright (C) 2014 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <http://webdiis.unizar.es/~raulmur/orbslam/>
*
* ORB-SLAM is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM. If not, see <http://www.gnu.org/licenses/>.
*/

#include "ImageGrabber.h"

namespace ORB_SLAM2 {
    ImageGrabber::ImageGrabber(std::string path_): mPath(path_) {
        // 初始化订阅器
        rgb_sub.subscribe(nh, "/rgb/image_raw", 1);
        depth_sub.subscribe(nh, "/depth_to_rgb/image_raw", 1);

        // 设置同步策略
        sync.reset(new message_filters::Synchronizer<sync_pol>(sync_pol(10), rgb_sub, depth_sub));
        sync->registerCallback(boost::bind(&ImageGrabber::GrabRGBD, this, _1, _2));
    }

    // 回调函数：接收 RGB 和深度图像并保存
    void ImageGrabber::GrabRGBD(const sensor_msgs::ImageConstPtr &rgb_msg,
                                const sensor_msgs::ImageConstPtr &depth_msg) {
        std::unique_lock<std::mutex> lock(mMutexImageSave);
        cv_bridge::CvImageConstPtr cv_ptrRGB;
        try {
            cv_ptrRGB = cv_bridge::toCvShare(rgb_msg);
        } catch (cv_bridge::Exception &e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        cv_bridge::CvImageConstPtr cv_ptrD;
        try {
            cv_ptrD = cv_bridge::toCvShare(depth_msg);
        } catch (cv_bridge::Exception &e) {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        // 获取当前时间戳，作为文件名的一部分
        ros::Time timestamp = rgb_msg->header.stamp;
        std::string time_str = std::to_string(timestamp.sec) + "_" + std::to_string(timestamp.nsec);

        // 保存 RGB 图像
        //        std::string rgb_filename = "/home/robotlab/ws_3d_vp/src/QSP-SLAM-Ellipsoidal-Prior/dataset/ros_realtime/rgb/rgb_" + time_str + ".png";
        std::string rgb_filename = mPath + "/rgb/000000.png";
        cv::imwrite(rgb_filename, cv_ptrRGB->image);

        // 保存深度图像
        //        std::string depth_filename = "/home/robotlab/ws_3d_vp/src/QSP-SLAM-Ellipsoidal-Prior/dataset/ros_realtime/depth/depth_" + time_str + ".png";
        std::string depth_filename = mPath + "/depth/000000.png";
        cv::imwrite(depth_filename, cv_ptrD->image);

        // ROS_INFO("Saved RGB and Depth images at time %s", time_str.c_str());
    }
} //namespace ORB_SLAM
