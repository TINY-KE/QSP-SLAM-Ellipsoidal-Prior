
#ifndef ImageGrabber_H
#define ImageGrabber_H

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <boost/bind.hpp>


// #include "Plane3D.h"
namespace ORB_SLAM2
{


class ImageGrabber
{

public:
    ImageGrabber(std::string path_);

    // 回调函数：接收 RGB 和深度图像并保存
    void GrabRGBD(const sensor_msgs::ImageConstPtr &rgb_msg,
                  const sensor_msgs::ImageConstPtr &depth_msg);

    std::mutex mMutexImageSave;   //    unique_lock<mutex> lock(mpImageGrabber->mMutexImageSave);

private:
    // ROS 节点句柄
    ros::NodeHandle nh;

    // 消息过滤器订阅器
    message_filters::Subscriber<sensor_msgs::Image> rgb_sub;
    message_filters::Subscriber<sensor_msgs::Image> depth_sub;

    // 同步策略
    typedef message_filters::sync_policies::ApproximateTime<
        sensor_msgs::Image, sensor_msgs::Image>
        sync_pol;
    std::shared_ptr<message_filters::Synchronizer<sync_pol>> sync;

    //
    std::string mPath;
};

} //namespace ORB_SLAM

#endif // MAPPUBLISHER_H
