#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <boost/bind.hpp>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

class ImageGrabber
{
public:
    ImageGrabber()
    {
        // 初始化订阅器
        rgb_sub.subscribe(nh, "/rgb/image_raw", 1);
        depth_sub.subscribe(nh, "/depth_to_rgb/image_raw", 1);

        // 设置同步策略
        sync.reset(new message_filters::Synchronizer<sync_pol>(sync_pol(10), rgb_sub, depth_sub));
        sync->registerCallback(boost::bind(&ImageGrabber::GrabRGBD, this, _1, _2));
    }

    // 回调函数：接收 RGB 和深度图像并保存
    void GrabRGBD(const sensor_msgs::ImageConstPtr &rgb_msg,
                  const sensor_msgs::ImageConstPtr &depth_msg)
    {
        ROS_INFO("RGB image encoding: %s", rgb_msg->encoding.c_str());
        cv_bridge::CvImageConstPtr cv_ptrRGB;
        try
        {
            cv_ptrRGB = cv_bridge::toCvShare(rgb_msg);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        cv_bridge::CvImageConstPtr cv_ptrD;
        try
        {
            cv_ptrD = cv_bridge::toCvShare(depth_msg);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

            // 获取当前时间戳，作为文件名的一部分
            ros::Time timestamp = rgb_msg->header.stamp;
            std::string time_str = std::to_string(timestamp.sec) + "_" + std::to_string(timestamp.nsec);

            // 保存 RGB 图像
            std::string rgb_filename = "/home/robotlab/ws_3d_vp/src/QSP-SLAM-Ellipsoidal-Prior/dataset/ros_realtime/rgb/rgb_" + time_str + ".png";
            cv::imwrite(rgb_filename, cv_ptrRGB->image);

            // 保存深度图像
            std::string depth_filename = "/home/robotlab/ws_3d_vp/src/QSP-SLAM-Ellipsoidal-Prior/dataset/ros_realtime/depth/depth_" + time_str + ".png";
            cv::imwrite(depth_filename, cv_ptrD->image);

            ROS_INFO("Saved RGB and Depth images at time %s", time_str.c_str());

    }

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
};

int main(int argc, char **argv)
{
    // 初始化 ROS 节点
    ros::init(argc, argv, "image_grabber_node");

    // 创建 ImageGrabber 对象
    ImageGrabber igb;

    // 运行 ROS 事件循环
    ros::spin();

    return 0;
}