/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/


#include<iostream>
#include<algorithm>
#include<fstream>
#include<chrono>

#include<ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <opencv2/core/core.hpp>
#include <geometry_msgs/PoseStamped.h>
#include <std_msgs/Float64.h>
#include <tf/tf.h> 
#include <tf/transform_datatypes.h> 
#include "Converter.h"
#include "System.h"


using namespace std;

std::vector<int> yolo_class;

int loop = 0;
//NBV MAM end

class ImageGrabber
{
public:
    ImageGrabber(ORB_SLAM2::System* pSLAM):mpSLAM(pSLAM){}
    
    void GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD);

    ORB_SLAM2::System* mpSLAM;

};

//darknet_ros_msgs::BoundingBox shrink(const darknet_ros_msgs::BoundingBox& box1, const darknet_ros_msgs::BoundingBox& box2) {
//    int xmin1 = box1.xmin, ymin1 = box1.ymin, xmax1 = box1.xmax, ymax1 = box1.ymax;
//    int xmin2 = box2.xmin, ymin2 = box2.ymin, xmax2 = box2.xmax, ymax2 = box2.ymax;
//    if()
//}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "RGBD");
    ros::start();
    string yamlfile, sensor; bool semanticOnline, rosBagFlag;
    //(1)从ros param中获取参数
    if(argc != 6)
    {
        cerr << endl << "Usage: ./qsp_slam_rgbd path_to_vocabulary path_to_settings path_to_sequence path_to_association path_to_saved_trajectory" << endl;
        return 1;
    }

    vector<string> vstrImageFilenamesRGB;
    vector<string> vstrImageFilenamesD;
    vector<double> vTimestamps;
    string strAssociationFilename = string(argv[4]);

    cv::FileStorage fSettings(string(argv[2]), cv::FileStorage::READ);

    if(fSettings["CoutToFile"].isNamed()) {
        ofstream fout("logs/log.txt"); //文件输出流对象
        streambuf* pOld =cout.rdbuf(fout.rdbuf());
    }

    auto msensor = ORB_SLAM2::System::MONOCULAR;

    if(fSettings["System.mode"].isNamed()) {
        string system_mode = fSettings["System.mode"];
        if (system_mode=="RGBD") {
            msensor = ORB_SLAM2::System::RGBD;
        }
    }

    ORB_SLAM2::System SLAM(argv[1], argv[2], argv[3], msensor);

    string strSettingsFile = argv[2];


    //(3)接受ros topic
    ImageGrabber igb(&SLAM);
    ros::NodeHandle nh;
    message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, "/rgb/image_raw", 1);
    message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "/depth_to_rgb/image_raw", 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image, darknet_ros_msgs::BoundingBoxes> sync_pol;
    message_filters::Synchronizer<sync_pol> sync(sync_pol(10), rgb_sub,depth_sub,bbox_sub);
    sync.registerCallback(boost::bind(&ImageGrabber::GrabRGBD,&igb,_1,_2,_3));

    ros::spin();

    // Stop all threads
    SLAM.Shutdown();

    // (4)Save camera trajectory
    SLAM.SaveKeyFrameTrajectoryTUM("/home/zhjd/active_eao/src/active_eao/eval/temp/KeyFrameTrajectory.txt");
    int SaveLocalObjects = fSettings["Viewer.savelocalobject"];
    if(SaveLocalObjects){
        SLAM.SaveObjects(   "/home/zhjd/active_eao/src/active_eao/eval/temp/Objects.txt",
                            "/home/zhjd/active_eao/src/active_eao/eval/temp/Objects_with_points.txt");
        SLAM.SaveGlobalNBVPose("/home/zhjd/active_eao/src/active_eao/eval/temp/GlobalNBV.txt");
    }

    ros::shutdown();

    return 0;
}

void ImageGrabber::GrabRGBD(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD,const darknet_ros_msgs::BoundingBoxesConstPtr& msgBbox)
{
    double current_time = msgRGB->header.stamp.toSec();
    //std::cout << std::endl << std::endl;
    //std::cout << "[Get a Frame with bbox] Timestamp:" << current_time << std::endl;

    // Copy the ros image message to cv::Mat.
    cv_bridge::CvImageConstPtr cv_ptrRGB;
    try
    {
        cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    cv_bridge::CvImageConstPtr cv_ptrD;
    try
    {
        cv_ptrD = cv_bridge::toCvShare(msgD);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    std::vector<darknet_ros_msgs::BoundingBox> boxes = msgBbox->bounding_boxes;
    std::vector<BoxSE> BboxVector = darknetRosMsgToBoxSE(boxes);


    cv::Mat Tcw;
    Tcw = mpSLAM->TrackRGBD(cv_ptrRGB->image,  cv_ptrD->image,  cv_ptrRGB->header.stamp.toSec(),  BboxVector);


}

vector<BoxSE> ImageGrabber::darknetRosMsgToBoxSE(vector<darknet_ros_msgs::BoundingBox>& boxes){
        //修改
        if (boxes.size() == 0)
        {
            std::cout << "[WARNNING] YOLO OBJECTS SIZE IS ZERO" << std::endl;
        }
        /* darknet_ros_msgs::BoundingBox
         * float64 probability
         * int64 xmin
         * int64 ymin
         * int64 xmax
         * int64 ymax
         * int16 id
         * string Class
         * */
        std::vector<BoxSE> boxes_online;
        for (auto &objInfo : boxes)
        {
            if (objInfo.probability < 0.5)
                continue;
            // TODO: 检测和yolo3ros 相同.
            // 0: person; 24: handbag?24应该是backpack背包,26是handbag手提包; 28: suitcase; 39: bottle; 56: chair;
            // 57: couch; 58:potted plant; 59: bed; 60: dining table; 62: tv;
            // 63: laptop; 66: keyboard; 67: phone; 73: book;
            std::vector<int>::iterator iter = std::find( yolo_class.begin(),  yolo_class.end(), objInfo.id);
            if(iter == yolo_class.end())
                continue;
            BoxSE box;
            box.m_class = objInfo.id;
            box.m_score = objInfo.probability;
            box.x = objInfo.xmin;
            box.y = objInfo.ymin;
            box.width = (objInfo.xmax - objInfo.xmin);
            box.height = (objInfo.ymax - objInfo.ymin );
            // box.m_class_name = "";
            boxes_online.push_back(box);
        }
        //安装识别的概率，进行排序
        std::sort(boxes_online.begin(), boxes_online.end(), [](BoxSE a, BoxSE b) -> bool { return a.m_score > b.m_score; });
        return boxes_online;
}

//float computeIntersectionOverUnion(const darknet_ros_msgs::BoundingBox& box1, const darknet_ros_msgs::BoundingBox& box2) {
//    float interArea = std::max(0.f, std::min(box1.x2, box2.x2) - std::max(box1.x1, box2.x1)) *
//                      std::max(0.f, std::min(box1.y2, box2.y2) - std::max(box1.y1, box2.y1));
//    float box1Area = (box1.x2 - box1.x1) * (box1.y2 - box1.y1);
//    float box2Area = (box2.x2 - box2.x1) * (box2.y2 - box2.y1);
//    float unionArea = box1Area + box2Area - interArea;
//    return interArea / unionArea;
//}
