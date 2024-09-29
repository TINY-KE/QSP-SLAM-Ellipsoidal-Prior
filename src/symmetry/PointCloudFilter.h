// process point cloud: segmentation, downsample, filter...

#ifndef ELLIPSOIDSLAM_POINTCLOUDFILTER_H
#define ELLIPSOIDSLAM_POINTCLOUDFILTER_H

// pcl
#include <pcl/common/transforms.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudPCL;

#include <core/Ellipsoid.h>
#include <core/Geometry.h>

#include <opencv2/opencv.hpp>

#include <iostream>
#include <string>

#include <ctime>

Vector2d getXYCenterOfPointCloud(ORB_SLAM2::PointCloud *pPoints);
ORB_SLAM2::PointCloud getPointCloudInRect(cv::Mat &depth, cv::Mat &rgb, const VectorXd &detect, ORB_SLAM2::camera_intrinsic &camera, double range = 100);
ORB_SLAM2::PointCloud getPointCloudInRect(cv::Mat &depth, const VectorXd &detect, ORB_SLAM2::camera_intrinsic &camera, double range = 100);
void filterGround(ORB_SLAM2::PointCloud **ppCloud);
void outputCloud(ORB_SLAM2::PointCloud *pCloud, int num = 10);
ORB_SLAM2::PointCloud pclToQuadricPointCloud(PointCloudPCL &cloud);
ORB_SLAM2::PointCloud pclToQuadricPointCloud(PointCloudPCL::Ptr &pCloud);
PointCloudPCL::Ptr QuadricPointCloudToPcl(ORB_SLAM2::PointCloud &cloud);
pcl::PointCloud<pcl::PointXYZ>::Ptr QuadricPointCloudToPclXYZ(ORB_SLAM2::PointCloud &cloud);

ORB_SLAM2::PointCloud *pclToQuadricPointCloudPtr(PointCloudPCL::Ptr &pCloud);

ORB_SLAM2::PointCloud pclXYZToQuadricPointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr &pCloud);
ORB_SLAM2::PointCloud *pclXYZToQuadricPointCloudPtr(pcl::PointCloud<pcl::PointXYZ>::Ptr &pCloud);

// downsample and outlier filter
void DownSamplePointCloud(ORB_SLAM2::PointCloud &cloudIn, ORB_SLAM2::PointCloud &cloudOut, int param_num = 100);

// only downsample
void DownSamplePointCloudOnly(ORB_SLAM2::PointCloud &cloudIn, ORB_SLAM2::PointCloud &cloudOut, double grid = 0.02);

// filter outliers
void FiltOutliers(ORB_SLAM2::PointCloud &cloudIn, ORB_SLAM2::PointCloud &cloudOut, int num_neighbor = 100);

// filter points and keep those in the ellipsoid
void FiltPointsInBox(ORB_SLAM2::PointCloud *pPoints_global, ORB_SLAM2::PointCloud *pPoints_global_inBox, g2o::ellipsoid &e);

void CombinePointCloud(ORB_SLAM2::PointCloud *p1, ORB_SLAM2::PointCloud *p2);

#endif // POINTCLOUDFILTER