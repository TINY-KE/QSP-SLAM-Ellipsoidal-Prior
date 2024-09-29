#ifndef ELLIPSOIDSLAM_TEXTUREFEATURE_H
#define ELLIPSOIDSLAM_TEXTUREFEATURE_H

#include <vector>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>

class TextureFeature
{

public:

static void computeOrientation(const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints, int patchSize = 31);

private:

static void computeOrientation(const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints,
                               int halfPatchSize, const std::vector<int>& umax);

static float IC_Angle(const cv::Mat& image, const int half_k, cv::Point2f pt,
                      const std::vector<int> & u_max);

static std::vector<int> generateUmax(int halfPatchSize);
};

#endif