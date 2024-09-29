#include "TextureFeature.h"

using namespace cv;
using namespace std;

// image: Gray mat
void TextureFeature::computeOrientation(const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints, int patchSize)
{
    int halfPathSize = patchSize/2;
    std::vector<int> umax = generateUmax(halfPathSize);
    computeOrientation(image, keypoints, halfPathSize, umax);
}

/** Compute the ORB keypoint orientations
 * @param image the image to compute the features and descriptors on
 * @param integral_image the integral image of the iamge (can be empty, but the computation will be slower)
 * @param scale the scale at which we compute the orientation
 * @param keypoints the resulting keypoints
 */
void TextureFeature::computeOrientation(const Mat& image, vector<KeyPoint>& keypoints,
                               int halfPatchSize, const vector<int>& umax)
{
    // Process each keypoint
    for (vector<KeyPoint>::iterator keypoint = keypoints.begin(),
         keypointEnd = keypoints.end(); keypoint != keypointEnd; ++keypoint)
    {
        keypoint->angle = IC_Angle(image, halfPatchSize, keypoint->pt, umax);
    }
}

float TextureFeature::IC_Angle(const Mat& image, const int half_k, Point2f pt,
                      const vector<int> & u_max)
{
    int m_01 = 0, m_10 = 0;

    const uchar* center = &image.at<uchar> (cvRound(pt.y), cvRound(pt.x));

    // Treat the center line differently, v=0
    for (int u = -half_k; u <= half_k; ++u)
        m_10 += u * center[u];

    // Go line by line in the circular patch
    int step = (int)image.step1();
    for (int v = 1; v <= half_k; ++v)
    {
        // Proceed over the two lines
        int v_sum = 0;
        int d = u_max[v];
        for (int u = -d; u <= d; ++u)
        {
            int val_plus = center[u + v*step], val_minus = center[u - v*step];
            v_sum += (val_plus - val_minus);
            m_10 += u * (val_plus + val_minus);
        }
        m_01 += v * v_sum;
    }

    return fastAtan2((float)m_01, (float)m_10);
}

std::vector<int> TextureFeature::generateUmax(int halfPatchSize)
{
    vector<int> umax(halfPatchSize + 2);

    int v, v0, vmax = cvFloor(halfPatchSize * sqrt(2.f) / 2 + 1);
    int vmin = cvCeil(halfPatchSize * sqrt(2.f) / 2);
    for (v = 0; v <= vmax; ++v)
        umax[v] = cvRound(sqrt((double)halfPatchSize * halfPatchSize - v * v));

    // Make sure we are symmetric
    for (v = halfPatchSize, v0 = 0; v >= vmin; --v)
    {
        while (umax[v0] == umax[v0 + 1])
            ++v0;
        umax[v] = v0;
        ++v0;
    }

    return umax;
}