// 该文件测试速度理论上更快的优化方式。 DT变换以及二值化


#include <opencv2/opencv.hpp>
#include <iostream>
using namespace std;
using namespace cv;

Mat src, src_gray;
Mat dst, detected_edges;

int edgeThresh = 1;
int lowThreshold;
int const max_lowThreshold = 100;
int ratio = 3;
int kernel_size = 3;
char* window_name = "dst";


void CannyThreshold(int, void*)
{
    clock_t time_start = clock();

    /// 使用 3x3内核降噪
    blur( src_gray, detected_edges, Size(3,3) );

    /// 运行Canny算子
    Canny( detected_edges, detected_edges, lowThreshold, lowThreshold*ratio, kernel_size );

    /// 使用 Canny算子输出边缘作为掩码显示原图像
    // dst = Scalar::all(0);

    // src.copyTo( dst, detected_edges);
    imshow( window_name, detected_edges );
    // imshow( window_name, dst ); // dst 是带颜色的

    cv::bitwise_not(detected_edges, detected_edges);

    Mat dtmap;
    distanceTransform(detected_edges, dtmap, CV_DIST_L1, 3);

    clock_t time_end = clock();
    std::cout << "Time : " << double(time_end-time_start)/CLOCKS_PER_SEC << std::endl;

    imshow("before normal", dtmap);

    // Normalize the distance image for range = {0.0, 1.0}
    // so we can visualize and threshold it
    normalize(dtmap, dtmap, 0, 1., NORM_MINMAX);
    imshow("Distance Transform Image", dtmap);

}

int main(int, char** argv)
{
    // Load the image
    src = imread(argv[1]);
    // Check if everything was fine
    if (!src.data)
        return -1;
    // Show source image
    imshow("Source Image", src);

    /// 创建与src同类型和大小的矩阵(dst)
    dst.create( src.size(), src.type() );

    /// 原图像转换为灰度图像
    cvtColor( src, src_gray, CV_BGR2GRAY );

    /// 创建显示窗口
    namedWindow( window_name, CV_WINDOW_AUTOSIZE );

    /// 创建trackbar
    createTrackbar( "Min Threshold:", window_name, &lowThreshold, max_lowThreshold, CannyThreshold );

    /// 显示图像
    CannyThreshold(0, 0);

    /// 等待用户反应
    waitKey(0);

    return 0;
}