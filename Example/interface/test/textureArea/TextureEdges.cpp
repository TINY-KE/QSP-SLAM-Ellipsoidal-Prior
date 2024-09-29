#include "TextureEdges.h"
#include "TextureOptimizer.h"

namespace g2o
{
// EdgeSE3EllipsoidTexture
bool EdgeSE3EllipsoidTexture::read(std::istream &is)
{
    return true;
};

bool EdgeSE3EllipsoidTexture::write(std::ostream &os) const
{
    return os.good();
};

void EdgeSE3EllipsoidTexture::computeError()
{
    const VertexSE3Expmap *SE3Vertex = static_cast<const VertexSE3Expmap *>(_vertices[0]);                      //  world to camera pose
    const VertexEllipsoidXYZABCYaw *eVertex = static_cast<const VertexEllipsoidXYZABCYaw *>(_vertices[1]); //  object pose to world

    SE3Quat cam_pose_Twc = SE3Vertex->estimate().inverse();
    ellipsoid global_ellipsoid = eVertex->estimate();

    ellipsoid local_e = global_ellipsoid.transform_from(cam_pose_Twc.inverse());

    // 开始构造误差!
    // CalculateEllipsoidTextureCost(local_e, keypo)
    std::vector<cv::KeyPoint> keyPointsFlected;
    double cost = CalculateEllipsoidTextureCost(local_e, mKeyPoints, mKeyPointsDiscri, mCalib, mGray, keyPointsFlected);
    
    _error[0] = cost;

    return;
}

void EdgeSE3EllipsoidTexture::setParam(const cv::Mat& imGray, const Vector4d& bbox, const std::vector<cv::KeyPoint>& keypoints, 
        const cv::Mat& descri, const Matrix3d& K)
{
    mGray = imGray;
    mBbox = bbox;
    mCalib = K;
    mKeyPoints = keypoints;
    mKeyPointsDiscri = descri;

    mbParamSet = true;
}

// EdgeSE3EllipsoidTextureDT
bool EdgeSE3EllipsoidTextureDT::read(std::istream &is)
{
    return true;
};

bool EdgeSE3EllipsoidTextureDT::write(std::ostream &os) const
{
    return os.good();
};

void EdgeSE3EllipsoidTextureDT::computeError()
{
    const VertexSE3Expmap *SE3Vertex = static_cast<const VertexSE3Expmap *>(_vertices[0]);                      //  world to camera pose
    const VertexEllipsoidXYZABCYaw *eVertex = static_cast<const VertexEllipsoidXYZABCYaw *>(_vertices[1]); //  object pose to world

    SE3Quat cam_pose_Twc = SE3Vertex->estimate().inverse();
    ellipsoid global_ellipsoid = eVertex->estimate();

    ellipsoid local_e = global_ellipsoid.transform_from(cam_pose_Twc.inverse());

    // 开始构造误差!
    // CalculateEllipsoidTextureCost(local_e, keypo)
    std::vector<cv::KeyPoint> keyPointsFlected;
    double cost = CalculateEllipsoidTextureCostEdges(local_e, mKeyPoints, mCalib, mDTMat, keyPointsFlected);
    
    _error[0] = cost;

    return;
}

void EdgeSE3EllipsoidTextureDT::setParam(const cv::Mat& dtMat, const Vector4d& bbox, const std::vector<cv::KeyPoint>& keypoints, 
        const Matrix3d& K)
{
    mDTMat = dtMat;
    mBbox = bbox;
    mCalib = K;
    mKeyPoints = keypoints;

    mbParamSet = true;
}


// 定义常用函数
vector<cv::Point2i> toPoint2i(const vector<cv::KeyPoint> &in)
{
    vector<cv::Point2i> out;
    out.reserve(in.size());
    for (auto &p : in)
    {
        cv::Point2i pout(p.pt.x, p.pt.y);
        out.push_back(pout);
    }
    return out;
};

// template: 输入一个keypoint模板，将其他数据拷贝过去
vector<cv::KeyPoint> toKeyPoints(const vector<cv::Point2i> &in, 
    const vector<cv::KeyPoint> &templateVec = vector<cv::KeyPoint>())
{
    vector<cv::KeyPoint> out;
    out.resize(in.size());
    int i = 0;
    for (auto &p : in)
    {
        cv::KeyPoint pout;
        if (templateVec.size() == in.size())
        {
            pout = templateVec[i];
        }
        pout.pt = cv::Point2f(p.x, p.y);
        out[i] = pout;
        i++;
    }

    // std::cout << " ** In.size : " << in.size() << std::endl;
    // std::cout << " ** out.size : " << out.size() << std::endl;

    return out;
};


cv::Mat selectDiscri(const cv::Mat &discri, 
    const vector<cv::KeyPoint> &keypoints_all, const vector<cv::KeyPoint> &keypoints_valid)
{

    // 先转类型
    vector<KeyPointSon> keypoints_all_son = KeyPointSon::Generate(keypoints_all);
    vector<KeyPointSon> keypoints_valid_son = KeyPointSon::Generate(keypoints_valid);

    std::vector<int> validId;
    for (int i = 0; i < keypoints_all_son.size(); i++)
    {

        if (std::find(keypoints_valid_son.begin(), keypoints_valid_son.end(), keypoints_all_son[i]) != keypoints_valid_son.end()) // 若该id是有效的（即在有效列表中）
        {
            // 插入该行到 Mat中
            validId.push_back(i);
        }
    }

    int valid_num = validId.size();
    cv::Mat out(valid_num, discri.cols, discri.type());
    for (int i = 0; i < valid_num; i++)
    {
        discri.row(validId[i]).copyTo(out.row(i));
    }

    return out;
}

double sumPixelDiff(const cv::Mat &descri1, const cv::Mat &descri2)
{
    // 1, 确保二者mat的rows or cols 是一致的
    // 2, 确保非0
    if ((descri1.size() != descri2.size()) || descri1.rows == 0)
    {
        std::cerr << "Descriptors have incorrect size : " << descri1.size() << ", " << descri2.size() << std::endl;
        return 0;
    }

    // 计算描述子差距
    // cv::Mat mat_diff = descri1 - descri2;
    // Eigen::MatrixXd eigen_mat;
    // cv::cv2eigen(mat_diff, eigen_mat);
    // eigen_mat = eigen_mat.cwiseAbs();
    // int diff_num = eigen_mat.sum();

    // NEW: opencv 内部汉明顿距离的对比
    double diff_num = cv::norm(descri1, descri2, cv::NORM_HAMMING);

    // DEBUG: 输出descri看看
    // ofstream fout("./log_descri1.txt");
    // fout << descri1 << std::endl;
    // fout.close();
    // ofstream fout2("./log_descri2.txt");
    // fout2 << descri2 << std::endl;
    // fout2.close();

    int pixel_num = descri1.rows;

    double average_diff = double(diff_num) / pixel_num;

    // DEBUG OUTPUT
    // std::cout << "diff_num in descri: " << diff_num << std::endl;
    // std::cout << "pixel_num : " << pixel_num << std::endl;
    // std::cout << "average_diff : " << average_diff << std::endl;

    return average_diff;
}

bool isInRange(int x, int y, int range_x, int range_x2, int range_y, int range_y2)
{
    if( range_x < x && x < range_x2 )
        if( range_y<y && y< range_y2 )
            return true;
    return false;
}

double CalculateEllipsoidTextureCostEdges(const g2o::ellipsoid& e_local, const vector<cv::KeyPoint>& keyPoints, 
        const Matrix3d &calib, const cv::Mat& dtmap, vector<cv::KeyPoint>& out_keypointsflected)
{
    EllipsoidTex eUpdate(e_local);
    vector<cv::Point2i> points_flected = eUpdate.generateFlectedPoints(toPoint2i(keyPoints), g2o::SE3Quat(), calib); // 必然在图像内部，但不一定 >0

    // 注意有部分点不位于椭圆上，导致没有对应点，它们的 x,y < 0
    
    // 接下来直接访问 Cost Map 即可输出!
    int valid_num = 0;
    double total_diff = 0;
    for(int i=0;i<points_flected.size();i++)
    {
        cv::Point2i& p_flected = points_flected[i];

        if(!isInRange(p_flected.x, p_flected.y, 0, dtmap.cols, 0, dtmap.rows)) continue;

        double value_ori = dtmap.at<float>(keyPoints[i].pt); // CV_32F
        double value_flected = dtmap.at<float>(p_flected);

        double diff = value_flected - value_ori;

        total_diff+=diff;
        valid_num++;
    }

    double average_diff = 0;
    if(valid_num > 0)
        average_diff = total_diff / valid_num;

    out_keypointsflected = toKeyPoints(points_flected);

    return average_diff;
}

// 封装一个计算Cost 的函数
double CalculateEllipsoidTextureCost(const g2o::ellipsoid& e, const vector<cv::KeyPoint>& keyPoints, const cv::Mat& keyPointsDiscri, 
        const Matrix3d &calib, const cv::Mat& gray, vector<cv::KeyPoint>& out_keypointsflected)
{
    static int run_time = 0;
    run_time++;

    clock_t time1_start = clock();
    EllipsoidTex eUpdate(e);

    clock_t time2_generateFlectedPoints = clock();
    // 生成keyPoints的三维对称点
    vector<cv::Point2i> points_flected = eUpdate.generateFlectedPoints(toPoint2i(keyPoints), g2o::SE3Quat(), calib); // 必然在图像内部，但不一定 >0

    clock_t time3_toKeyPoints = clock();
    // 计算点对的像素差, 遍历360度观察其非凸性
    auto keyPointsFlected = toKeyPoints(points_flected, keyPoints); // 剔除了无效点，所有都有效

    clock_t time4_computeOrientation = clock();
    // 更新所有点的Orientation
    TextureFeature::computeOrientation(gray, keyPointsFlected);

    clock_t time5_orb = clock();
    // 计算投影点的描述子
    cv::Mat keyPointsFlectedDiscri;
    auto keyPointsFlectedValid = keyPointsFlected;  // 经过compute后，边缘部分的投影点将被滤除
    cv::ORB orb;
    orb.compute(gray, keyPointsFlectedValid, keyPointsFlectedDiscri);

    clock_t time6_selectDiscri = clock();
    cv::Mat keyPointsDiscriSelected = selectDiscri(keyPointsDiscri, keyPointsFlected, keyPointsFlectedValid); // 仅取出在范围内的点 (即非-1,-1)

    clock_t time7_sumPixelDiff = clock();
    double pixelDiff_average = sumPixelDiff(keyPointsDiscriSelected, keyPointsFlectedDiscri); // 计算描述子差值
    
    clock_t time8_copyPoints = clock();
    out_keypointsflected = keyPointsFlected;

    clock_t time9_end = clock();

    // 统计时间
    double current_1 = (double)(time2_generateFlectedPoints - time1_start)/CLOCKS_PER_SEC;
    double current_2 = (double)(time3_toKeyPoints - time2_generateFlectedPoints)/CLOCKS_PER_SEC;
    double current_3 = (double)(time4_computeOrientation - time3_toKeyPoints)/CLOCKS_PER_SEC;
    double current_4 = (double)(time5_orb - time4_computeOrientation)/CLOCKS_PER_SEC;
    double current_5 = (double)(time6_selectDiscri - time5_orb)/CLOCKS_PER_SEC;
    double current_6 = (double)(time7_sumPixelDiff - time6_selectDiscri)/CLOCKS_PER_SEC;
    double current_7 = (double)(time8_copyPoints - time7_sumPixelDiff)/CLOCKS_PER_SEC;
    double current_8 = (double)(time9_end - time8_copyPoints)/CLOCKS_PER_SEC;

    static double time_total_1 = 0; time_total_1+=current_1;
    static double time_total_2 = 0; time_total_2+=current_2;
    static double time_total_3 = 0; time_total_3+=current_3;
    static double time_total_4 = 0; time_total_4+=current_4;
    static double time_total_5 = 0; time_total_5+=current_5;
    static double time_total_6 = 0; time_total_6+=current_6;
    static double time_total_7 = 0; time_total_7+=current_7;
    static double time_total_8 = 0; time_total_8+=current_8;

    if(run_time%100==0){
        std::cout << "time1_start " << time_total_1 << std::endl;
        std::cout << "time2_generateFlectedPoints " << time_total_2 << std::endl;
        std::cout << "time3_toKeyPoints " << time_total_3 << std::endl;
        std::cout << "time4_computeOrientation " << time_total_4 << std::endl;
        std::cout << "time5_orb " << time_total_5 << std::endl;
        std::cout << "time6_selectDiscri " << time_total_6 << std::endl;
        std::cout << "time7_sumPixelDiff " << time_total_7 << std::endl;
        std::cout << "time8_copyPoints " << time_total_8 << std::endl;
        std::cout << " **** Total Run Time : " << run_time << std::endl;
    }

    return pixelDiff_average;
}

} // namespace g2o