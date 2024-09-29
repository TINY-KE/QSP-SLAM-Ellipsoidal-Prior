#ifndef ELLIPSOIDSLAM_PRIORINFER_H
#define ELLIPSOIDSLAM_PRIORINFER_H

// 该文件基于先验表对椭球体做优化
#include "include/core/Ellipsoid.h"
#include "include/core/BasicEllipsoidEdges.h"

#include "Thirdparty/g2o/g2o/core/base_multi_edge.h"
#include "Thirdparty/g2o/g2o/types/types_six_dof_expmap.h"

#include "Eigen/Core"

using namespace Eigen;

namespace ORB_SLAM2
{
    class Pri
    {
    public:
        Pri(double a, double b, double c); // a:b:c
        Pri(double a, double b);           // 1:a:b
        Pri(const g2o::ellipsoid& e);   // 直接从椭球体读取 pri 
        Pri(){d=1;e=1;}

        Vector2d operator-(const Pri& p)
        {
            return Vector2d(this->d-p.d, this->e-p.e);
        }

        void print();
        double GetD() const;
        double GetE() const;

    private:
        double d; // 1 : d : e   small -> big
        double e;
    };

    class priorInfer
    {

    public:
        priorInfer(int rows, int cols, const Matrix3d& calib);

        g2o::ellipsoid infer(g2o::ellipsoid &e, const Pri &pri, double weight, g2o::plane& plane_ground);

        g2o::ellipsoid MonocularInfer(g2o::ellipsoid &e, const Pri &pri, double weight, g2o::plane& plane_ground);
        g2o::ellipsoid GenerateInitGuess(Vector4d& bbox, const Vector4d& plane_local);

        double GetLastCost();
        g2o::ellipsoid MonocularInferExpand(g2o::ellipsoid &e, const Pri &pri, double weight, g2o::plane& plane_ground);
        std::vector<g2o::ellipsoid> GetAllPossibleEllipsoids();

    private:

        // ground_plane_weight: 仅仅在单目情况下激活，默认将第一个planes添加权重。
        g2o::ellipsoid optimizeEllipsoidWithPlanesAndPrior(const g2o::ellipsoid &init_guess, std::vector<g2o::plane> &planes,
                                                       std::vector<g2o::plane> &planesWithNormal, const Pri &pri, double weight, double ground_plane_weight = -1);


        int mCols, mRows;
        Matrix3d mCalib;

        double mdCost;
        std::vector<g2o::ellipsoid> mvePossibleEllipsoids;
    };

    // 2 degrees
    class EdgePri : public g2o::BaseUnaryEdge<2, Pri, g2o::VertexEllipsoidXYZABCYaw>
    {
    public:
        EdgePri();
        virtual bool read(std::istream &is);
        virtual bool write(std::ostream &os) const;
        void computeError();
    };

    // Pri 工厂： 负责管理label生成满足条件的pri
    class PriFactor
    {
    public:
        bool LoadPriConfigurations(const std::string& str_path);    // 从txt文件中读取Pri的配置
        Pri CreatePri(int label);
    private:
        const std::string PREFIX_D = "PARAM_SEMANTICPRIOR_D_";
        const std::string PREFIX_E = "PARAM_SEMANTICPRIOR_E_";
    };

} // namespace EllipsoidSLAM

#endif
