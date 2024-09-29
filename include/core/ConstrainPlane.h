// 约束平面由bbox和Depth矩形框产生, 属于 plane 的装饰对象
#ifndef ELLIPSOIDSLAM_CONSTRAINPLANE_H
#define ELLIPSOIDSLAM_CONSTRAINPLANE_H

#include <Eigen/Core>

namespace g2o {
class plane;
class ConstrainPlane {
  public:
    ConstrainPlane(plane *ppl);
    // 特有属性
    bool valid;              // 最终flag : 若属于边界, 则无效; 不再参与关联与优化
    bool image_border;       // 是否位于图像边缘
    bool association_border; // 是否在关联时被判断在边缘
    int state;               // invalid 时, 两种state: 1, 内部  2, 外部

    int type; // 0 : 来自bbox  ,  1: 来自 cuboids

    // 任务:
    // 构造函数
    // 拷贝构造函数
    // 保持上述三个参数能够继续传递

    // 更新.->不参与优化 不需要.
    plane *pPlane;

    // load and save
    Eigen::VectorXd toVector();
    void fromVector(Eigen::VectorXd &vec);
    static int vectorSize();
};

} // namespace g2o
#endif