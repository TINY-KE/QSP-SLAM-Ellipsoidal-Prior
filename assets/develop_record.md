
“√”或者“×”

# 12.14
[] RGBD mode create KeyFrame every 2 Frame in average, costing too much time.
[] Backgroud rays in render loss depend on bg keypoints in Object mask, and the render loss doesn't constrain object scale efficiently.

[√] Add part of EllipsoldSLAM into QSP-SLAM and compile successfully.


# 1.2


目标仍然不够明确

[] 有待补充的功能： 手动设置地面位置
[] 希望建立的工具： 基于点云/网格模型，标注场景中物体的外包框（包含了物体位姿、尺度属性）
[] 希望能够显示出的最终地图： 全局rgb点云图，物体网格模型及椭球体，提取地面，相机轨迹真值
[] 物体重建质量评估：
    - 位置误差：中心点位置误差 e_x,e_y,e_z (这里需要注意先将世界模型转换到与重力坐标系对齐)
    - 朝向误差：偏航角误差 e_yaw, 俯仰角误差 e_pitch, 滚转角误差 e_roll
    - 形状误差：
        有表面距离（Chamfer Distance）和完整性（Completeness）
        - 网格模型与网格模型的形状对比：
        - 网格模型与点云模型的形状对比：
[] 评测轨迹误差：
    - 绝对轨迹误差（ATE）的均方根误差（RMSE）
    - 


ScanNet 有一个重要问题：rgb图像与depth图像的分辨率不一致

首先我要想清楚需要使用ScanNet做什么，ScanNet提供了真实场景的全扫描数据（rgb,depth,pose,intrinsics），以及网格重建结果、语义实例分割，

涉及到的语义类型包括：floor,wall,cabinet,bed,chair,sofa,table,door,window,refrigerator,bookshelf,picture,counter,desk,curtain,shower curtain,bathtub,floor mat,toilet,sink,clothes,blinds,shelves,dresser,pillow,mirror,ceiling,books,television,paper,towel,box,whiteboard,person,nightstand,lamp,bag,otherfurniture,otherprop,otherstructure,unannotated

今天任务： 实现一个标注工具，能够对点云或者网格模型进行物体真值标注


明天任务： 实现物体位姿、尺度、形状的误差计算


常见问题：fatal error: Python.h: No such file or directory
解决方法：


下一步工作：
    搭建一个包含多把椅子的仿真环境，规划相机移动轨迹，录制数据集（rgb/depth），并且保存相机轨迹、物体属性真值。

    对多类别/多物体场景运行RGBD模式，获得多个物体的位姿、形状估计结果，与真值进行误差计算。
