###
 # @Author: GetOverMassif 164567487@qq.com
 # @Date: 2022-10-11 13:50:58
 # @LastEditors: GetOverMassif 164567487@qq.com
 # @LastEditTime: 2022-10-13 19:14:25
 # @FilePath: /DSP-SLAM/scripts/run_redwood.sh
 # @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
### 



# configs/redwood_chair_01053.yaml \

# dishes # configs/tum_fr2_dishes.yaml \


# ./qsp_slam_rgbd \
# Vocabulary/ORBvoc.bin \
# configs/self_allobject_circle.yaml \
# /media/lj/TOSHIBA/dataset/FromZHJD/circle \
# /media/lj/TOSHIBA/dataset/FromZHJD/circle/associate.txt \
# map/self/circle


./ros_qsp_slam_rgbd \
Vocabulary/ORBvoc.bin \
configs/ros_self_allobject_ground.yaml \
/home/robotlab/ws_3d_vp/src/QSP-SLAM-Ellipsoidal-Prior/dataset/ros_realtime \
map/ros_realtime