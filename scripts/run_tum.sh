###
 # @Author: GetOverMassif 164567487@qq.com
 # @Date: 2022-10-11 13:50:58
 # @LastEditors: GetOverMassif 164567487@qq.com
 # @LastEditTime: 2022-10-13 19:14:25
 # @FilePath: /DSP-SLAM/scripts/run_redwood.sh
 # @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
### 


# ./qsp_slam_rgbd Vocabulary/ORBvoc.bin configs/tum_fr1_desk.yaml \
# /media/lj/TOSHIBA/dataset/TUM/fr1_desk/rgbd_dataset_freiburg1_desk \
# configs/associations/fr1_desk.txt \
# map/tum/fr1_desk

# configs/redwood_chair_01053.yaml \

# dishes # configs/tum_fr2_dishes.yaml \
./qsp_slam_rgbd \
Vocabulary/ORBvoc.bin \
configs/tum_fr2_dishes.yaml \
/home/lj/Documents/dataset/TUM/rgbd_dataset_freiburg2_dishes \
/home/lj/Documents/dataset/TUM/rgbd_dataset_freiburg2_dishes/associate.txt \
map/tum/fr2_dishes


# REDWOOD_SCAN_ID=01053
# ./qsp_slam_rgbd Vocabulary/ORBvoc.bin configs/redwood_chair_01053.yaml \
# $DATASET_PATH/RedwoodOS/data/rgbd/$REDWOOD_SCAN_ID  \
# $DATASET_PATH/RedwoodOS/data/rgbd/$REDWOOD_SCAN_ID/associate.txt \
# map/redwood/$REDWOOD_SCAN_ID
