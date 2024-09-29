###
 # @Author: GetOverMassif 164567487@qq.com
 # @Date: 2022-10-11 13:50:58
 # @LastEditors: GetOverMassif 164567487@qq.com
 # @LastEditTime: 2022-10-13 19:14:25
 # @FilePath: /DSP-SLAM/scripts/run_redwood.sh
 # @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
### 


# REDWOOD_SCAN_ID=01053  # 
# REDWOOD_SCAN_ID=01041  # 
# # REDWOOD_SCAN_ID=03815  # 效果不佳
# # REDWOOD_SCAN_ID=09374  # 效果较好

# mkdir -p map/redwood/$REDWOOD_SCAN_ID

# ./qsp_slam_mono Vocabulary/ORBvoc.bin configs/redwood_$REDWOOD_SCAN_ID.yaml \
# /media/lj/TOSHIBA/dataset/DSP-SLAM/data/redwood_chairs/$REDWOOD_SCAN_ID \
# map/redwood/$REDWOOD_SCAN_ID

# ./qsp_slam_rgbd Vocabulary/ORBvoc.bin configs/redwood_01053.yaml \
# /home/lj/Documents/ForkedRepo/redwood-3dscan/data/rgbd/01053 \
# /home/lj/Documents/ForkedRepo/redwood-3dscan/data/rgbd/01053/associate_short.txt \
# map/redwood/01053

# /home/lj/Documents/ForkedRepo/redwood-3dscan/data/rgbd/01053/associate.txt \

# ./qsp_slam_rgbd Vocabulary/ORBvoc.bin configs/redwood_03815.yaml \
# /home/lj/Documents/ForkedRepo/redwood-3dscan/data/rgbd/03815 \
# /home/lj/Documents/ForkedRepo/redwood-3dscan/data/rgbd/03815/associate.txt \
# map/redwood/03815


# ./qsp_slam_rgbd Vocabulary/ORBvoc.bin configs/redwood_monitor_01041.yaml \
# /media/lj/TOSHIBA/dataset/RedwoodOS/data/rgbd/01041  \
# /media/lj/TOSHIBA/dataset/RedwoodOS/data/rgbd/01041/associate.txt \
# map/redwood/01041

# DATASET_PATH=/media/lj/TOSHIBA/dataset
DATASET_PATH=/home/lj/Documents/dataset

# # REDWOOD_SCAN_ID=01053
# # REDWOOD_SCAN_ID=03815
# REDWOOD_SCAN_ID=09374

REDWOOD_SCAN_ID=01053
./qsp_slam_rgbd Vocabulary/ORBvoc.bin configs/redwood_chair_01053.yaml \
$DATASET_PATH/RedwoodOS/data/rgbd/$REDWOOD_SCAN_ID  \
$DATASET_PATH/RedwoodOS/data/rgbd/$REDWOOD_SCAN_ID/associate.txt \
map/redwood/$REDWOOD_SCAN_ID

# REDWOOD_SCAN_ID=01041
# ./qsp_slam_rgbd Vocabulary/ORBvoc.bin configs/redwood_monitor_01041.yaml \
# $DATASET_PATH/RedwoodOS/data/rgbd/$REDWOOD_SCAN_ID  \
# $DATASET_PATH/RedwoodOS/data/rgbd/$REDWOOD_SCAN_ID/associate.txt \
# map/redwood/$REDWOOD_SCAN_ID
