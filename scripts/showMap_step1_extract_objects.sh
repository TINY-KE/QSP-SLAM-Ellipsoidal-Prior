###
 # @Author: GetOverMassif 164567487@qq.com
 # @Date: 2022-10-11 14:21:53
 # @LastEditors: GetOverMassif 164567487@qq.com
 # @LastEditTime: 2022-10-12 00:17:24
 # @FilePath: /DSP-SLAM/scripts/extract_map_objects.sh
 # @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
### 
python extract_map_objects.py --config configs/config_redwood_09374.json --map_dir map/redwood/07229 --voxels_dim 64
python visualize_map.py --config configs/config_redwood_09374.json --map_dir map/redwood/07229