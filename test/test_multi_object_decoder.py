import sys

sys.path.append('./')
sys.path.append('./../')

import reconstruct.utils as io_utils
from deep_sdf.workspace import config_decoder

pyCfgPath = "configs/config_redwood_chair_01053.json"
# pyCfgPath2 = "configs/config_redwood_monitor_01041.json"

DeepSDF_DIR1 = "weights/deepsdf/chairs_64"
DeepSDF_DIR2 = "weights/deepsdf/displays_64"



decoder1 = config_decoder(DeepSDF_DIR1)
decoder2 = config_decoder(DeepSDF_DIR2)

import cv2
img_path = f"/home/lj/Documents/dataset/RedwoodOS/data/rgbd/01053/depth/0000001-000000000000.png"
img = cv2.imread(img_path)
cv2.imshow("HELLO", img)
cv2.waitKey(0)
cv2.destroyAllWindows()
