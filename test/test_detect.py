import sys
sys.path.append("./")
import cv2

from mmdet.apis import inference_detector

from reconstruct.detector2d import Detector2D
from reconstruct.utils import get_configs

if __name__=="__main__":
    config_path = f"configs/config_freiburg_001.json"
    img_path = f"files/test_depth_img/redwood_01053_1_rgb.jpg"
    image = cv2.imread(img_path)
    configs = get_configs(config_path)
    detector = Detector2D(configs)
    # detector.make_prediction()
    predictions = inference_detector(detector.model, image)
    print("hello")
