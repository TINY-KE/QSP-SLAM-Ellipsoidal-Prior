import argparse
import os
import os.path as osp
import subprocess
from associate import readAndAssociate

class Associater:
    def __init__(self, data_type) -> None:
        self.data_type = data_type
        self.rgb_img_path = osp.join(arg.data_dir, "rgb")
        self.depth_img_path = osp.join(arg.data_dir, "depth")
        self.rgb_txt_path = osp.join(arg.data_dir, "rgb.txt")
        self.depth_txt_path = osp.join(arg.data_dir, "depth.txt")
    
    def generate_rgb_and_depth_txt(self):
        self.get_img_txt(self.rgb_img_path, self.rgb_txt_path, "rgb")
        self.get_img_txt(self.depth_img_path, self.depth_txt_path, "depth")

    def get_img_txt(self, img_path, img_txt_path, prefix):
        img_list = sorted(os.listdir(img_path))
        with open(img_txt_path, 'w') as f:
            f.write(f"# color images\n")
            f.write(f"# file: '{self.data_type}'\n")
            f.write(f"# timestamp filename")
            for img_filename in img_list:
                if self.data_type == 'redwood':
                    timestamp = float(img_filename.split('.')[0].split('-')[1]) / 1000000
                elif self.data_type == 'normal':
                    timestamp = float(img_filename.split('.')[0]) / 30
                f.write(f"\n{'%.6f'%timestamp} {prefix}/{img_filename}")
    
    def associate_rgb_and_depth(self):
        readAndAssociate(self.rgb_txt_path, self.depth_txt_path)

if __name__ == "__main__":
    # parse command line
    parser = argparse.ArgumentParser(description='''
    This script takes two data files with timestamps and associates them   
    ''')
    parser.add_argument('-r','--data_dir', help='first text file (format: timestamp data)')
    parser.add_argument('-t','--data_type', required=str, default='normal')

    valid_types = ['redwood', 'normal']

    arg = parser.parse_args()

    assert arg.data_type in valid_types, f"type {arg.data_type} is not valid"

    associator = Associater(arg.data_type)
    associator.generate_rgb_and_depth_txt()
    associator.associate_rgb_and_depth()


    

    # cmd = f"python /home/lj/Documents/QSP-SLAM/scripts/associate.py {rgb_txt_path} {depth_txt_path}"
    


