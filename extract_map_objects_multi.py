#
# This file is part of https://github.com/JingwenWang95/DSP-SLAM
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>
#

import os
import numpy as np
import argparse
from reconstruct.optimizer import MeshExtractor
from reconstruct.utils import get_configs, get_decoder, write_mesh_to_ply
from deep_sdf.workspace import config_decoder
import yaml

from reconstruct.optimizer import Optimizer, MeshExtractor

def read_yaml_file(file_path):
    with open(file_path, 'r', encoding='utf-8') as f:
        f.readline()
        result = yaml.load(f.read(), Loader=yaml.FullLoader)
    return result

def config_parser():
    parser = argparse.ArgumentParser()

    default_map_dir = 'map/self/GroundObjects'
    default_yaml_file = '/home/lj/Documents/codes/QSP-SLAM/configs/self_allobject_ground.yaml'
    # default_voxels_dim = 128

    parser.add_argument('-y', '--yaml', type=str, default=default_yaml_file, help='path to yaml file')
    parser.add_argument('-m', '--map_dir', type=str, default=default_map_dir, help='path to map directory')
    parser.add_argument('-n', '--voxels_dim', type=int, default=128, help='voxels resolution for running marching cube')
    return parser

if __name__ == "__main__":
    parser = config_parser()
    args = parser.parse_args()


    yaml_file = args.yaml
    map_dir = args.map_dir
    voxels_dim = args.voxels_dim
    code_len = 64

    yaml_data = read_yaml_file(yaml_file)
    yolo_classes = yaml_data['YoloClasses']
    decoder_paths = yaml_data['DecoderPaths']
    mmPyDecoders = {}
    mmPyMeshExtractors = {}

    # print(yolo_classes)

    for i_class in range(len(yolo_classes)):
        class_id = yolo_classes[i_class]
        decoder = config_decoder(decoder_paths[i_class])
        mmPyDecoders[class_id] = decoder

    for class_id in list(mmPyDecoders.keys()):
        decoder = mmPyDecoders[class_id]
        mesh_extractor = MeshExtractor(decoder, code_len, voxels_dim)
        mmPyMeshExtractors[class_id] = mesh_extractor

    # map_dir = args.map_dir
    save_dir = os.path.join(map_dir, "objects")
    if not os.path.exists(save_dir):
        os.makedirs(save_dir)

    with open(os.path.join(map_dir, "MapObjects.txt")) as f:
        lines = f.readlines()
        N = int(len(lines) / 3)
        for i in range(N):
            line_info = lines[3 * i].strip().split(" ")
            obj_id = int(line_info[0])
            class_id = int(line_info[1])

            line_pose = lines[3 * i + 1]
            pose = np.asarray([float(x) for x in line_pose.strip().split(" ")]).reshape(3, 4)
            pose = np.concatenate([pose, np.array([0., 0., 0., 1.]).reshape(1, 4)], axis=0)
            np.save(os.path.join(save_dir, "%d.npy" % obj_id), pose)
            code = []
            line_code = lines[3 * i + 2]
            for item in line_code.strip().split(" "):
                if len(item) > 0:
                    code += [float(item)]

            code = np.asarray(code).astype(np.float32)
            mesh = mmPyMeshExtractors[class_id].extract_mesh_from_code(code)
            write_mesh_to_ply(mesh.vertices, mesh.faces, os.path.join(save_dir, "%d.ply" % obj_id))
