
# function git_clone(){
#   repo_dir=`basename "$1" .git`
#   git -C "$repo_dir" pull 2> /dev/null || eval "$1"
# }

# source Thirdparty/bashcolors/bash_colors.sh
# function highlight(){
#   clr_magentab clr_bold clr_white "$1"
# }

# highlight "Installing system-wise packages ..."
# sudo apt-get update > /dev/null 2>&1 &&
# sudo apt -y install gcc-8 g++-8 # gcc-8 is a safe version 
# sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-8 8
# sudo update-alternatives --install /usr/bin/g++ g++ /usr/bin/g++-8 8
# sudo apt-get install -y cmake
# sudo apt-get install -y \
#   libglew-dev \
#   libgtk2.0-dev \
#   pkg-config \
#   libegl1-mesa-dev \
#   libwayland-dev \
#   libxkbcommon-dev \
#   wayland-protocols

# # [ g2o ]
# cd Thirdparty/g2o
# mkdir build
# cd build
# cmake ..
# make -j4
# sudo make install
# cd ../../..

# # [ DBoW2 ]
# cd Thirdparty/DBoW2
# # if [ ! -d build ]; then
# mkdir build
# # fi
# cd build
# cmake ..
# # cmake -DOpenCV_DIR=$OpenCV_DIR ..
# make -j8
# cd ../../..

# # [ pybind11 ]
# git submodule update --init --recursive
# cd pybind11
# mkdir build
# cd build
# cmake ..
# make -j4
# sudo make install
# cd ../..

# # 

# conda env create -f environment_cuda113.yml
# conda activate dsp-slam

sudo apt install python3-dev

pip install pycocotools==2.0.1
pip install mmcv-full==1.4.0 -f https://download.openmmlab.com/mmcv/dist/cu113/torch1.10.0/index.html
pip install mmdet==2.14.0
pip install mmsegmentation==0.14.1