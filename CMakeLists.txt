cmake_policy(SET CMP0048 NEW)
project("QSP-SLAM-Ellipsoidal-Prior")
cmake_minimum_required(VERSION 3.2)

# c++ version
set(CMAKE_CXX_STANDARD 17)

#set(cv_bridge_DIR "/home/robotlab/thirdparty/for_dspslam/vision_opencv-indigo/cv_bridge/build")
find_package(catkin REQUIRED COMPONENTS
        roscpp
        sensor_msgs
        #        image_transport
        std_msgs
        tf

        # plane
        #        pcl_ros
        #        pcl_conversions

#        cv_bridge
)

catkin_package(
        #  INCLUDE_DIRS include
        #  LIBRARIES test
        #  CATKIN_DEPENDS roscpp
        #  DEPENDS system_lib
        #     INCLUDE_DIRS include
        #     LIBRARIES connect_gazebo
        #     CATKIN_DEPENDS roscpp std_msgs test_pkg gazebo_msgs
)   #for ros


# 关闭所有warning
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -w")

IF(NOT CMAKE_BUILD_TYPE)
    SET(CMAKE_BUILD_TYPE Release)
ENDIF()

SET(CMAKE_BUILD_TYPE Release)
# SET(CMAKE_BUILD_TYPE Debug)

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -O3 -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O3 -march=native")

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)


set(OpenCV_DIR "/home/robotlab/thirdparty/for_dspslam/opencv/build")
set(Pangolin_DIR "/home/robotlab/thirdparty/for_dspslam/Pangolin/build")
set(PYTHON_LIBRARIES  "/home/robotlab/anaconda3/envs/dsp-slam/lib/libpython3.7m.so")
set(PYTHON_INCLUDE_DIRS  "/home/robotlab/anaconda3/envs/dsp-slam/include/python3.7m")
set(PYTHON_EXECUTABLE  "/home/robotlab/anaconda3/envs/dsp-slam/bin/python3.7")

find_package(OpenCV 3.4.1 REQUIRED)
find_package(Eigen3 3.3.0 REQUIRED)
find_package(Pangolin REQUIRED)
include_directories(/usr/include/eigen3)

# 打印 Eigen 版本信息
message(STATUS "Eigen version: ${EIGEN3_VERSION}")
# 输出 Eigen 的 include 目录
message(STATUS "Eigen include directory: ${EIGEN3_INCLUDE_DIRS}")
message(STATUS "Eigen include directory2: ${EIGEN3_INCLUDE_DIR}")

# Pass the options to CMake
add_compile_options(
  -DPYTHON_LIBRARIES=${PYTHON_LIBRARIES}
  -DPYTHON_INCLUDE_DIRS=${PYTHON_INCLUDE_DIRS}
  -DPYTHON_EXECUTABLE=${PYTHON_EXECUTABLE}
)

if(DEFINED ENV{PCL_DIR})
   message("Environment variable PCL_DIR: " $ENV{PCL_DIR})
   set(PCL_DIR $ENV{PCL_DIR})
endif()

include_directories(/usr/include/vtk-7.1)

# seach
find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)
# include    
include_directories(${OPENGL_INCLUDE_DIRS})
include_directories(${GLUT_INCLUDE_DIRS})


#for ros

# set(PCL_DIR "home/zhjd/thirdparty/pcl-1.13.1/build")
set(PCL_DIR  /home/robotlab/thirdparty_install/pcl-1.13-eigen337/share/pcl-1.13)
FIND_PACKAGE( PCL REQUIRED )
list(REMOVE_ITEM PCL_LIBRARIES "vtkproj4")
ADD_DEFINITIONS( ${PCL_DEFINITIONS} )
INCLUDE_DIRECTORIES( ${PCL_INCLUDE_DIRS}  )
LINK_LIBRARIES( ${PCL_LIBRARY_DIRS} )





include_directories(
    ${catkin_INCLUDE_DIRS}   #for ros
    ${PROJECT_SOURCE_DIR}
    ${PROJECT_SOURCE_DIR}/include
    ${PROJECT_SOURCE_DIR}/include/core
    ${PROJECT_SOURCE_DIR}/Thirdparty/g2o
    ${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2
    ${EIGEN3_INCLUDE_DIR}
    ${Pangolin_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)


# You need to set to your own Python paths
add_subdirectory(pybind11)

# include_directories(/home/lj/Documents/DSC-SLAM/pybind11)

set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

add_library(utils SHARED
    src/utils/matrix_utils.cpp
    src/utils/dataprocess_utils.cpp
    src/utils/file_operate.cpp
)

target_link_libraries(utils
    ${OpenCV_LIBS}
)


add_library(${PROJECT_NAME} SHARED
    src/System.cc
    src/System_util.cc
    src/Tracking.cc
    src/Tracking_util.cc
    src/Tracking_ellipsoid.cc
    src/LocalMapping.cc
    src/LocalMapping_util.cc
    src/LoopClosing.cc
    src/LoopClosing_util.cc
    src/ORBextractor.cc
    src/ORBmatcher.cc
    src/FrameDrawer.cc
    src/Converter.cc
    src/MapPoint.cc
    src/MapPoint_util.cc
    src/KeyFrame.cc
    src/KeyFrame_util.cc
    src/Map.cc
    src/Map_util.cc
    src/MapDrawer.cc
    src/MapDrawer_util.cc
    src/Optimizer.cc
    src/Optimizer_util.cc
    src/PnPsolver.cc
    src/Frame.cc
    src/KeyFrameDatabase.cc
    src/Sim3Solver.cc
    src/Initializer.cc
    src/Viewer.cc
    src/ObjectDetection.cc
    src/MapObject.cc
    src/ObjectDrawer.cc
    src/ObjectRenderer.cc
    src/MapPublisher.cc

    src/Labeller.cc

    # src/VLab.cc

    src/core/Ellipsoid.cpp
    src/core/Geometry.cpp
    src/core/Plane.cpp
#     src/core/DataAssociation.cpp
    src/core/BasicEllipsoidEdges.cpp
    src/core/NonparamDA.cpp
    src/core/SupportingPlane.cpp
    src/core/ConstrainPlane.cpp
#     src/core/SemanticLabel.cpp
    src/core/PriorInfer.cpp


)



# find_package(PythonLibs REQUIRED)

# include_directories(${PYTHON_INCLUDE_DIRS})

# target_link_libraries(<your exe or lib> ${PYTHON_LIBRARIES})

message("PYTHON_INCLUDE_DIRS = " ${PYTHON_INCLUDE_DIRS})

target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES}  #for ros

        pybind11::embed
        ${OpenCV_LIBS}
        ${EIGEN3_LIBS}
        ${Pangolin_LIBRARIES}
        ${PROJECT_SOURCE_DIR}/Thirdparty/DBoW2/lib/libDBoW2.so
        ${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so
        # ${PYTHON_LIBRARIES}
        ${OPENGL_LIBRARIES} ${GLUT_LIBRARIES}
        
        utils
        Config
        symmetry
        EllipsoidExtractor
        PlaneExtractor
        dense_builder
        Polygon
        Relationship
        # tools
        )



# # set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR})
# add_executable(label_tool label_tool.cc)
# target_link_libraries(label_tool ${PROJECT_NAME})


#add_executable(pcd2bin src/tools/pcd2bin.cc)
#target_link_libraries(pcd2bin ${PROJECT_NAME})

# add modules
add_subdirectory(src/dense_builder)
add_subdirectory(src/symmetry)
add_subdirectory(src/config)
add_subdirectory(src/pca)
add_subdirectory(src/plane)
add_subdirectory(src/Polygon)
add_subdirectory(src/Relationship)
add_subdirectory(src/tum_rgbd)

# add_subdirectory(src/semantic-data-association-tool)

# add_subdirectory(src/eval)

# add_subdirectory(src/evo)
# add_subdirectory(src/Prior)

# add_subdirectory(Example/interface)


# add_subdirectory(src/tools)





# set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR})
# add_executable(qsp_slam qsp_slam.cc)
# target_link_libraries(qsp_slam ${PROJECT_NAME})

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR})
add_executable(qsp_slam_mono qsp_slam_mono.cc)
target_link_libraries(qsp_slam_mono ${PROJECT_NAME})

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR})
add_executable(qsp_slam_rgbd qsp_slam_rgbd.cc)
target_link_libraries(qsp_slam_rgbd ${PROJECT_NAME})

#set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR})
#add_executable(ros_qsp_slam_rgbd ros_qsp_slam_rgbd.cpp)
#target_link_libraries(ros_qsp_slam_rgbd ${PROJECT_NAME})

#set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR})
#add_executable(image_grabber_node image_grabber_node.cpp)
### 链接目标文件与catkin库和OpenCV库
#target_link_libraries(image_grabber_node
#        ${catkin_LIBRARIES}
#        ${OpenCV_LIBRARIES}
#)