# add_executable(rgbd_autoDA
#         ./rgbd_autoDA.cpp
# )
# target_link_libraries(rgbd_autoDA
#         tum_rgbd
#         boost_system
#         EllipsoidSLAM
#         ${PCL_LIBRARIES}
#         func
# )       

# add_executable(rgbd_odom
#         ./rgbd_odom.cpp
# )
# target_link_libraries(rgbd_odom
#         tum_rgbd
#         boost_system
#         EllipsoidSLAM
#         ${PCL_LIBRARIES}
#         func
# )

# 测试平面提取
add_executable(test_planeExtraction
        ./test/test_planeExtraction.cpp
)

target_link_libraries(test_planeExtraction
        tum_rgbd
        boost_system
        EllipsoidSLAM
        ${PCL_LIBRARIES}
)      

# 测试语义先验
add_executable(test_semanticprior
        ./test/test_semanticprior.cpp
)
target_link_libraries(test_semanticprior
        tum_rgbd
        boost_system
        EllipsoidSLAM
        ${PCL_LIBRARIES}
)   

# 
add_library(func SHARED
    ./func/func.cpp
)
target_link_libraries(func
    EllipsoidSLAM
)

# set(CMAKE_BUILD_TYPE Debug)
add_executable(experiment_lab
    ./lab/experiment_lab.cpp
)
target_link_libraries(experiment_lab
    func
    EllipsoidSLAM
    evo
)

# Load datset from yaml.
add_executable(rgbd_launcher
    ./rgbd_launcher.cpp
)
target_link_libraries(rgbd_launcher
    tum_rgbd
    boost_system
    EllipsoidSLAM
    ${PCL_LIBRARIES}
    func
)

add_executable(svd_test
    ./svd_test.cpp
)
target_link_libraries(svd_test
#        tum_rgbd
#        boost_system
        EllipsoidSLAM
#        ${PCL_LIBRARIES}
#        func
)

# add_executable(test_visualizeMap
#         ./test/test_visualizeMap.cpp
# )
# target_link_libraries(test_visualizeMap
#         EllipsoidSLAM
#         func
# )       

add_executable(baseline_NP
    ./baseline/NP.cpp
)
target_link_libraries(baseline_NP
    EllipsoidSLAM
    func
    tum_rgbd
)       

# # Math test
# add_executable(test_chi
#         ./test/test_chi.cpp
# )

# evo tool for hist-based orientation estimation
add_executable(evo_hist_orientation
    ./lab/evo_hist_orientation.cpp
)
target_link_libraries(evo_hist_orientation
    func
    EllipsoidSLAM
    evo
)

SET(CMAKE_BUILD_TYPE Debug)
add_executable(evo_local_det
    ./lab/evo_local_det.cpp
)

target_link_libraries(evo_local_det
    func
    EllipsoidSLAM
    evo
)

add_executable(test_priconfig
    ./test/test_priconfig.cpp
)
target_link_libraries(test_priconfig
    EllipsoidSLAM
)      

# SET(CMAKE_BUILD_TYPE Debug)
add_executable(test_textureArea
    ./test/test_textureArea.cpp
)

target_link_libraries(test_textureArea
    EllipsoidSLAM
)      

add_subdirectory(test/textureArea)

add_subdirectory(view_dataset)