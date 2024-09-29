
add_executable(dataset_viewer dataset_viewer.cpp)

target_link_libraries(dataset_viewer
    EllipsoidSLAM
    func
    tum_rgbd
    evo
)