set(CMAKE_BUILD_TYPE Debug)
add_library(evo SHARED
    evo.cpp
)

target_link_libraries(evo
        EllipsoidSLAM
        func
)
