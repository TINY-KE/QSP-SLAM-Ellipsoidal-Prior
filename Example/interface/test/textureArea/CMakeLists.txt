# add_library(EllipsoidTexture SHARED
# ./EllipsoidTexture.cpp)

# add_library(TextureFeature SHARED
# ./TextureFeature.cpp)

add_library(TextureTestFunctions SHARED
./TestFunctions.cpp
)
target_link_libraries(TextureTestFunctions
TextureArea
tum_rgbd)

add_library(TextureArea SHARED
./TextureFeature.cpp
./EllipsoidTexture.cpp
./TextureOptimizer.cpp
./TextureEdges.cpp
)
# target_link_libraries(TextureArea
# EllipsoidSLAM
# )

add_executable(test_textureAreaFrames
test_textureAreaFrames.cpp)
target_link_libraries(test_textureAreaFrames
EllipsoidSLAM
TextureArea
func
tum_rgbd
TextureTestFunctions)

# set(CMAKE_BUILD_TYPE Debug)
add_executable(test_textureAreaMultipleFrames
test_textureAreaMultipleFrames.cpp)
target_link_libraries(test_textureAreaMultipleFrames
EllipsoidSLAM
TextureArea
func
tum_rgbd
evo
TextureTestFunctions
)

add_executable(test_dt
test_dt.cpp)
target_link_libraries(test_dt
${OpenCV_LIBS}
)

add_executable(evo_checkAssociations
evo_checkAssociations.cpp)
target_link_libraries(evo_checkAssociations
EllipsoidSLAM
TextureArea
func
tum_rgbd
TextureTestFunctions
evo)