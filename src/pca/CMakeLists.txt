# set(CMAKE_BUILD_TYPE Debug)

add_library(EllipsoidExtractor SHARED
    EllipsoidExtractor.cpp
    EllipsoidExtractorMultiPlanes.cpp
    EllipsoidExtractorEdges.cpp
    EllipsoidExtractorLocalOptimization.cpp
)

target_link_libraries(EllipsoidExtractor
    pybind11::embed
    ${OpenCV_LIBS}
    symmetry
    utils
)
