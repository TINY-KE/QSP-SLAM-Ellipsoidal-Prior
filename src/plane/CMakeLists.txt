

# SET(CMAKE_BUILD_TYPE Debug)

add_library(PlaneExtractor SHARED
PlaneExtractor.cpp
PlaneExtractorManhattan.cpp
)

find_package(Boost 1.54 REQUIRED)

target_link_libraries(PlaneExtractor
        ${OpenCV_LIBS}
        ${PCL_LIBRARIES}
        Config
        )
