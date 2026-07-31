cmake_minimum_required(VERSION 3.22.1)

find_package(OpenCV REQUIRED COMPONENTS core imgproc)
find_package(Threads REQUIRED)

add_library(navigation
        src/gps_sources.cpp
        src/http_get.cpp
        src/osm_map.cpp
        src/router.cpp
        src/map_renderer.cpp
        src/navigation.cpp
)

set_target_properties(navigation PROPERTIES
        CXX_STANDARD 17
        CXX_STANDARD_REQUIRED ON
)

target_include_directories(navigation
        PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(navigation
        PUBLIC
        opencv_core
        opencv_imgproc
        Threads::Threads
        PRIVATE
        logging
)
