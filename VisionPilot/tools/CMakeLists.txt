cmake_minimum_required(VERSION 3.22.1)

add_executable(build_trt_engines
        build_trt_engines.cpp
)

target_link_libraries(build_trt_engines
        PRIVATE
        config
        engine
        logging
)

target_compile_features(build_trt_engines PRIVATE cxx_std_17)

# Renders one navigation panel to a PNG — checks a freshly built .vpmap with no
# phone, no camera and no GPU involved.
find_package(OpenCV REQUIRED COMPONENTS imgcodecs)

add_executable(nav_preview
        nav_preview.cpp
)

target_link_libraries(nav_preview
        PRIVATE
        navigation
        logging
        opencv_imgcodecs
)

target_compile_features(nav_preview PRIVATE cxx_std_17)
