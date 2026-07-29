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
