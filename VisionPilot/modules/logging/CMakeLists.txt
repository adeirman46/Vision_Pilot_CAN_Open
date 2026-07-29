cmake_minimum_required(VERSION 3.22.1)

add_library(logging
        src/logger.cpp
)

target_include_directories(logging
        PUBLIC
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)