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
        src/gps_imu_ekf.cpp
        src/fused_gps.cpp
)

# Sits with the other sensor drivers, and still builds standalone.
if (NOT TARGET imu_yahboom)
    add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/../sensing/imu_yahboom_10axis
            ${CMAKE_BINARY_DIR}/imu_yahboom EXCLUDE_FROM_ALL)
endif ()

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
        imu_yahboom
        PRIVATE
        logging
)

# ─── Tests ───────────────────────────────────────────────────────────────────
# A filter that runs is not a filter that works, and both failure modes are
# silent on a map. This drives it along a known route and measures the error.
add_executable(test_gps_imu_ekf tests/test_gps_imu_ekf.cpp)
target_link_libraries(test_gps_imu_ekf PRIVATE navigation)
target_compile_features(test_gps_imu_ekf PRIVATE cxx_std_17)

# The Nucleo's wire format, checked against a verbatim capture through a pty.
# A field read from the wrong end still checksums and still puts a plausible
# coordinate on the map, so known bytes are the only way to catch it — and this
# runs with no board plugged in.
add_executable(test_nucleo_gps tests/test_nucleo_gps.cpp)
target_link_libraries(test_nucleo_gps PRIVATE navigation)
target_compile_features(test_nucleo_gps PRIVATE cxx_std_17)
