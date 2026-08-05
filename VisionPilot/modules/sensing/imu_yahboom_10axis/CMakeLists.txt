cmake_minimum_required(VERSION 3.16)

# Builds on its own — `cmake -S . -B build && cmake --build build` — and also as
# an add_subdirectory() of VisionPilot, which is how the GPS/IMU fusion work
# will pull the library in.
project(imu_yahboom_10axis VERSION 1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if (CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})
    if (NOT CMAKE_BUILD_TYPE)
        set(CMAKE_BUILD_TYPE Release)
    endif ()
endif ()

find_package(Threads REQUIRED)

# ─── Library: driver + attitude estimate, no UI dependencies ─────────────────
add_library(imu_yahboom
        src/serial_port.cpp
        src/imu_decoder.cpp
        src/attitude.cpp
        src/imu_reader.cpp
)

target_include_directories(imu_yahboom
        PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(imu_yahboom PUBLIC Threads::Threads)

# ─── Console monitor ─────────────────────────────────────────────────────────
add_executable(imu_monitor tools/imu_monitor.cpp)
target_link_libraries(imu_monitor PRIVATE imu_yahboom)

# ─── Live 3D viewer ──────────────────────────────────────────────────────────
# Qt5 Widgets only: the scene is rendered with QPainter, so there is no GL
# context to negotiate with the driver the inference stack is already using.
find_package(Qt5 COMPONENTS Widgets QUIET)

if (Qt5_FOUND)
    set(CMAKE_AUTOMOC ON)
    add_executable(imu_view
            viewer/main.cpp
            viewer/imu_window.cpp
            viewer/scene3d.cpp
    )
    target_include_directories(imu_view PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/viewer)
    target_link_libraries(imu_view PRIVATE imu_yahboom Qt5::Widgets)
    install(TARGETS imu_view RUNTIME DESTINATION bin)
else ()
    message(WARNING
            "Qt5 Widgets not found — building imu_monitor only, without the 3D viewer. "
            "Install it with: sudo apt-get install -y qtbase5-dev")
endif ()

install(TARGETS imu_yahboom ARCHIVE DESTINATION lib)
install(TARGETS imu_monitor RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)

# ─── Tests ───────────────────────────────────────────────────────────────────
# Dead reckoning is the part that cannot be checked by looking at the screen:
# a track that drifts and a track that is simply wrong look identical. This
# drives the estimator with a motion profile whose answer is known exactly.
enable_testing()
add_executable(test_dead_reckoning tests/test_dead_reckoning.cpp)
target_link_libraries(test_dead_reckoning PRIVATE imu_yahboom)
add_test(NAME dead_reckoning COMMAND test_dead_reckoning)
