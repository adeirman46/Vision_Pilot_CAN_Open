cmake_minimum_required(VERSION 3.22.1)

# Qt5 is what this machine has (5.15.3) and what openpilot's UI targets too.
find_package(Qt5 COMPONENTS Widgets QUIET)

if (NOT Qt5_FOUND)
    message(WARNING "Qt5 Widgets not found — skipping the VisionPilotQt front-end. "
                    "Install with: sudo apt-get install -y qtbase5-dev")
    return()
endif ()

set(CMAKE_AUTOMOC ON)

add_executable(VisionPilotQt
        main.cpp
        main_window.cpp
        pipeline_worker.cpp
)

set_target_properties(VisionPilotQt PROPERTIES
        CXX_STANDARD 17
        CXX_STANDARD_REQUIRED ON
        BUILD_RPATH "${ONNXRUNTIME_ROOT}/lib"
        INSTALL_RPATH "/usr/share/visionpilot/onnxruntime/lib"
        BUILD_WITH_INSTALL_RPATH FALSE
)

add_dependencies(VisionPilotQt generate_config)

target_include_directories(VisionPilotQt PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${CMAKE_SOURCE_DIR}/modules/sensing/camera_interface/include
        ${CMAKE_SOURCE_DIR}/modules/sensing/vehicle_interface/include
)

target_link_libraries(VisionPilotQt
        PRIVATE
        Qt5::Widgets
        config
        logging
        common
        engine
        models
        fusion
        planning
        navigation
        vehicle_interface
        camera_interface
        image_preprocessing
        visualization
        vp_debug
)

# Same reason as the plain app: opencv_viz drags in system libs with unresolved
# symbols we never call.
target_link_options(VisionPilotQt PRIVATE -Wl,--allow-shlib-undefined)

install(TARGETS VisionPilotQt RUNTIME DESTINATION bin)
