#pragma once

#include <string>
#include <engine/onnx_engine.hpp>
#include <models/inference.hpp>
#include <navigation/nav_config.hpp>

namespace vpe = visionpilot::engine;
namespace vpm = visionpilot::models;

enum class SourceMode { Ros2 = 0, V4l2 = 1, Video = 2 };

struct SourceConfig {
    SourceMode  mode         = SourceMode::Video;
    std::string input_video;
    std::string input_vehicle_speed;
    std::string dataset;
    bool        video_realtime = true;
    bool        video_loop     = false;
    std::string input_camera_topic   = "/camera/image";
    std::string v4l2_device  = "/dev/video0";
    int         v4l2_fps     = 10;
};

// struct PipelineConfig {
//     bool initial_inference_check = true;
// };

// ── Route corridor ───────────────────────────────────────────────────────────
// Makes the vision path follow the navigation route: the route's own waypoints
// are converted into the ego frame and used to throw away the AutoSteer
// waypoints that describe a branch the route does not take. Without it, the
// path fit at a junction picks a branch at random and re-picks it every frame.
//
// It needs a route, so it does nothing at all unless nav is on and a
// destination is set. The corridor width itself lives in Config::inference,
// which is the struct LateralFusion is built from.
struct GuidanceConfig {
    bool   enabled     = false;
    // How far up the route to build the corridor, metres. Long enough to cover
    // the junction the car is approaching, short enough that the far end of it
    // is not being placed by a heading estimate that has since gone stale.
    double lookahead_m = 60.0;
};

struct Config {
    vpe::Config engine;
    vpm::Config inference;
    SourceConfig      source;
    // Offline OSM map panel drawn to the right of the inference view
    navigation::Config navigation;
    // Route corridor: the navigation route filtering the vision waypoints
    GuidanceConfig     guidance;
    // True once a real vehicle speed arrives over CAN. While false the HUD
    // shows a dash instead of the synthesised constant the planner is fed.
    bool vehicle_speed_valid = false;
    // PipelineConfig    pipeline;
    // Print per-frame fusion debug logs
    bool              fusion_debug   = false;
    // Directory with wheel_white.png / wheel_green.png for steering HUD
    std::string       wheel_dir;

    std::string vehicle_speed_topic;
    std::string vehicle_steering_topic;
    std::string vehicle_acceleration_topic;

    double speed_limit;
    double Lf;

    bool visualization_on = false;
    bool webrtc_on = false;
    int webrtc_port;
};

static std::string find_config(const std::string& filename) {
    const std::string local  = "config/" + filename;
    const std::string system = "/usr/share/visionpilot/config/" + filename;

    if (std::filesystem::exists(local))  return local;
    if (std::filesystem::exists(system)) return system;

    throw std::runtime_error("Config file not found: " + filename);
}

// Load from key=value .conf file. Expands ~ to $HOME.
// Throws std::runtime_error on missing or invalid config.
Config load_vision_pilot_config();

// Resolve config path from --config <path>, VISIONPILOT_CONFIG env var,
// or default candidates. Returns empty string if nothing found.
std::string resolve_vision_pilot_config_path(int argc, char** argv);

SourceMode parse_source_mode(const std::string& value);

// True when a source.input_video value is a network stream (http://, rtsp://, …)
// rather than a path on disk — a live source has no file to stat, no frame count
// and no end.
bool is_stream_url(const std::string& value);

// Short label for debug overlay (video / device path / ROS topic).
std::string source_label(const SourceConfig& source);
