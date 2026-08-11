#include <config/vision_pilot_config.hpp>

#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>

namespace {

std::string trim(const std::string& s)
{
    const auto a = s.find_first_not_of(" \t\r\n");
    if (a == std::string::npos) return "";
    return s.substr(a, s.find_last_not_of(" \t\r\n") - a + 1);
}

std::string expand_home(std::string p)
{
    if (!p.empty() && p[0] == '~') {
        const char* home = std::getenv("HOME");
        if (!home) throw std::runtime_error("HOME not set, cannot expand ~");
        if (p.size() == 1 || p[1] == '/') p.replace(0, 1, home);
        else throw std::runtime_error("Only ~ and ~/path are supported");
    }
    return p;
}

std::map<std::string, std::string> parse_conf(const std::string& path)
{
    std::ifstream f(path);
    if (!f) throw std::runtime_error("Cannot open config: " + path);

    std::map<std::string, std::string> kv;
    std::string line;
    int ln = 0;
    while (std::getline(f, line)) {
        ++ln;
        line = trim(line);
        if (line.empty() || line[0] == '#') continue;
        const auto eq = line.find('=');
        if (eq == std::string::npos)
            throw std::runtime_error(path + ":" + std::to_string(ln) + ": expected key=value");
        std::string key = trim(line.substr(0, eq));
        std::string val = trim(line.substr(eq + 1));
        if (key.empty())
            throw std::runtime_error(path + ":" + std::to_string(ln) + ": empty key");
        kv[key] = val;
    }
    return kv;
}

const std::string& require(const std::map<std::string, std::string>& kv, const std::string& k)
{
    const auto it = kv.find(k);
    if (it == kv.end() || it->second.empty())
        throw std::runtime_error("Missing required config key: " + k);
    return it->second;
}

std::string optional(const std::map<std::string, std::string>& kv,
                     const std::string& k, const std::string& def)
{
    const auto it = kv.find(k);
    return (it == kv.end() || it->second.empty()) ? def : it->second;
}

int parse_int(const std::string& s, const std::string& k)
{
    try { return std::stoi(s); }
    catch (...) { throw std::runtime_error("Invalid int for " + k + ": " + s); }
}

double parse_double(const std::string& s, const std::string& k)
{
    try { return std::stod(s); }
    catch (...) { throw std::runtime_error("Invalid number for " + k + ": " + s); }
}

bool parse_bool(const std::string& s, const std::string& k)
{
    if (s=="1"||s=="true"||s=="True"||s=="yes"||s=="on")  return true;
    if (s=="0"||s=="false"||s=="False"||s=="no"||s=="off") return false;
    throw std::runtime_error("Invalid bool for " + k + ": " + s);
}

bool file_ok(const std::string& p) { return !p.empty() && std::filesystem::is_regular_file(p); }

// "lat,lon" (whitespace tolerated). Throws on anything else.
void parse_lat_lon(const std::string& s, const std::string& k, double& lat, double& lon)
{
    const auto comma = s.find(',');
    if (comma == std::string::npos)
        throw std::runtime_error("Invalid lat,lon for " + k + ": " + s);
    lat = parse_double(trim(s.substr(0, comma)), k);
    lon = parse_double(trim(s.substr(comma + 1)), k);
    if (lat < -90.0 || lat > 90.0 || lon < -180.0 || lon > 180.0)
        throw std::runtime_error("Out-of-range lat,lon for " + k + ": " + s);
}

// http://host:port/video → http://host:port. The phone streaming the camera is
// the same device serving /sensors.json, so its GPS URL comes for free.
std::string url_origin(const std::string& url)
{
    const auto scheme = url.find("://");
    if (scheme == std::string::npos) return {};
    const auto slash = url.find('/', scheme + 3);
    return slash == std::string::npos ? url : url.substr(0, slash);
}

}  // namespace

SourceMode parse_source_mode(const std::string& v)
{
    if (v=="0"||v=="ros2")  return SourceMode::Ros2;
    if (v=="1"||v=="v4l2")  return SourceMode::V4l2;
    if (v=="2"||v=="video") return SourceMode::Video;
    throw std::runtime_error("Invalid source.mode: '" + v + "'. Use video|ros2|v4l2 (or 0/1/2)");
}

bool is_stream_url(const std::string& value)
{
    for (const char* scheme : {"http://", "https://", "rtsp://", "rtmp://",
                               "udp://", "tcp://", "rtp://"})
        if (value.rfind(scheme, 0) == 0) return true;
    return false;
}

std::string source_label(const SourceConfig& source)
{
    switch (source.mode) {
    case SourceMode::Video: return "video";
    case SourceMode::V4l2:  return source.v4l2_device;
    case SourceMode::Ros2:  return source.input_camera_topic;
    }
    return {};
}

Config load_vision_pilot_config()
{
    // Load default config
    auto kv = parse_conf(find_config("vision_pilot.conf"));
    Config cfg;

    cfg.engine.provider     = optional(kv, "engine.provider",     "cpu");
    cfg.engine.precision    = optional(kv, "model.precision",    "fp32");
    cfg.engine.device_id    = parse_int(optional(kv, "engine.device_id", "0"), "engine.device_id");
    cfg.engine.cache_dir    = expand_home(optional(kv, "engine.cache_dir", "../trt_cache"));
    cfg.engine.workspace_gb = parse_double(optional(kv, "engine.workspace_gb", "1.0"), "engine.workspace_gb");
    cfg.engine.gpu_mem_limit_gb = parse_double(
        optional(kv, "engine.gpu_mem_limit_gb", "0.0"), "engine.gpu_mem_limit_gb");

    cfg.inference.precision    = optional(kv, "model.precision",    "fp32");
    cfg.inference.precision_ad  = optional(kv, "model.precision.autodrive", "");
    cfg.inference.precision_as  = optional(kv, "model.precision.autosteer", "");
    cfg.inference.precision_asp = optional(kv, "model.precision.autospeed", "");
    cfg.inference.fusion_debug = parse_bool(optional(kv, "fusion.debug", "false"), "fusion.debug");
    cfg.inference.cte_bias_m   = static_cast<float>(
        parse_double(optional(kv, "fusion.cte_bias_m", "0.0"), "fusion.cte_bias_m"));

    cfg.source.mode          = parse_source_mode(optional(kv, "source.mode", "video"));

    cfg.source.v4l2_device   = optional(kv, "source.v4l2_device", "/dev/video0");
    cfg.source.v4l2_fps      = parse_int(optional(kv, "source.v4l2_fps", "10"), "source.v4l2_fps");
    // cfg.pipeline.initial_inference_check = parse_bool(
    //     optional(kv, "pipeline.initial_inference_check", "true"),
    //     "pipeline.initial_inference_check");

    cfg.fusion_debug = parse_bool(optional(kv, "fusion.debug", "false"), "fusion.debug");

    cfg.speed_limit = parse_double(optional(kv, "speed_limit", ""), "speed_limit");
    cfg.Lf = parse_double(optional(kv, "Lf", ""), "Lf");

    cfg.vehicle_speed_valid = parse_bool(
        optional(kv, "vehicle.speed_valid", "false"), "vehicle.speed_valid");

    cfg.visualization_on = parse_bool(optional(kv, "visualization_on", "false"), "visualization_on");
    cfg.webrtc_on = parse_bool(optional(kv, "webrtc_on", "false"), "webrtc_on");
    cfg.webrtc_port =  parse_int(optional(kv, "webrtc_port", "8080"), "webrtc_port");

    { const std::string raw = optional(kv, "debug.wheel_dir", "");
      cfg.wheel_dir = raw.empty() ? "" : expand_home(raw); }

    // ─── Navigation (offline OSM map panel) ──────────────────────────────────
    {
        navigation::Config& nav = cfg.navigation;
        nav.enabled  = parse_bool(optional(kv, "nav.enabled", "false"), "nav.enabled");
        nav.map_file = expand_home(optional(kv, "nav.map_file",
                                            "../maps/bandung-roads.vpmap"));
        nav.gps_source = navigation::parse_gps_source(
            optional(kv, "nav.gps_source", "ipwebcam"));
        // Left empty on purpose: filled in below from the camera stream URL.
        nav.gps_url     = optional(kv, "nav.gps_url", "");
        nav.gps_file    = expand_home(optional(kv, "nav.gps_file", ""));
        nav.gps_rate_hz = parse_double(optional(kv, "nav.gps_rate_hz", "2.0"),
                                       "nav.gps_rate_hz");
        // The Nucleo's ST-LINK port. Empty = the first /dev/ttyACM* found.
        nav.gps_port    = optional(kv, "nav.gps_port", "");
        nav.gps_baud    = parse_int(optional(kv, "nav.gps_baud", "115200"),
                                    "nav.gps_baud");

        nav.imu_enabled = parse_bool(optional(kv, "nav.imu_enabled", "false"),
                                     "nav.imu_enabled");
        nav.imu_port    = optional(kv, "nav.imu_port", "");
        nav.imu_baud    = parse_int(optional(kv, "nav.imu_baud", "0"), "nav.imu_baud");
        nav.compare_gps = parse_bool(optional(kv, "nav.compare_gps", "false"),
                                     "nav.compare_gps");
        parse_lat_lon(optional(kv, "nav.fixed_position", "-6.9175,107.6191"),
                      "nav.fixed_position", nav.fixed_lat, nav.fixed_lon);

        const std::string dest = optional(kv, "nav.destination", "");
        if (!dest.empty()) {
            parse_lat_lon(dest, "nav.destination", nav.dest_lat, nav.dest_lon);
            nav.has_destination = true;
            nav.dest_name = optional(kv, "nav.destination_name", "");
        }

        nav.panel_width = parse_int(optional(kv, "nav.panel_width", "512"),
                                    "nav.panel_width");
        // "WxH", or "0" / "none" to keep the plain side-by-side frame.
        {
            const std::string raw = optional(kv, "nav.output_size", "1920x1080");
            if (raw == "0" || raw == "none" || raw == "off") {
                nav.out_width = nav.out_height = 0;
            } else {
                const auto x = raw.find_first_of("xX");
                if (x == std::string::npos)
                    throw std::runtime_error("nav.output_size must be WxH, got: " + raw);
                nav.out_width  = parse_int(trim(raw.substr(0, x)), "nav.output_size");
                nav.out_height = parse_int(trim(raw.substr(x + 1)), "nav.output_size");
                if (nav.out_width < 320 || nav.out_height < 240)
                    throw std::runtime_error("nav.output_size must be at least 320x240");
            }
        }
        nav.zoom_m      = parse_double(optional(kv, "nav.zoom_m", "400"), "nav.zoom_m");
        nav.heading_up  = parse_bool(optional(kv, "nav.heading_up", "true"),
                                     "nav.heading_up");
        nav.dark        = parse_bool(optional(kv, "nav.dark", "true"), "nav.dark");
        if (nav.panel_width < 160)
            throw std::runtime_error("nav.panel_width must be at least 160 px");
        if (nav.zoom_m < 50.0)
            throw std::runtime_error("nav.zoom_m must be at least 50 m");
    }

    // Load ROS2 config config
#ifdef ENABLE_ROS2_INTERFACE
    kv = parse_conf(find_config("vision_pilot_ros2.conf"));
    cfg.source.input_camera_topic = optional(kv, "source.input_camera_topic",  "/camera/image");
    cfg.vehicle_speed_topic = optional(kv, "vehicle_speed_topic", "/vehicle/speed");
    cfg.vehicle_steering_topic = optional(kv, "vehicle_steering_topic", "/vehicle/steering_cmd");
    cfg.vehicle_acceleration_topic = optional(kv, "vehicle_acceleration_topic", "/vehicle/throttle_cmd");
#endif

    // Load test configuration
    if (cfg.source.mode == SourceMode::Video) {
        kv = parse_conf(find_config("vision_pilot_test.conf"));

        // Load input video config
        cfg.source.input_video = optional(kv, "source.input_video", "");
        if (cfg.source.input_video.empty())
            throw std::runtime_error("source.mode=video requires source.input_video");
        // A stream URL has nothing to stat — leave it to the camera interface to open.
        if (!is_stream_url(cfg.source.input_video) && !file_ok(cfg.source.input_video))
            throw std::runtime_error("source.video_path not found: " + cfg.source.input_video);
        // Load input vehicle speed config
        cfg.source.input_vehicle_speed = optional(kv, "source.input_vehicle_speed", "");
        if (cfg.source.input_vehicle_speed.empty())
            throw std::runtime_error("source.mode=video requires source.input_vehicle_speed");
        if (!file_ok(cfg.source.input_vehicle_speed))
            throw std::runtime_error("source.video_path not found: " + cfg.source.input_vehicle_speed);
        // Load dataset config
        cfg.source.dataset = optional(kv, "source.dataset", "");
        if (cfg.source.dataset.empty())
            throw std::runtime_error("source.mode=video requires source.dataset");

        cfg.source.video_realtime= parse_bool(optional(kv, "source.video_realtime", "true"), "source.video_realtime");
        cfg.source.video_loop    = parse_bool(optional(kv, "source.video_loop",     "false"), "source.video_loop");
    }

    // IP Webcam serves position and frames off one host and one port, so an
    // unset nav.gps_url just means "wherever the stream is". phyphox is a
    // separate app on its own port, so it always needs the URL spelled out.
    if (cfg.navigation.gps_url.empty() &&
        cfg.navigation.gps_source == navigation::GpsSource::IpWebcam &&
        is_stream_url(cfg.source.input_video))
        cfg.navigation.gps_url = url_origin(cfg.source.input_video);

    // A position source that cannot work here degrades navigation, it does not
    // stop VisionPilot: replaying a clip with the config still pointing at the
    // phone must not refuse to drive.
    if (cfg.navigation.enabled) {
        navigation::Config& nav = cfg.navigation;
        if (nav.gps_source == navigation::GpsSource::IpWebcam && nav.gps_url.empty()) {
            nav.degraded_reason =
                "nav.gps_source = ipwebcam but no nav.gps_url, and source.input_video "
                "is not a stream URL";
            nav.gps_source = navigation::GpsSource::None;
        } else if (nav.gps_source == navigation::GpsSource::Phyphox &&
                   nav.gps_url.empty()) {
            nav.degraded_reason =
                "nav.gps_source = phyphox needs nav.gps_url — phyphox is a separate "
                "app on its own port, e.g. http://172.27.74.24:8080";
            nav.gps_source = navigation::GpsSource::None;
        } else if (nav.gps_source == navigation::GpsSource::File &&
                   !file_ok(nav.gps_file)) {
            nav.degraded_reason = "nav.gps_file not found: " + nav.gps_file;
            nav.gps_source = navigation::GpsSource::None;
        }
    }

    return cfg;
}

std::string resolve_vision_pilot_config_path(int argc, char** argv)
{
    for (int i = 1; i + 1 < argc; ++i) {
        const std::string a = argv[i];
        if (a == "--config" || a == "-c") return argv[i + 1];
        const std::string pfx = "--config=";
        if (a.rfind(pfx, 0) == 0) return a.substr(pfx.size());
    }
    if (const char* e = std::getenv("VISIONPILOT_CONFIG"); e && file_ok(e)) return e;
    for (const char* c : {"config/vision_pilot.conf", "vision_pilot.conf"})
        if (file_ok(c)) return c;
    return {};
}
