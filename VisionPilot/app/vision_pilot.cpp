// VisionPilot — preprocess → inference → fusion → display
#include <chrono>
#include <memory>
#include <string>
#include <thread>

#include <config/vision_pilot_config.hpp>
#include <common/utils.hpp>
#include <engine/onnx_engine.hpp>
#include <vehicle_interface/vehicle_interface.hpp>
#include <vehicle_interface/can_interface.hpp>
#include <image_preprocessing/image_preprocessor.hpp>
#include <logging/logger.hpp>
#include <models/inference.hpp>
#include <navigation/navigation.hpp>
#include <planning/planning.hpp>
#include <visualization/visualization.hpp>
#include <debug/debug_draw.hpp>

#include "camera_interface/v4l2_camera_interface.hpp"
#include "camera_interface/file_interface.hpp"
#include "vehicle_interface/file_interface.hpp"

#if ENABLE_ROS2_INTERFACE
#include <rclcpp/rclcpp.hpp>
#include <camera_ros2_interface/camera_ros2_interface.hpp>
#include <vehicle_ros2_interface/vehicle_ros2_interface.hpp>
#endif

namespace ve = visionpilot::engine;
namespace vm = visionpilot::models;
namespace vd = visionpilot::debug;

int main(int argc, char** argv)
{
    Config cfg;
    try { cfg = load_vision_pilot_config(); }
    catch (const std::exception& e)
    {
        VP_ERROR("Config: %s", e.what());
        return 1;
    }

    // ── CLI flags ─────────────────────────────────────────────────────────────
    bool show_window = true;
    bool debug_viz = false;
    for (int i = 1; i < argc; ++i)
    {
        const std::string arg(argv[i]);
        if (arg == "--debug-viz") debug_viz = true;
        else if (arg == "--no-window") show_window = false;
        else if (arg == "--nav") cfg.navigation.enabled = true;
        else if (arg == "--no-nav") cfg.navigation.enabled = false;
        else if (arg == "--guidance") cfg.guidance.enabled = true;
        else if (arg == "--no-guidance") cfg.guidance.enabled = false;
    }

    std::shared_ptr<CameraInterface> camera_interface;
    std::shared_ptr<VehicleInterface> vehicle_interface;
#if ENABLE_ROS2_INTERFACE
    rclcpp::init(argc, argv);
    camera_interface = std::make_unique<CameraRos2Interface>(cfg.source.input_camera_topic);
    vehicle_interface = std::make_shared<VehicleRos2Interface>(cfg.vehicle_speed_topic,
                                                               cfg.vehicle_steering_topic,
                                                               cfg.vehicle_acceleration_topic);
#else
    if (cfg.source.mode == SourceMode::Video)
    {
        camera_interface = std::make_unique<camera_interface::FileInterface>(
            cfg.source.input_video, cfg.source.video_loop, cfg.source.video_realtime);
        vehicle_interface = std::make_shared<FileInterface>(cfg.source.input_vehicle_speed);
    }
    else
    {
        camera_interface = std::make_unique<camera_interface::V4L2CameraInterface>(
            cfg.source.v4l2_device, static_cast<uint32_t>(cfg.source.v4l2_fps));
        vehicle_interface = std::make_shared<CanInterface>();
    }
#endif

    ImagePreprocessor preprocessor;
    ve::OnnxEngine engine(cfg.engine);
    vm::InferencePipeline pipeline(engine, cfg.inference);
    Planner planner(cfg.speed_limit, cfg.Lf);

    // ── Init visualization assets once based on mode ──────────────────────────
    if (debug_viz)
    {
        VP_INFO("[Viz] Debug mode — annotated telemetry overlay");
        vd::init_wheel_assets(cfg.wheel_dir);
        vd::init_homography();
    }
    else
    {
        VP_INFO("[Viz] Production mode — clean HUD");
        visualization::init_production_assets();
    }

    // ── Initialize camera interface ───────────────────────────────────────────

    if (!camera_interface || !camera_interface->is_device_open())
    {
        VP_ERROR("Cannot open frame source");
        return 1;
    }

    // ── Initialize navigation ─────────────────────────────────────────────────
    // Offline OSM panel drawn to the right of the HUD. It never gates the drive:
    // a missing map or a phone with no fix only changes what the panel says.
    navigation::Navigator navigator(cfg.navigation);
    if (cfg.navigation.enabled)
    {
        if (navigator.ready())
            VP_INFO("[Nav] enabled — %d px panel, %.0f m across",
                    cfg.navigation.panel_width, cfg.navigation.zoom_m);
        else
            VP_WARN("[Nav] enabled but degraded: %s", navigator.error().c_str());
    }

    // ── Route corridor ────────────────────────────────────────────────────────
    // The route filtering the vision waypoints. It needs a route to filter
    // against, so with no destination set it stays inert and the path fit
    // behaves exactly as it always has.
    if (cfg.guidance.enabled)
    {
        if (!cfg.navigation.enabled)
        {
            VP_WARN("[Guidance] asked for, but nav is off — no route to follow. Disabled.");
            cfg.guidance.enabled = false;
        }
        else
        {
            VP_INFO("[Guidance] route corridor on — %.1f m wide, %.0f m lookahead, "
                    "%d waypoints minimum",
                    static_cast<double>(cfg.inference.route_corridor_m) * 2.0,
                    cfg.guidance.lookahead_m,
                    cfg.inference.route_min_pts);
            if (!cfg.navigation.has_destination)
                VP_WARN("[Guidance] no destination set — the corridor stays off "
                        "until one is picked (tap the map, or press 'd')");
        }
    }

    // ── Initialize display ────────────────────────────────────────────────────
    visualization::Visualization visualization({cfg.webrtc_on, cfg.webrtc_port, show_window});

    const cv::Size net_size(vm::AutoDrive::NET_W, vm::AutoDrive::NET_H);
    cv::Mat frame, warped, resized;
    bool h_resized_set = false;
    cv::Mat H = load_matrix("H.yaml", "H");
    while (true)
    {
        auto [ok, frame] = camera_interface->get_latest_frame();
        if (!ok || frame.empty())
        {
            // The source closed itself: a live stream that never came back.
            if (!camera_interface->is_device_open())
            {
                VP_ERROR("Frame source closed — stopping");
                break;
            }
            // A file that ran out of frames is done; a live stream just dropped one.
            if (cfg.source.mode == SourceMode::Video && !cfg.source.video_loop
                && !is_stream_url(cfg.source.input_video))
                break;
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            continue;
        }

        preprocessor.preprocess(frame, warped, resized, net_size);
        cv::Size frame_size = frame.size();
        // One-time: tell the pipeline how to project AutoSteer/AutoSpeed outputs
        // back to world when those networks run on the plain-resized image.
        if (!h_resized_set)
        {
            pipeline.set_H_resized(H, frame_size);
            h_resized_set = true;
        }

        // ── Default frame no inference ────────────────────────────────────────────
        cv::Mat display_frame = resized;

        // ── Navigation ────────────────────────────────────────────────────────
        // Ahead of inference, not after it. The route corridor filters the very
        // waypoints this frame's path fit is about to be built from, so it has
        // to be drawn around this frame's position rather than the last one's.
        // The panel below is redrawn from the same update, so nothing is done
        // twice.
        navigator.update();
        if (cfg.guidance.enabled) {
            pipeline.set_route_ego(navigator.route_ahead_ego(cfg.guidance.lookahead_m));
            const auto turn = navigator.next_maneuver();
            pipeline.set_maneuver(turn.valid, static_cast<float>(turn.dist_m),
                                  static_cast<float>(turn.bend_deg));
        }

        if (const auto r = pipeline.process(warped, resized))
        {
            pipeline.latency().print();

            const double ego_v = vehicle_interface->read();
            const double cte = r->lateral.cte_m;
            const double epsi = r->lateral.yaw_rad;
            const double kappa = r->lateral.curvature;

            // has_cipo: tracker-based — true only when filter tracks a target
            // closer than D_MAX. cipo_raw_found alone must not gate the planner.
            static constexpr double D_MAX = 150.0;
            const bool has_cipo = r->cipo.valid && r->cipo.distance_m < D_MAX;
            const double cipo_v = has_cipo ? r->cipo.velocity_ms : cfg.speed_limit;
            const double cipo_dist = r->cipo.distance_m;

            const double raw_cte = r->lateral.path_valid
                                       ? static_cast<double>(r->lateral.raw_cte_m)
                                       : cte;
            const Plan plan = planner.compute_plan(
                cte, epsi, kappa, ego_v, has_cipo, cipo_v, cipo_dist);

            VP_INFO(
                "plan: tyre=%.4f rad  accel=%.3f m/s²  |  cte=%.2fm(raw=%.2fm)  |  cipo=%s  dist=%.1f m  vel=%+.2f m/s",
                plan.steering.empty() ? 0.0 : plan.steering[1],
                plan.acceleration,
                cte,
                raw_cte,
                has_cipo ? "true" : "false",
                cipo_dist,
                r->cipo.velocity_ms);

            // The corridor's own line, so it can be judged from a console with
            // no window attached — which is how a headless drive gets reviewed.
            if (r->lateral.route_active || r->lateral.route_weight > 0.f)
                VP_INFO("route: turn %+.0f deg in %.0f m  map=%.0f%%  "
                        "kappa_route=%+.4f  kappa_out=%+.4f  |  corridor %d/%d kept%s",
                        static_cast<double>(r->lateral.route_bend_deg),
                        static_cast<double>(r->lateral.route_dist_m),
                        static_cast<double>(r->lateral.route_weight) * 100.0,
                        static_cast<double>(r->lateral.route_kappa),
                        static_cast<double>(r->lateral.curvature),
                        r->lateral.route_pts_kept, r->lateral.route_pts_in,
                        r->lateral.route_fallback ? "  (corridor stood down)" : "");

            vehicle_interface->write(
                plan.steering.empty() ? 0.0 : plan.steering[1],
                plan.acceleration);

            if (cfg.visualization_on)
            {
                if (debug_viz)
                    vd::visualize(resized, *r, source_label(cfg.source), cfg.wheel_dir, pipeline.H_world2resized());
                else
                {
                    // Speed limit comes from the map's road match, not the
                    // planner's global cap: "MAX 120" on a 30 km/h kampung
                    // street is worse than showing nothing.
                    const int map_kph = navigator.speed_limit_kph();
                    const double limit_ms = map_kph > 0 ? map_kph / 3.6 : 0.0;
                    // Vehicle speed is synthesised until CAN is wired up; the
                    // HUD shows a dash rather than dressing it up as a reading.
                    display_frame = visualization.build_frame(
                        resized, *r, plan, ego_v, pipeline.H_resized(), limit_ms,
                        cfg.vehicle_speed_valid);
                }
            }
        }
        if (cfg.visualization_on)
        {
            // Map to the right of the inference view — one composed frame, so the
            // local window and the WebRTC stream both carry it.
            visualization.render_frame(navigator.compose(display_frame));
            // Keys typed into the video window drive the destination box —
            // press 'd' to open it, Esc to close, 'c' to clear the route.
            navigator.handle_key(visualization.last_key());
            // Wheel zooms the map. Dragging is deliberately not offered: the
            // panel can only repaint as fast as inference produces frames, and
            // a map that follows the mouse in 40 ms steps feels broken. Zoom
            // is a discrete action, so it does not suffer the same way.
            const auto m = visualization.mouse();
            navigator.handle_mouse(m.x, m.y, false, m.wheel);
            // A click on the map is "go there" — the destination a driver can
            // point at but cannot name is the normal one, and typing a street
            // into a box while moving is not an option.
            // Shift-click appends to the mission, a plain click retargets —
            // the same gesture the Qt window uses, so the two front-ends do not
            // have to be learned separately.
            if (m.clicks > 0) {
                if (m.shift_click) navigator.add_waypoint_at_canvas_px(m.x, m.y);
                else               navigator.set_destination_at_canvas_px(m.x, m.y);
            }

        }
    }

    // stop() returns true on a clean shutdown; translate that to a 0 exit code
    // so VisionPilot can be supervised as a batch/oneshot job (a successful run
    // must not exit non-zero).
    return visualization.stop() ? 0 : 1;
}
