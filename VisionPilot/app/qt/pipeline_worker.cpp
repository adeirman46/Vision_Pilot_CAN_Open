#include "pipeline_worker.hpp"

#include <chrono>
#include <memory>
#include <thread>

#include <opencv2/imgproc.hpp>

#include <common/utils.hpp>
#include <debug/debug_draw.hpp>
#include <engine/onnx_engine.hpp>
#include <image_preprocessing/image_preprocessor.hpp>
#include <logging/logger.hpp>
#include <models/inference.hpp>
#include <planning/planning.hpp>
#include <visualization/visualization.hpp>

#include "camera_interface/file_interface.hpp"
#include "camera_interface/v4l2_camera_interface.hpp"
#include "vehicle_interface/can_interface.hpp"
#include "vehicle_interface/file_interface.hpp"

namespace ve = visionpilot::engine;
namespace vm = visionpilot::models;
namespace vd = visionpilot::debug;

namespace visionpilot::ui {

QImage mat_to_qimage(const cv::Mat& bgr)
{
    if (bgr.empty()) return {};
    cv::Mat rgb;
    cv::cvtColor(bgr, rgb, cv::COLOR_BGR2RGB);
    // copy(): the cv::Mat dies with this scope, and the image is about to be
    // queued to another thread.
    return QImage(rgb.data, rgb.cols, rgb.rows,
                  static_cast<int>(rgb.step), QImage::Format_RGB888)
        .copy();
}

PipelineWorker::PipelineWorker(Config cfg, bool debug_viz)
    : cfg_(std::move(cfg)), debug_viz_(debug_viz)
{}

PipelineWorker::~PipelineWorker() = default;

void PipelineWorker::stop() { running_ = false; }

void PipelineWorker::run()
{
    std::shared_ptr<CameraInterface>  camera;
    std::shared_ptr<VehicleInterface> vehicle;

    if (cfg_.source.mode == SourceMode::Video) {
        camera = std::make_shared<camera_interface::FileInterface>(
            cfg_.source.input_video, cfg_.source.video_loop, cfg_.source.video_realtime);
        vehicle = std::make_shared<FileInterface>(cfg_.source.input_vehicle_speed);
    } else {
        camera = std::make_shared<camera_interface::V4L2CameraInterface>(
            cfg_.source.v4l2_device, static_cast<uint32_t>(cfg_.source.v4l2_fps));
        vehicle = std::make_shared<CanInterface>();
    }

    if (!camera || !camera->is_device_open()) {
        emit failed(QStringLiteral("Cannot open the camera source"));
        emit finished();
        return;
    }

    ImagePreprocessor preprocessor;
    ve::OnnxEngine engine(cfg_.engine);
    vm::InferencePipeline pipeline(engine, cfg_.inference);
    Planner planner(cfg_.speed_limit, cfg_.Lf);

    if (debug_viz_) {
        vd::init_wheel_assets(cfg_.wheel_dir);
        vd::init_homography();
    } else {
        visualization::init_production_assets();
    }

    const cv::Size net_size(vm::AutoDrive::NET_W, vm::AutoDrive::NET_H);
    cv::Mat warped, resized;
    bool h_set = false;
    const cv::Mat H = load_matrix("H.yaml", "H");

    while (running_) {
        auto [ok, frame] = camera->get_latest_frame();
        if (!ok || frame.empty()) {
            if (!camera->is_device_open()) {
                emit failed(QStringLiteral("Frame source closed"));
                break;
            }
            if (cfg_.source.mode == SourceMode::Video && !cfg_.source.video_loop
                && !is_stream_url(cfg_.source.input_video))
                break;
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
            continue;
        }

        preprocessor.preprocess(frame, warped, resized, net_size);
        if (!h_set) {
            pipeline.set_H_resized(H, frame.size());
            h_set = true;
        }

        cv::Mat display = resized;
        double ego_v = 0.0;

        if (const auto r = pipeline.process(warped, resized)) {
            ego_v = vehicle->read();

            static constexpr double D_MAX = 150.0;
            const bool has_cipo = r->cipo.valid && r->cipo.distance_m < D_MAX;
            const Plan plan = planner.compute_plan(
                r->lateral.cte_m, r->lateral.yaw_rad, r->lateral.curvature, ego_v,
                has_cipo, has_cipo ? r->cipo.velocity_ms : cfg_.speed_limit,
                r->cipo.distance_m);

            vehicle->write(plan.steering.empty() ? 0.0 : plan.steering[1],
                           plan.acceleration);

            if (debug_viz_)
                vd::visualize(resized, *r, source_label(cfg_.source), cfg_.wheel_dir,
                              pipeline.H_world2resized());
            else
            {
                // The limit comes from the map match the GUI thread keeps
                // up to date; 0 hides the MAX sign entirely.
                const int kph = speed_limit_kph_.load();
                display = visualization::ProductionView::visualize(
                    resized, *r, plan, ego_v, pipeline.H_resized(),
                    kph > 0 ? kph / 3.6 : 0.0, cfg_.vehicle_speed_valid);
            }
        }

        emit frameReady(mat_to_qimage(display));
        emit telemetry(ego_v, cfg_.vehicle_speed_valid);
    }

    emit finished();
}

}  // namespace visionpilot::ui
