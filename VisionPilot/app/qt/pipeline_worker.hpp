#ifndef VISIONPILOT_QT_PIPELINE_WORKER_HPP
#define VISIONPILOT_QT_PIPELINE_WORKER_HPP

#include <atomic>

#include <QImage>
#include <QObject>

#include <opencv2/core.hpp>

#include <config/vision_pilot_config.hpp>

namespace visionpilot::ui {

// Runs camera → preprocess → inference → planner → HUD on its own thread and
// hands finished frames to the UI.
//
// The whole point of moving it off the GUI thread: the models take ~25 ms a
// frame, and while they run the window has to stay alive — typing in the search
// bar, panning the map, pressing a button. Under the old OpenCV loop those
// shared one thread, which is why the map only moved 25 times a second.
class PipelineWorker : public QObject
{
    Q_OBJECT

public:
    explicit PipelineWorker(Config cfg, bool debug_viz = false);
    ~PipelineWorker() override;

    // Speed limit for the HUD's MAX sign, km/h, 0 to hide it. Written by the
    // GUI thread from the map match, read by the worker as it draws.
    void set_speed_limit_kph(int kph) { speed_limit_kph_ = kph; }

public slots:
    void run();     // blocks until stop() — invoke on the worker thread
    void stop();

signals:
    // The HUD frame, already drawn. Queued to the GUI thread, so it must own
    // its pixels: QImage is copied before it is emitted.
    void frameReady(const QImage& frame);
    // Vehicle speed in m/s and whether it came from a real source.
    void telemetry(double speed_ms, bool speed_valid);
    void failed(const QString& reason);
    void finished();

private:
    Config            cfg_;
    bool              debug_viz_;
    std::atomic<bool> running_{true};
    std::atomic<int>  speed_limit_kph_{0};
};

// BGR cv::Mat → RGB QImage, deep-copied so it can cross threads safely.
QImage mat_to_qimage(const cv::Mat& bgr);

}  // namespace visionpilot::ui

#endif  // VISIONPILOT_QT_PIPELINE_WORKER_HPP
