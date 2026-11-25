#ifndef VISIONPILOT_QT_PIPELINE_WORKER_HPP
#define VISIONPILOT_QT_PIPELINE_WORKER_HPP

#include <atomic>
#include <mutex>
#include <vector>

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

    // Route ahead in the ego frame, for the route corridor. Same arrangement as
    // the speed limit above and for the same reason: the Navigator lives on the
    // GUI thread, and this is what the worker is allowed to see of it. A vector
    // cannot be an atomic, so it goes under a mutex — written once per tick,
    // read once per inference frame, never contended for longer than a copy.
    void set_route_ego(const std::vector<cv::Point2f>& route);
    // The next turn the route asks for. Plain scalars, so unlike the route
    // above they need no lock.
    void set_maneuver(bool valid, float dist_m, float bend_deg)
    { man_valid_ = valid; man_dist_m_ = dist_m; man_bend_deg_ = bend_deg; }

    // The newest frame, whatever the GUI last managed to draw. Handing the
    // image over through a slot instead of through the signal is what keeps the
    // view live: see the note on frameReady().
    QImage takeFrame();

public slots:
    void run();     // blocks until stop() — invoke on the worker thread
    void stop();

signals:
    // "A new frame exists", carrying nothing. A queued signal with the image
    // attached looks simpler but is why the view ran behind: every frame the
    // pipeline produced became an event, the GUI thread could not scale them
    // that fast, and the backlog — and with it the delay — grew without limit.
    // Only ever one of these is outstanding, and it always refers to the newest
    // frame, so the picture on screen is at most one frame old however slow the
    // GUI gets.
    void frameReady();
    // Vehicle speed in m/s and whether it came from a real source.
    void telemetry(double speed_ms, bool speed_valid);
    void failed(const QString& reason);
    void finished();

private:
    Config            cfg_;
    bool              debug_viz_;
    std::atomic<bool> running_{true};
    std::atomic<int>  speed_limit_kph_{0};

    std::mutex               route_mtx_;
    std::vector<cv::Point2f> route_ego_;
    std::atomic<bool>        man_valid_{false};
    std::atomic<float>       man_dist_m_{0.f};
    std::atomic<float>       man_bend_deg_{0.f};

    std::mutex        frame_mtx_;
    QImage            latest_;
    std::atomic<bool> frame_pending_{false};
};

// BGR cv::Mat → RGB QImage, deep-copied so it can cross threads safely.
QImage mat_to_qimage(const cv::Mat& bgr);

}  // namespace visionpilot::ui

#endif  // VISIONPILOT_QT_PIPELINE_WORKER_HPP
