#ifndef VISIONPILOT_VIDEO_FILE_INTERFACE_HPP
#define VISIONPILOT_VIDEO_FILE_INTERFACE_HPP

#include <atomic>
#include <camera_interface/camera_interface.hpp>
#include <chrono>
#include <mutex>
#include <opencv2/videoio.hpp>
#include <string>
#include <thread>

namespace camera_interface {

class FileInterface : public CameraInterface {
public:
    FileInterface(const std::string& path, bool loop, bool realtime);
    ~FileInterface() override;

    bool is_device_open() const override;
    std::tuple<bool, cv::Mat> get_latest_frame() override;
    std::vector<std::string> get_overlay() const override;

private:
    // Reconnect a dropped live stream. Returns false once the retries are spent,
    // leaving cap_ closed.
    bool reopen_stream();

    // Live streams only: pull frames as fast as they arrive and keep just the
    // newest. See the note in the constructor.
    void grab_loop();

    std::string path_;
    bool loop_;
    bool realtime_;
    bool stream_;
    cv::VideoCapture cap_;
    std::chrono::duration<double> frame_period_{0};

    std::thread        grabber_;
    mutable std::mutex frame_mtx_;
    cv::Mat            latest_;
    std::atomic<bool>  grabbing_{false};
    std::atomic<bool>  stream_dead_{false};
};

}  // namespace camera_interface

#endif  // VISIONPILOT_VIDEO_FILE_INTERFACE_HPP
