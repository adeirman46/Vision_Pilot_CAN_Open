#include "camera_interface/file_interface.hpp"

#include <thread>

#include <config/vision_pilot_config.hpp>
#include <logging/logger.hpp>

namespace camera_interface {

namespace {
// A live source drops out when the network does. Retry for a while before calling it
// dead, so a Wi-Fi blip does not end the run.
constexpr int  kStreamRetries = 10;
constexpr auto kStreamRetryDelay = std::chrono::seconds(1);
}  // namespace

FileInterface::FileInterface(const std::string& path, bool loop, bool realtime)
    : path_(path), loop_(loop), realtime_(realtime), stream_(is_stream_url(path))
{
    cap_.open(path);
    if (!cap_.isOpened()) {
        return;
    }

    // A live stream already arrives at its own rate; sleeping to its nominal frame
    // rate on top of that only backs frames up in the decoder queue and adds lag.
    const double fps = cap_.get(cv::CAP_PROP_FPS);
    if (realtime_ && !stream_ && fps > 1.0) {
        frame_period_ = std::chrono::duration<double>(1.0 / fps);
    }
}

bool FileInterface::is_device_open() const
{
    return cap_.isOpened();
}

bool FileInterface::reopen_stream()
{
    for (int attempt = 1; attempt <= kStreamRetries; ++attempt) {
        VP_WARN("stream stalled — reconnecting to %s (%d/%d)",
                path_.c_str(), attempt, kStreamRetries);
        std::this_thread::sleep_for(kStreamRetryDelay);
        cap_.release();
        if (cap_.open(path_)) {
            return true;
        }
    }
    // Give up: leave the capture closed so is_device_open() reports the source gone.
    cap_.release();
    VP_ERROR("stream %s did not come back after %d attempts", path_.c_str(), kStreamRetries);
    return false;
}

std::tuple<bool, cv::Mat> FileInterface::get_latest_frame()
{
    const auto t0 = std::chrono::steady_clock::now();

    cv::Mat frame;
    if (!cap_.read(frame) || frame.empty()) {
        if (stream_) {
            if (!reopen_stream() || !cap_.read(frame) || frame.empty()) {
                return {false, {}};
            }
        } else if (!loop_) {
            return {false, {}};
        } else {
            cap_.set(cv::CAP_PROP_POS_FRAMES, 0);
            if (!cap_.read(frame) || frame.empty()) {
                return {false, {}};
            }
        }
    }

    if (frame_period_.count() > 0) {
        const auto elapsed = std::chrono::steady_clock::now() - t0;
        const auto rem = frame_period_ - elapsed;
        if (rem.count() > 0) {
            std::this_thread::sleep_for(rem);
        }
    }

    return {true, frame};
}

std::vector<std::string> FileInterface::get_overlay() const
{
    return {std::string(stream_ ? "stream: " : "video: ") + path_};
}

}  // namespace camera_interface
