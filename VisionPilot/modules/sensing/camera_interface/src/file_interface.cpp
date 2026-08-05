#include "camera_interface/file_interface.hpp"

#include <cstdlib>
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
    if (stream_) {
        // FFmpeg buffers a live stream before OpenCV ever sees it, and that
        // buffer is where seconds of delay hide: draining frames faster on our
        // side cannot help, because the queue refills at the rate the phone
        // sends. It has to be told not to build one.
        //
        // OpenCV exposes no API for this, only an environment variable read when
        // the capture is opened, so it is set here rather than in a launcher —
        // the setting belongs with the code that depends on it.
        //   nobuffer/low_delay  hand each frame over as it decodes
        //   max_delay 0         do not wait to reorder
        //   probesize/analyzeduration  stop sampling the stream before starting
        setenv("OPENCV_FFMPEG_CAPTURE_OPTIONS",
               "fflags;nobuffer|flags;low_delay|max_delay;0|reorder_queue_size;0"
               "|probesize;32|analyzeduration;0",
               1);
    }

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

    if (stream_) {
        // A phone sends about 30 frames a second; the models consume rather
        // fewer. read() hands over the *oldest* frame still queued in the
        // decoder, so every frame the pipeline cannot keep up with adds a
        // permanent frame of lag — the picture falls further behind the longer
        // the run goes on, which is why the view ends up seconds late.
        //
        // Draining the queue on a thread of its own and keeping only the newest
        // frame fixes it at the source: the pipeline is then always handed the
        // present, and frames it had no time for are dropped instead of stored.
        cap_.set(cv::CAP_PROP_BUFFERSIZE, 1);   // honoured by some backends only
        grabbing_ = true;
        grabber_  = std::thread(&FileInterface::grab_loop, this);
    }
}

FileInterface::~FileInterface()
{
    grabbing_ = false;
    if (grabber_.joinable()) grabber_.join();
}

void FileInterface::grab_loop()
{
    while (grabbing_) {
        cv::Mat frame;
        if (!cap_.read(frame) || frame.empty()) {
            if (!reopen_stream()) {
                stream_dead_ = true;
                return;
            }
            continue;
        }
        std::lock_guard<std::mutex> lock(frame_mtx_);
        latest_ = std::move(frame);
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

    if (stream_) {
        // Whatever arrived most recently, however many were dropped meanwhile.
        for (int wait = 0; wait < 100; ++wait) {
            {
                std::lock_guard<std::mutex> lock(frame_mtx_);
                if (!latest_.empty()) return {true, latest_.clone()};
            }
            if (stream_dead_) return {false, {}};
            std::this_thread::sleep_for(std::chrono::milliseconds(5));
        }
        return {false, {}};
    }

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
