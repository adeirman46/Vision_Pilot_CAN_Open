#ifndef IMU_YAHBOOM_IMU_READER_HPP
#define IMU_YAHBOOM_IMU_READER_HPP

#include <atomic>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

#include <imu/attitude.hpp>
#include <imu/imu_decoder.hpp>
#include <imu/imu_types.hpp>
#include <imu/serial_port.hpp>

namespace imu {

struct ReaderConfig
{
    std::string device;                  // empty = pick the first USB serial port
    int         baud     = 0;            // 0 = scan the candidates
    Protocol    protocol = Protocol::Unknown;   // Unknown = auto-detect
    AttitudeEstimator::Options attitude{};
};

// Everything the UI needs for one repaint, copied out under the lock in one go
// so the numbers on screen always belong to the same instant.
struct ImuState
{
    ImuSample   sample;
    Vec3        euler_deg{};             // fused attitude (roll, pitch, yaw)
    Quat        orientation{};
    Vec3        gyro_bias_dps{};
    Vec3        linear_accel{};
    Vec3        velocity{};
    Vec3        position{};
    double      heading_deg   = 0.0;
    bool        calibrating   = false;
    double      cal_progress  = 0.0;
    bool        still         = false;
    bool        mag_yaw       = false;
    bool        connected     = false;
    Protocol    protocol      = Protocol::Unknown;
    StreamStats stats{};
    std::string device;
    int         baud = 0;
    std::string status;                  // one line, ready to print
    uint64_t    age_ms = 0;              // since the last decoded sample
};

// Opens the port, finds the baud rate and the wire format, then keeps a reader
// thread pulling samples and folding them into the attitude estimate. Everything
// the caller touches is a snapshot, so the GUI never blocks on the serial line.
class ImuReader
{
public:
    explicit ImuReader(const ReaderConfig& cfg);
    ~ImuReader();

    ImuReader(const ImuReader&)            = delete;
    ImuReader& operator=(const ImuReader&) = delete;

    bool     start(std::string& error);
    void     stop();
    ImuState state() const;

    void zero_here();
    void reset_position();
    void recalibrate();

    // Port + format discovery on its own, for the tools that report it before
    // deciding to stream. Returns false with `error` set when nothing answers.
    static bool probe(const std::string& device, int baud_hint, int& baud_out,
                      Protocol& proto_out, std::string& error);

private:
    void run();
    bool open_and_detect(std::string& error);

    ReaderConfig               cfg_;
    SerialPort                 port_;
    ImuDecoder                 decoder_;
    mutable std::mutex         mtx_;
    std::unique_ptr<AttitudeEstimator> att_;
    ImuState                   state_;
    std::thread                thread_;
    std::atomic<bool>          running_{false};
    std::atomic<bool>          want_zero_{false};
    std::atomic<bool>          want_reset_pos_{false};
    std::atomic<bool>          want_recal_{false};
    uint64_t                   last_sample_us_ = 0;
};

// Turns the usual failures — no port, no permission, driver not bound — into
// the exact command that fixes them.
std::string port_diagnosis(const std::string& device);

}  // namespace imu

#endif  // IMU_YAHBOOM_IMU_READER_HPP
