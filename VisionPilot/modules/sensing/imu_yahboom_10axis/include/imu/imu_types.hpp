#ifndef IMU_YAHBOOM_IMU_TYPES_HPP
#define IMU_YAHBOOM_IMU_TYPES_HPP

#include <cstdint>
#include <string>

namespace imu {

constexpr double kGravity = 9.80665;              // m/s^2
constexpr double kDeg2Rad = 0.017453292519943295;
constexpr double kRad2Deg = 57.29577951308232;

struct Vec3
{
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;

    double norm() const;
};

// One reading of the whole unit. A 10-axis module reports accelerometer,
// gyroscope, magnetometer and barometer; each has_* flag says whether this
// particular sample actually carried that channel, because most protocols split
// the channels across separate frames arriving a few milliseconds apart.
//
// Axes are the ones silk-screened on the board, right-handed:
//   x forward (roll axis), y left (pitch axis), z up (yaw axis)
struct ImuSample
{
    Vec3 accel;              // m/s^2, gravity included
    Vec3 gyro;               // rad/s
    Vec3 mag;                // sensor units (uT on most parts, raw LSB on some)

    double roll_deg  = 0.0;  // attitude as reported by the device itself
    double pitch_deg = 0.0;
    double yaw_deg   = 0.0;

    double qw = 1.0, qx = 0.0, qy = 0.0, qz = 0.0;

    double temperature_c = 0.0;
    double pressure_pa   = 0.0;
    double altitude_m    = 0.0;

    uint64_t stamp_us = 0;   // steady clock, host side

    bool has_accel = false;
    bool has_gyro  = false;
    bool has_mag   = false;
    bool has_euler = false;
    bool has_quat  = false;
    bool has_temp  = false;
    bool has_baro  = false;

    bool empty() const
    {
        return !(has_accel || has_gyro || has_mag || has_euler || has_quat ||
                 has_temp || has_baro);
    }
};

// Wire formats this driver understands. Unknown means bytes are arriving but
// nothing has decoded them yet — `imu_monitor --raw` prints the dump needed to
// add the missing one.
enum class Protocol
{
    Unknown,
    Yahboom,     // 0x7E 0x23 <len> <id> ... <sum> — what the 10-axis board sends
    WitMotion,   // 0x55 <type> <8 bytes> <sum> — WT901/HWT906/JY901 and clones
    Yesense,     // 0x59 0x53 ... TLV payload — YIS100 family
    Ascii,       // printable lines: "#YPR=..", key=value, or plain CSV
};

const char* protocol_name(Protocol p);

// Health of the byte stream. Any decoder failure shows up here rather than as a
// silent gap in the readings, so a wrong baud rate is obvious instead of just
// looking like a dead sensor.
struct StreamStats
{
    uint64_t bytes        = 0;
    uint64_t frames_ok    = 0;
    uint64_t frames_bad   = 0;   // checksum rejected
    uint64_t bytes_junk   = 0;   // skipped while hunting for a header
    uint64_t samples      = 0;
    double   sample_hz    = 0.0;
};

std::string format_stats(const StreamStats& s);

}  // namespace imu

#endif  // IMU_YAHBOOM_IMU_TYPES_HPP
