#ifndef VP_STM32_IMU_MRA02_HPP
#define VP_STM32_IMU_MRA02_HPP

#include <stdint.h>
#include <stddef.h>

/*!
 * @file  imu_mra02.hpp
 * @brief Streaming decoder for the Yahboom MRA02-V1.0 10-axis IMU.
 *
 *     0x7E 0x23 | len | id | payload | checksum
 *
 * `len` counts the whole frame including the header and the checksum; the
 * checksum is the low byte of the sum of everything before it. Four frames make
 * one reading, sent back to back at 25 Hz on the factory settings:
 *
 *   id 0x04, 18 B   9 x int16   accel, gyro, magnetometer
 *   id 0x16, 16 B   4 x float   quaternion w x y z
 *   id 0x26, 12 B   3 x float   euler roll pitch yaw, in RADIANS
 *   id 0x32, 16 B   4 x float   altitude m, temperature C, pressure Pa, ref Pa
 *
 * This is not the protocol in Yahboom's published "10-axis IMU module
 * communication protocol" PDF — that document describes an older module using
 * WitMotion-style 0x55 framing, and no MRA02 spec exists online. The format here
 * was reverse-engineered from a live capture and is the same one the host
 * decoder uses (modules/sensing/imu_yahboom_10axis/src/imu_decoder.cpp), where
 * it is cross-checked three ways: the quaternion decodes to exactly the Euler
 * angles in id 0x26, and the accelerometer vector matches the attitude those
 * angles describe to three decimal places.
 *
 * Everything here is little-endian, as every part in this family is, and the
 * MCU is little-endian too — so the floats can be memcpy'd straight in.
 */

struct ImuSample
{
    // Body frame, as the module is mounted: x forward, y left, z up.
    double ax = 0.0, ay = 0.0, az = 0.0;      // m/s^2, gravity still included
    double gx = 0.0, gy = 0.0, gz = 0.0;      // rad/s
    double qw = 1.0, qx = 0.0, qy = 0.0, qz = 0.0;   // body -> world
    double roll_rad = 0.0, pitch_rad = 0.0, yaw_rad = 0.0;

    bool has_accel = false;
    bool has_gyro  = false;
    bool has_quat  = false;
    bool has_euler = false;

    uint32_t stamp_ms = 0;

    bool empty() const { return !has_accel && !has_gyro && !has_quat && !has_euler; }
};

class Mra02Decoder
{
public:
    // Feed whatever the UART returned; call pop() until it comes back false.
    void feed(const uint8_t* data, size_t n);
    bool pop(ImuSample& out);

    void reset();

    uint32_t frames_ok() const { return frames_ok_; }
    uint32_t frames_bad() const { return frames_bad_; }
    uint32_t bytes_junk() const { return bytes_junk_; }

private:
    void store_channel(uint8_t bit);   // a channel seen twice ends the reading
    void flush();

    // Four frames of at most ~24 B each, so one full reading always fits with
    // room for a partial frame on either side of it.
    static const size_t kBufMax = 192;

    uint8_t buf_[kBufMax];
    size_t  len_ = 0;

    ImuSample working_;
    uint8_t   seen_mask_ = 0;

    // One completed reading, waiting to be popped. The consumer runs every few
    // milliseconds against a 25 Hz sensor, so a queue would only ever hold one.
    ImuSample ready_;
    bool      have_ready_ = false;

    uint32_t frames_ok_  = 0;
    uint32_t frames_bad_ = 0;
    uint32_t bytes_junk_ = 0;
};

#endif  // VP_STM32_IMU_MRA02_HPP
