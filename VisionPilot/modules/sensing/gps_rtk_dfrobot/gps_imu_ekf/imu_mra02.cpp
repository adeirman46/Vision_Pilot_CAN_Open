#include "imu_mra02.hpp"

#include <Arduino.h>
#include <string.h>

namespace {

const double kGravity = 9.80665;
const double kDeg2Rad = 3.14159265358979323846 / 180.0;

int16_t s16(const uint8_t* p)
{
    return (int16_t)((uint16_t)p[0] | ((uint16_t)p[1] << 8));
}

float f32(const uint8_t* p)
{
    float f;
    memcpy(&f, p, 4);      // IEEE-754 little endian, same as the MCU
    return f;
}

}  // namespace

void Mra02Decoder::reset()
{
    len_        = 0;
    seen_mask_  = 0;
    working_    = ImuSample();
    have_ready_ = false;
    frames_ok_  = 0;
    frames_bad_ = 0;
    bytes_junk_ = 0;
}

// A channel arriving twice means the previous reading was complete: the module
// sends its four frames back to back, so the repeat is where the next cycle
// begins. There is no sequence number to use instead.
void Mra02Decoder::store_channel(uint8_t bit)
{
    const uint8_t m = (uint8_t)(1u << bit);
    if (seen_mask_ & m) flush();
    seen_mask_ |= m;
}

void Mra02Decoder::flush()
{
    if (working_.empty()) {
        seen_mask_ = 0;
        return;
    }
    working_.stamp_ms = millis();
    // Overwrite rather than queue: a reading the consumer never collected is
    // stale, and the filter wants the newest one, not a backlog to catch up on.
    ready_      = working_;
    have_ready_ = true;

    working_   = ImuSample();
    seen_mask_ = 0;
}

bool Mra02Decoder::pop(ImuSample& out)
{
    if (!have_ready_) return false;
    out         = ready_;
    have_ready_ = false;
    return true;
}

void Mra02Decoder::feed(const uint8_t* data, size_t n)
{
    for (size_t k = 0; k < n; ++k) {
        if (len_ == kBufMax) {
            // Nothing in a full buffer parsed, so the oldest half is junk —
            // usually the wrong baud rate. Drop it rather than stall for ever.
            bytes_junk_ += kBufMax / 2;
            memmove(buf_, buf_ + kBufMax / 2, kBufMax / 2);
            len_ = kBufMax / 2;
        }
        buf_[len_++] = data[k];
    }

    size_t i = 0;
    while (i + 5 <= len_) {
        // Every byte stepped over here failed to start a frame. Counting them is
        // what turns "no data" into "bytes are arriving but nothing decodes",
        // which is the signature of a wrong baud rate — and what the scan in
        // main.cpp reads to decide it has found the right one.
        if (buf_[i] != 0x7E || buf_[i + 1] != 0x23) { ++i; ++bytes_junk_; continue; }

        const size_t flen = buf_[i + 2];
        if (flen < 6 || flen > 128) { ++i; ++bytes_junk_; continue; }
        if (i + flen > len_) break;                 // wait for the rest

        unsigned sum = 0;
        for (size_t j = 0; j < flen - 1; ++j) sum += buf_[i + j];
        if ((sum & 0xFFu) != buf_[i + flen - 1]) {
            ++frames_bad_;
            ++i;                                     // resync past this 0x7E
            ++bytes_junk_;
            continue;
        }

        const uint8_t  id   = buf_[i + 3];
        const uint8_t* d    = &buf_[i + 4];
        const size_t   plen = flen - 5;
        ++frames_ok_;

        switch (id) {
        case 0x04:
            if (plen >= 18) {
                store_channel(0);
                // 2048 LSB per g — the +-16 g full scale of the part, confirmed
                // by the vector length reading exactly 1 g at rest.
                working_.ax = s16(d + 0) / 2048.0 * kGravity;
                working_.ay = s16(d + 2) / 2048.0 * kGravity;
                working_.az = s16(d + 4) / 2048.0 * kGravity;
                // +-2000 deg/s, the fixed range across this vendor's IMU line.
                // This is the one constant not independently confirmed: a scale
                // error here is absorbed by the filter's yaw-rate bias only if
                // it is additive, which it is not — so if heading consistently
                // under- or over-turns by the same *fraction*, suspect this.
                working_.gx = s16(d + 6) / 32768.0 * 2000.0 * kDeg2Rad;
                working_.gy = s16(d + 8) / 32768.0 * 2000.0 * kDeg2Rad;
                working_.gz = s16(d + 10) / 32768.0 * 2000.0 * kDeg2Rad;
                // The magnetometer (d+12..17) is skipped: the filter takes its
                // heading from the GPS course, which no indoor iron disturbs.
                working_.has_accel = true;
                working_.has_gyro  = true;
            }
            break;

        case 0x16:
            if (plen >= 16) {
                store_channel(1);
                working_.qw = f32(d + 0);
                working_.qx = f32(d + 4);
                working_.qy = f32(d + 8);
                working_.qz = f32(d + 12);
                working_.has_quat = true;
            }
            break;

        case 0x26:
            if (plen >= 12) {
                store_channel(2);
                working_.roll_rad  = f32(d + 0);    // radians, not degrees
                working_.pitch_rad = f32(d + 4);
                working_.yaw_rad   = f32(d + 8);
                working_.has_euler = true;
            }
            break;

        case 0x32:
            // Altitude, temperature, pressure. Nothing here uses them, but the
            // channel still marks the cycle boundary.
            store_channel(3);
            break;

        default:
            break;    // velocity/position frames on other firmware builds
        }

        i += flen;
    }

    if (i > 0) {
        // Everything before i that was not part of a frame is junk — count it,
        // because "bytes are arriving but nothing decodes" is the signature of a
        // wrong baud rate and the scan in main.cpp reads exactly that.
        memmove(buf_, buf_ + i, len_ - i);
        len_ -= i;
    }
}
