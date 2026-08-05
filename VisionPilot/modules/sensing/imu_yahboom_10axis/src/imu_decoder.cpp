#include <imu/imu_decoder.hpp>

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <map>
#include <sstream>
#include <vector>

namespace imu {

namespace {

uint64_t now_us()
{
    using namespace std::chrono;
    return static_cast<uint64_t>(
        duration_cast<microseconds>(steady_clock::now().time_since_epoch()).count());
}

int16_t s16(const uint8_t* p)     // little endian, as every part in this family
{
    return static_cast<int16_t>(static_cast<uint16_t>(p[0]) |
                                (static_cast<uint16_t>(p[1]) << 8));
}

int32_t s32(const uint8_t* p)
{
    return static_cast<int32_t>(static_cast<uint32_t>(p[0]) |
                                (static_cast<uint32_t>(p[1]) << 8) |
                                (static_cast<uint32_t>(p[2]) << 16) |
                                (static_cast<uint32_t>(p[3]) << 24));
}

float f32(const uint8_t* p)
{
    float f;
    std::memcpy(&f, p, 4);      // IEEE-754 little endian, same as the host
    return f;
}

constexpr size_t kMaxBuf   = 8192;
constexpr int    kLockHits = 3;      // clean frames needed before locking on

}  // namespace

const char* protocol_name(Protocol p)
{
    switch (p) {
    case Protocol::Yahboom:   return "Yahboom MRA02";
    case Protocol::WitMotion: return "WitMotion 0x55";
    case Protocol::Yesense:   return "Yesense YIS";
    case Protocol::Ascii:     return "ASCII";
    default:                  return "unknown";
    }
}

double Vec3::norm() const
{
    return std::sqrt(x * x + y * y + z * z);
}

std::string format_stats(const StreamStats& s)
{
    char buf[192];
    std::snprintf(buf, sizeof(buf),
                  "%llu B  %llu frames  %llu bad  %llu junk  %llu samples  %.1f Hz",
                  static_cast<unsigned long long>(s.bytes),
                  static_cast<unsigned long long>(s.frames_ok),
                  static_cast<unsigned long long>(s.frames_bad),
                  static_cast<unsigned long long>(s.bytes_junk),
                  static_cast<unsigned long long>(s.samples), s.sample_hz);
    return buf;
}

std::string hex_dump(const uint8_t* data, size_t n, uint64_t offset)
{
    std::string out;
    char        line[128];
    for (size_t i = 0; i < n; i += 16) {
        std::snprintf(line, sizeof(line), "%08llx  ",
                      static_cast<unsigned long long>(offset + i));
        out += line;
        for (size_t k = 0; k < 16; ++k) {
            if (i + k < n) std::snprintf(line, sizeof(line), "%02x ", data[i + k]);
            else           std::snprintf(line, sizeof(line), "   ");
            out += line;
            if (k == 7) out += ' ';
        }
        out += " |";
        for (size_t k = 0; k < 16 && i + k < n; ++k) {
            const uint8_t c = data[i + k];
            out += std::isprint(c) ? static_cast<char>(c) : '.';
        }
        out += "|\n";
    }
    return out;
}

// ─── Buffer handling ─────────────────────────────────────────────────────────

void ImuDecoder::reset()
{
    buf_.clear();
    ready_.clear();
    line_.clear();
    working_       = ImuSample{};
    wit_seen_mask_ = 0;
    yah_hits_ = wit_hits_ = yes_hits_ = ascii_hits_ = 0;
    protocol_      = Protocol::Unknown;
    stats_         = StreamStats{};
    byte_hist_.clear();
}

void ImuDecoder::note_unknown_byte(uint8_t b)
{
    if (protocol_ == Protocol::Unknown) byte_hist_[b]++;
}

void ImuDecoder::feed(const uint8_t* data, size_t n)
{
    stats_.bytes += n;
    for (size_t i = 0; i < n; ++i) {
        buf_.push_back(data[i]);
        note_unknown_byte(data[i]);
    }

    // Every parser scans without consuming until it finds a frame it can prove,
    // so an undecided stream is never damaged by the parser that is wrong about
    // it. Loop until no parser can make further progress.
    for (;;) {
        bool progress = false;
        if (protocol_ == Protocol::Unknown || protocol_ == Protocol::Yahboom)
            progress |= try_yahboom();
        if (protocol_ == Protocol::Unknown || protocol_ == Protocol::WitMotion)
            progress |= try_witmotion();
        if (protocol_ == Protocol::Unknown || protocol_ == Protocol::Yesense)
            progress |= try_yesense();
        if (protocol_ == Protocol::Unknown || protocol_ == Protocol::Ascii)
            progress |= try_ascii();
        if (!progress) break;
    }

    // Nothing recognised it: drop the oldest half rather than growing forever.
    // The count is what tells the tools "bytes are arriving but the format is
    // not one we know" — most often a wrong baud rate.
    if (buf_.size() > kMaxBuf) {
        const size_t drop = buf_.size() - kMaxBuf / 2;
        stats_.bytes_junk += drop;
        buf_.erase(buf_.begin(), buf_.begin() + static_cast<long>(drop));
    }
}

bool ImuDecoder::pop(ImuSample& out)
{
    if (ready_.empty()) return false;
    out = ready_.front();
    ready_.erase(ready_.begin());
    return true;
}

// ─── Sample assembly ─────────────────────────────────────────────────────────
//
// Shared by both multi-frame formats: seeing a channel twice means the previous
// reading is complete.
void ImuDecoder::group_store(uint8_t channel_bit)
{
    const uint32_t bit = 1u << channel_bit;
    if (wit_seen_mask_ & bit) group_flush();
    wit_seen_mask_ |= bit;
}

void ImuDecoder::group_flush()
{
    if (working_.empty()) {
        wit_seen_mask_ = 0;
        return;
    }

    working_.stamp_us = now_us();
    if (last_emit_us_ != 0 && working_.stamp_us > last_emit_us_) {
        const double hz = 1e6 / static_cast<double>(working_.stamp_us - last_emit_us_);
        hz_avg_ = hz_avg_ > 0.0 ? hz_avg_ * 0.9 + hz * 0.1 : hz;
        stats_.sample_hz = hz_avg_;
    }
    last_emit_us_ = working_.stamp_us;

    ready_.push_back(working_);
    stats_.samples++;
    if (ready_.size() > 256) ready_.erase(ready_.begin());   // UI fell behind

    working_       = ImuSample{};
    wit_seen_mask_ = 0;
}

// ─── Yahboom MRA02 ───────────────────────────────────────────────────────────
//
//   0x7E 0x23 | len | id | payload | checksum
//
// len counts the whole frame including the header and the checksum; the
// checksum is the low byte of the sum of everything before it. Four frames make
// one reading, sent back to back at 25 Hz on the factory settings.
//
//   id 0x04, 18 B   9 x int16   accel, gyro, magnetometer
//   id 0x16, 16 B   4 x float   quaternion w x y z
//   id 0x26, 12 B   3 x float   euler roll pitch yaw, in RADIANS
//   id 0x32, 16 B   4 x float   altitude m, temperature C, pressure Pa,
//                               reference pressure Pa captured at power-on
//
// Verified against a live MRA02-V1.0: the quaternion decodes to exactly the
// Euler angles in id 0x26, and the accelerometer vector matches the attitude
// those angles describe to three decimal places.

bool ImuDecoder::try_yahboom()
{
    const size_t n = buf_.size();
    for (size_t i = 0; i + 5 <= n; ++i) {
        if (buf_[i] != 0x7E || buf_[i + 1] != 0x23) continue;

        const size_t len = buf_[i + 2];
        if (len < 6 || len > 128) continue;
        if (i + len > n) return false;                 // wait for the rest

        unsigned sum = 0;
        for (size_t k = 0; k < len - 1; ++k) sum += buf_[i + k];
        if ((sum & 0xFFu) != buf_[i + len - 1]) {
            if (protocol_ == Protocol::Yahboom && i == 0) stats_.frames_bad++;
            continue;
        }

        if (protocol_ == Protocol::Unknown) {
            if (++yah_hits_ < kLockHits) return false;
            protocol_ = Protocol::Yahboom;
        }

        const uint8_t id  = buf_[i + 3];
        const size_t  plen = len - 5;
        std::vector<uint8_t> d(plen);
        for (size_t k = 0; k < plen; ++k) d[k] = buf_[i + 4 + k];

        if (i > 0) stats_.bytes_junk += i;
        buf_.erase(buf_.begin(), buf_.begin() + static_cast<long>(i + len));
        stats_.frames_ok++;

        switch (id) {
        case 0x04:
            if (plen >= 18) {
                group_store(0);
                // 2048 LSB per g — the +-16 g full scale of the part, confirmed
                // by the vector length reading exactly 1 g at rest.
                working_.accel.x = s16(&d[0]) / 2048.0 * kGravity;
                working_.accel.y = s16(&d[2]) / 2048.0 * kGravity;
                working_.accel.z = s16(&d[4]) / 2048.0 * kGravity;
                // +-2000 deg/s, the fixed range across this vendor's IMU line.
                working_.gyro.x = s16(&d[6]) / 32768.0 * 2000.0 * kDeg2Rad;
                working_.gyro.y = s16(&d[8]) / 32768.0 * 2000.0 * kDeg2Rad;
                working_.gyro.z = s16(&d[10]) / 32768.0 * 2000.0 * kDeg2Rad;
                // Magnetometer stays in raw LSB: the vendor publishes no scale,
                // and the fusion only ever uses its direction.
                working_.mag.x = s16(&d[12]);
                working_.mag.y = s16(&d[14]);
                working_.mag.z = s16(&d[16]);
                working_.has_accel = working_.has_gyro = working_.has_mag = true;
            }
            break;

        case 0x16:
            if (plen >= 16) {
                group_store(1);
                working_.qw = f32(&d[0]);
                working_.qx = f32(&d[4]);
                working_.qy = f32(&d[8]);
                working_.qz = f32(&d[12]);
                working_.has_quat = true;
            }
            break;

        case 0x26:
            if (plen >= 12) {
                group_store(2);
                working_.roll_deg  = f32(&d[0]) * kRad2Deg;
                working_.pitch_deg = f32(&d[4]) * kRad2Deg;
                working_.yaw_deg   = f32(&d[8]) * kRad2Deg;
                working_.has_euler = true;
            }
            break;

        case 0x32:
            if (plen >= 16) {
                group_store(3);
                working_.altitude_m    = f32(&d[0]);
                working_.temperature_c = f32(&d[4]);
                working_.pressure_pa   = f32(&d[8]);
                working_.has_temp = working_.has_baro = true;
            }
            break;

        default:
            break;    // velocity/position frames on other firmware builds
        }
        return true;
    }
    return false;
}

// ─── WitMotion / WT901 / HWT906 / JY901 ──────────────────────────────────────
//
// 11 bytes: 0x55, type, 8 data bytes, checksum = sum of the first ten. One
// channel per frame, sent back to back.

void ImuDecoder::wit_store(uint8_t type, const uint8_t* d)
{
    group_store(static_cast<uint8_t>(type - 0x50));

    switch (type) {
    case 0x51:   // acceleration, +-16 g full scale, plus die temperature
        working_.accel.x = s16(d + 0) / 32768.0 * 16.0 * kGravity;
        working_.accel.y = s16(d + 2) / 32768.0 * 16.0 * kGravity;
        working_.accel.z = s16(d + 4) / 32768.0 * 16.0 * kGravity;
        working_.temperature_c = s16(d + 6) / 100.0;
        working_.has_accel = true;
        working_.has_temp  = true;
        break;

    case 0x52:   // angular rate, +-2000 deg/s full scale
        working_.gyro.x = s16(d + 0) / 32768.0 * 2000.0 * kDeg2Rad;
        working_.gyro.y = s16(d + 2) / 32768.0 * 2000.0 * kDeg2Rad;
        working_.gyro.z = s16(d + 4) / 32768.0 * 2000.0 * kDeg2Rad;
        working_.has_gyro = true;
        break;

    case 0x53:   // the module's own attitude solution
        working_.roll_deg  = s16(d + 0) / 32768.0 * 180.0;
        working_.pitch_deg = s16(d + 2) / 32768.0 * 180.0;
        working_.yaw_deg   = s16(d + 4) / 32768.0 * 180.0;
        working_.has_euler = true;
        break;

    case 0x54:   // magnetometer, raw LSB on most of the family
        working_.mag.x   = s16(d + 0);
        working_.mag.y   = s16(d + 2);
        working_.mag.z   = s16(d + 4);
        working_.has_mag = true;
        break;

    case 0x56:   // barometer: pressure in Pa, height in cm
        working_.pressure_pa = s32(d + 0);
        working_.altitude_m  = s32(d + 4) / 100.0;
        working_.has_baro    = true;
        break;

    case 0x59:   // quaternion, w x y z
        working_.qw = s16(d + 0) / 32768.0;
        working_.qx = s16(d + 2) / 32768.0;
        working_.qy = s16(d + 4) / 32768.0;
        working_.qz = s16(d + 6) / 32768.0;
        working_.has_quat = true;
        break;

    default:     // 0x50 time, 0x55 port status, 0x57/0x58/0x5A GNSS — not used here
        break;
    }
}

bool ImuDecoder::try_witmotion()
{
    const size_t n = buf_.size();
    for (size_t i = 0; i + 11 <= n; ++i) {
        if (buf_[i] != 0x55) continue;

        const uint8_t type = buf_[i + 1];
        if (type < 0x50 || type > 0x5A) continue;

        uint8_t payload[8];
        unsigned sum = 0x55 + type;
        for (size_t k = 0; k < 8; ++k) {
            payload[k] = buf_[i + 2 + k];
            sum += payload[k];
        }
        if ((sum & 0xFFu) != buf_[i + 10]) {
            if (protocol_ == Protocol::WitMotion && i == 0) stats_.frames_bad++;
            continue;
        }

        if (protocol_ == Protocol::Unknown) {
            if (++wit_hits_ < kLockHits) {
                // Proven but not yet trusted: leave the bytes in place so the
                // other parsers get their turn at the same window.
                return false;
            }
            protocol_ = Protocol::WitMotion;
        }

        if (i > 0) stats_.bytes_junk += i;
        buf_.erase(buf_.begin(), buf_.begin() + static_cast<long>(i + 11));
        stats_.frames_ok++;
        wit_store(type, payload);
        return true;
    }
    return false;
}

// ─── Yesense YIS ─────────────────────────────────────────────────────────────
//
// 0x59 0x53 | tid(2) | len(1) | payload | ck1 ck2   (Fletcher over tid..payload)
// The payload is a run of id/len/value records.
//
// The scale factors below are the published YIS100 ones. `imu_monitor --tlv`
// prints every record's id, length and raw bytes next to the decoded value, so
// if your unit's firmware uses a different table the mismatch is visible on the
// first line instead of hiding behind plausible-looking numbers.

bool ImuDecoder::try_yesense()
{
    const size_t n = buf_.size();
    for (size_t i = 0; i + 7 <= n; ++i) {
        if (buf_[i] != 0x59 || buf_[i + 1] != 0x53) continue;

        const size_t len   = buf_[i + 4];
        const size_t total = 5 + len + 2;
        if (i + total > n) return false;          // wait for the rest

        uint8_t ck1 = 0, ck2 = 0;
        for (size_t k = i + 2; k < i + 5 + len; ++k) {
            ck1 = static_cast<uint8_t>(ck1 + buf_[k]);
            ck2 = static_cast<uint8_t>(ck2 + ck1);
        }
        if (ck1 != buf_[i + 5 + len] || ck2 != buf_[i + 6 + len]) {
            if (protocol_ == Protocol::Yesense && i == 0) stats_.frames_bad++;
            continue;
        }

        if (protocol_ == Protocol::Unknown) {
            if (++yes_hits_ < kLockHits) return false;
            protocol_ = Protocol::Yesense;
        }

        std::vector<uint8_t> payload(len);
        for (size_t k = 0; k < len; ++k) payload[k] = buf_[i + 5 + k];

        if (i > 0) stats_.bytes_junk += i;
        buf_.erase(buf_.begin(), buf_.begin() + static_cast<long>(i + total));
        stats_.frames_ok++;

        ImuSample s;
        size_t    p = 0;
        while (p + 2 <= payload.size()) {
            const uint8_t id  = payload[p];
            const uint8_t dl  = payload[p + 1];
            const uint8_t* d  = payload.data() + p + 2;
            if (p + 2 + dl > payload.size()) break;

            switch (id) {
            case 0x10:                                   // temperature, 0.01 C
                if (dl >= 2) { s.temperature_c = s16(d) / 100.0; s.has_temp = true; }
                break;
            case 0x20:                                   // accel, 1e-6 g
                if (dl >= 12) {
                    s.accel.x = s32(d + 0) * 1e-6 * kGravity;
                    s.accel.y = s32(d + 4) * 1e-6 * kGravity;
                    s.accel.z = s32(d + 8) * 1e-6 * kGravity;
                    s.has_accel = true;
                }
                break;
            case 0x21:                                   // gyro, 1e-6 deg/s
                if (dl >= 12) {
                    s.gyro.x = s32(d + 0) * 1e-6 * kDeg2Rad;
                    s.gyro.y = s32(d + 4) * 1e-6 * kDeg2Rad;
                    s.gyro.z = s32(d + 8) * 1e-6 * kDeg2Rad;
                    s.has_gyro = true;
                }
                break;
            case 0x30:                                   // magnetometer, 1e-3 uT
                if (dl >= 12) {
                    s.mag.x = s32(d + 0) * 1e-3;
                    s.mag.y = s32(d + 4) * 1e-3;
                    s.mag.z = s32(d + 8) * 1e-3;
                    s.has_mag = true;
                }
                break;
            case 0x40:                                   // euler, 1e-6 deg —
                if (dl >= 12) {                          // pitch first here
                    s.pitch_deg = s32(d + 0) * 1e-6;
                    s.roll_deg  = s32(d + 4) * 1e-6;
                    s.yaw_deg   = s32(d + 8) * 1e-6;
                    s.has_euler = true;
                }
                break;
            case 0x41:                                   // quaternion, 1e-6
                if (dl >= 16) {
                    s.qw = s32(d + 0) * 1e-6;
                    s.qx = s32(d + 4) * 1e-6;
                    s.qy = s32(d + 8) * 1e-6;
                    s.qz = s32(d + 12) * 1e-6;
                    s.has_quat = true;
                }
                break;
            case 0x60:                                   // pressure, Pa
                if (dl >= 4) { s.pressure_pa = s32(d); s.has_baro = true; }
                break;
            default:
                break;
            }
            p += 2 + dl;
        }

        if (!s.empty()) {
            s.stamp_us = now_us();
            if (last_emit_us_ != 0 && s.stamp_us > last_emit_us_) {
                const double hz = 1e6 / static_cast<double>(s.stamp_us - last_emit_us_);
                hz_avg_ = hz_avg_ > 0.0 ? hz_avg_ * 0.9 + hz * 0.1 : hz;
                stats_.sample_hz = hz_avg_;
            }
            last_emit_us_ = s.stamp_us;
            ready_.push_back(s);
            stats_.samples++;
            if (ready_.size() > 256) ready_.erase(ready_.begin());
        }
        return true;
    }
    return false;
}

// ─── ASCII ───────────────────────────────────────────────────────────────────
//
// Firmware that prints instead of packing. Three shapes cover what these boards
// ship with:
//   #YPR=12.3,-4.5,0.8                 Razor-style attitude
//   roll:1.2 pitch:3.4 yaw:5.6         labelled pairs, any order
//   0.01,-0.02,9.79,0.1,0.0,-0.1,...   bare CSV: 3 = attitude, >=6 = a/g[/m]

bool ImuDecoder::try_ascii()
{
    bool progress = false;

    while (!buf_.empty()) {
        const uint8_t c = buf_.front();
        // A binary stream reaches here too; anything outside printable ASCII
        // means this parser is not the right one for those bytes.
        if (c != '\n' && c != '\r' && c != '\t' && (c < 0x20 || c > 0x7E)) {
            if (protocol_ != Protocol::Ascii) return progress;
            buf_.pop_front();
            stats_.bytes_junk++;
            line_.clear();
            continue;
        }

        buf_.pop_front();
        progress = true;

        if (c != '\n' && c != '\r') {
            if (line_.size() < 512) line_ += static_cast<char>(c);
            continue;
        }
        if (line_.empty()) continue;

        std::string line;
        line.swap(line_);

        // Pull out every number and every "label:value" pair in one pass.
        ImuSample           s;
        std::vector<double> nums;
        std::string         label;
        bool                labelled = false;

        for (size_t i = 0; i < line.size();) {
            const char ch = line[i];
            if (std::isalpha(static_cast<unsigned char>(ch))) {
                label.clear();
                while (i < line.size() &&
                       (std::isalnum(static_cast<unsigned char>(line[i])) ||
                        line[i] == '_')) {
                    label += static_cast<char>(std::tolower(line[i]));
                    ++i;
                }
                continue;
            }
            if (std::isdigit(static_cast<unsigned char>(ch)) || ch == '-' || ch == '+') {
                char*        end = nullptr;
                const double v   = std::strtod(line.c_str() + i, &end);
                if (end && end != line.c_str() + i) {
                    if (label == "roll")       { s.roll_deg  = v; s.has_euler = true; labelled = true; }
                    else if (label == "pitch") { s.pitch_deg = v; s.has_euler = true; labelled = true; }
                    else if (label == "yaw" ||
                             label == "heading"){ s.yaw_deg  = v; s.has_euler = true; labelled = true; }
                    else if (label == "temp")  { s.temperature_c = v; s.has_temp = true; labelled = true; }
                    else nums.push_back(v);
                    label.clear();
                    i = static_cast<size_t>(end - line.c_str());
                    continue;
                }
            }
            ++i;
        }

        const bool ypr = line.rfind("#YPR", 0) == 0 || line.rfind("YPR", 0) == 0;
        if (ypr && nums.size() >= 3) {
            s.yaw_deg   = nums[0];
            s.pitch_deg = nums[1];
            s.roll_deg  = nums[2];
            s.has_euler = true;
        } else if (!labelled && nums.size() >= 6) {
            s.accel.x = nums[0]; s.accel.y = nums[1]; s.accel.z = nums[2];
            s.gyro.x  = nums[3] * kDeg2Rad;
            s.gyro.y  = nums[4] * kDeg2Rad;
            s.gyro.z  = nums[5] * kDeg2Rad;
            s.has_accel = s.has_gyro = true;
            if (nums.size() >= 9) {
                s.mag.x = nums[6]; s.mag.y = nums[7]; s.mag.z = nums[8];
                s.has_mag = true;
            }
        } else if (!labelled && nums.size() == 3 &&
                   std::fabs(nums[0]) <= 180.0 && std::fabs(nums[1]) <= 180.0 &&
                   std::fabs(nums[2]) <= 360.0) {
            s.roll_deg = nums[0]; s.pitch_deg = nums[1]; s.yaw_deg = nums[2];
            s.has_euler = true;
        }

        if (s.empty()) continue;

        if (protocol_ == Protocol::Unknown) {
            if (++ascii_hits_ < kLockHits) continue;
            protocol_ = Protocol::Ascii;
        }

        s.stamp_us = now_us();
        if (last_emit_us_ != 0 && s.stamp_us > last_emit_us_) {
            const double hz = 1e6 / static_cast<double>(s.stamp_us - last_emit_us_);
            hz_avg_ = hz_avg_ > 0.0 ? hz_avg_ * 0.9 + hz * 0.1 : hz;
            stats_.sample_hz = hz_avg_;
        }
        last_emit_us_ = s.stamp_us;
        ready_.push_back(s);
        stats_.frames_ok++;
        stats_.samples++;
        if (ready_.size() > 256) ready_.erase(ready_.begin());
    }
    return progress;
}

// ─── Sniffer ─────────────────────────────────────────────────────────────────
//
// When nothing decodes, the useful question is "what repeats, and how far
// apart". A fixed-length binary frame shows up as one byte value appearing at a
// near-constant stride — that stride is the frame length and the byte is the
// header, which is most of what is needed to add a decoder.

std::string ImuDecoder::sniff_report() const
{
    if (protocol_ != Protocol::Unknown)
        return std::string("locked onto ") + protocol_name(protocol_);
    if (buf_.empty() && stats_.bytes == 0)
        return "no bytes received at all — wrong port, or the module is not powered";

    std::vector<uint8_t> window(buf_.begin(), buf_.end());
    if (window.size() < 64)
        return "only " + std::to_string(window.size()) +
               " bytes buffered — too few to guess a frame layout";

    struct Cand { uint8_t byte; int stride; int hits; };
    std::vector<Cand> cands;

    for (const auto& [b, count] : byte_hist_) {
        if (count < 4) continue;
        std::vector<size_t> at;
        for (size_t i = 0; i < window.size(); ++i)
            if (window[i] == b) at.push_back(i);
        if (at.size() < 4) continue;

        std::map<int, int> strides;
        for (size_t i = 1; i < at.size(); ++i)
            strides[static_cast<int>(at[i] - at[i - 1])]++;

        auto best = std::max_element(
            strides.begin(), strides.end(),
            [](const auto& a, const auto& c) { return a.second < c.second; });
        if (best == strides.end()) continue;
        // A stride that holds for most occurrences is a frame; anything less
        // regular is just a common byte value.
        if (best->second * 2 < static_cast<int>(at.size()) || best->first < 4) continue;
        cands.push_back({b, best->first, best->second});
    }

    std::sort(cands.begin(), cands.end(),
              [](const Cand& a, const Cand& b) { return a.hits > b.hits; });

    std::ostringstream os;
    os << "unrecognised format: " << stats_.bytes << " bytes in, "
       << stats_.bytes_junk << " discarded";
    if (cands.empty()) {
        os << "\n  no repeating frame structure — this usually means the baud rate"
              " is wrong (bytes are real, the bit timing is not)";
        return os.str();
    }
    os << "\n  repeating structure found:";
    for (size_t i = 0; i < cands.size() && i < 3; ++i) {
        char l[96];
        std::snprintf(l, sizeof(l), "\n    header 0x%02X every %d bytes (%d times)",
                      cands[i].byte, cands[i].stride, cands[i].hits);
        os << l;
    }
    os << "\n  run with --raw and send the dump to add this format";
    return os.str();
}

}  // namespace imu
