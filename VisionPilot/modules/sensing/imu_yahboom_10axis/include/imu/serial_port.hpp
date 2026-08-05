#ifndef IMU_YAHBOOM_SERIAL_PORT_HPP
#define IMU_YAHBOOM_SERIAL_PORT_HPP

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace imu {

// Raw 8N1 serial port. Non-blocking reads with a poll() timeout, so a silent
// sensor never wedges the caller's loop.
class SerialPort
{
public:
    SerialPort() = default;
    ~SerialPort();

    SerialPort(const SerialPort&)            = delete;
    SerialPort& operator=(const SerialPort&) = delete;

    bool open(const std::string& device, int baud, std::string& error);
    void close();
    bool is_open() const { return fd_ >= 0; }

    // Change the line speed without reopening — used by the baud scan.
    bool set_baud(int baud, std::string& error);

    // Up to `max` bytes, waiting at most `timeout_ms` for the first of them.
    // Returns bytes read, 0 on timeout, -1 on a real error.
    int read(uint8_t* buf, size_t max, int timeout_ms);

    bool write(const uint8_t* buf, size_t n);

    void flush_input();

    const std::string& device() const { return device_; }
    int                baud() const { return baud_; }

private:
    int         fd_ = -1;
    std::string device_;
    int         baud_ = 0;
};

// Baud rates worth trying, fastest-first among the ones these modules ship with.
const std::vector<int>& candidate_bauds();

// Serial devices that look like a USB-attached sensor, newest first. Lets the
// tools default to "the one thing that is plugged in" instead of hard-coding
// /dev/ttyUSB0.
std::vector<std::string> list_serial_ports();

}  // namespace imu

#endif  // IMU_YAHBOOM_SERIAL_PORT_HPP
