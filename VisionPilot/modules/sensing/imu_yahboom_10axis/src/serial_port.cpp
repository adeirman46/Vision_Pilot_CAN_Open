#include <imu/serial_port.hpp>

#include <algorithm>
#include <cerrno>
#include <cstring>

#include <dirent.h>
#include <fcntl.h>
#include <poll.h>
#include <termios.h>
#include <unistd.h>

namespace imu {

namespace {

speed_t baud_constant(int baud)
{
    switch (baud) {
    case 9600:    return B9600;
    case 19200:   return B19200;
    case 38400:   return B38400;
    case 57600:   return B57600;
    case 115200:  return B115200;
    case 230400:  return B230400;
    case 460800:  return B460800;
    case 500000:  return B500000;
    case 921600:  return B921600;
    case 1000000: return B1000000;
    default:      return 0;
    }
}

}  // namespace

SerialPort::~SerialPort()
{
    close();
}

bool SerialPort::open(const std::string& device, int baud, std::string& error)
{
    close();

    fd_ = ::open(device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) {
        error = device + ": " + std::strerror(errno);
        return false;
    }
    device_ = device;

    termios tio{};
    if (tcgetattr(fd_, &tio) != 0) {
        error = device + ": tcgetattr: " + std::strerror(errno);
        close();
        return false;
    }

    cfmakeraw(&tio);
    tio.c_cflag |= (CLOCAL | CREAD);
    tio.c_cflag &= ~CSTOPB;          // 8N1 — every module in this family
    tio.c_cflag &= ~PARENB;
    tio.c_cflag &= ~CRTSCTS;
    tio.c_cc[VMIN]  = 0;             // read() returns what is there; poll() waits
    tio.c_cc[VTIME] = 0;

    if (tcsetattr(fd_, TCSANOW, &tio) != 0) {
        error = device + ": tcsetattr: " + std::strerror(errno);
        close();
        return false;
    }

    if (!set_baud(baud, error)) {
        close();
        return false;
    }

    flush_input();
    return true;
}

bool SerialPort::set_baud(int baud, std::string& error)
{
    if (fd_ < 0) {
        error = "port not open";
        return false;
    }
    const speed_t speed = baud_constant(baud);
    if (speed == 0) {
        error = "unsupported baud rate " + std::to_string(baud);
        return false;
    }

    termios tio{};
    if (tcgetattr(fd_, &tio) != 0) {
        error = std::string("tcgetattr: ") + std::strerror(errno);
        return false;
    }
    cfsetispeed(&tio, speed);
    cfsetospeed(&tio, speed);
    if (tcsetattr(fd_, TCSANOW, &tio) != 0) {
        error = std::string("tcsetattr: ") + std::strerror(errno);
        return false;
    }

    baud_ = baud;
    flush_input();
    return true;
}

void SerialPort::close()
{
    if (fd_ >= 0) ::close(fd_);
    fd_ = -1;
}

int SerialPort::read(uint8_t* buf, size_t max, int timeout_ms)
{
    if (fd_ < 0) return -1;

    pollfd pfd{fd_, POLLIN, 0};
    const int rc = ::poll(&pfd, 1, timeout_ms);
    if (rc == 0) return 0;
    if (rc < 0) return errno == EINTR ? 0 : -1;
    if (pfd.revents & (POLLERR | POLLHUP | POLLNVAL)) return -1;

    const ssize_t n = ::read(fd_, buf, max);
    if (n < 0) return (errno == EAGAIN || errno == EINTR) ? 0 : -1;
    if (n == 0) return 0;
    return static_cast<int>(n);
}

bool SerialPort::write(const uint8_t* buf, size_t n)
{
    if (fd_ < 0) return false;
    size_t sent = 0;
    while (sent < n) {
        const ssize_t w = ::write(fd_, buf + sent, n - sent);
        if (w < 0) {
            if (errno == EAGAIN || errno == EINTR) continue;
            return false;
        }
        sent += static_cast<size_t>(w);
    }
    return true;
}

void SerialPort::flush_input()
{
    if (fd_ >= 0) tcflush(fd_, TCIFLUSH);
}

const std::vector<int>& candidate_bauds()
{
    // 9600 and 115200 are the two factory defaults across this family; the rest
    // show up on units someone has reconfigured.
    static const std::vector<int> bauds{115200, 9600, 921600, 230400,
                                        460800, 57600, 38400, 19200};
    return bauds;
}

std::vector<std::string> list_serial_ports()
{
    std::vector<std::string> ports;

    DIR* dir = ::opendir("/dev");
    if (!dir) return ports;

    while (dirent* e = ::readdir(dir)) {
        const std::string name = e->d_name;
        // USB bridges first (the CH340 on the Yahboom board is one), then the
        // CDC-ACM devices, then the AMA UARTs on a Jetson/Pi header.
        if (name.rfind("ttyUSB", 0) == 0 || name.rfind("ttyACM", 0) == 0 ||
            name.rfind("ttyAMA", 0) == 0 || name.rfind("ttyTHS", 0) == 0) {
            ports.push_back("/dev/" + name);
        }
    }
    ::closedir(dir);

    std::sort(ports.begin(), ports.end());
    return ports;
}

}  // namespace imu
