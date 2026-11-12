// SPDX-License-Identifier: Apache-2.0
#include "custom_quad_imu.hpp"
#include <array>
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <poll.h>
#include <stdexcept>
#include <sys/file.h>
#include <sys/stat.h>
#include <termios.h>
#include <unistd.h>

namespace custom_quad::imu
{
namespace
{
std::runtime_error failure(const std::string& operation)
{
    return std::runtime_error(operation + ": " + std::strerror(errno));
}
speed_t speed(int baud)
{
    switch (baud)
    {
        case 9600: return B9600;
        case 19200: return B19200;
        case 38400: return B38400;
        case 57600: return B57600;
        case 115200: return B115200;
        case 230400: return B230400;
        case 460800: return B460800;
        case 921600: return B921600;
        case 1000000: return B1000000;
        case 2000000: return B2000000;
        default: throw std::invalid_argument("不支持的 IMU 波特率");
    }
}
}

struct SerialReader::Port
{
    int fd = -1;
    termios original{};
    bool restore = false;
    ~Port()
    {
        if (fd >= 0)
        {
            if (restore) ::tcsetattr(fd, TCSANOW, &original);
            ::close(fd);
        }
    }
};

SerialReader::SerialReader(const Config& config)
{
    config.validate();
    if (!config.device) throw std::invalid_argument("未设置 IMU 串口设备");
    port_ = new Port;
    try
    {
        port_->fd = ::open(config.device->c_str(), O_RDONLY | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
        if (port_->fd < 0) throw failure("open IMU " + *config.device);
        struct stat info{};
        if (::fstat(port_->fd, &info) != 0) throw failure("IMU fstat");
        if (!S_ISCHR(info.st_mode)) throw std::runtime_error("IMU 路径必须指向串口字符设备");
        if (::flock(port_->fd, LOCK_EX | LOCK_NB) < 0) throw failure("lock IMU device");
        if (::tcgetattr(port_->fd, &port_->original) != 0) throw failure("IMU tcgetattr");
        auto settings = port_->original;
        ::cfmakeraw(&settings);
        settings.c_cflag &= ~(CSIZE | PARENB | CSTOPB | CRTSCTS | HUPCL);
        settings.c_cflag |= CS8 | CLOCAL | CREAD;
        settings.c_cc[VMIN] = 0;
        settings.c_cc[VTIME] = 0;
        const auto baud = speed(config.baudrate);
        if (::cfsetispeed(&settings, baud) != 0 || ::cfsetospeed(&settings, baud) != 0)
            throw failure("IMU baudrate");
        if (::tcsetattr(port_->fd, TCSANOW, &settings) != 0) throw failure("IMU tcsetattr");
        port_->restore = true;
        if (::tcflush(port_->fd, TCIFLUSH) != 0) throw failure("IMU receive flush");
    }
    catch (...) { delete port_; port_ = nullptr; throw; }
}
SerialReader::~SerialReader() { delete port_; }

bool SerialReader::poll(int timeout_ms)
{
    if (timeout_ms < 0 || timeout_ms > 100) throw std::invalid_argument("IMU 轮询等待时间必须为 0..100 ms");
    pollfd descriptor{port_->fd, POLLIN, 0};
    const auto result = ::poll(&descriptor, 1, timeout_ms);
    if (result < 0)
    {
        if (errno == EINTR) return false;
        throw failure("IMU poll");
    }
    if (!result) return false;
    if (descriptor.revents & (POLLERR | POLLHUP | POLLNVAL))
        throw std::runtime_error("IMU 串口设备断开或读取失败");
    if (!(descriptor.revents & POLLIN)) return false;
    std::array<std::uint8_t, 4096> bytes{};
    const auto count = ::read(port_->fd, bytes.data(), bytes.size());
    if (count < 0)
    {
        if (errno == EAGAIN || errno == EWOULDBLOCK || errno == EINTR) return false;
        throw failure("IMU read");
    }
    if (!count) return false;
    return decoder_.feed(bytes.data(), static_cast<std::size_t>(count), Clock::now());
}
} // namespace custom_quad::imu
