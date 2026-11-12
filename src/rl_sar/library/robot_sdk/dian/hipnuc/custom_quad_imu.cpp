// SPDX-License-Identifier: Apache-2.0
// HI91 packet layout and CRC reference: HiPNUC, Copyright (c) 2006-2024,
// atom01_deploy/src/imu/src/drivers/hipnuc/hipnuc_dec.{c,h} (Apache-2.0).
// This bounded decoder accepts only complete HI91 payloads, not the reference's other formats.
#include "custom_quad_imu.hpp"
#include <yaml-cpp/yaml.h>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <limits>
#include <stdexcept>

namespace custom_quad::imu
{
namespace
{
constexpr double kDegreesToRadians = 0.017453292519943295;
constexpr double kGravity = 9.8; // Preserve atom01's acceleration convention.
constexpr std::size_t kHi91Size = 76;
std::uint16_t u16(const std::uint8_t* data) { return data[0] | (std::uint16_t(data[1]) << 8); }
std::uint32_t u32(const std::uint8_t* data)
{
    return std::uint32_t(data[0]) | (std::uint32_t(data[1]) << 8) |
           (std::uint32_t(data[2]) << 16) | (std::uint32_t(data[3]) << 24);
}
double f32(const std::uint8_t* data)
{
    static_assert(sizeof(float) == 4 && std::numeric_limits<float>::is_iec559, "HI91 requires IEEE-754 float32");
    const auto bits = u32(data);
    float value;
    std::memcpy(&value, &bits, 4);
    return value;
}
std::uint16_t crc16(std::uint16_t crc, const std::uint8_t* data, std::size_t size)
{
    for (std::size_t i = 0; i < size; ++i)
    {
        crc ^= std::uint16_t(data[i]) << 8;
        for (int bit = 0; bit < 8; ++bit)
            crc = static_cast<std::uint16_t>((crc << 1) ^ ((crc & 0x8000) ? 0x1021 : 0));
    }
    return crc;
}
template<std::size_t N> bool finite(const std::array<double, N>& data)
{
    return std::all_of(data.begin(), data.end(), [](double value) { return std::isfinite(value); });
}
Quaternion normalized(Quaternion q)
{
    double square = 0;
    for (const auto v : q) square += v * v;
    if (!finite(q) || !std::isfinite(square) || square < 0.81 || square > 1.21)
        throw std::invalid_argument("IMU 四元数模长必须在 0.9..1.1 范围内");
    for (auto& v : q) v /= std::sqrt(square);
    return q;
}
Quaternion conjugate(Quaternion q) { for (int i = 1; i < 4; ++i) q[i] = -q[i]; return q; }
Quaternion multiply(const Quaternion& a, const Quaternion& b)
{
    return {a[0]*b[0]-a[1]*b[1]-a[2]*b[2]-a[3]*b[3],
            a[0]*b[1]+a[1]*b[0]+a[2]*b[3]-a[3]*b[2],
            a[0]*b[2]-a[1]*b[3]+a[2]*b[0]+a[3]*b[1],
            a[0]*b[3]+a[1]*b[2]-a[2]*b[1]+a[3]*b[0]};
}
Vector3 rotate(const Quaternion& q, const Vector3& v)
{
    const auto result = multiply(multiply(q, {0, v[0], v[1], v[2]}), conjugate(q));
    return {result[1], result[2], result[3]};
}
bool supplied(const YAML::Node& node) { return node && !node.IsNull(); }
}

void Config::validate() const
{
    if (device && (device->empty() || !std::filesystem::path(*device).is_absolute()))
        throw std::invalid_argument("imu.device 必须是串口设备的绝对路径");
    switch (baudrate)
    {
        case 9600: case 19200: case 38400: case 57600: case 115200: case 230400:
        case 460800: case 921600: case 1000000: case 2000000: break;
        default: throw std::invalid_argument("不支持的 IMU 串口波特率");
    }
    if (sensor_to_body_wxyz) (void)normalized(*sensor_to_body_wxyz);
}

Config load_config(const std::string& path)
{
    const auto root = YAML::LoadFile(path);
    if (!root.IsMap() || root["brand"].as<std::string>("") != "HIPNUC" ||
        root["protocol"].as<std::string>("") != "hi91_serial")
        throw std::invalid_argument("IMU 配置要求 brand: HIPNUC 且 protocol: hi91_serial");
    Config config;
    if (supplied(root["device"])) config.device = root["device"].as<std::string>();
    config.baudrate = root["baudrate"].as<int>();
    const auto mount = root["sensor_to_body_wxyz"];
    if (supplied(mount))
    {
        if (!mount.IsSequence() || mount.size() != 4)
            throw std::invalid_argument("sensor_to_body_wxyz 必须有四个值");
        Quaternion q;
        for (int i = 0; i < 4; ++i) q[i] = mount[i].as<double>();
        config.sensor_to_body_wxyz = normalized(q);
    }
    config.validate();
    return config;
}

BodySample to_body(const Sample& sample, const Quaternion& sensor_to_body_wxyz)
{
    const auto mount = normalized(sensor_to_body_wxyz);
    BodySample result;
    result.quaternion_wxyz = normalized(multiply(normalized(sample.quaternion_wxyz), conjugate(mount)));
    result.angular_velocity_rad_s = rotate(mount, sample.angular_velocity_rad_s);
    result.acceleration_m_s2 = rotate(mount, sample.acceleration_m_s2);
    result.projected_gravity = rotate(conjugate(result.quaternion_wxyz), {0, 0, -1});
    return result;
}

bool Decoder::feed(const std::uint8_t* bytes, std::size_t size, Clock::time_point now)
{
    if (!size) return false;
    if (!bytes) throw std::invalid_argument("IMU 输入指针为空");
    const auto previous = statistics_.accepted;
    for (std::size_t i = 0; i < size; ++i)
    {
        buffer_.push_back(bytes[i]);
        parse(now);
    }
    return statistics_.accepted > previous;
}

void Decoder::parse(Clock::time_point now)
{
    while (buffer_.size() >= 2)
    {
        if (buffer_[0] != 0x5a || buffer_[1] != 0xa5)
        {
            buffer_.erase(buffer_.begin());
            continue;
        }
        if (buffer_.size() < 6) return;
        const auto length = u16(buffer_.data() + 2);
        if (length == 0 || length > 506)
        {
            ++statistics_.malformed;
            buffer_.erase(buffer_.begin());
            continue;
        }
        const std::size_t total = length + 6;
        if (buffer_.size() < total) return;
        const auto crc = crc16(crc16(0, buffer_.data(), 4), buffer_.data() + 6, length);
        if (crc != u16(buffer_.data() + 4))
        {
            ++statistics_.crc_errors;
            buffer_.erase(buffer_.begin()); // Preserve a valid frame embedded after damaged framing.
            continue;
        }
        const auto* payload = buffer_.data() + 6;
        if (length != kHi91Size || payload[0] != 0x91) ++statistics_.unsupported;
        else
        {
            Sample sample{};
            sample.raw_status = u16(payload + 1);
            sample.temperature_c = payload[3] < 128 ? payload[3] : int(payload[3]) - 256;
            sample.device_time = u32(payload + 8);
            sample.received_at = now;
            for (int i = 0; i < 3; ++i)
            {
                sample.acceleration_m_s2[i] = f32(payload + 12 + 4*i) * kGravity;
                sample.angular_velocity_rad_s[i] = f32(payload + 24 + 4*i) * kDegreesToRadians;
            }
            for (int i = 0; i < 4; ++i) sample.quaternion_wxyz[i] = f32(payload + 60 + 4*i);
            bool valid = finite(sample.acceleration_m_s2) && finite(sample.angular_velocity_rad_s);
            try { sample.quaternion_wxyz = normalized(sample.quaternion_wxyz); }
            catch (const std::invalid_argument&) { valid = false; }
            if (!valid) ++statistics_.invalid_values;
            else { latest_ = sample; ++statistics_.accepted; }
        }
        buffer_.erase(buffer_.begin(), buffer_.begin() + total);
    }
}

} // namespace custom_quad::imu
