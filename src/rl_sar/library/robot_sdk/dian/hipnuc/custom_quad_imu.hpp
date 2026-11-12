// SPDX-License-Identifier: Apache-2.0
#ifndef CUSTOM_QUAD_IMU_HPP
#define CUSTOM_QUAD_IMU_HPP
#include <array>
#include <chrono>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace custom_quad::imu
{
using Clock = std::chrono::steady_clock;
using Vector3 = std::array<double, 3>;
using Quaternion = std::array<double, 4>; // w, x, y, z

struct Config
{
    std::optional<std::string> device;
    int baudrate = 115200; // Valid HI91 data observed on the deployment IMU.
    std::optional<Quaternion> sensor_to_body_wxyz; // v_body = R(q) * v_sensor.
    void validate() const;
};
Config load_config(const std::string& path);

struct Sample
{
    Quaternion quaternion_wxyz; // Sensor orientation in world coordinates, as in atom01.
    Vector3 angular_velocity_rad_s;
    Vector3 acceleration_m_s2; // Includes gravity; not a gravity-compensated linear acceleration.
    double temperature_c;
    std::uint16_t raw_status; // Reserved in the reference HI91 struct; no health inference.
    std::uint32_t device_time;
    Clock::time_point received_at;
};
struct BodySample
{
    Quaternion quaternion_wxyz;
    Vector3 angular_velocity_rad_s;
    Vector3 acceleration_m_s2;
    Vector3 projected_gravity; // Unit world-down vector expressed in body axes.
};
BodySample to_body(const Sample& sample, const Quaternion& sensor_to_body_wxyz);

struct Statistics
{
    std::uint64_t accepted = 0;
    std::uint64_t crc_errors = 0;
    std::uint64_t malformed = 0;
    std::uint64_t unsupported = 0;
    std::uint64_t invalid_values = 0;
};

class Decoder
{
public:
    // Only complete HI91 records are published; failed/other packets never refresh latest().
    bool feed(const std::uint8_t* bytes, std::size_t size, Clock::time_point received_at);
    const std::optional<Sample>& latest() const { return latest_; }
    const Statistics& statistics() const { return statistics_; }
private:
    void parse(Clock::time_point received_at);
    std::vector<std::uint8_t> buffer_;
    std::optional<Sample> latest_;
    Statistics statistics_;
};

class SerialReader
{
public:
    explicit SerialReader(const Config& config);
    ~SerialReader();
    SerialReader(const SerialReader&) = delete;
    SerialReader& operator=(const SerialReader&) = delete;
    // Reads at most one bounded chunk, sends no data or configuration commands.
    bool poll(int timeout_ms);
    const Decoder& decoder() const { return decoder_; }
private:
    struct Port;
    Port* port_ = nullptr;
    Decoder decoder_;
};
} // namespace custom_quad::imu
#endif
