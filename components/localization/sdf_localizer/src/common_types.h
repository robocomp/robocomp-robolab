#pragma once

#include <chrono>
#include <vector>
#include <Eigen/Dense>

// Minimal shared types required by room_model/room_concept code.
// Keep this header lightweight and dependency-free.

namespace rc
{
    enum class RoomState
    {
        MAPPING = 0,
        LOCALIZED = 1
    };

    struct VelocityCommand
    {
        float adv_x = 0.0f;  // lateral velocity, m/s (robot frame)
        float adv_y = 0.0f;  // forward velocity, m/s (robot frame)
        float rot = 0.0f;    // angular velocity, rad/s
        std::int64_t source_ts_ms = 0;   // timestamp from the source (sensor/planner epoch-ms)
        std::int64_t recv_ts_ms   = 0;   // local reception timestamp (epoch-ms)
        std::chrono::time_point<std::chrono::high_resolution_clock> timestamp; // kept for compat
        std::int64_t effective_ts_ms() const { return source_ts_ms > 0 ? source_ts_ms : recv_ts_ms; }
        VelocityCommand() = default;
        VelocityCommand(float x, float z, float r)
            : adv_x(x), adv_y(z), rot(r)
            , timestamp(std::chrono::high_resolution_clock::now())
        {}
    };

    /// Measured odometry reading from encoders/IMU (robot frame velocities)
    /// Received via FullPoseEstimationPub from real robot or Webots
    struct OdometryReading
    {
        float adv = 0.0f;    // forward velocity, m/s (robot frame)
        float side = 0.0f;   // lateral velocity, m/s (robot frame)
        float rot = 0.0f;    // angular velocity, rad/s
        std::int64_t source_ts_ms = 0;   // timestamp from the sensor (epoch-ms)
        std::int64_t recv_ts_ms   = 0;   // local reception timestamp (epoch-ms)
        std::chrono::time_point<std::chrono::high_resolution_clock> timestamp; // kept for compat
        std::int64_t effective_ts_ms() const { return source_ts_ms > 0 ? source_ts_ms : recv_ts_ms; }
    };

    using VelocityHistory = std::vector<VelocityCommand>;
    using TimeStamp = std::chrono::time_point<std::chrono::high_resolution_clock>;
}
