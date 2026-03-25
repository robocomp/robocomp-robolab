#pragma once

/// Concrete type aliases for the thread-safe BufferSync buffers used across
/// the component.  Keeps common_types.h lightweight (no BufferSync dependency)
/// while letting subsystems (e.g. RoomConcept) accept buffer pointers
/// without knowing SpecificWorker.

#include <Eigen/Dense>
#include <vector>
#include <cstdint>
#include <utility>
#include "doublebuffer_sync/doublebuffer_sync.h"
#include "common_types.h"

namespace rc
{
    /// Timestamped lidar scan (points + epoch-ms timestamp)
    using LidarData = std::pair<std::vector<Eigen::Vector3f>, std::int64_t>;

    /// Main sensor buffer: slot 0 = GT pose, slot 1 = high lidar, slot 2 = low lidar
    using SensorBuffer = BufferSync<InOut<Eigen::Affine2f, Eigen::Affine2f>,
                                    InOut<LidarData, LidarData>>;

    /// Single-channel circular buffer for velocity commands (joystick / controller)
    using VelocityBuffer = BufferSync<InOut<VelocityCommand, VelocityCommand>>;

    /// Single-channel circular buffer for measured odometry readings
    using OdometryBuffer = BufferSync<InOut<OdometryReading, OdometryReading>>;

} // namespace rc
