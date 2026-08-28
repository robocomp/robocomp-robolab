/*
 *    Copyright (C) 2026 by YOUR NAME HERE
 *
 *    This file is part of RoboComp
 *
 *    RoboComp is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    RoboComp is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU General Public License for more details.
 *
 *    You should have received a copy of the GNU General Public License
 *    along with RoboComp.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

// PoseDDSPublisher — publishes imu_fusion's fused pose on a dedicated zero-copy DDS
// "media plane" (FastDDS), reusing common/media_transport (rc::media::PosePublisher).
// Same role as lidar3d_dds/zed_camera's DDS publishers, for the PoseFrame type (one
// "pose" stream: position+velocity in room/world frame plus robot-frame velocity, SI
// units throughout — metres/rad/s, NOT the mm/s SVD48VBase convention the ICE
// FullPoseEstimationPub output keeps for its existing consumers).
//
// It does NOT touch DSR: the discovery descriptor is relayed onto the graph by the
// robot_concept agent (MediaPlaneDDS_getMediaDescriptor, wired in specificworker.cpp),
// same as lidar3d_dds. This class only moves the fused-pose sample, gated by config.
// FastDDS headers stay behind a PIMPL so this header stays includable from the
// pybind11 binding without pulling FastDDS into the Python extension's ABI surface.
// Not thread-safe: drive publish() from one thread (imu_fusion_dds's compute() loop).

#include <cstdint>
#include <memory>
#include <string>

class PoseDDSPublisher
{
public:
    struct Config
    {
        std::uint32_t domain_id = 7;                      // CORTEX media domain (shared with lidar/imu/image planes)
        std::string   topic     = "rc/imu_fusion/pose";   // "pose" stream topic
        // --- QoS (carried in the descriptor; both ends must agree) ---
        int           history_depth      = 8;
        bool          shared_memory_only = true;
        bool          data_sharing       = false;       // OFF = churn-safe (see media_transport.h)
    };

    PoseDDSPublisher();
    ~PoseDDSPublisher();
    PoseDDSPublisher(const PoseDDSPublisher&) = delete;
    PoseDDSPublisher& operator=(const PoseDDSPublisher&) = delete;

    bool init(const Config& cfg);
    [[nodiscard]] bool ready() const { return ready_; }

    // rc::media::MediaDescriptor JSON (domain, topic, type tag, QoS) for the robot_concept
    // agent to relay onto the graph. Returns "" if not ready.
    [[nodiscard]] std::string descriptor_json() const;

    // Publish one fused-pose sample. Position/velocity in SI units (metres, rad, m/s,
    // rad/s) — see pose_frame.idl. Returns false (dropped) on: not ready or loan
    // unavailable/publish failure.
    bool publish(std::uint64_t stamp_ms,
                 float x, float y, float yaw,
                 float vx, float vy, float omega,
                 float adv, float side, float rot,
                 float sx, float sy, float syaw,
                 float svx, float svy,
                 int confidence);

private:
    struct Impl;
    std::unique_ptr<Impl> pimpl_;
    Config cfg_;
    bool ready_ = false;
};
