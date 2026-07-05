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

// LidarDDSPublisher — publishes the 3D LiDAR scan on a dedicated zero-copy DDS "media
// plane" (FastDDS), reusing common/media_transport (rc::media::LidarPublisher). Same
// role as zed_camera/ricoh_omni_dds's DDS publishers, for the LidarFrame type (a single
// "lidar" stream of interleaved x,y,z floats in METRES).
//
// It does NOT touch DSR: the discovery descriptor is relayed onto the "lidar3D" graph
// node by the robot_concept agent. This class only moves points, gated by config.
// FastDDS headers stay behind a PIMPL. Not thread-safe: drive publish() from one thread.

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>

class LidarDDSPublisher
{
public:
    struct Config
    {
        std::uint32_t domain_id = 7;                    // CORTEX media domain
        std::string   topic     = "rc/lidar3d/points";  // "lidar" stream topic
        // --- QoS (carried in the descriptor; both ends must agree) ---
        int           history_depth      = 8;
        bool          shared_memory_only = true;
        bool          data_sharing       = false;       // OFF = churn-safe (see media_transport.h)
    };

    LidarDDSPublisher();
    ~LidarDDSPublisher();
    LidarDDSPublisher(const LidarDDSPublisher&) = delete;
    LidarDDSPublisher& operator=(const LidarDDSPublisher&) = delete;

    bool init(const Config& cfg);
    [[nodiscard]] bool ready() const { return ready_; }

    // rc::media::MediaDescriptor JSON (domain, topic, type tag, QoS) for the robot_concept
    // agent to relay onto the "lidar3D" DSR node. Returns "" if not ready.
    [[nodiscard]] std::string descriptor_json() const;

    // Publish one scan: `xyz` is interleaved x,y,z in METRES, `count` points (stride 3).
    // Returns false (dropped) on: not ready, empty/oversize payload, loan unavailable, or
    // publish failure.
    bool publish(std::uint64_t stamp_ms, const float* xyz, std::uint32_t count);

private:
    struct Impl;
    std::unique_ptr<Impl> pimpl_;
    Config cfg_;
    bool ready_ = false;
};
