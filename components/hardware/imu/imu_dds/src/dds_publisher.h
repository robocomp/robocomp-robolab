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

// ImuDDSPublisher — publishes the IMU sample on the dedicated zero-copy DDS "media
// plane" (FastDDS), reusing common/media_transport (rc::media::ImuPublisher). Exact
// same role as lidar3d_dds's LidarDDSPublisher, for the ImuFrame type (one "imu"
// stream: acc/gyro/mag/rpy scalars plus both clocks and the two variances).
//
// The frame this writes is byte-for-byte the one robot_concept's read_imu_thread used
// to bridge from Ice, on the same topic (rc/imu/data, domain 7) — so a consumer
// (room_concept's ImuIngestor) cannot tell the two producers apart, which is the whole
// point: with this component running, robot_concept stops bridging and the consumer's
// code is IDENTICAL in simulation and on hardware.
//
// It does NOT touch DSR: the discovery descriptor is relayed onto the "imu" graph node
// by the robot_concept agent (via MediaPlaneDDS_getMediaDescriptor). This class only
// moves the sample, gated by config. FastDDS headers stay behind a PIMPL so the
// component's headers never pull them in. Not thread-safe: drive publish() from one
// thread (compute()).

#include <cstdint>
#include <memory>
#include <string>

#include "imu_sample.h"

class ImuDDSPublisher
{
public:
    struct Config
    {
        std::uint32_t domain_id = 7;                // CORTEX media domain (shared with lidar/image planes)
        std::string   topic     = "rc/imu/data";    // "imu" stream topic (matches robot_concept Media.imu_topic)
        // --- QoS (carried in the descriptor; both ends must agree) ---
        int           history_depth      = 8;
        bool          shared_memory_only = true;
        bool          data_sharing       = false;   // OFF = churn-safe (see media_transport.h)
        // How long after the last published sample the plane still counts as live(). Generous
        // next to any real IMU period (~9 ms here), so a healthy stream can never trip it, while
        // a stalled source hands the plane back to robot_concept within a couple of seconds.
        int           stale_after_ms     = 2000;
    };

    ImuDDSPublisher();
    ~ImuDDSPublisher();
    ImuDDSPublisher(const ImuDDSPublisher&) = delete;
    ImuDDSPublisher& operator=(const ImuDDSPublisher&) = delete;

    bool init(const Config& cfg);
    [[nodiscard]] bool ready() const { return ready_; }

    // Is the stream actually LIVE — at least one sample published, and one recently enough
    // (within Config::stale_after_ms)? This is NOT the same as ready(): the DDS writer comes up
    // fine even when the ICE source is refused, so ready() alone says nothing about data flowing.
    [[nodiscard]] bool live() const;

    // rc::media::MediaDescriptor JSON (domain, topic, type tag, QoS) for the robot_concept
    // agent to relay onto the "imu" DSR node.
    //
    // Returns "" unless the stream is live() — and "" is what tells robot_concept to keep
    // bridging the IMU itself. This matters: robot_concept adopts a plane on the strength of a
    // non-empty descriptor, so advertising one the moment the WRITER exists lets an imu_dds that
    // cannot reach its ICE source (wrong port, driver down) silently take ownership of
    // rc/imu/data and take the IMU dark — a real failure, not a hypothetical. Claim the plane
    // only while actually producing, and hand it back by going quiet.
    [[nodiscard]] std::string descriptor_json() const;

    // Publish one IMU sample (SI units — see imu_sample.h). Returns false (dropped) on:
    // not ready, loan unavailable, or publish failure.
    bool publish(const ImuSample& s);

private:
    struct Impl;
    std::unique_ptr<Impl> pimpl_;
    Config cfg_;
    bool ready_ = false;
};
