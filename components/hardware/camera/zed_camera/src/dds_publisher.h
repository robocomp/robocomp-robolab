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

// ZedDDSPublisher — publishes the ZED RGB and depth frames on a dedicated
// zero-copy DDS "media plane" (FastDDS), reusing common/media_transport (rc::media).
//
// This is an ADDITIONAL output protocol for the multiprotocol zed_camera
// component: the heavy pixels go out-of-band on their own DDS domain so any
// CORTEX consumer (e.g. the voxelizer agent) can attach with SHM data-sharing.
//
// It deliberately does NOT touch DSR: the camera *metadata* (intrinsics, topic
// discovery via the media_descriptor) is advertised on the DSR robot node by the
// robot_concept agent, not here. This class only moves pixels, and is gated by a
// config.toml flag (see SpecificWorker::initialize).
//
// FastDDS headers are kept behind a PIMPL so they never leak into specificworker.h.
// Not thread-safe: drive both publish_*() from a single producer thread.

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>

class ZedDDSPublisher
{
    public:
        struct Config
        {
            std::uint32_t domain_id     = 7;              // CORTEX media domain
            std::string   rgb_topic     = "rc/zed/rgb";
            std::string   depth_topic   = "rc/zed/depth";
            // Byte order of the RGB buffer. BOTH live sources deliver RGB: the real ZED path
            // converts native BGRA->RGB (see build_real_rgbd), and the Webots CameraRGBDSimple
            // proxy is RGB-ordered too. This only sets the frame's format LABEL (no copy), so the
            // consumer converts correctly. false = RGB8 (default, correct for both), true = BGR8.
            bool          rgb_is_bgr    = false;
            // --- QoS (carried in the descriptor; both ends must agree) ---
            int           history_depth      = 8;      // KEEP_LAST depth
            bool          shared_memory_only = true;   // SHM transport for same-board peers
            bool          data_sharing       = false;  // zero-copy loans; OFF = churn-safe (see media_transport.h)
        };

        ZedDDSPublisher();
        ~ZedDDSPublisher();
        ZedDDSPublisher(const ZedDDSPublisher&) = delete;
        ZedDDSPublisher& operator=(const ZedDDSPublisher&) = delete;

        // Create the two writers. Returns true iff both came up.
        bool init(const Config& cfg);
        [[nodiscard]] bool ready() const { return ready_; }

        // Serialize this plane as an rc::media::MediaDescriptor JSON string (domain,
        // topics, type tag, QoS). This is what the robot_concept DSR agent relays onto
        // the "zed" graph node so consumers discover the plane. Returns "" if not ready.
        [[nodiscard]] std::string descriptor_json() const;

        // Publish one RGB frame. The frame's format label is RGB8 or BGR8 per Config::rgb_is_bgr.
        // Returns false (frame dropped) on: not ready, empty/oversize payload, loan unavailable,
        // or publish failure. `step` is the row stride in bytes, `nbytes` the total payload size.
        bool publish_rgb(std::uint64_t stamp_ms, std::uint32_t width, std::uint32_t height,
                        std::uint32_t step, const std::uint8_t* data, std::size_t nbytes);

        // Publish one depth frame (published as FORMAT_DEPTH_F32 — float32 metres/mm per pixel).
        bool publish_depth(std::uint64_t stamp_ms, std::uint32_t width, std::uint32_t height,
                        std::uint32_t step, const std::uint8_t* data, std::size_t nbytes);

    private:
        struct Impl;
        std::unique_ptr<Impl> pimpl_;
        Config cfg_;              // remembered so descriptor_json() reports the live plane
        bool ready_ = false;
};
