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

// Ricoh360DDSPublisher — publishes the Ricoh 360 panorama on a dedicated zero-copy
// DDS "media plane" (FastDDS), reusing common/media_transport (rc::media). Same role
// as zed_camera's ZedDDSPublisher, but for the WIDE Image360Frame type (single "rgb360"
// stream, ~5.5 MB buffer) rather than the ImageFrame RGB/depth pair.
//
// It does NOT touch DSR: the discovery descriptor is relayed onto the "ricoh" graph
// node by the robot_concept agent. This class only moves pixels, gated by config.
// FastDDS headers stay behind a PIMPL. Not thread-safe: drive publish() from one thread.

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>

class Ricoh360DDSPublisher
{
public:
    struct Config
    {
        std::uint32_t domain_id = 7;                // CORTEX media domain
        std::string   topic     = "rc/ricoh/rgb";   // "rgb360" stream topic
        // Byte order of the panorama buffer. Real Theta hardware runs a gstreamer BGR pipeline
        // (is_bgr=true, the default). The Webots simulator path reads the Camera360RGB proxy, which
        // is RGB-ordered — set DDS.Format="rgb" for that (see config_webots.toml). Sets only the
        // frame's format LABEL (no copy) so the consumer converts right.
        bool          is_bgr    = true;
        // --- QoS (carried in the descriptor; both ends must agree) ---
        int           history_depth      = 8;
        bool          shared_memory_only = true;
        bool          data_sharing       = false;   // OFF = churn-safe (see media_transport.h)
    };

    Ricoh360DDSPublisher();
    ~Ricoh360DDSPublisher();
    Ricoh360DDSPublisher(const Ricoh360DDSPublisher&) = delete;
    Ricoh360DDSPublisher& operator=(const Ricoh360DDSPublisher&) = delete;

    bool init(const Config& cfg);
    [[nodiscard]] bool ready() const { return ready_; }

    // rc::media::MediaDescriptor JSON (domain, topic, type tag, QoS) for the robot_concept
    // agent to relay onto the "ricoh" DSR node. Returns "" if not ready.
    [[nodiscard]] std::string descriptor_json() const;

    // Publish one panorama frame (format label BGR8/RGB8 per Config::is_bgr). Returns
    // false (dropped) on: not ready, empty/oversize payload, loan unavailable, or publish
    // failure. `step` is the row stride in bytes, `nbytes` the total payload size.
    bool publish(std::uint64_t stamp_ms, std::uint32_t width, std::uint32_t height,
                 std::uint32_t step, const std::uint8_t* data, std::size_t nbytes);

private:
    struct Impl;
    std::unique_ptr<Impl> pimpl_;
    Config cfg_;
    bool ready_ = false;
};
