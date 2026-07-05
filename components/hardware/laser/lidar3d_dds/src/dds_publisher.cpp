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

#include "dds_publisher.h"

#include <chrono>
#include <cstring>
#include <print>

#include "media_transport.h"   // common/media_transport (added to the include path in CMake)

struct LidarDDSPublisher::Impl
{
    rc::media::LidarPublisher pub;
    std::uint64_t frame_id = 0;
    // Diagnostic publish stats (reported every ~5 s from publish()).
    std::uint64_t stat_ok = 0;
    std::uint64_t stat_drop = 0;
    std::chrono::steady_clock::time_point stat_t0{};
};

LidarDDSPublisher::LidarDDSPublisher() : pimpl_(std::make_unique<Impl>()) {}
LidarDDSPublisher::~LidarDDSPublisher() = default;

bool LidarDDSPublisher::init(const Config& cfg)
{
    cfg_ = cfg;

    rc::media::PublisherConfig pc;
    pc.domain_id          = cfg.domain_id;
    pc.topic_name         = cfg.topic;
    pc.history_depth      = cfg.history_depth;
    pc.shared_memory_only = cfg.shared_memory_only;
    pc.data_sharing       = cfg.data_sharing;

    pimpl_->stat_t0 = std::chrono::steady_clock::now();
    ready_ = pimpl_->pub.init(pc);

    if (ready_)
        std::print("[lidar3d_dds] DDS lidar media plane ready domain={} topic='{}' data_sharing={}\n",
                   cfg.domain_id, cfg.topic, pimpl_->pub.data_sharing_active());
    else
        std::print(stderr, "[lidar3d_dds] DDS lidar media plane init FAILED (topic='{}')\n", cfg.topic);
    return ready_;
}

std::string LidarDDSPublisher::descriptor_json() const
{
    if (!ready_)
        return {};

    rc::media::MediaDescriptor d;
    d.version                = 1;
    d.domain_id              = cfg_.domain_id;
    d.type_name              = "LidarFrame";
    d.type_tag               = rc::media::LIDAR_FRAME_TYPE_TAG;
    d.history_depth          = cfg_.history_depth;
    d.shared_memory_only     = cfg_.shared_memory_only;
    d.data_sharing           = cfg_.data_sharing;
    d.ready                  = ready_;
    d.streams["lidar"]       = cfg_.topic;
    d.stream_types["lidar"]  = "LidarFrame";
    return d.to_json();
}

bool LidarDDSPublisher::publish(std::uint64_t stamp_ms, const float* xyz, std::uint32_t count)
{
    if (!ready_)
        return false;

    const bool oversize = count > rc::media::MAX_LIDAR_POINTS;
    bool ok = false;
    if (xyz != nullptr && count != 0 && not oversize)
    {
        if (rc::media::LidarFrame* s = pimpl_->pub.loan(); s != nullptr)
        {
            s->stream_id(rc::media::STREAM_LIDAR);
            s->frame_id(pimpl_->frame_id++);
            s->stamp_ms(stamp_ms);
            s->count(count);
            s->stride(3);
            s->format(rc::media::LIDAR_FORMAT_XYZ_F32);
            std::memcpy(s->points().data(), xyz, static_cast<std::size_t>(count) * 3 * sizeof(float));
            ok = pimpl_->pub.publish(s);
        }
        // else: SHM pool exhausted -> counted as a drop below
    }

    // Diagnostic: report published/dropped every ~5 s, with the last scan size so an
    // oversize drop (count > the LidarFrame bound) is immediately visible.
    ok ? ++pimpl_->stat_ok : ++pimpl_->stat_drop;
    const auto now = std::chrono::steady_clock::now();
    if (std::chrono::duration<double>(now - pimpl_->stat_t0).count() >= 5.0)
    {
        std::print("[Lidar] published {} / dropped {}  last={} pts (max {}){}\n",
                   pimpl_->stat_ok, pimpl_->stat_drop, count,
                   static_cast<std::size_t>(rc::media::MAX_LIDAR_POINTS),
                   oversize ? "  <-- OVERSIZE: decimate or raise the LidarFrame bound" : "");
        pimpl_->stat_ok = 0;
        pimpl_->stat_drop = 0;
        pimpl_->stat_t0 = now;
    }
    return ok;
}
