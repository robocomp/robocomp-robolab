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
#include <print>

#include "media_transport.h"   // common/media_transport (added to the include path in CMake)

struct PoseDDSPublisher::Impl
{
    rc::media::PosePublisher pub;
    std::uint64_t frame_id = 0;
    // Diagnostic publish stats (reported every ~5 s from publish()).
    std::uint64_t stat_ok = 0;
    std::uint64_t stat_drop = 0;
    std::chrono::steady_clock::time_point stat_t0{};
};

PoseDDSPublisher::PoseDDSPublisher() : pimpl_(std::make_unique<Impl>()) {}
PoseDDSPublisher::~PoseDDSPublisher() = default;

bool PoseDDSPublisher::init(const Config& cfg)
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
        std::print("[imu_fusion_dds] DDS pose media plane ready domain={} topic='{}' data_sharing={}\n",
                   cfg.domain_id, cfg.topic, pimpl_->pub.data_sharing_active());
    else
        std::print(stderr, "[imu_fusion_dds] DDS pose media plane init FAILED (topic='{}')\n", cfg.topic);
    return ready_;
}

std::string PoseDDSPublisher::descriptor_json() const
{
    if (!ready_)
        return {};

    rc::media::MediaDescriptor d;
    d.version                = 1;
    d.domain_id              = cfg_.domain_id;
    d.type_name              = "PoseFrame";
    d.type_tag               = rc::media::POSE_FRAME_TYPE_TAG;
    d.history_depth          = cfg_.history_depth;
    d.shared_memory_only     = cfg_.shared_memory_only;
    d.data_sharing           = cfg_.data_sharing;
    d.ready                  = ready_;
    d.streams["pose"]        = cfg_.topic;
    d.stream_types["pose"]   = "PoseFrame";
    return d.to_json();
}

bool PoseDDSPublisher::publish(std::uint64_t stamp_ms,
                                float x, float y, float yaw,
                                float vx, float vy, float omega,
                                float adv, float side, float rot,
                                float sx, float sy, float syaw,
                                float svx, float svy,
                                int confidence)
{
    bool ok = false;
    if (ready_)
    {
        if (rc::media::PoseFrame* s = pimpl_->pub.loan(); s != nullptr)
        {
            s->stream_id(rc::media::STREAM_POSE);
            s->frame_id(pimpl_->frame_id++);
            s->stamp_ms(stamp_ms);
            s->x(x);       s->y(y);       s->yaw(yaw);
            s->vx(vx);     s->vy(vy);     s->omega(omega);
            s->adv(adv);   s->side(side); s->rot(rot);
            s->sx(sx);     s->sy(sy);     s->syaw(syaw);
            s->svx(svx);   s->svy(svy);
            s->confidence(confidence);
            ok = pimpl_->pub.publish(s);
        }
        // else: SHM pool exhausted -> counted as a drop below
    }

    // Diagnostic: report published/dropped every ~5 s.
    ok ? ++pimpl_->stat_ok : ++pimpl_->stat_drop;
    const auto now = std::chrono::steady_clock::now();
    if (std::chrono::duration<double>(now - pimpl_->stat_t0).count() >= 5.0)
    {
        std::print("[Pose] published {} / dropped {}\n", pimpl_->stat_ok, pimpl_->stat_drop);
        pimpl_->stat_ok = 0;
        pimpl_->stat_drop = 0;
        pimpl_->stat_t0 = now;
    }
    return ok;
}
