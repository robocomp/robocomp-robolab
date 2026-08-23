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

#include <atomic>
#include <chrono>
#include <print>

#include "media_transport.h"   // common/media_transport (added to the include path in CMake)

namespace
{
// steady_clock "now" in ms, as a plain integer so it can live in an atomic.
std::int64_t steady_ms()
{
    return std::chrono::duration_cast<std::chrono::milliseconds>(
               std::chrono::steady_clock::now().time_since_epoch()).count();
}
}  // namespace

struct ImuDDSPublisher::Impl
{
    rc::media::ImuPublisher pub;
    std::uint64_t frame_id = 0;
    // Last successful publish, for live(). Written by the publishing thread, read by the ICE
    // dispatch thread serving getMediaDescriptor() -> atomic. 0 = nothing published yet.
    std::atomic<std::int64_t> last_ok_ms{0};
    // Diagnostic publish stats (reported every ~5 s from publish()).
    std::uint64_t stat_ok = 0;
    std::uint64_t stat_drop = 0;
    std::chrono::steady_clock::time_point stat_t0{};
};

ImuDDSPublisher::ImuDDSPublisher() : pimpl_(std::make_unique<Impl>()) {}
ImuDDSPublisher::~ImuDDSPublisher() = default;

bool ImuDDSPublisher::init(const Config& cfg)
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
        std::print("[imu_dds] DDS imu media plane ready domain={} topic='{}' data_sharing={}\n",
                   cfg.domain_id, cfg.topic, pimpl_->pub.data_sharing_active());
    else
        std::print(stderr, "[imu_dds] DDS imu media plane init FAILED (topic='{}')\n", cfg.topic);
    return ready_;
}

bool ImuDDSPublisher::live() const
{
    const std::int64_t last = pimpl_->last_ok_ms.load(std::memory_order_relaxed);
    if (last == 0)
        return false;                                  // never published a sample
    return (steady_ms() - last) <= cfg_.stale_after_ms;
}

std::string ImuDDSPublisher::descriptor_json() const
{
    // Not live -> no descriptor. See the header: a non-empty descriptor is robot_concept's signal
    // to STOP bridging, so advertising one while no sample is flowing takes the IMU dark.
    if (!ready_ or not live())
        return {};

    rc::media::MediaDescriptor d;
    d.version              = 1;
    d.domain_id            = cfg_.domain_id;
    d.type_name            = "ImuFrame";
    d.type_tag             = rc::media::IMU_FRAME_TYPE_TAG;
    d.history_depth        = cfg_.history_depth;
    d.shared_memory_only   = cfg_.shared_memory_only;
    d.data_sharing         = cfg_.data_sharing;
    d.ready                = ready_;
    d.streams["imu"]       = cfg_.topic;
    d.stream_types["imu"]  = "ImuFrame";
    return d.to_json();
}

bool ImuDDSPublisher::publish(const Sample& smp)
{
    if (!ready_)
        return false;

    bool ok = false;
    if (rc::media::ImuFrame* f = pimpl_->pub.loan(); f != nullptr)
    {
        f->stream_id(rc::media::STREAM_IMU);
        f->frame_id(pimpl_->frame_id++);
        f->stamp_ms(smp.stamp_ms);
        f->sim_stamp_ms(smp.sim_stamp_ms);
        f->acc_x(smp.acc[0]);   f->acc_y(smp.acc[1]);   f->acc_z(smp.acc[2]);
        f->gyro_x(smp.gyro[0]); f->gyro_y(smp.gyro[1]); f->gyro_z(smp.gyro[2]);
        f->mag_x(smp.mag[0]);   f->mag_y(smp.mag[1]);   f->mag_z(smp.mag[2]);
        f->roll(smp.rpy[0]);    f->pitch(smp.rpy[1]);   f->yaw(smp.rpy[2]);
        f->temperature(smp.temperature);
        f->gyro_var(smp.gyro_var);
        f->acc_var(smp.acc_var);
        ok = pimpl_->pub.publish(f);
        if (ok)
            pimpl_->last_ok_ms.store(steady_ms(), std::memory_order_relaxed);
    }
    // else: SHM pool exhausted -> counted as a drop below

    // Diagnostic: report published/dropped every ~5 s. On a ~125 Hz IMU a steady drop
    // count means the pool is too shallow for the rate, not a bad sample.
    ok ? ++pimpl_->stat_ok : ++pimpl_->stat_drop;
    const auto now = std::chrono::steady_clock::now();
    if (const double secs = std::chrono::duration<double>(now - pimpl_->stat_t0).count(); secs >= 5.0)
    {
        std::print("[Imu] published {} / dropped {}  ({:.1f} Hz)\n",
                   pimpl_->stat_ok, pimpl_->stat_drop,
                   static_cast<double>(pimpl_->stat_ok) / secs);
        pimpl_->stat_ok = 0;
        pimpl_->stat_drop = 0;
        pimpl_->stat_t0 = now;
    }
    return ok;
}
