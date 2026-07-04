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
#include <cstring>
#include <print>
#include "media_transport.h"   // common/media_transport (added to the include path in CMake)

struct ZedDDSPublisher::Impl
{
    rc::media::MediaPublisher rgb;
    rc::media::MediaPublisher depth;
    std::uint64_t rgb_frame_id   = 0;
    std::uint64_t depth_frame_id = 0;
    std::uint32_t rgb_format     = rc::media::FORMAT_RGB8;   // set from cfg.rgb_is_bgr in init()
};

ZedDDSPublisher::ZedDDSPublisher() : pimpl_(std::make_unique<Impl>()) {}
ZedDDSPublisher::~ZedDDSPublisher() = default;

bool ZedDDSPublisher::init(const Config& cfg)
{
    cfg_ = cfg;

    rc::media::PublisherConfig rgb_cfg;
    rgb_cfg.domain_id          = cfg.domain_id;
    rgb_cfg.topic_name         = cfg.rgb_topic;
    rgb_cfg.history_depth      = cfg.history_depth;
    rgb_cfg.shared_memory_only = cfg.shared_memory_only;
    rgb_cfg.data_sharing       = cfg.data_sharing;

    rc::media::PublisherConfig depth_cfg = rgb_cfg;
    depth_cfg.topic_name = cfg.depth_topic;

    pimpl_->rgb_format = cfg.rgb_is_bgr ? rc::media::FORMAT_BGR8 : rc::media::FORMAT_RGB8;

    const bool rgb_ok   = pimpl_->rgb.init(rgb_cfg);
    const bool depth_ok = pimpl_->depth.init(depth_cfg);
    ready_ = rgb_ok && depth_ok;

    if (ready_)
        std::print("[zed_camera] DDS media plane ready domain={} rgb='{}' depth='{}' data_sharing={}\n",
                   cfg.domain_id, cfg.rgb_topic, cfg.depth_topic,
                   pimpl_->rgb.data_sharing_active() && pimpl_->depth.data_sharing_active());
    else
        std::print(stderr, "[zed_camera] DDS media plane init FAILED (rgb={}, depth={})\n", rgb_ok, depth_ok);
    return ready_;
}

std::string ZedDDSPublisher::descriptor_json() const
{
    if (!ready_)
        return {};

    // Mirror the QoS the writers actually use so the advertised descriptor matches.
    rc::media::MediaDescriptor d;
    d.version            = 1;
    d.domain_id          = cfg_.domain_id;
    d.type_name          = "ImageFrame";
    d.type_tag           = rc::media::IMAGE_FRAME_TYPE_TAG;
    d.history_depth      = cfg_.history_depth;
    d.shared_memory_only = cfg_.shared_memory_only;
    d.data_sharing       = cfg_.data_sharing;
    d.ready              = ready_;
    d.streams["rgb"]     = cfg_.rgb_topic;
    d.streams["depth"]   = cfg_.depth_topic;
    return d.to_json();
}

namespace
{
// Shared loan -> fill -> memcpy -> publish path for both streams.
bool publish_one(rc::media::MediaPublisher& pub, std::uint32_t stream_id, std::uint64_t& frame_id,
                 std::uint64_t stamp_ms, std::uint32_t width, std::uint32_t height,
                 std::uint32_t step, std::uint32_t format,
                 const std::uint8_t* data, std::size_t nbytes)
{
    if (data == nullptr || nbytes == 0 || nbytes > rc::media::MAX_IMAGE_BYTES)
        return false;

    rc::media::ImageFrame* s = pub.loan();
    if (s == nullptr)
        return false;   // SHM pool exhausted -> drop this frame

    s->stream_id(stream_id);
    s->frame_id(frame_id++);
    s->stamp_ms(stamp_ms);
    s->width(width);
    s->height(height);
    s->step(step);
    s->format(format);
    s->size(static_cast<std::uint32_t>(nbytes));
    std::memcpy(s->data().data(), data, nbytes);

    return pub.publish(s);
}
}  // namespace

bool ZedDDSPublisher::publish_rgb(std::uint64_t stamp_ms, std::uint32_t width, std::uint32_t height,
                                  std::uint32_t step, const std::uint8_t* data, std::size_t nbytes)
{
    if (!ready_)
        return false;
    return publish_one(pimpl_->rgb, rc::media::STREAM_ZED_RGB, pimpl_->rgb_frame_id,
                       stamp_ms, width, height, step, pimpl_->rgb_format, data, nbytes);
}

bool ZedDDSPublisher::publish_depth(std::uint64_t stamp_ms, std::uint32_t width, std::uint32_t height,
                                    std::uint32_t step, const std::uint8_t* data, std::size_t nbytes)
{
    if (!ready_)
        return false;
    return publish_one(pimpl_->depth, rc::media::STREAM_ZED_DEPTH, pimpl_->depth_frame_id,
                       stamp_ms, width, height, step, rc::media::FORMAT_DEPTH_F32, data, nbytes);
}
