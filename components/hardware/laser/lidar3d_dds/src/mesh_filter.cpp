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

#include "mesh_filter.h"
#include "urdf_mesh_loader.h"

#include <cstdint>
#include <cstdio>
#include <iostream>
#include <vector>

MeshFilter::MeshFilter() = default;

MeshFilter::~MeshFilter()
{
    delete m_loader;
    if (m_scene)  rtcReleaseScene(m_scene);
    if (m_device) rtcReleaseDevice(m_device);
}

bool MeshFilter::init(const Config& cfg)
{
    cfg_ = cfg;

    m_device = rtcNewDevice(nullptr);
    m_scene  = rtcNewScene(m_device);
    rtcSetSceneFlags(m_scene, RTC_SCENE_FLAG_DYNAMIC);
    rtcSetSceneBuildQuality(m_scene, RTC_BUILD_QUALITY_LOW);

    m_loader = new URDFMeshLoader(m_device, m_scene);

    bool loaded = false;
    if (cfg.robot_name == "Shadow")
        loaded = m_loader->loadSingleSTL(cfg.mesh_dir + "/Shadow/shadow.stl");
    else if (cfg.robot_name == "P3Bot")
        loaded = m_loader->loadURDF(cfg.mesh_dir + "/P3Bot/P3Bot.urdf", cfg.mesh_dir + "/P3Bot/");
    else
        std::cerr << "[MeshFilter] unknown Robot.name '" << cfg.robot_name << "'" << std::endl;

    if (not loaded)
    {
        std::cerr << "[MeshFilter] failed to load mesh for '" << cfg.robot_name << "'" << std::endl;
        return false;
    }

    rtcCommitScene(m_scene);

    RTCBounds b;
    rtcGetSceneBounds(m_scene, &b);
    std::printf("[MeshFilter] robot='%s' bounds Min(%.3f,%.3f,%.3f) Max(%.3f,%.3f,%.3f) "
                "floor_z=%.0f top_z=%.0f dilate=%.3f footprint_r=%.3fm z[%.0f,%.0f] "
                "skirt_r=%.3fm z[%.0f,%.0f]\n",
                cfg.robot_name.c_str(), b.lower_x, b.lower_y, b.lower_z,
                b.upper_x, b.upper_y, b.upper_z, cfg.floor_z, cfg.top_z, cfg.dilate,
                cfg.footprint_radius, cfg.footprint_z_min, cfg.footprint_z_max,
                cfg.skirt_radius, cfg.skirt_z_min, cfg.skirt_z_max);

    ready_ = true;
    return true;
}

namespace
{
// True if the ROBOT-frame point (mm) is within `dilate` metres of the mesh: 6 orthogonal
// rays approximate a sphere query (max corner error ~1.4·dilate). rtcIntersect1 is
// thread-safe on a committed scene, so filter() calls this from many threads.
inline bool point_hits_mesh(RTCScene scene, const Eigen::Vector3f& p_mm, float dilate)
{
    const float org[3] = { p_mm.x() * 0.001f, p_mm.y() * 0.001f, p_mm.z() * 0.001f };   // mm -> m
    static constexpr float dirs[6][3] = {{1,0,0},{-1,0,0},{0,1,0},{0,-1,0},{0,0,1},{0,0,-1}};
    for (const auto& d : dirs)
    {
        RTCRayHit rh{};
        rh.ray.org_x = org[0]; rh.ray.org_y = org[1]; rh.ray.org_z = org[2];
        rh.ray.dir_x = d[0];   rh.ray.dir_y = d[1];   rh.ray.dir_z = d[2];
        rh.ray.tnear = 0.0f;   rh.ray.tfar = dilate;  rh.ray.time = 0.0f;
        rh.ray.mask  = -1;     rh.ray.flags = 0;
        rh.hit.geomID = RTC_INVALID_GEOMETRY_ID;
        rtcIntersect1(scene, &rh, nullptr);
        if (rh.hit.geomID != RTC_INVALID_GEOMETRY_ID)
            return true;
    }
    return false;
}
}  // namespace

void MeshFilter::filter(RoboCompLidar3D::TData& scan) const
{
    if (not ready_ || scan.points.empty())
        return;

    auto& pts = scan.points;
    const std::size_t n = pts.size();
    const float fz = cfg_.floor_z, tz = cfg_.top_z, dil = cfg_.dilate;
    // Footprint disc + near-floor skirt (robot frame). Radius m -> mm² to compare
    // against x²+y² directly.
    const float fr_mm2 = cfg_.footprint_radius * cfg_.footprint_radius * 1e6f;
    const float fp_z0 = cfg_.footprint_z_min, fp_z1 = cfg_.footprint_z_max;
    const float sr_mm2 = cfg_.skirt_radius * cfg_.skirt_radius * 1e6f;
    const float sk_z0 = cfg_.skirt_z_min, sk_z1 = cfg_.skirt_z_max;
    RTCScene scene = m_scene;
    // Mount device -> robot (points arrive in the device frame; the mesh is robot-frame).
    const Eigen::Matrix3f R = cfg_.mount.linear();
    const Eigen::Vector3f t = cfg_.mount.translation();

    // Hot path: compute the keep-mask in parallel (each Embree query is independent).
    std::vector<std::uint8_t> keep(n);
    #pragma omp parallel for schedule(static)
    for (std::size_t i = 0; i < n; ++i)
    {
        const auto& p = pts[i];
        const Eigen::Vector3f pr = R * Eigen::Vector3f(p.x, p.y, p.z) + t;   // -> robot frame (mm)
        const float rxy2 = pr.x() * pr.x() + pr.y() * pr.y();
        // Self returns: inside the body footprint disc, or inside the wider near-floor
        // skirt -> drop. Each is gated by its own z-band; radius 0 disables it.
        const bool in_footprint = fr_mm2 > 0.0f && rxy2 < fr_mm2 && pr.z() > fp_z0 && pr.z() < fp_z1;
        const bool in_skirt     = sr_mm2 > 0.0f && rxy2 < sr_mm2 && pr.z() > sk_z0 && pr.z() < sk_z1;
        keep[i] = (not in_footprint && not in_skirt && pr.z() > fz && pr.z() < tz
                   && not point_hits_mesh(scene, pr, dil)) ? 1u : 0u;
    }

    // Stable in-place compaction (cheap, sequential).
    std::size_t w = 0;
    for (std::size_t r = 0; r < n; ++r)
        if (keep[r])
        {
            if (w != r) pts[w] = pts[r];
            ++w;
        }
    pts.resize(w);
}
