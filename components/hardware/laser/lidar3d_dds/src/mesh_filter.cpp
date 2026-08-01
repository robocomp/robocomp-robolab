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
#include "webots_proto_loader.h"

#include <cstdint>
#include <cstdio>
#include <iostream>
#include <vector>

MeshFilter::MeshFilter() = default;

MeshFilter::~MeshFilter()
{
    delete m_loader;
    delete m_proto_loader;
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

    bool loaded = false;
    if (cfg.geometry_source == "proto")
    {
        // ONE proto describes the WHOLE robot: body mesh + wheel assemblies + sensor solids, each
        // at its own composed transform. See webots_proto_loader.h for why a single mesh is not the
        // robot, and for the simulation-vs-hardware caveat on `proto_file`.
        if (cfg.proto_file.empty())
        {
            std::cerr << "[MeshFilter] geometry_source=\"proto\" but MeshFilter.proto_file is empty"
                      << std::endl;
            return false;
        }
        m_proto_loader = new WebotsProtoLoader(
            m_device, m_scene,
            {.include_bounding_objects = cfg.proto_include_bounding_objects, .verbose = true});
        loaded = m_proto_loader->load(cfg.proto_file);
        if (loaded)
            std::printf("[MeshFilter] proto='%s' %zu shapes, %zu triangles\n", cfg.proto_file.c_str(),
                        m_proto_loader->shapes().size(), m_proto_loader->triangle_count());
    }
    else if (cfg.geometry_source == "mesh")
    {
        m_loader = new URDFMeshLoader(m_device, m_scene);
        Eigen::Matrix4f placement = Eigen::Matrix4f::Identity();
        placement(0, 3) = cfg.mesh_off_x;
        placement(1, 3) = cfg.mesh_off_y;
        placement(2, 3) = cfg.mesh_off_z;

        if (cfg.robot_name == "Shadow")
            loaded = m_loader->loadSingleMesh(cfg.mesh_dir + "/" + cfg.mesh_file, placement);
        else if (cfg.robot_name == "P3Bot")
            loaded = m_loader->loadURDF(cfg.mesh_dir + "/P3Bot/P3Bot.urdf", cfg.mesh_dir + "/P3Bot/");
        else
            std::cerr << "[MeshFilter] unknown Robot.name '" << cfg.robot_name << "'" << std::endl;
        if (loaded)
            std::printf("[MeshFilter] mesh='%s' offset(%.4f,%.4f,%.4f)\n",
                        cfg.mesh_file.c_str(), cfg.mesh_off_x, cfg.mesh_off_y, cfg.mesh_off_z);
    }
    else
        std::cerr << "[MeshFilter] unknown MeshFilter.geometry_source '" << cfg.geometry_source
                  << "' (expected \"proto\" or \"mesh\")" << std::endl;

    if (not loaded)
    {
        std::cerr << "[MeshFilter] failed to build the self-filter scene for '" << cfg.robot_name
                  << "' from source '" << cfg.geometry_source << "'" << std::endl;
        return false;
    }

    rtcCommitScene(m_scene);

    RTCBounds b;
    rtcGetSceneBounds(m_scene, &b);
    m_bounds = b;
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
static constexpr float kAxisDirs[6][3] = {{1,0,0},{-1,0,0},{0,1,0},{0,-1,0},{0,0,1},{0,0,-1}};

// Does a ray from `org` along `d` hit the mesh within `reach` metres?
inline bool ray_hits(RTCScene scene, const float org[3], const float d[3], float reach)
{
    RTCRayHit rh{};
    rh.ray.org_x = org[0]; rh.ray.org_y = org[1]; rh.ray.org_z = org[2];
    rh.ray.dir_x = d[0];   rh.ray.dir_y = d[1];   rh.ray.dir_z = d[2];
    rh.ray.tnear = 0.0f;   rh.ray.tfar = reach;   rh.ray.time = 0.0f;
    rh.ray.mask  = -1;     rh.ray.flags = 0;
    rh.hit.geomID = RTC_INVALID_GEOMETRY_ID;
    rtcIntersect1(scene, &rh, nullptr);
    return rh.hit.geomID != RTC_INVALID_GEOMETRY_ID;
}

// True if the ROBOT-frame point (mm) is a SELF return. Two questions, because one does not imply the other:
//
//   (1) NEAR THE SKIN — any of 6 orthogonal rays hits within `dilate` (max corner error ~1.4·dilate).
//       This is what the filter has always done.
//
//   (2) INSIDE THE BODY — the test that was missing, and its absence is why the 0.55 m disc existed.
//       (1) only detects proximity to a SURFACE, so a point deeper than `dilate` inside the body finds
//       nothing within tfar along any axis and was KEPT. Measured live 2026-08-01 with the disc removed:
//       ~600 returns per cycle, on EVERY cycle, sitting inside the footprint at z ~ 0.515 m — the robot's
//       waist, where the body is only 0.150 m in radius, so a return on the axis is ~0.15 m inside the
//       surface and three times beyond the 0.05 m query. The disc had been absorbing all of them.
//       ENCLOSURE, not parity: a point inside a solid has mesh in EVERY axis direction, and since this
//       scene contains nothing but the robot, "surrounded" can only mean "inside the robot". Parity would
//       be the textbook test but needs a watertight mesh, which a display OBJ is not required to be.
//       Bounded by the scene's own bbox, so the extra rays run only for returns already inside the robot's
//       box — for everything else this costs one comparison.
inline bool point_hits_mesh(RTCScene scene, const Eigen::Vector3f& p_mm, float dilate,
                            const RTCBounds& bounds, float enclose_reach)
{
    const float org[3] = { p_mm.x() * 0.001f, p_mm.y() * 0.001f, p_mm.z() * 0.001f };   // mm -> m
    for (const auto& d : kAxisDirs)
        if (ray_hits(scene, org, d, dilate))
            return true;

    if (org[0] < bounds.lower_x or org[0] > bounds.upper_x or
        org[1] < bounds.lower_y or org[1] > bounds.upper_y or
        org[2] < bounds.lower_z or org[2] > bounds.upper_z)
        return false;                       // outside the robot's own box: cannot be inside it

    for (const auto& d : kAxisDirs)
        if (not ray_hits(scene, org, d, enclose_reach))
            return false;                   // escaped along this axis ⇒ not enclosed by the body
    return true;
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
    const RTCBounds bounds = m_bounds;
    // Long enough to cross the robot from any interior point; the mesh spans ~1.42 m.
    const float enclose_reach = 1.05f * std::max({bounds.upper_x - bounds.lower_x,
                                                  bounds.upper_y - bounds.lower_y,
                                                  bounds.upper_z - bounds.lower_z});
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
                   && not point_hits_mesh(scene, pr, dil, bounds, enclose_reach)) ? 1u : 0u;
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
