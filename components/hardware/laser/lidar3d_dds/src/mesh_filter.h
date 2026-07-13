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

// MeshFilter — self-filtering of a LiDAR scan against the robot body mesh, folded in
// from the standalone lidar3DFilter component to avoid its extra round-trip latency.
//
// It loads the robot mesh into an Embree scene once, then for each point does a fast
// vertical-band reject (floor_z / top_z) followed by a 6-ray Embree query approximating
// a `dilate`-radius sphere: a point within `dilate` of the mesh is dropped as a body hit.
// The per-point query is the hot path, so filter() runs it in parallel (OpenMP).
//
// Scope here is the STATIC Shadow body STL (no moving arms, no KinovaArm dependency) —
// arm returns are intentionally KEPT (used by self-calibration). P3Bot/URDF loading is
// supported by the loader but its dynamic-arm posing is out of scope for now.

#include <string>
#include <Eigen/Dense>
#include <embree4/rtcore.h>
#include <Lidar3D.h>          // RoboCompLidar3D::TData / TPoint

class URDFMeshLoader;         // pointer member; defined in urdf_mesh_loader.h

class MeshFilter
{
public:
    struct Config
    {
        std::string robot_name = "Shadow";   // "Shadow" (static STL) | "P3Bot" (URDF)
        std::string mesh_dir   = "robots";   // base dir holding <robot>/... meshes
        float floor_z = 110.0f;              // mm — drop points at/below this height (ground)
        float top_z   = 2100.0f;             // mm — drop points at/above this height (ceiling)
        float dilate  = 0.05f;               // m  — body self-radius (ray tfar)
        // Robot-frame footprint disc: the STL models only the central column, so returns
        // off the wider base/wheels/arm/protrusions sit past the mesh and survive the 5cm
        // mesh query. Drop any point whose XY distance from the robot origin is below
        // `footprint_radius`, within [footprint_z_min, footprint_z_max]. Inside the robot's
        // own footprint nothing can be a real obstacle, so this is navigation-safe.
        float footprint_radius = 0.0f;       // m  — XY disc around robot origin (0 disables)
        float footprint_z_min  = 110.0f;     // mm — disc active from this height…
        float footprint_z_max  = 1350.0f;    // mm — …up to this height (top of body)
        // Near-floor skirt: a second, WIDER disc confined to a low z-band. Catches
        // near-floor self / ground-reflection returns that sit past the body footprint,
        // without blanking upright obstacles (whose returns above skirt_z_max stay).
        float skirt_radius = 0.0f;           // m  — wider XY disc near the floor (0 disables)
        float skirt_z_min  = 110.0f;         // mm — skirt active from this height…
        float skirt_z_max  = 350.0f;         // mm — …up to this (low) height
        // Mount (device -> robot). Points arrive in the sensor/device frame (mm); the
        // filter transforms each into the robot frame (where the mesh lives) to test.
        Eigen::Affine3f mount = Eigen::Affine3f::Identity();
    };

    MeshFilter();
    ~MeshFilter();
    MeshFilter(const MeshFilter&) = delete;
    MeshFilter& operator=(const MeshFilter&) = delete;

    // Build the Embree scene from the robot mesh. Returns false if the mesh can't load.
    bool init(const Config& cfg);
    [[nodiscard]] bool ready() const { return ready_; }

    // Remove, in place, points outside [floor_z, top_z] or within `dilate` of the mesh.
    void filter(RoboCompLidar3D::TData& scan) const;

private:
    RTCDevice       m_device = nullptr;
    RTCScene        m_scene  = nullptr;
    URDFMeshLoader* m_loader = nullptr;
    Config          cfg_;
    bool            ready_ = false;
};
