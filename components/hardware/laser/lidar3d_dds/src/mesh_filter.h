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
