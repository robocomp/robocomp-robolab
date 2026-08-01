/*
 *    Copyright (C) 2026 by RoboLab - UEx
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

// ============================================================================================
//  WebotsProtoLoader — build the LiDAR self-filter's Embree scene from a Webots `.proto`
// ============================================================================================
//
//  ##  WHY THIS EXISTS (measured, 2026-08-01)
//
//  The self-filter used to test returns against ONE mesh (`robots/Shadow/shadow.stl`, later
//  `shadow.obj`). A SINGLE MESH IS NOT THE ROBOT. `webots-shadow/protos/Shadow.proto` composes
//  the robot from a body Mesh PLUS four HingeJoint wheel assemblies (28 inline IndexedFaceSets)
//  PLUS camera/lidar solids, each at its own transform. Anything living in the OTHER solids was
//  simply absent from the filter's scene, so NO proximity radius, dilate value or ray trick could
//  ever remove its returns — which is why a 0.55 m blind disc got bolted on instead, and that disc
//  in turn blinded the robot to real obstacles within ~0.22 m of its skin.
//
//  So: assemble the scene from the WHOLE proto, every Shape at its composed transform.
//
//  ##  ⚠ THIS FILE PARSES THE *WEBOTS SIMULATION'S* ROBOT DEFINITION ⚠
//
//  `Shadow.proto` describes the robot AS SIMULATED. The REAL robot may be described by something
//  else entirely — a URDF, a different proto, or a CAD export — and nothing here checks that the
//  two agree. If they ever diverge, THE SELF-FILTER WILL BE CORRECT IN SIMULATION AND WRONG ON
//  HARDWARE: geometry present on the real robot but absent from the proto produces phantom
//  obstacles glued to the robot, and geometry present in the proto but absent on the real robot
//  silently deletes real returns near the body — the more dangerous of the two.
//
//  The proto path is therefore a PER-DEPLOYMENT INPUT (`[MeshFilter] proto_file`), NOT a constant.
//  Point it at whatever file describes the machine this driver is actually running on.
//
//  ##  FRAME
//
//  The scene is emitted in the ROBOT frame == the frame of the proto's root `Robot`/`Solid` node,
//  which is the frame `shadow.json` calls `body` and the frame the sensor mounts are expressed in
//  (`body->helios` = [0, -0.155, 1.075] appears verbatim as the helios `Lidar.translation` inside
//  the proto — that agreement is the check that this is the right frame).
//  The proto's own top-level `translation` field (default `0 0 0.033`) is the robot's placement
//  IN THE WORLD and is deliberately NOT applied; applying it would shift the whole robot 33 mm up
//  relative to the returns, which at Dilate=0.05 leaves 17 mm of margin — exactly the class of
//  silent offset that the 45 mm stale helios mount already cost this project.
//
//  ##  FAIL LOUDLY
//
//  Every rejection prints `file:line` and the construct, and `load()` returns false. There is no
//  "load what we can and carry on" path: a self-filter that silently lost a solid is worse than
//  one that refuses to start, because the loss shows up days later as a phantom obstacle.
//  (`URDFMeshLoader::loadSingleMesh` used to return true on a failed mesh load. That cost hours.)

#include <cstddef>
#include <string>
#include <vector>
#include <Eigen/Dense>
#include <embree4/rtcore.h>

class WebotsProtoLoader
{
public:
    struct Options
    {
        // Webots `boundingObject` is the PHYSICS collision envelope, not the rendered surface.
        // A simulated LiDAR cannot see it (Webots ray-casts the graphics), so for simulation
        // fidelity the default is OFF: the filter's scene should contain exactly what the sensor
        // can return from. Turn it ON only if you deliberately want to delete every return inside
        // the robot's conservative collision hull — which for Shadow is a 0.44 x 0.46 x 0.76 m box,
        // i.e. considerably fatter than the body it encloses (body radius at z=0.5 m is 0.150 m,
        // the box half-extent there is 0.22 x 0.23 m).
        bool include_bounding_objects = false;
        bool verbose                  = true;
    };

    struct Tri { Eigen::Vector3f a, b, c; };

    // Where each batch of triangles came from, for the "which solid contributed what" report.
    struct ShapeStat
    {
        std::string node_path;    // e.g. "Robot/Group/HingeJoint(WHEEL1)/Solid(wheel1)/Shape"
        std::string geom_type;    // "Mesh(shadow.stl)" | "IndexedFaceSet" | "Box" | ...
        std::string origin;       // "<file>:<line>"
        std::size_t tris = 0;
    };

    // No default argument: a nested aggregate's member initialisers are not yet available inside the
    // enclosing class definition, and more usefully it forces every call site to state, in writing,
    // whether it wants the collision envelope or the visible surface.
    WebotsProtoLoader(RTCDevice device, RTCScene scene, Options opt);
    ~WebotsProtoLoader();
    WebotsProtoLoader(const WebotsProtoLoader&)            = delete;
    WebotsProtoLoader& operator=(const WebotsProtoLoader&) = delete;

    // Parse `proto_path` and attach one RTCGeometry per Shape to the scene, vertices already
    // baked into the robot frame. Does NOT commit the scene (the caller owns that, as with
    // URDFMeshLoader). Returns false on ANY unsupported/unresolvable construct.
    bool load(const std::string& proto_path);

    // Baked triangles, robot frame, metres. Kept after load so the geometry can be audited
    // (per-bearing radius census, bounds) without re-parsing — ~4 MB for the Shadow assembly.
    [[nodiscard]] const std::vector<Tri>&       triangles() const { return m_tris; }
    [[nodiscard]] std::size_t                   triangle_count() const { return m_tris.size(); }
    [[nodiscard]] const std::vector<ShapeStat>& shapes() const { return m_shapes; }
    [[nodiscard]] const std::string&            error() const { return m_error; }

    // ------------------------------------------------------------------------------------------
    //  SECOND CONSUMER — the navigation footprint (`common/robot_footprint`)
    // ------------------------------------------------------------------------------------------
    //  The planner's exact collision test and the MPPI support function need the robot's PHYSICAL
    //  ENVELOPE, not what the LiDAR can see: the robot occupies the space its wheels sweep whether
    //  or not any sensor ever returns from them. Same assembly, different projection — which is the
    //  point. A hand-transcribed polygon and a filter mesh are two descriptions of one object, and
    //  they drifted 32.5 mm apart precisely because nothing derived one from the other.
    //
    //  `simplify_area_frac` is the fraction of area the simplified polygon may ADD. Simplification
    //  is OUTWARD ONLY — a vertex is removed by extending its two neighbouring edges to their
    //  intersection — so the result is always a strict SUPERSET of the true hull. A footprint that
    //  shrank under simplification would be an unannounced reduction in clearance.
    //
    //  Frame: robot frame, metres, x right / y forward, origin on the rotation centre.
    //  CCW winding, no repeated closing vertex.
    [[nodiscard]] std::vector<Eigen::Vector2f> xy_hull(float simplify_area_frac = 0.003f) const;

    // Same, restricted to triangles with any vertex in [z0, z1). Empty if the band holds no
    // geometry. Height-banded hulls matter because the single projected polygon takes the MAX over
    // ALL heights: Shadow's radius varies 2.4x with z, so the flat polygon makes the robot up to
    // 2.4x wider than it really is at, say, table-top height.
    [[nodiscard]] std::vector<Eigen::Vector2f> xy_hull_band(float z0, float z1,
                                                            float simplify_area_frac = 0.003f) const;

private:
    struct Impl;
    RTCDevice              m_device;
    RTCScene               m_scene;
    Options                m_opt;
    std::vector<Tri>       m_tris;
    std::vector<ShapeStat> m_shapes;
    std::string            m_error;
};
