/*
 *  proto_scene_probe — standalone acceptance harness for WebotsProtoLoader.
 *
 *  NOT part of the driver. It is deliberately outside src/CMakeLists.txt so it cannot drag the
 *  Ice/RoboComp stack into a geometry question. Build it by hand:
 *
 *    g++ -O2 -std=c++23 -o bin/proto_scene_probe \
 *        src/proto_scene_probe.cpp src/webots_proto_loader.cpp \
 *        -I/usr/include/eigen3 -lembree4 -lassimp
 *
 *  Run:  bin/proto_scene_probe [proto_path] [body_mesh_path]
 *
 *  It answers, with numbers rather than assertions:
 *    1. what the assembly contains (shapes, triangles, bounds) vs the body mesh alone;
 *    2. the per-30deg-bearing radius census at z in [0.49,0.54] for both — the gap test;
 *    3. whether the measured self-returns (radius ~0.001 m, z ~0.514 m, bpearl) are explained by
 *       the assembly and would now be filtered — the test that actually decides the bug;
 *    4. the derived navigation footprint: XY hull, simplified polygon, per-0.1 m height bands;
 *    5. the wheel-cylinder axis, which robot_footprint.h could not settle from the proto text.
 */

#include "webots_proto_loader.h"

#include <assimp/Importer.hpp>
#include <assimp/postprocess.h>
#include <assimp/scene.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <functional>
#include <numbers>
#include <string>
#include <vector>

using Tri = WebotsProtoLoader::Tri;
using V3  = Eigen::Vector3f;
using V2  = Eigen::Vector2f;

namespace
{
constexpr float kPi = std::numbers::pi_v<float>;

// Sample triangle SURFACES, not vertices. robot_footprint.h learned this the hard way: the STL base
// is a few very large triangles, so a vertex-only profile reports empty height bands that are not.
// The step is ADAPTIVE (2 mm) rather than a fixed subdivision count: a fixed count aliased the
// boundingObject box — its side face is 2 triangles spanning z 0..0.76, and an 8x8 grid puts samples
// at z = 0.475 and 0.570, straddling the 0.49..0.54 band and reporting it empty when it is solid.
void sample_tri(const Tri& t, const std::function<void(const V3&)>& f)
{
    const float e = std::max({(t.b - t.a).norm(), (t.c - t.b).norm(), (t.a - t.c).norm()});
    const int   n = std::clamp(int(std::ceil(e / 0.002f)), 1, 400);
    for (int i = 0; i <= n; ++i)
        for (int j = 0; i + j <= n; ++j)
        {
            const float a = float(i) / n, b = float(j) / n, c = 1.f - a - b;
            f(a * t.a + b * t.b + c * t.c);
        }
}

std::vector<Tri> load_mesh_tris(const std::string& path, float dz)
{
    std::vector<Tri> out;
    Assimp::Importer imp;
    const aiScene* s = imp.ReadFile(path, aiProcess_Triangulate | aiProcess_PreTransformVertices);
    if (not s or not s->HasMeshes()) { std::printf("  !! cannot read %s\n", path.c_str()); return out; }
    for (unsigned m = 0; m < s->mNumMeshes; ++m)
    {
        const aiMesh* me = s->mMeshes[m];
        for (unsigned k = 0; k < me->mNumFaces; ++k)
        {
            const aiFace& f = me->mFaces[k];
            if (f.mNumIndices != 3) continue;
            const auto V = [&](unsigned q) {
                const aiVector3D& v = me->mVertices[f.mIndices[q]];
                return V3(v.x, v.y, v.z + dz);
            };
            out.push_back({V(0), V(1), V(2)});
        }
    }
    return out;
}

struct Bounds { V3 lo{1e9f,1e9f,1e9f}, hi{-1e9f,-1e9f,-1e9f}; };
Bounds bounds_of(const std::vector<Tri>& ts)
{
    Bounds b;
    for (const auto& t : ts)
        for (const V3* p : {&t.a, &t.b, &t.c}) { b.lo = b.lo.cwiseMin(*p); b.hi = b.hi.cwiseMax(*p); }
    return b;
}

// The gap census the whole exercise is about.
void bearing_census(const char* label, const std::vector<Tri>& ts, float z0, float z1)
{
    constexpr int kBins = 12;                      // 30 deg each
    float rmin[kBins], rmax[kBins];
    long  cnt[kBins];
    for (int i = 0; i < kBins; ++i) { rmin[i] = 1e9f; rmax[i] = -1.f; cnt[i] = 0; }

    for (const auto& t : ts)
    {
        const float tz0 = std::min({t.a.z(), t.b.z(), t.c.z()});
        const float tz1 = std::max({t.a.z(), t.b.z(), t.c.z()});
        if (tz1 < z0 or tz0 > z1) continue;
        sample_tri(t, [&](const V3& p) {
            if (p.z() < z0 or p.z() > z1) return;
            const float bg = std::atan2(p.x(), p.y()) * 180.f / kPi;   // robot frame: x right, y forward
            int b = int(std::floor((bg + 180.f) / 30.f));
            b = std::clamp(b, 0, kBins - 1);
            const float r = std::hypot(p.x(), p.y());
            rmin[b] = std::min(rmin[b], r);
            rmax[b] = std::max(rmax[b], r);
            ++cnt[b];
        });
    }
    std::printf("  %s   z in [%.2f, %.2f]\n", label, z0, z1);
    for (int i = 0; i < kBins; ++i)
    {
        const int lo = -180 + 30 * i;
        if (cnt[i] == 0) std::printf("    [%+5d,%+5d)  ---- EMPTY ----\n", lo, lo + 30);
        else std::printf("    [%+5d,%+5d)  n=%-9ld rmin=%.4f rmax=%.4f\n", lo, lo + 30, cnt[i], rmin[i], rmax[i]);
    }
}

void print_poly(const char* name, const std::vector<V2>& h)
{
    // Area / inscribed / circumscribed, then the polygon in RobotFootprint::shadow() paste form.
    double area = 0; float rmax = 0, insc = 1e9f, xmax = 0, ymax = 0;
    const std::size_t n = h.size();
    for (std::size_t i = 0; i < n; ++i)
    {
        const V2& p = h[i]; const V2& q = h[(i + 1) % n];
        area += (double)p.x() * q.y() - (double)q.x() * p.y();
        rmax = std::max(rmax, p.norm());
        xmax = std::max(xmax, std::abs(p.x()));
        ymax = std::max(ymax, std::abs(p.y()));
        const V2 e = q - p; const float L = e.norm();
        if (L > 1e-9f) insc = std::min(insc, std::abs(e.x() * (0 - p.y()) - e.y() * (0 - p.x())) / L);
    }
    area = std::abs(0.5 * area);
    std::printf("  %s: %zu verts  area=%.4f m^2  inscribed=%.4f  circumscribed=%.4f  |x|max=%.4f  |y|max=%.4f\n",
                name, n, area, insc, rmax, xmax, ymax);
    std::printf("    f.poly_ = {\n");
    for (std::size_t i = 0; i < n; ++i)
        std::printf("%s{%+.4ff, %+.4ff},%s", (i % 4 == 0 ? "        " : " "), h[i].x(), h[i].y(),
                    ((i % 4 == 3 or i + 1 == n) ? "\n" : ""));
    std::printf("    };\n");
}

bool ray_hits(RTCScene s, const V3& o, const V3& d, float reach)
{
    RTCRayHit rh{};
    rh.ray.org_x = o.x(); rh.ray.org_y = o.y(); rh.ray.org_z = o.z();
    rh.ray.dir_x = d.x(); rh.ray.dir_y = d.y(); rh.ray.dir_z = d.z();
    rh.ray.tnear = 0.f; rh.ray.tfar = reach; rh.ray.mask = -1;
    rh.hit.geomID = RTC_INVALID_GEOMETRY_ID;
    rtcIntersect1(s, &rh, nullptr);
    return rh.hit.geomID != RTC_INVALID_GEOMETRY_ID;
}

float ray_dist(RTCScene s, const V3& o, const V3& d, float reach)
{
    RTCRayHit rh{};
    rh.ray.org_x = o.x(); rh.ray.org_y = o.y(); rh.ray.org_z = o.z();
    rh.ray.dir_x = d.x(); rh.ray.dir_y = d.y(); rh.ray.dir_z = d.z();
    rh.ray.tnear = 1e-4f; rh.ray.tfar = reach; rh.ray.mask = -1;
    rh.hit.geomID = RTC_INVALID_GEOMETRY_ID;
    rtcIntersect1(s, &rh, nullptr);
    return rh.hit.geomID == RTC_INVALID_GEOMETRY_ID ? -1.f : rh.ray.tfar;
}

// The self-filter's own predicate, copied from mesh_filter.cpp so the harness tests what ships.
bool self_filter_drops(RTCScene s, const V3& p, float dilate, const RTCBounds& b, float reach)
{
    static constexpr float D[6][3] = {{1,0,0},{-1,0,0},{0,1,0},{0,-1,0},{0,0,1},{0,0,-1}};
    for (const auto& d : D) if (ray_hits(s, p, V3(d[0], d[1], d[2]), dilate)) return true;
    if (p.x() < b.lower_x or p.x() > b.upper_x or p.y() < b.lower_y or p.y() > b.upper_y or
        p.z() < b.lower_z or p.z() > b.upper_z) return false;
    for (const auto& d : D) if (not ray_hits(s, p, V3(d[0], d[1], d[2]), reach)) return false;
    return true;
}

}  // namespace

int main(int argc, char** argv)
{
    const std::string proto = argc > 1 ? argv[1]
        : "/home/pbustos/robocomp/components/webots-shadow/protos/Shadow.proto";
    const std::string body  = argc > 2 ? argv[2]
        : "/home/pbustos/robocomp/components/webots-shadow/protos/meshes/shadow.stl";

    RTCDevice dev = rtcNewDevice(nullptr);

    struct Run { const char* name; bool bo; RTCScene scene; WebotsProtoLoader* ldr; };
    std::vector<Run> runs;
    for (bool bo : {false, true})
    {
        RTCScene sc = rtcNewScene(dev);
        auto* L = new WebotsProtoLoader(dev, sc, {.include_bounding_objects = bo, .verbose = not bo});
        std::printf("\n================ LOAD  boundingObject=%s ================\n", bo ? "INCLUDED" : "excluded");
        if (not L->load(proto)) { std::printf("LOAD FAILED: %s\n", L->error().c_str()); return 1; }
        rtcCommitScene(sc);
        runs.push_back({bo ? "assembly+boundingObject" : "assembly(visual)", bo, sc, L});
    }

    const auto body_tris = load_mesh_tris(body, 0.f);

    // ---------------------------------------------------------------- (c) counts + bounds
    std::printf("\n================ TRIANGLES AND BOUNDS ================\n");
    {
        const auto b = bounds_of(body_tris);
        std::printf("  body mesh alone (%s)\n", body.c_str());
        std::printf("    %zu tris   x[%.4f,%.4f] y[%.4f,%.4f] z[%.4f,%.4f]\n", body_tris.size(),
                    b.lo.x(), b.hi.x(), b.lo.y(), b.hi.y(), b.lo.z(), b.hi.z());
    }
    for (auto& r : runs)
    {
        const auto b = bounds_of(r.ldr->triangles());
        std::printf("  %s\n    %zu tris   x[%.4f,%.4f] y[%.4f,%.4f] z[%.4f,%.4f]\n", r.name,
                    r.ldr->triangle_count(), b.lo.x(), b.hi.x(), b.lo.y(), b.hi.y(), b.lo.z(), b.hi.z());
    }

    // ---------------------------------------------------------------- (b) the bearing gap census
    std::printf("\n================ BEARING CENSUS  z in [0.49, 0.54] ================\n");
    bearing_census("body mesh alone", body_tris, 0.49f, 0.54f);
    for (auto& r : runs) bearing_census(r.name, r.ldr->triangles(), 0.49f, 0.54f);

    // ------------------------------------------- the test that actually decides the reported bug
    // MEASURED (proximity_obstacles.csv, 984/984 cycles): the surviving self-returns sit at plan-view
    // radius ~0.001 m (i.e. ON the rotation axis) at z ~ 0.514 m, and they are bpearl returns at device
    // range ~0.235 m. The published "-90..+90 bearing" for them is NOT their bearing: it is the argmax
    // of the footprint SUPPORT function, which for a point at r~0 is pure noise. So the real questions
    // are (i) does the assembly contain anything a bpearl ray can hit near the axis at that height, and
    // (ii) would the shipped self-filter predicate now drop such a point.
    std::printf("\n================ MEASURED SELF-RETURN (r~0.001 m, z~0.514 m) ================\n");
    {
        const V3 bpearl(0.f, 0.140f, 0.670f);          // shadow.json body->bpearl
        const V3 helios(0.f, -0.155f, 1.075f);
        for (auto& r : runs)
        {
            RTCBounds b; rtcGetSceneBounds(r.scene, &b);
            const float reach = 1.05f * std::max({b.upper_x - b.lower_x, b.upper_y - b.lower_y,
                                                  b.upper_z - b.lower_z});
            std::printf("  %s\n", r.name);
            for (float z : {0.494f, 0.514f, 0.534f})
                for (float rad : {0.001f, 0.05f, 0.10f, 0.15f})
                {
                    const V3 p(rad, 0.f, z);
                    std::printf("    point r=%.3f z=%.3f -> filter %s\n", rad, z,
                                self_filter_drops(r.scene, p, 0.05f, b, reach) ? "DROPS" : "KEEPS  <-- survives");
                }
            // Would a bpearl beam aimed at the axis at z=0.514 terminate on the assembly, and where?
            for (const auto& [nm, org] : {std::pair{"bpearl", bpearl}, std::pair{"helios", helios}})
            {
                const V3 tgt(0.f, 0.f, 0.514f);
                const V3 d = (tgt - org).normalized();
                const float t = ray_dist(r.scene, org, d, 5.f);
                if (t < 0) std::printf("    %s beam -> axis@z=0.514 : NO HIT within 5 m\n", nm);
                else
                {
                    const V3 h = org + t * d;
                    std::printf("    %s beam -> axis@z=0.514 : hit at range %.4f m, robot xyz(%.4f,%.4f,%.4f) r=%.4f\n",
                                nm, t, h.x(), h.y(), h.z(), std::hypot(h.x(), h.y()));
                }
            }
        }
    }

    // ---------------------------------------------------------------- (d) navigation footprint
    std::printf("\n================ DERIVED NAVIGATION FOOTPRINT ================\n");
    for (auto& r : runs)
    {
        std::printf("  --- %s ---\n", r.name);
        print_poly("raw hull", r.ldr->xy_hull(0.f));
        print_poly("simplified (+0.3% area budget)", r.ldr->xy_hull(0.003f));
    }

    std::printf("\n================ HEIGHT-BANDED HULLS (0.1 m bands) ================\n");
    std::printf("  band            verts   |x|max   |y|max   circumsc  inscribed   area\n");
    for (auto& r : runs)
    {
        std::printf("  --- %s ---\n", r.name);
        for (int i = 0; i < 15; ++i)
        {
            const float z0 = 0.1f * i, z1 = z0 + 0.1f;
            const auto h = r.ldr->xy_hull_band(z0, z1, 0.003f);
            if (h.empty()) { std::printf("  [%.1f,%.1f)        --- no geometry ---\n", z0, z1); continue; }
            double area = 0; float rmax = 0, insc = 1e9f, xmax = 0, ymax = 0;
            for (std::size_t k = 0; k < h.size(); ++k)
            {
                const V2& p = h[k]; const V2& q = h[(k + 1) % h.size()];
                area += (double)p.x() * q.y() - (double)q.x() * p.y();
                rmax = std::max(rmax, p.norm()); xmax = std::max(xmax, std::abs(p.x()));
                ymax = std::max(ymax, std::abs(p.y()));
                const V2 e = q - p; const float L = e.norm();
                if (L > 1e-9f) insc = std::min(insc, std::abs(e.x() * -p.y() - e.y() * -p.x()) / L);
            }
            std::printf("  [%.1f,%.1f)      %4zu   %.4f   %.4f   %.4f    %.4f   %.4f\n",
                        z0, z1, h.size(), xmax, ymax, rmax, insc, std::abs(0.5 * area));
        }
    }

    // ---------------------------------------------------------------- (e) the wheel-axis question
    std::printf("\n================ WHEEL CYLINDER AXIS (robot_footprint.h open item) ================\n");
    for (auto& r : runs)
    {
        // Everything at |x| > 0.19 and z < 0.1 is wheel; the STL base half-width is 0.177.
        float xmax = 0, ymin = 1e9f, ymax = -1e9f, zmin = 1e9f, zmax = -1e9f;
        long n = 0;
        for (const auto& t : r.ldr->triangles())
            sample_tri(t, [&](const V3& p) {
                if (std::abs(p.x()) > 0.19f and p.z() < 0.12f)
                {
                    xmax = std::max(xmax, std::abs(p.x())); ymin = std::min(ymin, p.y());
                    ymax = std::max(ymax, p.y()); zmin = std::min(zmin, p.z());
                    zmax = std::max(zmax, p.z()); ++n;
                } });
        if (n == 0) { std::printf("  %-26s  no wheel geometry\n", r.name); continue; }
        std::printf("  %-26s  n=%-7ld outer |x|=%.4f  y[%.4f,%.4f]  z[%.4f,%.4f]\n",
                    r.name, n, xmax, ymin, ymax, zmin, zmax);
    }
    // Per-shape bounds for the boundingObject cylinders: this is what settles the axis, because
    // |x|max alone cannot separate "axis forward" (0.26, z half-extent 0.050) from "axis vertical"
    // (0.26, z half-extent 0.036).
    std::printf("\n  boundingObject shapes, individually:\n");
    {
        std::size_t first = 0;
        for (const auto& sh : runs[1].ldr->shapes())
        {
            if (sh.geom_type.find("boundingObject") != std::string::npos and sh.tris > 0)
            {
                V3 lo(1e9f,1e9f,1e9f), hi(-1e9f,-1e9f,-1e9f);
                for (std::size_t k = 0; k < sh.tris; ++k)
                {
                    const Tri& t = runs[1].ldr->triangles()[first + k];
                    for (const V3* q : {&t.a, &t.b, &t.c}) { lo = lo.cwiseMin(*q); hi = hi.cwiseMax(*q); }
                }
                std::printf("    %-8s x[%+.4f,%+.4f] y[%+.4f,%+.4f] z[%+.4f,%+.4f]  half-extents(%.4f,%.4f,%.4f)  %s\n",
                            sh.geom_type.c_str(), lo.x(), hi.x(), lo.y(), hi.y(), lo.z(), hi.z(),
                            0.5f*(hi.x()-lo.x()), 0.5f*(hi.y()-lo.y()), 0.5f*(hi.z()-lo.z()), sh.node_path.c_str());
            }
            first += sh.tris;
        }
    }
    std::printf("  candidates in robot_footprint.h: 0.2460 (axis lateral) / 0.2600 (forward or vertical)"
                " / 0.2716 (orientation-independent sphere bound, what is SHIPPING)\n");

    // Outermost-geometry-by-height: where do the wheels stop being the widest thing?
    std::printf("\n  outermost |x| by 0.05 m band (visual assembly): band  |x|max  source\n");
    for (int i = 0; i < 8; ++i)
    {
        const float z0 = 0.05f * i, z1 = z0 + 0.05f;
        float xa = 0, xb = 0;                             // whole assembly vs body-mesh-only
        const auto scan = [&](const std::vector<Tri>& ts, float& acc) {
            for (const auto& t : ts)
            {
                if (std::max({t.a.z(), t.b.z(), t.c.z()}) < z0 or std::min({t.a.z(), t.b.z(), t.c.z()}) > z1) continue;
                sample_tri(t, [&](const V3& p) {
                    if (p.z() >= z0 and p.z() < z1) acc = std::max(acc, std::abs(p.x())); });
            }
        };
        scan(runs[0].ldr->triangles(), xa);
        scan(body_tris, xb);
        std::printf("    [%.2f,%.2f)  assembly %.4f   body-only %.4f   %s\n", z0, z1, xa, xb,
                    xa > xb + 1e-4f ? "WHEELS OUTERMOST" : "body outermost");
    }

    for (auto& r : runs) { delete r.ldr; rtcReleaseScene(r.scene); }
    rtcReleaseDevice(dev);
    return 0;
}
