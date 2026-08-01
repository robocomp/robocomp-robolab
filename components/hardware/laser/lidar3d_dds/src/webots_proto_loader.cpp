/*
 *    Copyright (C) 2026 by RoboLab - UEx
 *    Part of RoboComp — GPLv3 (see webots_proto_loader.h).
 */

#include "webots_proto_loader.h"

#include <assimp/Importer.hpp>
#include <assimp/postprocess.h>
#include <assimp/scene.h>

#include <algorithm>
#include <cctype>
#include <charconv>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <map>
#include <memory>
#include <numbers>
#include <optional>
#include <set>
#include <sstream>
#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

namespace fs = std::filesystem;

// ================================================================================================
//  VRML/Webots-proto tokenizer + parser
// ================================================================================================
namespace
{

struct Token
{
    std::string text;
    int         line     = 0;
    bool        is_str   = false;   // came from "..." — never confusable with a field name
    bool        is_punct = false;   // { } [ ]
};

// Locale-independent number parse. The machine here runs a Spanish locale, where a comma-decimal
// strtod would silently truncate "0.0311" to 0 — from_chars is immune by construction.
bool to_double(std::string_view s, double& out)
{
    if (s.empty()) return false;
    const char* b = s.data();
    const char* e = s.data() + s.size();
    auto [p, ec] = std::from_chars(b, e, out);
    return ec == std::errc{} and p == e;
}
bool is_number(std::string_view s) { double d; return to_double(s, d); }

struct Lexer
{
    std::string file;
    std::string src;
    std::vector<Token> toks;
    std::string err;

    bool run()
    {
        int line = 1;
        const std::size_t n = src.size();
        for (std::size_t i = 0; i < n;)
        {
            const char c = src[i];
            if (c == '\n') { ++line; ++i; continue; }
            if (std::isspace(static_cast<unsigned char>(c)) or c == ',') { ++i; continue; }
            // Webots template statements/expressions are JavaScript evaluated by Webots itself.
            // We are not a JS engine; a silently-skipped %<= >% could be the very transform that
            // places a solid, so this is fatal by design.
            if (c == '%' and i + 1 < n and (src[i + 1] == '<' or src[i + 1] == '{'))
            {
                err = file + ":" + std::to_string(line) +
                      ": template expression '" + src.substr(i, std::min<std::size_t>(24, n - i)) +
                      "...' — Webots proto templates (JavaScript) are not evaluated by this loader";
                return false;
            }
            if (c == '#')                                     // comment to end of line
            {
                while (i < n and src[i] != '\n') ++i;
                continue;
            }
            if (c == '"')
            {
                std::string s;
                ++i;
                while (i < n and src[i] != '"')
                {
                    if (src[i] == '\\' and i + 1 < n) { s += src[i + 1]; i += 2; continue; }
                    if (src[i] == '\n') ++line;
                    s += src[i++];
                }
                if (i >= n) { err = file + ":" + std::to_string(line) + ": unterminated string"; return false; }
                ++i;
                toks.push_back({std::move(s), line, true, false});
                continue;
            }
            if (c == '{' or c == '}' or c == '[' or c == ']')
            {
                toks.push_back({std::string(1, c), line, false, true});
                ++i;
                continue;
            }
            std::size_t j = i;
            while (j < n and not std::isspace(static_cast<unsigned char>(src[j])) and src[j] != ',' and
                   src[j] != '{' and src[j] != '}' and src[j] != '[' and src[j] != ']' and src[j] != '#')
                ++j;
            toks.push_back({src.substr(i, j - i), line, false, false});
            i = j;
        }
        return true;
    }
};

struct PNode;
using PNodePtr = std::shared_ptr<PNode>;

struct PValue
{
    std::vector<PNodePtr> nodes;      // SFNode / MFNode
    std::vector<Token>    values;     // SFxxx / MFxxx scalars, strings, bools
    std::string           is_name;    // non-empty => `IS <name>` forwarding
    bool                  is_null = false;
};

struct PNode
{
    std::string type;                 // "Solid", "Shape", "__USE__", ...
    std::string def;                  // DEF name, or the USE target when type == "__USE__"
    std::string file;
    int         line = 0;
    std::vector<std::pair<std::string, PValue>> fields;

    [[nodiscard]] const PValue* field(std::string_view name) const
    {
        for (const auto& [k, v] : fields)
            if (k == name) return &v;
        return nullptr;
    }
    [[nodiscard]] std::string where() const { return file + ":" + std::to_string(line); }
};

struct ProtoDef
{
    std::string                                 name;
    std::string                                 file;
    std::vector<std::pair<std::string, PValue>> interface;   // declared fields + defaults
    PNodePtr                                    body;
    // EXTERNPROTO declarations seen in this file, protoName -> resolved absolute path.
    std::map<std::string, std::string>          externs;
    fs::path                                    dir;         // for relative url / proto resolution
};

class Parser
{
public:
    Parser(std::string file, std::vector<Token> toks) : m_file(std::move(file)), m_t(std::move(toks)) {}

    std::string err;

    // Parses a whole `.proto` file: [EXTERNPROTO ...]* PROTO Name [ iface ] { body }
    bool parse_proto_file(ProtoDef& out)
    {
        out.file = m_file;
        out.dir  = fs::path(m_file).parent_path();

        while (m_i < m_t.size() and cur().text == "EXTERNPROTO")
        {
            ++m_i;
            if (m_i >= m_t.size() or not cur().is_str)
                return fail("EXTERNPROTO must be followed by a quoted path");
            out.externs[fs::path(cur().text).stem().string()] = cur().text;   // resolved later
            ++m_i;
        }
        if (m_i >= m_t.size() or cur().text != "PROTO")
            return fail("expected `PROTO` (only .proto files are supported as the geometry source)");
        ++m_i;
        if (m_i >= m_t.size()) return fail("expected proto name after PROTO");
        out.name = cur().text; ++m_i;

        if (m_i >= m_t.size() or cur().text != "[") return fail("expected `[` opening the PROTO interface");
        ++m_i;
        while (m_i < m_t.size() and cur().text != "]")
        {
            static const std::set<std::string> kFieldKw = {
                "field", "vrmlField", "exposedField", "w3dField", "deprecatedField",
                "unconnectedField", "hiddenField"};
            const std::string kw = cur().text;
            if (not kFieldKw.contains(kw))
                return fail("unsupported PROTO interface keyword `" + kw + "`");
            ++m_i;
            if (m_i + 1 >= m_t.size()) return fail("truncated PROTO interface entry");
            ++m_i;                                              // the SF*/MF* type, unused
            const std::string fname = cur().text; ++m_i;
            PValue def;
            if (not parse_value(def)) return false;
            out.interface.emplace_back(fname, std::move(def));
        }
        if (m_i >= m_t.size()) return fail("unterminated PROTO interface");
        ++m_i;                                                  // ]

        if (m_i >= m_t.size() or cur().text != "{") return fail("expected `{` opening the PROTO body");
        ++m_i;
        out.body = parse_node();
        if (not out.body) return false;
        if (m_i >= m_t.size() or cur().text != "}") return fail("expected `}` closing the PROTO body");
        return true;
    }

private:
    std::string        m_file;
    std::vector<Token> m_t;
    std::size_t        m_i = 0;

    const Token& cur() const { return m_t[m_i]; }
    const Token* peek(std::size_t k = 1) const { return m_i + k < m_t.size() ? &m_t[m_i + k] : nullptr; }

    bool fail(const std::string& msg)
    {
        const int ln = m_i < m_t.size() ? m_t[m_i].line : (m_t.empty() ? 0 : m_t.back().line);
        err = m_file + ":" + std::to_string(ln) + ": " + msg;
        return false;
    }

    PNodePtr parse_node()
    {
        if (m_i >= m_t.size()) { fail("unexpected end of file, expected a node"); return nullptr; }
        auto n   = std::make_shared<PNode>();
        n->file  = m_file;
        n->line  = cur().line;

        if (cur().text == "USE")
        {
            ++m_i;
            if (m_i >= m_t.size()) { fail("USE without a name"); return nullptr; }
            n->type = "__USE__";
            n->def  = cur().text;
            ++m_i;
            return n;
        }
        if (cur().text == "DEF")
        {
            ++m_i;
            if (m_i >= m_t.size()) { fail("DEF without a name"); return nullptr; }
            n->def = cur().text; ++m_i;
        }
        if (m_i >= m_t.size()) { fail("expected a node type"); return nullptr; }
        n->type = cur().text; ++m_i;
        if (m_i >= m_t.size() or cur().text != "{") { fail("expected `{` after node type `" + n->type + "`"); return nullptr; }
        ++m_i;

        while (m_i < m_t.size() and cur().text != "}")
        {
            if (cur().is_punct or cur().is_str) { fail("expected a field name, got `" + cur().text + "`"); return nullptr; }
            const std::string fname = cur().text; ++m_i;
            PValue v;
            if (not parse_value(v)) return nullptr;
            n->fields.emplace_back(fname, std::move(v));
        }
        if (m_i >= m_t.size()) { fail("unterminated node `" + n->type + "`"); return nullptr; }
        ++m_i;                                                  // }
        return n;
    }

    // A field value: `IS x` | NULL | [ ... ] | <node> | <scalars...>
    bool parse_value(PValue& v)
    {
        if (m_i >= m_t.size()) return fail("expected a field value");

        if (cur().text == "IS" and not cur().is_str)
        {
            ++m_i;
            if (m_i >= m_t.size()) return fail("`IS` without a field name");
            v.is_name = cur().text; ++m_i;
            return true;
        }
        if (cur().text == "NULL" and not cur().is_str) { v.is_null = true; ++m_i; return true; }

        if (cur().text == "[")
        {
            ++m_i;
            while (m_i < m_t.size() and cur().text != "]")
            {
                if (cur().is_str or is_number(cur().text)) { v.values.push_back(cur()); ++m_i; }
                else if (cur().is_punct) return fail("unexpected `" + cur().text + "` inside a list");
                else
                {
                    auto n = parse_node();
                    if (not n) return false;
                    v.nodes.push_back(std::move(n));
                }
            }
            if (m_i >= m_t.size()) return fail("unterminated `[`");
            ++m_i;
            return true;
        }

        // SFNode: `Type {`, `DEF n Type {`, `USE n`
        const Token* p1 = peek(1);
        const bool starts_node = not cur().is_str and not cur().is_punct and
                                 (cur().text == "DEF" or cur().text == "USE" or (p1 and p1->text == "{"));
        if (starts_node)
        {
            auto n = parse_node();
            if (not n) return false;
            v.nodes.push_back(std::move(n));
            return true;
        }

        // Scalars. First token may also be TRUE/FALSE; subsequent ones continue a numeric/string run.
        if (cur().is_str or is_number(cur().text) or cur().text == "TRUE" or cur().text == "FALSE")
        {
            v.values.push_back(cur()); ++m_i;
            while (m_i < m_t.size() and not cur().is_punct and (cur().is_str or is_number(cur().text)))
            { v.values.push_back(cur()); ++m_i; }
            return true;
        }
        return fail("unrecognised field value `" + cur().text + "`");
    }
};

// ------------------------------------------------------------------------------------------------
//  Small geometry helpers
// ------------------------------------------------------------------------------------------------

Eigen::Matrix4f rot_from_axis_angle(double ax, double ay, double az, double angle)
{
    Eigen::Vector3f axis(static_cast<float>(ax), static_cast<float>(ay), static_cast<float>(az));
    Eigen::Matrix4f m = Eigen::Matrix4f::Identity();
    const float nrm = axis.norm();
    // Webots normalises the axis; the proto does NOT always store a unit one — bpearl is written
    // `rotation -1 -1 0 3.14159`. Taking that at face value would scale-and-shear the sensor frame.
    if (nrm < 1e-12f or std::abs(angle) < 1e-12) return m;
    axis /= nrm;
    m.block<3, 3>(0, 0) = Eigen::AngleAxisf(static_cast<float>(angle), axis).toRotationMatrix();
    return m;
}

}  // namespace

// ================================================================================================
//  Impl — proto registry, traversal, tessellation
// ================================================================================================

struct WebotsProtoLoader::Impl
{
    WebotsProtoLoader* self;
    Options            opt;
    std::string        err;

    std::map<std::string, std::shared_ptr<ProtoDef>> protos;      // by proto name
    std::vector<fs::path>                            search_dirs;
    std::set<std::string>                            unknown_types;   // reported once each

    // Bindings for the proto instance currently being expanded (`IS` resolution) and the DEF scope.
    struct Ctx
    {
        const ProtoDef*                         proto = nullptr;
        std::map<std::string, const PValue*>     binds;      // field name -> concrete value
        std::shared_ptr<std::map<std::string, PNodePtr>> defs; // DEF scope (shared within a body)
        const Ctx*                              bind_owner = nullptr;  // where `binds` values live
    };

    bool set_err(const std::string& where, const std::string& msg)
    {
        err = where + ": " + msg;
        std::fprintf(stderr, "[WebotsProtoLoader] REJECTED  %s\n", err.c_str());
        return false;
    }

    // --------------------------------------------------------------------------------------
    //  proto files
    // --------------------------------------------------------------------------------------
    std::shared_ptr<ProtoDef> load_proto_file(const fs::path& path, std::string& why)
    {
        const std::string key = fs::weakly_canonical(path).string();
        for (auto& [n, p] : protos)
            if (p->file == key) return p;

        std::ifstream f(path, std::ios::binary);
        if (not f) { why = "cannot open `" + path.string() + "`"; return nullptr; }
        std::ostringstream ss; ss << f.rdbuf();

        Lexer lx{key, ss.str(), {}, {}};
        if (not lx.run()) { why = lx.err; return nullptr; }

        Parser ps(key, std::move(lx.toks));
        auto def = std::make_shared<ProtoDef>();
        if (not ps.parse_proto_file(*def)) { why = ps.err; return nullptr; }
        def->file = key;
        def->dir  = fs::path(key).parent_path();

        // Resolve this file's EXTERNPROTO declarations to absolute paths (declaration only —
        // the file is not parsed until something actually instantiates it).
        for (auto& [pname, rel] : def->externs)
        {
            fs::path cand = rel;
            if (cand.is_relative()) cand = def->dir / rel;
            def->externs[pname] = fs::weakly_canonical(cand).string();
        }
        protos[def->name] = def;
        if (std::ranges::find(search_dirs, def->dir) == search_dirs.end())
            search_dirs.push_back(def->dir);
        return def;
    }

    // A node type is a PROTO if it was EXTERNPROTO'd, already registered, or `<Type>.proto` exists.
    std::shared_ptr<ProtoDef> find_proto(const std::string& type, const ProtoDef* from, std::string& why)
    {
        if (auto it = protos.find(type); it != protos.end()) return it->second;
        if (from)
            if (auto it = from->externs.find(type); it != from->externs.end())
            {
                auto p = load_proto_file(it->second, why);
                if (not p) why = "EXTERNPROTO `" + type + "` -> " + why;
                return p;
            }
        for (const auto& d : search_dirs)
        {
            const fs::path cand = d / (type + ".proto");
            if (fs::exists(cand)) return load_proto_file(cand, why);
        }
        return nullptr;
    }

    // --------------------------------------------------------------------------------------
    //  field access with `IS` resolution
    // --------------------------------------------------------------------------------------
    const PValue* resolve(const PValue* v, const Ctx& ctx, const PNode& owner, bool& ok)
    {
        ok = true;
        if (not v) return nullptr;
        if (v->is_name.empty()) return v;
        auto it = ctx.binds.find(v->is_name);
        if (it == ctx.binds.end())
        {
            ok = false;
            set_err(owner.where(), "`IS " + v->is_name + "` has no matching field in PROTO `" +
                                   (ctx.proto ? ctx.proto->name : std::string("<none>")) + "`");
            return nullptr;
        }
        return it->second;
    }

    bool nums(const PValue* v, std::size_t want, std::vector<double>& out, const PNode& owner,
              std::string_view fname)
    {
        out.clear();
        if (not v) return true;
        for (const auto& t : v->values)
        {
            double d;
            if (not to_double(t.text, d))
                return set_err(owner.where(), std::string("field `") + std::string(fname) +
                                              "`: `" + t.text + "` is not a number");
            out.push_back(d);
        }
        if (out.empty()) return true;
        if (out.size() != want)
            return set_err(owner.where(), std::string("field `") + std::string(fname) + "` expects " +
                                          std::to_string(want) + " numbers, got " + std::to_string(out.size()));
        return true;
    }

    // Local transform contributed by a node's own translation / rotation / scale.
    bool local_xform(const PNode& n, const Ctx& ctx, Eigen::Matrix4f& out)
    {
        out = Eigen::Matrix4f::Identity();
        bool ok = true;
        std::vector<double> v;

        const PValue* t = resolve(n.field("translation"), ctx, n, ok); if (not ok) return false;
        if (not nums(t, 3, v, n, "translation")) return false;
        if (v.size() == 3) { out(0,3)=(float)v[0]; out(1,3)=(float)v[1]; out(2,3)=(float)v[2]; }

        const PValue* r = resolve(n.field("rotation"), ctx, n, ok); if (not ok) return false;
        if (not nums(r, 4, v, n, "rotation")) return false;
        if (v.size() == 4) out.block<3,3>(0,0) = rot_from_axis_angle(v[0], v[1], v[2], v[3]).block<3,3>(0,0);

        const PValue* s = resolve(n.field("scale"), ctx, n, ok); if (not ok) return false;
        if (not nums(s, 3, v, n, "scale")) return false;
        if (v.size() == 3)
            out.block<3,3>(0,0) *= Eigen::Vector3f((float)v[0], (float)v[1], (float)v[2]).asDiagonal();
        return true;
    }

    // --------------------------------------------------------------------------------------
    //  tessellation
    // --------------------------------------------------------------------------------------
    std::vector<Tri>* out_tris = nullptr;

    void emit(const Eigen::Matrix4f& M, const Eigen::Vector3f& a, const Eigen::Vector3f& b,
              const Eigen::Vector3f& c)
    {
        // Explicit Vector3f return type, NOT deduced: `(M * v).head<3>()` deduces to an Eigen Block
        // expression that references the temporary Vector4f, which dies at the return — the classic
        // Eigen `auto` dangle. It showed up here as an entire assembly of NaN vertices.
        const auto xf = [&](const Eigen::Vector3f& p) -> Eigen::Vector3f {
            return (M * Eigen::Vector4f(p.x(), p.y(), p.z(), 1.f)).head<3>();
        };
        out_tris->push_back({xf(a), xf(b), xf(c)});
    }

    void tess_box(const Eigen::Matrix4f& M, float sx, float sy, float sz)
    {
        const float x = sx * 0.5f, y = sy * 0.5f, z = sz * 0.5f;
        const Eigen::Vector3f v[8] = {{-x,-y,-z},{ x,-y,-z},{ x, y,-z},{-x, y,-z},
                                      {-x,-y, z},{ x,-y, z},{ x, y, z},{-x, y, z}};
        static const int f[12][3] = {{0,2,1},{0,3,2},{4,5,6},{4,6,7},{0,1,5},{0,5,4},
                                     {1,2,6},{1,6,5},{2,3,7},{2,7,6},{3,0,4},{3,4,7}};
        for (const auto& t : f) emit(M, v[t[0]], v[t[1]], v[t[2]]);
    }

    // VRML/Webots convention: the cylinder axis is +Y, centred on the origin.
    void tess_cylinder(const Eigen::Matrix4f& M, float h, float r, int sub, bool top, bool bottom, bool side)
    {
        sub = std::max(3, sub);
        const float hy = h * 0.5f;
        const auto  P  = [&](int i, float y) {
            const float a = 2.f * std::numbers::pi_v<float> * static_cast<float>(i) / static_cast<float>(sub);
            return Eigen::Vector3f(r * std::cos(a), y, r * std::sin(a));
        };
        for (int i = 0; i < sub; ++i)
        {
            const int j = (i + 1) % sub;
            if (side)
            {
                emit(M, P(i,-hy), P(j,-hy), P(j, hy));
                emit(M, P(i,-hy), P(j, hy), P(i, hy));
            }
            if (top)    emit(M, Eigen::Vector3f(0, hy, 0), P(i, hy), P(j, hy));
            if (bottom) emit(M, Eigen::Vector3f(0,-hy, 0), P(j,-hy), P(i,-hy));
        }
    }

    void tess_sphere(const Eigen::Matrix4f& M, float r, int sub)
    {
        const int nu = std::max(8, sub * 4), nv = std::max(4, sub * 2);
        const auto P = [&](int i, int j) {
            const float u = 2.f * std::numbers::pi_v<float> * i / nu;
            const float v = std::numbers::pi_v<float> * j / nv;
            return Eigen::Vector3f(r * std::sin(v) * std::cos(u), r * std::cos(v), r * std::sin(v) * std::sin(u));
        };
        for (int i = 0; i < nu; ++i)
            for (int j = 0; j < nv; ++j)
            {
                const int i2 = (i + 1) % nu, j2 = j + 1;
                if (j != 0)      emit(M, P(i,j), P(i2,j), P(i2,j2));
                if (j2 != nv)    emit(M, P(i,j), P(i2,j2), P(i,j2));
            }
    }

    bool tess_ifs(const PNode& n, const Ctx& ctx, const Eigen::Matrix4f& M, std::size_t& n_tris)
    {
        bool ok = true;
        const PValue* coordv = resolve(n.field("coord"), ctx, n, ok); if (not ok) return false;
        const PValue* idxv   = resolve(n.field("coordIndex"), ctx, n, ok); if (not ok) return false;
        if (not coordv or coordv->nodes.empty())
            return set_err(n.where(), "IndexedFaceSet without a `coord Coordinate` node");
        if (not idxv or idxv->values.empty())
            return set_err(n.where(), "IndexedFaceSet without `coordIndex`");

        PNodePtr coord = coordv->nodes.front();
        if (coord->type == "__USE__")
        {
            auto it = ctx.defs->find(coord->def);
            if (it == ctx.defs->end())
                return set_err(coord->where(), "USE `" + coord->def + "` has no matching DEF in scope");
            coord = it->second;
        }
        if (coord->type != "Coordinate")
            return set_err(coord->where(), "IndexedFaceSet.coord must be a `Coordinate`, got `" + coord->type + "`");

        const PValue* ptv = resolve(coord->field("point"), ctx, *coord, ok); if (not ok) return false;
        if (not ptv or ptv->values.size() % 3 != 0)
            return set_err(coord->where(), "Coordinate.point must be a multiple of 3 numbers");

        std::vector<Eigen::Vector3f> pts(ptv->values.size() / 3);
        for (std::size_t i = 0; i < pts.size(); ++i)
        {
            double x, y, z;
            if (not to_double(ptv->values[3*i+0].text, x) or not to_double(ptv->values[3*i+1].text, y) or
                not to_double(ptv->values[3*i+2].text, z))
                return set_err(coord->where(), "Coordinate.point holds a non-numeric entry");
            pts[i] = {(float)x, (float)y, (float)z};
        }

        std::vector<int> face;
        const std::size_t before = out_tris->size();
        const auto flush = [&]() -> bool {
            for (std::size_t k = 1; k + 1 < face.size(); ++k)     // fan-triangulate (faces are convex here)
                emit(M, pts[face[0]], pts[face[k]], pts[face[k+1]]);
            face.clear();
            return true;
        };
        for (const auto& t : idxv->values)
        {
            double d;
            if (not to_double(t.text, d))
                return set_err(n.where(), "coordIndex holds a non-numeric entry `" + t.text + "`");
            const int idx = static_cast<int>(d);
            if (idx < 0) { flush(); continue; }
            if (static_cast<std::size_t>(idx) >= pts.size())
                return set_err(n.where(), "coordIndex " + std::to_string(idx) + " is out of range (" +
                                          std::to_string(pts.size()) + " points)");
            face.push_back(idx);
        }
        flush();
        n_tris = out_tris->size() - before;
        return true;
    }

    bool load_mesh_url(const PNode& n, const Ctx& ctx, const Eigen::Matrix4f& M,
                       std::string& label, std::size_t& n_tris)
    {
        bool ok = true;
        const PValue* u = resolve(n.field("url"), ctx, n, ok); if (not ok) return false;
        if (not u or u->values.empty())
            return set_err(n.where(), "Mesh node without a `url`");

        const std::size_t before = out_tris->size();
        std::vector<std::string> names;
        for (const auto& t : u->values)
        {
            if (not t.is_str) return set_err(n.where(), "Mesh.url entry `" + t.text + "` is not a quoted path");
            fs::path p = t.text;
            if (p.is_relative())
                p = fs::weakly_canonical(fs::path(n.file).parent_path() / p);
            if (not fs::exists(p))
                return set_err(n.where(), "Mesh url `" + t.text + "` resolves to `" + p.string() +
                                          "` which does not exist");

            Assimp::Importer imp;
            const aiScene* sc = imp.ReadFile(p.string(), aiProcess_Triangulate | aiProcess_JoinIdenticalVertices |
                                                        aiProcess_SortByPType | aiProcess_PreTransformVertices);
            if (not sc or not sc->HasMeshes())
                return set_err(n.where(), "assimp cannot read mesh `" + p.string() + "`: " + imp.GetErrorString());

            for (unsigned m = 0; m < sc->mNumMeshes; ++m)
            {
                const aiMesh* me = sc->mMeshes[m];
                for (unsigned fi = 0; fi < me->mNumFaces; ++fi)
                {
                    const aiFace& f = me->mFaces[fi];
                    if (f.mNumIndices != 3) continue;            // aiProcess_Triangulate leaves only tris + degenerates
                    const auto V = [&](unsigned k) {
                        const aiVector3D& a = me->mVertices[f.mIndices[k]];
                        return Eigen::Vector3f(a.x, a.y, a.z);
                    };
                    emit(M, V(0), V(1), V(2));
                }
            }
            names.push_back(p.filename().string());
        }
        label = "Mesh(";
        for (std::size_t i = 0; i < names.size(); ++i) label += (i ? "," : "") + names[i];
        label += ")";
        n_tris = out_tris->size() - before;
        return true;
    }

    bool tess_geometry(const PNode& g, const Ctx& ctx, const Eigen::Matrix4f& M,
                       std::string& label, std::size_t& n_tris)
    {
        bool ok = true;
        std::vector<double> v;
        n_tris = 0;
        const std::size_t before = out_tris->size();

        if (g.type == "Box")
        {
            const PValue* s = resolve(g.field("size"), ctx, g, ok); if (not ok) return false;
            if (not nums(s, 3, v, g, "size")) return false;
            if (v.size() != 3) v = {2, 2, 2};                    // VRML Box default
            tess_box(M, (float)v[0], (float)v[1], (float)v[2]);
            label = "Box";
        }
        else if (g.type == "Cylinder")
        {
            const PValue* hv = resolve(g.field("height"), ctx, g, ok); if (not ok) return false;
            const PValue* rv = resolve(g.field("radius"), ctx, g, ok); if (not ok) return false;
            const PValue* sv = resolve(g.field("subdivision"), ctx, g, ok); if (not ok) return false;
            std::vector<double> a, b, c;
            if (not nums(hv, 1, a, g, "height") or not nums(rv, 1, b, g, "radius") or
                not nums(sv, 1, c, g, "subdivision")) return false;
            const auto flag = [&](const char* f) {
                const PValue* p = g.field(f);
                return not (p and p->values.size() == 1 and p->values[0].text == "FALSE");
            };
            tess_cylinder(M, a.empty() ? 2.f : (float)a[0], b.empty() ? 1.f : (float)b[0],
                          c.empty() ? 36 : (int)c[0], flag("top"), flag("bottom"), flag("side"));
            label = "Cylinder";
        }
        else if (g.type == "Sphere")
        {
            const PValue* rv = resolve(g.field("radius"), ctx, g, ok); if (not ok) return false;
            const PValue* sv = resolve(g.field("subdivision"), ctx, g, ok); if (not ok) return false;
            std::vector<double> a, b;
            if (not nums(rv, 1, a, g, "radius") or not nums(sv, 1, b, g, "subdivision")) return false;
            tess_sphere(M, a.empty() ? 1.f : (float)a[0], b.empty() ? 4 : (int)b[0]);
            label = "Sphere";
        }
        else if (g.type == "IndexedFaceSet")
        {
            if (not tess_ifs(g, ctx, M, n_tris)) return false;
            label = "IndexedFaceSet";
            return true;
        }
        else if (g.type == "Mesh")
        {
            return load_mesh_url(g, ctx, M, label, n_tris);
        }
        else
            // Cone / Capsule / Plane / ElevationGrid / PointSet / IndexedLineSet / Extrusion …
            // Silently skipping any of these would punch exactly the kind of hole in the scene that
            // this whole loader exists to close.
            return set_err(g.where(), "unsupported geometry node `" + g.type +
                                      "` (supported: Box, Cylinder, Sphere, IndexedFaceSet, Mesh)");

        n_tris = out_tris->size() - before;
        return true;
    }

    // --------------------------------------------------------------------------------------
    //  traversal
    // --------------------------------------------------------------------------------------

    // Node-valued fields that provably carry no renderable surface. Anything node-valued and NOT
    // listed here is an error rather than a skip, because an unrecognised node-valued field is the
    // one place a whole solid can vanish without a trace (Propeller.fastHelix, Track.device, …).
    static bool skippable_node_field(std::string_view f)
    {
        static const std::set<std::string, std::less<>> kSkip = {
            "appearance", "material", "texture", "textureTransform",
            "baseColorMap", "roughnessMap", "metalnessMap", "normalMap", "occlusionMap",
            "emissiveColorMap", "physics", "device", "device2", "device3",
            "jointParameters", "jointParameters2", "jointParameters3",
            "lens", "focus", "zoom", "recognitionColors", "damping", "immersionProperties",
            "muscles", "contactMaterial", "texCoord", "normal", "color", "coord",
            "lensFlare", "spotLight", "pointLight", "directionalLight", "geometry"};
        return kSkip.contains(f);
    }

    static bool is_geometry_type(std::string_view t)
    {
        return t == "Box" or t == "Cylinder" or t == "Sphere" or t == "IndexedFaceSet" or t == "Mesh";
    }

    // `bo` = we are inside a boundingObject subtree. It matters because a boundingObject holds BARE
    // geometry nodes (`Pose { children [ Cylinder {…} ] }`) with no enclosing Shape — treating those
    // as ordinary grouping nodes would drop them without a word, which is the exact failure this
    // loader exists to eliminate.
    bool walk(const PNodePtr& node, const Ctx& ctx_in, const Eigen::Matrix4f& parent, std::string path,
              bool bo = false, bool ignore_local = false)
    {
        const PNode* np  = node.get();
        const Ctx*   cp  = &ctx_in;

        // ---- USE: re-traverse the DEF'd node here, under this transform (Webots sharing semantics)
        if (np->type == "__USE__")
        {
            auto it = cp->defs->find(np->def);
            if (it == cp->defs->end())
                return set_err(np->where(), "USE `" + np->def + "` has no matching DEF in scope");
            return walk(it->second, *cp, parent, path + "/USE(" + np->def + ")", bo, ignore_local);
        }

        // ---- PROTO instantiation: bind interface defaults, override with this instance's fields
        std::string why;
        if (auto pd = find_proto(np->type, cp->proto, why); pd)
        {
            Ctx inner;
            inner.proto = pd.get();
            inner.defs  = std::make_shared<std::map<std::string, PNodePtr>>();
            for (const auto& [fname, fdef] : pd->interface) inner.binds[fname] = &fdef;
            for (const auto& [fname, fval] : np->fields)
            {
                if (not inner.binds.contains(fname))
                    return set_err(np->where(), "PROTO `" + pd->name + "` has no field `" + fname +
                                                "` (declared in " + pd->file + ")");
                bool ok = true;
                const PValue* concrete = resolve(&fval, *cp, *np, ok);   // may itself be `IS` in the outer proto
                if (not ok) return false;
                inner.binds[fname] = concrete;
            }
            if (not pd->body) return set_err(np->where(), "PROTO `" + pd->name + "` has an empty body");
            return walk(pd->body, inner, parent, path + "/" + pd->name, bo, ignore_local);
        }
        else if (not why.empty())
            return set_err(np->where(), "cannot instantiate `" + np->type + "`: " + why);

        // ---- built-in node: record its DEF, compose its transform, descend
        if (not np->def.empty()) (*cp->defs)[np->def] = node;

        Eigen::Matrix4f local;
        if (not local_xform(*np, *cp, local)) return false;
        Eigen::Matrix4f M = ignore_local ? parent : (parent * local);
        path += "/" + np->type + (np->def.empty() ? "" : "(" + np->def + ")");

        // ---- bare geometry inside a boundingObject (no Shape wrapper)
        if (bo and is_geometry_type(np->type))
        {
            std::string label; std::size_t nt = 0;
            if (not tess_geometry(*np, *cp, M, label, nt)) return false;
            self->m_shapes.push_back({path, label + " [boundingObject]", np->where(), nt});
            return true;
        }

        // HingeJoint & friends: the endPoint Solid carries its own placement (equal to the anchor at
        // position 0), so the static scene is exact at rest. A non-zero `position` is a real pose and
        // is applied about the anchor — refusing to model it would misplace a wheel by up to its radius.
        if (np->type.ends_with("Joint"))
        {
            bool ok = true;
            const PValue* jp = resolve(np->field("jointParameters"), *cp, *np, ok); if (not ok) return false;
            if (jp and not jp->nodes.empty())
            {
                const PNode& j = *jp->nodes.front();
                std::vector<double> ax, an, po;
                const PValue* axv = resolve(j.field("axis"),     *cp, j, ok); if (not ok) return false;
                const PValue* anv = resolve(j.field("anchor"),   *cp, j, ok); if (not ok) return false;
                const PValue* pov = resolve(j.field("position"), *cp, j, ok); if (not ok) return false;
                if (not nums(axv, 3, ax, j, "axis") or not nums(anv, 3, an, j, "anchor") or
                    not nums(pov, 1, po, j, "position")) return false;
                if (not po.empty() and std::abs(po[0]) > 1e-9)
                {
                    if (ax.size() != 3) ax = {1, 0, 0};
                    if (an.size() != 3) an = {0, 0, 0};
                    const Eigen::Vector3f a((float)an[0], (float)an[1], (float)an[2]);
                    Eigen::Matrix4f R = rot_from_axis_angle(ax[0], ax[1], ax[2], po[0]);
                    Eigen::Matrix4f T = Eigen::Matrix4f::Identity(); T.block<3,1>(0,3) =  a;
                    Eigen::Matrix4f Ti= Eigen::Matrix4f::Identity(); Ti.block<3,1>(0,3) = -a;
                    M = M * T * R * Ti;
                }
            }
        }

        // ---- Shape: the only place geometry is emitted
        if (np->type == "Shape")
        {
            bool ok = true;
            const PValue* gv = resolve(np->field("geometry"), *cp, *np, ok); if (not ok) return false;
            if (not gv or gv->nodes.empty() or gv->is_null)
                return set_err(np->where(), "Shape without a `geometry` node");
            PNodePtr g = gv->nodes.front();
            if (g->type == "__USE__")
            {
                auto it = cp->defs->find(g->def);
                if (it == cp->defs->end())
                    return set_err(g->where(), "USE `" + g->def + "` has no matching DEF in scope");
                g = it->second;
            }
            std::string label; std::size_t nt = 0;
            if (not tess_geometry(*g, *cp, M, label, nt)) return false;
            self->m_shapes.push_back({path, label + (bo ? " [boundingObject]" : ""), g->where(), nt});
            return true;
        }

        // ---- descend
        for (const auto& [fname, fval_raw] : np->fields)
        {
            if (fname == "boundingObject" and not opt.include_bounding_objects) continue;
            bool ok = true;
            const PValue* fval = resolve(&fval_raw, *cp, *np, ok); if (not ok) return false;
            if (not fval or fval->nodes.empty()) continue;
            if (skippable_node_field(fname)) continue;

            const bool descend = (fname == "children" or fname == "endPoint" or fname == "boundingObject" or
                                  fname == "slot" or fname == "fastHelix" or fname == "slowHelix");
            if (not descend)
                return set_err(np->where(), "node-valued field `" + fname + "` on `" + np->type +
                                            "` is not understood; it may hide geometry, so this is fatal "
                                            "(add it to skippable_node_field() or to the descend list)");
            for (const auto& child : fval->nodes)
                if (not walk(child, *cp, M, path, bo or fname == "boundingObject")) return false;
        }
        return true;
    }
};

// ================================================================================================
//  XY hull (navigation-envelope consumer)
// ================================================================================================
namespace
{

using V2 = Eigen::Vector2f;

double cross2(const V2& o, const V2& a, const V2& b)
{
    return (double)(a.x() - o.x()) * (b.y() - o.y()) - (double)(a.y() - o.y()) * (b.x() - o.x());
}

std::vector<V2> convex_hull(std::vector<V2> p)          // monotone chain, CCW, no closing vertex
{
    std::ranges::sort(p, [](const V2& a, const V2& b) {
        return a.x() < b.x() or (a.x() == b.x() and a.y() < b.y()); });
    p.erase(std::ranges::unique(p, [](const V2& a, const V2& b) { return a.x() == b.x() and a.y() == b.y(); }).begin(),
            p.end());
    if (p.size() < 3) return p;

    std::vector<V2> h(2 * p.size());
    std::size_t k = 0;
    for (std::size_t i = 0; i < p.size(); ++i)
    {
        while (k >= 2 and cross2(h[k-2], h[k-1], p[i]) <= 0) --k;
        h[k++] = p[i];
    }
    for (std::size_t i = p.size() - 1, t = k + 1; i-- > 0;)
    {
        while (k >= t and cross2(h[k-2], h[k-1], p[i]) <= 0) --k;
        h[k++] = p[i];
    }
    h.resize(k > 0 ? k - 1 : 0);
    return h;
}

double poly_area(const std::vector<V2>& h)
{
    double a = 0;
    for (std::size_t i = 0, n = h.size(); i < n; ++i)
    {
        const V2& p = h[i];
        const V2& q = h[(i + 1) % n];
        a += (double)p.x() * q.y() - (double)q.x() * p.y();
    }
    return 0.5 * a;
}

// Cost of deleting vertex i: the area of the triangle added when its two neighbouring edges are
// extended to meet. Infinite when they are (near-)parallel — such a vertex cannot be removed
// outward at finite cost, and forcing it would blow the envelope up.
double removal_cost(const std::vector<V2>& h, std::size_t i, V2& meet)
{
    const std::size_t n = h.size();
    if (n < 5) return std::numeric_limits<double>::infinity();
    const V2& a = h[(i + n - 2) % n];
    const V2& b = h[(i + n - 1) % n];
    const V2& c = h[(i + 1) % n];
    const V2& d = h[(i + 2) % n];
    const V2 u = b - a, v = d - c;
    const double den = (double)u.x() * v.y() - (double)u.y() * v.x();
    if (std::abs(den) < 1e-12) return std::numeric_limits<double>::infinity();
    const double t = ((double)(c.x() - b.x()) * v.y() - (double)(c.y() - b.y()) * v.x()) / den;
    if (t < 0) return std::numeric_limits<double>::infinity();      // intersection behind: not outward
    meet = V2(b.x() + (float)(t * u.x()), b.y() + (float)(t * u.y()));
    return std::abs(0.5 * cross2(b, meet, c));                      // triangle b-meet-c
}

std::vector<V2> simplify_outward(std::vector<V2> h, double budget_area)
{
    double spent = 0;
    while (h.size() > 4)
    {
        std::size_t best = 0;
        double       bc  = std::numeric_limits<double>::infinity();
        V2           bm{};
        for (std::size_t i = 0; i < h.size(); ++i)
        {
            V2 m{};
            const double c = removal_cost(h, i, m);
            if (c < bc) { bc = c; best = i; bm = m; }
        }
        if (not std::isfinite(bc) or spent + bc > budget_area) break;
        spent += bc;
        const std::size_t n = h.size();
        h[(best + n - 1) % n] = bm;                                 // b and c collapse onto the meet
        h.erase(h.begin() + static_cast<long>((best + 1) % n == 0 ? 0 : (best + 1) % n));
        // erase() above removed c; `best` itself is now redundant (it lies inside b-meet-c)
        const std::size_t n2 = h.size();
        for (std::size_t i = 0; i < n2; ++i)
            if (h[i].x() == 0 and false) {}                          // no-op guard, hull re-cleaned below
        h = convex_hull(h);
    }
    return h;
}

}  // namespace

std::vector<Eigen::Vector2f> WebotsProtoLoader::xy_hull(float simplify_area_frac) const
{
    return xy_hull_band(-1e9f, 1e9f, simplify_area_frac);
}

std::vector<Eigen::Vector2f> WebotsProtoLoader::xy_hull_band(float z0, float z1, float f) const
{
    // Clip each triangle to the z-slab EXACTLY (Sutherland-Hodgman against z>=z0 and z<z1) rather
    // than keeping only vertices that fall inside it. The convex hull of the clipped polygons'
    // vertices is the exact hull of (geometry ∩ slab); a vertex-only filter would miss every band
    // crossed by a large triangle, which for this STL is the whole base — the same trap
    // robot_footprint.h records having fallen into.
    std::vector<V2> pts;
    pts.reserve(m_tris.size() * 3);
    std::vector<Eigen::Vector3f> poly, cut;
    for (const auto& t : m_tris)
    {
        const float lo = std::min({t.a.z(), t.b.z(), t.c.z()});
        const float hi = std::max({t.a.z(), t.b.z(), t.c.z()});
        if (hi < z0 or lo >= z1) continue;
        if (lo >= z0 and hi < z1)                       // wholly inside: no clipping needed
        {
            for (const Eigen::Vector3f* p : {&t.a, &t.b, &t.c}) pts.emplace_back(p->x(), p->y());
            continue;
        }
        poly = {t.a, t.b, t.c};
        for (int side = 0; side < 2; ++side)
        {
            const float  plane = side == 0 ? z0 : z1;
            const auto   inside = [&](const Eigen::Vector3f& p) { return side == 0 ? p.z() >= plane : p.z() <= plane; };
            cut.clear();
            for (std::size_t i = 0; i < poly.size(); ++i)
            {
                const Eigen::Vector3f& a = poly[i];
                const Eigen::Vector3f& b = poly[(i + 1) % poly.size()];
                const bool ia = inside(a), ib = inside(b);
                if (ia) cut.push_back(a);
                if (ia != ib and std::abs(b.z() - a.z()) > 1e-12f)
                    cut.push_back(a + (b - a) * ((plane - a.z()) / (b.z() - a.z())));
            }
            poly.swap(cut);
            if (poly.empty()) break;
        }
        for (const auto& p : poly) pts.emplace_back(p.x(), p.y());
    }
    if (pts.size() < 3) return {};
    auto h = convex_hull(std::move(pts));
    if (h.size() < 5 or f <= 0.f) return h;
    return simplify_outward(std::move(h), std::abs(poly_area(h)) * static_cast<double>(f));
}

// ================================================================================================
//  public API
// ================================================================================================

WebotsProtoLoader::WebotsProtoLoader(RTCDevice device, RTCScene scene, Options opt)
    : m_device(device), m_scene(scene), m_opt(opt) {}

WebotsProtoLoader::~WebotsProtoLoader() = default;

bool WebotsProtoLoader::load(const std::string& proto_path)
{
    m_tris.clear();
    m_shapes.clear();
    m_error.clear();

    Impl impl;
    impl.self     = this;
    impl.opt      = m_opt;
    impl.out_tris = &m_tris;

    if (not fs::exists(proto_path))
    {
        m_error = "proto file `" + proto_path + "` does not exist";
        std::fprintf(stderr, "[WebotsProtoLoader] REJECTED  %s\n", m_error.c_str());
        return false;
    }
    impl.search_dirs.push_back(fs::path(proto_path).parent_path());

    std::string why;
    auto root = impl.load_proto_file(proto_path, why);
    if (not root) { m_error = why; std::fprintf(stderr, "[WebotsProtoLoader] REJECTED  %s\n", why.c_str()); return false; }

    // Root PROTO instance: every interface field takes its declared default. The root's own
    // `translation`/`rotation` therefore evaluate to the DEFAULT world placement — and are then
    // discarded below, because the self-filter scene must be in the robot's own frame.
    Impl::Ctx ctx;
    ctx.proto = root.get();
    ctx.defs  = std::make_shared<std::map<std::string, PNodePtr>>();
    for (const auto& [fname, fdef] : root->interface) ctx.binds[fname] = &fdef;

    if (not root->body) { m_error = "PROTO `" + root->name + "` has an empty body"; return false; }

    // `ignore_local = true` on the root: the root body's own translation/rotation are the robot's
    // pose in the WORLD (`0 0 0.033` by default, overridden per-world), not part of its shape.
    // Applying them would shift the entire scene 33 mm in z against a 50 mm Dilate.
    if (not impl.walk(root->body, ctx, Eigen::Matrix4f::Identity(), "", false, true))
    { m_error = impl.err; return false; }

    if (m_tris.empty())
    {
        m_error = "proto `" + proto_path + "` produced ZERO triangles";
        std::fprintf(stderr, "[WebotsProtoLoader] REJECTED  %s\n", m_error.c_str());
        return false;
    }

    // One RTCGeometry per Shape keeps provenance (which solid a hit came from) and matches the
    // per-link geometry layout URDFMeshLoader uses. Vertices are already in the robot frame, so
    // there is no instance transform to get wrong.
    std::size_t first = 0;
    for (const auto& s : m_shapes)
    {
        if (s.tris == 0) { continue; }
        RTCGeometry geom = rtcNewGeometry(m_device, RTC_GEOMETRY_TYPE_TRIANGLE);
        auto* vtx = static_cast<float*>(rtcSetNewGeometryBuffer(
            geom, RTC_BUFFER_TYPE_VERTEX, 0, RTC_FORMAT_FLOAT3, 3 * sizeof(float), 3 * s.tris));
        auto* idx = static_cast<unsigned*>(rtcSetNewGeometryBuffer(
            geom, RTC_BUFFER_TYPE_INDEX, 0, RTC_FORMAT_UINT3, 3 * sizeof(unsigned), s.tris));
        for (std::size_t i = 0; i < s.tris; ++i)
        {
            const Tri& t = m_tris[first + i];
            const Eigen::Vector3f* p[3] = {&t.a, &t.b, &t.c};
            for (int k = 0; k < 3; ++k)
            {
                vtx[(3*i + k)*3 + 0] = p[k]->x();
                vtx[(3*i + k)*3 + 1] = p[k]->y();
                vtx[(3*i + k)*3 + 2] = p[k]->z();
            }
            idx[3*i + 0] = static_cast<unsigned>(3*i + 0);
            idx[3*i + 1] = static_cast<unsigned>(3*i + 1);
            idx[3*i + 2] = static_cast<unsigned>(3*i + 2);
        }
        rtcCommitGeometry(geom);
        rtcAttachGeometry(m_scene, geom);
        rtcReleaseGeometry(geom);
        first += s.tris;
    }

    if (m_opt.verbose)
    {
        std::printf("[WebotsProtoLoader] %s: %zu shapes, %zu triangles (boundingObject=%s)\n",
                    proto_path.c_str(), m_shapes.size(), m_tris.size(),
                    m_opt.include_bounding_objects ? "INCLUDED" : "excluded");
        for (const auto& s : m_shapes)
            std::printf("[WebotsProtoLoader]   %-7zu tris  %-24s %s\n", s.tris, s.geom_type.c_str(),
                        s.node_path.c_str());
    }
    return true;
}
