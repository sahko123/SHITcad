#include "MeshImport.h"
#include "Constants.h"
#include "UnitUtils.h"
#include "Utf8Path.h"

#include <RWStl.hxx>
#include <Poly_Triangulation.hxx>
#include <gp_Pnt.hxx>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <array>
#include <map>
#include <fstream>
#include <memory>

namespace shitcad {

// ---- placement ----------------------------------------------------------

bool MeshTransform::isIdentity() const {
    static const double id[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
    for (int i = 0; i < 9; i++) if (r[i] != id[i]) return false;
    return t[0] == 0.0 && t[1] == 0.0 && t[2] == 0.0;
}

void MeshTransform::apply(const double p[3], double out[3]) const {
    for (int i = 0; i < 3; i++)
        out[i] = r[i * 3] * p[0] + r[i * 3 + 1] * p[1] + r[i * 3 + 2] * p[2] + t[i];
}

void axisRotation(int axis, double degrees, double out[9]) {
    double c, s;
    double quarters = degrees / 90.0;
    double rounded = std::round(quarters);
    if (std::fabs(quarters - rounded) < 1e-9) {
        static const double cs[4][2] = {{1, 0}, {0, 1}, {-1, 0}, {0, -1}};
        int k = (int)(((long long)rounded % 4 + 4) % 4);
        c = cs[k][0];
        s = cs[k][1];
    } else {
        const double rad = degrees * kDegToRadD;
        c = std::cos(rad);
        s = std::sin(rad);
    }
    const int a = axis, b = (axis + 1) % 3, d = (axis + 2) % 3;
    for (int i = 0; i < 9; i++) out[i] = 0.0;
    out[a * 3 + a] = 1.0;
    out[b * 3 + b] = c;  out[b * 3 + d] = -s;
    out[d * 3 + b] = s;  out[d * 3 + d] = c;
}

void rotateAbout(MeshTransform& xf, const double q[9], const double pivot[3]) {
    // p'' = Q (R p + t - c) + c  =>  R' = Q R,  t' = Q (t - c) + c
    double r[9];
    for (int i = 0; i < 3; i++)
        for (int j = 0; j < 3; j++)
            r[i * 3 + j] = q[i * 3] * xf.r[j] + q[i * 3 + 1] * xf.r[3 + j] + q[i * 3 + 2] * xf.r[6 + j];
    const double tc[3] = {xf.t[0] - pivot[0], xf.t[1] - pivot[1], xf.t[2] - pivot[2]};
    for (int i = 0; i < 3; i++)
        xf.t[i] = q[i * 3] * tc[0] + q[i * 3 + 1] * tc[1] + q[i * 3 + 2] * tc[2] + pivot[i];
    for (int i = 0; i < 9; i++) xf.r[i] = r[i];
}

void placedBounds(const MeshFileInfo& info, float unitToMm, const MeshTransform& xf,
                  double outMin[3], double outMax[3]) {
    // Transformed corners of the raw box. Exact for any rotation by multiples
    // of 90 degrees; a slight over-estimate for other angles, which is fine
    // for sizing and "drop to ground" readouts.
    for (int k = 0; k < 3; k++) { outMin[k] = 1e300; outMax[k] = -1e300; }
    for (int corner = 0; corner < 8; corner++) {
        const double p[3] = {
            ((corner & 1) ? info.rawMax[0] : info.rawMin[0]) * (double)unitToMm,
            ((corner & 2) ? info.rawMax[1] : info.rawMin[1]) * (double)unitToMm,
            ((corner & 4) ? info.rawMax[2] : info.rawMin[2]) * (double)unitToMm,
        };
        double q[3];
        xf.apply(p, q);
        for (int k = 0; k < 3; k++) {
            outMin[k] = std::min(outMin[k], q[k]);
            outMax[k] = std::max(outMax[k], q[k]);
        }
    }
}

void applyMeshTransform(std::vector<MeshVertex>& verts, const MeshTransform& xf) {
    if (xf.isIdentity()) return;
    const double* r = xf.r;
    for (auto& v : verts) {
        const double p[3] = {v.px, v.py, v.pz};
        const double n[3] = {v.nx, v.ny, v.nz};
        v.px = (float)(r[0] * p[0] + r[1] * p[1] + r[2] * p[2] + xf.t[0]);
        v.py = (float)(r[3] * p[0] + r[4] * p[1] + r[5] * p[2] + xf.t[1]);
        v.pz = (float)(r[6] * p[0] + r[7] * p[1] + r[8] * p[2] + xf.t[2]);
        v.nx = (float)(r[0] * n[0] + r[1] * n[1] + r[2] * n[2]);
        v.ny = (float)(r[3] * n[0] + r[4] * n[1] + r[5] * n[2]);
        v.nz = (float)(r[6] * n[0] + r[7] * n[1] + r[8] * n[2]);
    }
}

bool trianglesAreClosed(const float* xyz, size_t triangles) {
    if (triangles == 0) return false;
    // Weld by rounded position: an STL stores each vertex once per triangle, so
    // shared edges only match after welding. 1e-4 mm is far below any real
    // feature and far above float32 noise at vessel scale.
    std::map<std::array<long long, 3>, int> ids;
    auto vertexId = [&](const float* p) {
        std::array<long long, 3> key{(long long)std::llround(p[0] * 10000.0),
                                     (long long)std::llround(p[1] * 10000.0),
                                     (long long)std::llround(p[2] * 10000.0)};
        auto it = ids.find(key);
        if (it != ids.end()) return it->second;
        const int id = (int)ids.size();
        ids.emplace(key, id);
        return id;
    };
    std::map<std::pair<int, int>, int> edges;
    for (size_t t = 0; t < triangles; t++) {
        const float* p = xyz + t * 9;
        const int v[3] = {vertexId(p), vertexId(p + 3), vertexId(p + 6)};
        for (int e = 0; e < 3; e++) {
            int a = v[e], b = v[(e + 1) % 3];
            if (a == b) return false;                     // degenerate
            edges[{std::min(a, b), std::max(a, b)}]++;
        }
    }
    for (const auto& kv : edges)
        if (kv.second != 2) return false;
    return true;
}

// ---- file loading ---------------------------------------------------------

const UnitInfo* findLengthUnit(const std::string& name) {
    for (int i = 0; i < kUnitCount; i++) {
        if (name == kUnits[i].name) return &kUnits[i];
    }
    return nullptr;
}

namespace {

struct CachedMesh {
    FileStamp stamp;
    std::string error;           // the read failed; cached so it is not retried until the file changes
    std::vector<MeshVertex> raw; // unscaled, with per-face normals
    MeshFileInfo info;
    std::vector<uint32_t> skinOf; // skin index per triangle
    // resolveSkins results by point list. The spec is rebuilt every frame (the
    // stale-results check), and each resolve is a pass over every triangle.
    mutable std::map<std::vector<std::array<double, 3>>, std::pair<std::vector<int>, std::string>> resolved;
};

// Split into skins: triangles that share a vertex are one skin. Welds by exact
// coordinates, as cip-sim does, so both sides see the same skins.
void findSkins(CachedMesh& m) {
    const size_t nt = m.info.triangleCount;
    std::map<std::array<float, 3>, uint32_t> ids;
    std::vector<uint32_t> vert(nt * 3);
    for (size_t i = 0; i < nt * 3; i++) {
        const MeshVertex& v = m.raw[i];
        // + 0.0f folds -0 into +0: equal coordinates, different bits.
        const std::array<float, 3> key{v.px + 0.0f, v.py + 0.0f, v.pz + 0.0f};
        vert[i] = ids.emplace(key, (uint32_t)ids.size()).first->second;
    }

    std::vector<uint32_t> parent(ids.size());
    for (uint32_t i = 0; i < parent.size(); i++) parent[i] = i;
    auto find = [&](uint32_t a) {
        while (parent[a] != a) a = parent[a] = parent[parent[a]];
        return a;
    };
    for (size_t t = 0; t < nt; t++)
        for (int k = 1; k < 3; k++) {
            const uint32_t a = find(vert[t * 3]), b = find(vert[t * 3 + k]);
            if (a != b) parent[std::max(a, b)] = std::min(a, b);
        }

    // Number skins in order of their first triangle.
    std::map<uint32_t, uint32_t> skinOfRoot;
    m.skinOf.resize(nt);
    for (size_t t = 0; t < nt; t++) {
        auto it = skinOfRoot.emplace(find(vert[t * 3]), (uint32_t)skinOfRoot.size()).first;
        m.skinOf[t] = it->second;
        if (it->second == m.info.skins.size()) {
            MeshSkinInfo s;
            s.firstTriangle = t;
            for (int k = 0; k < 3; k++) {
                s.rawMin[k] = 1e30f;
                s.rawMax[k] = -1e30f;
            }
            const MeshVertex* p = &m.raw[t * 3];
            s.point[0] = ((double)p[0].px + p[1].px + p[2].px) / 3.0;
            s.point[1] = ((double)p[0].py + p[1].py + p[2].py) / 3.0;
            s.point[2] = ((double)p[0].pz + p[1].pz + p[2].pz) / 3.0;
            s.closed = true;
            m.info.skins.push_back(s);
        }
    }

    std::map<std::pair<uint32_t, uint32_t>, int> edges;
    for (size_t t = 0; t < nt; t++) {
        MeshSkinInfo& s = m.info.skins[m.skinOf[t]];
        s.triangleCount++;
        const MeshVertex* p = &m.raw[t * 3];
        for (int c = 0; c < 3; c++) {
            const float q[3] = {p[c].px, p[c].py, p[c].pz};
            for (int k = 0; k < 3; k++) {
                s.rawMin[k] = std::min(s.rawMin[k], q[k]);
                s.rawMax[k] = std::max(s.rawMax[k], q[k]);
            }
        }
        // Divergence theorem: the sum of signed tetrahedra to the origin.
        const double a[3] = {p[0].px, p[0].py, p[0].pz}, b[3] = {p[1].px, p[1].py, p[1].pz},
                     c[3] = {p[2].px, p[2].py, p[2].pz};
        s.signedVolume += (a[0] * (b[1] * c[2] - b[2] * c[1]) - a[1] * (b[0] * c[2] - b[2] * c[0]) +
                           a[2] * (b[0] * c[1] - b[1] * c[0])) / 6.0;
        for (int e = 0; e < 3; e++) {
            const uint32_t u = vert[t * 3 + e], v = vert[t * 3 + (e + 1) % 3];
            if (u == v) s.closed = false;   // degenerate
            else edges[{std::min(u, v), std::max(u, v)}]++;
        }
    }
    // An edge's skin is its vertex's skin, which the root map gives directly.
    for (const auto& [e, count] : edges)
        if (count != 2) m.info.skins[skinOfRoot[find(e.first)]].closed = false;
}

// Closest point on triangle abc to p (Ericson, Real-Time Collision Detection 5.1.5).
double distanceToTriangle(const double p[3], const double a[3], const double b[3], const double c[3]) {
    auto sub = [](const double* x, const double* y, double* o) { for (int i = 0; i < 3; i++) o[i] = x[i] - y[i]; };
    auto dot = [](const double* x, const double* y) { return x[0] * y[0] + x[1] * y[1] + x[2] * y[2]; };
    double ab[3], ac[3], ap[3], q[3];
    sub(b, a, ab);
    sub(c, a, ac);
    sub(p, a, ap);
    auto at = [&](double v, double w) { for (int i = 0; i < 3; i++) q[i] = a[i] + ab[i] * v + ac[i] * w; };
    const double d1 = dot(ab, ap), d2 = dot(ac, ap);
    if (d1 <= 0 && d2 <= 0) at(0, 0);
    else {
        double bp[3];
        sub(p, b, bp);
        const double d3 = dot(ab, bp), d4 = dot(ac, bp);
        double cp[3];
        sub(p, c, cp);
        const double d5 = dot(ab, cp), d6 = dot(ac, cp);
        const double vc = d1 * d4 - d3 * d2, vb = d5 * d2 - d1 * d6, va = d3 * d6 - d5 * d4;
        if (d3 >= 0 && d4 <= d3) at(1, 0);
        else if (vc <= 0 && d1 >= 0 && d3 <= 0) at(d1 / (d1 - d3), 0);
        else if (d6 >= 0 && d5 <= d6) at(0, 1);
        else if (vb <= 0 && d2 >= 0 && d6 <= 0) at(0, d2 / (d2 - d6));
        else if (va <= 0 && (d4 - d3) >= 0 && (d5 - d6) >= 0) {
            const double w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
            at(1 - w, w);
        } else {
            const double denom = 1.0 / (va + vb + vc);
            at(vb * denom, vc * denom);
        }
    }
    double d[3];
    sub(p, q, d);
    return std::sqrt(dot(d, d));
}

// One entry per path. Bounded by the number of distinct files a project
// references, which is small, so no eviction.
std::map<std::string, std::shared_ptr<const CachedMesh>>& cache() {
    static std::map<std::string, std::shared_ptr<const CachedMesh>> c;
    return c;
}

// FNV-1a over the head, middle and tail of the file. Cheap on a 50 MB export,
// and re-exported geometry differs somewhere in those windows in practice.
uint64_t sampleHash(const std::filesystem::path& path, uintmax_t size) {
    std::ifstream f(path, std::ios::binary);
    if (!f) return 0;
    uint64_t h = 1469598103934665603ull;
    char buf[4096];
    const uintmax_t spots[3] = {0, size > 4096 ? size / 2 : 0, size > 4096 ? size - 4096 : 0};
    for (uintmax_t at : spots) {
        f.clear();
        f.seekg((std::streamoff)at, std::ios::beg);
        f.read(buf, sizeof(buf));
        const std::streamsize got = f.gcount();
        for (std::streamsize i = 0; i < got; i++) {
            h ^= (unsigned char)buf[i];
            h *= 1099511628211ull;
        }
    }
    h ^= (uint64_t)size;
    return h * 1099511628211ull;
}

std::shared_ptr<const CachedMesh> readCached(const std::string& path, std::string& error) {
    FileStamp stamp;
    if (!stampFile(path, stamp, error)) return nullptr;
    auto it = cache().find(path);
    if (it != cache().end() && it->second->stamp == stamp) {
        if (!it->second->error.empty()) {
            error = it->second->error;
            return nullptr;
        }
        return it->second;
    }

    Handle(Poly_Triangulation) tri = RWStl::ReadFile(path.c_str()); // OCCT takes UTF-8; path::string() would be ANSI
    if (tri.IsNull() || tri->NbTriangles() == 0) {
        // Remembered until the file changes: the Place panel asks every frame.
        auto failed = std::make_shared<CachedMesh>();
        failed->stamp = stamp;
        failed->error = "Not a readable STL, or it contains no triangles: " + path;
        cache()[path] = failed;
        error = failed->error;
        return nullptr;
    }

    auto mesh = std::make_shared<CachedMesh>();
    mesh->stamp = stamp;
    mesh->raw.reserve((size_t)tri->NbTriangles() * 3);
    mesh->info.triangleCount = (size_t)tri->NbTriangles();

    float lo[3] = {1e30f, 1e30f, 1e30f};
    float hi[3] = {-1e30f, -1e30f, -1e30f};

    for (int i = 1; i <= tri->NbTriangles(); i++) {
        int n1, n2, n3;
        tri->Triangle(i).Get(n1, n2, n3);
        const gp_Pnt p[3] = {tri->Node(n1), tri->Node(n2), tri->Node(n3)};

        // Face normal from winding. Computed in raw units: a uniform scale does
        // not change its direction, so it stays valid after scaling to mm.
        double ax = p[1].X() - p[0].X(), ay = p[1].Y() - p[0].Y(), az = p[1].Z() - p[0].Z();
        double bx = p[2].X() - p[0].X(), by = p[2].Y() - p[0].Y(), bz = p[2].Z() - p[0].Z();
        double nx = ay * bz - az * by, ny = az * bx - ax * bz, nz = ax * by - ay * bx;
        double len = std::sqrt(nx * nx + ny * ny + nz * nz);
        if (len > 0.0) { nx /= len; ny /= len; nz /= len; }

        for (const gp_Pnt& q : p) {
            const float v[3] = {(float)q.X(), (float)q.Y(), (float)q.Z()};
            for (int k = 0; k < 3; k++) {
                lo[k] = std::min(lo[k], v[k]);
                hi[k] = std::max(hi[k], v[k]);
            }
            mesh->raw.push_back({v[0], v[1], v[2], (float)nx, (float)ny, (float)nz});
        }
    }
    for (int k = 0; k < 3; k++) {
        mesh->info.rawMin[k] = lo[k];
        mesh->info.rawMax[k] = hi[k];
    }
    // trianglesAreClosed takes packed positions. `raw` interleaves normals, and
    // passing it directly (as this once did) read normals as corners, so an
    // imported STL's watertightness - and with it section capping - was noise.
    std::vector<float> xyz;
    xyz.reserve(mesh->raw.size() * 3);
    for (const MeshVertex& v : mesh->raw) xyz.insert(xyz.end(), {v.px, v.py, v.pz});
    mesh->info.closed = trianglesAreClosed(xyz.data(), mesh->info.triangleCount);
    findSkins(*mesh);

    cache()[path] = mesh;
    return mesh;
}

} // namespace

bool stampFile(const std::string& path, FileStamp& out, std::string& error) {
    std::error_code ec;
    // Paths are stored as UTF-8 (see Utf8Path.h), so they have to be
    // converted rather than handed to the narrow constructor.
    const std::filesystem::path file = fsPath(path);
    out.size = std::filesystem::file_size(file, ec);
    if (ec) {
        error = "File not found: " + path;
        return false;
    }
    out.mtime = std::filesystem::last_write_time(file, ec);
    if (ec) {
        error = "Cannot read file time: " + path;
        return false;
    }
    // Size + mtime alone is not enough: a re-exported STL with the same
    // triangle count has an identical size (84 + 50N bytes), Windows file times
    // are ~4 ms granular, and timestamp-preserving copies (unzip, sync restore)
    // collide exactly. Hash a sample of the bytes as well - the whole file would
    // cost too much on every replay.
    out.hash = sampleHash(file, out.size);
    return true;
}

bool probeMeshFile(const std::string& path, MeshFileInfo& info, std::string& error) {
    auto mesh = readCached(path, error);
    if (!mesh) return false;
    info = mesh->info;
    return true;
}

bool loadMeshFile(const std::string& path, const std::string& unit,
                  std::vector<MeshVertex>& outMm, MeshFileInfo& info, std::string& error) {
    const UnitInfo* u = findLengthUnit(unit);
    if (!u) {
        error = "Unknown unit '" + unit + "'";
        return false;
    }
    auto mesh = readCached(path, error);
    if (!mesh) return false;

    info = mesh->info;
    const float s = u->toMm;
    outMm.resize(mesh->raw.size());
    for (size_t i = 0; i < mesh->raw.size(); i++) {
        const MeshVertex& r = mesh->raw[i];
        outMm[i] = {r.px * s, r.py * s, r.pz * s, r.nx, r.ny, r.nz};
    }
    return true;
}

namespace {

bool resolveIn(const CachedMesh& mesh, const std::string& path,
               const std::vector<std::array<double, 3>>& points, std::vector<int>& out, std::string& error) {
    auto hit = mesh.resolved.find(points);
    if (hit == mesh.resolved.end()) {
        std::vector<int> found;
        std::string err;
        double diag = 0;
        for (int k = 0; k < 3; k++) {
            const double d = (double)mesh.info.rawMax[k] - mesh.info.rawMin[k];
            diag += d * d;
        }
        const double tol = kSkinPointTolerance * std::sqrt(diag);
        for (const auto& p : points) {
            double best = 1e300;
            size_t bestTri = 0;
            for (size_t t = 0; t < mesh.info.triangleCount; t++) {
                const MeshVertex* v = &mesh.raw[t * 3];
                const double a[3] = {v[0].px, v[0].py, v[0].pz}, b[3] = {v[1].px, v[1].py, v[1].pz},
                             c[3] = {v[2].px, v[2].py, v[2].pz};
                const double d = distanceToTriangle(p.data(), a, b, c);
                if (d < best) { best = d; bestTri = t; }
            }
            if (best > tol) {
                char buf[160];
                snprintf(buf, sizeof(buf), "a chosen skin is %.4g (file units) off the geometry", best);
                err = std::string(buf) + " - " + path + " has changed since its skins were picked. Pick them again.";
                found.clear();
                break;
            }
            found.push_back((int)mesh.skinOf[bestTri]);
        }
        hit = mesh.resolved.emplace(points, std::make_pair(std::move(found), std::move(err))).first;
    }
    if (!hit->second.second.empty()) {
        error = hit->second.second;
        return false;
    }
    out = hit->second.first;
    return true;
}

bool statesIn(const CachedMesh& mesh, const std::string& path, const MeshSkinChoice& choice,
              std::vector<MeshSkinState>& out, std::string& error) {
    const auto& skins = mesh.info.skins;
    out.assign(skins.size(), MeshSkinState{});
    if (!choice.keep.empty()) {
        std::vector<int> keep;
        if (!resolveIn(mesh, path, choice.keep, keep, error)) return false;
        for (auto& st : out) st.kept = false;
        for (int k : keep) out[k].kept = true;
    }
    if (!choice.flip.empty()) {
        std::vector<int> flip;
        if (!resolveIn(mesh, path, choice.flip, flip, error)) return false;
        for (int k : flip) out[k].userFlipped = !out[k].userFlipped; // two points on one skin cancel
    }
    for (size_t k = 0; k < skins.size(); k++) out[k].flipped = skinAutoFlipped(skins[k]) != out[k].userFlipped;
    return true;
}

} // namespace

bool resolveSkins(const std::string& path, const std::vector<std::array<double, 3>>& points,
                  std::vector<int>& out, std::string& error) {
    auto mesh = readCached(path, error);
    return mesh && resolveIn(*mesh, path, points, out, error);
}

bool skinStates(const std::string& path, const MeshSkinChoice& choice,
                std::vector<MeshSkinState>& out, std::string& error) {
    auto mesh = readCached(path, error);
    return mesh && statesIn(*mesh, path, choice, out, error);
}

bool loadMeshFile(const std::string& path, const std::string& unit, const MeshSkinChoice& skins,
                  std::vector<MeshVertex>& outMm, MeshFileInfo& info, std::string& error) {
    const UnitInfo* u = findLengthUnit(unit);
    if (!u) {
        error = "Unknown unit '" + unit + "'";
        return false;
    }
    auto mesh = readCached(path, error);
    if (!mesh) return false;
    std::vector<MeshSkinState> st;
    if (!statesIn(*mesh, path, skins, st, error)) return false;

    info = mesh->info;
    const float s = u->toMm;
    outMm.clear();
    outMm.reserve(mesh->raw.size());
    bool allClosed = true, any = false;
    for (size_t t = 0; t < info.triangleCount; t++) {
        const MeshSkinState& k = st[mesh->skinOf[t]];
        if (!k.kept) continue;
        any = true;
        allClosed = allClosed && info.skins[mesh->skinOf[t]].closed;
        const MeshVertex* r = &mesh->raw[t * 3];
        const float sign = k.flipped ? -1.0f : 1.0f;
        // Rewound, not only renormalised: the picker and the section cap work
        // the facing out from the winding.
        const int order[3] = {0, k.flipped ? 2 : 1, k.flipped ? 1 : 2};
        for (int c : order)
            outMm.push_back({r[c].px * s, r[c].py * s, r[c].pz * s, r[c].nx * sign, r[c].ny * sign, r[c].nz * sign});
    }
    // Every skin kept: keep the whole-file verdict, which welds with a
    // tolerance and so forgives an ASCII export's rounding.
    bool all = true;
    for (const auto& k : st) all = all && k.kept;
    if (!all) info.closed = any && allClosed;
    return true;
}

} // namespace shitcad
