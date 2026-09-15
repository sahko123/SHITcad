#include "MeshImport.h"
#include "UnitUtils.h"

#include <RWStl.hxx>
#include <Poly_Triangulation.hxx>
#include <gp_Pnt.hxx>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <map>
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
        const double rad = degrees * 3.14159265358979323846 / 180.0;
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

// ---- file loading ---------------------------------------------------------

const UnitInfo* findLengthUnit(const std::string& name) {
    for (int i = 0; i < kUnitCount; i++) {
        if (name == kUnits[i].name) return &kUnits[i];
    }
    return nullptr;
}

namespace {

struct CachedMesh {
    uintmax_t fileSize = 0;
    std::filesystem::file_time_type mtime;
    std::vector<MeshVertex> raw; // unscaled, with per-face normals
    MeshFileInfo info;
};

// One entry per path. Bounded by the number of distinct files a project
// references, which is small, so no eviction.
std::map<std::string, std::shared_ptr<const CachedMesh>>& cache() {
    static std::map<std::string, std::shared_ptr<const CachedMesh>> c;
    return c;
}

std::shared_ptr<const CachedMesh> readCached(const std::string& path, std::string& error) {
    std::error_code ec;
    // Narrow (ANSI) path, matching what the native file dialogs return.
    const std::filesystem::path fsPath(path);
    uintmax_t size = std::filesystem::file_size(fsPath, ec);
    if (ec) {
        error = "File not found: " + path;
        return nullptr;
    }
    auto mtime = std::filesystem::last_write_time(fsPath, ec);
    if (ec) {
        error = "Cannot read file time: " + path;
        return nullptr;
    }

    auto it = cache().find(path);
    if (it != cache().end() && it->second->fileSize == size && it->second->mtime == mtime) {
        return it->second;
    }

    Handle(Poly_Triangulation) tri = RWStl::ReadFile(path.c_str());
    if (tri.IsNull() || tri->NbTriangles() == 0) {
        error = "Not a readable STL, or it contains no triangles: " + path;
        return nullptr;
    }

    auto mesh = std::make_shared<CachedMesh>();
    mesh->fileSize = size;
    mesh->mtime = mtime;
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

    cache()[path] = mesh;
    return mesh;
}

} // namespace

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

} // namespace shitcad
