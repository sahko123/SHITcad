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
