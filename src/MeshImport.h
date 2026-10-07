#pragma once
#include "Scene3D.h"
#include <array>
#include <cstdint>
#include <filesystem>
#include <string>
#include <vector>

namespace shitcad {

// Not including UnitUtils.h here: this header reaches every App*.cpp through
// App.h, and some of those define file-local helpers UnitUtils.h also declares.
struct UnitInfo;

// Length units offered for a mesh file. STL carries no units, so the user has to
// say what the numbers mean; everything inside SHITcad is mm.
const UnitInfo* findLengthUnit(const std::string& name);

// A skin: triangles joined through shared vertices (identical coordinates).
// An Onshape export of a vessel modelled as a solid is two skins in one file -
// the outside of the wall and the inside, which is the cavity - and only the
// inner one is wall that gets sprayed.
struct MeshSkinInfo {
    size_t triangleCount = 0;
    size_t firstTriangle = 0;     // skins are numbered by this, ascending
    bool closed = false;
    // File units cubed. > 0: the triangles wind outward, the usual export of
    // a solid's outside. < 0 on a closed skin: it faces inward - the inside
    // wall of a solid, i.e. a cavity.
    double signedVolume = 0.0;
    float rawMin[3] = {0, 0, 0};
    float rawMax[3] = {0, 0, 0};
    // On the skin (centroid of firstTriangle), file coordinates. What a spec
    // selects it by: cip-sim keeps the skin nearest each point it is given.
    double point[3] = {0, 0, 0};

    bool isCavity() const { return closed && signedVolume < 0.0; }
};

struct MeshFileInfo {
    size_t triangleCount = 0;
    // Every edge shared by exactly two triangles. Only a closed surface can be
    // section-capped; capping an open one paints a slab over the model.
    bool closed = false;
    float rawMin[3] = {0, 0, 0}; // in the file's own (unknown) units
    float rawMax[3] = {0, 0, 0};
    std::vector<MeshSkinInfo> skins;
};

// Which skins of a file to use and which to turn round, each named by a point
// in FILE coordinates (as a cip-sim spec does), not by number: numbering
// follows triangle order, which changes on every re-export, and file
// coordinates do not move when the import is re-placed or its unit changes.
//
// Orientation convention: normals point away from the fluid, so a nozzle
// placed on a surface sprays along -normal. A closed skin that winds inward
// (a cavity) is turned round automatically; `flip` reverses that choice.
struct MeshSkinChoice {
    std::vector<std::array<double, 3>> keep; // empty = keep every skin
    std::vector<std::array<double, 3>> flip; // skins oriented against the automatic choice
    bool empty() const { return keep.empty() && flip.empty(); }
    bool operator==(const MeshSkinChoice& o) const { return keep == o.keep && flip == o.flip; }
};

// The outside of a solid's wall: a closed, outward-wound skin whose box holds a
// cavity's. "Keep the inside only" drops exactly these. Internals exported as
// their own solids (a baffle, a dip tube) sit inside the cavity and enclose
// none, so they stay; dropping every non-cavity skin would lose them.
bool isWallOutside(const MeshFileInfo& info, size_t skin);

// Orientation a skin gets without a user flip: cavities are turned round.
inline bool skinAutoFlipped(const MeshSkinInfo& s) { return s.isCavity(); }

// A stored point lies on its skin, so it only moves when the file is
// re-exported - by about the faceting error of a curved wall. It is stale when
// further than this fraction of the file's diagonal from every triangle, or
// when its skin is not clearly the nearest (at most kSkinPointAmbiguity times
// the distance to the next). Both are needed: at 1% a point drifted 8 mm off
// the inside of a 12 mm wall resolved to the outside. Same rule as cip-sim.
inline constexpr double kSkinPointTolerance = 0.001;
inline constexpr double kSkinPointAmbiguity = 0.5;

// The skin nearest each point (indices into MeshFileInfo::skins). Fails, naming
// the file, if a point is stale by the rule above.
bool resolveSkins(const std::string& path, const std::vector<std::array<double, 3>>& points,
                  std::vector<int>& out, std::string& error);

// Per skin of a file under a choice: kept? and is its winding reversed from
// the file's? Empty choice: all kept, cavities reversed.
struct MeshSkinState {
    bool kept = true;
    bool flipped = false;     // final orientation is opposite to the file's winding
    bool userFlipped = false; // ... because of an entry in choice.flip
};
bool skinStates(const std::string& path, const MeshSkinChoice& choice,
                std::vector<MeshSkinState>& out, std::string& error);

// Placement of an imported mesh: p' = R * p + t, applied after the file's
// coordinates are scaled to mm. The file itself is never modified.
//
// Rotation is stored as a matrix, not Euler angles, so repeated quarter turns
// about different axes compose exactly and in the order they were applied.
struct MeshTransform {
    double r[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1}; // row-major
    double t[3] = {0, 0, 0};                   // mm

    bool isIdentity() const;
    void apply(const double p[3], double out[3]) const;
};

// Rotation about a world axis (0 = X, 1 = Y, 2 = Z), right-handed. Multiples
// of 90 degrees are exact (no 6e-17 residue) so "right way up" stays exact.
void axisRotation(int axis, double degrees, double out[9]);

// Rotate a placed mesh by Q about a world-space pivot (typically its centre),
// so it turns in place instead of swinging around the origin.
void rotateAbout(MeshTransform& xf, const double q[9], const double pivot[3]);

// Axis-aligned bounds in mm after unit scaling and placement.
void placedBounds(const MeshFileInfo& info, float unitToMm, const MeshTransform& xf,
                  double outMin[3], double outMax[3]);

// Apply placement to vertices already in mm. Normals are rotated, not translated.
void applyMeshTransform(std::vector<MeshVertex>& verts, const MeshTransform& xf);

// True when every edge is shared by exactly two triangles. `xyz` is 9 floats
// per triangle.
bool trianglesAreClosed(const float* xyz, size_t triangles);

// Read an STL and return its triangles scaled to mm.
//
// Replay calls this on every model edit, so files are cached by path, size and
// modification time: an unchanged file is read once, and a re-exported file is
// picked up automatically on the next replay.
bool loadMeshFile(const std::string& path, const std::string& unit,
                  std::vector<MeshVertex>& outMm, MeshFileInfo& info, std::string& error);

// The same, keeping only the chosen skins and orienting each by the convention
// above (flipped triangles are rewound, not just given negated normals, so
// anything that derives a normal from winding agrees). `info.closed` then
// describes the kept triangles; `info.skins` still lists every skin.
bool loadMeshFile(const std::string& path, const std::string& unit, const MeshSkinChoice& skins,
                  std::vector<MeshVertex>& outMm, MeshFileInfo& info, std::string& error);

// Identity of a file for the import caches: size, modification time and a
// hash of samples of its bytes. Size + mtime alone collide - see readCached.
struct FileStamp {
    uintmax_t size = 0;
    std::filesystem::file_time_type mtime{};
    uint64_t hash = 0;
    bool operator==(const FileStamp& o) const { return size == o.size && mtime == o.mtime && hash == o.hash; }
};
bool stampFile(const std::string& path, FileStamp& out, std::string& error);

// Size and triangle count without scaling - for showing the user what a unit
// choice would mean before committing to it.
bool probeMeshFile(const std::string& path, MeshFileInfo& info, std::string& error);

} // namespace shitcad
