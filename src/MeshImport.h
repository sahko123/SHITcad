#pragma once
#include "Scene3D.h"
#include <string>
#include <vector>

namespace shitcad {

// Not including UnitUtils.h here: this header reaches every App*.cpp through
// App.h, and some of those define file-local helpers UnitUtils.h also declares.
struct UnitInfo;

// Length units offered for a mesh file. STL carries no units, so the user has to
// say what the numbers mean; everything inside SHITcad is mm.
const UnitInfo* findLengthUnit(const std::string& name);

struct MeshFileInfo {
    size_t triangleCount = 0;
    // Every edge shared by exactly two triangles. Only a closed surface can be
    // section-capped; capping an open one paints a slab over the model.
    bool closed = false;
    float rawMin[3] = {0, 0, 0}; // in the file's own (unknown) units
    float rawMax[3] = {0, 0, 0};
};

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

// Size and triangle count without scaling - for showing the user what a unit
// choice would mean before committing to it.
bool probeMeshFile(const std::string& path, MeshFileInfo& info, std::string& error);

} // namespace shitcad
