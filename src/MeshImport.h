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
    float rawMin[3] = {0, 0, 0}; // in the file's own (unknown) units
    float rawMax[3] = {0, 0, 0};
};

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
