#pragma once
#include "FeatureHistory.h"
#include "SketchPlane.h"
#include "Scene3D.h"
#include "Simulation.h"
#include <string>
#include <vector>

namespace shitcad {

// `simulation` is optional so callers that only care about geometry need not
// pass one. A file without a simulation block loads as an empty set-up.
bool saveProject(const std::string& filepath,
                 const FeatureHistory& history,
                 const std::vector<SketchPlane>& planes,
                 const SimulationSetup* simulation = nullptr);

bool loadProject(const std::string& filepath,
                 FeatureHistory& history,
                 std::vector<SketchPlane>& planes,
                 SimulationSetup* simulation = nullptr);

const std::string& lastLoadError();


bool exportSTL(const std::string& filepath, const Scene3D& scene);
// Imports are features, not direct scene insertions (a body added straight to
// the scene is wiped by the next replay and never saved): STL is a MeshImport
// (App::beginMeshImport), STEP and IGES a CadImport (CadImport.h,
// App::beginCadImport).
bool exportSTEP(const std::string& filepath, const Scene3D& scene);
bool exportIGES(const std::string& filepath, const Scene3D& scene);
bool exportOBJ(const std::string& filepath, const Scene3D& scene);
bool exportDXF(const std::string& filepath, const Sketch& sketch);

} // namespace shitcad
