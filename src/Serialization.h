#pragma once
#include "FeatureHistory.h"
#include "SketchPlane.h"
#include "Scene3D.h"
#include <string>
#include <vector>

namespace shitcad {

bool saveProject(const std::string& filepath,
                 const FeatureHistory& history,
                 const std::vector<SketchPlane>& planes);

bool loadProject(const std::string& filepath,
                 FeatureHistory& history,
                 std::vector<SketchPlane>& planes);

const std::string& lastLoadError();

std::string openNativeOpenDialog();
std::string openNativeSaveDialog();
std::string openNativeStlSaveDialog();
std::string openNativeStepSaveDialog();
std::string openNativeStepOpenDialog();
std::string openNativeIgesSaveDialog();
std::string openNativeIgesOpenDialog();
std::string openNativeObjSaveDialog();
std::string openNativeDxfSaveDialog();
std::string openNativeStlOpenDialog();
std::string openNativeImportDialog(); // combined import dialog (STEP + IGES + STL)

bool exportSTL(const std::string& filepath, const Scene3D& scene);
// STL import is a MeshImport feature (see FeatureHistory.h / App::beginMeshImport),
// not a direct scene insertion: a body added straight to the scene is wiped by
// the next replay and never saved.
bool exportSTEP(const std::string& filepath, const Scene3D& scene);
bool importSTEP(const std::string& filepath, Scene3D& scene);
bool exportIGES(const std::string& filepath, const Scene3D& scene);
bool importIGES(const std::string& filepath, Scene3D& scene);
bool exportOBJ(const std::string& filepath, const Scene3D& scene);
bool exportDXF(const std::string& filepath, const Sketch& sketch);

} // namespace shitcad
