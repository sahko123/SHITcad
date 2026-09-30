#pragma once
#include "MeshImport.h"
#include <TopoDS_Shape.hxx>
#include <string>
#include <vector>

namespace shitcad {

// STEP and IGES import. Unlike an STL, these are B-rep: the bodies an import
// creates are ordinary solids that booleans, face picking, sketch-on-face and
// export all work on. Only the file path and a placement are stored in the
// project (CadImportFeatureData); replay reads the file again, cached.

// One body an import creates: a solid, or a loose shell / face group for
// surface models. Name and colour come from the file when it has them.
struct CadPart {
    TopoDS_Shape shape;       // in mm, file coordinates (no placement)
    std::string name;
    bool hasColor = false;
    float color[3] = {0, 0, 0};
};

struct CadFileInfo {
    std::string format;       // "STEP" or "IGES"
    std::string fileUnit;     // the file's own length unit, e.g. "mm", "inch"
    int solids = 0;
    int surfaces = 0;         // shells or face groups that are not solids
    int skippedWires = 0;     // curves and points, which make no body
    double lo[3] = {0, 0, 0}; // bounds in mm, before placement
    double hi[3] = {0, 0, 0};
    double deflection = 0.01; // display tessellation tolerance, mm
};

// True for .step/.stp/.igs/.iges (any case).
bool isCadFile(const std::string& path);

// Read a STEP or IGES file, converted to mm, split into parts.
//
// Cached by path, size, modification time and a sampled content hash (the
// same key as STL imports), so replay reads an unchanged file once and picks
// up a re-exported one on the next replay. The parts are tessellated here, at
// `info.deflection`, and placing them only changes their location, so every
// replay shares the tessellation: add them with Scene3D::addBody(..., true),
// or a face OCCT could not mesh is retried (and fails again) on every replay.
bool loadCadFile(const std::string& path, std::vector<CadPart>& parts, CadFileInfo& info,
                 std::string& error);

// The part moved by a placement (rotation + translation only, so the shape's
// geometry and tessellation are shared with the cached original).
TopoDS_Shape placeCadShape(const TopoDS_Shape& shape, const MeshTransform& xf);

// Bounds in mm of every part after placement, from their tessellation - exact
// for any rotation, unlike transforming the file's bounding box.
bool cadPlacedBounds(const std::vector<CadPart>& parts, const MeshTransform& xf,
                     double lo[3], double hi[3]);

// Display tessellation tolerance for a part of this size: 0.01 mm up to a
// 50 mm part, then proportional, so a metre-scale model is not meshed into
// millions of triangles on the UI thread.
double tessellationDeflection(const double lo[3], const double hi[3]);

} // namespace shitcad
