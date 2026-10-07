#pragma once
#include "SketchPlane.h"
#include "Scene3D.h"
#include "Section.h"
#include <TopoDS_Face.hxx>

namespace shitcad {

struct FacePickResult {
    bool hit = false;
    int bodyIndex = -1;
    TopoDS_Face face;
    float hitWorld[3] = {0, 0, 0};
    float t = 1e30f;
};

// `section`: hits on the cut-away side are ignored, so a click in a section
// view lands on what is actually on screen rather than on the hidden near wall.
FacePickResult pickFace(const Scene3D& scene, const float rayOrigin[3], const float rayDir[3],
                        const SectionPlane* section = nullptr);

// Picking on mesh-only bodies (imported STL), which pickFace cannot see because
// they have no TopoDS faces.
struct MeshPickResult {
    bool hit = false;
    int bodyIndex = -1;
    int triangleIndex = -1;       // index into the body's triangles (vertices / 3)
    float hitWorld[3] = {0, 0, 0}; // mm
    float normal[3] = {0, 0, 1};  // geometric face normal, oriented by the file's winding
    bool frontFacing = false;     // true if the ray hit the side the normal points to
    float t = 1e30f;
};

// Nearest triangle hit over all visible mesh-only bodies. Brute force: fine for
// clicks and hover on exports of a few hundred thousand triangles.
MeshPickResult pickMesh(const Scene3D& scene, const float rayOrigin[3], const float rayDir[3],
                        const SectionPlane* section = nullptr);

// Extract a sketch plane from a face. For non-planar faces, pass the hit point
// to compute a tangent plane at that location.
bool extractPlaneFromFace(const TopoDS_Face& face, SketchPlane& out,
                          const float* hitWorld = nullptr);

// Project all edges of a face onto the sketch plane as projected (locked) entities
void projectFaceOntoSketch(const TopoDS_Face& face, const SketchPlane& plane, Sketch& sketch);

// Check if a face is cylindrical
bool isCylindricalFace(const TopoDS_Face& face);

// Build a tangent plane to a cylinder at a given angle (degrees) around the axis.
// angle=0 corresponds to the hit point direction. The plane origin is at the
// cylinder surface, midway along the cylinder height.
bool buildCylinderTangentPlane(const TopoDS_Face& face, float angleDeg,
                                const float* hitWorld, SketchPlane& out);

} // namespace shitcad
