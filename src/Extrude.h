#pragma once
#include "SketchData.h"
#include "ProfileDetector.h"
#include "SketchPlane.h"
#include <TopoDS_Shape.hxx>

namespace shitcad {

// Result from shape-building operations, with optional error message.
struct ShapeResult {
    TopoDS_Shape shape;
    std::string error;  // non-empty on failure
    bool ok() const { return !shape.IsNull() && error.empty(); }
};

// Extrude a closed profile into a 3D OCCT shape (prism).
// Sketch points are in the plane's local 2D coords, transformed to 3D via the plane.
// Extrudes along the plane normal by the given height.
TopoDS_Shape extrudeProfile(const Sketch& sketch, const ClosedProfile& profile,
                            float height, const SketchPlane& plane);

// Extended extrusion with offset and support for negative height (other-side direction).
// offset: translates base face along normal before extruding.
// height: can be negative (extrudes in -normal direction).
TopoDS_Shape extrudeProfileEx(const Sketch& sketch, const ClosedProfile& profile,
                               float height, float offset, const SketchPlane& plane);

struct ExtrudeToolState; // forward decl

// Build a combined OCCT tool shape from the extrude tool state (selected profiles).
// Returns ShapeResult with error details on failure.
ShapeResult buildExtrudeToolShapeEx(const ExtrudeToolState& state,
                                    const Sketch& sketch,
                                    const SketchPlane& plane);

// Legacy wrapper (returns shape only, no error).
TopoDS_Shape buildExtrudeToolShape(const ExtrudeToolState& state,
                                   const Sketch& sketch,
                                   const SketchPlane& plane);

// Revolve a closed profile around a sketch-line axis.
// axisA, axisB are the 2D local endpoints of the axis line.
// angleDeg is the revolution angle in degrees (360 = full).
TopoDS_Shape revolveProfile(const Sketch& sketch, const ClosedProfile& profile,
                            Point2D axisA, Point2D axisB,
                            float angleDeg, const SketchPlane& plane);

struct RevolveToolState; // forward decl

// Build a combined OCCT tool shape from the revolve tool state.
ShapeResult buildRevolveToolShapeEx(const RevolveToolState& state,
                                    const Sketch& sketch,
                                    const SketchPlane& plane);

TopoDS_Shape buildRevolveToolShape(const RevolveToolState& state,
                                   const Sketch& sketch,
                                   const SketchPlane& plane);

// Loft through a series of wires to create a smooth solid.
// Each wire is built from a profile on its sketch plane.
struct LoftWireInput {
    const Sketch* sketch;
    const ClosedProfile* profile;
    const SketchPlane* plane;
};
ShapeResult loftProfilesEx(const std::vector<LoftWireInput>& sections, bool solid);
TopoDS_Shape loftProfiles(const std::vector<LoftWireInput>& sections, bool solid);

// Enumerate disconnected solids in a shape.
std::vector<TopoDS_Shape> enumerateSolids(const TopoDS_Shape& shape);

} // namespace shitcad
