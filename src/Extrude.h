#pragma once
#include "SketchData.h"
#include "ProfileDetector.h"
#include "SketchPlane.h"
#include <TopoDS_Shape.hxx>

namespace shitcad {

// Extended extrusion with offset and support for negative height (other-side direction).
// offset: translates base face along normal before extruding.
// height: can be negative (extrudes in -normal direction).
TopoDS_Shape extrudeProfileEx(const Sketch& sketch, const ClosedProfile& profile,
                               float height, float offset, const SketchPlane& plane);

struct ExtrudeToolState; // forward decl

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
TopoDS_Shape loftProfiles(const std::vector<LoftWireInput>& sections, bool solid);

// Enumerate disconnected solids in a shape.
std::vector<TopoDS_Shape> enumerateSolids(const TopoDS_Shape& shape);

} // namespace shitcad
