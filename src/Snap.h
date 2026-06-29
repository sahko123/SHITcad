#pragma once
#include "SketchData.h"

namespace shitcad {

enum class SnapType : uint8_t {
    None,
    Grid,
    Point,
    Midpoint,
    Intersection,
    Quadrant,        // N/S/E/W extreme points of a circle or arc
    NearestOnCurve,  // nearest point on circle/arc edge
    Tangent,         // geometric tangent point from line anchor to circle/arc
};

struct SnapResult {
    SnapType type = SnapType::None;
    Point2D position = {};
    EntityID pointID = NullID;
    EntityID curveID = NullID;  // circle/arc ID when snapping to curve
};

class SnapEngine {
public:
    float snapTolerancePx = 10.0f;
    float curveSnapTolerancePx = 15.0f;
    bool gridSnapEnabled = true;
    bool pointSnapEnabled = true;
    bool midpointSnapEnabled = true;
    bool intersectionSnapEnabled = true;

    SnapResult snap(Point2D cursorWorld, float pixelsPerUnit,
                    const Sketch& sketch, float gridStep) const;
};

} // namespace shitcad
