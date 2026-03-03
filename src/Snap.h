#pragma once
#include "SketchData.h"

namespace shitcad {

enum class SnapType : uint8_t {
    None,
    Grid,
    Point,
    Midpoint,
};

struct SnapResult {
    SnapType type = SnapType::None;
    Point2D position = {};
    EntityID pointID = NullID;
};

class SnapEngine {
public:
    float snapTolerancePx = 10.0f;
    bool gridSnapEnabled = true;
    bool pointSnapEnabled = true;
    bool midpointSnapEnabled = true;

    SnapResult snap(Point2D cursorWorld, float pixelsPerUnit,
                    const Sketch& sketch, float gridStep) const;
};

} // namespace shitcad
