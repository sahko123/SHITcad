#pragma once
#include "SketchData.h"

namespace shitcad {

enum class HitType : uint8_t {
    None,
    Point,
    Line,
    Circle,
};

struct HitResult {
    HitType type = HitType::None;
    EntityID entityID = NullID;
    float distance = 0.0f; // screen-space distance to entity
};

// Distance from a point to a line segment (in world space)
float pointToSegmentDist(Point2D p, Point2D a, Point2D b);

// Distance from a point to a circle edge (in world space)
float pointToCircleDist(Point2D p, Point2D center, float radius);

// Find the closest entity to the cursor. Returns HitType::None if nothing within tolerance.
// All distances are in screen pixels.
HitResult hitTest(Point2D cursorWorld, float pixelsPerUnit,
                  const Sketch& sketch, float tolerancePx);

} // namespace shitcad
