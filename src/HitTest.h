#pragma once
#include "SketchData.h"
#include <vector>

namespace shitcad {

enum class HitType : uint8_t {
    None,
    Point,
    Line,
    Circle,
    Arc,
    Ellipse,
    EllipseArc,
    Spline,
    Dimension,
};

struct HitResult {
    HitType type = HitType::None;
    EntityID entityID = NullID;
    float distance = 0.0f; // screen-space distance to entity
};

// Distance from a point to a line segment (in world space)
double pointToSegmentDist(Point2D p, Point2D a, Point2D b);

// Distance from a point to a circle edge (in world space)
double pointToCircleDist(Point2D p, Point2D center, double radius);

// Distance from a point to an arc edge (in world space)
double pointToArcDist(Point2D p, Point2D center, double radius, double startAngle, double endAngle);

// Distance from a point to an ellipse edge (in world space)
double pointToEllipseDist(Point2D p, Point2D center, double semiMajor, double semiMinor, double rotation);

// Distance from a point to a spline (polyline approximation, in world space)
double pointToSplineDist(Point2D p, const std::vector<Point2D>& samplePts);

// Find the closest entity to the cursor. Returns HitType::None if nothing within tolerance.
// All distances are in screen pixels.
HitResult hitTest(Point2D cursorWorld, float pixelsPerUnit,
                  const Sketch& sketch, float tolerancePx);

// --- Containment tests for box/lasso selection ---

// Point inside axis-aligned rect [min, max]
bool pointInRect(Point2D p, Point2D min, Point2D max);

// Point inside arbitrary polygon (ray-casting algorithm)
bool pointInPolygon(Point2D p, const std::vector<Point2D>& poly);

// Line segment fully inside polygon (both endpoints)
bool segmentInPolygon(Point2D a, Point2D b, const std::vector<Point2D>& poly);

// Circle fully inside polygon (sample circumference + center)
bool circleInPolygon(Point2D center, double radius, const std::vector<Point2D>& poly);

} // namespace shitcad
