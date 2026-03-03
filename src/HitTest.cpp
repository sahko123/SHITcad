#include "HitTest.h"
#include <cmath>
#include <algorithm>
#include <limits>

namespace shitcad {

float pointToSegmentDist(Point2D p, Point2D a, Point2D b) {
    float dx = b.x - a.x;
    float dy = b.y - a.y;
    float lenSq = dx * dx + dy * dy;

    if (lenSq < 1e-10f) {
        // Degenerate segment (zero length)
        return distance(p, a);
    }

    // Project p onto the line defined by a-b, clamped to [0,1]
    float t = ((p.x - a.x) * dx + (p.y - a.y) * dy) / lenSq;
    t = std::clamp(t, 0.0f, 1.0f);

    Point2D closest = {a.x + t * dx, a.y + t * dy};
    return distance(p, closest);
}

float pointToCircleDist(Point2D p, Point2D center, float radius) {
    float distToCenter = distance(p, center);
    return std::fabs(distToCenter - radius);
}

HitResult hitTest(Point2D cursorWorld, float pixelsPerUnit,
                  const Sketch& sketch, float tolerancePx) {
    HitResult best;
    best.distance = std::numeric_limits<float>::max();

    // Priority 1: Points (smallest entities, easiest to miss without priority)
    for (const auto& pt : sketch.points) {
        float dist = distance(cursorWorld, {pt.x, pt.y});
        float screenDist = dist * pixelsPerUnit;
        if (screenDist < tolerancePx && screenDist < best.distance) {
            best.type = HitType::Point;
            best.entityID = pt.id;
            best.distance = screenDist;
        }
    }

    // If we hit a point, return it (highest priority)
    if (best.type == HitType::Point)
        return best;

    // Priority 2: Lines
    for (const auto& line : sketch.lines) {
        Point2D a = sketch.getPointPos(line.startPt);
        Point2D b = sketch.getPointPos(line.endPt);
        float dist = pointToSegmentDist(cursorWorld, a, b);
        float screenDist = dist * pixelsPerUnit;
        if (screenDist < tolerancePx && screenDist < best.distance) {
            best.type = HitType::Line;
            best.entityID = line.id;
            best.distance = screenDist;
        }
    }

    if (best.type == HitType::Line)
        return best;

    // Priority 3: Circles
    for (const auto& circle : sketch.circles) {
        Point2D center = sketch.getPointPos(circle.centerPt);
        float dist = pointToCircleDist(cursorWorld, center, circle.radius);
        float screenDist = dist * pixelsPerUnit;
        if (screenDist < tolerancePx && screenDist < best.distance) {
            best.type = HitType::Circle;
            best.entityID = circle.id;
            best.distance = screenDist;
        }
    }

    return best;
}

} // namespace shitcad
