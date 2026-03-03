#include "Snap.h"
#include <cmath>
#include <limits>

namespace shitcad {

SnapResult SnapEngine::snap(Point2D cursorWorld, float pixelsPerUnit,
                            const Sketch& sketch, float gridStep) const {
    float worldTolerance = snapTolerancePx / pixelsPerUnit;
    float bestDist = std::numeric_limits<float>::max();
    SnapResult result;
    result.position = cursorWorld;

    // Priority 1: Point snap
    if (pointSnapEnabled) {
        for (const auto& pt : sketch.points) {
            float dist = distance(cursorWorld, {pt.x, pt.y});
            if (dist < worldTolerance && dist < bestDist) {
                bestDist = dist;
                result.type = SnapType::Point;
                result.position = {pt.x, pt.y};
                result.pointID = pt.id;
            }
        }
        if (result.type == SnapType::Point)
            return result;
    }

    // Priority 2: Midpoint snap
    if (midpointSnapEnabled) {
        for (const auto& line : sketch.lines) {
            Point2D a = sketch.getPointPos(line.startPt);
            Point2D b = sketch.getPointPos(line.endPt);
            Point2D mid = midpoint(a, b);
            float dist = distance(cursorWorld, mid);
            if (dist < worldTolerance && dist < bestDist) {
                bestDist = dist;
                result.type = SnapType::Midpoint;
                result.position = mid;
                result.pointID = NullID;
            }
        }
        if (result.type == SnapType::Midpoint)
            return result;
    }

    // Priority 3: Grid snap
    if (gridSnapEnabled && gridStep > 0.0f) {
        float snappedX = std::round(cursorWorld.x / gridStep) * gridStep;
        float snappedY = std::round(cursorWorld.y / gridStep) * gridStep;
        Point2D snapped = {snappedX, snappedY};
        float dist = distance(cursorWorld, snapped);
        if (dist < worldTolerance) {
            result.type = SnapType::Grid;
            result.position = snapped;
            result.pointID = NullID;
            return result;
        }
    }

    return result; // SnapType::None, position = raw cursor
}

} // namespace shitcad
