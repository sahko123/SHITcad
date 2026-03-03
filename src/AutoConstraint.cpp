#include "AutoConstraint.h"
#include <cmath>

namespace shitcad {

static constexpr float kAutoAngleThresholdDeg = 5.0f;

std::vector<PendingConstraint> detectLineAutoConstraints(
    const Sketch& sketch, EntityID lineID) {
    std::vector<PendingConstraint> result;

    const LineEntity* line = nullptr;
    for (const auto& l : sketch.lines) {
        if (l.id == lineID) { line = &l; break; }
    }
    if (!line) return result;

    Point2D a = sketch.getPointPos(line->startPt);
    Point2D b = sketch.getPointPos(line->endPt);

    float dx = b.x - a.x;
    float dy = b.y - a.y;
    float len = std::sqrt(dx * dx + dy * dy);
    if (len < 1e-6f) return result;

    float sinAngle = std::sin(kAutoAngleThresholdDeg * 3.14159265f / 180.0f);

    // Check horizontal: |dy/len| < sin(threshold)
    if (std::fabs(dy / len) < sinAngle) {
        result.push_back({ConstraintType::Horizontal, lineID});
    }
    // Check vertical: |dx/len| < sin(threshold)
    else if (std::fabs(dx / len) < sinAngle) {
        result.push_back({ConstraintType::Vertical, lineID});
    }

    return result;
}

} // namespace shitcad
