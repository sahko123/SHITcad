#include "SketchData.h"
#include <algorithm>
#include <limits>

namespace shitcad {

PointEntity* Sketch::findPoint(EntityID id) {
    for (auto& p : points)
        if (p.id == id) return &p;
    return nullptr;
}

const PointEntity* Sketch::findPoint(EntityID id) const {
    for (auto& p : points)
        if (p.id == id) return &p;
    return nullptr;
}

LineEntity* Sketch::findLine(EntityID id) {
    for (auto& l : lines)
        if (l.id == id) return &l;
    return nullptr;
}

CircleEntity* Sketch::findCircle(EntityID id) {
    for (auto& c : circles)
        if (c.id == id) return &c;
    return nullptr;
}

Constraint* Sketch::findConstraint(EntityID id) {
    for (auto& c : constraints)
        if (c.id == id) return &c;
    return nullptr;
}

EntityID Sketch::addPoint(float x, float y) {
    EntityID id = genID();
    points.push_back({id, x, y});
    return id;
}

EntityID Sketch::addLine(EntityID startPt, EntityID endPt) {
    EntityID id = genID();
    lines.push_back({id, startPt, endPt});
    return id;
}

EntityID Sketch::addCircle(EntityID centerPt, float radius) {
    EntityID id = genID();
    circles.push_back({id, centerPt, radius});
    return id;
}

EntityID Sketch::addConstraint(ConstraintType type, EntityID a, EntityID b,
                               float value, bool isAuto) {
    EntityID id = genID();
    constraints.push_back({id, type, a, b, value, isAuto});
    return id;
}

void Sketch::removeConstraint(EntityID id) {
    constraints.erase(
        std::remove_if(constraints.begin(), constraints.end(),
                        [id](const Constraint& c) { return c.id == id; }),
        constraints.end());
}

EntityID Sketch::findPointNear(float wx, float wy, float tolerance) const {
    EntityID bestID = NullID;
    float bestDist = std::numeric_limits<float>::max();

    for (const auto& p : points) {
        float dx = p.x - wx;
        float dy = p.y - wy;
        float dist = std::sqrt(dx * dx + dy * dy);
        if (dist < tolerance && dist < bestDist) {
            bestDist = dist;
            bestID = p.id;
        }
    }
    return bestID;
}

Point2D Sketch::getPointPos(EntityID id) const {
    const PointEntity* p = findPoint(id);
    if (p) return {p->x, p->y};
    return {0.0f, 0.0f};
}

void Sketch::clear() {
    points.clear();
    lines.clear();
    circles.clear();
    constraints.clear();
    nextID = 1;
}

} // namespace shitcad
