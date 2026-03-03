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

void Sketch::removeConstraintsReferencing(EntityID id) {
    constraints.erase(
        std::remove_if(constraints.begin(), constraints.end(),
            [id](const Constraint& c) {
                return c.entityA == id || c.entityB == id;
            }),
        constraints.end());
}

bool Sketch::isPointReferenced(EntityID pointID) const {
    for (const auto& line : lines) {
        if (line.startPt == pointID || line.endPt == pointID)
            return true;
    }
    for (const auto& circle : circles) {
        if (circle.centerPt == pointID)
            return true;
    }
    return false;
}

void Sketch::removePoint(EntityID id) {
    // Remove all lines that reference this point
    std::vector<EntityID> linesToRemove;
    for (const auto& line : lines) {
        if (line.startPt == id || line.endPt == id)
            linesToRemove.push_back(line.id);
    }
    for (EntityID lineID : linesToRemove) {
        removeLine(lineID);
    }

    // Remove all circles that reference this point
    std::vector<EntityID> circlesToRemove;
    for (const auto& circle : circles) {
        if (circle.centerPt == id)
            circlesToRemove.push_back(circle.id);
    }
    for (EntityID circleID : circlesToRemove) {
        removeCircle(circleID);
    }

    // Remove constraints referencing this point
    removeConstraintsReferencing(id);

    // Remove the point itself
    points.erase(
        std::remove_if(points.begin(), points.end(),
            [id](const PointEntity& p) { return p.id == id; }),
        points.end());
}

void Sketch::removeLine(EntityID id) {
    LineEntity* line = findLine(id);
    if (!line) return;

    EntityID startPt = line->startPt;
    EntityID endPt = line->endPt;

    // Remove constraints referencing this line
    removeConstraintsReferencing(id);

    // Remove the line
    lines.erase(
        std::remove_if(lines.begin(), lines.end(),
            [id](const LineEntity& l) { return l.id == id; }),
        lines.end());

    // Remove orphaned points
    if (!isPointReferenced(startPt)) {
        removeConstraintsReferencing(startPt);
        points.erase(
            std::remove_if(points.begin(), points.end(),
                [startPt](const PointEntity& p) { return p.id == startPt; }),
            points.end());
    }
    if (!isPointReferenced(endPt)) {
        removeConstraintsReferencing(endPt);
        points.erase(
            std::remove_if(points.begin(), points.end(),
                [endPt](const PointEntity& p) { return p.id == endPt; }),
            points.end());
    }
}

void Sketch::removeCircle(EntityID id) {
    CircleEntity* circle = findCircle(id);
    if (!circle) return;

    EntityID centerPt = circle->centerPt;

    // Remove constraints referencing this circle
    removeConstraintsReferencing(id);

    // Remove the circle
    circles.erase(
        std::remove_if(circles.begin(), circles.end(),
            [id](const CircleEntity& c) { return c.id == id; }),
        circles.end());

    // Remove orphaned center point
    if (!isPointReferenced(centerPt)) {
        removeConstraintsReferencing(centerPt);
        points.erase(
            std::remove_if(points.begin(), points.end(),
                [centerPt](const PointEntity& p) { return p.id == centerPt; }),
            points.end());
    }
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
