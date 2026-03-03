#pragma once
#include <vector>
#include <cstdint>
#include <cmath>

namespace shitcad {

using EntityID = uint32_t;
constexpr EntityID NullID = 0;

struct Point2D {
    float x = 0.0f;
    float y = 0.0f;
};

inline float distance(Point2D a, Point2D b) {
    float dx = a.x - b.x, dy = a.y - b.y;
    return std::sqrt(dx * dx + dy * dy);
}

inline Point2D midpoint(Point2D a, Point2D b) {
    return {(a.x + b.x) * 0.5f, (a.y + b.y) * 0.5f};
}

struct PointEntity {
    EntityID id = NullID;
    float x = 0.0f;
    float y = 0.0f;
};

struct LineEntity {
    EntityID id = NullID;
    EntityID startPt = NullID;
    EntityID endPt = NullID;
};

struct CircleEntity {
    EntityID id = NullID;
    EntityID centerPt = NullID;
    float radius = 0.0f;
};

enum class ConstraintType : uint8_t {
    Coincident,
    Horizontal,
    Vertical,
    Distance,
    PointOnLine,
    EqualLength,
    Perpendicular,
    Parallel,
    Tangent,
};

struct Constraint {
    EntityID id = NullID;
    ConstraintType type = ConstraintType::Coincident;
    EntityID entityA = NullID;
    EntityID entityB = NullID;
    float value = 0.0f;
    bool isAuto = false;
};

struct Sketch {
    std::vector<PointEntity> points;
    std::vector<LineEntity> lines;
    std::vector<CircleEntity> circles;
    std::vector<Constraint> constraints;

    EntityID nextID = 1;

    EntityID genID() { return nextID++; }

    // Lookups
    PointEntity* findPoint(EntityID id);
    const PointEntity* findPoint(EntityID id) const;
    LineEntity* findLine(EntityID id);
    CircleEntity* findCircle(EntityID id);
    Constraint* findConstraint(EntityID id);

    // Mutators
    EntityID addPoint(float x, float y);
    EntityID addLine(EntityID startPt, EntityID endPt);
    EntityID addCircle(EntityID centerPt, float radius);
    EntityID addConstraint(ConstraintType type, EntityID a, EntityID b,
                           float value = 0.0f, bool isAuto = false);
    void removeConstraint(EntityID id);

    // Find nearest point within tolerance (world units). Returns NullID if none.
    EntityID findPointNear(float wx, float wy, float tolerance) const;

    Point2D getPointPos(EntityID id) const;

    void clear();
};

} // namespace shitcad
