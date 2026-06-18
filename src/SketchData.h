#pragma once
#include <vector>
#include <unordered_map>
#include <string>
#include <cstdint>
#include <cmath>

namespace shitcad {

using EntityID = uint32_t;
constexpr EntityID NullID = 0;

struct Point2D {
    double x = 0.0;
    double y = 0.0;
};

inline double distance(Point2D a, Point2D b) {
    double dx = a.x - b.x, dy = a.y - b.y;
    return std::sqrt(dx * dx + dy * dy);
}

inline Point2D midpoint(Point2D a, Point2D b) {
    return {(a.x + b.x) * 0.5, (a.y + b.y) * 0.5};
}

struct PointEntity {
    EntityID id = NullID;
    double x = 0.0;
    double y = 0.0;
    bool projected = false;
};

struct LineEntity {
    EntityID id = NullID;
    EntityID startPt = NullID;
    EntityID endPt = NullID;
    bool projected = false;
};

struct CircleEntity {
    EntityID id = NullID;
    EntityID centerPt = NullID;
    double radius = 0.0;
    bool projected = false;
};

struct ArcEntity {
    EntityID id = NullID;
    EntityID centerPt = NullID;
    EntityID startPt = NullID;
    EntityID endPt = NullID;
    double startAngle = 0.0;  // radians, CCW sweep from start to end
    double endAngle = 0.0;
    bool projected = false;
};

struct EllipseEntity {
    EntityID id = NullID;
    EntityID centerPt = NullID;
    double semiMajor = 0.0;   // half-length of major axis
    double semiMinor = 0.0;   // half-length of minor axis
    double rotation = 0.0;    // angle of major axis in radians (CCW from +X)
    bool projected = false;
};

struct EllipseArcEntity {
    EntityID id = NullID;
    EntityID centerPt = NullID;
    EntityID startPt = NullID;
    EntityID endPt = NullID;
    double semiMajor = 0.0;
    double semiMinor = 0.0;
    double rotation = 0.0;
    double startAngle = 0.0;
    double endAngle = 0.0;
    bool projected = false;
};

struct SplineEntity {
    EntityID id = NullID;
    std::vector<EntityID> controlPtIDs;
    int degree = 3;
    bool periodic = false;
    std::vector<double> knots;
    std::vector<double> weights; // empty = non-rational (uniform weights)
    bool projected = false;
};

enum class ConstraintType : uint8_t {
    Coincident,
    Horizontal,
    Vertical,
    Distance,
    Radius,
    Diameter,
    PointDistance,
    PointOnLine,
    PointLineDistance,
    EqualLength,
    Perpendicular,
    Parallel,
    Collinear,
    Tangent,
    Angle,
    Symmetric,
    Concentric,
    Midpoint,
    PointOnCircle,
};

struct Constraint {
    EntityID id = NullID;
    ConstraintType type = ConstraintType::Coincident;
    EntityID entityA = NullID;
    EntityID entityB = NullID;
    EntityID entityC = NullID;  // for Symmetric: axis line ID
    double value = 0.0;        // always stored in primary units (mm)
    bool isAuto = false;
    std::string inputUnit;     // unit as entered (e.g. "in", "cm", "foot"); empty = primary
    double inputValue = 0.0;   // value in the input unit
    double dimOffsetX = 0.0;   // local 2D placement offset from geometry midpoint
    double dimOffsetY = 0.0;
    bool driven = false;       // true = reference-only (not enforced by solver)
    bool angleCW = false;      // for Angle constraints: true = CW sector, false = CCW sector
    bool negativeSide = false;  // for PointLineDistance: true = point on negative (CW) side of line
};

struct Sketch {
    std::vector<PointEntity> points;
    std::vector<LineEntity> lines;
    std::vector<CircleEntity> circles;
    std::vector<ArcEntity> arcs;
    std::vector<EllipseEntity> ellipses;
    std::vector<EllipseArcEntity> ellipseArcs;
    std::vector<SplineEntity> splines;
    std::vector<Constraint> constraints;

    EntityID nextID = 1;
    bool dirty = true; // set when sketch changes; cleared after solver runs

    EntityID genID() { return nextID++; }

    // Lookups
    PointEntity* findPoint(EntityID id);
    const PointEntity* findPoint(EntityID id) const;
    LineEntity* findLine(EntityID id);
    const LineEntity* findLine(EntityID id) const;
    CircleEntity* findCircle(EntityID id);
    const CircleEntity* findCircle(EntityID id) const;
    ArcEntity* findArc(EntityID id);
    const ArcEntity* findArc(EntityID id) const;
    EllipseEntity* findEllipse(EntityID id);
    const EllipseEntity* findEllipse(EntityID id) const;
    EllipseArcEntity* findEllipseArc(EntityID id);
    const EllipseArcEntity* findEllipseArc(EntityID id) const;
    SplineEntity* findSpline(EntityID id);
    const SplineEntity* findSpline(EntityID id) const;
    Constraint* findConstraint(EntityID id);

    // Mutators
    EntityID addPoint(double x, double y);
    EntityID addLine(EntityID startPt, EntityID endPt);
    EntityID addCircle(EntityID centerPt, double radius);
    EntityID addArc(EntityID centerPt, EntityID startPt, EntityID endPt);
    EntityID addEllipse(EntityID centerPt, double semiMajor, double semiMinor, double rotation);
    EntityID addEllipseArc(EntityID centerPt, EntityID startPt, EntityID endPt,
                            double semiMajor, double semiMinor, double rotation);
    EntityID addSpline(const std::vector<EntityID>& controlPts, int degree = 3, bool periodic = false);
    EntityID addConstraint(ConstraintType type, EntityID a, EntityID b,
                           double value = 0.0, bool isAuto = false);
    void removeConstraint(EntityID id);

    // Remove entities with cascade (removes referencing constraints + orphaned points)
    void removePoint(EntityID id);
    void removeLine(EntityID id);
    void removeCircle(EntityID id);
    void removeArc(EntityID id);
    void removeEllipse(EntityID id);
    void removeEllipseArc(EntityID id);
    void removeSpline(EntityID id);

    // Check if a point is referenced by any line, circle, arc, ellipse, or spline
    bool isPointReferenced(EntityID pointID) const;

    // Returns true if the point can be removed (not projected and not referenced)
    bool canRemovePoint(EntityID pointID) const;

    // Remove all constraints that reference a given entity
    void removeConstraintsReferencing(EntityID id);

    // Find nearest point within tolerance (world units). Returns NullID if none.
    EntityID findPointNear(double wx, double wy, double tolerance) const;

    Point2D getPointPos(EntityID id) const;

    // Recompute arc start/end angles from current point positions
    void recomputeArcAngles(ArcEntity& arc) const;

    void clear();
    void clearProjected();

    // Rebuild O(1) lookup indices. Call after bulk modifications.
    void rebuildIndices();

private:
    // O(1) lookup maps (EntityID → index in vector)
    std::unordered_map<EntityID, size_t> pointIndex_;
    std::unordered_map<EntityID, size_t> lineIndex_;
    std::unordered_map<EntityID, size_t> circleIndex_;
    std::unordered_map<EntityID, size_t> arcIndex_;
    std::unordered_map<EntityID, size_t> ellipseIndex_;
    std::unordered_map<EntityID, size_t> ellipseArcIndex_;
    std::unordered_map<EntityID, size_t> splineIndex_;
    std::unordered_map<EntityID, size_t> constraintIndex_;
};

// ─── Curve sampling utilities ──────────────────────────────────────

// Sample a full ellipse into numSamples+1 points (closed loop)
std::vector<Point2D> sampleEllipse(Point2D center, double semiMajor, double semiMinor,
                                    double rotation, int numSamples = 64);

// Sample an ellipse arc
std::vector<Point2D> sampleEllipseArc(Point2D center, double semiMajor, double semiMinor,
                                       double rotation, double startAngle, double endAngle,
                                       int numSamples = 32);

// Evaluate a B-spline/NURBS curve at parameter t (de Boor's algorithm)
Point2D evaluateBSpline(const std::vector<Point2D>& ctrlPts, const std::vector<double>& knots,
                         const std::vector<double>& weights, int degree, double t);

// Generate a uniform clamped knot vector for n control points and given degree
std::vector<double> generateUniformKnots(int numCtrlPts, int degree);

// Sample a spline entity into numSamples+1 points
std::vector<Point2D> sampleSpline(const SplineEntity& sp, const Sketch& sketch, int numSamples = 64);

} // namespace shitcad
