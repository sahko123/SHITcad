#pragma once
#include "SketchData.h"
#include <vector>

namespace shitcad {

struct SegIntersectResult {
    Point2D point;
    float tA; // parametric position on segment A (0..1)
    float tB; // parametric position on segment B (0..1)
};

// Returns intersection of two line segments if they cross.
// tA/tB are parametric positions along each segment.
bool segSegIntersection(Point2D a1, Point2D a2, Point2D b1, Point2D b2,
                        SegIntersectResult& out);

struct IntersectionInfo {
    Point2D point;
    int lineIdxA;
    int lineIdxB;
    float tA;
    float tB;
};

// Compute all line-line intersection points in a sketch (including T-junctions).
std::vector<IntersectionInfo> computeAllLineLineIntersections(const Sketch& sketch);

// Circle-line intersection: returns 0-2 hits
struct CircleLineHit {
    Point2D point;
    float tLine;  // parametric position on line segment (0..1)
    float angle;  // angle on circle (radians, atan2-convention)
};
std::vector<CircleLineHit> circleLineIntersection(
    Point2D center, float radius, Point2D a, Point2D b);

// Circle-circle intersection: returns 0-2 hits
struct CircleCircleHit {
    Point2D point;
    float angle1; // angle on circle 1
    float angle2; // angle on circle 2
};
std::vector<CircleCircleHit> circleCircleIntersection(
    Point2D c1, float r1, Point2D c2, float r2);

} // namespace shitcad
