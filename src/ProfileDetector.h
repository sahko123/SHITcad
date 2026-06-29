#pragma once
#include "SketchData.h"
#include "Constants.h"
#include <vector>
#include <cmath>
#include <algorithm>

namespace shitcad {

// Segment type for profile boundary edges
enum class SegmentType : uint8_t { Line, Arc, Ellipse, EllipseArc, Spline };

// Describes one edge of a profile boundary
struct BoundarySegment {
    SegmentType type = SegmentType::Line;

    // Arc data (type == Arc):
    Point2D arcCenter;
    double arcRadius = 0;
    double arcStartAngle = 0; // angle at start vertex (radians)
    double arcEndAngle = 0;   // angle at end vertex (radians)
    EntityID origCircleID = NullID;

    // Ellipse / EllipseArc data (type == Ellipse or EllipseArc):
    EntityID origEllipseID = NullID;
    Point2D ellipseCenter;
    double semiMajor = 0;
    double semiMinor = 0;
    double ellipseRotation = 0;    // major axis angle (radians, CCW from +X)
    double ellipseStartAngle = 0;  // parametric start angle on ellipse
    double ellipseEndAngle = 0;    // parametric end angle on ellipse

    // Spline data (type == Spline):
    EntityID origSplineID = NullID;
    std::vector<Point2D> splineControlPts;
    std::vector<double> splineKnots;
    std::vector<double> splineWeights;
    int splineDegree = 3;
    double splineParamStart = 0;  // parameter range on the B-spline
    double splineParamEnd = 1;
};

// A closed profile (region): an outer boundary with optional inner holes.
// In Fusion 360 style, an inner boundary creates a hole in the containing profile.
struct ClosedProfile {
    std::vector<EntityID> pointIDs;  // ordered loop of points (empty for circles)
    std::vector<EntityID> lineIDs;   // lines connecting consecutive points (empty for circles)
    std::vector<EntityID> ellipseIDs;   // ellipse/ellipseArc entities on boundary
    std::vector<EntityID> splineIDs;    // spline entities on boundary
    EntityID circleID = NullID;      // non-null if this profile is a circle
    bool isCircle() const { return circleID != NullID; }

    // Resolved 2D coordinates for the boundary (used when profile contains
    // virtual subdivision points from line intersections). When non-empty,
    // the extrude pipeline uses these instead of looking up pointIDs.
    std::vector<Point2D> resolvedPoints;

    // Per-edge segment info (same length as resolvedPoints).
    // segments[i] describes the edge from resolvedPoints[i] to resolvedPoints[(i+1)%n].
    std::vector<BoundarySegment> segments;

    // Inner holes: boundaries contained within this profile that cut out material
    std::vector<ClosedProfile> holes;
};

// Tessellate a profile's boundary into a dense polyline, subdividing arc
// segments into small straight segments (~1 point per 5 degrees).
// For line-only profiles this just copies resolvedPoints/pointIDs.
inline std::vector<Point2D> tessellateProfile(const Sketch& sketch, const ClosedProfile& profile) {
    std::vector<Point2D> tess;
    if (profile.isCircle()) {
        const CircleEntity* circle = sketch.findCircle(profile.circleID);
        if (!circle) return tess;
        Point2D center = sketch.getPointPos(circle->centerPt);
        int steps = kCircleTessSteps;
        for (int i = 0; i < steps; i++) {
            double angle = kTwoPi * i / steps;
            tess.push_back({center.x + circle->radius * std::cos(angle),
                            center.y + circle->radius * std::sin(angle)});
        }
        return tess;
    }

    const bool useResolved = !profile.resolvedPoints.empty();
    int n = useResolved ? (int)profile.resolvedPoints.size() : (int)profile.pointIDs.size();

    for (int i = 0; i < n; i++) {
        Point2D pt = useResolved ? profile.resolvedPoints[i] : sketch.getPointPos(profile.pointIDs[i]);
        tess.push_back(pt);

        if (!profile.segments.empty() && i < (int)profile.segments.size()) {
            const auto& seg = profile.segments[i];
            if (seg.type == SegmentType::Arc) {
                double span = seg.arcEndAngle - seg.arcStartAngle;
                int steps = std::max(4, (int)(std::fabs(span) / (kArcTessDegreesPerStep * kDegToRad)));
                for (int s = 1; s < steps; s++) {
                    double t = (double)s / steps;
                    double angle = seg.arcStartAngle + t * span;
                    tess.push_back({seg.arcCenter.x + seg.arcRadius * std::cos(angle),
                                    seg.arcCenter.y + seg.arcRadius * std::sin(angle)});
                }
            } else if (seg.type == SegmentType::Ellipse || seg.type == SegmentType::EllipseArc) {
                double span = seg.ellipseEndAngle - seg.ellipseStartAngle;
                int steps = std::max(8, (int)(std::fabs(span) / (kArcTessDegreesPerStep * kDegToRad)));
                double cosR = std::cos(seg.ellipseRotation);
                double sinR = std::sin(seg.ellipseRotation);
                for (int s = 1; s < steps; s++) {
                    double t = (double)s / steps;
                    double angle = seg.ellipseStartAngle + t * span;
                    double lx = seg.semiMajor * std::cos(angle);
                    double ly = seg.semiMinor * std::sin(angle);
                    tess.push_back({seg.ellipseCenter.x + lx * cosR - ly * sinR,
                                    seg.ellipseCenter.y + lx * sinR + ly * cosR});
                }
            } else if (seg.type == SegmentType::Spline) {
                int steps = std::max(16, (int)seg.splineControlPts.size() * 8);
                double paramSpan = seg.splineParamEnd - seg.splineParamStart;
                for (int s = 1; s < steps; s++) {
                    double t = seg.splineParamStart + paramSpan * (double)s / steps;
                    Point2D segPt = evaluateBSpline(seg.splineControlPts, seg.splineKnots,
                                                     seg.splineWeights, seg.splineDegree, t);
                    tess.push_back(segPt);
                }
            }
        }
    }
    return tess;
}

// ---- Geometry utilities for tessellated polygons ----

// Centroid (average of vertices) of a tessellated polygon.
inline Point2D polygonCentroid(const std::vector<Point2D>& poly) {
    if (poly.empty()) return {0, 0};
    double cx = 0, cy = 0;
    for (const auto& p : poly) { cx += p.x; cy += p.y; }
    return {cx / poly.size(), cy / poly.size()};
}

// Unsigned area of a tessellated polygon (shoelace formula).
inline double polygonArea(const std::vector<Point2D>& poly) {
    int n = (int)poly.size();
    if (n < 3) return 0;
    double area = 0;
    for (int i = 0; i < n; i++) {
        Point2D a = poly[i], b = poly[(i + 1) % n];
        area += a.x * b.y - b.x * a.y;
    }
    return std::fabs(area) * 0.5;
}

// Signed area (positive = CCW, negative = CW).
inline double polygonSignedArea(const std::vector<Point2D>& poly) {
    int n = (int)poly.size();
    if (n < 3) return 0;
    double area = 0;
    for (int i = 0; i < n; i++) {
        Point2D a = poly[i], b = poly[(i + 1) % n];
        area += a.x * b.y - b.x * a.y;
    }
    return area;
}

// Ray-casting point-in-polygon test.
inline bool pointInsidePolygon(const std::vector<Point2D>& poly, Point2D p) {
    int n = (int)poly.size();
    if (n < 3) return false;
    bool inside = false;
    for (int i = 0, j = n - 1; i < n; j = i++) {
        Point2D a = poly[i], b = poly[j];
        if (((a.y > p.y) != (b.y > p.y)) &&
            (p.x < (b.x - a.x) * (p.y - a.y) / (b.y - a.y) + a.x)) {
            inside = !inside;
        }
    }
    return inside;
}

// Winding number test: returns the winding number of a polygon around point p.
// Non-zero winding number means the point is inside (handles complex/self-intersecting cases).
inline int windingNumber(const std::vector<Point2D>& poly, Point2D p) {
    int n = (int)poly.size();
    if (n < 3) return 0;
    int wn = 0;
    for (int i = 0; i < n; i++) {
        Point2D a = poly[i], b = poly[(i + 1) % n];
        if (a.y <= p.y) {
            if (b.y > p.y) {
                // Upward crossing: check if p is left of edge a->b
                double cross = (b.x - a.x) * (p.y - a.y) - (p.x - a.x) * (b.y - a.y);
                if (cross > 0) ++wn;
            }
        } else {
            if (b.y <= p.y) {
                // Downward crossing: check if p is right of edge a->b
                double cross = (b.x - a.x) * (p.y - a.y) - (p.x - a.x) * (b.y - a.y);
                if (cross < 0) --wn;
            }
        }
    }
    return wn;
}

// Point-in-polygon using winding number (more robust than ray-casting for edge cases).
inline bool pointInsidePolygonWinding(const std::vector<Point2D>& poly, Point2D p) {
    return windingNumber(poly, p) != 0;
}

// Detect all closed profiles using the custom half-edge tracer.
std::vector<ClosedProfile> detectClosedProfilesCustom(const Sketch& sketch);

// Detect all closed profiles using OCCT's BOPAlgo_BuilderFace (exact geometry).
// Requires a SketchPlane for 2D→3D conversion.
struct SketchPlane; // forward decl
std::vector<ClosedProfile> detectClosedProfilesOCCT(const Sketch& sketch, const SketchPlane& plane);

// Router: picks backend based on the active preference.
// Falls back to Custom if plane is not provided and OCCT is selected.
std::vector<ClosedProfile> detectClosedProfiles(const Sketch& sketch);
std::vector<ClosedProfile> detectClosedProfiles(const Sketch& sketch, const SketchPlane& plane);

} // namespace shitcad
