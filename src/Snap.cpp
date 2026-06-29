#include "Snap.h"
#include "Intersect.h"
#include "HitTest.h"
#include "Constants.h"
#include <cmath>
#include <limits>
#include <algorithm>

namespace shitcad {

SnapResult SnapEngine::snap(Point2D cursorWorld, float pixelsPerUnit,
                            const Sketch& sketch, float gridStep) const {
    double worldTolerance = snapTolerancePx / pixelsPerUnit;
    double bestDist = std::numeric_limits<double>::max();
    SnapResult result;
    result.position = cursorWorld;

    // Priority 1: Point snap
    if (pointSnapEnabled) {
        for (const auto& pt : sketch.points) {
            double dist = distance(cursorWorld, {pt.x, pt.y});
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

    // Priority 2: Intersection snap (line-line, circle-line, circle-circle)
    if (intersectionSnapEnabled) {
        // Line-line intersections
        auto lineLineIx = computeAllLineLineIntersections(sketch);
        for (const auto& ix : lineLineIx) {
            double dist = distance(cursorWorld, ix.point);
            if (dist < worldTolerance && dist < bestDist) {
                bestDist = dist;
                result.type = SnapType::Intersection;
                result.position = ix.point;
                result.pointID = NullID;
            }
        }

        // Circle-line intersections
        for (const auto& circle : sketch.circles) {
            Point2D center = sketch.getPointPos(circle.centerPt);
            for (const auto& line : sketch.lines) {
                Point2D a = sketch.getPointPos(line.startPt);
                Point2D b = sketch.getPointPos(line.endPt);
                auto hits = circleLineIntersection(center, f(circle.radius), a, b);
                for (const auto& h : hits) {
                    double dist = distance(cursorWorld, h.point);
                    if (dist < worldTolerance && dist < bestDist) {
                        bestDist = dist;
                        result.type = SnapType::Intersection;
                        result.position = h.point;
                        result.pointID = NullID;
                    }
                }
            }
        }

        // Circle-circle intersections
        for (int ci = 0; ci < (int)sketch.circles.size(); ci++) {
            Point2D c1 = sketch.getPointPos(sketch.circles[ci].centerPt);
            double r1 = sketch.circles[ci].radius;
            for (int cj = ci + 1; cj < (int)sketch.circles.size(); cj++) {
                Point2D c2 = sketch.getPointPos(sketch.circles[cj].centerPt);
                double r2 = sketch.circles[cj].radius;
                auto hits = circleCircleIntersection(c1, f(r1), c2, f(r2));
                for (const auto& h : hits) {
                    double dist = distance(cursorWorld, h.point);
                    if (dist < worldTolerance && dist < bestDist) {
                        bestDist = dist;
                        result.type = SnapType::Intersection;
                        result.position = h.point;
                        result.pointID = NullID;
                    }
                }
            }
        }

        if (result.type == SnapType::Intersection)
            return result;
    }

    // Priority 3: Midpoint snap
    if (midpointSnapEnabled) {
        for (const auto& line : sketch.lines) {
            Point2D a = sketch.getPointPos(line.startPt);
            Point2D b = sketch.getPointPos(line.endPt);
            Point2D mid = midpoint(a, b);
            double dist = distance(cursorWorld, mid);
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

    // Priority 3.5: Quadrant snap — N/S/E/W extremes of circles and arcs
    {
        static constexpr double kPiD = 3.14159265358979323846;
        auto normA = [](double a) {
            a = std::fmod(a, 2.0 * 3.14159265358979323846);
            if (a < 0.0) a += 2.0 * 3.14159265358979323846;
            return a;
        };

        for (const auto& circle : sketch.circles) {
            Point2D center = sketch.getPointPos(circle.centerPt);
            double r = circle.radius;
            const Point2D quads[4] = {
                {center.x + r, center.y},  // East  0°
                {center.x, center.y + r},  // North 90°
                {center.x - r, center.y},  // West  180°
                {center.x, center.y - r},  // South 270°
            };
            for (const auto& qp : quads) {
                double dist = distance(cursorWorld, qp);
                if (dist < worldTolerance && dist < bestDist) {
                    bestDist = dist;
                    result.type = SnapType::Quadrant;
                    result.position = qp;
                    result.pointID = NullID;
                    result.curveID = circle.id;
                }
            }
        }

        for (const auto& arc : sketch.arcs) {
            Point2D center = sketch.getPointPos(arc.centerPt);
            Point2D sp = sketch.getPointPos(arc.startPt);
            double r = distance(center, sp);
            double nSA = normA(arc.startAngle);
            double nEA = normA(arc.endAngle);
            double sweep = nEA - nSA;
            if (sweep <= 0.0) sweep += 2.0 * kPiD;

            const double quadAngles[4] = {0.0, kPiD * 0.5, kPiD, kPiD * 1.5};
            for (double qa : quadAngles) {
                double nQA = normA(qa);
                double toQ = nQA - nSA;
                if (toQ < 0.0) toQ += 2.0 * kPiD;
                if (toQ >= sweep) continue; // outside arc sweep

                Point2D qp = {center.x + r * std::cos(qa), center.y + r * std::sin(qa)};
                double dist = distance(cursorWorld, qp);
                if (dist < worldTolerance && dist < bestDist) {
                    bestDist = dist;
                    result.type = SnapType::Quadrant;
                    result.position = qp;
                    result.pointID = NullID;
                    result.curveID = arc.id;
                }
            }
        }

        if (result.type == SnapType::Quadrant)
            return result;
    }

    // Priority 4: Nearest point on circle/arc edge
    {
        double curveTolerance = curveSnapTolerancePx / pixelsPerUnit;
        for (const auto& circle : sketch.circles) {
            Point2D center = sketch.getPointPos(circle.centerPt);
            double distToEdge = std::fabs(distance(cursorWorld, center) - circle.radius);
            if (distToEdge < curveTolerance && distToEdge < bestDist) {
                bestDist = distToEdge;
                double angle = std::atan2(cursorWorld.y - center.y, cursorWorld.x - center.x);
                result.type = SnapType::NearestOnCurve;
                result.position = {center.x + circle.radius * std::cos(angle),
                                   center.y + circle.radius * std::sin(angle)};
                result.pointID = NullID;
                result.curveID = circle.id;
            }
        }
        for (const auto& arc : sketch.arcs) {
            Point2D center = sketch.getPointPos(arc.centerPt);
            Point2D sp = sketch.getPointPos(arc.startPt);
            double radius = distance(center, sp);
            double distToEdge = pointToArcDist(cursorWorld, center, radius, arc.startAngle, arc.endAngle);
            if (distToEdge < curveTolerance && distToEdge < bestDist) {
                bestDist = distToEdge;
                double angle = std::atan2(cursorWorld.y - center.y, cursorWorld.x - center.x);
                result.type = SnapType::NearestOnCurve;
                result.position = {center.x + radius * std::cos(angle),
                                   center.y + radius * std::sin(angle)};
                result.pointID = NullID;
                result.curveID = arc.id;
            }
        }
        // Nearest point on ellipse edge
        for (const auto& ellipse : sketch.ellipses) {
            Point2D center = sketch.getPointPos(ellipse.centerPt);
            auto pts = sampleEllipse(center, ellipse.semiMajor, ellipse.semiMinor, ellipse.rotation, 64);
            for (int i = 0; i + 1 < (int)pts.size(); i++) {
                double segDist = pointToSegmentDist(cursorWorld, pts[i], pts[i + 1]);
                if (segDist < curveTolerance && segDist < bestDist) {
                    bestDist = segDist;
                    // Project cursor onto segment for snap position
                    double dx = pts[i+1].x - pts[i].x, dy = pts[i+1].y - pts[i].y;
                    double len2 = dx*dx + dy*dy;
                    double t = len2 > 1e-10 ? std::clamp(((cursorWorld.x - pts[i].x)*dx + (cursorWorld.y - pts[i].y)*dy) / len2, 0.0, 1.0) : 0.0;
                    result.type = SnapType::NearestOnCurve;
                    result.position = {pts[i].x + t * dx, pts[i].y + t * dy};
                    result.pointID = NullID;
                    result.curveID = ellipse.id;
                }
            }
        }
        // Nearest point on ellipse arc edge
        for (const auto& ea : sketch.ellipseArcs) {
            Point2D center = sketch.getPointPos(ea.centerPt);
            auto pts = sampleEllipseArc(center, ea.semiMajor, ea.semiMinor, ea.rotation, ea.startAngle, ea.endAngle, 32);
            for (int i = 0; i + 1 < (int)pts.size(); i++) {
                double segDist = pointToSegmentDist(cursorWorld, pts[i], pts[i + 1]);
                if (segDist < curveTolerance && segDist < bestDist) {
                    bestDist = segDist;
                    double dx = pts[i+1].x - pts[i].x, dy = pts[i+1].y - pts[i].y;
                    double len2 = dx*dx + dy*dy;
                    double t = len2 > 1e-10 ? std::clamp(((cursorWorld.x - pts[i].x)*dx + (cursorWorld.y - pts[i].y)*dy) / len2, 0.0, 1.0) : 0.0;
                    result.type = SnapType::NearestOnCurve;
                    result.position = {pts[i].x + t * dx, pts[i].y + t * dy};
                    result.pointID = NullID;
                    result.curveID = ea.id;
                }
            }
        }
        // Nearest point on spline edge
        for (const auto& sp : sketch.splines) {
            if (sp.controlPtIDs.size() < 2) continue;
            auto pts = sampleSpline(sp, sketch, 64);
            for (int i = 0; i + 1 < (int)pts.size(); i++) {
                double segDist = pointToSegmentDist(cursorWorld, pts[i], pts[i + 1]);
                if (segDist < curveTolerance && segDist < bestDist) {
                    bestDist = segDist;
                    double dx = pts[i+1].x - pts[i].x, dy = pts[i+1].y - pts[i].y;
                    double len2 = dx*dx + dy*dy;
                    double t = len2 > 1e-10 ? std::clamp(((cursorWorld.x - pts[i].x)*dx + (cursorWorld.y - pts[i].y)*dy) / len2, 0.0, 1.0) : 0.0;
                    result.type = SnapType::NearestOnCurve;
                    result.position = {pts[i].x + t * dx, pts[i].y + t * dy};
                    result.pointID = NullID;
                    result.curveID = sp.id;
                }
            }
        }
        if (result.type == SnapType::NearestOnCurve)
            return result;
    }

    // Priority 5: Grid snap
    if (gridSnapEnabled && gridStep > 0.0f) {
        double snappedX = std::round(cursorWorld.x / gridStep) * gridStep;
        double snappedY = std::round(cursorWorld.y / gridStep) * gridStep;
        Point2D snapped = {snappedX, snappedY};
        double dist = distance(cursorWorld, snapped);
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
