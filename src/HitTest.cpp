#include "HitTest.h"
#include "Constants.h"
#include <cmath>
#include <algorithm>
#include <limits>

namespace shitcad {

double pointToSegmentDist(Point2D p, Point2D a, Point2D b) {
    double dx = b.x - a.x;
    double dy = b.y - a.y;
    double lenSq = dx * dx + dy * dy;

    if (lenSq < 1e-10) {
        // Degenerate segment (zero length)
        return distance(p, a);
    }

    // Project p onto the line defined by a-b, clamped to [0,1]
    double t = ((p.x - a.x) * dx + (p.y - a.y) * dy) / lenSq;
    t = std::clamp(t, 0.0, 1.0);

    Point2D closest = {a.x + t * dx, a.y + t * dy};
    return distance(p, closest);
}

double pointToCircleDist(Point2D p, Point2D center, double radius) {
    double distToCenter = distance(p, center);
    return std::fabs(distToCenter - radius);
}

double pointToArcDist(Point2D p, Point2D center, double radius, double startAngle, double endAngle) {
    double angle = std::atan2(p.y - center.y, p.x - center.x);

    // Normalize sweep to CCW
    double sweep = endAngle - startAngle;
    if (sweep <= 0) sweep += kTwoPi;

    // Normalize angle relative to start
    double rel = angle - startAngle;
    double twoPi = kTwoPi;
    rel = std::fmod(rel, twoPi);
    if (rel < 0) rel += twoPi;

    if (rel <= sweep) {
        // Point's angle is within the arc range — distance to the arc curve
        return std::fabs(distance(p, center) - radius);
    } else {
        // Outside arc range — distance to nearest endpoint
        Point2D startPt = {center.x + radius * std::cos(startAngle),
                           center.y + radius * std::sin(startAngle)};
        Point2D endPt = {center.x + radius * std::cos(endAngle),
                         center.y + radius * std::sin(endAngle)};
        return std::min(distance(p, startPt), distance(p, endPt));
    }
}

double pointToEllipseDist(Point2D p, Point2D center, double semiMajor, double semiMinor, double rotation) {
    // Sample the ellipse and find closest point
    double bestDist = std::numeric_limits<double>::max();
    int segments = 64;
    double cosR = std::cos(rotation), sinR = std::sin(rotation);
    for (int i = 0; i < segments; i++) {
        double a0 = kTwoPi * i / segments;
        double a1 = kTwoPi * (i + 1) / segments;

        double ex0 = semiMajor * std::cos(a0), ey0 = semiMinor * std::sin(a0);
        double ex1 = semiMajor * std::cos(a1), ey1 = semiMinor * std::sin(a1);

        Point2D p0 = {center.x + ex0 * cosR - ey0 * sinR, center.y + ex0 * sinR + ey0 * cosR};
        Point2D p1 = {center.x + ex1 * cosR - ey1 * sinR, center.y + ex1 * sinR + ey1 * cosR};

        double dist = pointToSegmentDist(p, p0, p1);
        if (dist < bestDist) bestDist = dist;
    }
    return bestDist;
}

double pointToSplineDist(Point2D p, const std::vector<Point2D>& samplePts) {
    double bestDist = std::numeric_limits<double>::max();
    for (int i = 0; i + 1 < (int)samplePts.size(); i++) {
        double dist = pointToSegmentDist(p, samplePts[i], samplePts[i + 1]);
        if (dist < bestDist) bestDist = dist;
    }
    return bestDist;
}

HitResult hitTest(Point2D cursorWorld, float pixelsPerUnit,
                  const Sketch& sketch, float tolerancePx) {
    HitResult best;
    best.distance = std::numeric_limits<float>::max();

    // Priority 1: Points (smallest entities, easiest to miss without priority)
    for (const auto& pt : sketch.points) {
        double dist = distance(cursorWorld, {pt.x, pt.y});
        double screenDist = dist * pixelsPerUnit;
        if (screenDist < tolerancePx && screenDist < best.distance) {
            best.type = HitType::Point;
            best.entityID = pt.id;
            best.distance = f(screenDist);
        }
    }

    // If we hit a point, return it (highest priority)
    if (best.type == HitType::Point)
        return best;

    // Priority 2: Lines
    for (const auto& line : sketch.lines) {
        Point2D a = sketch.getPointPos(line.startPt);
        Point2D b = sketch.getPointPos(line.endPt);
        double dist = pointToSegmentDist(cursorWorld, a, b);
        double screenDist = dist * pixelsPerUnit;
        if (screenDist < tolerancePx && screenDist < best.distance) {
            best.type = HitType::Line;
            best.entityID = line.id;
            best.distance = f(screenDist);
        }
    }

    if (best.type == HitType::Line)
        return best;

    // Priority 3: Circles and Arcs
    for (const auto& circle : sketch.circles) {
        Point2D center = sketch.getPointPos(circle.centerPt);
        double dist = pointToCircleDist(cursorWorld, center, circle.radius);
        double screenDist = dist * pixelsPerUnit;
        if (screenDist < tolerancePx && screenDist < best.distance) {
            best.type = HitType::Circle;
            best.entityID = circle.id;
            best.distance = f(screenDist);
        }
    }

    for (const auto& arc : sketch.arcs) {
        Point2D center = sketch.getPointPos(arc.centerPt);
        Point2D startP = sketch.getPointPos(arc.startPt);
        double radius = distance(center, startP);
        double dist = pointToArcDist(cursorWorld, center, radius, arc.startAngle, arc.endAngle);
        double screenDist = dist * pixelsPerUnit;
        if (screenDist < tolerancePx && screenDist < best.distance) {
            best.type = HitType::Arc;
            best.entityID = arc.id;
            best.distance = f(screenDist);
        }
    }

    for (const auto& ellipse : sketch.ellipses) {
        Point2D center = sketch.getPointPos(ellipse.centerPt);
        double dist = pointToEllipseDist(cursorWorld, center, ellipse.semiMajor, ellipse.semiMinor, ellipse.rotation);
        double screenDist = dist * pixelsPerUnit;
        if (screenDist < tolerancePx && screenDist < best.distance) {
            best.type = HitType::Ellipse;
            best.entityID = ellipse.id;
            best.distance = f(screenDist);
        }
    }

    for (const auto& ea : sketch.ellipseArcs) {
        Point2D center = sketch.getPointPos(ea.centerPt);
        // Sample the ellipse arc and find closest segment
        double cosR = std::cos(ea.rotation), sinR = std::sin(ea.rotation);
        double sweep = ea.endAngle - ea.startAngle;
        if (sweep <= 0) sweep += kTwoPi;
        int segments = std::max(8, (int)(std::fabs(sweep) / (kTwoPi) * 64));
        double bestSegDist = std::numeric_limits<double>::max();
        for (int i = 0; i < segments; i++) {
            double a0 = ea.startAngle + sweep * i / segments;
            double a1 = ea.startAngle + sweep * (i + 1) / segments;
            double ex0 = ea.semiMajor * std::cos(a0), ey0 = ea.semiMinor * std::sin(a0);
            double ex1 = ea.semiMajor * std::cos(a1), ey1 = ea.semiMinor * std::sin(a1);
            Point2D p0 = {center.x + ex0*cosR - ey0*sinR, center.y + ex0*sinR + ey0*cosR};
            Point2D p1 = {center.x + ex1*cosR - ey1*sinR, center.y + ex1*sinR + ey1*cosR};
            double d = pointToSegmentDist(cursorWorld, p0, p1);
            if (d < bestSegDist) bestSegDist = d;
        }
        double screenDist = bestSegDist * pixelsPerUnit;
        if (screenDist < tolerancePx && screenDist < best.distance) {
            best.type = HitType::EllipseArc;
            best.entityID = ea.id;
            best.distance = f(screenDist);
        }
    }

    for (const auto& sp : sketch.splines) {
        if (sp.controlPtIDs.size() < 2) continue;
        auto pts = sampleSpline(sp, sketch, 64);
        double dist = pointToSplineDist(cursorWorld, pts);
        double screenDist = dist * pixelsPerUnit;
        if (screenDist < tolerancePx && screenDist < best.distance) {
            best.type = HitType::Spline;
            best.entityID = sp.id;
            best.distance = f(screenDist);
        }
    }

    return best;
}

// --- Containment tests ---

bool pointInRect(Point2D p, Point2D mn, Point2D mx) {
    return p.x >= mn.x && p.x <= mx.x && p.y >= mn.y && p.y <= mx.y;
}

bool segmentInRect(Point2D a, Point2D b, Point2D mn, Point2D mx) {
    return pointInRect(a, mn, mx) && pointInRect(b, mn, mx);
}

bool circleInRect(Point2D center, double radius, Point2D mn, Point2D mx) {
    return (center.x - radius) >= mn.x && (center.x + radius) <= mx.x &&
           (center.y - radius) >= mn.y && (center.y + radius) <= mx.y;
}

bool pointInPolygon(Point2D p, const std::vector<Point2D>& poly) {
    int n = (int)poly.size();
    if (n < 3) return false;
    bool inside = false;
    for (int i = 0, j = n - 1; i < n; j = i++) {
        if (((poly[i].y > p.y) != (poly[j].y > p.y)) &&
            (p.x < (poly[j].x - poly[i].x) * (p.y - poly[i].y) / (poly[j].y - poly[i].y) + poly[i].x)) {
            inside = !inside;
        }
    }
    return inside;
}

bool segmentInPolygon(Point2D a, Point2D b, const std::vector<Point2D>& poly) {
    return pointInPolygon(a, poly) && pointInPolygon(b, poly);
}

bool circleInPolygon(Point2D center, double radius, const std::vector<Point2D>& poly) {
    if (!pointInPolygon(center, poly)) return false;
    // Sample 16 points on circumference
    for (int i = 0; i < 16; i++) {
        double angle = kTwoPi * i / 16.0;
        Point2D p = {center.x + radius * std::cos(angle),
                     center.y + radius * std::sin(angle)};
        if (!pointInPolygon(p, poly)) return false;
    }
    return true;
}

} // namespace shitcad
