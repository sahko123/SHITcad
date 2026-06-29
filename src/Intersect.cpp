#include "Intersect.h"
#include <cmath>

namespace shitcad {

static constexpr float kEps = 1e-5f;

bool segSegIntersection(Point2D a1, Point2D a2, Point2D b1, Point2D b2,
                        SegIntersectResult& out) {
    double d1x = a2.x - a1.x, d1y = a2.y - a1.y;
    double d2x = b2.x - b1.x, d2y = b2.y - b1.y;

    double cross = d1x * d2y - d1y * d2x;
    if (std::fabs(cross) < kEps) return false; // parallel

    double dx = b1.x - a1.x, dy = b1.y - a1.y;
    double t = (dx * d2y - dy * d2x) / cross;
    double s = (dx * d1y - dy * d1x) / cross;

    if (t < -kEps || t > 1.0 + kEps || s < -kEps || s > 1.0 + kEps)
        return false;

    // Clamp to [0,1]
    t = std::fmax(0.0, std::fmin(1.0, t));
    s = std::fmax(0.0, std::fmin(1.0, s));

    out.point = { a1.x + t * d1x, a1.y + t * d1y };
    out.tA = static_cast<float>(t);
    out.tB = static_cast<float>(s);
    return true;
}

std::vector<IntersectionInfo> computeAllLineLineIntersections(const Sketch& sketch) {
    std::vector<IntersectionInfo> results;
    int n = (int)sketch.lines.size();

    for (int i = 0; i < n; i++) {

        Point2D a1 = sketch.getPointPos(sketch.lines[i].startPt);
        Point2D a2 = sketch.getPointPos(sketch.lines[i].endPt);

        for (int j = i + 1; j < n; j++) {

            // Skip lines that share an endpoint (they meet at a real vertex)
            if (sketch.lines[i].startPt == sketch.lines[j].startPt ||
                sketch.lines[i].startPt == sketch.lines[j].endPt ||
                sketch.lines[i].endPt == sketch.lines[j].startPt ||
                sketch.lines[i].endPt == sketch.lines[j].endPt)
            {
                // Still check for T-junctions: one endpoint on the other's interior
                Point2D b1 = sketch.getPointPos(sketch.lines[j].startPt);
                Point2D b2 = sketch.getPointPos(sketch.lines[j].endPt);

                SegIntersectResult r;
                if (segSegIntersection(a1, a2, b1, b2, r)) {
                    bool tAendpoint = (r.tA < kEps || r.tA > 1.0f - kEps);
                    bool tBendpoint = (r.tB < kEps || r.tB > 1.0f - kEps);
                    // T-junction: one is at endpoint, other is in interior
                    if (tAendpoint != tBendpoint) {
                        results.push_back({r.point, i, j, r.tA, r.tB});
                    }
                    // If both endpoints → shared vertex, skip (already handled)
                }
                continue;
            }

            Point2D b1 = sketch.getPointPos(sketch.lines[j].startPt);
            Point2D b2 = sketch.getPointPos(sketch.lines[j].endPt);

            SegIntersectResult r;
            if (segSegIntersection(a1, a2, b1, b2, r)) {
                // Skip if both are at endpoints (shouldn't happen since we check shared endpoints above)
                bool tAendpoint = (r.tA < kEps || r.tA > 1.0f - kEps);
                bool tBendpoint = (r.tB < kEps || r.tB > 1.0f - kEps);
                if (tAendpoint && tBendpoint) continue;

                results.push_back({r.point, i, j, r.tA, r.tB});
            }
        }
    }

    return results;
}

std::vector<CircleLineHit> circleLineIntersection(
    Point2D center, float radius, Point2D a, Point2D b) {
    std::vector<CircleLineHit> results;

    // Line parametric: P(t) = a + t*(b-a), t in [0,1]
    double dx = b.x - a.x, dy = b.y - a.y;
    double fx = a.x - center.x, fy = a.y - center.y;
    double r = radius;

    double A = dx * dx + dy * dy;
    if (A < kEps * kEps) return results; // degenerate segment

    double B = 2.0 * (fx * dx + fy * dy);
    double C = fx * fx + fy * fy - r * r;

    double disc = B * B - 4.0 * A * C;
    if (disc < 0) return results;

    double sqrtDisc = std::sqrt(disc);
    double t1 = (-B - sqrtDisc) / (2.0 * A);
    double t2 = (-B + sqrtDisc) / (2.0 * A);

    for (double t : {t1, t2}) {
        if (t < -kEps || t > 1.0 + kEps) continue;
        t = std::fmax(0.0, std::fmin(1.0, t));
        Point2D p = { a.x + t * dx, a.y + t * dy };
        float angle = static_cast<float>(std::atan2(p.y - center.y, p.x - center.x));
        results.push_back({p, static_cast<float>(t), angle});
    }

    // Deduplicate if t1 ≈ t2 (tangent case)
    if (results.size() == 2) {
        float dt = std::fabs(results[0].tLine - results[1].tLine);
        if (dt < kEps) results.pop_back();
    }

    return results;
}

std::vector<CircleCircleHit> circleCircleIntersection(
    Point2D c1, float r1, Point2D c2, float r2) {
    std::vector<CircleCircleHit> results;

    double dx = c2.x - c1.x, dy = c2.y - c1.y;
    double d = std::sqrt(dx * dx + dy * dy);
    double ra1 = r1, ra2 = r2;

    if (d < kEps) return results; // concentric
    if (d > ra1 + ra2 + kEps) return results; // too far
    if (d < std::fabs(ra1 - ra2) - kEps) return results; // one inside other

    double a = (ra1 * ra1 - ra2 * ra2 + d * d) / (2.0 * d);
    double h2 = ra1 * ra1 - a * a;
    if (h2 < 0) h2 = 0; // clamp floating point
    double h = std::sqrt(h2);

    // Midpoint along line between centers
    double mx = c1.x + a * dx / d;
    double my = c1.y + a * dy / d;

    // Perpendicular offset
    double px = -dy / d * h;
    double py = dx / d * h;

    Point2D p1 = { mx + px, my + py };
    Point2D p2 = { mx - px, my - py };

    float angle1_1 = static_cast<float>(std::atan2(p1.y - c1.y, p1.x - c1.x));
    float angle2_1 = static_cast<float>(std::atan2(p1.y - c2.y, p1.x - c2.x));
    results.push_back({p1, angle1_1, angle2_1});

    if (h > kEps) { // two distinct points (not tangent)
        float angle1_2 = static_cast<float>(std::atan2(p2.y - c1.y, p2.x - c1.x));
        float angle2_2 = static_cast<float>(std::atan2(p2.y - c2.y, p2.x - c2.x));
        results.push_back({p2, angle1_2, angle2_2});
    }

    return results;
}

} // namespace shitcad
