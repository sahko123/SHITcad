#include "ProfileDetector.h"
#include "SketchPlane.h"
#include "Intersect.h"
#include <unordered_map>
#include <unordered_set>

namespace shitcad {

// ---- Subdivision structures ----

static constexpr float kMergeEps = 1e-4f;

struct SubVertex {
    Point2D pos;
    EntityID origPointID; // NullID for virtual intersection points
};

struct SubEdge {
    int fromVtx;
    int toVtx;
    EntityID origLineID;   // NullID for arc edges
    EntityID origCircleID; // NullID for line edges
    EntityID origArcID = NullID; // NullID for non-arc-entity edges
    EntityID origEllipseID = NullID;
    EntityID origEllipseArcID = NullID;
    EntityID origSplineID = NullID;
    Point2D arcCenter;     // only meaningful for arc edges
    float arcRadius = 0;
    // Arc angles at the from/to vertices (for arc edges)
    float arcFromAngle = 0;
    float arcToAngle = 0;
    bool isArc() const { return origCircleID != NullID || origArcID != NullID; }
    bool isCurve() const { return isArc() || origEllipseID != NullID || origEllipseArcID != NullID || origSplineID != NullID; }
};

struct SubHalfEdge {
    int edgeIdx;
    int fromVtx;
    int toVtx;
    float angle; // departure tangent angle at fromVtx
};

// ---- Helper functions ----

static int findOrAddVertex(std::vector<SubVertex>& verts, Point2D pos, EntityID origID) {
    for (int i = 0; i < (int)verts.size(); i++) {
        float dx = verts[i].pos.x - pos.x;
        float dy = verts[i].pos.y - pos.y;
        if (dx * dx + dy * dy < kMergeEps * kMergeEps) {
            if (origID != NullID && verts[i].origPointID == NullID)
                verts[i].origPointID = origID;
            return i;
        }
    }
    verts.push_back({pos, origID});
    return (int)verts.size() - 1;
}

// Compute tangent departure angle for a sub-edge at its fromVtx
static float computeDepartureAngle(const SubEdge& edge, const std::vector<SubVertex>& verts, bool forward) {
    if (edge.isArc()) {
        // Arc edge: sample a point slightly along the arc to get the departure direction.
        // Using the exact tangent (perpendicular to radius) fails when two arcs from the
        // same circle meet at a vertex — both would have identical tangent angles.
        // Sampling slightly along the arc naturally distinguishes them.
        float fromA = edge.arcFromAngle;
        float toA = edge.arcToAngle;

        // Clamp sample offset to 25% of the arc span so we never overshoot
        // past the far endpoint on very small arcs.
        float span = std::fabs(toA - fromA);
        float sampleOff = std::min(0.01f, span * 0.25f);

        float sampleAngle;
        if (forward) {
            // Departing from fromVtx along the arc toward toVtx
            sampleAngle = fromA + sampleOff;
        } else {
            // Departing from toVtx backward along the arc toward fromVtx
            sampleAngle = toA - sampleOff;
        }

        // Sample point on the arc at sampleAngle
        Point2D samplePt = {
            edge.arcCenter.x + edge.arcRadius * std::cos(sampleAngle),
            edge.arcCenter.y + edge.arcRadius * std::sin(sampleAngle)
        };

        int vtx = forward ? edge.fromVtx : edge.toVtx;
        Point2D p = verts[vtx].pos;
        return std::atan2(samplePt.y - p.y, samplePt.x - p.x);
    } else {
        // Line edge: angle from fromVtx to toVtx
        int fv = forward ? edge.fromVtx : edge.toVtx;
        int tv = forward ? edge.toVtx : edge.fromVtx;
        Point2D a = verts[fv].pos;
        Point2D b = verts[tv].pos;
        return std::atan2(b.y - a.y, b.x - a.x);
    }
}

// ---- Build subdivided edge graph ----

static void buildSubdivision(const Sketch& sketch,
                              std::vector<SubVertex>& subVerts,
                              std::vector<SubEdge>& subEdges,
                              std::unordered_set<EntityID>& splitCircleIDs) {
    // Add all sketch points as vertices
    for (const auto& pt : sketch.points) {

        findOrAddVertex(subVerts, {pt.x, pt.y}, pt.id);
    }

    // ---- Line-line intersections ----
    auto lineLineIx = computeAllLineLineIntersections(sketch);

    int numLines = (int)sketch.lines.size();
    int numCircles = (int)sketch.circles.size();
    int numArcs = (int)sketch.arcs.size();

    // Per-line intersection list: (parametric t, vertex index)
    std::vector<std::vector<std::pair<float, int>>> lineIxPts(numLines);
    // Per-circle intersection list: (angle, vertex index)
    std::vector<std::vector<std::pair<float, int>>> circleIxPts(numCircles);
    // Per-arc intersection list: (angle, vertex index)
    std::vector<std::vector<std::pair<float, int>>> arcIxPts(numArcs);

    // Per-curve intersection lists: (polyline parameter = segIdx + t, vertex index)
    int numEllipses = (int)sketch.ellipses.size();
    int numEllipseArcs = (int)sketch.ellipseArcs.size();
    int numSplines = (int)sketch.splines.size();
    std::vector<std::vector<std::pair<float, int>>> ellipseIxPts(numEllipses);
    std::vector<std::vector<std::pair<float, int>>> ellipseArcIxPts(numEllipseArcs);
    std::vector<std::vector<std::pair<float, int>>> splineIxPts(numSplines);

    // Precompute sample points for all curves (used for intersection detection and sub-edge creation)
    std::vector<std::vector<Point2D>> ellipseSamples(numEllipses);
    for (int ei = 0; ei < numEllipses; ei++) {
        Point2D center = sketch.getPointPos(sketch.ellipses[ei].centerPt);
        ellipseSamples[ei] = sampleEllipse(center, sketch.ellipses[ei].semiMajor,
                                            sketch.ellipses[ei].semiMinor, sketch.ellipses[ei].rotation, 64);
    }
    std::vector<std::vector<Point2D>> ellipseArcSamples(numEllipseArcs);
    for (int ei = 0; ei < numEllipseArcs; ei++) {
        Point2D center = sketch.getPointPos(sketch.ellipseArcs[ei].centerPt);
        ellipseArcSamples[ei] = sampleEllipseArc(center, sketch.ellipseArcs[ei].semiMajor,
                                                   sketch.ellipseArcs[ei].semiMinor, sketch.ellipseArcs[ei].rotation,
                                                   sketch.ellipseArcs[ei].startAngle, sketch.ellipseArcs[ei].endAngle, 32);
    }
    std::vector<std::vector<Point2D>> splineSamples(numSplines);
    for (int si = 0; si < numSplines; si++) {
        if (sketch.splines[si].controlPtIDs.size() < 2) continue;
        splineSamples[si] = sampleSpline(sketch.splines[si], sketch, 64);
    }

    for (const auto& ix : lineLineIx) {
        int vtxIdx = findOrAddVertex(subVerts, ix.point, NullID);
        lineIxPts[ix.lineIdxA].push_back({ix.tA, vtxIdx});
        lineIxPts[ix.lineIdxB].push_back({ix.tB, vtxIdx});
    }

    // ---- Circle-line intersections ----
    for (int ci = 0; ci < numCircles; ci++) {
        const auto& circle = sketch.circles[ci];

        Point2D center = sketch.getPointPos(circle.centerPt);

        for (int li = 0; li < numLines; li++) {

            Point2D a = sketch.getPointPos(sketch.lines[li].startPt);
            Point2D b = sketch.getPointPos(sketch.lines[li].endPt);

            auto hits = circleLineIntersection(center, circle.radius, a, b);
            for (const auto& h : hits) {
                // Skip if at line endpoint that's already on the circle
                bool atLineEnd = (h.tLine < kMergeEps || h.tLine > 1.0f - kMergeEps);
                if (atLineEnd) {
                    // Check if this endpoint is already a shared point
                    // (i.e., coincident with an existing vertex on both the line and circle)
                    // Still add to circle's intersection list even if at line endpoint
                }

                int vtxIdx = findOrAddVertex(subVerts, h.point, NullID);
                if (!atLineEnd) {
                    lineIxPts[li].push_back({h.tLine, vtxIdx});
                }
                circleIxPts[ci].push_back({h.angle, vtxIdx});
            }
        }
    }

    // ---- Circle-circle intersections ----
    for (int ci = 0; ci < numCircles; ci++) {
        Point2D c1 = sketch.getPointPos(sketch.circles[ci].centerPt);
        float r1 = sketch.circles[ci].radius;

        for (int cj = ci + 1; cj < numCircles; cj++) {

            Point2D c2 = sketch.getPointPos(sketch.circles[cj].centerPt);
            float r2 = sketch.circles[cj].radius;

            auto hits = circleCircleIntersection(c1, r1, c2, r2);
            for (const auto& h : hits) {
                int vtxIdx = findOrAddVertex(subVerts, h.point, NullID);
                circleIxPts[ci].push_back({h.angle1, vtxIdx});
                circleIxPts[cj].push_back({h.angle2, vtxIdx});
            }
        }
    }

    // ---- Arc-line intersections ----
    auto angleInArcRange = [](float angle, float startA, float endA) -> bool {
        float sweep = endA - startA;
        if (sweep <= 0) sweep += 2.0f * kPi;
        float rel = angle - startA;
        rel = std::fmod(rel, 2.0f * kPi);
        if (rel < 0) rel += 2.0f * kPi;
        return rel <= sweep + 1e-5f;
    };

    for (int ai = 0; ai < numArcs; ai++) {
        const auto& arc = sketch.arcs[ai];

        Point2D center = sketch.getPointPos(arc.centerPt);
        Point2D sp = sketch.getPointPos(arc.startPt);
        float radius = distance(center, sp);

        for (int li = 0; li < numLines; li++) {

            Point2D a = sketch.getPointPos(sketch.lines[li].startPt);
            Point2D b = sketch.getPointPos(sketch.lines[li].endPt);
            auto hits = circleLineIntersection(center, radius, a, b);
            for (const auto& h : hits) {
                if (!angleInArcRange(h.angle, arc.startAngle, arc.endAngle)) continue;
                bool atLineEnd = (h.tLine < kMergeEps || h.tLine > 1.0f - kMergeEps);
                int vtxIdx = findOrAddVertex(subVerts, h.point, NullID);
                if (!atLineEnd) lineIxPts[li].push_back({h.tLine, vtxIdx});
                arcIxPts[ai].push_back({h.angle, vtxIdx});
            }
        }

        // Arc-circle intersections
        for (int ci = 0; ci < numCircles; ci++) {

            Point2D c2 = sketch.getPointPos(sketch.circles[ci].centerPt);
            float r2 = sketch.circles[ci].radius;
            auto hits = circleCircleIntersection(center, radius, c2, r2);
            for (const auto& h : hits) {
                if (!angleInArcRange(h.angle1, arc.startAngle, arc.endAngle)) continue;
                int vtxIdx = findOrAddVertex(subVerts, h.point, NullID);
                arcIxPts[ai].push_back({h.angle1, vtxIdx});
                circleIxPts[ci].push_back({h.angle2, vtxIdx});
            }
        }

        // Arc-arc intersections
        for (int aj = ai + 1; aj < numArcs; aj++) {

            const auto& arc2 = sketch.arcs[aj];
            Point2D c2 = sketch.getPointPos(arc2.centerPt);
            Point2D sp2 = sketch.getPointPos(arc2.startPt);
            float r2 = distance(c2, sp2);
            auto hits = circleCircleIntersection(center, radius, c2, r2);
            for (const auto& h : hits) {
                if (!angleInArcRange(h.angle1, arc.startAngle, arc.endAngle)) continue;
                if (!angleInArcRange(h.angle2, arc2.startAngle, arc2.endAngle)) continue;
                int vtxIdx = findOrAddVertex(subVerts, h.point, NullID);
                arcIxPts[ai].push_back({h.angle1, vtxIdx});
                arcIxPts[aj].push_back({h.angle2, vtxIdx});
            }
        }
    }

    // ---- Sampled curve intersections (ellipse, ellipseArc, spline vs all entity types) ----

    // Helper: find intersections between a sampled polyline and a line segment,
    // adding to both the curve's and line's intersection lists.
    auto findCurveLineIx = [&](const std::vector<Point2D>& samples,
                                std::vector<std::pair<float,int>>& curveIx,
                                int li) {
        Point2D a = sketch.getPointPos(sketch.lines[li].startPt);
        Point2D b = sketch.getPointPos(sketch.lines[li].endPt);
        for (int si = 0; si + 1 < (int)samples.size(); si++) {
            SegIntersectResult r;
            if (segSegIntersection(samples[si], samples[si+1], a, b, r)) {
                bool atLineEnd = (r.tB < kMergeEps || r.tB > 1.0f - kMergeEps);
                int vtxIdx = findOrAddVertex(subVerts, r.point, NullID);
                if (!atLineEnd) lineIxPts[li].push_back({r.tB, vtxIdx});
                curveIx.push_back({si + r.tA, vtxIdx});
            }
        }
    };

    // Helper: find intersections between a sampled polyline and a circle,
    // adding to both the curve's and circle's intersection lists.
    auto findCurveCircleIx = [&](const std::vector<Point2D>& samples,
                                  std::vector<std::pair<float,int>>& curveIx,
                                  int ci) {
        Point2D center = sketch.getPointPos(sketch.circles[ci].centerPt);
        float radius = sketch.circles[ci].radius;
        for (int si = 0; si + 1 < (int)samples.size(); si++) {
            auto hits = circleLineIntersection(center, radius, samples[si], samples[si+1]);
            for (const auto& h : hits) {
                int vtxIdx = findOrAddVertex(subVerts, h.point, NullID);
                curveIx.push_back({si + h.tLine, vtxIdx});
                circleIxPts[ci].push_back({h.angle, vtxIdx});
            }
        }
    };

    // Helper: find intersections between a sampled polyline and an arc,
    // adding to both the curve's and arc's intersection lists.
    auto findCurveArcIx = [&](const std::vector<Point2D>& samples,
                               std::vector<std::pair<float,int>>& curveIx,
                               int ai) {
        const auto& arc = sketch.arcs[ai];
        Point2D center = sketch.getPointPos(arc.centerPt);
        Point2D sp = sketch.getPointPos(arc.startPt);
        float radius = distance(center, sp);
        for (int si = 0; si + 1 < (int)samples.size(); si++) {
            auto hits = circleLineIntersection(center, radius, samples[si], samples[si+1]);
            for (const auto& h : hits) {
                if (!angleInArcRange(h.angle, arc.startAngle, arc.endAngle)) continue;
                int vtxIdx = findOrAddVertex(subVerts, h.point, NullID);
                curveIx.push_back({si + h.tLine, vtxIdx});
                arcIxPts[ai].push_back({h.angle, vtxIdx});
            }
        }
    };

    // Helper: find intersections between two sampled polylines
    auto findCurveCurveIx = [&](const std::vector<Point2D>& samplesA,
                                 const std::vector<Point2D>& samplesB,
                                 std::vector<std::pair<float,int>>& ixA,
                                 std::vector<std::pair<float,int>>& ixB) {
        for (int si = 0; si + 1 < (int)samplesA.size(); si++) {
            for (int sj = 0; sj + 1 < (int)samplesB.size(); sj++) {
                SegIntersectResult r;
                if (segSegIntersection(samplesA[si], samplesA[si+1], samplesB[sj], samplesB[sj+1], r)) {
                    int vtxIdx = findOrAddVertex(subVerts, r.point, NullID);
                    ixA.push_back({si + r.tA, vtxIdx});
                    ixB.push_back({sj + r.tB, vtxIdx});
                }
            }
        }
    };

    // Ellipse vs line, circle, arc
    for (int ei = 0; ei < numEllipses; ei++) {
        if (ellipseSamples[ei].empty()) continue;
        for (int li = 0; li < numLines; li++)
            findCurveLineIx(ellipseSamples[ei], ellipseIxPts[ei], li);
        for (int ci = 0; ci < numCircles; ci++)
            findCurveCircleIx(ellipseSamples[ei], ellipseIxPts[ei], ci);
        for (int ai = 0; ai < numArcs; ai++)
            findCurveArcIx(ellipseSamples[ei], ellipseIxPts[ei], ai);
    }

    // EllipseArc vs line, circle, arc
    for (int ei = 0; ei < numEllipseArcs; ei++) {
        if (ellipseArcSamples[ei].empty()) continue;
        for (int li = 0; li < numLines; li++)
            findCurveLineIx(ellipseArcSamples[ei], ellipseArcIxPts[ei], li);
        for (int ci = 0; ci < numCircles; ci++)
            findCurveCircleIx(ellipseArcSamples[ei], ellipseArcIxPts[ei], ci);
        for (int ai = 0; ai < numArcs; ai++)
            findCurveArcIx(ellipseArcSamples[ei], ellipseArcIxPts[ei], ai);
    }

    // Spline vs line, circle, arc
    for (int si = 0; si < numSplines; si++) {
        if (splineSamples[si].empty()) continue;
        for (int li = 0; li < numLines; li++)
            findCurveLineIx(splineSamples[si], splineIxPts[si], li);
        for (int ci = 0; ci < numCircles; ci++)
            findCurveCircleIx(splineSamples[si], splineIxPts[si], ci);
        for (int ai = 0; ai < numArcs; ai++)
            findCurveArcIx(splineSamples[si], splineIxPts[si], ai);
    }

    // Ellipse vs ellipse
    for (int ei = 0; ei < numEllipses; ei++) {
        if (ellipseSamples[ei].empty()) continue;
        for (int ej = ei + 1; ej < numEllipses; ej++) {
            if (ellipseSamples[ej].empty()) continue;
            findCurveCurveIx(ellipseSamples[ei], ellipseSamples[ej],
                             ellipseIxPts[ei], ellipseIxPts[ej]);
        }
    }

    // Ellipse vs ellipseArc
    for (int ei = 0; ei < numEllipses; ei++) {
        if (ellipseSamples[ei].empty()) continue;
        for (int ej = 0; ej < numEllipseArcs; ej++) {
            if (ellipseArcSamples[ej].empty()) continue;
            findCurveCurveIx(ellipseSamples[ei], ellipseArcSamples[ej],
                             ellipseIxPts[ei], ellipseArcIxPts[ej]);
        }
    }

    // Ellipse vs spline
    for (int ei = 0; ei < numEllipses; ei++) {
        if (ellipseSamples[ei].empty()) continue;
        for (int sj = 0; sj < numSplines; sj++) {
            if (splineSamples[sj].empty()) continue;
            findCurveCurveIx(ellipseSamples[ei], splineSamples[sj],
                             ellipseIxPts[ei], splineIxPts[sj]);
        }
    }

    // EllipseArc vs ellipseArc
    for (int ei = 0; ei < numEllipseArcs; ei++) {
        if (ellipseArcSamples[ei].empty()) continue;
        for (int ej = ei + 1; ej < numEllipseArcs; ej++) {
            if (ellipseArcSamples[ej].empty()) continue;
            findCurveCurveIx(ellipseArcSamples[ei], ellipseArcSamples[ej],
                             ellipseArcIxPts[ei], ellipseArcIxPts[ej]);
        }
    }

    // EllipseArc vs spline
    for (int ei = 0; ei < numEllipseArcs; ei++) {
        if (ellipseArcSamples[ei].empty()) continue;
        for (int sj = 0; sj < numSplines; sj++) {
            if (splineSamples[sj].empty()) continue;
            findCurveCurveIx(ellipseArcSamples[ei], splineSamples[sj],
                             ellipseArcIxPts[ei], splineIxPts[sj]);
        }
    }

    // Spline vs spline
    for (int si = 0; si < numSplines; si++) {
        if (splineSamples[si].empty()) continue;
        for (int sj = si + 1; sj < numSplines; sj++) {
            if (splineSamples[sj].empty()) continue;
            findCurveCurveIx(splineSamples[si], splineSamples[sj],
                             splineIxPts[si], splineIxPts[sj]);
        }
    }

    // ---- Sort intersection points ----
    for (auto& li : lineIxPts) {
        std::sort(li.begin(), li.end(), [](const auto& a, const auto& b) {
            return a.first < b.first;
        });
    }
    for (auto& ci : circleIxPts) {
        std::sort(ci.begin(), ci.end(), [](const auto& a, const auto& b) {
            return a.first < b.first;
        });
    }
    for (auto& ai : arcIxPts) {
        std::sort(ai.begin(), ai.end(), [](const auto& a, const auto& b) {
            return a.first < b.first;
        });
    }
    for (auto& ei : ellipseIxPts) {
        std::sort(ei.begin(), ei.end(), [](const auto& a, const auto& b) {
            return a.first < b.first;
        });
    }
    for (auto& ei : ellipseArcIxPts) {
        std::sort(ei.begin(), ei.end(), [](const auto& a, const auto& b) {
            return a.first < b.first;
        });
    }
    for (auto& si : splineIxPts) {
        std::sort(si.begin(), si.end(), [](const auto& a, const auto& b) {
            return a.first < b.first;
        });
    }

    // ---- Subdivide lines into sub-edges ----
    for (int i = 0; i < numLines; i++) {
        const auto& line = sketch.lines[i];

        Point2D startPos = sketch.getPointPos(line.startPt);
        Point2D endPos = sketch.getPointPos(line.endPt);

        int startVtx = findOrAddVertex(subVerts, startPos, line.startPt);
        int endVtx = findOrAddVertex(subVerts, endPos, line.endPt);

        std::vector<int> chainVerts;
        chainVerts.push_back(startVtx);
        for (const auto& [t, vtxIdx] : lineIxPts[i]) {
            if (vtxIdx != chainVerts.back())
                chainVerts.push_back(vtxIdx);
        }
        if (endVtx != chainVerts.back())
            chainVerts.push_back(endVtx);

        for (int j = 0; j + 1 < (int)chainVerts.size(); j++) {
            float dx = subVerts[chainVerts[j]].pos.x - subVerts[chainVerts[j+1]].pos.x;
            float dy = subVerts[chainVerts[j]].pos.y - subVerts[chainVerts[j+1]].pos.y;
            if (dx * dx + dy * dy < kMergeEps * kMergeEps) continue;

            SubEdge e;
            e.fromVtx = chainVerts[j];
            e.toVtx = chainVerts[j+1];
            e.origLineID = line.id;
            e.origCircleID = NullID;
            subEdges.push_back(e);
        }
    }

    // ---- Subdivide circles into arc sub-edges ----
    for (int ci = 0; ci < numCircles; ci++) {
        const auto& circle = sketch.circles[ci];

        auto& ixPts = circleIxPts[ci];

        if (ixPts.size() < 2) continue; // Need ≥2 intersection points to split

        // Deduplicate by vertex index — keep first occurrence of each vertex
        std::vector<std::pair<float, int>> deduped;
        std::unordered_set<int> seenVtx;
        for (const auto& p : ixPts) {
            if (seenVtx.insert(p.second).second)
                deduped.push_back(p);
        }
        if (deduped.size() < 2) continue;

        splitCircleIDs.insert(circle.id);
        Point2D center = sketch.getPointPos(circle.centerPt);

        // Create arc sub-edges between consecutive intersection points (sorted by angle)
        int nPts = (int)deduped.size();
        for (int j = 0; j < nPts; j++) {
            int next = (j + 1) % nPts;
            int fromVtx = deduped[j].second;
            int toVtx = deduped[next].second;
            if (fromVtx == toVtx) continue;

            float fromAngle = deduped[j].first;
            float toAngle = deduped[next].first;

            // Ensure we go CCW from fromAngle to toAngle
            if (toAngle <= fromAngle) toAngle += 2.0f * kPi;

            SubEdge e;
            e.fromVtx = fromVtx;
            e.toVtx = toVtx;
            e.origLineID = NullID;
            e.origCircleID = circle.id;
            e.arcCenter = center;
            e.arcRadius = circle.radius;
            e.arcFromAngle = fromAngle;
            e.arcToAngle = toAngle;
            subEdges.push_back(e);
        }
    }

    // Helper: build a vertex chain from sample points + sorted intersection points.
    // Intersection points are inserted at the correct positions to split the curve.
    auto buildSplitChain = [&](const std::vector<Point2D>& samples,
                                const std::vector<std::pair<float,int>>& ixPts,
                                EntityID startPtID, EntityID endPtID,
                                bool closed) -> std::vector<int> {
        // Combine sample points (param = integer index) with intersection points
        std::vector<std::pair<float, int>> allPts;
        for (int i = 0; i < (int)samples.size(); i++) {
            EntityID origID = NullID;
            if (i == 0 && startPtID != NullID) origID = startPtID;
            else if (i == (int)samples.size()-1 && endPtID != NullID) origID = endPtID;
            int vi = findOrAddVertex(subVerts, samples[i], origID);
            allPts.push_back({(float)i, vi});
        }
        for (const auto& ix : ixPts) {
            allPts.push_back(ix);
        }
        std::sort(allPts.begin(), allPts.end(), [](const auto& a, const auto& b) {
            return a.first < b.first;
        });
        // Deduplicate consecutive identical vertices
        std::vector<int> chain;
        for (const auto& [param, vi] : allPts) {
            if (chain.empty() || chain.back() != vi)
                chain.push_back(vi);
        }
        if (closed && !chain.empty() && chain.back() != chain.front())
            chain.push_back(chain.front());
        return chain;
    };

    // ---- Add ellipse entities as polyline sub-edges ----
    for (int ei = 0; ei < numEllipses; ei++) {
        const auto& ellipse = sketch.ellipses[ei];
        if (ellipseSamples[ei].empty()) continue;

        auto vtxChain = buildSplitChain(ellipseSamples[ei], ellipseIxPts[ei],
                                         NullID, NullID, true);

        for (int si = 0; si + 1 < (int)vtxChain.size(); si++) {
            if (vtxChain[si] == vtxChain[si + 1]) continue;
            SubEdge e;
            e.fromVtx = vtxChain[si];
            e.toVtx = vtxChain[si + 1];
            e.origLineID = NullID;
            e.origCircleID = NullID;
            e.origEllipseID = ellipse.id;
            subEdges.push_back(e);
        }
    }

    // ---- Add ellipse arc entities as polyline sub-edges ----
    for (int ei = 0; ei < numEllipseArcs; ei++) {
        const auto& ea = sketch.ellipseArcs[ei];
        if (ellipseArcSamples[ei].empty()) continue;

        auto vtxChain = buildSplitChain(ellipseArcSamples[ei], ellipseArcIxPts[ei],
                                         ea.startPt, ea.endPt, false);

        for (int si = 0; si + 1 < (int)vtxChain.size(); si++) {
            if (vtxChain[si] == vtxChain[si + 1]) continue;
            SubEdge e;
            e.fromVtx = vtxChain[si];
            e.toVtx = vtxChain[si + 1];
            e.origLineID = NullID;
            e.origCircleID = NullID;
            e.origEllipseArcID = ea.id;
            subEdges.push_back(e);
        }
    }

    // ---- Add spline entities as polyline sub-edges ----
    for (int si = 0; si < numSplines; si++) {
        const auto& sp = sketch.splines[si];
        if (sp.controlPtIDs.size() < 2 || splineSamples[si].empty()) continue;

        auto vtxChain = buildSplitChain(splineSamples[si], splineIxPts[si],
                                         sp.controlPtIDs.front(), sp.controlPtIDs.back(),
                                         sp.periodic);

        for (int j = 0; j + 1 < (int)vtxChain.size(); j++) {
            if (vtxChain[j] == vtxChain[j + 1]) continue;
            SubEdge e;
            e.fromVtx = vtxChain[j];
            e.toVtx = vtxChain[j + 1];
            e.origLineID = NullID;
            e.origCircleID = NullID;
            e.origSplineID = sp.id;
            subEdges.push_back(e);
        }
    }

    // ---- Add arc entities as sub-edges ----
    for (int ai = 0; ai < numArcs; ai++) {
        const auto& arc = sketch.arcs[ai];

        Point2D center = sketch.getPointPos(arc.centerPt);
        Point2D sp = sketch.getPointPos(arc.startPt);
        Point2D ep = sketch.getPointPos(arc.endPt);
        float radius = distance(center, sp);

        int startVtx = findOrAddVertex(subVerts, sp, arc.startPt);
        int endVtx = findOrAddVertex(subVerts, ep, arc.endPt);

        auto& ixPts = arcIxPts[ai];

        if (ixPts.size() < 1) {
            // No intersections — single arc sub-edge from start to end
            SubEdge e;
            e.fromVtx = startVtx;
            e.toVtx = endVtx;
            e.origLineID = NullID;
            e.origCircleID = NullID;
            e.origArcID = arc.id;
            e.arcCenter = center;
            e.arcRadius = radius;
            e.arcFromAngle = arc.startAngle;
            e.arcToAngle = arc.endAngle;
            subEdges.push_back(e);
        } else {
            // Build chain: start, intersection points (sorted by angle), end
            std::vector<std::pair<float, int>> chain;
            chain.push_back({arc.startAngle, startVtx});
            for (const auto& ix : ixPts) {
                if (ix.second != startVtx && ix.second != endVtx)
                    chain.push_back(ix);
            }
            chain.push_back({arc.endAngle, endVtx});

            // Sort by angle relative to start
            // (angles should already be in order since arcIxPts was sorted)

            for (int j = 0; j + 1 < (int)chain.size(); j++) {
                if (chain[j].second == chain[j+1].second) continue;
                float dx = subVerts[chain[j].second].pos.x - subVerts[chain[j+1].second].pos.x;
                float dy = subVerts[chain[j].second].pos.y - subVerts[chain[j+1].second].pos.y;
                if (dx*dx + dy*dy < kMergeEps * kMergeEps) continue;

                SubEdge e;
                e.fromVtx = chain[j].second;
                e.toVtx = chain[j+1].second;
                e.origLineID = NullID;
                e.origCircleID = NullID;
                e.origArcID = arc.id;
                e.arcCenter = center;
                e.arcRadius = radius;
                e.arcFromAngle = chain[j].first;
                e.arcToAngle = chain[j+1].first;
                subEdges.push_back(e);
            }
        }
    }
}

// ---- Main detection function ----

std::vector<ClosedProfile> detectClosedProfilesCustom(const Sketch& sketch) {
    // ---- Step 0: Build subdivided edge graph ----
    std::vector<SubVertex> subVerts;
    std::vector<SubEdge> subEdges;
    std::unordered_set<EntityID> splitCircleIDs; // circles that got split into arcs
    buildSubdivision(sketch, subVerts, subEdges, splitCircleIDs);
    // ---- Step 1: Build directed half-edges with tangent angles ----
    std::vector<SubHalfEdge> halfEdges;
    std::unordered_map<int, std::vector<int>> outgoing;

    for (int i = 0; i < (int)subEdges.size(); i++) {
        const auto& e = subEdges[i];

        float angle_fwd = computeDepartureAngle(e, subVerts, true);
        float angle_rev = computeDepartureAngle(e, subVerts, false);

        int idx = (int)halfEdges.size();
        halfEdges.push_back({i, e.fromVtx, e.toVtx, angle_fwd});
        halfEdges.push_back({i, e.toVtx, e.fromVtx, angle_rev});

        outgoing[e.fromVtx].push_back(idx);
        outgoing[e.toVtx].push_back(idx + 1);
    }

    // Sort outgoing edges at each vertex by angle
    for (auto& [vtx, edges] : outgoing) {
        std::sort(edges.begin(), edges.end(), [&](int a, int b) {
            return halfEdges[a].angle < halfEdges[b].angle;
        });
    }

    // For each half-edge, compute the "next" half-edge in the face
    std::vector<int> next(halfEdges.size(), -1);

    for (int i = 0; i < (int)halfEdges.size(); i++) {
        int twin = i ^ 1;
        if (twin >= (int)halfEdges.size()) continue;
        int vertex = halfEdges[twin].fromVtx;
        auto it = outgoing.find(vertex);
        if (it == outgoing.end() || it->second.empty()) continue;
        const auto& out = it->second;

        int twinPos = -1;
        for (int k = 0; k < (int)out.size(); k++) {
            if (out[k] == twin) { twinPos = k; break; }
        }
        if (twinPos < 0) continue;

        // At a valence-1 vertex (out.size()==1), prevPos would point back to twin itself,
        // creating a degenerate face. Skip this — no valid face can start here.
        if ((int)out.size() < 2) continue;

        int prevPos = (twinPos - 1 + (int)out.size()) % (int)out.size();
        next[i] = out[prevPos];
    }

    // ---- Step 2: Trace all minimal faces ----
    std::unordered_set<int> visited;
    std::vector<ClosedProfile> rawBoundaries;

    for (int startHE = 0; startHE < (int)halfEdges.size(); startHE++) {
        if (visited.count(startHE)) continue;

        std::vector<int> face;
        int cur = startHE;
        bool valid = true;
        int maxSteps = (int)halfEdges.size() + 1;

        while (maxSteps-- > 0) {
            if (visited.count(cur)) {
                if (cur == startHE && !face.empty()) break;
                valid = false;
                break;
            }
            face.push_back(cur);
            visited.insert(cur);

            int nx = next[cur];
            if (nx < 0) { valid = false; break; }
            cur = nx;
        }

        if (!valid || face.size() < 2) continue;
        if (cur != startHE) continue;

        // Build ClosedProfile from this face
        ClosedProfile profile;
        std::unordered_set<EntityID> lineIDset;
        for (int he : face) {
            const auto& edge = subEdges[halfEdges[he].edgeIdx];

            profile.pointIDs.push_back(subVerts[halfEdges[he].fromVtx].origPointID);
            profile.resolvedPoints.push_back(subVerts[halfEdges[he].fromVtx].pos);

            if (edge.origLineID != NullID && lineIDset.insert(edge.origLineID).second) {
                profile.lineIDs.push_back(edge.origLineID);
            }

            // Build BoundarySegment
            BoundarySegment seg;
            if (edge.isArc()) {
                bool isForward = (halfEdges[he].fromVtx == edge.fromVtx);
                seg.type = SegmentType::Arc;
                seg.arcCenter = edge.arcCenter;
                seg.arcRadius = edge.arcRadius;
                seg.origCircleID = (edge.origCircleID != NullID) ? edge.origCircleID : edge.origArcID;
                if (isForward) {
                    seg.arcStartAngle = edge.arcFromAngle;
                    seg.arcEndAngle = edge.arcToAngle;
                    // Ensure CCW span is positive (raw atan2 values may cross ±π boundary)
                    if (seg.arcEndAngle <= seg.arcStartAngle)
                        seg.arcEndAngle += 2.0f * kPi;
                } else {
                    seg.arcStartAngle = edge.arcToAngle;
                    seg.arcEndAngle = edge.arcFromAngle;
                    if (seg.arcEndAngle >= seg.arcStartAngle)
                        seg.arcEndAngle -= 2.0f * kPi;
                }
            }
            profile.segments.push_back(seg);
        }

        // Skip exterior faces (negative signed area = CW winding)
        bool hasArcs = false;
        for (const auto& s : profile.segments) {
            if (s.type == SegmentType::Arc) { hasArcs = true; break; }
        }

        float signedArea;
        if (!hasArcs) {
            signedArea = polygonSignedArea(profile.resolvedPoints);
        } else {
            signedArea = polygonSignedArea(tessellateProfile(sketch, profile));
        }
        if (signedArea <= 0) continue;

        rawBoundaries.push_back(std::move(profile));
    }

    // ---- Step 3: Add non-split circles ----
    for (const auto& circle : sketch.circles) {

        if (circle.radius > 1e-6f && splitCircleIDs.find(circle.id) == splitCircleIDs.end()) {
            ClosedProfile cp;
            cp.circleID = circle.id;
            rawBoundaries.push_back(std::move(cp));
        }
    }

    // ---- Step 4: Compute containment tree ----
    int n = (int)rawBoundaries.size();
    if (n <= 1) return rawBoundaries;

    // Cache tessellations to avoid O(n²) redundant recomputation
    std::vector<std::vector<Point2D>> tessCache(n);
    for (int i = 0; i < n; i++)
        tessCache[i] = tessellateProfile(sketch, rawBoundaries[i]);

    // Sort by area (largest first) for containment testing
    std::vector<int> order(n);
    for (int i = 0; i < n; i++) order[i] = i;
    std::sort(order.begin(), order.end(), [&](int a, int b) {
        return polygonArea(tessCache[a]) > polygonArea(tessCache[b]);
    });

    // Check if two profiles share any boundary vertex (within tolerance).
    // Profiles that share a vertex are adjacent regions from the half-edge
    // tracer and cannot be nested — skip containment test between them.
    auto sharesVertex = [&](const ClosedProfile& a, const ClosedProfile& b) -> bool {
        if (a.resolvedPoints.empty() || b.resolvedPoints.empty()) return false;
        for (const auto& pa : a.resolvedPoints) {
            for (const auto& pb : b.resolvedPoints) {
                float dx = pa.x - pb.x, dy = pa.y - pb.y;
                if (dx * dx + dy * dy < kMergeEps * kMergeEps) return true;
            }
        }
        return false;
    };

    // Any profile can be contained inside a larger one (becoming a hole).
    // Profiles that share boundary vertices are adjacent (not nested).
    std::vector<int> parent(n, -1);
    for (int i = 0; i < n; i++) {
        Point2D sample = polygonCentroid(tessCache[order[i]]);
        for (int j = i - 1; j >= 0; j--) {
            if (sharesVertex(rawBoundaries[order[i]], rawBoundaries[order[j]]))
                continue;
            if (pointInsidePolygonWinding(tessCache[order[j]], sample)) {
                parent[i] = j;
                break;
            }
        }
    }

    std::vector<int> depth(n, 0);
    for (int i = 0; i < n; i++) {
        if (parent[i] >= 0)
            depth[i] = depth[parent[i]] + 1;
    }

    std::vector<ClosedProfile> results;
    for (int i = 0; i < n; i++) {
        ClosedProfile region = rawBoundaries[order[i]];
        region.holes.clear();
        for (int j = 0; j < n; j++) {
            if (parent[j] == i && depth[j] == depth[i] + 1)
                region.holes.push_back(rawBoundaries[order[j]]);
        }
        results.push_back(std::move(region));
    }

    return results;
}

// ---- Router functions ----

// Default: always uses custom tracer (no plane available for OCCT path)
std::vector<ClosedProfile> detectClosedProfiles(const Sketch& sketch) {
    return detectClosedProfilesCustom(sketch);
}

// With plane: can route to OCCT backend if preference is set
std::vector<ClosedProfile> detectClosedProfiles(const Sketch& sketch, const SketchPlane& plane) {
    // TODO: check preference for backend selection
    // For now, always use custom tracer until OCCT backend is implemented
    (void)plane;
    return detectClosedProfilesCustom(sketch);
}

} // namespace shitcad
