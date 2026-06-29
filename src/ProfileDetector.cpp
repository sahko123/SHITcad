#include "ProfileDetector.h"
#include "SketchPlane.h"
#include "Preferences.h"
#include "Intersect.h"
#include <unordered_map>
#include <unordered_set>

// OCCT includes for BOPAlgo_BuilderFace profile detection
#include <BOPAlgo_BuilderFace.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRep_Tool.hxx>
#include <Geom_Circle.hxx>
#include <Geom_Ellipse.hxx>
#include <Geom_BSplineCurve.hxx>
#include <Geom_Line.hxx>
#include <Geom_Plane.hxx>
#include <GeomAPI_ProjectPointOnCurve.hxx>
#include <gp_Circ.hxx>
#include <gp_Elips.hxx>
#include <gp_Pln.hxx>
#include <gp_Ax2.hxx>
#include <TColgp_Array1OfPnt.hxx>
#include <TColStd_Array1OfReal.hxx>
#include <TColStd_Array1OfInteger.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Edge.hxx>
#include <TopoDS_Wire.hxx>
#include <TopoDS_Face.hxx>
#include <TopExp.hxx>
#include <TopTools_IndexedMapOfShape.hxx>
#include <TopTools_ListOfShape.hxx>
#include <BRepAdaptor_Curve.hxx>
#include <GCPnts_AbscissaPoint.hxx>
#include <ShapeAnalysis.hxx>
#include <ShapeAnalysis_Wire.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>

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
        double dx = verts[i].pos.x - pos.x;
        double dy = verts[i].pos.y - pos.y;
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
        double r1 = sketch.circles[ci].radius;

        for (int cj = ci + 1; cj < numCircles; cj++) {

            Point2D c2 = sketch.getPointPos(sketch.circles[cj].centerPt);
            double r2 = sketch.circles[cj].radius;

            auto hits = circleCircleIntersection(c1, f(r1), c2, f(r2));
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
        double radius = distance(center, sp);

        for (int li = 0; li < numLines; li++) {

            Point2D a = sketch.getPointPos(sketch.lines[li].startPt);
            Point2D b = sketch.getPointPos(sketch.lines[li].endPt);
            auto hits = circleLineIntersection(center, f(radius), a, b);
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
            double r2 = sketch.circles[ci].radius;
            auto hits = circleCircleIntersection(center, f(radius), c2, f(r2));
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
            double r2 = distance(c2, sp2);
            auto hits = circleCircleIntersection(center, f(radius), c2, f(r2));
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
        double radius = sketch.circles[ci].radius;
        for (int si = 0; si + 1 < (int)samples.size(); si++) {
            auto hits = circleLineIntersection(center, f(radius), samples[si], samples[si+1]);
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
        double radius = distance(center, sp);
        for (int si = 0; si + 1 < (int)samples.size(); si++) {
            auto hits = circleLineIntersection(center, f(radius), samples[si], samples[si+1]);
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
            double dx = subVerts[chainVerts[j]].pos.x - subVerts[chainVerts[j+1]].pos.x;
            double dy = subVerts[chainVerts[j]].pos.y - subVerts[chainVerts[j+1]].pos.y;
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
            e.arcRadius = static_cast<float>(circle.radius);
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
        double radius = distance(center, sp);

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
            e.arcRadius = static_cast<float>(radius);
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
                double dx = subVerts[chain[j].second].pos.x - subVerts[chain[j+1].second].pos.x;
                double dy = subVerts[chain[j].second].pos.y - subVerts[chain[j+1].second].pos.y;
                if (dx*dx + dy*dy < kMergeEps * kMergeEps) continue;

                SubEdge e;
                e.fromVtx = chain[j].second;
                e.toVtx = chain[j+1].second;
                e.origLineID = NullID;
                e.origCircleID = NullID;
                e.origArcID = arc.id;
                e.arcCenter = center;
                e.arcRadius = static_cast<float>(radius);
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

        double signedArea;
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
                double dx = pa.x - pb.x, dy = pa.y - pb.y;
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

// ===========================================================================
// OCCT-based profile detection using BOPAlgo_BuilderFace
// ===========================================================================

// Helper: convert local 2D point to 3D on the sketch plane
static gp_Pnt toGpPnt(const SketchPlane& plane, double lx, double ly) {
    float wx, wy, wz;
    plane.localToWorld((float)lx, (float)ly, wx, wy, wz);
    return gp_Pnt(wx, wy, wz);
}

// Metadata for an OCCT edge: which sketch entity it came from
struct EdgeOrigin {
    EntityID entityID = NullID;
    enum Kind { KLine, KCircle, KArc, KEllipse, KEllipseArc, KSpline } kind = KLine;
};

// Build OCCT edges from sketch entities and track their origins
static void buildOCCTEdges(const Sketch& sketch, const SketchPlane& plane,
                           std::vector<TopoDS_Edge>& edges,
                           std::vector<EdgeOrigin>& origins) {
    gp_Dir normalDir(plane.normal[0], plane.normal[1], plane.normal[2]);

    // Lines
    for (const auto& line : sketch.lines) {
        if (line.projected) continue;
        Point2D a = sketch.getPointPos(line.startPt);
        Point2D b = sketch.getPointPos(line.endPt);
        gp_Pnt pa = toGpPnt(plane, a.x, a.y);
        gp_Pnt pb = toGpPnt(plane, b.x, b.y);
        if (pa.Distance(pb) < 1e-6) continue;

        BRepBuilderAPI_MakeEdge me(pa, pb);
        if (me.IsDone()) {
            edges.push_back(me.Edge());
            origins.push_back({line.id, EdgeOrigin::KLine});
        }
    }

    // Circles
    for (const auto& circle : sketch.circles) {
        if (circle.projected) continue;
        if (circle.radius < 1e-6) continue;
        Point2D center = sketch.getPointPos(circle.centerPt);
        gp_Pnt cp = toGpPnt(plane, center.x, center.y);
        gp_Ax2 ax(cp, normalDir);
        gp_Circ circ(ax, circle.radius);

        BRepBuilderAPI_MakeEdge me(circ);
        if (me.IsDone()) {
            edges.push_back(me.Edge());
            origins.push_back({circle.id, EdgeOrigin::KCircle});
        }
    }

    // Arcs
    for (const auto& arc : sketch.arcs) {
        if (arc.projected) continue;
        Point2D center = sketch.getPointPos(arc.centerPt);
        Point2D sp = sketch.getPointPos(arc.startPt);
        Point2D ep = sketch.getPointPos(arc.endPt);
        double radius = distance(center, sp);
        if (radius < 1e-6) continue;

        gp_Pnt cp = toGpPnt(plane, center.x, center.y);
        gp_Ax2 ax(cp, normalDir);
        gp_Circ circ(ax, radius);

        // Build arc edge between start and end angles
        BRepBuilderAPI_MakeEdge me(circ, arc.startAngle, arc.endAngle);
        if (me.IsDone()) {
            edges.push_back(me.Edge());
            origins.push_back({arc.id, EdgeOrigin::KArc});
        }
    }

    // Ellipses
    for (const auto& ellipse : sketch.ellipses) {
        if (ellipse.projected) continue;
        if (ellipse.semiMajor < 1e-6 || ellipse.semiMinor < 1e-6) continue;
        Point2D center = sketch.getPointPos(ellipse.centerPt);
        gp_Pnt cp = toGpPnt(plane, center.x, center.y);

        double cosR = std::cos(ellipse.rotation);
        double sinR = std::sin(ellipse.rotation);
        gp_Dir majorDir(
            plane.uAxis[0] * cosR + plane.vAxis[0] * sinR,
            plane.uAxis[1] * cosR + plane.vAxis[1] * sinR,
            plane.uAxis[2] * cosR + plane.vAxis[2] * sinR
        );
        gp_Ax2 ax(cp, normalDir, majorDir);
        gp_Elips elips(ax, ellipse.semiMajor, ellipse.semiMinor);

        BRepBuilderAPI_MakeEdge me(elips);
        if (me.IsDone()) {
            edges.push_back(me.Edge());
            origins.push_back({ellipse.id, EdgeOrigin::KEllipse});
        }
    }

    // Ellipse arcs
    for (const auto& ea : sketch.ellipseArcs) {
        if (ea.projected) continue;
        if (ea.semiMajor < 1e-6 || ea.semiMinor < 1e-6) continue;
        Point2D center = sketch.getPointPos(ea.centerPt);
        gp_Pnt cp = toGpPnt(plane, center.x, center.y);

        double cosR = std::cos(ea.rotation);
        double sinR = std::sin(ea.rotation);
        gp_Dir majorDir(
            plane.uAxis[0] * cosR + plane.vAxis[0] * sinR,
            plane.uAxis[1] * cosR + plane.vAxis[1] * sinR,
            plane.uAxis[2] * cosR + plane.vAxis[2] * sinR
        );
        gp_Ax2 ax(cp, normalDir, majorDir);
        gp_Elips elips(ax, ea.semiMajor, ea.semiMinor);

        BRepBuilderAPI_MakeEdge me(elips, ea.startAngle, ea.endAngle);
        if (me.IsDone()) {
            edges.push_back(me.Edge());
            origins.push_back({ea.id, EdgeOrigin::KEllipseArc});
        }
    }

    // Splines
    for (const auto& sp : sketch.splines) {
        if (sp.projected) continue;
        int nCtrl = (int)sp.controlPtIDs.size();
        if (nCtrl < 2) continue;

        TColgp_Array1OfPnt poles(1, nCtrl);
        for (int ci = 0; ci < nCtrl; ci++) {
            Point2D pt = sketch.getPointPos(sp.controlPtIDs[ci]);
            poles.SetValue(ci + 1, toGpPnt(plane, pt.x, pt.y));
        }

        // Generate knots if not stored
        std::vector<double> knots = sp.knots;
        if (knots.empty()) knots = generateUniformKnots(nCtrl, sp.degree);

        // Convert full knot vector to unique knots + multiplicities
        std::vector<double> uniqueKnots;
        std::vector<int> mults;
        for (double k : knots) {
            if (uniqueKnots.empty() || std::fabs(k - uniqueKnots.back()) > 1e-10) {
                uniqueKnots.push_back(k);
                mults.push_back(1);
            } else {
                mults.back()++;
            }
        }

        int nKnots = (int)uniqueKnots.size();
        TColStd_Array1OfReal knotsArr(1, nKnots);
        TColStd_Array1OfInteger multsArr(1, nKnots);
        for (int ki = 0; ki < nKnots; ki++) {
            knotsArr.SetValue(ki + 1, uniqueKnots[ki]);
            multsArr.SetValue(ki + 1, mults[ki]);
        }

        bool hasWeights = !sp.weights.empty() && (int)sp.weights.size() == nCtrl;
        TColStd_Array1OfReal weightsArr(1, nCtrl);
        for (int ci = 0; ci < nCtrl; ci++)
            weightsArr.SetValue(ci + 1, hasWeights ? sp.weights[ci] : 1.0);

        try {
            Handle(Geom_BSplineCurve) bspline = new Geom_BSplineCurve(
                poles, weightsArr, knotsArr, multsArr, sp.degree);

            BRepBuilderAPI_MakeEdge me(bspline);
            if (me.IsDone()) {
                edges.push_back(me.Edge());
                origins.push_back({sp.id, EdgeOrigin::KSpline});
            }
        } catch (...) {
            // Skip invalid spline
        }
    }
}

// Map an output edge from BOPAlgo back to its source sketch entity.
// BOPAlgo_BuilderFace doesn't expose Modified()/Generated(), so we match
// by checking if the output edge shares the same underlying curve (IsPartner/IsSame)
// or by sampling a midpoint and finding the closest input edge.
static EdgeOrigin findEdgeOrigin(const TopoDS_Edge& outputEdge,
                                  const std::vector<TopoDS_Edge>& inputEdges,
                                  const std::vector<EdgeOrigin>& inputOrigins) {
    // First: check topological identity (IsSame covers un-split edges)
    for (int i = 0; i < (int)inputEdges.size(); i++) {
        if (outputEdge.IsSame(inputEdges[i]))
            return inputOrigins[i];
    }

    // Second: check if they share the same underlying curve (IsPartner — same TShape)
    for (int i = 0; i < (int)inputEdges.size(); i++) {
        if (outputEdge.IsPartner(inputEdges[i]))
            return inputOrigins[i];
    }

    // Third: sample midpoint of output edge and find closest input edge
    BRepAdaptor_Curve outCurve(outputEdge);
    double outMid = (outCurve.FirstParameter() + outCurve.LastParameter()) * 0.5;
    gp_Pnt midPt = outCurve.Value(outMid);

    double bestDist = 1e9;
    int bestIdx = -1;
    for (int i = 0; i < (int)inputEdges.size(); i++) {
        BRepAdaptor_Curve inCurve(inputEdges[i]);
        // Project midpoint onto input curve
        double inFirst = inCurve.FirstParameter();
        double inLast = inCurve.LastParameter();
        // Sample a few points on the input curve and find minimum distance
        double minD = 1e9;
        for (int s = 0; s <= 10; s++) {
            double t = inFirst + (inLast - inFirst) * s / 10.0;
            double d = midPt.Distance(inCurve.Value(t));
            if (d < minD) minD = d;
        }
        if (minD < bestDist) {
            bestDist = minD;
            bestIdx = i;
        }
    }

    if (bestIdx >= 0 && bestDist < 0.01)
        return inputOrigins[bestIdx];

    return {}; // unknown origin
}

// Build a BoundarySegment from an OCCT edge + its origin info
static BoundarySegment buildSegmentFromEdge(const TopoDS_Edge& edge,
                                             const EdgeOrigin& origin,
                                             const Sketch& sketch,
                                             const SketchPlane& plane) {
    BoundarySegment seg;

    BRepAdaptor_Curve adaptor(edge);
    double paramFirst = adaptor.FirstParameter();
    double paramLast = adaptor.LastParameter();

    switch (adaptor.GetType()) {
        case GeomAbs_Line:
            seg.type = SegmentType::Line;
            break;

        case GeomAbs_Circle: {
            seg.type = SegmentType::Arc;
            gp_Circ circ = adaptor.Circle();
            gp_Pnt center3D = circ.Location();
            float lx, ly;
            plane.worldToLocal((float)center3D.X(), (float)center3D.Y(), (float)center3D.Z(), lx, ly);
            seg.arcCenter = {lx, ly};
            seg.arcRadius = static_cast<float>(circ.Radius());
            seg.arcStartAngle = paramFirst;
            seg.arcEndAngle = paramLast;
            seg.origCircleID = origin.entityID;
            break;
        }

        case GeomAbs_Ellipse: {
            seg.type = (origin.kind == EdgeOrigin::KEllipseArc) ? SegmentType::EllipseArc : SegmentType::Ellipse;
            gp_Elips elips = adaptor.Ellipse();
            gp_Pnt center3D = elips.Location();
            float lx, ly;
            plane.worldToLocal((float)center3D.X(), (float)center3D.Y(), (float)center3D.Z(), lx, ly);
            seg.ellipseCenter = {lx, ly};
            seg.semiMajor = elips.MajorRadius();
            seg.semiMinor = elips.MinorRadius();

            // Recover rotation: project major axis direction back to local 2D
            gp_Dir majorDir = elips.XAxis().Direction();
            // Major axis direction is a direction vector, project its tip
            float tipX = (float)(center3D.X() + majorDir.X());
            float tipY = (float)(center3D.Y() + majorDir.Y());
            float tipZ = (float)(center3D.Z() + majorDir.Z());
            float tlx, tly;
            plane.worldToLocal(tipX, tipY, tipZ, tlx, tly);
            seg.ellipseRotation = std::atan2(tly - ly, tlx - lx);

            seg.ellipseStartAngle = paramFirst;
            seg.ellipseEndAngle = paramLast;
            seg.origEllipseID = origin.entityID;
            break;
        }

        case GeomAbs_BSplineCurve: {
            seg.type = SegmentType::Spline;
            seg.origSplineID = origin.entityID;
            seg.splineParamStart = paramFirst;
            seg.splineParamEnd = paramLast;

            // Extract spline data from the sketch entity
            const SplineEntity* sp = sketch.findSpline(origin.entityID);
            if (sp) {
                seg.splineDegree = sp->degree;
                for (auto id : sp->controlPtIDs)
                    seg.splineControlPts.push_back(sketch.getPointPos(id));
                seg.splineKnots = sp->knots;
                if (seg.splineKnots.empty())
                    seg.splineKnots = generateUniformKnots((int)sp->controlPtIDs.size(), sp->degree);
                seg.splineWeights = sp->weights;
            }
            break;
        }

        default:
            seg.type = SegmentType::Line;
            break;
    }

    return seg;
}

std::vector<ClosedProfile> detectClosedProfilesOCCT(const Sketch& sketch, const SketchPlane& plane) {
    std::vector<ClosedProfile> results;

    // Step 1: Build OCCT edges from all sketch entities
    std::vector<TopoDS_Edge> inputEdges;
    std::vector<EdgeOrigin> inputOrigins;
    buildOCCTEdges(sketch, plane, inputEdges, inputOrigins);
    fprintf(stderr, "[OCCT] Step 1: %d input edges built from sketch (%d pts, %d lines, %d circles, %d arcs)\n",
            (int)inputEdges.size(), (int)sketch.points.size(), (int)sketch.lines.size(),
            (int)sketch.circles.size(), (int)sketch.arcs.size());
    if (inputEdges.empty()) { fprintf(stderr, "[OCCT] ABORT: no input edges\n"); return results; }

    // Step 2: Create the base face (bounded plane)
    gp_Pnt planeOrigin(plane.origin[0], plane.origin[1], plane.origin[2]);
    gp_Dir normal(plane.normal[0], plane.normal[1], plane.normal[2]);
    gp_Pln gpPlane(planeOrigin, normal);
    BRepBuilderAPI_MakeFace faceMaker(gpPlane, -1e6, 1e6, -1e6, 1e6);
    if (!faceMaker.IsDone()) { fprintf(stderr, "[OCCT] ABORT: MakeFace failed\n"); return results; }
    TopoDS_Face baseFace = faceMaker.Face();
    fprintf(stderr, "[OCCT] Step 2: base face created\n");

    // Step 3: Feed edges to BOPAlgo_BuilderFace
    BOPAlgo_BuilderFace faceBuilder;
    faceBuilder.SetFace(baseFace);

    TopTools_ListOfShape edgeList;
    for (const auto& e : inputEdges)
        edgeList.Append(e);
    faceBuilder.SetShapes(edgeList);

    faceBuilder.Perform();
    if (faceBuilder.HasErrors()) {
        fprintf(stderr, "[OCCT] ABORT: BOPAlgo_BuilderFace has errors\n");
        return results;
    }
    if (faceBuilder.HasWarnings()) {
        fprintf(stderr, "[OCCT] WARNING: BOPAlgo_BuilderFace has warnings\n");
    }

    // Step 4: Extract result faces and convert to ClosedProfile
    const TopTools_ListOfShape& resultFaces = faceBuilder.Areas();
    int faceCount = 0;
    for (auto it2 = resultFaces.cbegin(); it2 != resultFaces.cend(); ++it2) faceCount++;
    fprintf(stderr, "[OCCT] Step 4: %d result faces from BOPAlgo\n", faceCount);

    int faceIdx = 0;
    for (auto it = resultFaces.cbegin(); it != resultFaces.cend(); ++it, ++faceIdx) {
        const TopoDS_Face& face = TopoDS::Face(*it);

        // Get outer wire
        TopoDS_Wire outerWire = ShapeAnalysis::OuterWire(face);
        if (outerWire.IsNull()) {
            fprintf(stderr, "[OCCT]   face[%d]: outer wire is null, skipping\n", faceIdx);
            continue;
        }

        // Count edges in wire
        int edgeCount = 0;
        for (TopExp_Explorer expCount(outerWire, TopAbs_EDGE); expCount.More(); expCount.Next()) edgeCount++;
        fprintf(stderr, "[OCCT]   face[%d]: outer wire has %d edges\n", faceIdx, edgeCount);

        // Build profile from outer wire
        ClosedProfile profile;
        std::unordered_set<EntityID> lineIDset, ellipseIDset, splineIDset;

        for (TopExp_Explorer expEdge(outerWire, TopAbs_EDGE); expEdge.More(); expEdge.Next()) {
            const TopoDS_Edge& edge = TopoDS::Edge(expEdge.Current());

            // Get start vertex position in local coords
            gp_Pnt startPt = BRep_Tool::Pnt(TopExp::FirstVertex(edge, Standard_True));
            float lx, ly;
            plane.worldToLocal((float)startPt.X(), (float)startPt.Y(), (float)startPt.Z(), lx, ly);
            profile.resolvedPoints.push_back({(double)lx, (double)ly});

            // Find which sketch entity this edge came from
            EdgeOrigin origin = findEdgeOrigin(edge, inputEdges, inputOrigins);

            // Build segment with real curve data
            BoundarySegment seg = buildSegmentFromEdge(edge, origin, sketch, plane);
            profile.segments.push_back(seg);

            // Track entity IDs
            if (origin.entityID != NullID) {
                switch (origin.kind) {
                    case EdgeOrigin::KLine:
                        if (lineIDset.insert(origin.entityID).second)
                            profile.lineIDs.push_back(origin.entityID);
                        break;
                    case EdgeOrigin::KCircle:
                    case EdgeOrigin::KArc:
                        break; // origCircleID already set in buildSegmentFromEdge
                    case EdgeOrigin::KEllipse:
                    case EdgeOrigin::KEllipseArc:
                        if (ellipseIDset.insert(origin.entityID).second)
                            profile.ellipseIDs.push_back(origin.entityID);
                        break;
                    case EdgeOrigin::KSpline:
                        if (splineIDset.insert(origin.entityID).second)
                            profile.splineIDs.push_back(origin.entityID);
                        break;
                }
            }
        }

        fprintf(stderr, "[OCCT]   face[%d]: %d resolved points, %d segments, %d lineIDs\n",
                faceIdx, (int)profile.resolvedPoints.size(), (int)profile.segments.size(),
                (int)profile.lineIDs.size());

        // Check if this is a single full circle (before the point count check,
        // because a full circle OCCT edge has only 1 vertex)
        if (profile.segments.size() == 1 && profile.segments[0].type == SegmentType::Arc) {
            const auto& seg = profile.segments[0];
            double span = std::fabs(seg.arcEndAngle - seg.arcStartAngle);
            if (span > 6.0) { // ~2*pi = full circle
                for (const auto& circle : sketch.circles) {
                    if (std::fabs(circle.radius - seg.arcRadius) < 1e-4) {
                        Point2D center = sketch.getPointPos(circle.centerPt);
                        if (std::fabs(center.x - seg.arcCenter.x) < 1e-4 &&
                            std::fabs(center.y - seg.arcCenter.y) < 1e-4) {
                            profile.circleID = circle.id;
                            profile.resolvedPoints.clear();
                            profile.segments.clear();
                            profile.lineIDs.clear();
                            fprintf(stderr, "[OCCT]   face[%d]: identified as full circle (id=%u)\n", faceIdx, circle.id);
                            break;
                        }
                    }
                }
            }
        }

        // Non-circle profiles need at least 2 points
        if (!profile.isCircle() && profile.resolvedPoints.size() < 2) {
            fprintf(stderr, "[OCCT]   face[%d]: too few points and not a circle, skipping\n", faceIdx);
            continue;
        }

        // Also handle full ellipses (single edge, type Ellipse)
        if (!profile.isCircle() && profile.segments.size() == 1 &&
            profile.segments[0].type == SegmentType::Ellipse) {
            // Full ellipse — treat similarly to circle but keep as ellipse segment
            // Just ensure it passes through (resolvedPoints may have only 1 entry)
            fprintf(stderr, "[OCCT]   face[%d]: full ellipse detected\n", faceIdx);
        }

        // Compute signed area to filter exterior face
        if (!profile.isCircle()) {
            auto tess = tessellateProfile(sketch, profile);
            double area = polygonSignedArea(tess);
            fprintf(stderr, "[OCCT]   face[%d]: signed area = %.4f (tessPoints=%d)\n",
                    faceIdx, area, (int)tess.size());
            if (area <= 0) {
                fprintf(stderr, "[OCCT]   face[%d]: exterior face (CW), skipping\n", faceIdx);
                continue;
            }
        }

        // Add holes from inner wires
        int holeCount = 0;
        for (TopExp_Explorer expWire(face, TopAbs_WIRE); expWire.More(); expWire.Next()) {
            const TopoDS_Wire& wire = TopoDS::Wire(expWire.Current());
            if (wire.IsSame(outerWire)) continue;

            ClosedProfile hole;
            for (TopExp_Explorer expEdge(wire, TopAbs_EDGE); expEdge.More(); expEdge.Next()) {
                const TopoDS_Edge& edge = TopoDS::Edge(expEdge.Current());
                gp_Pnt startPt = BRep_Tool::Pnt(TopExp::FirstVertex(edge, Standard_True));
                float hlx, hly;
                plane.worldToLocal((float)startPt.X(), (float)startPt.Y(), (float)startPt.Z(), hlx, hly);
                hole.resolvedPoints.push_back({(double)hlx, (double)hly});

                EdgeOrigin hOrigin = findEdgeOrigin(edge, inputEdges, inputOrigins);
                hole.segments.push_back(buildSegmentFromEdge(edge, hOrigin, sketch, plane));
            }
            if (hole.resolvedPoints.size() >= 2) {
                profile.holes.push_back(std::move(hole));
                holeCount++;
            }
        }

        fprintf(stderr, "[OCCT]   face[%d]: ACCEPTED (%d holes)\n", faceIdx, holeCount);
        results.push_back(std::move(profile));
    }

    fprintf(stderr, "[OCCT] Final: %d profiles returned\n", (int)results.size());
    return results;
}

// ---- Router functions ----

// Default: always uses custom tracer (no plane available for OCCT path)
std::vector<ClosedProfile> detectClosedProfiles(const Sketch& sketch) {
    return detectClosedProfilesCustom(sketch);
}

// With plane: routes to OCCT backend if preference is set
std::vector<ClosedProfile> detectClosedProfiles(const Sketch& sketch, const SketchPlane& plane) {
    if (activeProfileBackend() == ProfileDetectorBackend::OCCT) {
        auto result = detectClosedProfilesOCCT(sketch, plane);
        fprintf(stderr, "[ProfileDetector] OCCT backend: %d profiles detected\n", (int)result.size());
        return result;
    }
    auto result = detectClosedProfilesCustom(sketch);
    fprintf(stderr, "[ProfileDetector] Custom backend: %d profiles detected\n", (int)result.size());
    return result;
}

} // namespace shitcad
