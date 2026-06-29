#include "Tools.h"
#include <cmath>

namespace shitcad {

// ─── Fillet helpers ────────────────────────────────────────────────────────

static double normalizeAngle(double a) {
    const double kTwoPi = 6.28318530717958647692;
    a = std::fmod(a, kTwoPi);
    if (a < 0.0) a += kTwoPi;
    return a;
}

// Returns the unit direction pointing FROM vertexID along the entity (into the entity).
static Point2D directionFromVertex(const Sketch& sketch, EntityID entityID, bool isArc, EntityID vertexID) {
    if (!isArc) {
        const LineEntity* line = sketch.findLine(entityID);
        if (!line) return {};
        EntityID otherPt = (line->startPt == vertexID) ? line->endPt : line->startPt;
        Point2D V = sketch.getPointPos(vertexID);
        Point2D O = sketch.getPointPos(otherPt);
        double dx = O.x - V.x, dy = O.y - V.y;
        double len = std::sqrt(dx * dx + dy * dy);
        if (len < 1e-10) return {};
        return {dx / len, dy / len};
    } else {
        const ArcEntity* arc = sketch.findArc(entityID);
        if (!arc) return {};
        if (arc->startPt == vertexID) {
            // Arc leaves startPt in the CCW direction: (-sin(startAngle), cos(startAngle))
            double sa = arc->startAngle;
            return {-std::sin(sa), std::cos(sa)};
        } else {
            // Arc arrives at endPt in the CCW direction; going back along arc = (sin(endAngle), -cos(endAngle))
            double ea = arc->endAngle;
            return {std::sin(ea), -std::cos(ea)};
        }
    }
}

bool handleFilletVertexClick(const Sketch& sketch, FilletToolState& filletTool, EntityID vertexID) {
    filletTool.reset();

    EntityID ids[2] = {NullID, NullID};
    bool arcFlags[2] = {false, false};
    int found = 0;

    for (const auto& line : sketch.lines) {
        if (line.projected) continue;
        if (line.startPt == vertexID || line.endPt == vertexID) {
            if (found < 2) { ids[found] = line.id; arcFlags[found] = false; }
            found++;
        }
    }
    for (const auto& arc : sketch.arcs) {
        if (arc.projected) continue;
        if (arc.startPt == vertexID || arc.endPt == vertexID) {
            if (found < 2) { ids[found] = arc.id; arcFlags[found] = true; }
            found++;
        }
    }

    if (found != 2) return false;

    filletTool.entity1ID = ids[0];
    filletTool.entity2ID = ids[1];
    filletTool.entity1IsArc = arcFlags[0];
    filletTool.entity2IsArc = arcFlags[1];
    return true;
}

bool applyFillet(Sketch& sketch, FilletToolState& filletTool, EntityID vertexID, double radius) {
    if (radius <= 0.0) return false;

    PointEntity* vertex = sketch.findPoint(vertexID);
    if (!vertex) return false;
    Point2D V = {vertex->x, vertex->y};

    Point2D d1 = directionFromVertex(sketch, filletTool.entity1ID, filletTool.entity1IsArc, vertexID);
    Point2D d2 = directionFromVertex(sketch, filletTool.entity2ID, filletTool.entity2IsArc, vertexID);

    if (d1.x == 0.0 && d1.y == 0.0) return false;
    if (d2.x == 0.0 && d2.y == 0.0) return false;

    // Bisector of the two directions
    double bisX = d1.x + d2.x, bisY = d1.y + d2.y;
    double bisLen = std::sqrt(bisX * bisX + bisY * bisY);
    if (bisLen < 1e-6) return false; // collinear / degenerate
    bisX /= bisLen;
    bisY /= bisLen;

    double cosHalf = d1.x * bisX + d1.y * bisY;
    if (cosHalf < 1e-6) return false; // angle too wide (≥ 180°)
    double sinHalf = std::sqrt(std::max(0.0, 1.0 - cosHalf * cosHalf));
    if (sinHalf < 1e-6) return false; // nearly collinear

    double trimDist = radius * cosHalf / sinHalf; // r / tan(θ/2)

    // Validate trim distance fits within entity lengths for lines
    if (!filletTool.entity1IsArc) {
        const LineEntity* line = sketch.findLine(filletTool.entity1ID);
        if (line) {
            EntityID otherPt = (line->startPt == vertexID) ? line->endPt : line->startPt;
            double lineLen = distance(V, sketch.getPointPos(otherPt));
            if (trimDist >= lineLen) return false;
        }
    }
    if (!filletTool.entity2IsArc) {
        const LineEntity* line = sketch.findLine(filletTool.entity2ID);
        if (line) {
            EntityID otherPt = (line->startPt == vertexID) ? line->endPt : line->startPt;
            double lineLen = distance(V, sketch.getPointPos(otherPt));
            if (trimDist >= lineLen) return false;
        }
    }

    double centerDist = radius / sinHalf;
    double CX = V.x + centerDist * bisX;
    double CY = V.y + centerDist * bisY;

    double T1x = V.x + trimDist * d1.x;
    double T1y = V.y + trimDist * d1.y;
    double T2x = V.x + trimDist * d2.x;
    double T2y = V.y + trimDist * d2.y;

    // Pre-validate all entity pointers before any mutations so we never half-apply.
    LineEntity* eLine1 = filletTool.entity1IsArc ? nullptr : sketch.findLine(filletTool.entity1ID);
    ArcEntity*  eArc1  = filletTool.entity1IsArc ? sketch.findArc(filletTool.entity1ID)  : nullptr;
    LineEntity* eLine2 = filletTool.entity2IsArc ? nullptr : sketch.findLine(filletTool.entity2ID);
    ArcEntity*  eArc2  = filletTool.entity2IsArc ? sketch.findArc(filletTool.entity2ID)  : nullptr;
    if (!filletTool.entity1IsArc && !eLine1) return false;
    if ( filletTool.entity1IsArc && !eArc1)  return false;
    if (!filletTool.entity2IsArc && !eLine2) return false;
    if ( filletTool.entity2IsArc && !eArc2)  return false;

    // Create trim and center points
    EntityID centerID = sketch.addPoint(CX, CY);
    EntityID t1ID = sketch.addPoint(T1x, T1y);
    EntityID t2ID = sketch.addPoint(T2x, T2y);

    // Redirect entity endpoints from V to trim points (using pre-validated pointers)
    if (!filletTool.entity1IsArc) {
        if (eLine1->startPt == vertexID) eLine1->startPt = t1ID;
        else                             eLine1->endPt   = t1ID;
    } else {
        if (eArc1->startPt == vertexID) eArc1->startPt = t1ID;
        else                            eArc1->endPt   = t1ID;
        sketch.recomputeArcAngles(*eArc1);
    }

    if (!filletTool.entity2IsArc) {
        if (eLine2->startPt == vertexID) eLine2->startPt = t2ID;
        else                             eLine2->endPt   = t2ID;
    } else {
        if (eArc2->startPt == vertexID) eArc2->startPt = t2ID;
        else                            eArc2->endPt   = t2ID;
        sketch.recomputeArcAngles(*eArc2);
    }

    // Shrink Distance constraints on the two trimmed lines.
    // Each line is shorter by trimDist; a non-driven Distance constraint still
    // encodes the old length and would push the far endpoint outward if left as-is.
    auto adjustLineLengthConstraints = [&](EntityID lineID) {
        for (auto& c : sketch.constraints) {
            if (c.type == ConstraintType::Distance && c.entityA == lineID && !c.driven) {
                c.value = std::max(0.001, c.value - trimDist);
            }
        }
    };
    if (!filletTool.entity1IsArc) adjustLineLengthConstraints(filletTool.entity1ID);
    if (!filletTool.entity2IsArc) adjustLineLengthConstraints(filletTool.entity2ID);

    // Remove EqualLength constraints that involve a trimmed line paired with any
    // line outside this fillet.  Both fillet lines shrink by the same trimDist so
    // an EqualLength between the two fillet lines themselves remains valid.
    {
        EntityID e1 = filletTool.entity1IsArc ? NullID : filletTool.entity1ID;
        EntityID e2 = filletTool.entity2IsArc ? NullID : filletTool.entity2ID;
        auto isFillet = [&](EntityID id) { return id != NullID && (id == e1 || id == e2); };
        sketch.constraints.erase(
            std::remove_if(sketch.constraints.begin(), sketch.constraints.end(),
                [&](const Constraint& c) {
                    if (c.type != ConstraintType::EqualLength) return false;
                    bool aIsFillet = isFillet(c.entityA);
                    bool bIsFillet = isFillet(c.entityB);
                    // Keep only if both sides are fillet lines (still equal after same trim)
                    return aIsFillet != bIsFillet;
                }),
            sketch.constraints.end());
    }

    // Remove original vertex (it's now unreferenced by the entities we redirected)
    if (!sketch.isPointReferenced(vertexID)) {
        sketch.removePoint(vertexID); // cascades to remove any constraints at V
    }

    // Create fillet arc and fix sweep direction so it passes through V's side
    EntityID filletArcID = sketch.addArc(centerID, t1ID, t2ID);
    ArcEntity* filletArc = sketch.findArc(filletArcID);
    if (filletArc) {
        double vAngle = std::atan2(V.y - CY, V.x - CX);
        double nVA = normalizeAngle(vAngle);
        double nSA = normalizeAngle(filletArc->startAngle);
        double nEA = normalizeAngle(filletArc->endAngle);
        double sweep = nEA - nSA;
        if (sweep <= 0.0) sweep += 6.28318530717958647692;
        double toV = nVA - nSA;
        if (toV < 0.0) toV += 6.28318530717958647692;
        if (toV >= sweep) {
            // V is outside the CCW arc — swap endpoints to flip sweep direction
            std::swap(filletArc->startPt, filletArc->endPt);
            sketch.recomputeArcAngles(*filletArc);
        }
    }

    // Tangent constraints: entityA=line, entityB=arc (solver expects this order)
    sketch.addConstraint(ConstraintType::Tangent, filletTool.entity1ID, filletArcID, 0.0, true);
    sketch.addConstraint(ConstraintType::Tangent, filletTool.entity2ID, filletArcID, 0.0, true);

    return true;
}

bool handleLineTool(Sketch& sketch, ToolState& tool, Point2D worldPos, EntityID snapPointID, EntityID snapCurveID) {
    if (!tool.hasFirstPoint) {
        // Place first point
        if (snapPointID != NullID) {
            tool.firstPointID = snapPointID;
            tool.firstPoint = sketch.getPointPos(snapPointID);
        } else {
            tool.firstPointID = sketch.addPoint(worldPos.x, worldPos.y);
            tool.firstPoint = worldPos;
        }
        tool.hasFirstPoint = true;
        // Remember curve for auto-tangent (only when placing on curve, not on existing point)
        tool.tangentSourceID = (snapPointID == NullID) ? snapCurveID : NullID;
        return false; // not a complete action yet
    }

    // Place second point and create line
    EntityID endPtID;
    EntityID endCurveID = NullID;
    if (snapPointID != NullID) {
        endPtID = snapPointID;
    } else {
        endPtID = sketch.addPoint(worldPos.x, worldPos.y);
        endCurveID = snapCurveID;
    }

    // Prevent zero-length lines (same point clicked twice, or solver moved first point)
    if (endPtID == tool.firstPointID) return false;
    Point2D endPos = sketch.getPointPos(endPtID);
    if (distance(sketch.getPointPos(tool.firstPointID), endPos) < 0.001f) {
        if (snapPointID == NullID) sketch.removePoint(endPtID);
        return false;
    }

    sketch.addLine(tool.firstPointID, endPtID);

    // If end point was placed on a curve, record it as the tangent source for the NEXT line
    // (the current line's tangent source was set when the first point was placed)
    EntityID nextTangentSource = (snapPointID == NullID) ? endCurveID : NullID;

    // Continuous mode: end becomes start of next line
    tool.firstPointID = endPtID;
    tool.firstPoint = sketch.getPointPos(endPtID);
    tool.tangentSourceID = nextTangentSource;

    return true; // action completed
}

bool handleCircleTool(Sketch& sketch, ToolState& tool, Point2D worldPos, EntityID snapPointID) {
    if (!tool.hasFirstPoint) {
        // Place center
        if (snapPointID != NullID) {
            tool.firstPointID = snapPointID;
            tool.firstPoint = sketch.getPointPos(snapPointID);
        } else {
            tool.firstPointID = sketch.addPoint(worldPos.x, worldPos.y);
            tool.firstPoint = worldPos;
        }
        tool.hasFirstPoint = true;
        return false;
    }

    // Compute radius from center to click position (use snap position if snapped)
    Point2D radiusPos = (snapPointID != NullID) ? sketch.getPointPos(snapPointID) : worldPos;
    double radius = distance(tool.firstPoint, radiusPos);
    if (radius > 0.001f) {
        sketch.addCircle(tool.firstPointID, radius);
    } else {
        // Near-zero radius — reject; clean up a freshly created center point
        if (!sketch.isPointReferenced(tool.firstPointID))
            sketch.removePoint(tool.firstPointID);
    }

    tool.reset();
    return true;
}

bool handleRectangleTool(Sketch& sketch, ToolState& tool, Point2D worldPos, EntityID snapPointID) {
    if (!tool.hasFirstPoint) {
        if (snapPointID != NullID) {
            tool.firstPointID = snapPointID;
            tool.firstPoint = sketch.getPointPos(snapPointID);
        } else {
            tool.firstPointID = sketch.addPoint(worldPos.x, worldPos.y);
            tool.firstPoint = worldPos;
        }
        tool.hasFirstPoint = true;
        return false;
    }

    // Create rectangle from two corners
    double x1 = tool.firstPoint.x, y1 = tool.firstPoint.y;
    double x2 = worldPos.x, y2 = worldPos.y;

    // Reject degenerate rectangles (zero-width or zero-height)
    if (std::fabs(x2 - x1) < 0.001 || std::fabs(y2 - y1) < 0.001) {
        if (!sketch.isPointReferenced(tool.firstPointID))
            sketch.removePoint(tool.firstPointID);
        tool.hasFirstPoint = false;
        tool.firstPointID = NullID;
        return false;
    }

    // Four corner points: A(x1,y1) B(x2,y1) C(x2,y2) D(x1,y2)
    // pA always exists (created or reused on first click)
    EntityID pA = tool.firstPointID;
    EntityID pB = sketch.addPoint(x2, y1);
    EntityID pC = (snapPointID != NullID && snapPointID != pA) ? snapPointID : sketch.addPoint(x2, y2);
    EntityID pD = sketch.addPoint(x1, y2);

    // Four lines
    EntityID lAB = sketch.addLine(pA, pB);
    EntityID lBC = sketch.addLine(pB, pC);
    EntityID lCD = sketch.addLine(pC, pD);
    EntityID lDA = sketch.addLine(pD, pA);

    // Horizontal constraints on top and bottom
    sketch.addConstraint(ConstraintType::Horizontal, lAB, NullID, 0.0f, true);
    sketch.addConstraint(ConstraintType::Horizontal, lCD, NullID, 0.0f, true);

    // Vertical constraints on left and right
    sketch.addConstraint(ConstraintType::Vertical, lBC, NullID, 0.0f, true);
    sketch.addConstraint(ConstraintType::Vertical, lDA, NullID, 0.0f, true);

    tool.reset();
    return true;
}

bool handleArc3PointTool(Sketch& sketch, ArcToolState& arcTool, Point2D worldPos, EntityID snapPointID) {
    if (arcTool.clickCount == 0) {
        // Click 1: start point
        if (snapPointID != NullID) {
            arcTool.point1ID = snapPointID;
            arcTool.point1 = sketch.getPointPos(snapPointID);
        } else {
            arcTool.point1ID = sketch.addPoint(worldPos.x, worldPos.y);
            arcTool.point1 = worldPos;
        }
        arcTool.clickCount = 1;
        return false;
    }

    if (arcTool.clickCount == 1) {
        // Click 2: through-point (just record position, no entity)
        if (snapPointID != NullID) {
            arcTool.point2 = sketch.getPointPos(snapPointID);
        } else {
            arcTool.point2 = worldPos;
        }
        arcTool.point2ID = NullID; // no entity for through-point
        arcTool.clickCount = 2;
        return false;
    }

    // Click 3: end point — compute circumcircle and create arc
    Point2D p1 = arcTool.point1;
    Point2D p2 = arcTool.point2;
    Point2D p3 = worldPos;

    // Circumcircle center from 3 points
    double ax = p1.x, ay = p1.y;
    double bx = p2.x, by = p2.y;
    double cx = p3.x, cy = p3.y;
    double D = 2.0 * (ax * (by - cy) + bx * (cy - ay) + cx * (ay - by));
    if (std::fabs(D) < 1e-6) {
        // Collinear — cancel; clean up freshly created start point if unreferenced
        if (!sketch.isPointReferenced(arcTool.point1ID))
            sketch.removePoint(arcTool.point1ID);
        arcTool.reset();
        return false;
    }

    double a2 = ax * ax + ay * ay;
    double b2 = bx * bx + by * by;
    double c2 = cx * cx + cy * cy;
    double ux = (a2 * (by - cy) + b2 * (cy - ay) + c2 * (ay - by)) / D;
    double uy = (a2 * (cx - bx) + b2 * (ax - cx) + c2 * (bx - ax)) / D;

    // Create center point
    EntityID centerID = sketch.addPoint(ux, uy);

    // Create/snap end point
    EntityID endID;
    if (snapPointID != NullID) {
        endID = snapPointID;
    } else {
        endID = sketch.addPoint(p3.x, p3.y);
    }

    // Create arc — angles computed by addArc from point positions
    EntityID arcID = sketch.addArc(centerID, arcTool.point1ID, endID);

    // Fix sweep direction: the arc should pass through point2
    // Check if CCW sweep from start to end passes near point2
    ArcEntity* arc = sketch.findArc(arcID);
    if (arc) {
        double throughAngle = std::atan2(p2.y - uy, p2.x - ux);
        double sa = arc->startAngle;
        double ea = arc->endAngle;

        // Normalize angles to [0, 2pi)
        auto normAngle = [](double a) {
            constexpr double kTwoPiD = 6.28318530717958647692;
            a = std::fmod(a, kTwoPiD);
            if (a < 0) a += kTwoPiD;
            return a;
        };

        double nsa = normAngle(sa);
        double nea = normAngle(ea);
        double nta = normAngle(throughAngle);

        // CCW sweep from start to end
        constexpr double kTwoPiD = 6.28318530717958647692;
        double ccwSweep = nea - nsa;
        if (ccwSweep <= 0) ccwSweep += kTwoPiD;

        // Check if through-angle is within CCW sweep
        double toThrough = nta - nsa;
        if (toThrough < 0) toThrough += kTwoPiD;

        bool throughInCCW = (toThrough < ccwSweep);
        if (!throughInCCW) {
            // Swap start and end to reverse sweep direction
            EntityID tmp = arc->startPt;
            arc->startPt = arc->endPt;
            arc->endPt = tmp;
            sketch.recomputeArcAngles(*arc);
        }
    }

    arcTool.reset();
    return true;
}

bool handleArcCenterTool(Sketch& sketch, ArcToolState& arcTool, Point2D worldPos, EntityID snapPointID) {
    if (arcTool.clickCount == 0) {
        // Click 1: center point
        if (snapPointID != NullID) {
            arcTool.point1ID = snapPointID;
            arcTool.point1 = sketch.getPointPos(snapPointID);
        } else {
            arcTool.point1ID = sketch.addPoint(worldPos.x, worldPos.y);
            arcTool.point1 = worldPos;
        }
        arcTool.clickCount = 1;
        return false;
    }

    if (arcTool.clickCount == 1) {
        // Click 2: start point (defines radius)
        if (snapPointID != NullID) {
            arcTool.point2ID = snapPointID;
            arcTool.point2 = sketch.getPointPos(snapPointID);
        } else {
            arcTool.point2ID = sketch.addPoint(worldPos.x, worldPos.y);
            arcTool.point2 = worldPos;
        }
        arcTool.clickCount = 2;
        return false;
    }

    // Click 3: end point — project onto circle at locked radius
    double radius = distance(arcTool.point1, arcTool.point2);
    if (radius < 0.001) {
        // Degenerate: clean up freshly created points before resetting
        if (!sketch.isPointReferenced(arcTool.point2ID))
            sketch.removePoint(arcTool.point2ID);
        if (!sketch.isPointReferenced(arcTool.point1ID))
            sketch.removePoint(arcTool.point1ID);
        arcTool.reset();
        return false;
    }

    // Compute end angle: use direction from center toward snap/cursor,
    // then project onto the arc circle so the endpoint is always on-circle.
    double endAngle;
    if (snapPointID != NullID) {
        Point2D sp = sketch.getPointPos(snapPointID);
        endAngle = std::atan2(sp.y - arcTool.point1.y, sp.x - arcTool.point1.x);
    } else {
        endAngle = std::atan2(worldPos.y - arcTool.point1.y, worldPos.x - arcTool.point1.x);
    }
    double ex = arcTool.point1.x + radius * std::cos(endAngle);
    double ey = arcTool.point1.y + radius * std::sin(endAngle);
    EntityID endID = sketch.addPoint(ex, ey);

    sketch.addArc(arcTool.point1ID, arcTool.point2ID, endID);

    arcTool.reset();
    return true;
}

bool handleCenterRectTool(Sketch& sketch, ToolState& tool, Point2D worldPos, EntityID snapPointID) {
    if (!tool.hasFirstPoint) {
        if (snapPointID != NullID) {
            tool.firstPointID = snapPointID;
            tool.firstPoint = sketch.getPointPos(snapPointID);
        } else {
            // Center is a construction reference only — no entity created unless snapping
            // to an existing point. The 4 corners are derived offsets and none reference
            // this position directly, so creating a point here would orphan it.
            tool.firstPointID = NullID;
            tool.firstPoint = worldPos;
        }
        tool.hasFirstPoint = true;
        return false;
    }

    // Create rectangle symmetric around center
    double cx = tool.firstPoint.x, cy = tool.firstPoint.y;
    double dx = worldPos.x - cx, dy = worldPos.y - cy;

    EntityID pA = sketch.addPoint(cx - dx, cy - dy);
    EntityID pB = sketch.addPoint(cx + dx, cy - dy);
    EntityID pC = sketch.addPoint(cx + dx, cy + dy);
    EntityID pD = sketch.addPoint(cx - dx, cy + dy);

    EntityID lAB = sketch.addLine(pA, pB);
    EntityID lBC = sketch.addLine(pB, pC);
    EntityID lCD = sketch.addLine(pC, pD);
    EntityID lDA = sketch.addLine(pD, pA);

    sketch.addConstraint(ConstraintType::Horizontal, lAB, NullID, 0.0f, true);
    sketch.addConstraint(ConstraintType::Horizontal, lCD, NullID, 0.0f, true);
    sketch.addConstraint(ConstraintType::Vertical, lBC, NullID, 0.0f, true);
    sketch.addConstraint(ConstraintType::Vertical, lDA, NullID, 0.0f, true);

    tool.reset();
    return true;
}

bool handlePointTool(Sketch& sketch, Point2D worldPos, EntityID snapPointID) {
    if (snapPointID != NullID) {
        return false; // point already exists at snap location
    }
    sketch.addPoint(worldPos.x, worldPos.y);
    return true;
}

} // namespace shitcad
