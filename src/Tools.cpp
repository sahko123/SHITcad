#include "Tools.h"

namespace shitcad {

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

    // Prevent zero-length lines (same point clicked twice)
    if (endPtID == tool.firstPointID) return false;
    Point2D endPos = sketch.getPointPos(endPtID);
    if (distance(tool.firstPoint, endPos) < 0.001f) {
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
    float radius = distance(tool.firstPoint, radiusPos);
    if (radius > 0.001f) {
        sketch.addCircle(tool.firstPointID, radius);
    }

    tool.reset();
    return true;
}

bool handleRectangleTool(Sketch& sketch, ToolState& tool, Point2D worldPos, EntityID /*snapPointID*/) {
    if (!tool.hasFirstPoint) {
        tool.firstPoint = worldPos;
        tool.hasFirstPoint = true;
        return false;
    }

    // Create rectangle from two corners
    float x1 = tool.firstPoint.x, y1 = tool.firstPoint.y;
    float x2 = worldPos.x, y2 = worldPos.y;

    // Four corner points: A(x1,y1) B(x2,y1) C(x2,y2) D(x1,y2)
    EntityID pA = sketch.addPoint(x1, y1);
    EntityID pB = sketch.addPoint(x2, y1);
    EntityID pC = sketch.addPoint(x2, y2);
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
    float ax = p1.x, ay = p1.y;
    float bx = p2.x, by = p2.y;
    float cx = p3.x, cy = p3.y;
    float D = 2.0f * (ax * (by - cy) + bx * (cy - ay) + cx * (ay - by));
    if (std::fabs(D) < 1e-6f) {
        // Collinear — cancel
        arcTool.reset();
        return false;
    }

    float a2 = ax * ax + ay * ay;
    float b2 = bx * bx + by * by;
    float c2 = cx * cx + cy * cy;
    float ux = (a2 * (by - cy) + b2 * (cy - ay) + c2 * (ay - by)) / D;
    float uy = (a2 * (cx - bx) + b2 * (ax - cx) + c2 * (bx - ax)) / D;

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
        float throughAngle = std::atan2(p2.y - uy, p2.x - ux);
        float sa = arc->startAngle;
        float ea = arc->endAngle;

        // Normalize angles to [0, 2pi)
        auto normAngle = [](float a) {
            const float kTwoPi = 6.28318530718f;
            a = std::fmod(a, kTwoPi);
            if (a < 0) a += kTwoPi;
            return a;
        };

        float nsa = normAngle(sa);
        float nea = normAngle(ea);
        float nta = normAngle(throughAngle);

        // CCW sweep from start to end
        float ccwSweep = nea - nsa;
        if (ccwSweep <= 0) ccwSweep += 6.28318530718f;

        // Check if through-angle is within CCW sweep
        float toThrough = nta - nsa;
        if (toThrough < 0) toThrough += 6.28318530718f;

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
    float radius = distance(arcTool.point1, arcTool.point2);
    if (radius < 0.001f) {
        arcTool.reset();
        return false;
    }

    float angle = std::atan2(worldPos.y - arcTool.point1.y, worldPos.x - arcTool.point1.x);
    float ex = arcTool.point1.x + radius * std::cos(angle);
    float ey = arcTool.point1.y + radius * std::sin(angle);

    EntityID endID;
    if (snapPointID != NullID) {
        endID = snapPointID;
    } else {
        endID = sketch.addPoint(ex, ey);
    }

    sketch.addArc(arcTool.point1ID, arcTool.point2ID, endID);

    arcTool.reset();
    return true;
}

bool handleCenterRectTool(Sketch& sketch, ToolState& tool, Point2D worldPos, EntityID /*snapPointID*/) {
    if (!tool.hasFirstPoint) {
        tool.firstPoint = worldPos;
        tool.hasFirstPoint = true;
        return false;
    }

    // Create rectangle symmetric around center
    float cx = tool.firstPoint.x, cy = tool.firstPoint.y;
    float dx = worldPos.x - cx, dy = worldPos.y - cy;

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
