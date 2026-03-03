#include "Tools.h"

namespace shitcad {

bool handleLineTool(Sketch& sketch, ToolState& tool, Point2D worldPos, EntityID snapPointID) {
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
        return false; // not a complete action yet
    }

    // Place second point and create line
    EntityID endPtID;
    if (snapPointID != NullID) {
        endPtID = snapPointID;
    } else {
        endPtID = sketch.addPoint(worldPos.x, worldPos.y);
    }

    sketch.addLine(tool.firstPointID, endPtID);

    // Continuous mode: end becomes start of next line
    tool.firstPointID = endPtID;
    tool.firstPoint = sketch.getPointPos(endPtID);

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

    // Compute radius from center to click position
    float radius = distance(tool.firstPoint, worldPos);
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

} // namespace shitcad
