#pragma once
#include "SketchData.h"

namespace shitcad {

enum class ToolType : uint8_t {
    None,
    Point,
    Line,
    Circle,
    Rectangle,
    Arc3Point,
    ArcCenter,
    CenterRect,
    Dimension,
    Fillet,
    Extrude,
    Revolve,
    Loft,
    BooleanUnion,
    BooleanSubtract,
};

struct ToolState {
    ToolType type = ToolType::None;
    bool hasFirstPoint = false;
    EntityID firstPointID = NullID;
    Point2D firstPoint = {};
    EntityID tangentSourceID = NullID; // circle/arc ID for auto-tangent

    // Inline dimension input (active during tool use)
    bool inlineInputActive = false;
    char inlineInputBuf[64] = {};
    bool inlineInputFocus = false; // request focus on next frame

    void reset() {
        hasFirstPoint = false;
        firstPointID = NullID;
        firstPoint = {};
        tangentSourceID = NullID;
        inlineInputActive = false;
        inlineInputBuf[0] = '\0';
        inlineInputFocus = false;
    }
};

struct ArcToolState {
    int clickCount = 0;
    EntityID point1ID = NullID;
    Point2D point1 = {};
    EntityID point2ID = NullID;
    Point2D point2 = {};
    void reset() { *this = {}; }
};

struct FilletToolState {
    EntityID entity1ID = NullID;  // first entity at the corner vertex
    EntityID entity2ID = NullID;  // second entity at the corner vertex
    bool entity1IsArc = false;
    bool entity2IsArc = false;
    void reset() { *this = {}; }
};

struct SnapResult;

// Tool handler functions. Return true if an action was completed (for undo snapshot).
bool handleLineTool(Sketch& sketch, ToolState& tool, Point2D worldPos, EntityID snapPointID, EntityID snapCurveID = NullID);
bool handleCircleTool(Sketch& sketch, ToolState& tool, Point2D worldPos, EntityID snapPointID);
bool handleRectangleTool(Sketch& sketch, ToolState& tool, Point2D worldPos, EntityID snapPointID);
bool handleArc3PointTool(Sketch& sketch, ArcToolState& arcTool, Point2D worldPos, EntityID snapPointID);
bool handleArcCenterTool(Sketch& sketch, ArcToolState& arcTool, Point2D worldPos, EntityID snapPointID);
bool handleCenterRectTool(Sketch& sketch, ToolState& tool, Point2D worldPos, EntityID snapPointID);
bool handlePointTool(Sketch& sketch, Point2D worldPos, EntityID snapPointID);

// Fillet: click on vertex to select it (returns true if exactly 2 entities found).
bool handleFilletVertexClick(const Sketch& sketch, FilletToolState& filletTool, EntityID vertexID);
// Apply fillet geometry. Returns true on success.
bool applyFillet(Sketch& sketch, FilletToolState& filletTool, EntityID vertexID, double radius);

} // namespace shitcad
