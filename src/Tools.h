#pragma once
#include "SketchData.h"

namespace shitcad {

enum class ToolType : uint8_t {
    None,
    Line,
    Circle,
    Rectangle,
};

struct ToolState {
    ToolType type = ToolType::None;
    bool hasFirstPoint = false;
    EntityID firstPointID = NullID;
    Point2D firstPoint = {};

    void reset() {
        hasFirstPoint = false;
        firstPointID = NullID;
        firstPoint = {};
    }
};

struct SnapResult;

// Tool handler functions. Return true if an action was completed (for undo snapshot).
bool handleLineTool(Sketch& sketch, ToolState& tool, Point2D worldPos, EntityID snapPointID);
bool handleCircleTool(Sketch& sketch, ToolState& tool, Point2D worldPos, EntityID snapPointID);
bool handleRectangleTool(Sketch& sketch, ToolState& tool, Point2D worldPos, EntityID snapPointID);

} // namespace shitcad
