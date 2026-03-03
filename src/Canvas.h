#pragma once
#include "SketchData.h"
#include "Tools.h"
#include "Snap.h"
#include "History.h"
#include "Solver.h"
#include "HitTest.h"
#include "Selection.h"
#include <imgui.h>

namespace shitcad {

class Canvas {
public:
    void draw(Sketch& sketch);

    ImVec2 worldToScreen(Point2D world) const;
    Point2D screenToWorld(ImVec2 screen) const;

    History& history() { return history_; }
    ToolState& toolState() { return tool_; }

private:
    ImVec2 viewOffset_ = {0.0f, 0.0f};
    float zoom_ = 40.0f;

    ImVec2 canvasOrigin_ = {0.0f, 0.0f};
    ImVec2 canvasSize_ = {0.0f, 0.0f};
    float currentGridStep_ = 1.0f;

    ToolState tool_;
    SnapEngine snapEngine_;
    History history_;
    Solver solver_;
    SnapResult currentSnap_;
    SelectionState selection_;

    // Dimension input state
    bool dimInputActive_ = false;
    char dimInputBuf_[32] = {};
    bool dimInputFocusNeeded_ = false;

    void handleInput(Sketch& sketch);
    void handleToolAction(Sketch& sketch, Point2D worldPos);
    void handleSelection(Sketch& sketch);
    void handleDrag(Sketch& sketch);
    void handleDeletion(Sketch& sketch);
    void handleDimensionInput(Sketch& sketch);

    void drawGrid(ImDrawList* dl);
    void drawGeometry(ImDrawList* dl, const Sketch& sketch);
    void drawCircles(ImDrawList* dl, const Sketch& sketch);
    void drawConstraintIndicators(ImDrawList* dl, const Sketch& sketch);
    void drawDimensions(ImDrawList* dl, const Sketch& sketch);
    void drawActivePreview(ImDrawList* dl, const Sketch& sketch);
    void drawSnapIndicator(ImDrawList* dl);
    void drawToolbar(ImDrawList* dl);
    void drawDimensionInputBox(Sketch& sketch);

    // Color helpers
    ImU32 getLineColor(EntityID lineID) const;
    ImU32 getPointColor(EntityID pointID) const;
    ImU32 getCircleColor(EntityID circleID) const;
};

} // namespace shitcad
