#include "Canvas.h"
#include "AutoConstraint.h"
#include <cmath>
#include <algorithm>
#include <cstdio>
#include <cstring>
#include <cstdlib>

namespace shitcad {

static const ImU32 kColorWhite       = IM_COL32(255, 255, 255, 255);
static const ImU32 kColorSelected    = IM_COL32(255, 165, 0, 255);   // orange
static const ImU32 kColorActive      = IM_COL32(100, 180, 255, 200);
static const ImU32 kColorConstraint  = IM_COL32(100, 180, 255, 200);
static const ImU32 kColorDimension   = IM_COL32(80, 220, 80, 255);   // green
static const ImU32 kColorSnap        = IM_COL32(255, 200, 0, 255);

ImVec2 Canvas::worldToScreen(Point2D world) const {
    float cx = canvasOrigin_.x + canvasSize_.x * 0.5f + viewOffset_.x;
    float cy = canvasOrigin_.y + canvasSize_.y * 0.5f + viewOffset_.y;
    return {cx + world.x * zoom_, cy - world.y * zoom_};
}

Point2D Canvas::screenToWorld(ImVec2 screen) const {
    float cx = canvasOrigin_.x + canvasSize_.x * 0.5f + viewOffset_.x;
    float cy = canvasOrigin_.y + canvasSize_.y * 0.5f + viewOffset_.y;
    return {(screen.x - cx) / zoom_, -(screen.y - cy) / zoom_};
}

ImU32 Canvas::getLineColor(EntityID lineID) const {
    return (selection_.isLineSelected() && selection_.entityID == lineID)
        ? kColorSelected : kColorWhite;
}

ImU32 Canvas::getPointColor(EntityID pointID) const {
    return (selection_.isPointSelected() && selection_.entityID == pointID)
        ? kColorSelected : kColorWhite;
}

ImU32 Canvas::getCircleColor(EntityID circleID) const {
    return (selection_.isCircleSelected() && selection_.entityID == circleID)
        ? kColorSelected : kColorWhite;
}

void Canvas::draw(Sketch& sketch) {
    // Toolbar at top
    ImGui::BeginChild("toolbar", {0, 30}, false, ImGuiWindowFlags_NoScrollbar);
    drawToolbar(nullptr);
    ImGui::EndChild();

    ImVec2 avail = ImGui::GetContentRegionAvail();
    canvasOrigin_ = ImGui::GetCursorScreenPos();
    canvasSize_ = avail;

    ImGui::InvisibleButton("canvas", avail,
        ImGuiButtonFlags_MouseButtonLeft |
        ImGuiButtonFlags_MouseButtonRight |
        ImGuiButtonFlags_MouseButtonMiddle);

    // Compute grid step for snap
    float targetScreenSpacing = 50.0f;
    float worldSpacing = targetScreenSpacing / zoom_;
    float logSpacing = std::floor(std::log10(worldSpacing));
    currentGridStep_ = std::pow(10.0f, logSpacing);
    if (currentGridStep_ * zoom_ < 10.0f)
        currentGridStep_ *= 10.0f;

    // Compute snap
    Point2D rawCursor = screenToWorld(ImGui::GetIO().MousePos);
    currentSnap_ = snapEngine_.snap(rawCursor, zoom_, sketch, currentGridStep_);

    ImDrawList* dl = ImGui::GetWindowDrawList();
    dl->PushClipRect(canvasOrigin_,
        {canvasOrigin_.x + canvasSize_.x, canvasOrigin_.y + canvasSize_.y}, true);

    drawGrid(dl);
    drawGeometry(dl, sketch);
    drawCircles(dl, sketch);
    drawConstraintIndicators(dl, sketch);
    drawDimensions(dl, sketch);
    drawActivePreview(dl, sketch);
    drawSnapIndicator(dl);
    handleInput(sketch);

    dl->PopClipRect();

    // Dimension input popup (drawn outside clip rect)
    if (dimInputActive_) {
        drawDimensionInputBox(sketch);
    }
}

void Canvas::drawToolbar(ImDrawList*) {
    const char* toolNames[] = {"[None]", "[L]ine", "[C]ircle", "[R]ect"};
    ToolType toolTypes[] = {ToolType::None, ToolType::Line, ToolType::Circle, ToolType::Rectangle};

    ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, {4, 0});
    ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, {8, 4});
    for (int i = 0; i < 4; i++) {
        if (i > 0) ImGui::SameLine();
        bool selected = (tool_.type == toolTypes[i]);
        if (selected) ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.3f, 0.5f, 0.8f, 1.0f));
        if (ImGui::Button(toolNames[i])) {
            tool_.type = toolTypes[i];
            tool_.reset();
            selection_.clear();
        }
        if (selected) ImGui::PopStyleColor();
    }
    ImGui::PopStyleVar(2);
}

void Canvas::handleInput(Sketch& sketch) {
    ImGuiIO& io = ImGui::GetIO();

    // Undo/redo (global)
    if (!dimInputActive_ && io.KeyCtrl && ImGui::IsKeyPressed(ImGuiKey_Z)) {
        if (io.KeyShift) {
            history_.redo(sketch);
        } else {
            history_.undo(sketch);
        }
        tool_.reset();
        selection_.clear();
    }
    if (!dimInputActive_ && io.KeyCtrl && ImGui::IsKeyPressed(ImGuiKey_Y)) {
        history_.redo(sketch);
        tool_.reset();
        selection_.clear();
    }

    if (!ImGui::IsItemHovered())
        return;

    // Don't handle most input while dimension input is active
    if (dimInputActive_)
        return;

    // Tool selection shortcuts
    if (ImGui::IsKeyPressed(ImGuiKey_L)) { tool_.type = ToolType::Line; tool_.reset(); selection_.clear(); }
    if (ImGui::IsKeyPressed(ImGuiKey_C)) { tool_.type = ToolType::Circle; tool_.reset(); selection_.clear(); }
    if (ImGui::IsKeyPressed(ImGuiKey_R)) { tool_.type = ToolType::Rectangle; tool_.reset(); selection_.clear(); }
    if (ImGui::IsKeyPressed(ImGuiKey_Escape)) {
        if (tool_.hasFirstPoint) {
            tool_.reset(); // cancel in-progress tool first
        } else {
            tool_.type = ToolType::None;
            tool_.reset();
            selection_.clear();
        }
    }

    // Dimension input shortcut
    if (ImGui::IsKeyPressed(ImGuiKey_D) && selection_.hasSelection()) {
        handleDimensionInput(sketch);
    }

    // Delete selected entity
    if (ImGui::IsKeyPressed(ImGuiKey_Delete) || ImGui::IsKeyPressed(ImGuiKey_Backspace)) {
        handleDeletion(sketch);
    }

    // Pan: middle mouse drag
    if (ImGui::IsMouseDragging(ImGuiMouseButton_Middle, 0.0f)) {
        ImVec2 delta = io.MouseDelta;
        viewOffset_.x += delta.x;
        viewOffset_.y += delta.y;
    }

    // Zoom: scroll wheel, centered on cursor
    if (io.MouseWheel != 0.0f) {
        float zoomFactor = std::pow(1.15f, io.MouseWheel);
        ImVec2 mouseScreen = io.MousePos;
        Point2D worldBefore = screenToWorld(mouseScreen);

        zoom_ *= zoomFactor;
        zoom_ = std::clamp(zoom_, 0.1f, 10000.0f);

        ImVec2 screenAfter = worldToScreen(worldBefore);
        viewOffset_.x += mouseScreen.x - screenAfter.x;
        viewOffset_.y += mouseScreen.y - screenAfter.y;
    }

    // Handle dragging of selected points
    if (selection_.isPointSelected() && selection_.isDragging) {
        handleDrag(sketch);
    }

    // Left click
    if (ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
        if (tool_.type != ToolType::None) {
            // Tool action
            Point2D effectivePos = (currentSnap_.type != SnapType::None)
                ? currentSnap_.position : screenToWorld(io.MousePos);
            handleToolAction(sketch, effectivePos);
        } else {
            // Selection mode
            handleSelection(sketch);
        }
    }

    // Start drag on selected point
    if (selection_.isPointSelected() && !selection_.isDragging &&
        ImGui::IsMouseDragging(ImGuiMouseButton_Left, 3.0f)) {
        // Check if drag started near the selected point
        Point2D ptPos = sketch.getPointPos(selection_.entityID);
        ImVec2 ptScreen = worldToScreen(ptPos);
        ImVec2 mousePos = io.MousePos;
        float screenDist = std::sqrt(
            (ptScreen.x - mousePos.x) * (ptScreen.x - mousePos.x) +
            (ptScreen.y - mousePos.y) * (ptScreen.y - mousePos.y));
        if (screenDist < 20.0f) {
            selection_.isDragging = true;
            selection_.dragStarted = false;
        }
    }

    // End drag on mouse release
    if (selection_.isDragging && ImGui::IsMouseReleased(ImGuiMouseButton_Left)) {
        if (selection_.dragStarted) {
            history_.pushState(sketch);
        }
        selection_.isDragging = false;
        selection_.dragStarted = false;
    }

    // Cancel: right click
    if (ImGui::IsMouseClicked(ImGuiMouseButton_Right)) {
        if (tool_.hasFirstPoint) {
            tool_.reset();
        } else {
            selection_.clear();
        }
    }
}

void Canvas::handleSelection(Sketch& sketch) {
    Point2D cursorWorld = screenToWorld(ImGui::GetIO().MousePos);
    HitResult hit = hitTest(cursorWorld, zoom_, sketch, 10.0f);

    if (hit.type != HitType::None) {
        selection_.select(hit.type, hit.entityID);
    } else {
        selection_.clear();
    }
}

void Canvas::handleDrag(Sketch& sketch) {
    ImGuiIO& io = ImGui::GetIO();
    Point2D cursorWorld = (currentSnap_.type != SnapType::None)
        ? currentSnap_.position : screenToWorld(io.MousePos);

    PointEntity* pt = sketch.findPoint(selection_.entityID);
    if (!pt) return;

    pt->x = cursorWorld.x;
    pt->y = cursorWorld.y;
    selection_.dragStarted = true;

    // Live constraint solving
    solver_.solve(sketch, selection_.entityID);
}

void Canvas::handleDeletion(Sketch& sketch) {
    if (!selection_.hasSelection()) return;

    switch (selection_.type) {
        case HitType::Point:
            sketch.removePoint(selection_.entityID);
            break;
        case HitType::Line:
            sketch.removeLine(selection_.entityID);
            break;
        case HitType::Circle:
            sketch.removeCircle(selection_.entityID);
            break;
        default:
            return;
    }

    selection_.clear();
    history_.pushState(sketch);
}

void Canvas::handleDimensionInput(Sketch& sketch) {
    if (selection_.isLineSelected()) {
        // Get current line length for default value
        LineEntity* line = sketch.findLine(selection_.entityID);
        if (!line) return;
        Point2D a = sketch.getPointPos(line->startPt);
        Point2D b = sketch.getPointPos(line->endPt);
        float len = distance(a, b);
        snprintf(dimInputBuf_, sizeof(dimInputBuf_), "%.3f", len);
        dimInputActive_ = true;
        dimInputFocusNeeded_ = true;
    } else if (selection_.isCircleSelected()) {
        CircleEntity* circle = sketch.findCircle(selection_.entityID);
        if (!circle) return;
        snprintf(dimInputBuf_, sizeof(dimInputBuf_), "%.3f", circle->radius);
        dimInputActive_ = true;
        dimInputFocusNeeded_ = true;
    }
}

void Canvas::drawDimensionInputBox(Sketch& sketch) {
    // Position the input near the selected entity
    ImVec2 inputPos = {canvasOrigin_.x + canvasSize_.x * 0.5f,
                       canvasOrigin_.y + canvasSize_.y * 0.5f};

    if (selection_.isLineSelected()) {
        LineEntity* line = sketch.findLine(selection_.entityID);
        if (line) {
            Point2D a = sketch.getPointPos(line->startPt);
            Point2D b = sketch.getPointPos(line->endPt);
            Point2D mid = midpoint(a, b);
            inputPos = worldToScreen(mid);
            inputPos.y -= 30.0f;
        }
    } else if (selection_.isCircleSelected()) {
        CircleEntity* circle = sketch.findCircle(selection_.entityID);
        if (circle) {
            Point2D center = sketch.getPointPos(circle->centerPt);
            inputPos = worldToScreen(center);
            inputPos.y -= 30.0f;
        }
    }

    ImGui::SetNextWindowPos(inputPos, ImGuiCond_Always, {0.5f, 1.0f});
    ImGui::SetNextWindowSize({120, 0});
    ImGui::Begin("##dimin", nullptr,
        ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize |
        ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoScrollbar |
        ImGuiWindowFlags_AlwaysAutoResize);

    if (dimInputFocusNeeded_) {
        ImGui::SetKeyboardFocusHere();
        dimInputFocusNeeded_ = false;
    }

    bool entered = ImGui::InputText("##dimval", dimInputBuf_, sizeof(dimInputBuf_),
        ImGuiInputTextFlags_EnterReturnsTrue | ImGuiInputTextFlags_AutoSelectAll);

    if (entered) {
        float value = (float)atof(dimInputBuf_);
        if (value > 0.001f) {
            if (selection_.isLineSelected()) {
                // Add or update distance constraint on this line
                sketch.addConstraint(ConstraintType::Distance,
                    selection_.entityID, NullID, value, false);
                solver_.solve(sketch);
                history_.pushState(sketch);
            } else if (selection_.isCircleSelected()) {
                // Set circle radius directly
                CircleEntity* circle = sketch.findCircle(selection_.entityID);
                if (circle) {
                    circle->radius = value;
                    history_.pushState(sketch);
                }
            }
        }
        dimInputActive_ = false;
    }

    if (ImGui::IsKeyPressed(ImGuiKey_Escape)) {
        dimInputActive_ = false;
    }

    ImGui::End();
}

void Canvas::handleToolAction(Sketch& sketch, Point2D worldPos) {
    EntityID snapPtID = currentSnap_.pointID;
    bool actionCompleted = false;

    switch (tool_.type) {
        case ToolType::Line:
            actionCompleted = handleLineTool(sketch, tool_, worldPos, snapPtID);
            break;
        case ToolType::Circle:
            actionCompleted = handleCircleTool(sketch, tool_, worldPos, snapPtID);
            break;
        case ToolType::Rectangle:
            actionCompleted = handleRectangleTool(sketch, tool_, worldPos, snapPtID);
            break;
        case ToolType::None:
            break;
    }

    if (actionCompleted) {
        // Auto-constraints for lines
        if (tool_.type == ToolType::Line && !sketch.lines.empty()) {
            EntityID lastLineID = sketch.lines.back().id;
            auto pending = detectLineAutoConstraints(sketch, lastLineID);
            for (const auto& pc : pending) {
                sketch.addConstraint(pc.type, pc.entityA, pc.entityB, pc.value, true);
            }
        }

        // Run constraint solver
        solver_.solve(sketch);

        history_.pushState(sketch);
    }
}

void Canvas::drawGrid(ImDrawList* dl) {
    Point2D topLeft = screenToWorld(canvasOrigin_);
    Point2D bottomRight = screenToWorld(
        {canvasOrigin_.x + canvasSize_.x, canvasOrigin_.y + canvasSize_.y});

    float worldLeft = std::min(topLeft.x, bottomRight.x);
    float worldRight = std::max(topLeft.x, bottomRight.x);
    float worldBottom = std::min(topLeft.y, bottomRight.y);
    float worldTop = std::max(topLeft.y, bottomRight.y);

    ImU32 minorColor = IM_COL32(40, 40, 40, 255);
    ImU32 majorColor = IM_COL32(70, 70, 70, 255);

    float startX = std::floor(worldLeft / currentGridStep_) * currentGridStep_;
    float startY = std::floor(worldBottom / currentGridStep_) * currentGridStep_;

    for (float x = startX; x <= worldRight; x += currentGridStep_) {
        ImVec2 top = worldToScreen({x, worldTop});
        ImVec2 bot = worldToScreen({x, worldBottom});
        bool isMajor = std::fabs(std::fmod(x, currentGridStep_ * 10.0f)) < currentGridStep_ * 0.5f;
        dl->AddLine(top, bot, isMajor ? majorColor : minorColor, 1.0f);
    }

    for (float y = startY; y <= worldTop; y += currentGridStep_) {
        ImVec2 left = worldToScreen({worldLeft, y});
        ImVec2 right = worldToScreen({worldRight, y});
        bool isMajor = std::fabs(std::fmod(y, currentGridStep_ * 10.0f)) < currentGridStep_ * 0.5f;
        dl->AddLine(left, right, isMajor ? majorColor : minorColor, 1.0f);
    }

    ImVec2 originScreen = worldToScreen({0.0f, 0.0f});
    dl->AddLine(
        {canvasOrigin_.x, originScreen.y},
        {canvasOrigin_.x + canvasSize_.x, originScreen.y},
        IM_COL32(180, 50, 50, 200), 1.5f);
    dl->AddLine(
        {originScreen.x, canvasOrigin_.y},
        {originScreen.x, canvasOrigin_.y + canvasSize_.y},
        IM_COL32(50, 180, 50, 200), 1.5f);
}

void Canvas::drawGeometry(ImDrawList* dl, const Sketch& sketch) {
    // Draw lines
    for (const auto& line : sketch.lines) {
        const PointEntity* a = sketch.findPoint(line.startPt);
        const PointEntity* b = sketch.findPoint(line.endPt);
        if (!a || !b) continue;

        ImVec2 sa = worldToScreen({a->x, a->y});
        ImVec2 sb = worldToScreen({b->x, b->y});
        ImU32 color = getLineColor(line.id);
        float thickness = (selection_.isLineSelected() && selection_.entityID == line.id) ? 3.0f : 2.0f;
        dl->AddLine(sa, sb, color, thickness);
    }

    // Draw points
    for (const auto& pt : sketch.points) {
        ImVec2 sp = worldToScreen({pt.x, pt.y});
        ImU32 color = getPointColor(pt.id);
        float radius = (selection_.isPointSelected() && selection_.entityID == pt.id) ? 5.0f : 3.0f;
        dl->AddCircleFilled(sp, radius, color);
    }
}

void Canvas::drawCircles(ImDrawList* dl, const Sketch& sketch) {
    for (const auto& circle : sketch.circles) {
        const PointEntity* center = sketch.findPoint(circle.centerPt);
        if (!center) continue;

        ImVec2 screenCenter = worldToScreen({center->x, center->y});
        float screenRadius = circle.radius * zoom_;
        ImU32 color = getCircleColor(circle.id);
        float thickness = (selection_.isCircleSelected() && selection_.entityID == circle.id) ? 3.0f : 2.0f;
        dl->AddCircle(screenCenter, screenRadius, color, 0, thickness);
        dl->AddCircleFilled(screenCenter, 3.0f, color);
    }
}

void Canvas::drawConstraintIndicators(ImDrawList* dl, const Sketch& sketch) {
    for (const auto& c : sketch.constraints) {
        if (c.type == ConstraintType::Horizontal || c.type == ConstraintType::Vertical) {
            const LineEntity* line = nullptr;
            for (const auto& l : sketch.lines) {
                if (l.id == c.entityA) { line = &l; break; }
            }
            if (!line) continue;

            Point2D a = sketch.getPointPos(line->startPt);
            Point2D b = sketch.getPointPos(line->endPt);
            Point2D mid = midpoint(a, b);
            ImVec2 screenMid = worldToScreen(mid);

            const char* label = (c.type == ConstraintType::Horizontal) ? "H" : "V";
            dl->AddText({screenMid.x - 4, screenMid.y - 16}, kColorConstraint, label);
        }
    }
}

void Canvas::drawDimensions(ImDrawList* dl, const Sketch& sketch) {
    for (const auto& c : sketch.constraints) {
        if (c.type != ConstraintType::Distance) continue;

        // Distance constraint on a line
        const LineEntity* line = nullptr;
        for (const auto& l : sketch.lines) {
            if (l.id == c.entityA) { line = &l; break; }
        }
        if (!line) continue;

        Point2D a = sketch.getPointPos(line->startPt);
        Point2D b = sketch.getPointPos(line->endPt);
        Point2D mid = midpoint(a, b);

        // Offset the dimension slightly from the line
        float dx = b.x - a.x;
        float dy = b.y - a.y;
        float len = std::sqrt(dx * dx + dy * dy);
        if (len < 1e-6f) continue;

        // Perpendicular offset (10 pixels worth)
        float perpX = -dy / len;
        float perpY = dx / len;
        float offsetWorld = 15.0f / zoom_;
        Point2D dimPos = {mid.x + perpX * offsetWorld, mid.y + perpY * offsetWorld};

        ImVec2 screenDim = worldToScreen(dimPos);
        ImVec2 screenA = worldToScreen(a);
        ImVec2 screenB = worldToScreen(b);
        ImVec2 screenMid = worldToScreen(mid);

        // Draw dimension line
        ImVec2 dimA = {screenA.x + perpX * 15.0f, screenA.y - perpY * 15.0f};
        ImVec2 dimB = {screenB.x + perpX * 15.0f, screenB.y - perpY * 15.0f};
        dl->AddLine(dimA, dimB, kColorDimension, 1.0f);

        // Extension lines
        dl->AddLine(screenA, dimA, kColorDimension, 1.0f);
        dl->AddLine(screenB, dimB, kColorDimension, 1.0f);

        // Value text
        char buf[32];
        snprintf(buf, sizeof(buf), "%.2f", c.value);
        dl->AddText(screenDim, kColorDimension, buf);
    }
}

void Canvas::drawActivePreview(ImDrawList* dl, const Sketch& /*sketch*/) {
    if (!tool_.hasFirstPoint)
        return;

    ImVec2 start = worldToScreen(tool_.firstPoint);
    Point2D cursorWorld = (currentSnap_.type != SnapType::None)
        ? currentSnap_.position : screenToWorld(ImGui::GetIO().MousePos);
    ImVec2 end = worldToScreen(cursorWorld);

    switch (tool_.type) {
        case ToolType::Line:
            dl->AddLine(start, end, kColorActive, 2.0f);
            dl->AddCircleFilled(start, 3.0f, kColorActive);
            break;
        case ToolType::Circle: {
            float screenRadius = distance(tool_.firstPoint, cursorWorld) * zoom_;
            dl->AddCircle(start, screenRadius, kColorActive, 0, 2.0f);
            dl->AddCircleFilled(start, 3.0f, kColorActive);
            break;
        }
        case ToolType::Rectangle: {
            Point2D p1 = tool_.firstPoint;
            Point2D p2 = cursorWorld;
            ImVec2 a = worldToScreen({p1.x, p1.y});
            ImVec2 b = worldToScreen({p2.x, p1.y});
            ImVec2 c = worldToScreen({p2.x, p2.y});
            ImVec2 d = worldToScreen({p1.x, p2.y});
            dl->AddLine(a, b, kColorActive, 2.0f);
            dl->AddLine(b, c, kColorActive, 2.0f);
            dl->AddLine(c, d, kColorActive, 2.0f);
            dl->AddLine(d, a, kColorActive, 2.0f);
            break;
        }
        default:
            break;
    }
}

void Canvas::drawSnapIndicator(ImDrawList* dl) {
    if (currentSnap_.type == SnapType::None) return;

    ImVec2 sp = worldToScreen(currentSnap_.position);

    switch (currentSnap_.type) {
        case SnapType::Grid:
            dl->AddLine({sp.x - 5, sp.y}, {sp.x + 5, sp.y}, kColorSnap, 1.5f);
            dl->AddLine({sp.x, sp.y - 5}, {sp.x, sp.y + 5}, kColorSnap, 1.5f);
            break;
        case SnapType::Point:
            dl->AddCircle(sp, 6.0f, kColorSnap, 0, 2.0f);
            break;
        case SnapType::Midpoint:
            dl->AddTriangleFilled(
                {sp.x, sp.y - 5},
                {sp.x - 5, sp.y + 3},
                {sp.x + 5, sp.y + 3}, kColorSnap);
            break;
        default:
            break;
    }
}

} // namespace shitcad
