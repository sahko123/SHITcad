#include "Canvas.h"
#include "AutoConstraint.h"
#include <cmath>
#include <algorithm>
#include <cstdio>

namespace shitcad {

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
    drawActivePreview(dl, sketch);
    drawSnapIndicator(dl);
    handleInput(sketch);

    dl->PopClipRect();
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
        }
        if (selected) ImGui::PopStyleColor();
    }
    ImGui::PopStyleVar(2);
}

void Canvas::handleInput(Sketch& sketch) {
    ImGuiIO& io = ImGui::GetIO();

    // Undo/redo (global, not just when hovered)
    if (io.KeyCtrl && ImGui::IsKeyPressed(ImGuiKey_Z)) {
        if (io.KeyShift) {
            history_.redo(sketch);
        } else {
            history_.undo(sketch);
        }
        tool_.reset();
    }
    if (io.KeyCtrl && ImGui::IsKeyPressed(ImGuiKey_Y)) {
        history_.redo(sketch);
        tool_.reset();
    }

    if (!ImGui::IsItemHovered())
        return;

    // Tool selection shortcuts
    if (ImGui::IsKeyPressed(ImGuiKey_L)) { tool_.type = ToolType::Line; tool_.reset(); }
    if (ImGui::IsKeyPressed(ImGuiKey_C)) { tool_.type = ToolType::Circle; tool_.reset(); }
    if (ImGui::IsKeyPressed(ImGuiKey_R)) { tool_.type = ToolType::Rectangle; tool_.reset(); }
    if (ImGui::IsKeyPressed(ImGuiKey_Escape)) { tool_.type = ToolType::None; tool_.reset(); }

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

    // Tool action: left click
    if (ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
        Point2D effectivePos = (currentSnap_.type != SnapType::None)
            ? currentSnap_.position : screenToWorld(io.MousePos);
        handleToolAction(sketch, effectivePos);
    }

    // Cancel: right click
    if (ImGui::IsMouseClicked(ImGuiMouseButton_Right)) {
        tool_.reset();
    }
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
    ImU32 lineColor = IM_COL32(255, 255, 255, 255);
    ImU32 pointColor = IM_COL32(255, 255, 255, 255);

    // Draw lines
    for (const auto& line : sketch.lines) {
        const PointEntity* a = sketch.findPoint(line.startPt);
        const PointEntity* b = sketch.findPoint(line.endPt);
        if (!a || !b) continue;

        ImVec2 sa = worldToScreen({a->x, a->y});
        ImVec2 sb = worldToScreen({b->x, b->y});
        dl->AddLine(sa, sb, lineColor, 2.0f);
    }

    // Draw points
    for (const auto& pt : sketch.points) {
        ImVec2 sp = worldToScreen({pt.x, pt.y});
        dl->AddCircleFilled(sp, 3.0f, pointColor);
    }
}

void Canvas::drawCircles(ImDrawList* dl, const Sketch& sketch) {
    ImU32 circleColor = IM_COL32(255, 255, 255, 255);

    for (const auto& circle : sketch.circles) {
        const PointEntity* center = sketch.findPoint(circle.centerPt);
        if (!center) continue;

        ImVec2 screenCenter = worldToScreen({center->x, center->y});
        float screenRadius = circle.radius * zoom_;
        dl->AddCircle(screenCenter, screenRadius, circleColor, 0, 2.0f);

        // Draw center point
        dl->AddCircleFilled(screenCenter, 3.0f, circleColor);
    }
}

void Canvas::drawConstraintIndicators(ImDrawList* dl, const Sketch& sketch) {
    ImU32 constraintColor = IM_COL32(100, 180, 255, 200);

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
            // Draw slightly above the line
            dl->AddText({screenMid.x - 4, screenMid.y - 16}, constraintColor, label);
        }
    }
}

void Canvas::drawActivePreview(ImDrawList* dl, const Sketch& /*sketch*/) {
    if (!tool_.hasFirstPoint)
        return;

    ImVec2 start = worldToScreen(tool_.firstPoint);
    Point2D cursorWorld = (currentSnap_.type != SnapType::None)
        ? currentSnap_.position : screenToWorld(ImGui::GetIO().MousePos);
    ImVec2 end = worldToScreen(cursorWorld);

    ImU32 activeColor = IM_COL32(100, 180, 255, 200);

    switch (tool_.type) {
        case ToolType::Line:
            dl->AddLine(start, end, activeColor, 2.0f);
            dl->AddCircleFilled(start, 3.0f, activeColor);
            break;
        case ToolType::Circle: {
            float screenRadius = distance(tool_.firstPoint, cursorWorld) * zoom_;
            dl->AddCircle(start, screenRadius, activeColor, 0, 2.0f);
            dl->AddCircleFilled(start, 3.0f, activeColor);
            break;
        }
        case ToolType::Rectangle: {
            // Draw rectangle preview
            Point2D p1 = tool_.firstPoint;
            Point2D p2 = cursorWorld;
            ImVec2 a = worldToScreen({p1.x, p1.y});
            ImVec2 b = worldToScreen({p2.x, p1.y});
            ImVec2 c = worldToScreen({p2.x, p2.y});
            ImVec2 d = worldToScreen({p1.x, p2.y});
            dl->AddLine(a, b, activeColor, 2.0f);
            dl->AddLine(b, c, activeColor, 2.0f);
            dl->AddLine(c, d, activeColor, 2.0f);
            dl->AddLine(d, a, activeColor, 2.0f);
            break;
        }
        default:
            break;
    }
}

void Canvas::drawSnapIndicator(ImDrawList* dl) {
    if (currentSnap_.type == SnapType::None) return;

    ImVec2 sp = worldToScreen(currentSnap_.position);
    ImU32 snapColor = IM_COL32(255, 200, 0, 255);

    switch (currentSnap_.type) {
        case SnapType::Grid:
            dl->AddLine({sp.x - 5, sp.y}, {sp.x + 5, sp.y}, snapColor, 1.5f);
            dl->AddLine({sp.x, sp.y - 5}, {sp.x, sp.y + 5}, snapColor, 1.5f);
            break;
        case SnapType::Point:
            dl->AddCircle(sp, 6.0f, snapColor, 0, 2.0f);
            break;
        case SnapType::Midpoint:
            dl->AddTriangleFilled(
                {sp.x, sp.y - 5},
                {sp.x - 5, sp.y + 3},
                {sp.x + 5, sp.y + 3}, snapColor);
            break;
        default:
            break;
    }
}

} // namespace shitcad
