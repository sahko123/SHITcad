#include "App.h"
#include "ProfileDetector.h"
#include "AutoConstraint.h"

#include <glad/gl.h>
#include <GLFW/glfw3.h>
#include <imgui.h>
#include <imgui_internal.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>
#include "UnitUtils.h"
#include <cstdio>
#include <cmath>
#include <algorithm>
#include <string>
#include <unordered_map>

namespace shitcad {

// Helper: render a two-point dimension (line distance / point-to-point)
struct DimLabelRectOut {
    EntityID constraintID; int planeIndex;
    float x0, y0, x1, y1;
};

static void renderTwoPointDim(ImDrawList* dl, const SketchPlane& sp, const Constraint& c,
    Point2D a2d, Point2D b2d, const char* prefix,
    const float view[16], const float proj[16], float vpW, float vpH,
    const Theme& theme, float arrowLen, float arrowWidth, float extGap, float extOvershoot,
    float pxPerLocal,
    bool selected = false, std::vector<DimLabelRectOut>* labelRects = nullptr, int planeIndex = -1) {

    // All geometry is computed in sketch-plane local 2D, then projected to screen.
    double ldx = b2d.x - a2d.x, ldy = b2d.y - a2d.y;
    double llen = std::sqrt(ldx * ldx + ldy * ldy);
    if (llen < 1e-6) return;

    double lux = ldx / llen, luy = ldy / llen; // unit along measurement (local)
    double lpx = -luy, lpy = lux;              // perpendicular (local)

    // Convert screen-pixel sizes to local units
    float s = (pxPerLocal > 1e-6f) ? pxPerLocal : 1.0f;
    double arrowLenL = arrowLen / s;
    double arrowWidthL = arrowWidth / s;
    double extGapL = extGap / s;
    double extOverL = extOvershoot / s;

    // Perpendicular offset in local coords
    bool hasPlacement = (c.dimOffsetX != 0 || c.dimOffsetY != 0);
    double offsetLocal;
    if (hasPlacement) {
        offsetLocal = c.dimOffsetX * lpx + c.dimOffsetY * lpy;
    } else {
        offsetLocal = 22.0 / s;
    }

    // Dimension line endpoints in local space
    double dax = a2d.x + lpx * offsetLocal, day = a2d.y + lpy * offsetLocal;
    double dbx = b2d.x + lpx * offsetLocal, dby = b2d.y + lpy * offsetLocal;

    // Extension lines in local space
    double signExt = (offsetLocal >= 0) ? 1.0 : -1.0;
    double ea0x = a2d.x + lpx * extGapL * signExt,  ea0y = a2d.y + lpy * extGapL * signExt;
    double ea1x = a2d.x + lpx * (offsetLocal + extOverL * signExt), ea1y = a2d.y + lpy * (offsetLocal + extOverL * signExt);
    double eb0x = b2d.x + lpx * extGapL * signExt,  eb0y = b2d.y + lpy * extGapL * signExt;
    double eb1x = b2d.x + lpx * (offsetLocal + extOverL * signExt), eb1y = b2d.y + lpy * (offsetLocal + extOverL * signExt);

    // Arrow geometry in local space
    double aA1x = dax + lux*arrowLenL + lpx*arrowWidthL, aA1y = day + luy*arrowLenL + lpy*arrowWidthL;
    double aA2x = dax + lux*arrowLenL - lpx*arrowWidthL, aA2y = day + luy*arrowLenL - lpy*arrowWidthL;
    double aB1x = dbx - lux*arrowLenL + lpx*arrowWidthL, aB1y = dby - luy*arrowLenL + lpy*arrowWidthL;
    double aB2x = dbx - lux*arrowLenL - lpx*arrowWidthL, aB2y = dby - luy*arrowLenL - lpy*arrowWidthL;

    // Project all local-space points to screen
    auto toScr = [&](double lx, double ly, float& sx, float& sy) -> bool {
        float w[3];
        sp.localToWorld(f(lx), f(ly), w[0], w[1], w[2]);
        return worldToScreen(w, view, proj, vpW, vpH, sx, sy);
    };

    float sda[2], sdb[2], sea0[2], sea1[2], seb0[2], seb1[2];
    float saA1[2], saA2[2], saB1[2], saB2[2];
    if (!toScr(dax, day, sda[0], sda[1])) return;
    if (!toScr(dbx, dby, sdb[0], sdb[1])) return;
    if (!toScr(ea0x, ea0y, sea0[0], sea0[1])) return;
    if (!toScr(ea1x, ea1y, sea1[0], sea1[1])) return;
    if (!toScr(eb0x, eb0y, seb0[0], seb0[1])) return;
    if (!toScr(eb1x, eb1y, seb1[0], seb1[1])) return;
    if (!toScr(aA1x, aA1y, saA1[0], saA1[1])) return;
    if (!toScr(aA2x, aA2y, saA2[0], saA2[1])) return;
    if (!toScr(aB1x, aB1y, saB1[0], saB1[1])) return;
    if (!toScr(aB2x, aB2y, saB2[0], saB2[1])) return;

    ImU32 dimColor = c.driven ? theme.dimDrivenLineColor : theme.dimLineColor;
    ImU32 textColor = c.driven ? theme.dimDrivenTextColor : theme.dimTextColor;

    // Extension lines
    dl->AddLine({sea0[0], sea0[1]}, {sea1[0], sea1[1]}, dimColor, 1.0f);
    dl->AddLine({seb0[0], seb0[1]}, {seb1[0], seb1[1]}, dimColor, 1.0f);
    // Dimension line
    dl->AddLine({sda[0], sda[1]}, {sdb[0], sdb[1]}, dimColor, 1.0f);
    // Arrows
    dl->AddTriangleFilled({sda[0], sda[1]}, {saA1[0], saA1[1]}, {saA2[0], saA2[1]}, dimColor);
    dl->AddTriangleFilled({sdb[0], sdb[1]}, {saB1[0], saB1[1]}, {saB2[0], saB2[1]}, dimColor);

    // Text at midpoint of dimension line (screen-aligned for readability)
    char buf[80];
    char valBuf[64];
    if (c.driven) {
        // Show the actual current measurement so driven dims always reflect geometry state.
        // llen is the distance between the two geometry points passed in (already in mm).
        formatDimensionText(valBuf, sizeof(valBuf), f(llen), "", 0.0f);
        snprintf(buf, sizeof(buf), "(%s%s)", prefix, valBuf);
    } else {
        formatDimensionText(valBuf, sizeof(valBuf), f(c.value), c.inputUnit, f(c.inputValue));
        snprintf(buf, sizeof(buf), "%s%s", prefix, valBuf);
    }
    ImVec2 textSize = ImGui::CalcTextSize(buf);
    float tmx = (sda[0] + sdb[0]) * 0.5f, tmy = (sda[1] + sdb[1]) * 0.5f;
    float pad = 2.0f;
    ImVec2 textPos = { tmx - textSize.x * 0.5f, tmy - textSize.y * 0.5f };
    float rx0 = textPos.x - pad, ry0 = textPos.y - pad;
    float rx1 = textPos.x + textSize.x + pad, ry1 = textPos.y + textSize.y + pad;
    dl->AddRectFilled({rx0, ry0}, {rx1, ry1}, theme.dimBgColor, 2.0f);
    if (selected) {
        dl->AddRect({rx0 - 1, ry0 - 1}, {rx1 + 1, ry1 + 1},
            IM_COL32(255, 165, 0, 220), 2.0f, 0, 2.0f);
    }
    dl->AddText(textPos, textColor, buf);

    if (labelRects) {
        labelRects->push_back({c.id, planeIndex, rx0, ry0, rx1, ry1});
    }
}

static void renderAngleDim(ImDrawList* dl, const SketchPlane& sp, const Sketch& sketch,
    const Constraint& c, const float view[16], const float proj[16], float vpW, float vpH,
    const Theme& theme, float pxPerLocal, bool selected = false,
    std::vector<DimLabelRectOut>* labelRects = nullptr, int planeIndex = -1) {

    const LineEntity* line1 = sketch.findLine(c.entityA);
    const LineEntity* line2 = sketch.findLine(c.entityB);
    if (!line1 || !line2) return;

    Point2D a1 = sketch.getPointPos(line1->startPt);
    Point2D b1 = sketch.getPointPos(line1->endPt);
    Point2D a2 = sketch.getPointPos(line2->startPt);
    Point2D b2 = sketch.getPointPos(line2->endPt);

    // Find shared vertex or line-line intersection
    Point2D vertex;
    Point2D dir1End, dir2End;
    double eps = 1e-3;
    auto ptEq = [eps](Point2D a, Point2D b) { return std::fabs(a.x-b.x)<eps && std::fabs(a.y-b.y)<eps; };

    if (ptEq(a1, a2) || ptEq(a1, b2))      { vertex = a1; }
    else if (ptEq(b1, a2) || ptEq(b1, b2)) { vertex = b1; }
    else {
        // No shared vertex — compute intersection of infinite lines
        double ldx1 = b1.x-a1.x, ldy1 = b1.y-a1.y;
        double ldx2 = b2.x-a2.x, ldy2 = b2.y-a2.y;
        double denom = ldx1*ldy2 - ldy1*ldx2;
        if (std::fabs(denom) < 1e-6) return; // parallel
        double t = ((a2.x-a1.x)*ldy2 - (a2.y-a1.y)*ldx2) / denom;
        vertex = { a1.x + t*ldx1, a1.y + t*ldy1 };
    }
    // Direction from vertex toward farther endpoint of each line
    dir1End = (distance(vertex, b1) >= distance(vertex, a1)) ? b1 : a1;
    dir2End = (distance(vertex, b2) >= distance(vertex, a2)) ? b2 : a2;

    // Direction vectors from vertex
    double dx1 = dir1End.x - vertex.x, dy1 = dir1End.y - vertex.y;
    double dx2 = dir2End.x - vertex.x, dy2 = dir2End.y - vertex.y;
    double len1 = std::sqrt(dx1*dx1 + dy1*dy1);
    double len2 = std::sqrt(dx2*dx2 + dy2*dy2);
    if (len1 < 1e-6 || len2 < 1e-6) return;

    double angle1 = std::atan2(dy1, dx1); // local angle of dir1

    // Determine sweep direction: CCW or CW from dir1
    // Compute the CCW angle from dir1 to dir2
    double crossV = dx1*dy2 - dy1*dx2;
    double dotV = dx1*dx2 + dy1*dy2;
    double ccwRad = std::atan2(crossV, dotV);
    if (ccwRad < 0) ccwRad += 2.0 * 3.14159265358979;
    double ccwDeg = ccwRad * 180.0 / 3.14159265358979;
    double cwDeg = 360.0 - ccwDeg;

    // If c.value is closer to the CCW angle, sweep CCW; otherwise sweep CW
    double spanRad;
    if (std::fabs(c.value - ccwDeg) <= std::fabs(c.value - cwDeg))
        spanRad = c.value * 3.14159265358979 / 180.0;   // CCW (positive)
    else
        spanRad = -c.value * 3.14159265358979 / 180.0;  // CW (negative)

    // Arc radius in local coords — use dimOffset to control, default ~20% of shortest line
    double arcRadiusLocal = std::min(len1, len2) * 0.3;
    if (c.dimOffsetX != 0 || c.dimOffsetY != 0) {
        double offDist = std::sqrt(c.dimOffsetX * c.dimOffsetX + c.dimOffsetY * c.dimOffsetY);
        if (offDist > 1e-3) arcRadiusLocal = offDist;
    }

    ImU32 dimColor = c.driven ? theme.dimDrivenLineColor : theme.dimLineColor;
    ImU32 textColor = c.driven ? theme.dimDrivenTextColor : theme.dimTextColor;

    // Tessellate arc in local space, transform each point to screen
    int steps = std::max(12, (int)(c.value / 5.0f));
    std::vector<ImVec2> arcScreen(steps + 1);
    bool allVisible = true;
    for (int i = 0; i <= steps; i++) {
        double t = (double)i / steps;
        double a = angle1 + t * spanRad;
        double lx = vertex.x + arcRadiusLocal * std::cos(a);
        double ly = vertex.y + arcRadiusLocal * std::sin(a);
        float w[3];
        sp.localToWorld(f(lx), f(ly), w[0], w[1], w[2]);
        float sc[2];
        if (!worldToScreen(w, view, proj, vpW, vpH, sc[0], sc[1])) { allVisible = false; break; }
        arcScreen[i] = { sc[0], sc[1] };
    }
    if (!allVisible) return;

    // Draw arc segments
    for (int i = 0; i < steps; i++) {
        dl->AddLine(arcScreen[i], arcScreen[i + 1], dimColor, 1.0f);
    }

    // Arrowheads at arc endpoints — computed in local space
    float arrowLenL = 8.0f / pxPerLocal, arrowWidthL = 3.5f / pxPerLocal;

    auto toScr = [&](double lx, double ly, float& sx, float& sy) -> bool {
        float w[3];
        sp.localToWorld(f(lx), f(ly), w[0], w[1], w[2]);
        return worldToScreen(w, view, proj, vpW, vpH, sx, sy);
    };

    // Arrow at start (tangent into arc in local space)
    {
        double startA = angle1;
        double nextA = angle1 + spanRad / steps;
        double tipLx = vertex.x + arcRadiusLocal * std::cos(startA);
        double tipLy = vertex.y + arcRadiusLocal * std::sin(startA);
        double tdx = std::cos(nextA) - std::cos(startA);
        double tdy = std::sin(nextA) - std::sin(startA);
        double tlen = std::sqrt(tdx*tdx + tdy*tdy);
        if (tlen > 1e-8) {
            double tux = tdx / tlen, tuy = tdy / tlen;
            double pnx = -tuy, pny = tux;
            double a1x = tipLx + tux*arrowLenL + pnx*arrowWidthL, a1y = tipLy + tuy*arrowLenL + pny*arrowWidthL;
            double a2x = tipLx + tux*arrowLenL - pnx*arrowWidthL, a2y = tipLy + tuy*arrowLenL - pny*arrowWidthL;
            float st[2], s1[2], s2[2];
            if (toScr(tipLx, tipLy, st[0], st[1]) && toScr(a1x, a1y, s1[0], s1[1]) && toScr(a2x, a2y, s2[0], s2[1]))
                dl->AddTriangleFilled({st[0], st[1]}, {s1[0], s1[1]}, {s2[0], s2[1]}, dimColor);
        }
    }
    // Arrow at end (tangent backward into arc in local space)
    {
        double endA = angle1 + spanRad;
        double prevA = angle1 + spanRad * (steps - 1.0) / steps;
        double tipLx = vertex.x + arcRadiusLocal * std::cos(endA);
        double tipLy = vertex.y + arcRadiusLocal * std::sin(endA);
        double tdx = std::cos(prevA) - std::cos(endA);
        double tdy = std::sin(prevA) - std::sin(endA);
        double tlen = std::sqrt(tdx*tdx + tdy*tdy);
        if (tlen > 1e-8) {
            double tux = tdx / tlen, tuy = tdy / tlen;
            double pnx = -tuy, pny = tux;
            double a1x = tipLx + tux*arrowLenL + pnx*arrowWidthL, a1y = tipLy + tuy*arrowLenL + pny*arrowWidthL;
            double a2x = tipLx + tux*arrowLenL - pnx*arrowWidthL, a2y = tipLy + tuy*arrowLenL - pny*arrowWidthL;
            float st[2], s1[2], s2[2];
            if (toScr(tipLx, tipLy, st[0], st[1]) && toScr(a1x, a1y, s1[0], s1[1]) && toScr(a2x, a2y, s2[0], s2[1]))
                dl->AddTriangleFilled({st[0], st[1]}, {s1[0], s1[1]}, {s2[0], s2[1]}, dimColor);
        }
    }

    // Text at arc midpoint — offset outward from vertex in local space
    double midA = angle1 + spanRad * 0.5;
    double textOffL = 10.0 / pxPerLocal;
    double textLx = vertex.x + (arcRadiusLocal + textOffL) * std::cos(midA);
    double textLy = vertex.y + (arcRadiusLocal + textOffL) * std::sin(midA);
    float tmx, tmy;
    if (!toScr(textLx, textLy, tmx, tmy)) return;

    char buf[80];
    formatAngleText(buf, sizeof(buf), f(c.value));
    if (c.driven) {
        char tmp[80];
        snprintf(tmp, sizeof(tmp), "(%s)", buf);
        strncpy(buf, tmp, sizeof(buf));
    }
    ImVec2 textSize = ImGui::CalcTextSize(buf);
    float pad = 2.0f;
    ImVec2 textPos = { tmx - textSize.x * 0.5f, tmy - textSize.y * 0.5f };
    float rx0 = textPos.x - pad, ry0 = textPos.y - pad;
    float rx1 = textPos.x + textSize.x + pad, ry1 = textPos.y + textSize.y + pad;
    dl->AddRectFilled({rx0, ry0}, {rx1, ry1}, theme.dimBgColor, 2.0f);
    if (selected) {
        dl->AddRect({rx0 - 1, ry0 - 1}, {rx1 + 1, ry1 + 1},
            IM_COL32(255, 165, 0, 220), 2.0f, 0, 2.0f);
    }
    dl->AddText(textPos, textColor, buf);

    if (labelRects) {
        labelRects->push_back({c.id, planeIndex, rx0, ry0, rx1, ry1});
    }
}

// ---- Extracted App methods ----

void App::handleDimToolClick(Sketch& sketch) {
    int w, h;
    glfwGetFramebufferSize(window_, &w, &h);
    float view[16], proj[16];
    getViewProj(w, h, view, proj);
    float apparentScale = computeApparentScale(activePlane(), view, proj, (float)w, (float)h);

    // When editing existing dimension, clicking elsewhere should select another dim or deselect
    if (dimTool_.editingExisting && dimTool_.phase == DimToolState::Editing) {
        handleSelection(sketch, ImGui::GetIO().KeyCtrl);
        return;
    }

    // Check if user clicked on an existing dimension label first
    ImVec2 mouse = ImGui::GetIO().MousePos;
    for (const auto& r : dimLabelRects_) {
        if (r.sketchPlaneIndex != activeSketchPlane_) continue;
        if (mouse.x >= r.x0 && mouse.x <= r.x1 && mouse.y >= r.y0 && mouse.y <= r.y1) {
            // Clicked on existing dimension — switch to editing it
            Constraint* cc = sketch.findConstraint(r.constraintID);
            if (cc) {
                dimTool_.reset();
                dimTool_.phase = DimToolState::Editing;
                dimTool_.constraintID = r.constraintID;
                dimTool_.editingExisting = true;
                dimTool_.driven = cc->driven;
                if (cc->type == ConstraintType::Distance)
                    dimTool_.selType = HitType::Line;
                else if (cc->type == ConstraintType::Diameter || cc->type == ConstraintType::Radius)
                    dimTool_.selType = HitType::Circle;
                else if (cc->type == ConstraintType::PointDistance || cc->type == ConstraintType::PointLineDistance)
                    dimTool_.selType = HitType::Point;
                else if (cc->type == ConstraintType::Angle)
                    dimTool_.selType = HitType::Line;
                dimTool_.entityA = cc->entityA;
                dimTool_.entityB = cc->entityB;
                dimTool_.measuredMm = f(cc->value);
                if (cc->type == ConstraintType::Angle)
                    formatAngleText(dimTool_.inputBuf, sizeof(dimTool_.inputBuf), f(cc->value));
                else if (!cc->inputUnit.empty())
                    snprintf(dimTool_.inputBuf, sizeof(dimTool_.inputBuf), "%.4g%s", cc->inputValue, cc->inputUnit.c_str());
                else
                    snprintf(dimTool_.inputBuf, sizeof(dimTool_.inputBuf), "%.4gmm", cc->value);
                dimTool_.focusNeeded = true;
                selection_.select(HitType::Dimension, r.constraintID);
            }
            return;
        }
    }

    HitResult hit = hitTest(cursorLocal_, apparentScale, sketch, 10.0f);

    // Helper: check if a dimension constraint already exists for the given entity/entities
    auto hasDuplicate = [&](ConstraintType newType, EntityID eA, EntityID eB) -> bool {
        for (const auto& c : sketch.constraints) {
            bool typeMatch = (c.type == newType);
            // Radius and Diameter dimension the same freedom, so either blocks the other.
            // This used to be checked in one direction only, which let a Radius be added to a
            // circle that already had a Diameter (and then the two fought over the value).
            if (newType == ConstraintType::Diameter || newType == ConstraintType::Radius)
                typeMatch = (c.type == ConstraintType::Diameter || c.type == ConstraintType::Radius);
            if (!typeMatch) continue;
            if (newType == ConstraintType::PointDistance || newType == ConstraintType::Angle || newType == ConstraintType::PointLineDistance) {
                if ((c.entityA == eA && c.entityB == eB) ||
                    (c.entityA == eB && c.entityB == eA))
                    return true;
            } else {
                if (c.entityA == eA) return true;
            }
        }
        return false;
    };

    if (dimTool_.phase == DimToolState::Selecting) {
        // Point already selected + line clicked → point-line distance
        if (hit.type == HitType::Line && dimTool_.selType == HitType::Point && dimTool_.entityA != NullID) {
            LineEntity* line = sketch.findLine(hit.entityID);
            if (!line) return;
            PointEntity* pt = sketch.findPoint(dimTool_.entityA);
            if (!pt) return;
            if (hasDuplicate(ConstraintType::PointLineDistance, dimTool_.entityA, hit.entityID)) {
                snprintf(dimTool_.warningMsg, sizeof(dimTool_.warningMsg),
                         "This point-line pair already has a distance constraint");
                dimTool_.warningTimer = 3.0f;
                dimTool_.entityA = NullID;
                dimTool_.selType = HitType::None;
                selection_.clear();
                return;
            }
            Point2D la = sketch.getPointPos(line->startPt);
            Point2D lb = sketch.getPointPos(line->endPt);
            double ldx = lb.x - la.x, ldy = lb.y - la.y;
            double lineLen = std::sqrt(ldx*ldx + ldy*ldy);
            if (lineLen < 1e-6) return;
            double cross = (pt->x - la.x)*ldy - (pt->y - la.y)*ldx;
            bool negativeSide = (cross < 0);
            dimTool_.measuredMm = f(std::fabs(cross) / lineLen);
            dimTool_.entityB = hit.entityID;
            snprintf(dimTool_.inputBuf, sizeof(dimTool_.inputBuf), "%.4gmm", dimTool_.measuredMm);

            auto geoBak = sketch.captureGeometry();
            EntityID cid = sketch.addConstraint(ConstraintType::PointLineDistance, dimTool_.entityA, hit.entityID, dimTool_.measuredMm, false);
            auto res = solver_.solve(sketch);
            lastSketchDof_ = res.dof;
            if (!res.ok) { sketch.restoreGeometry(geoBak); dimTool_.driven = true; }
            Constraint* cc = sketch.findConstraint(cid);
            if (cc) { cc->driven = dimTool_.driven; cc->negativeSide = negativeSide; }
            dimTool_.constraintID = cid;
            dimTool_.focusNeeded = true;
            dimTool_.placingFirstFrame = true;
            dimTool_.phase = DimToolState::EditingAndPlacing;
            selection_.select(HitType::Dimension, cid);
        // Point already selected + circle clicked where point is center → radius
        } else if (hit.type == HitType::Circle && dimTool_.selType == HitType::Point && dimTool_.entityA != NullID) {
            CircleEntity* circle = sketch.findCircle(hit.entityID);
            if (!circle) return;
            if (circle->centerPt != dimTool_.entityA) {
                snprintf(dimTool_.warningMsg, sizeof(dimTool_.warningMsg),
                         "Select the circle's center point first to dimension its radius");
                dimTool_.warningTimer = 3.0f;
                dimTool_.entityA = NullID;
                dimTool_.selType = HitType::None;
                selection_.clear();
                return;
            }
            if (hasDuplicate(ConstraintType::Diameter, hit.entityID, NullID)) {
                snprintf(dimTool_.warningMsg, sizeof(dimTool_.warningMsg),
                         "This circle already has a dimension constraint");
                dimTool_.warningTimer = 3.0f;
                dimTool_.entityA = NullID;
                dimTool_.selType = HitType::None;
                selection_.clear();
                return;
            }
            dimTool_.selType = HitType::Circle;
            dimTool_.entityA = hit.entityID;
            dimTool_.entityB = NullID;
            dimTool_.measuredMm = f(circle->radius);
            snprintf(dimTool_.inputBuf, sizeof(dimTool_.inputBuf), "%.4gmm", dimTool_.measuredMm);

            {
                auto geoBak = sketch.captureGeometry();
                EntityID cid = sketch.addConstraint(ConstraintType::Radius, hit.entityID, NullID, dimTool_.measuredMm, false);
                auto res = solver_.solve(sketch);
                lastSketchDof_ = res.dof;
                if (!res.ok) { sketch.restoreGeometry(geoBak); dimTool_.driven = true; }
                Constraint* cc = sketch.findConstraint(cid);
                if (cc) cc->driven = dimTool_.driven;
                dimTool_.constraintID = cid;
                dimTool_.focusNeeded = true;
                dimTool_.placingFirstFrame = true;
                dimTool_.phase = DimToolState::EditingAndPlacing;
                selection_.select(HitType::Dimension, cid);
            }
        } else if (hit.type == HitType::Line) {
            if (hasDuplicate(ConstraintType::Distance, hit.entityID, NullID)) {
                snprintf(dimTool_.warningMsg, sizeof(dimTool_.warningMsg),
                         "This line already has a dimension constraint");
                dimTool_.warningTimer = 3.0f;
                return;
            }
            dimTool_.selType = HitType::Line;
            dimTool_.entityA = hit.entityID;
            dimTool_.entityB = NullID;
            LineEntity* line = sketch.findLine(hit.entityID);
            if (!line) return;
            Point2D a = sketch.getPointPos(line->startPt);
            Point2D b = sketch.getPointPos(line->endPt);
            dimTool_.measuredMm = f(distance(a, b));
            snprintf(dimTool_.inputBuf, sizeof(dimTool_.inputBuf), "%.4gmm", dimTool_.measuredMm);

            // Create constraint immediately
            auto geoBak = sketch.captureGeometry();
            EntityID cid = sketch.addConstraint(ConstraintType::Distance, hit.entityID, NullID, dimTool_.measuredMm, false);
            auto res = solver_.solve(sketch);
            lastSketchDof_ = res.dof;
            if (!res.ok) { sketch.restoreGeometry(geoBak); dimTool_.driven = true; }
            Constraint* cc = sketch.findConstraint(cid);
            if (cc) cc->driven = dimTool_.driven;
            dimTool_.constraintID = cid;
            dimTool_.focusNeeded = true;
            dimTool_.placingFirstFrame = true;
            dimTool_.phase = DimToolState::EditingAndPlacing;
            selection_.select(HitType::Dimension, cid);
        } else if (hit.type == HitType::Circle) {
            if (hasDuplicate(ConstraintType::Diameter, hit.entityID, NullID)) {
                snprintf(dimTool_.warningMsg, sizeof(dimTool_.warningMsg),
                         "This circle already has a dimension constraint");
                dimTool_.warningTimer = 3.0f;
                return;
            }
            dimTool_.selType = HitType::Circle;
            dimTool_.entityA = hit.entityID;
            dimTool_.entityB = NullID;
            CircleEntity* circle = sketch.findCircle(hit.entityID);
            if (!circle) return;
            dimTool_.measuredMm = f(circle->radius * 2.0);
            snprintf(dimTool_.inputBuf, sizeof(dimTool_.inputBuf), "%.4gmm", dimTool_.measuredMm);

            auto geoBak = sketch.captureGeometry();
            EntityID cid = sketch.addConstraint(ConstraintType::Diameter, hit.entityID, NullID, dimTool_.measuredMm, false);
            auto res = solver_.solve(sketch);
            lastSketchDof_ = res.dof;
            if (!res.ok) { sketch.restoreGeometry(geoBak); dimTool_.driven = true; }
            Constraint* cc = sketch.findConstraint(cid);
            if (cc) cc->driven = dimTool_.driven;
            dimTool_.constraintID = cid;
            dimTool_.focusNeeded = true;
            dimTool_.placingFirstFrame = true;
            dimTool_.phase = DimToolState::EditingAndPlacing;
            selection_.select(HitType::Dimension, cid);
        } else if (hit.type == HitType::Point) {
            if (dimTool_.entityA == NullID) {
                // First point
                dimTool_.selType = HitType::Point;
                dimTool_.entityA = hit.entityID;
                dimTool_.entityB = NullID;
                selection_.select(hit.type, hit.entityID);
            } else if (dimTool_.selType == HitType::Point && dimTool_.entityB == NullID) {
                if (hasDuplicate(ConstraintType::PointDistance, dimTool_.entityA, hit.entityID)) {
                    snprintf(dimTool_.warningMsg, sizeof(dimTool_.warningMsg),
                             "These points already have a distance constraint");
                    dimTool_.warningTimer = 3.0f;
                    dimTool_.entityA = NullID;
                    selection_.clear();
                    return;
                }
                dimTool_.entityB = hit.entityID;
                Point2D a = sketch.getPointPos(dimTool_.entityA);
                Point2D b = sketch.getPointPos(dimTool_.entityB);
                dimTool_.measuredMm = f(distance(a, b));
                snprintf(dimTool_.inputBuf, sizeof(dimTool_.inputBuf), "%.4gmm", dimTool_.measuredMm);

                auto geoBak = sketch.captureGeometry();
                EntityID cid = sketch.addConstraint(ConstraintType::PointDistance, dimTool_.entityA, dimTool_.entityB, dimTool_.measuredMm, false);
                auto res = solver_.solve(sketch);
                lastSketchDof_ = res.dof;
                if (!res.ok) { sketch.restoreGeometry(geoBak); dimTool_.driven = true; }
                Constraint* cc = sketch.findConstraint(cid);
                if (cc) cc->driven = dimTool_.driven;
                dimTool_.constraintID = cid;
                dimTool_.focusNeeded = true;
                dimTool_.placingFirstFrame = true;
                dimTool_.phase = DimToolState::EditingAndPlacing;
                selection_.select(HitType::Dimension, cid);
            }
        }
    }
}

void App::drawDimensionPanel(Sketch& sketch) {
    ImGuiIO& io = ImGui::GetIO();
    float panelW = 220.0f;

    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, {10, 10});
    ImGui::SetNextWindowPos({io.DisplaySize.x - panelW, 30});
    ImGui::SetNextWindowSize({panelW, 0});
    ImGui::Begin("Dimension", nullptr,
        ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove |
        ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_AlwaysAutoResize);

    // Show warning if duplicate dimension was attempted
    if (dimTool_.warningTimer > 0) {
        dimTool_.warningTimer -= io.DeltaTime;
        ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 0.8f, 0.2f, 1.0f));
        ImGui::TextWrapped("%s", dimTool_.warningMsg);
        ImGui::PopStyleColor();
        ImGui::Separator();
    }

    if (dimTool_.phase == DimToolState::Selecting) {
        if (dimTool_.selType == HitType::Point && dimTool_.entityA != NullID) {
            ImGui::TextWrapped("Select second point or a line");
        } else {
            ImGui::TextWrapped("Select a line, circle, or two points");
        }
    } else if (dimTool_.phase == DimToolState::Editing || dimTool_.phase == DimToolState::EditingAndPlacing) {
        // Show what's selected
        bool isAngleConstraint = false;
        bool isPointLineDistance = false;
        bool isRadiusConstraint = false;
        if (dimTool_.constraintID != NullID) {
            Constraint* chk = sketch.findConstraint(dimTool_.constraintID);
            if (chk && chk->type == ConstraintType::Angle) isAngleConstraint = true;
            if (chk && chk->type == ConstraintType::PointLineDistance) isPointLineDistance = true;
            if (chk && chk->type == ConstraintType::Radius) isRadiusConstraint = true;
        }
        if (isAngleConstraint)
            ImGui::Text("Angle:");
        else if (isPointLineDistance)
            ImGui::Text("Point-Line Distance:");
        else if (isRadiusConstraint)
            ImGui::Text("Circle Radius:");
        else if (dimTool_.selType == HitType::Line)
            ImGui::Text("Line Distance:");
        else if (dimTool_.selType == HitType::Circle)
            ImGui::Text("Circle Diameter:");
        else if (dimTool_.selType == HitType::Point)
            ImGui::Text("Point Distance:");

        if (dimTool_.phase == DimToolState::EditingAndPlacing) {
            if (isAngleConstraint)
                ImGui::TextColored({0.6f, 0.8f, 1.0f, 1.0f}, "Click to place label");
            else if (dimTool_.selType == HitType::Line && dimTool_.entityB == NullID)
                ImGui::TextColored({0.6f, 0.8f, 1.0f, 1.0f}, "Click to place, or click another line for angle");
            else if (dimTool_.selType == HitType::Circle && !isRadiusConstraint)
                ImGui::TextColored({0.6f, 0.8f, 1.0f, 1.0f}, "Click to place, or click center for radius");
            else
                ImGui::TextColored({0.6f, 0.8f, 1.0f, 1.0f}, "Click to place label");
        }

        ImGui::SetNextItemWidth(-1);
        // During EditingAndPlacing, always keep input focused so user can type while placing
        if (dimTool_.focusNeeded || dimTool_.phase == DimToolState::EditingAndPlacing) {
            ImGui::SetKeyboardFocusHere();
            dimTool_.focusNeeded = false;
        }

        bool entered = ImGui::InputText("##dimval", dimTool_.inputBuf, sizeof(dimTool_.inputBuf),
            ImGuiInputTextFlags_EnterReturnsTrue | ImGuiInputTextFlags_AutoSelectAll);

        ImGui::Checkbox("Driven (reference only)", &dimTool_.driven);

        // Sync input value to constraint live (so label updates in real-time)
        if (dimTool_.constraintID != NullID) {
            Constraint* cc = sketch.findConstraint(dimTool_.constraintID);
            if (cc) {
                if (isAngleConstraint && dimTool_.angleAutoSide) {
                    // Auto-side mode: handleSketchInput controls the value, skip live sync
                    cc->driven = dimTool_.driven;
                } else if (isAngleConstraint) {
                    float liveDeg = parseAngleInput(dimTool_.inputBuf);
                    if (liveDeg >= 0.001f && liveDeg <= 359.999f) {
                        cc->value = liveDeg;
                        cc->driven = dimTool_.driven;
                    }
                } else {
                    std::string liveUnit;
                    float liveInput = 0;
                    float liveMm = parseUnitInput(dimTool_.inputBuf, liveUnit, liveInput);
                    bool allowNeg = (cc->type == ConstraintType::PointLineDistance);
                    if (allowNeg ? (std::fabs(liveMm) > 0.0001f || liveMm == 0.0f) : (liveMm > 0.001f)) {
                        cc->value = liveMm;
                        cc->inputUnit = liveUnit;
                        cc->inputValue = liveInput;
                        cc->driven = dimTool_.driven;
                    }
                }
            }
        }

        ImGui::Separator();
        const char* applyLabel = dimTool_.editingExisting ? "Update [Enter]" : "Apply [Enter]";
        bool apply = ImGui::Button(applyLabel, {95, 0});
        ImGui::SameLine();
        if (ImGui::Button("Cancel [Esc]", {95, 0})) {
            if (dimTool_.phase == DimToolState::EditingAndPlacing && dimTool_.constraintID != NullID) {
                sketch.removeConstraint(dimTool_.constraintID);
            } else if (dimTool_.editingExisting && dimTool_.constraintID != NullID) {
                // Restore original value that live-sync may have changed
                Constraint* cc = sketch.findConstraint(dimTool_.constraintID);
                if (cc) cc->value = dimTool_.measuredMm;
            }
            dimTool_.reset();
            selection_.clear();
        }

        if (entered || apply) {
            bool valueOk = false;
            Constraint* cc = sketch.findConstraint(dimTool_.constraintID);
            if (cc) {
                if (isAngleConstraint) {
                    float deg = parseAngleInput(dimTool_.inputBuf);
                    if (deg >= 0.001f && deg <= 359.999f) {
                        // Determine correct sector (CW/CCW) for the new value
                        LineEntity* l1 = sketch.findLine(cc->entityA);
                        LineEntity* l2 = sketch.findLine(cc->entityB);
                        if (l1 && l2) {
                            PointEntity* pa1 = sketch.findPoint(l1->startPt);
                            PointEntity* pb1 = sketch.findPoint(l1->endPt);
                            PointEntity* pa2 = sketch.findPoint(l2->startPt);
                            PointEntity* pb2 = sketch.findPoint(l2->endPt);
                            if (pa1 && pb1 && pa2 && pb2) {
                                float eps = 1e-3f;
                                auto pEq = [eps](PointEntity* p, PointEntity* q) {
                                    return std::fabs(p->x-q->x)<eps && std::fabs(p->y-q->y)<eps;
                                };
                                double vx = 0.0, vy = 0.0;
                                bool hasV = false;
                                if (pEq(pa1,pa2)||pEq(pa1,pb2))      { vx=pa1->x; vy=pa1->y; hasV=true; }
                                else if (pEq(pb1,pa2)||pEq(pb1,pb2)) { vx=pb1->x; vy=pb1->y; hasV=true; }
                                else {
                                    double ldx1=pb1->x-pa1->x, ldy1=pb1->y-pa1->y;
                                    double ldx2=pb2->x-pa2->x, ldy2=pb2->y-pa2->y;
                                    double denom=ldx1*ldy2-ldy1*ldx2;
                                    if (std::fabs(denom)>1e-6) {
                                        double t=((pa2->x-pa1->x)*ldy2-(pa2->y-pa1->y)*ldx2)/denom;
                                        vx=pa1->x+t*ldx1; vy=pa1->y+t*ldy1; hasV=true;
                                    }
                                }
                                if (hasV) {
                                    auto d2 = [](double ax,double ay,double bx,double by){ double dx=ax-bx,dy=ay-by; return dx*dx+dy*dy; };
                                    PointEntity* f1=(d2(vx,vy,pb1->x,pb1->y)>=d2(vx,vy,pa1->x,pa1->y))?pb1:pa1;
                                    PointEntity* f2=(d2(vx,vy,pb2->x,pb2->y)>=d2(vx,vy,pa2->x,pa2->y))?pb2:pa2;
                                    double dx1=f1->x-vx, dy1=f1->y-vy;
                                    double dx2=f2->x-vx, dy2=f2->y-vy;
                                    double cross=dx1*dy2-dy1*dx2, dot=dx1*dx2+dy1*dy2;
                                    double ccwRad=std::atan2(cross,dot);
                                    if (ccwRad<0) ccwRad+=kTwoPi;
                                    double ccwDeg=ccwRad*180.0/kPi;
                                    double cwDeg=360.0-ccwDeg;
                                    // Pick the sector closest to the user's typed value
                                    cc->angleCW = (std::fabs(cwDeg-deg) < std::fabs(ccwDeg-deg));
                                }
                            }
                        }
                        cc->value = deg;
                        cc->driven = dimTool_.driven;
                        valueOk = true;
                    }
                } else {
                    std::string unitName;
                    float inputValue = 0;
                    float valueMm = parseUnitInput(dimTool_.inputBuf, unitName, inputValue);
                    bool allowNeg = (cc->type == ConstraintType::PointLineDistance);
                    if (allowNeg ? true : (valueMm > 0.001f)) {
                        cc->value = valueMm;
                        cc->inputUnit = unitName;
                        cc->inputValue = inputValue;
                        cc->driven = dimTool_.driven;
                        valueOk = true;
                    }
                }

                if (valueOk && !cc->driven) {
                    auto geoBak = sketch.captureGeometry();
                    PointEntity* pinA = nullptr, *pinB = nullptr;
                    bool wasA = false, wasB = false;
                    if (cc->type == ConstraintType::PointLineDistance) {
                        LineEntity* refLine = sketch.findLine(cc->entityB);
                        if (refLine) {
                            pinA = sketch.findPoint(refLine->startPt);
                            pinB = sketch.findPoint(refLine->endPt);
                            if (pinA) { wasA = pinA->projected; pinA->projected = true; }
                            if (pinB) { wasB = pinB->projected; pinB->projected = true; }
                        }
                    }
                    auto res = solver_.solve(sketch);
                    lastSketchDof_ = res.dof;
                    if (pinA) pinA->projected = wasA;
                    if (pinB) pinB->projected = wasB;
                    if (!res.ok) {
                        sketch.restoreGeometry(geoBak);
                        cc->driven = true;
                        dimTool_.driven = true;
                    }
                }
            }

            if (valueOk) {
                history_.pushState(sketch);
                dimTool_.reset();
                selection_.clear();
            }
        }

        if (ImGui::IsKeyPressed(ImGuiKey_Escape)) {
            if (dimTool_.phase == DimToolState::EditingAndPlacing && dimTool_.constraintID != NullID) {
                sketch.removeConstraint(dimTool_.constraintID);
            } else if (dimTool_.editingExisting && dimTool_.constraintID != NullID) {
                Constraint* cc = sketch.findConstraint(dimTool_.constraintID);
                if (cc) cc->value = dimTool_.measuredMm;
            }
            dimTool_.reset();
            selection_.clear();
        }
    }

    ImGui::End();
    ImGui::PopStyleVar();
}

void App::renderDimensions(const float view[16], const float proj[16], float vpW, float vpH) {
    ImDrawList* dl = ImGui::GetForegroundDrawList();
    const auto& theme = activeTheme();
    const float arrowLen = 8.0f;
    const float arrowWidth = 3.5f;
    const float extGap = 3.0f;
    const float extOvershoot = 4.0f;

    // Rebuild label rects for hit-testing this frame
    std::vector<DimLabelRectOut> labelRects;

    for (int pi = 0; pi < (int)sketchPlanes_.size(); pi++) {
        const auto& sp = sketchPlanes_[pi];
        // Only show dimensions when actively editing this sketch
        if (pi != activeSketchPlane_) continue;

        const Sketch& sketch = sp.sketch;

        // Compute pixels-per-local-unit for this sketch plane
        float w0[3], w1[3];
        sp.localToWorld(0, 0, w0[0], w0[1], w0[2]);
        sp.localToWorld(1, 0, w1[0], w1[1], w1[2]);
        float s0[2], s1[2];
        float pxPerLocal = 100.0f; // fallback
        if (worldToScreen(w0, view, proj, vpW, vpH, s0[0], s0[1]) &&
            worldToScreen(w1, view, proj, vpW, vpH, s1[0], s1[1])) {
            float sdx = s1[0] - s0[0], sdy = s1[1] - s0[1];
            pxPerLocal = std::sqrt(sdx*sdx + sdy*sdy);
            if (pxPerLocal < 1e-3f) pxPerLocal = 1e-3f;
        }

        for (const auto& c : sketch.constraints) {
            bool sel = selection_.isSelected(HitType::Dimension, c.id);

            // --- Radius constraint (legacy) ---
            if (c.type == ConstraintType::Radius) {
                const CircleEntity* circle = nullptr;
                for (const auto& ci : sketch.circles)
                    if (ci.id == c.entityA) { circle = &ci; break; }
                if (!circle) continue;
                Point2D center = sketch.getPointPos(circle->centerPt);
                Point2D edge = { center.x + circle->radius, center.y };
                renderTwoPointDim(dl, sp, c, center, edge, "R ",
                    view, proj, vpW, vpH, theme, arrowLen, arrowWidth, extGap, extOvershoot,
                    pxPerLocal, sel, &labelRects, pi);
                continue;
            }

            // --- Diameter constraint ---
            if (c.type == ConstraintType::Diameter) {
                const CircleEntity* circle = nullptr;
                for (const auto& ci : sketch.circles)
                    if (ci.id == c.entityA) { circle = &ci; break; }
                if (!circle) continue;
                Point2D center = sketch.getPointPos(circle->centerPt);
                Point2D left = { center.x - circle->radius, center.y };
                Point2D right = { center.x + circle->radius, center.y };
                renderTwoPointDim(dl, sp, c, left, right, "\xC3\xB8 ",
                    view, proj, vpW, vpH, theme, arrowLen, arrowWidth, extGap, extOvershoot,
                    pxPerLocal, sel, &labelRects, pi);
                continue;
            }

            // --- Distance constraint on a line ---
            if (c.type == ConstraintType::Distance) {
                const LineEntity* line = sketch.findLine(c.entityA);
                if (!line) continue;
                Point2D a2d = sketch.getPointPos(line->startPt);
                Point2D b2d = sketch.getPointPos(line->endPt);
                renderTwoPointDim(dl, sp, c, a2d, b2d, "",
                    view, proj, vpW, vpH, theme, arrowLen, arrowWidth, extGap, extOvershoot,
                    pxPerLocal, sel, &labelRects, pi);
                continue;
            }

            // --- Point-to-line distance ---
            if (c.type == ConstraintType::PointLineDistance) {
                Point2D pt2d = sketch.getPointPos(c.entityA);
                const LineEntity* line = sketch.findLine(c.entityB);
                if (!line) continue;
                Point2D la = sketch.getPointPos(line->startPt);
                Point2D lb = sketch.getPointPos(line->endPt);
                double ldx = lb.x - la.x, ldy = lb.y - la.y;
                double len2 = ldx*ldx + ldy*ldy;
                if (len2 < 1e-12) continue;
                double t = ((pt2d.x - la.x)*ldx + (pt2d.y - la.y)*ldy) / len2;
                Point2D projPt = {la.x + t*ldx, la.y + t*ldy};
                renderTwoPointDim(dl, sp, c, pt2d, projPt, "",
                    view, proj, vpW, vpH, theme, arrowLen, arrowWidth, extGap, extOvershoot,
                    pxPerLocal, sel, &labelRects, pi);
                continue;
            }

            // --- Point-to-point distance ---
            if (c.type == ConstraintType::PointDistance) {
                Point2D a2d = sketch.getPointPos(c.entityA);
                Point2D b2d = sketch.getPointPos(c.entityB);
                renderTwoPointDim(dl, sp, c, a2d, b2d, "",
                    view, proj, vpW, vpH, theme, arrowLen, arrowWidth, extGap, extOvershoot,
                    pxPerLocal, sel, &labelRects, pi);
                continue;
            }

            // --- Angle constraint ---
            if (c.type == ConstraintType::Angle) {
                renderAngleDim(dl, sp, sketch, c,
                    view, proj, vpW, vpH, theme,
                    pxPerLocal, sel, &labelRects, pi);
                continue;
            }
        }
    }

    // Copy label rects to member for hit-testing
    dimLabelRects_.clear();
    for (const auto& r : labelRects)
        dimLabelRects_.push_back({r.constraintID, r.planeIndex, r.x0, r.y0, r.x1, r.y1});

    // ---- Constraint icons (non-dimensional constraints) ----
    if (activeSketchPlane_ >= 0 && activeSketchPlane_ < (int)sketchPlanes_.size()) {
        const auto& sp = sketchPlanes_[activeSketchPlane_];
        const Sketch& sketch = sp.sketch;

        // Collect icons to draw: (screenX, screenY, label, color)
        struct CIcon { float sx, sy; const char* label; ImU32 col; };
        std::vector<CIcon> icons;

        ImU32 iconCol = IM_COL32((int)(prefs_.conTextCol[0]*255), (int)(prefs_.conTextCol[1]*255),
                                  (int)(prefs_.conTextCol[2]*255), (int)(prefs_.conTextCol[3]*255));
        ImU32 iconBg  = IM_COL32((int)(prefs_.conBgCol[0]*255), (int)(prefs_.conBgCol[1]*255),
                                  (int)(prefs_.conBgCol[2]*255), (int)(prefs_.conBgCol[3]*255));

        auto toScreen = [&](Point2D local, float& sx, float& sy) -> bool {
            float w3[3];
            sp.localToWorld(f(local.x), f(local.y), w3[0], w3[1], w3[2]);
            return worldToScreen(w3, view, proj, vpW, vpH, sx, sy);
        };

        // Track icon positions to offset stacked icons at the same location
        auto addIcon = [&](Point2D pos, float offsetX, float offsetY, const char* label) {
            float sx, sy;
            if (!toScreen(pos, sx, sy)) return;
            icons.push_back({sx + offsetX, sy + offsetY, label, iconCol});
        };

        for (const auto& c : sketch.constraints) {
            switch (c.type) {
                case ConstraintType::Horizontal: {
                    const LineEntity* line = sketch.findLine(c.entityA);
                    if (!line) break;
                    Point2D a = sketch.getPointPos(line->startPt);
                    Point2D b = sketch.getPointPos(line->endPt);
                    addIcon(midpoint(a, b), 0, -14, "H");
                    break;
                }
                case ConstraintType::Vertical: {
                    const LineEntity* line = sketch.findLine(c.entityA);
                    if (!line) break;
                    Point2D a = sketch.getPointPos(line->startPt);
                    Point2D b = sketch.getPointPos(line->endPt);
                    addIcon(midpoint(a, b), 10, 0, "V");
                    break;
                }
                case ConstraintType::Coincident: {
                    Point2D p = sketch.getPointPos(c.entityA);
                    addIcon(p, 8, -8, "c");
                    break;
                }
                case ConstraintType::Perpendicular: {
                    // Show near the midpoint of entityB (second line)
                    const LineEntity* line = sketch.findLine(c.entityB);
                    if (!line) { line = sketch.findLine(c.entityA); if (!line) break; }
                    Point2D a = sketch.getPointPos(line->startPt);
                    Point2D b = sketch.getPointPos(line->endPt);
                    addIcon(midpoint(a, b), 0, -14, "\xe2\x9f\x82");
                    break;
                }
                case ConstraintType::Parallel: {
                    const LineEntity* line = sketch.findLine(c.entityB);
                    if (!line) { line = sketch.findLine(c.entityA); if (!line) break; }
                    Point2D a = sketch.getPointPos(line->startPt);
                    Point2D b = sketch.getPointPos(line->endPt);
                    addIcon(midpoint(a, b), 0, -14, "//");
                    break;
                }
                case ConstraintType::Collinear: {
                    const LineEntity* line = sketch.findLine(c.entityB);
                    if (!line) { line = sketch.findLine(c.entityA); if (!line) break; }
                    Point2D a = sketch.getPointPos(line->startPt);
                    Point2D b = sketch.getPointPos(line->endPt);
                    addIcon(midpoint(a, b), 0, -14, "—");
                    break;
                }
                case ConstraintType::EqualLength: {
                    // Show on both lines
                    const LineEntity* l1 = sketch.findLine(c.entityA);
                    const LineEntity* l2 = sketch.findLine(c.entityB);
                    if (l1) {
                        Point2D a = sketch.getPointPos(l1->startPt);
                        Point2D b = sketch.getPointPos(l1->endPt);
                        addIcon(midpoint(a, b), 0, -14, "=");
                    }
                    if (l2) {
                        Point2D a = sketch.getPointPos(l2->startPt);
                        Point2D b = sketch.getPointPos(l2->endPt);
                        addIcon(midpoint(a, b), 0, -14, "=");
                    }
                    break;
                }
                case ConstraintType::Tangent: {
                    const LineEntity* line = sketch.findLine(c.entityA);
                    if (!line) line = sketch.findLine(c.entityB);
                    if (line) {
                        Point2D a = sketch.getPointPos(line->startPt);
                        Point2D b = sketch.getPointPos(line->endPt);
                        addIcon(midpoint(a, b), 0, -14, "T");
                    }
                    break;
                }
                case ConstraintType::PointOnLine: {
                    Point2D p = sketch.getPointPos(c.entityA);
                    addIcon(p, 8, -8, "c");
                    break;
                }
                case ConstraintType::PointOnCircle: {
                    Point2D p = sketch.getPointPos(c.entityA);
                    addIcon(p, 8, -8, "c");
                    break;
                }
                case ConstraintType::Midpoint: {
                    Point2D p = sketch.getPointPos(c.entityA);
                    addIcon(p, 8, -8, "M");
                    break;
                }
                case ConstraintType::Symmetric: {
                    Point2D p1 = sketch.getPointPos(c.entityA);
                    Point2D p2 = sketch.getPointPos(c.entityB);
                    addIcon(midpoint(p1, p2), 0, -14, "SYM");
                    break;
                }
                case ConstraintType::Concentric: {
                    // Show at first entity's center
                    EntityID cpID = NullID;
                    const CircleEntity* ci = sketch.findCircle(c.entityA);
                    const ArcEntity* ai = sketch.findArc(c.entityA);
                    if (ci) cpID = ci->centerPt;
                    else if (ai) cpID = ai->centerPt;
                    if (cpID != NullID) {
                        Point2D p = sketch.getPointPos(cpID);
                        addIcon(p, 8, -8, "CC");
                    }
                    break;
                }
                default:
                    break; // Distance, Radius, Diameter, Angle handled above as dimensions
            }
        }

        // Coincident indicators at shared junction points (structural — no explicit constraint
        // entity exists; coincidence is implicit through shared point IDs).
        {
            std::unordered_map<EntityID, int> refCount;
            auto ref = [&](EntityID ptID) { if (ptID != NullID) refCount[ptID]++; };
            for (const auto& l  : sketch.lines)      { ref(l.startPt);  ref(l.endPt); }
            for (const auto& a  : sketch.arcs)       { ref(a.startPt);  ref(a.endPt); }
            for (const auto& spl : sketch.splines)   { for (auto id : spl.controlPtIDs) ref(id); }
            for (const auto& [ptID, cnt] : refCount) {
                if (cnt < 2) continue;
                addIcon(sketch.getPointPos(ptID), 6, -6, "c");
            }
        }

        // Draw all icons
        ImFont* font = ImGui::GetFont();
        float fontSize = ImGui::GetFontSize() * 0.85f;
        for (const auto& ic : icons) {
            ImVec2 textSize = font->CalcTextSizeA(fontSize, FLT_MAX, 0, ic.label);
            float px = ic.sx - textSize.x * 0.5f;
            float py = ic.sy - textSize.y * 0.5f;
            // Background pill
            float pad = 2.0f;
            dl->AddRectFilled(ImVec2(px - pad, py - pad),
                              ImVec2(px + textSize.x + pad, py + textSize.y + pad),
                              iconBg, 3.0f);
            dl->AddText(font, fontSize, ImVec2(px, py), ic.col, ic.label);
        }
    }
}

} // namespace shitcad
