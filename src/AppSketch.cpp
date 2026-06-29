#include "App.h"
#include "ProfileDetector.h"
#include "Extrude.h"
#include "AutoConstraint.h"
#include "FacePicker.h"
#include "ExtrudeTool.h"
#include "FeatureReplay.h"

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
#include <cstring>

namespace shitcad {

void App::applyGeometricConstraint(Sketch& sketch, ConstraintType type) {
    auto& sel = selection_.selected;

    // Extract entity IDs by type from selection
    auto findSelOfType = [&](HitType t, int nth = 0) -> EntityID {
        int count = 0;
        for (auto& s : sel) {
            if (s.type == t) {
                if (count == nth) return s.id;
                count++;
            }
        }
        return NullID;
    };

    EntityID eA = NullID, eB = NullID, eC = NullID;

    switch (type) {
        case ConstraintType::Perpendicular:
        case ConstraintType::Parallel:
        case ConstraintType::Collinear:
        case ConstraintType::EqualLength:
            eA = findSelOfType(HitType::Line, 0);
            eB = findSelOfType(HitType::Line, 1);
            break;
        case ConstraintType::Tangent:
            eA = findSelOfType(HitType::Line, 0);
            eB = findSelOfType(HitType::Circle, 0);
            if (eB == NullID) eB = findSelOfType(HitType::Arc, 0);
            break;
        case ConstraintType::PointOnLine:
        case ConstraintType::Midpoint:
            eA = findSelOfType(HitType::Point, 0);
            eB = findSelOfType(HitType::Line, 0);
            break;
        case ConstraintType::Symmetric:
            eA = findSelOfType(HitType::Point, 0);
            eB = findSelOfType(HitType::Point, 1);
            eC = findSelOfType(HitType::Line, 0);
            break;
        case ConstraintType::Concentric: {
            // Could be circles or arcs
            EntityID c1 = findSelOfType(HitType::Circle, 0);
            EntityID a1 = findSelOfType(HitType::Arc, 0);
            EntityID c2 = findSelOfType(HitType::Circle, 1);
            EntityID a2 = findSelOfType(HitType::Arc, 1);
            // Pick first two non-null
            EntityID ids[4] = {c1, c2, a1, a2};
            int found = 0;
            for (auto id : ids) {
                if (id != NullID) {
                    if (found == 0) eA = id;
                    else if (found == 1) eB = id;
                    found++;
                }
            }
            break;
        }
        default: return;
    }

    if (eA == NullID || eB == NullID) {
        if (type == ConstraintType::Tangent && eA == NullID) {
            snprintf(sketchMsg_, sizeof(sketchMsg_), "Tangent requires a line and an arc or circle");
            sketchMsgTimer_ = 3.0f;
        }
        return;
    }
    if (type == ConstraintType::Symmetric && eC == NullID) return;

    // Check for duplicate constraint
    for (const auto& c : sketch.constraints) {
        if (c.type != type) continue;
        bool match = (c.entityA == eA && c.entityB == eB) || (c.entityA == eB && c.entityB == eA);
        if (type == ConstraintType::Symmetric)
            match = match && c.entityC == eC;
        if (match) return; // already exists
    }

    // Backup for solver rollback
    auto ptsBak = sketch.points;
    auto circBak = sketch.circles;

    EntityID cid = sketch.addConstraint(type, eA, eB, 0.0f, false);
    if (type == ConstraintType::Symmetric) {
        Constraint* cc = sketch.findConstraint(cid);
        if (cc) cc->entityC = eC;
    }

    auto res = solver_.solve(sketch);
    lastSketchDof_ = res.dof;
    if (!res.ok) {
        // Solver couldn't satisfy — mark as driven and notify user
        sketch.points = ptsBak;
        sketch.circles = circBak;
        Constraint* cc = sketch.findConstraint(cid);
        if (cc) cc->driven = true;
        snprintf(sketchMsg_, sizeof(sketchMsg_),
                 "Over-constrained: constraint added as reference only");
        sketchMsgTimer_ = 3.0f;
    }

    history_.pushState(sketch);
    selection_.clear();
}

void App::handleSketchInput(float vpW, float vpH) {
    ImGuiIO& io = ImGui::GetIO();

    // Extrude input is now handled in renderFrame before this function
    if (tool_.type == ToolType::Extrude) {
        return;
    }

    Sketch& sketch = activeSketch();
    const SketchPlane& plane = activePlane();
    bool mouseOverUI = io.WantCaptureMouse;

    int w, h;
    glfwGetFramebufferSize(window_, &w, &h);
    float view[16], proj[16];
    getViewProj(w, h, view, proj);

    float apparentScale = computeApparentScale(plane, view, proj, vpW, vpH);
    if (apparentScale < 1e-9f) apparentScale = 1e-9f; // guard against degenerate projection

    if (!mouseOverUI) {
        // Project mouse to sketch plane
        float rayOrig[3], rayDir[3];
        screenToRay(io.MousePos.x, io.MousePos.y, 0, 0, vpW, vpH, view, proj, rayOrig, rayDir);

        float lx, ly, t;
        if (plane.rayIntersect(rayOrig, rayDir, lx, ly, t)) {
            cursorLocal_ = {lx, ly};
        }
    }

    // Dimension label drag end — must be before early returns
    if (selection_.dragMode == SelectionDragMode::DimDrag && ImGui::IsMouseReleased(ImGuiMouseButton_Left)) {
        if (selection_.dragStarted) {
            history_.pushState(sketch);
        }
        selection_.dragMode = SelectionDragMode::None;
        selection_.dragDimID = NullID;
        selection_.dragStarted = false;
    }

    // Dimension label dragging — works regardless of tool/phase
    if (selection_.dragMode == SelectionDragMode::DimDrag) {
        Constraint* cc = sketch.findConstraint(selection_.dragDimID);
        if (cc) {
            // Use local coords directly, same as placement
            Point2D mid = {};
            auto computeMid = [&](EntityID cid) -> Point2D {
                for (const auto& c : sketch.constraints) {
                    if (c.id != cid) continue;
                    if (c.type == ConstraintType::Distance) {
                        const LineEntity* line = sketch.findLine(c.entityA);
                        if (line) {
                            Point2D a = sketch.getPointPos(line->startPt);
                            Point2D b = sketch.getPointPos(line->endPt);
                            return { (a.x + b.x) * 0.5f, (a.y + b.y) * 0.5f };
                        }
                    } else if (c.type == ConstraintType::Diameter || c.type == ConstraintType::Radius) {
                        const CircleEntity* circle = sketch.findCircle(c.entityA);
                        if (circle) return sketch.getPointPos(circle->centerPt);
                    } else if (c.type == ConstraintType::PointDistance) {
                        Point2D a = sketch.getPointPos(c.entityA);
                        Point2D b = sketch.getPointPos(c.entityB);
                        return { (a.x + b.x) * 0.5f, (a.y + b.y) * 0.5f };
                    } else if (c.type == ConstraintType::PointLineDistance) {
                        Point2D pt = sketch.getPointPos(c.entityA);
                        const LineEntity* line = sketch.findLine(c.entityB);
                        if (line) {
                            Point2D la = sketch.getPointPos(line->startPt);
                            Point2D lb = sketch.getPointPos(line->endPt);
                            double ldx = lb.x - la.x, ldy = lb.y - la.y;
                            double len2 = ldx*ldx + ldy*ldy;
                            if (len2 > 1e-12) {
                                double t = ((pt.x - la.x)*ldx + (pt.y - la.y)*ldy) / len2;
                                Point2D projPt = {la.x + t*ldx, la.y + t*ldy};
                                return Point2D{(pt.x + projPt.x) * 0.5, (pt.y + projPt.y) * 0.5};
                            }
                        }
                        return pt;
                    } else if (c.type == ConstraintType::Angle) {
                        const LineEntity* l1 = sketch.findLine(c.entityA);
                        const LineEntity* l2 = sketch.findLine(c.entityB);
                        if (l1 && l2) {
                            Point2D a1 = sketch.getPointPos(l1->startPt);
                            Point2D b1 = sketch.getPointPos(l1->endPt);
                            Point2D a2 = sketch.getPointPos(l2->startPt);
                            Point2D b2 = sketch.getPointPos(l2->endPt);
                            float eps = 1e-3f;
                            auto ptEq = [eps](Point2D p, Point2D q) { return std::fabs(p.x-q.x)<eps && std::fabs(p.y-q.y)<eps; };
                            if (ptEq(a1, a2) || ptEq(a1, b2)) return a1;
                            if (ptEq(b1, a2) || ptEq(b1, b2)) return b1;
                            // No shared vertex — compute intersection
                            double ldx1 = b1.x-a1.x, ldy1 = b1.y-a1.y;
                            double ldx2 = b2.x-a2.x, ldy2 = b2.y-a2.y;
                            double denom = ldx1*ldy2 - ldy1*ldx2;
                            if (std::fabs(denom) > 1e-6) {
                                double t = ((a2.x-a1.x)*ldy2 - (a2.y-a1.y)*ldx2) / denom;
                                return Point2D{ a1.x + t*ldx1, a1.y + t*ldy1 };
                            }
                            return Point2D{ (a1.x + a2.x) * 0.5, (a1.y + a2.y) * 0.5 };
                        }
                    }
                    break;
                }
                return {};
            };
            mid = computeMid(selection_.dragDimID);
            cc->dimOffsetX = cursorLocal_.x - mid.x;
            cc->dimOffsetY = cursorLocal_.y - mid.y;
            selection_.dragStarted = true;

            // For angle constraints, flip between acute/reflex based on drag side
            if (cc->type == ConstraintType::Angle) {
                const LineEntity* l1 = sketch.findLine(cc->entityA);
                const LineEntity* l2 = sketch.findLine(cc->entityB);
                if (l1 && l2) {
                    Point2D a1 = sketch.getPointPos(l1->startPt);
                    Point2D b1 = sketch.getPointPos(l1->endPt);
                    Point2D a2 = sketch.getPointPos(l2->startPt);
                    Point2D b2 = sketch.getPointPos(l2->endPt);
                    Point2D vtx = mid;
                    Point2D d1 = (distance(vtx, b1) >= distance(vtx, a1)) ? b1 : a1;
                    Point2D d2 = (distance(vtx, b2) >= distance(vtx, a2)) ? b2 : a2;
                    double dx1 = d1.x - vtx.x, dy1 = d1.y - vtx.y;
                    double dx2 = d2.x - vtx.x, dy2 = d2.y - vtx.y;
                    double crossV = dx1*dy2 - dy1*dx2;
                    double dotV = dx1*dx2 + dy1*dy2;
                    double ccwRad = std::atan2(crossV, dotV);
                    if (ccwRad < 0) ccwRad += kTwoPi;
                    double dir1A = std::atan2(dy1, dx1);
                    double dxm = cursorLocal_.x - vtx.x, dym = cursorLocal_.y - vtx.y;
                    double mouseA = std::atan2(dym, dxm);
                    double mouseSpan = mouseA - dir1A;
                    if (mouseSpan < 0) mouseSpan += kTwoPi;
                    // Mouse in CCW sector → use CCW angle; else CW
                    double sideDeg = (mouseSpan < ccwRad)
                        ? ccwRad * 180.0 / 3.14159265358979
                        : (kTwoPi - ccwRad) * 180.0 / 3.14159265358979;
                    if (sideDeg < 0.001) sideDeg = 0.001;
                    if (sideDeg > 359.999) sideDeg = 359.999;
                    // Only flip if we're crossing from one sector to the other
                    double ccwDeg = ccwRad * 180.0 / 3.14159265358979;
                    double cwDeg = 360.0 - ccwDeg;
                    bool wasInCcw = (std::fabs(cc->value - ccwDeg) <= std::fabs(cc->value - cwDeg));
                    bool nowInCcw = (mouseSpan < ccwRad);
                    if (wasInCcw != nowInCcw) {
                        cc->value = sideDeg;
                        // Update dim tool inputBuf so live sync doesn't overwrite the flip
                        if (dimTool_.phase == DimToolState::Editing && dimTool_.constraintID == selection_.dragDimID) {
                            formatAngleText(dimTool_.inputBuf, sizeof(dimTool_.inputBuf), f(sideDeg));
                            GImGui->ActiveId = 0;
                        }
                    }
                }
            }
        }
    }
    if (selection_.dragMode == SelectionDragMode::None &&
        ImGui::IsMouseDragging(ImGuiMouseButton_Left, 3.0f)) {
        ImVec2 clickPos = ImGui::GetIO().MouseClickedPos[0];
        for (const auto& r : dimLabelRects_) {
            if (r.sketchPlaneIndex != activeSketchPlane_) continue;
            if (clickPos.x >= r.x0 && clickPos.x <= r.x1 &&
                clickPos.y >= r.y0 && clickPos.y <= r.y1) {
                selection_.select(HitType::Dimension, r.constraintID);
                selection_.dragMode = SelectionDragMode::DimDrag;
                selection_.dragDimID = r.constraintID;
                selection_.dragStarted = false;
                break;
            }
        }
    }

    if (tool_.type == ToolType::Dimension &&
        (dimTool_.phase == DimToolState::Editing || dimTool_.phase == DimToolState::EditingAndPlacing)) {
        // Escape during EditingAndPlacing cancels label placement, removing the constraint
        if (dimTool_.phase == DimToolState::EditingAndPlacing &&
            ImGui::IsKeyPressed(ImGuiKey_Escape) && !io.WantCaptureKeyboard) {
            sketch.removeConstraint(dimTool_.constraintID);
            dimTool_.reset();
            tool_.type = ToolType::Dimension;
            selection_.clear();
            history_.pushState(sketch); // record removal so undo doesn't re-add it
            return;
        }
        // Delete key removes the dimension being edited
        if (dimTool_.phase == DimToolState::Editing &&
            ImGui::IsKeyPressed(ImGuiKey_Delete) && !io.WantCaptureKeyboard) {
            sketch.removeConstraint(dimTool_.constraintID);
            dimTool_.reset();
            selection_.clear();
            history_.pushState(sketch);
        }
        // EditingAndPlacing: update label position from cursor each frame
        if (dimTool_.phase == DimToolState::EditingAndPlacing && dimTool_.constraintID != NullID) {
            Point2D mid = {};
            Constraint* cc = sketch.findConstraint(dimTool_.constraintID);
            if (cc && cc->type == ConstraintType::Angle) {
                // For angle, mid is the shared vertex or line intersection
                LineEntity* l1 = sketch.findLine(cc->entityA);
                LineEntity* l2 = sketch.findLine(cc->entityB);
                if (l1 && l2) {
                    Point2D a1 = sketch.getPointPos(l1->startPt);
                    Point2D b1 = sketch.getPointPos(l1->endPt);
                    Point2D a2 = sketch.getPointPos(l2->startPt);
                    Point2D b2 = sketch.getPointPos(l2->endPt);
                    float eps = 1e-3f;
                    auto ptEq = [eps](Point2D p, Point2D q) { return std::fabs(p.x-q.x)<eps && std::fabs(p.y-q.y)<eps; };
                    if (ptEq(a1, a2) || ptEq(a1, b2)) mid = a1;
                    else if (ptEq(b1, a2) || ptEq(b1, b2)) mid = b1;
                    else {
                        double ldx1 = b1.x-a1.x, ldy1 = b1.y-a1.y;
                        double ldx2 = b2.x-a2.x, ldy2 = b2.y-a2.y;
                        double denom = ldx1*ldy2 - ldy1*ldx2;
                        if (std::fabs(denom) > 1e-6) {
                            double t = ((a2.x-a1.x)*ldy2 - (a2.y-a1.y)*ldx2) / denom;
                            mid = { a1.x + t*ldx1, a1.y + t*ldy1 };
                        }
                    }

                    // Auto-side: flip between acute/reflex based on which side cursor is on
                    if (dimTool_.angleAutoSide) {
                        // Check if user started typing → exit auto mode
                        if (io.InputQueueCharacters.Size > 0) {
                            dimTool_.angleAutoSide = false;
                        } else {
                            Point2D vtx = mid;
                            Point2D d1 = (distance(vtx, b1) >= distance(vtx, a1)) ? b1 : a1;
                            Point2D d2 = (distance(vtx, b2) >= distance(vtx, a2)) ? b2 : a2;
                            double dx1 = d1.x - vtx.x, dy1 = d1.y - vtx.y;
                            double dx2 = d2.x - vtx.x, dy2 = d2.y - vtx.y;
                            double dir1A = std::atan2(dy1, dx1);
                            double crossV = dx1*dy2 - dy1*dx2;
                            double dotV = dx1*dx2 + dy1*dy2;
                            // CCW angle from dir1 to dir2 (0 to 2pi)
                            double ccwRad = std::atan2(crossV, dotV);
                            if (ccwRad < 0) ccwRad += kTwoPi;

                            // Where is mouse relative to dir1?
                            double dxm = cursorLocal_.x - vtx.x, dym = cursorLocal_.y - vtx.y;
                            double mouseA = std::atan2(dym, dxm);
                            double mouseSpan = mouseA - dir1A;
                            if (mouseSpan < 0) mouseSpan += kTwoPi;

                            // Mouse in CCW sector → use CCW angle; else use CW (reflex)
                            double newDeg;
                            bool newCW;
                            if (mouseSpan < ccwRad) {
                                newDeg = ccwRad * 180.0 / 3.14159265358979;
                                newCW = false;
                            } else {
                                newDeg = (kTwoPi - ccwRad) * 180.0 / 3.14159265358979;
                                newCW = true;
                            }

                            if (newDeg < 0.001) newDeg = 0.001;
                            if (newDeg > 359.999) newDeg = 359.999;

                            cc->value = newDeg;
                            cc->angleCW = newCW;
                            dimTool_.measuredMm = f(newDeg);
                            formatAngleText(dimTool_.inputBuf, sizeof(dimTool_.inputBuf), f(newDeg));
                            GImGui->ActiveId = 0; // force InputText to re-read buffer
                        }
                    }
                }
            } else if (dimTool_.selType == HitType::Line) {
                LineEntity* line = sketch.findLine(dimTool_.entityA);
                if (line) {
                    Point2D a = sketch.getPointPos(line->startPt);
                    Point2D b = sketch.getPointPos(line->endPt);
                    mid = { (a.x + b.x) * 0.5f, (a.y + b.y) * 0.5f };
                }
            } else if (dimTool_.selType == HitType::Circle) {
                CircleEntity* circle = sketch.findCircle(dimTool_.entityA);
                if (circle) mid = sketch.getPointPos(circle->centerPt);
            } else if (dimTool_.selType == HitType::Point) {
                Point2D a = sketch.getPointPos(dimTool_.entityA);
                // Check if entityB is a line (point-line distance) or a point
                if (cc && cc->type == ConstraintType::PointLineDistance) {
                    LineEntity* line = sketch.findLine(dimTool_.entityB);
                    if (line) {
                        Point2D la = sketch.getPointPos(line->startPt);
                        Point2D lb = sketch.getPointPos(line->endPt);
                        double ldx = lb.x - la.x, ldy = lb.y - la.y;
                        double len2 = ldx*ldx + ldy*ldy;
                        if (len2 > 1e-12) {
                            double t = ((a.x - la.x)*ldx + (a.y - la.y)*ldy) / len2;
                            Point2D projPt = {la.x + t*ldx, la.y + t*ldy};
                            mid = {(a.x + projPt.x) * 0.5, (a.y + projPt.y) * 0.5};
                        }
                    }
                } else {
                    Point2D b = sketch.getPointPos(dimTool_.entityB);
                    mid = { (a.x + b.x) * 0.5f, (a.y + b.y) * 0.5f };
                }
            }
            if (cc) {
                cc->dimOffsetX = cursorLocal_.x - mid.x;
                cc->dimOffsetY = cursorLocal_.y - mid.y;
            }

            // Left click finalizes placement (skip the frame we entered this phase)
            if (dimTool_.placingFirstFrame) {
                dimTool_.placingFirstFrame = false;
            } else if (ImGui::IsMouseClicked(ImGuiMouseButton_Left) && !mouseOverUI) {
                // If placing a line length and user clicks a second line → switch to angle
                if (dimTool_.selType == HitType::Line && dimTool_.entityB == NullID) {
                    HitResult hit2 = hitTest(cursorLocal_, apparentScale, sketch, 10.0f);
                    if (hit2.type == HitType::Line && hit2.entityID != dimTool_.entityA) {
                        // Check if lines are parallel — if so, make a distance dimension instead of angle
                        {
                            LineEntity* l1 = sketch.findLine(dimTool_.entityA);
                            LineEntity* l2 = sketch.findLine(hit2.entityID);
                            if (l1 && l2) {
                                Point2D a1 = sketch.getPointPos(l1->startPt);
                                Point2D b1 = sketch.getPointPos(l1->endPt);
                                Point2D a2 = sketch.getPointPos(l2->startPt);
                                Point2D b2 = sketch.getPointPos(l2->endPt);
                                double d1x = b1.x - a1.x, d1y = b1.y - a1.y;
                                double d2x = b2.x - a2.x, d2y = b2.y - a2.y;
                                double len1 = std::sqrt(d1x*d1x + d1y*d1y);
                                double len2 = std::sqrt(d2x*d2x + d2y*d2y);
                                if (len1 > 1e-6 && len2 > 1e-6) {
                                    d1x /= len1; d1y /= len1;
                                    d2x /= len2; d2y /= len2;
                                    double cross = d1x*d2y - d1y*d2x;
                                    constexpr float kParallelSin = 0.087f; // ~5°
                                    if (std::fabs(cross) < kParallelSin) {
                                        // Parallel lines — create point-line distance
                                        // Check for duplicate
                                        bool dupPLD = false;
                                        for (const auto& cx : sketch.constraints) {
                                            if (cx.type == ConstraintType::PointLineDistance) {
                                                if ((cx.entityA == l1->startPt || cx.entityA == l1->endPt) && cx.entityB == hit2.entityID) {
                                                    dupPLD = true; break;
                                                }
                                            }
                                        }
                                        if (dupPLD) {
                                            snprintf(dimTool_.warningMsg, sizeof(dimTool_.warningMsg),
                                                     "These parallel lines already have a distance constraint");
                                            dimTool_.warningTimer = 3.0f;
                                        } else {
                                            sketch.removeConstraint(dimTool_.constraintID);

                                            // Use whichever endpoint of l1 is farther from l2's line
                                            // (gives the more representative measurement)
                                            EntityID ptID = l1->startPt;
                                            {
                                                Point2D la = sketch.getPointPos(l2->startPt);
                                                Point2D lb = sketch.getPointPos(l2->endPt);
                                                double ldx = lb.x - la.x, ldy = lb.y - la.y;
                                                double ll = std::sqrt(ldx*ldx + ldy*ldy);
                                                if (ll > 1e-6) {
                                                    auto ptDist = [&](EntityID id) {
                                                        Point2D p = sketch.getPointPos(id);
                                                        return std::fabs((p.x - la.x)*ldy - (p.y - la.y)*ldx) / ll;
                                                    };
                                                    if (ptDist(l1->endPt) > ptDist(l1->startPt))
                                                        ptID = l1->endPt;
                                                }
                                            }

                                            Point2D pt = sketch.getPointPos(ptID);
                                            Point2D la = sketch.getPointPos(l2->startPt);
                                            Point2D lb = sketch.getPointPos(l2->endPt);
                                            double ldx = lb.x - la.x, ldy = lb.y - la.y;
                                            double ll = std::sqrt(ldx*ldx + ldy*ldy);
                                            double measDist = 0.0;
                                            bool negSide = false;
                                            if (ll > 1e-6) {
                                                double cr = (pt.x - la.x)*ldy - (pt.y - la.y)*ldx;
                                                negSide = (cr < 0);
                                                measDist = std::fabs(cr) / ll;
                                            }

                                            EntityID cid = sketch.addConstraint(ConstraintType::PointLineDistance, ptID, hit2.entityID, measDist, false);
                                            Constraint* newCon = sketch.findConstraint(cid);
                                            if (newCon) { newCon->driven = dimTool_.driven; newCon->negativeSide = negSide; }

                                            dimTool_.selType = HitType::Point;
                                            dimTool_.entityA = ptID;
                                            dimTool_.entityB = hit2.entityID;
                                            dimTool_.constraintID = cid;
                                            dimTool_.measuredMm = f(measDist);
                                            snprintf(dimTool_.inputBuf, sizeof(dimTool_.inputBuf), "%.4gmm", measDist);
                                            dimTool_.focusNeeded = true;
                                            dimTool_.placingFirstFrame = true;
                                            selection_.select(HitType::Dimension, cid);
                                            GImGui->ActiveId = 0;
                                        }
                                        goto skipFinalize;
                                    }
                                }
                            }
                        }

                        // Check duplicate angle
                        bool dupAngle = false;
                        for (const auto& cx : sketch.constraints) {
                            if (cx.type == ConstraintType::Angle) {
                                if ((cx.entityA == dimTool_.entityA && cx.entityB == hit2.entityID) ||
                                    (cx.entityA == hit2.entityID && cx.entityB == dimTool_.entityA)) {
                                    dupAngle = true; break;
                                }
                            }
                        }
                        if (dupAngle) {
                            snprintf(dimTool_.warningMsg, sizeof(dimTool_.warningMsg),
                                     "These lines already have an angle constraint");
                            dimTool_.warningTimer = 3.0f;
                        } else {
                            // Remove the length constraint, create angle constraint
                            EntityID lineA = dimTool_.entityA;
                            sketch.removeConstraint(dimTool_.constraintID);

                            // Compute angle between lines
                            LineEntity* l1 = sketch.findLine(lineA);
                            LineEntity* l2 = sketch.findLine(hit2.entityID);
                            float angleDeg = 90.0f; // fallback
                            bool angleCWsector = false;
                            if (l1 && l2) {
                                Point2D a1 = sketch.getPointPos(l1->startPt);
                                Point2D b1 = sketch.getPointPos(l1->endPt);
                                Point2D a2 = sketch.getPointPos(l2->startPt);
                                Point2D b2 = sketch.getPointPos(l2->endPt);
                                float eps = 1e-3f;
                                auto ptEq2 = [eps](Point2D p, Point2D q) { return std::fabs(p.x-q.x)<eps && std::fabs(p.y-q.y)<eps; };

                                // Find vertex: shared endpoint or line-line intersection
                                Point2D vtx;
                                bool hasVtx = false;
                                if (ptEq2(a1, a2) || ptEq2(a1, b2))      { vtx = a1; hasVtx = true; }
                                else if (ptEq2(b1, a2) || ptEq2(b1, b2)) { vtx = b1; hasVtx = true; }
                                else {
                                    double ldx1 = b1.x-a1.x, ldy1 = b1.y-a1.y;
                                    double ldx2 = b2.x-a2.x, ldy2 = b2.y-a2.y;
                                    double denom = ldx1*ldy2 - ldy1*ldx2;
                                    if (std::fabs(denom) > 1e-6) {
                                        double t = ((a2.x-a1.x)*ldy2 - (a2.y-a1.y)*ldx2) / denom;
                                        vtx = { a1.x + t*ldx1, a1.y + t*ldy1 };
                                        hasVtx = true;
                                    }
                                }
                                if (hasVtx) {
                                    // Direction from vertex toward farther endpoint
                                    Point2D d1 = (distance(vtx, b1) >= distance(vtx, a1)) ? b1 : a1;
                                    Point2D d2 = (distance(vtx, b2) >= distance(vtx, a2)) ? b2 : a2;
                                    double dx1 = d1.x - vtx.x, dy1 = d1.y - vtx.y;
                                    double dx2 = d2.x - vtx.x, dy2 = d2.y - vtx.y;
                                    double dot = dx1*dx2 + dy1*dy2;
                                    double cross = dx1*dy2 - dy1*dx2;
                                    // Unsigned angle between lines (0-180°), always pick the smaller one
                                    double rad = std::atan2(std::fabs(cross), dot); // 0 to pi
                                    angleDeg = f(rad * 180.0 / 3.14159265358979);

                                    if (angleDeg < 0.001f) angleDeg = 0.001f;
                                    if (angleDeg > 179.999f) angleDeg = 179.999f;
                                    angleCWsector = (cross < 0);
                                }
                            }

                            EntityID cid = sketch.addConstraint(ConstraintType::Angle, lineA, hit2.entityID, angleDeg, false);
                            Constraint* ac = sketch.findConstraint(cid);
                            if (ac) {
                                ac->driven = dimTool_.driven;
                                ac->angleCW = angleCWsector;
                            }

                            dimTool_.entityB = hit2.entityID;
                            dimTool_.constraintID = cid;
                            dimTool_.measuredMm = angleDeg;
                            dimTool_.angleBase = angleDeg; // unsigned angle for auto-side
                            dimTool_.angleAutoSide = true;
                            formatAngleText(dimTool_.inputBuf, sizeof(dimTool_.inputBuf), angleDeg);
                            dimTool_.focusNeeded = true;
                            dimTool_.placingFirstFrame = true;
                            selection_.select(HitType::Dimension, cid);
                            // Force ImGui to drop its internal InputText editing state
                            // so it picks up the new buffer content (angle instead of length)
                            GImGui->ActiveId = 0;
                        }
                        // Don't finalize — stay in EditingAndPlacing
                        goto skipFinalize;
                    }
                    // If placing a line length and user clicks a point → switch to point-line distance
                    if (hit2.type == HitType::Point) {
                        EntityID lineID = dimTool_.entityA;
                        LineEntity* line = sketch.findLine(lineID);
                        PointEntity* pt = sketch.findPoint(hit2.entityID);
                        if (line && pt) {
                            // Check that the point is not an endpoint of this line
                            if (hit2.entityID != line->startPt && hit2.entityID != line->endPt) {
                                // Check for duplicate point-line distance
                                bool dupPLD = false;
                                for (const auto& cx : sketch.constraints) {
                                    if (cx.type == ConstraintType::PointLineDistance) {
                                        if ((cx.entityA == hit2.entityID && cx.entityB == lineID) ||
                                            (cx.entityA == lineID && cx.entityB == hit2.entityID)) {
                                            dupPLD = true; break;
                                        }
                                    }
                                }
                                if (dupPLD) {
                                    snprintf(dimTool_.warningMsg, sizeof(dimTool_.warningMsg),
                                             "This point-line pair already has a distance constraint");
                                    dimTool_.warningTimer = 3.0f;
                                    goto skipFinalize;
                                }
                                sketch.removeConstraint(dimTool_.constraintID);

                                Point2D la = sketch.getPointPos(line->startPt);
                                Point2D lb = sketch.getPointPos(line->endPt);
                                double ldx = lb.x - la.x, ldy = lb.y - la.y;
                                double lineLen = std::sqrt(ldx*ldx + ldy*ldy);
                                double measDist = 0;
                                bool negSide = false;
                                if (lineLen > 1e-6) {
                                    double cross = (pt->x - la.x)*ldy - (pt->y - la.y)*ldx;
                                    negSide = (cross < 0);
                                    measDist = std::fabs(cross) / lineLen;
                                }

                                EntityID cid = sketch.addConstraint(ConstraintType::PointLineDistance, hit2.entityID, lineID, measDist, false);
                                Constraint* newCon = sketch.findConstraint(cid);
                                if (newCon) { newCon->driven = dimTool_.driven; newCon->negativeSide = negSide; }

                                dimTool_.selType = HitType::Point;
                                dimTool_.entityA = hit2.entityID;
                                dimTool_.entityB = lineID;
                                dimTool_.constraintID = cid;
                                dimTool_.measuredMm = f(measDist);
                                snprintf(dimTool_.inputBuf, sizeof(dimTool_.inputBuf), "%.4gmm", measDist);
                                dimTool_.focusNeeded = true;
                                dimTool_.placingFirstFrame = true;
                                selection_.select(HitType::Dimension, cid);
                                GImGui->ActiveId = 0;
                                goto skipFinalize;
                            }
                        }
                    }
                }
                // If placing a diameter and user clicks the center point → switch to radius
                if (dimTool_.selType == HitType::Circle) {
                    HitResult hit2 = hitTest(cursorLocal_, apparentScale, sketch, 10.0f);
                    if (hit2.type == HitType::Point) {
                        CircleEntity* circle = sketch.findCircle(dimTool_.entityA);
                        if (circle && hit2.entityID == circle->centerPt) {
                            // Check for existing radius constraint
                            bool dupRadius = false;
                            for (const auto& cx : sketch.constraints) {
                                if ((cx.type == ConstraintType::Radius || cx.type == ConstraintType::Diameter) && cx.entityA == dimTool_.entityA && cx.id != dimTool_.constraintID) {
                                    dupRadius = true; break;
                                }
                            }
                            if (!dupRadius) {
                                sketch.removeConstraint(dimTool_.constraintID);
                                double radius = circle->radius;
                                EntityID cid = sketch.addConstraint(ConstraintType::Radius, dimTool_.entityA, NullID, radius, false);
                                auto ptsBak = sketch.points; auto circBak = sketch.circles;
                                auto res = solver_.solve(sketch);
                                lastSketchDof_ = res.dof;
                                if (!res.ok) { sketch.points = ptsBak; sketch.circles = circBak; dimTool_.driven = true; }
                                Constraint* rc = sketch.findConstraint(cid);
                                if (rc) rc->driven = dimTool_.driven;
                                dimTool_.constraintID = cid;
                                dimTool_.measuredMm = f(radius);
                                snprintf(dimTool_.inputBuf, sizeof(dimTool_.inputBuf), "%.4gmm", radius);
                                dimTool_.focusNeeded = true;
                                dimTool_.placingFirstFrame = true;
                                selection_.select(HitType::Dimension, cid);
                                GImGui->ActiveId = 0;
                                goto skipFinalize;
                            }
                        }
                    }
                }

                // Apply the constraint value and solve before finalizing.
                // For PointLineDistance, pin the reference line (entityB) so only
                // the point side moves to satisfy the constraint.
                Constraint* fc = sketch.findConstraint(dimTool_.constraintID);
                if (fc && !fc->driven) {
                    auto ptsBak = sketch.points;
                    auto circBak = sketch.circles;
                    PointEntity* pinA = nullptr, *pinB = nullptr;
                    bool wasA = false, wasB = false;
                    if (fc->type == ConstraintType::PointLineDistance) {
                        LineEntity* refLine = sketch.findLine(fc->entityB);
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
                        sketch.points = ptsBak;
                        sketch.circles = circBak;
                        fc->driven = true;
                    }
                }
                history_.pushState(sketch);
                dimTool_.reset();
                selection_.clear();
                skipFinalize:;
            }
        }
        return;
    }

    // Grid step
    float gridStep = 1.0f;
    float worldSpacing = 20.0f * dpiScale_ / apparentScale;
    gridStep = std::pow(10.0f, std::floor(std::log10(worldSpacing)));
    if (gridStep * apparentScale < 5.0f * dpiScale_) gridStep *= 10.0f;

    // Snap
    currentSnap_ = snapEngine_.snap(cursorLocal_, apparentScale, sketch, gridStep);

    // Tangent snap override: when line tool has a first point and cursor is near a circle/arc,
    // snap to the geometric tangent point instead of nearest-on-curve
    bool showTangentLabel = false;
    if (tool_.type == ToolType::Line && tool_.hasFirstPoint &&
        currentSnap_.type == SnapType::NearestOnCurve && currentSnap_.curveID != NullID) {
        Point2D anchor = tool_.firstPoint;
        Point2D center;
        double radius = 0;
        CircleEntity* circ = sketch.findCircle(currentSnap_.curveID);
        ArcEntity* arcE = sketch.findArc(currentSnap_.curveID);
        if (circ) {
            center = sketch.getPointPos(circ->centerPt);
            radius = circ->radius;
        } else if (arcE) {
            center = sketch.getPointPos(arcE->centerPt);
            Point2D sp = sketch.getPointPos(arcE->startPt);
            radius = distance(center, sp);
        }
        double d = distance(anchor, center);
        if (d > radius + 1e-6) {
            // External point: compute two tangent points
            double halfAngle = std::acos(radius / d);
            double baseAngle = std::atan2(anchor.y - center.y, anchor.x - center.x);
            Point2D t1 = {center.x + radius * std::cos(baseAngle + halfAngle),
                          center.y + radius * std::sin(baseAngle + halfAngle)};
            Point2D t2 = {center.x + radius * std::cos(baseAngle - halfAngle),
                          center.y + radius * std::sin(baseAngle - halfAngle)};
            double d1 = distance(cursorLocal_, t1);
            double d2 = distance(cursorLocal_, t2);
            // Only override if cursor is actually close to a tangent point (tight zone)
            constexpr float kTangentSnapPx = 12.0f;
            float worldTol = kTangentSnapPx / apparentScale;
            double best = std::min(d1, d2);
            if (best < worldTol) {
                currentSnap_.position = (d1 <= d2) ? t1 : t2;
                currentSnap_.type = SnapType::Tangent;
                showTangentLabel = true;
            }
        }
    }
    // Draw "T" constraint label near the snap indicator
    if (showTangentLabel) {
        float w3[3];
        plane.localToWorld(f(currentSnap_.position.x), f(currentSnap_.position.y), w3[0], w3[1], w3[2]);
        float sx, sy;
        if (worldToScreen(w3, view, proj, vpW, vpH, sx, sy)) {
            ImDrawList* dl = ImGui::GetForegroundDrawList();
            const auto& sc = activeTheme().snapColor;
            ImU32 col = IM_COL32((int)(sc[0]*255), (int)(sc[1]*255), (int)(sc[2]*255), 255);
            dl->AddText(ImVec2(sx + 8, sy - 16), col, "T");
        }
    }

    // H/V snap: when the in-progress line is near-horizontal or near-vertical, lock
    // currentSnap_.position to the H/V axis so the rubber-band line itself snaps.
    // Also snaps to intersections of the H/V rail with existing geometry.
    // Tangent snap takes priority (more specific geometric lock).
    hvCrossEntityID_ = NullID;
    if (tool_.type == ToolType::Line && tool_.hasFirstPoint && !mouseOverUI &&
        currentSnap_.type != SnapType::Tangent) {
        float dx = (float)(cursorLocal_.x - tool_.firstPoint.x);
        float dy = (float)(cursorLocal_.y - tool_.firstPoint.y);
        float len = std::sqrt(dx * dx + dy * dy);
        if (len > 0.5f / apparentScale) {
            constexpr float kThreshDeg = 5.0f;
            const float sinThresh = std::sin(kThreshDeg * 3.14159265f / 180.0f);
            bool nearH = std::fabs(dy / len) < sinThresh;
            bool nearV = !nearH && std::fabs(dx / len) < sinThresh;

            if (nearH || nearV) {
                // Project raw cursor onto H or V axis through the start point
                Point2D hvPos = nearH
                    ? Point2D{cursorLocal_.x, tool_.firstPoint.y}
                    : Point2D{tool_.firstPoint.x, cursorLocal_.y};

                double worldTol = snapEngine_.snapTolerancePx / (double)apparentScale;
                EntityID hvPtID = NullID;
                EntityID hvCrossID = NullID;
                double bestDist = worldTol;

                // 1. Existing coincident points on the rail
                for (const auto& pt : sketch.points) {
                    if (pt.id == tool_.firstPointID) continue;
                    double d = distance(hvPos, {pt.x, pt.y});
                    if (d < bestDist) {
                        bestDist = d;
                        hvPtID   = pt.id;
                        hvCrossID = NullID;
                    }
                }

                // 2. Intersections of the H/V rail with existing lines
                for (const auto& line : sketch.lines) {
                    Point2D P = sketch.getPointPos(line.startPt);
                    Point2D Q = sketch.getPointPos(line.endPt);
                    Point2D ix;
                    if (nearH) {
                        double denom = Q.y - P.y;
                        if (std::fabs(denom) < 1e-10) continue;
                        double t = (tool_.firstPoint.y - P.y) / denom;
                        if (t < 0.0 || t > 1.0) continue;
                        ix = {P.x + t*(Q.x - P.x), tool_.firstPoint.y};
                    } else {
                        double denom = Q.x - P.x;
                        if (std::fabs(denom) < 1e-10) continue;
                        double t = (tool_.firstPoint.x - P.x) / denom;
                        if (t < 0.0 || t > 1.0) continue;
                        ix = {tool_.firstPoint.x, P.y + t*(Q.y - P.y)};
                    }
                    double d = distance(hvPos, ix);
                    if (d < bestDist) {
                        bestDist  = d;
                        hvPos     = ix;
                        hvPtID    = NullID;
                        hvCrossID = line.id;
                    }
                }

                // 3. Intersections of the H/V rail with circles
                for (const auto& circ : sketch.circles) {
                    Point2D c = sketch.getPointPos(circ.centerPt);
                    double r = circ.radius;
                    Point2D pts[2]; int nPts = 0;
                    if (nearH) {
                        double dy2 = tool_.firstPoint.y - c.y;
                        if (std::fabs(dy2) > r) continue;
                        double dxC = std::sqrt(r*r - dy2*dy2);
                        pts[nPts++] = {c.x + dxC, tool_.firstPoint.y};
                        pts[nPts++] = {c.x - dxC, tool_.firstPoint.y};
                    } else {
                        double dx2 = tool_.firstPoint.x - c.x;
                        if (std::fabs(dx2) > r) continue;
                        double dyC = std::sqrt(r*r - dx2*dx2);
                        pts[nPts++] = {tool_.firstPoint.x, c.y + dyC};
                        pts[nPts++] = {tool_.firstPoint.x, c.y - dyC};
                    }
                    for (int i = 0; i < nPts; i++) {
                        double d = distance(hvPos, pts[i]);
                        if (d < bestDist) {
                            bestDist  = d;
                            hvPos     = pts[i];
                            hvPtID    = NullID;
                            hvCrossID = circ.id;
                        }
                    }
                }

                // 4. Intersections of the H/V rail with arcs
                {
                    static constexpr double kPiD = 3.14159265358979323846;
                    auto normA = [](double a) {
                        a = std::fmod(a, 2.0*3.14159265358979323846);
                        if (a < 0.0) a += 2.0*3.14159265358979323846;
                        return a;
                    };
                    for (const auto& arc : sketch.arcs) {
                        Point2D c  = sketch.getPointPos(arc.centerPt);
                        Point2D sp = sketch.getPointPos(arc.startPt);
                        double r = distance(c, sp);
                        double nSA = normA(arc.startAngle);
                        double nEA = normA(arc.endAngle);
                        double sweep = nEA - nSA;
                        if (sweep <= 0.0) sweep += 2.0*kPiD;

                        Point2D pts[2]; int nPts = 0;
                        double angles[2];
                        if (nearH) {
                            double dy2 = tool_.firstPoint.y - c.y;
                            if (std::fabs(dy2) > r) continue;
                            double dxC = std::sqrt(r*r - dy2*dy2);
                            pts[nPts] = {c.x + dxC, tool_.firstPoint.y};
                            angles[nPts] = std::atan2(tool_.firstPoint.y - c.y,  dxC); nPts++;
                            pts[nPts] = {c.x - dxC, tool_.firstPoint.y};
                            angles[nPts] = std::atan2(tool_.firstPoint.y - c.y, -dxC); nPts++;
                        } else {
                            double dx2 = tool_.firstPoint.x - c.x;
                            if (std::fabs(dx2) > r) continue;
                            double dyC = std::sqrt(r*r - dx2*dx2);
                            pts[nPts] = {tool_.firstPoint.x, c.y + dyC};
                            angles[nPts] = std::atan2( dyC, tool_.firstPoint.x - c.x); nPts++;
                            pts[nPts] = {tool_.firstPoint.x, c.y - dyC};
                            angles[nPts] = std::atan2(-dyC, tool_.firstPoint.x - c.x); nPts++;
                        }
                        for (int i = 0; i < nPts; i++) {
                            double toA = normA(angles[i]) - nSA;
                            if (toA < 0.0) toA += 2.0*kPi;
                            if (toA >= sweep) continue; // outside arc
                            double d = distance(hvPos, pts[i]);
                            if (d < bestDist) {
                                bestDist  = d;
                                hvPos     = pts[i];
                                hvPtID    = NullID;
                                hvCrossID = arc.id;
                            }
                        }
                    }
                }

                // If a coincident point won, snap exactly to it
                if (hvPtID != NullID)
                    hvPos = sketch.getPointPos(hvPtID);

                // Record crossing entity for actionCompleted to add constraints
                hvCrossEntityID_ = hvCrossID;

                // Lock snap to the H/V position
                currentSnap_.position = hvPos;
                currentSnap_.pointID  = hvPtID;
                currentSnap_.curveID  = NullID;
                currentSnap_.type     = (hvPtID != NullID) ? SnapType::Point : SnapType::Grid;

                // Show H or V label at the snap indicator position
                float wb[3];
                plane.localToWorld((float)hvPos.x, (float)hvPos.y, wb[0], wb[1], wb[2]);
                float sbx, sby;
                if (worldToScreen(wb, view, proj, vpW, vpH, sbx, sby)) {
                    ImDrawList* dl = ImGui::GetForegroundDrawList();
                    const auto& sc = activeTheme().snapColor;
                    ImU32 col = IM_COL32((int)(sc[0]*255), (int)(sc[1]*255), (int)(sc[2]*255), 200);
                    dl->AddText(ImVec2(sbx + 7.0f, sby - 9.0f), col, nearH ? "H" : "V");
                }
            }
        }
    }

    // Inline dimension input — intercept number keys when circle tool has center placed
    // or fillet tool has vertex placed
    if (!io.WantCaptureKeyboard && !tool_.inlineInputActive &&
        ((tool_.type == ToolType::Circle && tool_.hasFirstPoint) ||
         (tool_.type == ToolType::Fillet && tool_.hasFirstPoint))) {
        // Check for number, decimal key press to activate inline input (main + numpad)
        auto activateInline = [&](char ch) {
            tool_.inlineInputActive = true;
            tool_.inlineInputBuf[0] = ch;
            tool_.inlineInputBuf[1] = '\0';
            tool_.inlineInputFocus = true;
        };
        for (int k = ImGuiKey_0; k <= ImGuiKey_9; k++) {
            if (ImGui::IsKeyPressed((ImGuiKey)k)) { activateInline((char)('0' + (k - ImGuiKey_0))); break; }
        }
        if (!tool_.inlineInputActive) {
            for (int k = ImGuiKey_Keypad0; k <= ImGuiKey_Keypad9; k++) {
                if (ImGui::IsKeyPressed((ImGuiKey)k)) { activateInline((char)('0' + (k - ImGuiKey_Keypad0))); break; }
            }
        }
        if (!tool_.inlineInputActive && (ImGui::IsKeyPressed(ImGuiKey_Period) || ImGui::IsKeyPressed(ImGuiKey_KeypadDecimal))) {
            activateInline('.');
        }
    }

    // Show inline dimension input floating window
    if (tool_.inlineInputActive) {
        ImVec2 mouse = io.MousePos;
        ImGui::SetNextWindowPos(ImVec2(mouse.x + 20, mouse.y - 10), ImGuiCond_Always);
        ImGui::SetNextWindowSize(ImVec2(120, 0));
        ImGui::Begin("##InlineDim", nullptr,
            ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize |
            ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoScrollbar |
            ImGuiWindowFlags_AlwaysAutoResize);

        ImGui::SetNextItemWidth(100);
        if (tool_.inlineInputFocus) {
            ImGui::SetKeyboardFocusHere();
            tool_.inlineInputFocus = false;
        }

        // Callback to move cursor to end and clear selection on first focus
        auto cursorEndCb = [](ImGuiInputTextCallbackData* data) -> int {
            data->CursorPos = data->BufTextLen;
            data->SelectionStart = data->SelectionEnd = data->BufTextLen;
            return 0;
        };

        bool submitted = ImGui::InputText("##inlineDimInput", tool_.inlineInputBuf,
            sizeof(tool_.inlineInputBuf),
            ImGuiInputTextFlags_EnterReturnsTrue | ImGuiInputTextFlags_CallbackAlways,
            cursorEndCb);

        if (submitted) {
            std::string unit;
            float inputVal = 0;
            float valueMm = parseUnitInput(tool_.inlineInputBuf, unit, inputVal);
            if (tool_.type == ToolType::Circle) {
                if (valueMm > 0.001f) {
                    float radius = valueMm * 0.5f;
                    EntityID circID = sketch.addCircle(tool_.firstPointID, radius);
                    sketch.addConstraint(ConstraintType::Diameter, circID, NullID, valueMm, false);
                    lastSketchDof_ = solver_.solve(sketch).dof;
                    history_.pushState(sketch);
                    tool_.reset();
                } else {
                    snprintf(sketchMsg_, sizeof(sketchMsg_), "Enter a positive diameter");
                    sketchMsgTimer_ = 2.0f;
                    tool_.inlineInputActive = false;
                    tool_.inlineInputBuf[0] = '\0';
                }
            } else if (tool_.type == ToolType::Fillet) {
                if (valueMm > 0.001f) {
                    bool ok = applyFillet(sketch, filletTool_, tool_.firstPointID, valueMm);
                    if (ok) {
                        lastSketchDof_ = solver_.solve(sketch).dof;
                        history_.pushState(sketch);
                    }
                    tool_.reset();
                    filletTool_.reset();
                } else {
                    tool_.inlineInputActive = false;
                    tool_.inlineInputBuf[0] = '\0';
                }
            }
        }

        if (ImGui::IsKeyPressed(ImGuiKey_Escape)) {
            if (tool_.type == ToolType::Fillet && tool_.hasFirstPoint) {
                // Cancel the whole fillet vertex selection, stay in fillet tool
                switchTool(ToolType::Fillet);
            } else {
                tool_.inlineInputActive = false;
                tool_.inlineInputBuf[0] = '\0';
            }
        }

        ImGui::End();
    }

    // Sketch status message overlay (warnings, constraint feedback, etc.)
    if (sketchMsgTimer_ > 0.0f) {
        sketchMsgTimer_ -= io.DeltaTime;
        ImVec2 displaySize = io.DisplaySize;
        ImGui::SetNextWindowPos(ImVec2(displaySize.x * 0.5f, displaySize.y - 60.0f),
                                ImGuiCond_Always, ImVec2(0.5f, 0.5f));
        ImGui::SetNextWindowBgAlpha(0.78f);
        ImGui::Begin("##SketchMsg", nullptr,
            ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize |
            ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoScrollbar |
            ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoFocusOnAppearing |
            ImGuiWindowFlags_NoInputs);
        ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(1.0f, 0.85f, 0.3f, 1.0f));
        ImGui::Text("%s", sketchMsg_);
        ImGui::PopStyleColor();
        ImGui::End();
    }

    // Keyboard shortcuts
    if (!io.WantCaptureKeyboard && !tool_.inlineInputActive) {
        // Undo/redo
        if (io.KeyCtrl && ImGui::IsKeyPressed(ImGuiKey_Z)) {
            if (io.KeyShift) { history_.redo(sketch); }
            else { history_.undo(sketch); }
            switchTool(tool_.type);
        }
        if (io.KeyCtrl && ImGui::IsKeyPressed(ImGuiKey_Y)) {
            history_.redo(sketch); switchTool(tool_.type);
        }

        if (ImGui::IsKeyPressed(ImGuiKey_P) && !io.KeyShift) { switchTool(ToolType::Point); }
        if (ImGui::IsKeyPressed(ImGuiKey_L)) { switchTool(ToolType::Line); }
        if (ImGui::IsKeyPressed(ImGuiKey_C)) { switchTool(ToolType::Circle); }
        if (ImGui::IsKeyPressed(ImGuiKey_R) && !io.KeyShift) { switchTool(ToolType::Rectangle); }
        if (ImGui::IsKeyPressed(ImGuiKey_A) && !io.KeyShift) { switchTool(ToolType::Arc3Point); }
        if (ImGui::IsKeyPressed(ImGuiKey_A) && io.KeyShift) { switchTool(ToolType::ArcCenter); }
        if (ImGui::IsKeyPressed(ImGuiKey_R) && io.KeyShift) { switchTool(ToolType::CenterRect); }

        if (ImGui::IsKeyPressed(ImGuiKey_E) && tool_.type != ToolType::Extrude) {
            enterExtrudeMode();
        }
        if (ImGui::IsKeyPressed(ImGuiKey_V) && tool_.type != ToolType::Revolve) {
            enterRevolveMode();
        }

        if (ImGui::IsKeyPressed(ImGuiKey_N)) {
            orientCameraToPlane(activePlane());
        }

        if (ImGui::IsKeyPressed(ImGuiKey_D)) { switchTool(ToolType::Dimension); }
        if (ImGui::IsKeyPressed(ImGuiKey_F)) { switchTool(ToolType::Fillet); }

        if (ImGui::IsKeyPressed(ImGuiKey_Delete) || ImGui::IsKeyPressed(ImGuiKey_Backspace)) {
            handleDeletion(sketch);
        }

        if (ImGui::IsKeyPressed(ImGuiKey_Escape)) {
            if (tool_.hasFirstPoint || arcTool_.clickCount > 0) {
                // Cancel in-progress tool action, stay in same tool
                switchTool(tool_.type);
            } else if (tool_.type != ToolType::None) {
                switchTool(ToolType::None);
            } else {
                finishSketch();
                return;
            }
        }
    }

    if (mouseOverUI) return;

    // Active point drag
    if (selection_.dragMode == SelectionDragMode::PointDrag) {
        handleDrag(sketch);
    }

    // (Dimension label drag is handled above, before the dim tool early return)

    // Double-click: end continuous line chain (Fusion 360 parity)
    if (ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left)) {
        if (tool_.type == ToolType::Line && tool_.hasFirstPoint) {
            switchTool(ToolType::None);
            goto skipLeftClick;
        }
    }

    // Left click
    if (ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
        if (tool_.type != ToolType::None) {
            // Always use currentSnap_.position: for type=None it equals cursorLocal_,
            // but H/V snap may have overridden it to the guide-locked position.
            Point2D effectivePos = currentSnap_.position;
            handleToolAction(sketch, effectivePos);
        } else {
            handleSelection(sketch, io.KeyCtrl);
        }
    }
    skipLeftClick:;

    // Start drag (dim drag is handled above; here handle point drag, box/lasso)
    if (tool_.type == ToolType::None &&
        selection_.dragMode == SelectionDragMode::None &&
        ImGui::IsMouseDragging(ImGuiMouseButton_Left, 3.0f)) {

        // Check if drag started on a selected point → PointDrag
        bool startedOnPoint = false;
        {
            for (const auto& e : selection_.selected) {
                if (e.type == HitType::Point) {
                    // Don't allow dragging projected (locked) points
                    const PointEntity* ptEnt = sketch.findPoint(e.id);
                    if (ptEnt && ptEnt->projected) continue;
                    Point2D ptPos = sketch.getPointPos(e.id);
                    double ptDist = distance(ptPos, cursorLocal_) * apparentScale;
                    if (ptDist < 20.0f) {
                        selection_.dragMode = SelectionDragMode::PointDrag;
                        selection_.dragPointID = e.id;
                        selection_.dragStarted = false;
                        startedOnPoint = true;
                        break;
                    }
                }
            }
        }

        if (!startedOnPoint) {
            // Check if we hit anything at the drag start point — if so, don't start box/lasso
            HitResult hitAtAnchor = hitTest(selection_.dragAnchor, apparentScale, sketch, 10.0f);
            if (hitAtAnchor.type == HitType::None) {
                if (io.KeyAlt) {
                    selection_.dragMode = SelectionDragMode::LassoSelect;
                    selection_.lassoPoints.clear();
                    selection_.lassoPoints.push_back(selection_.dragAnchor);
                } else {
                    selection_.dragMode = SelectionDragMode::BoxSelect;
                }
            }
        }
    }

    // During lasso drag: append cursor with distance threshold
    if (selection_.dragMode == SelectionDragMode::LassoSelect) {
        if (!selection_.lassoPoints.empty()) {
            double dist = distance(selection_.lassoPoints.back(), cursorLocal_);
            float threshold = 5.0f / apparentScale; // ~5 screen pixels
            if (dist > threshold) {
                selection_.lassoPoints.push_back(cursorLocal_);
            }
        }
    }

    // End drag
    if (ImGui::IsMouseReleased(ImGuiMouseButton_Left)) {
        if (selection_.dragMode == SelectionDragMode::PointDrag) {
            if (selection_.dragStarted) {
                history_.pushState(sketch);
            }
            selection_.dragMode = SelectionDragMode::None;
            selection_.dragPointID = NullID;
            selection_.dragStarted = false;

        } else if (selection_.dragMode == SelectionDragMode::DimDrag) {
            if (selection_.dragStarted) {
                history_.pushState(sketch);
            }
            selection_.dragMode = SelectionDragMode::None;
            selection_.dragDimID = NullID;
            selection_.dragStarted = false;

        } else if (selection_.dragMode == SelectionDragMode::BoxSelect) {
            // Select entities contained in screen-space box
            Point2D a = selection_.dragAnchorScreen;
            Point2D b = {io.MousePos.x, io.MousePos.y};
            Point2D mn = {std::min(a.x, b.x), std::min(a.y, b.y)};
            Point2D mx = {std::max(a.x, b.x), std::max(a.y, b.y)};

            // Helper: project a local sketch point to screen space
            auto localToScreen = [&](double lx, double ly, float& sx, float& sy) -> bool {
                float wx, wy, wz;
                plane.localToWorld(f(lx), f(ly), wx, wy, wz);
                float w3[3] = {wx, wy, wz};
                return worldToScreen(w3, view, proj, vpW, vpH, sx, sy);
            };

            auto screenPtInRect = [&](double lx, double ly) -> bool {
                float sx, sy;
                if (!localToScreen(lx, ly, sx, sy)) return false;
                return sx >= mn.x && sx <= mx.x && sy >= mn.y && sy <= mx.y;
            };

            if (!io.KeyCtrl) selection_.selected.clear();

            for (const auto& pt : sketch.points) {
                if (screenPtInRect(pt.x, pt.y))
                    selection_.addToSelection(HitType::Point, pt.id);
            }
            for (const auto& line : sketch.lines) {
                Point2D la = sketch.getPointPos(line.startPt);
                Point2D lb = sketch.getPointPos(line.endPt);
                if (screenPtInRect(la.x, la.y) && screenPtInRect(lb.x, lb.y))
                    selection_.addToSelection(HitType::Line, line.id);
            }
            for (const auto& circle : sketch.circles) {
                Point2D center = sketch.getPointPos(circle.centerPt);
                // Check center and 4 cardinal points on the circle
                if (screenPtInRect(center.x, center.y) &&
                    screenPtInRect(center.x + circle.radius, center.y) &&
                    screenPtInRect(center.x - circle.radius, center.y) &&
                    screenPtInRect(center.x, center.y + circle.radius) &&
                    screenPtInRect(center.x, center.y - circle.radius))
                    selection_.addToSelection(HitType::Circle, circle.id);
            }
            for (const auto& arc : sketch.arcs) {
                Point2D sp = sketch.getPointPos(arc.startPt);
                Point2D ep = sketch.getPointPos(arc.endPt);
                Point2D center = sketch.getPointPos(arc.centerPt);
                if (screenPtInRect(sp.x, sp.y) && screenPtInRect(ep.x, ep.y) && screenPtInRect(center.x, center.y))
                    selection_.addToSelection(HitType::Arc, arc.id);
            }
            for (const auto& el : sketch.ellipses) {
                Point2D center = sketch.getPointPos(el.centerPt);
                // Check 4 rotated extremal points
                double cosR = std::cos(el.rotation), sinR = std::sin(el.rotation);
                bool allIn = true;
                double testPts[4][2] = {
                    { el.semiMajor, 0.0}, {-el.semiMajor, 0.0},
                    {0.0,  el.semiMinor}, {0.0, -el.semiMinor}
                };
                for (auto& tp : testPts) {
                    double rx = tp[0]*cosR - tp[1]*sinR + center.x;
                    double ry = tp[0]*sinR + tp[1]*cosR + center.y;
                    if (!screenPtInRect(rx, ry)) { allIn = false; break; }
                }
                if (allIn) selection_.addToSelection(HitType::Ellipse, el.id);
            }
            for (const auto& ea : sketch.ellipseArcs) {
                Point2D sp = sketch.getPointPos(ea.startPt);
                Point2D ep = sketch.getPointPos(ea.endPt);
                Point2D cp = sketch.getPointPos(ea.centerPt);
                if (screenPtInRect(sp.x, sp.y) && screenPtInRect(ep.x, ep.y) && screenPtInRect(cp.x, cp.y))
                    selection_.addToSelection(HitType::EllipseArc, ea.id);
            }
            for (const auto& sp : sketch.splines) {
                bool allInside = true;
                for (auto ptID : sp.controlPtIDs) {
                    Point2D p = sketch.getPointPos(ptID);
                    if (!screenPtInRect(p.x, p.y)) { allInside = false; break; }
                }
                if (allInside) selection_.addToSelection(HitType::Spline, sp.id);
            }

            selection_.dragMode = SelectionDragMode::None;

        } else if (selection_.dragMode == SelectionDragMode::LassoSelect) {
            // Add current cursor as final point and close polygon
            selection_.lassoPoints.push_back(cursorLocal_);
            const auto& poly = selection_.lassoPoints;

            if (!io.KeyCtrl) selection_.selected.clear();

            if (poly.size() >= 3) {
                for (const auto& pt : sketch.points) {
                    if (pointInPolygon({pt.x, pt.y}, poly))
                        selection_.addToSelection(HitType::Point, pt.id);
                }
                for (const auto& line : sketch.lines) {
                    Point2D la = sketch.getPointPos(line.startPt);
                    Point2D lb = sketch.getPointPos(line.endPt);
                    if (segmentInPolygon(la, lb, poly))
                        selection_.addToSelection(HitType::Line, line.id);
                }
                for (const auto& circle : sketch.circles) {
                    Point2D center = sketch.getPointPos(circle.centerPt);
                    if (circleInPolygon(center, circle.radius, poly))
                        selection_.addToSelection(HitType::Circle, circle.id);
                }
                for (const auto& arc : sketch.arcs) {
                    Point2D sp = sketch.getPointPos(arc.startPt);
                    Point2D ep = sketch.getPointPos(arc.endPt);
                    Point2D cp = sketch.getPointPos(arc.centerPt);
                    if (pointInPolygon(sp, poly) && pointInPolygon(ep, poly) && pointInPolygon(cp, poly))
                        selection_.addToSelection(HitType::Arc, arc.id);
                }
                for (const auto& el : sketch.ellipses) {
                    Point2D center = sketch.getPointPos(el.centerPt);
                    double cosR = std::cos(el.rotation), sinR = std::sin(el.rotation);
                    bool allIn = pointInPolygon(center, poly);
                    if (allIn) {
                        for (int si = 0; si < 16 && allIn; si++) {
                            double a = 2.0 * 3.14159265358979 * si / 16.0;
                            double ex = el.semiMajor * std::cos(a), ey = el.semiMinor * std::sin(a);
                            Point2D p = {center.x + ex*cosR - ey*sinR, center.y + ex*sinR + ey*cosR};
                            if (!pointInPolygon(p, poly)) allIn = false;
                        }
                    }
                    if (allIn) selection_.addToSelection(HitType::Ellipse, el.id);
                }
                for (const auto& ea : sketch.ellipseArcs) {
                    Point2D sp = sketch.getPointPos(ea.startPt);
                    Point2D ep = sketch.getPointPos(ea.endPt);
                    Point2D cp = sketch.getPointPos(ea.centerPt);
                    if (pointInPolygon(sp, poly) && pointInPolygon(ep, poly) && pointInPolygon(cp, poly))
                        selection_.addToSelection(HitType::EllipseArc, ea.id);
                }
                for (const auto& spl : sketch.splines) {
                    bool allInside = true;
                    for (auto ptID : spl.controlPtIDs) {
                        Point2D p = sketch.getPointPos(ptID);
                        if (!pointInPolygon(p, poly)) { allInside = false; break; }
                    }
                    if (allInside) selection_.addToSelection(HitType::Spline, spl.id);
                }
            }

            selection_.lassoPoints.clear();
            selection_.dragMode = SelectionDragMode::None;
        }
    }

    // Right click: cancel current action, then tool, then selection
    if (ImGui::IsMouseClicked(ImGuiMouseButton_Right)) {
        if (tool_.hasFirstPoint || arcTool_.clickCount > 0) {
            switchTool(tool_.type);
        } else if (selection_.dragMode == SelectionDragMode::PointDrag) {
            // Cancel point drag: undo the move if it already started
            if (selection_.dragStarted) history_.undo(sketch);
            selection_.dragMode = SelectionDragMode::None;
            selection_.dragPointID = NullID;
            selection_.dragStarted = false;
        } else if (selection_.dragMode != SelectionDragMode::None) {
            selection_.dragMode = SelectionDragMode::None;
            selection_.lassoPoints.clear();
        } else if (tool_.type != ToolType::None) {
            switchTool(ToolType::None);
        } else {
            selection_.clear();
        }
    }
}

void App::handleToolAction(Sketch& sketch, Point2D localPos) {
    EntityID snapPtID = currentSnap_.pointID;
    EntityID snapCurveID = currentSnap_.curveID;
    bool snapWasTangent = (currentSnap_.type == SnapType::Tangent);
    // Save before handleLineTool overwrites it with the end-point curve
    EntityID startCurveID = tool_.tangentSourceID;
    bool actionCompleted = false;

    // Capture before dispatch — rect/center-rect handlers call tool_.reset() on completion
    ToolType dispatchedToolType = tool_.type;
    EntityID dispatchedFirstPtID = tool_.firstPointID;

    switch (tool_.type) {
        case ToolType::Point:
            actionCompleted = handlePointTool(sketch, localPos, snapPtID);
            break;
        case ToolType::Line:
            actionCompleted = handleLineTool(sketch, tool_, localPos, snapPtID, snapCurveID);
            break;
        case ToolType::Circle: {
            // Count circles before/after: handleCircleTool always returns true on
            // second click even when the radius is too small and nothing was created.
            size_t circlesBefore = sketch.circles.size();
            actionCompleted = handleCircleTool(sketch, tool_, localPos, snapPtID);
            if (actionCompleted && sketch.circles.size() == circlesBefore) {
                snprintf(sketchMsg_, sizeof(sketchMsg_), "Radius too small");
                sketchMsgTimer_ = 2.0f;
                actionCompleted = false; // suppress spurious history push
            }
            break;
        }
        case ToolType::Rectangle: {
            bool hadFirst = tool_.hasFirstPoint;
            actionCompleted = handleRectangleTool(sketch, tool_, localPos, snapPtID);
            if (hadFirst && !actionCompleted && !tool_.hasFirstPoint) {
                snprintf(sketchMsg_, sizeof(sketchMsg_), "Rectangle too small — click further away");
                sketchMsgTimer_ = 2.0f;
            }
            break;
        }
        case ToolType::Arc3Point: {
            int prevCount = arcTool_.clickCount;
            actionCompleted = handleArc3PointTool(sketch, arcTool_, localPos, snapPtID);
            if (prevCount == 2 && !actionCompleted && arcTool_.clickCount == 0) {
                snprintf(sketchMsg_, sizeof(sketchMsg_), "Points are collinear — arc not placed");
                sketchMsgTimer_ = 2.5f;
            }
            break;
        }
        case ToolType::ArcCenter: {
            int prevCount = arcTool_.clickCount;
            actionCompleted = handleArcCenterTool(sketch, arcTool_, localPos, snapPtID);
            if (prevCount == 2 && !actionCompleted && arcTool_.clickCount == 0) {
                snprintf(sketchMsg_, sizeof(sketchMsg_), "Start and center are coincident — arc not placed");
                sketchMsgTimer_ = 2.5f;
            }
            break;
        }
        case ToolType::CenterRect:
            actionCompleted = handleCenterRectTool(sketch, tool_, localPos, snapPtID);
            break;
        case ToolType::Fillet:
            if (!tool_.hasFirstPoint) {
                if (snapPtID == NullID) {
                    snprintf(sketchMsg_, sizeof(sketchMsg_), "Click on a vertex to fillet");
                    sketchMsgTimer_ = 2.0f;
                    break;
                }
                if (handleFilletVertexClick(sketch, filletTool_, snapPtID)) {
                    tool_.firstPointID = snapPtID;
                    tool_.firstPoint = sketch.getPointPos(snapPtID);
                    tool_.hasFirstPoint = true;
                    tool_.inlineInputActive = true;
                    tool_.inlineInputFocus = true;
                } else {
                    snprintf(sketchMsg_, sizeof(sketchMsg_),
                             "Vertex must connect exactly 2 lines or arcs");
                    sketchMsgTimer_ = 2.5f;
                }
            }
            break;
        case ToolType::Dimension: {
            handleDimToolClick(sketch);
            break;
        }
        case ToolType::None:
        case ToolType::Extrude:
            break;
    }

    if (actionCompleted) {
        if (dispatchedToolType == ToolType::Line && !sketch.lines.empty()) {
            EntityID lastLineID = sketch.lines.back().id;
            EntityID startPtID  = sketch.lines.back().startPt;

            // Auto-tangent only when the user explicitly snapped to the geometric tangent
            // point on a circle/arc (SnapType::Tangent). A plain NearestOnCurve snap means
            // the user just wants the endpoint to land on the curve — no tangent implied.
            EntityID endPtID = sketch.lines.back().endPt;

            // Auto-apply PointOnCircle for endpoints snapped to circle/arc curves.
            // Only when a new point was created (curve snap, no existing pointID).
            auto addPointOnCircle = [&](EntityID ptID, EntityID curveID) {
                if (curveID == NullID || ptID == NullID) return;
                bool isCircle = sketch.findCircle(curveID) != nullptr;
                bool isArc    = !isCircle && sketch.findArc(curveID) != nullptr;
                if (!isCircle && !isArc) return;
                // Check for existing PointOnCircle to avoid duplicates
                for (const auto& c : sketch.constraints) {
                    if (c.type == ConstraintType::PointOnCircle &&
                        c.entityA == ptID && c.entityB == curveID) return;
                }
                sketch.addConstraint(ConstraintType::PointOnCircle, ptID, curveID, 0.0f, true);
            };

            // Start point: startCurveID was captured before handleLineTool ran
            // (only set when first click had no existing pointID to snap to)
            addPointOnCircle(startPtID, startCurveID);

            // End point: snapCurveID is the curve snapped at this click
            if (!snapWasTangent)
                addPointOnCircle(endPtID, snapCurveID);

            if (snapWasTangent && snapCurveID != NullID) {
                sketch.addConstraint(ConstraintType::Tangent, lastLineID, snapCurveID, 0.0f, true);
            } else {
                // Apply H/V auto-constraints based on the line's actual geometry.
                auto pending = detectLineAutoConstraints(sketch, lastLineID);
                for (const auto& pc : pending) {
                    sketch.addConstraint(pc.type, pc.entityA, pc.entityB, pc.value, true);
                }

                // If the H/V rail crossed an existing entity, add the appropriate constraint.
                // Line cross → PointOnLine; circle/arc cross → PointOnCircle.
                // (When H/V rail is active it clears snapCurveID, so this is the only
                //  path that can add PointOnCircle for the end point in H/V mode.)
                if (hvCrossEntityID_ != NullID) {
                    if (sketch.findLine(hvCrossEntityID_)) {
                        sketch.addConstraint(ConstraintType::PointOnLine, endPtID, hvCrossEntityID_, 0.0f, true);
                    } else {
                        addPointOnCircle(endPtID, hvCrossEntityID_);
                    }
                }
                hvCrossEntityID_ = NullID;
            }

            // Temporarily pin the start point so auto-constraints only move the new
            // free endpoint, leaving previously placed geometry undisturbed.
            PointEntity* startPt = nullptr;
            for (auto& pt : sketch.points) {
                if (pt.id == startPtID) { startPt = &pt; break; }
            }
            bool wasPinned = startPt && startPt->projected;
            if (startPt) startPt->projected = true;
            lastSketchDof_ = solver_.solve(sketch).dof;
            if (startPt) startPt->projected = wasPinned;
        } else if ((dispatchedToolType == ToolType::Rectangle ||
                    dispatchedToolType == ToolType::CenterRect) &&
                   dispatchedFirstPtID != NullID) {
            // Pin the first corner so H/V constraints settle without drifting the anchor.
            PointEntity* anchorPt = sketch.findPoint(dispatchedFirstPtID);
            bool wasPinned = anchorPt && anchorPt->projected;
            if (anchorPt) anchorPt->projected = true;
            lastSketchDof_ = solver_.solve(sketch).dof;
            if (anchorPt) anchorPt->projected = wasPinned;
        } else {
            lastSketchDof_ = solver_.solve(sketch).dof;
        }

        // Refresh tool's first point from solved position (solver may have moved it)
        if (tool_.hasFirstPoint && tool_.firstPointID != NullID) {
            tool_.firstPoint = sketch.getPointPos(tool_.firstPointID);
        }

        history_.pushState(sketch);
    }
}

void App::switchTool(ToolType newTool) {
    if (hasActiveSketch()) {
        Sketch& sketch = activeSketch();
        // Remove orphan points left by incomplete tools
        auto isOrphan = [&](EntityID ptID) {
            if (ptID == NullID) return false;
            const PointEntity* pt = sketch.findPoint(ptID);
            if (pt && pt->projected) return false;
            if (sketch.isPointReferenced(ptID)) return false;
            for (const auto& co : sketch.constraints)
                if (co.entityA == ptID || co.entityB == ptID || co.entityC == ptID) return false;
            return true;
        };
        if (tool_.hasFirstPoint && isOrphan(tool_.firstPointID))
            sketch.removePoint(tool_.firstPointID);
        if (arcTool_.clickCount >= 1 && isOrphan(arcTool_.point1ID))
            sketch.removePoint(arcTool_.point1ID);
        if (arcTool_.clickCount >= 2 && isOrphan(arcTool_.point2ID))
            sketch.removePoint(arcTool_.point2ID);
    }
    tool_.type = newTool;
    tool_.reset();
    arcTool_.reset();
    filletTool_.reset();
    dimTool_.reset();
    selection_.clear();
    sketchMsg_[0] = '\0';
    sketchMsgTimer_ = 0.0f;
}

void App::handleSelection(Sketch& sketch, bool ctrlHeld) {
    int w, h;
    glfwGetFramebufferSize(window_, &w, &h);
    float view[16], proj[16];
    getViewProj(w, h, view, proj);
    float apparentScale = computeApparentScale(activePlane(), view, proj, (float)w, (float)h);

    // Check dimension label rects first (they're drawn on top)
    ImVec2 mouse = ImGui::GetIO().MousePos;
    EntityID dimHitID = NullID;
    for (const auto& r : dimLabelRects_) {
        if (r.sketchPlaneIndex != activeSketchPlane_) continue;
        if (mouse.x >= r.x0 && mouse.x <= r.x1 && mouse.y >= r.y0 && mouse.y <= r.y1) {
            dimHitID = r.constraintID;
            break;
        }
    }

    if (dimHitID != NullID) {
        if (ctrlHeld) {
            selection_.toggleSelection(HitType::Dimension, dimHitID);
        } else {
            selection_.select(HitType::Dimension, dimHitID);
        }
        // Double-click: enter editing mode for this existing constraint
        Constraint* cc = sketch.findConstraint(dimHitID);
        if (cc && !ctrlHeld && ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left)) {
            tool_.type = ToolType::Dimension;
            dimTool_.reset();
            dimTool_.phase = DimToolState::Editing;
            dimTool_.constraintID = dimHitID;
            dimTool_.editingExisting = true;
            dimTool_.driven = cc->driven;
            // Infer selType from constraint type
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
            // Fill input buffer with current value
            if (cc->type == ConstraintType::Angle)
                formatAngleText(dimTool_.inputBuf, sizeof(dimTool_.inputBuf), f(cc->value));
            else if (!cc->inputUnit.empty())
                snprintf(dimTool_.inputBuf, sizeof(dimTool_.inputBuf), "%.4g%s", cc->inputValue, cc->inputUnit.c_str());
            else
                snprintf(dimTool_.inputBuf, sizeof(dimTool_.inputBuf), "%.4gmm", cc->value);
            dimTool_.focusNeeded = true;
        }
        return;
    }

    HitResult hit = hitTest(cursorLocal_, apparentScale, sketch, 10.0f);

    if (hit.type != HitType::None) {
        if (ctrlHeld) {
            selection_.toggleSelection(hit.type, hit.entityID);
        } else {
            selection_.select(hit.type, hit.entityID);
        }
        // If we selected geometry and had a dim editing session, reset it
        if (dimTool_.editingExisting) dimTool_.reset();
    } else if (!ctrlHeld) {
        selection_.clear();
        if (dimTool_.editingExisting) dimTool_.reset();
        // Store drag anchor for potential box/lasso select
        selection_.dragAnchor = cursorLocal_;
        ImGuiIO& anchorIO = ImGui::GetIO();
        selection_.dragAnchorScreen = {anchorIO.MousePos.x, anchorIO.MousePos.y};
    }
}

void App::handleDrag(Sketch& sketch) {
    Point2D dragPos = (currentSnap_.type != SnapType::None)
        ? currentSnap_.position : cursorLocal_;

    PointEntity* pt = sketch.findPoint(selection_.dragPointID);
    if (!pt || pt->projected) return;

    pt->x = dragPos.x;
    pt->y = dragPos.y;
    selection_.dragStarted = true;
    lastSketchDof_ = solver_.solve(sketch, selection_.dragPointID).dof;
}

void App::handleDeletion(Sketch& sketch) {
    if (!selection_.hasSelection()) return;

    // Copy selection since removing entities may invalidate iterators
    auto toDelete = selection_.selected;
    for (const auto& e : toDelete) {
        // Skip projected (locked) entities
        if (e.type == HitType::Point) { auto* p = sketch.findPoint(e.id); if (p && p->projected) continue; }
        if (e.type == HitType::Line) { auto* l = sketch.findLine(e.id); if (l && l->projected) continue; }
        if (e.type == HitType::Circle) { auto* c = sketch.findCircle(e.id); if (c && c->projected) continue; }
        if (e.type == HitType::Arc) { auto* a = sketch.findArc(e.id); if (a && a->projected) continue; }
        if (e.type == HitType::Ellipse) { auto* el = sketch.findEllipse(e.id); if (el && el->projected) continue; }
        if (e.type == HitType::EllipseArc) { auto* ea = sketch.findEllipseArc(e.id); if (ea && ea->projected) continue; }
        if (e.type == HitType::Spline) { auto* sp = sketch.findSpline(e.id); if (sp && sp->projected) continue; }
        switch (e.type) {
            case HitType::Point: sketch.removePoint(e.id); break;
            case HitType::Line: sketch.removeLine(e.id); break;
            case HitType::Circle: sketch.removeCircle(e.id); break;
            case HitType::Arc: sketch.removeArc(e.id); break;
            case HitType::Ellipse: sketch.removeEllipse(e.id); break;
            case HitType::EllipseArc: sketch.removeEllipseArc(e.id); break;
            case HitType::Spline: sketch.removeSpline(e.id); break;
            case HitType::Dimension: sketch.removeConstraint(e.id); break;
            default: break;
        }
    }

    selection_.clear();
    if (dimTool_.editingExisting) dimTool_.reset();
    history_.pushState(sketch);
}

} // namespace shitcad
