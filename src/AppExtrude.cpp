#include "App.h"
#include "ProfileDetector.h"
#include "Extrude.h"
#include "FacePicker.h"
#include "ExtrudeTool.h"
#include "FeatureReplay.h"
#include "UnitUtils.h"
#include <BRep_Builder.hxx>
#include <TopoDS_Compound.hxx>
#include <BRepAlgoAPI_Cut.hxx>
#include <BRepAlgoAPI_Fuse.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Solid.hxx>

#include <glad/gl.h>
#include <cstdio>
#include <cmath>
#include <algorithm>
#include <string>

namespace shitcad {

void App::enterExtrudeMode() {
    int planeIdx = -1;
    std::vector<ClosedProfile> profiles;
    if (!findProfilePlane(planeIdx, profiles)) {
        fprintf(stderr, "No closed profiles found in any sketch\n");
        return;
    }

    extrudeTool_.reset();
    beginProfileTool(ToolType::Extrude, extrudeTool_, planeIdx, std::move(profiles));

    // Auto-select all profiles if there's only one
    if (extrudeTool_.allProfiles.size() == 1) {
        extrudeTool_.selectedProfileIndices.insert(0);
    }
}

void App::handleExtrudeInput(float vpW, float vpH) {
    if (!hasExtrudeSketch()) return;

    const SketchPlane& plane = extrudePlane();

    // Project mouse to sketch plane
    {
        int w, h;
        framebufferSize(w, h);
        float view[16], proj[16];
        getViewProj(w, h, view, proj);

        float rayOrig[3], rayDir[3];
        screenToRay(in_.mouseX, in_.mouseY, 0, 0, vpW, vpH, view, proj, rayOrig, rayDir);

        float lx, ly, t;
        if (plane.rayIntersect(rayOrig, rayDir, lx, ly, t)) {
            cursorLocal_ = {lx, ly};
        }
    }

    // Keyboard
    if (in_.keyPressed(Key::Escape)) {
        cancelExtrude();
        return;
    }
    if (in_.keyPressed(Key::Enter) || in_.keyPressed(Key::KeypadEnter)) {
        if (extrudeTool_.hasSelectedProfiles()) {
            commitExtrude();
            return;
        }
    }


    // Profile selection: click to toggle (disabled during handle drag)
    if (!extrudeTool_.isDragging) {
        if (in_.mouseClicked(MouseButton::Left)) {
            int hitIdx = hitTestProfile(extrudeSketch(), extrudeTool_.allProfiles, cursorLocal_,
                                        extrudeTool_.renderCache);
            if (hitIdx >= 0) {
                if (extrudeTool_.selectedProfileIndices.count(hitIdx))
                    extrudeTool_.selectedProfileIndices.erase(hitIdx);
                else
                    extrudeTool_.selectedProfileIndices.insert(hitIdx);
                extrudeTool_.previewDirty = true;
            }
        }
    }

    // Drag handle hit test: only start drag when clicking near the handle arrow tip
    if (extrudeTool_.hasSelectedProfiles() && !extrudeTool_.isDragging &&
        extrudeTool_.handleVisible && in_.mouseClicked(MouseButton::Left)) {
        // Compute handle tip position in screen space
        int w, h;
        framebufferSize(w, h);
        float view[16], proj[16];
        getViewProj(w, h, view, proj);

        const float* n = plane.normal;
        float tipWorld[3] = {
            extrudeTool_.handleBaseWorld[0] + n[0] * extrudeTool_.height,
            extrudeTool_.handleBaseWorld[1] + n[1] * extrudeTool_.height,
            extrudeTool_.handleBaseWorld[2] + n[2] * extrudeTool_.height
        };
        float tipSx, tipSy;
        if (worldToScreen(tipWorld, view, proj, vpW, vpH, tipSx, tipSy)) {
            float dx = in_.mouseX - tipSx;
            float dy = in_.mouseY - tipSy;
            float handleRadius = 20.0f; // pixels
            if (dx*dx + dy*dy < handleRadius * handleRadius) {
                extrudeTool_.isDragging = true;
                extrudeTool_.dragStartMouseX = in_.mouseX;
                extrudeTool_.dragStartMouseY = in_.mouseY;
                extrudeTool_.dragStartHeight = extrudeTool_.height;
            }
        }
    }

    // During drag: update height (throttled preview rebuild)
    if (extrudeTool_.isDragging) {
        // Compute pixels-per-unit along the extrude normal for 1:1 screen mapping
        int w, h;
        framebufferSize(w, h);
        float view[16], proj[16];
        getViewProj(w, h, view, proj);

        const float* n = extrudePlane().normal;
        const float* base = extrudeTool_.handleBaseWorld;

        // Project base and base+1unit along normal to screen space
        float baseSx, baseSy;
        float unitWorld[3] = { base[0] + n[0], base[1] + n[1], base[2] + n[2] };
        float unitSx, unitSy;
        float pxPerUnit = 200.0f; // fallback
        if (worldToScreen(base, view, proj, vpW, vpH, baseSx, baseSy) &&
            worldToScreen(unitWorld, view, proj, vpW, vpH, unitSx, unitSy)) {
            // Compute the screen-space direction & scale of the normal
            float dsx = unitSx - baseSx;
            float dsy = unitSy - baseSy;
            pxPerUnit = std::sqrt(dsx * dsx + dsy * dsy);
            if (pxPerUnit < 0.1f) pxPerUnit = 0.1f;
        }

        // Mouse delta projected onto the normal's screen direction
        float mouseDx = in_.mouseX - extrudeTool_.dragStartMouseX;
        float mouseDy = in_.mouseY - extrudeTool_.dragStartMouseY;

        // Screen direction of the normal (base → tip)
        float tipAtStart[3] = {
            base[0] + n[0] * extrudeTool_.dragStartHeight,
            base[1] + n[1] * extrudeTool_.dragStartHeight,
            base[2] + n[2] * extrudeTool_.dragStartHeight
        };
        float tipSx2, tipSy2;
        float nScreenX = 0, nScreenY = -1; // default: up
        if (worldToScreen(base, view, proj, vpW, vpH, baseSx, baseSy) &&
            worldToScreen(tipAtStart, view, proj, vpW, vpH, tipSx2, tipSy2)) {
            float sdx = tipSx2 - baseSx, sdy = tipSy2 - baseSy;
            float slen = std::sqrt(sdx * sdx + sdy * sdy);
            if (slen > 1.0f) { nScreenX = sdx / slen; nScreenY = sdy / slen; }
        }

        // Project mouse delta onto normal screen direction
        float projectedPx = mouseDx * nScreenX + mouseDy * nScreenY;
        float raw = extrudeTool_.dragStartHeight + projectedPx / pxPerUnit;
        extrudeTool_.height = std::round(raw * 10.0f) / 10.0f; // snap to nearest 0.1
        if (extrudeTool_.height < 0.1f) extrudeTool_.height = 0.1f;
        snprintf(extrudeTool_.heightBuf, sizeof(extrudeTool_.heightBuf), "%.1f", extrudeTool_.height);

        // Throttle: only rebuild the preview every kPreviewThrottleMs during drag
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            now - extrudeTool_.lastPreviewTime).count();
        if (elapsed >= kPreviewThrottleMs) {
            extrudeTool_.previewDirty = true;
            extrudeTool_.lastPreviewTime = now;
        }
    }

    // End drag — force final preview rebuild
    if (extrudeTool_.isDragging && in_.mouseReleased(MouseButton::Left)) {
        extrudeTool_.isDragging = false;
        extrudeTool_.previewDirty = true;
    }

    // Right click: cancel
    if (in_.mouseClicked(MouseButton::Right)) {
        cancelExtrude();
    }
}

App::ExtrudePanelModel App::extrudePanelModel() const {
    ExtrudePanelModel m;
    m.open = tool_.type == ToolType::Extrude && hasExtrudeSketch();
    if (!m.open) return m;
    m.operation = (int)extrudeTool_.operation;
    m.cutAllowed = !scene_.empty();
    m.distanceText = extrudeTool_.heightBuf;
    m.offsetText = extrudeTool_.offsetBuf;
    m.direction = (int)extrudeTool_.direction;
    m.selectedProfiles = (int)extrudeTool_.selectedProfileIndices.size();
    m.totalProfiles = (int)extrudeTool_.allProfiles.size();
    m.canCommit = extrudeTool_.hasSelectedProfiles();
    return m;
}

void App::setExtrudeOperation(int op) {
    if (op == (int)ExtrudeOperation::Cut && scene_.empty()) return;   // nothing to cut
    extrudeTool_.operation = (ExtrudeOperation)op;
    extrudeTool_.previewDirty = true;
}

void App::setExtrudeDistanceText(const std::string& text) {
    snprintf(extrudeTool_.heightBuf, sizeof(extrudeTool_.heightBuf), "%s", text.c_str());
    float val = (float)atof(extrudeTool_.heightBuf);
    if (val > 0.001f) {
        extrudeTool_.height = val;
        extrudeTool_.previewDirty = true;
    } else {
        // Revert to last valid value
        snprintf(extrudeTool_.heightBuf, sizeof(extrudeTool_.heightBuf), "%.3f", extrudeTool_.height);
    }
}

void App::setExtrudeDirection(int dir) {
    extrudeTool_.direction = (ExtrudeDirection)dir;
    extrudeTool_.previewDirty = true;
}

void App::setExtrudeOffsetText(const std::string& text) {
    snprintf(extrudeTool_.offsetBuf, sizeof(extrudeTool_.offsetBuf), "%s", text.c_str());
    extrudeTool_.offset = (float)atof(extrudeTool_.offsetBuf);
    extrudeTool_.previewDirty = true;
}

// Helper: build the tool shape from selected profiles for current extrude settings


void App::updateExtrudePreview() {
    extrudeTool_.previewDirty = false;

    extrudeTool_.previewBody = Body3D{};

    if (!extrudeTool_.hasSelectedProfiles() || !hasExtrudeSketch()) {
        extrudeTool_.handleVisible = false;
        return;
    }

    const SketchPlane& plane = extrudePlane();

    // Compute handle base: centroid of selected profiles in world space
    {
        double cx = 0, cy = 0;
        int count = 0;
        for (int idx : extrudeTool_.selectedProfileIndices) {
            if (idx < 0 || idx >= (int)extrudeTool_.renderCache.size()) continue;
            const auto& cache = extrudeTool_.renderCache[idx];
            for (const auto& pt : cache.tessPoints) {
                cx += pt.x; cy += pt.y;
                count++;
            }
        }
        if (count > 0) {
            cx /= count; cy /= count;
            plane.localToWorld(f(cx), f(cy),
                extrudeTool_.handleBaseWorld[0],
                extrudeTool_.handleBaseWorld[1],
                extrudeTool_.handleBaseWorld[2]);
            extrudeTool_.handleVisible = true;
        }
    }

    // Build preview using the same OCCT pipeline as the actual extrusion
    setToolPreview(extrudeTool_, buildExtrudeToolShape(extrudeTool_, extrudeSketch(), plane));
}

void App::renderExtrudeHandle(const float* view, const float* proj, float vpW, float vpH) {
    if (!extrudeTool_.handleVisible || !extrudeTool_.hasSelectedProfiles()) return;

    const float* n = extrudePlane().normal;
    const float* base = extrudeTool_.handleBaseWorld;

    // Base point (on sketch plane)
    float baseSx, baseSy;
    if (!worldToScreen(base, view, proj, vpW, vpH, baseSx, baseSy)) return;

    // Tip point (at current height along normal)
    float tipWorld[3] = {
        base[0] + n[0] * extrudeTool_.height,
        base[1] + n[1] * extrudeTool_.height,
        base[2] + n[2] * extrudeTool_.height
    };
    float tipSx, tipSy;
    if (!worldToScreen(tipWorld, view, proj, vpW, vpH, tipSx, tipSy)) return;

    Overlay2D& ov = overlay_;
    Color32 handleColor = rgba32(255, 180, 0, 220);
    Color32 tipColor = rgba32(255, 200, 50, 255);

    // Line from base to tip
    ov.addLine({baseSx, baseSy}, {tipSx, tipSy}, handleColor, 2.5f);

    // Arrowhead at tip
    float dx = tipSx - baseSx, dy = tipSy - baseSy;
    float len = std::sqrt(dx*dx + dy*dy);
    if (len > 5.0f) {
        float ux = dx / len, uy = dy / len;
        float arrowLen = 12.0f;
        float arrowW = 6.0f;
        OvVec2 p1 = {tipSx - ux * arrowLen + uy * arrowW, tipSy - uy * arrowLen - ux * arrowW};
        OvVec2 p2 = {tipSx - ux * arrowLen - uy * arrowW, tipSy - uy * arrowLen + ux * arrowW};
        ov.addTriangleFilled({tipSx, tipSy}, p1, p2, handleColor);
    }

    // Drag handle circle at tip
    float handleRadius = 8.0f;
    ov.addCircleFilled({tipSx, tipSy}, handleRadius, tipColor);
    ov.addCircle({tipSx, tipSy}, handleRadius, handleColor, 0, 2.0f);

    // Small circle at base
    ov.addCircleFilled({baseSx, baseSy}, 4.0f, handleColor);

    // Height label near tip
    char label[32];
    snprintf(label, sizeof(label), "%.2f", extrudeTool_.height);
    ov.addText({tipSx + 14.0f, tipSy - 8.0f}, rgba32(255, 220, 100, 255), label);
}

void App::editExtrudeFeature(FeatureID id) {
    const Feature* feat = featureHistory_.findFeature(id);
    if (!feat || feat->type != FeatureType::Extrude) return;

    const auto& ed = std::get<ExtrudeFeatureData>(feat->data);

    int planeIdx = -1;
    std::vector<ClosedProfile> profiles;
    if (!loadSourceSketch(ed.sourceSketchFeature, planeIdx, profiles)) return;

    extrudeTool_.reset();
    beginProfileTool(ToolType::Extrude, extrudeTool_, planeIdx, std::move(profiles));
    extrudeTool_.editingFeatureID = id;

    // Restore saved parameters
    extrudeTool_.height = ed.height;
    extrudeTool_.offset = ed.offset;
    extrudeTool_.operation = ed.operation;
    extrudeTool_.direction = ed.direction;
    snprintf(extrudeTool_.heightBuf, sizeof(extrudeTool_.heightBuf), "%.3f", ed.height);
    snprintf(extrudeTool_.offsetBuf, sizeof(extrudeTool_.offsetBuf), "%.3f", ed.offset);

    // Match saved profile signatures to current detected profiles (closest centroid)
    extrudeTool_.selectedProfileIndices = matchProfiles(ed.profileSigs, ed.profileIndicesFallback,
        extrudeTool_.allProfiles, sketchPlanes_[planeIdx].sketch);

    // Suppress the feature being edited so replay removes its geometry
    // (the transparent preview will show instead)
    featureHistory_.suppressFeature(id);
    replayAllFeatures();
}

void App::commitExtrude() {
    if (!extrudeTool_.hasSelectedProfiles() || !hasExtrudeSketch()) return;

    // Build extrude feature data with profile signatures
    ExtrudeFeatureData ed;
    ed.sourceSketchFeature = ensureSketchFeature(extrudeTool_.sketchPlaneIndex);
    recordProfileSelection(extrudeTool_.selectedProfileIndices, extrudeTool_.allProfiles,
                           extrudeSketch(), ed.profileSigs, ed.profileIndicesFallback);
    ed.height = extrudeTool_.height;
    ed.offset = extrudeTool_.offset;
    ed.operation = extrudeTool_.operation;
    ed.direction = extrudeTool_.direction;

    if (extrudeTool_.editingFeatureID != NullFeatureID) {
        // Editing existing extrude feature — unsuppress first (was suppressed for preview)
        featureHistory_.unsuppressFeature(extrudeTool_.editingFeatureID);

        Feature* existing = featureHistory_.findFeature(extrudeTool_.editingFeatureID);
        if (existing) {
            UndoCommand cmd;
            cmd.type = UndoActionType::ModifyExtrude;
            cmd.featureID = extrudeTool_.editingFeatureID;
            cmd.oldExtrude = std::get<ExtrudeFeatureData>(existing->data);
            cmd.newExtrude = ed;
            globalUndo_.push(std::move(cmd)); markDirty();

            featureHistory_.updateExtrudeData(extrudeTool_.editingFeatureID, ed);
        }
    } else {
        // Creating new extrude feature
        FeatureID extFID = featureHistory_.addExtrudeFeature(ed);

        UndoCommand cmd;
        cmd.type = UndoActionType::AddFeature;
        cmd.addedFeature = *featureHistory_.findFeature(extFID);
        globalUndo_.push(std::move(cmd)); markDirty();
    }

    replayAllFeatures(); // builds actual geometry

    extrudeTool_.reset();
    finishFeatureTool(false);
}

void App::cancelExtrude() {
    // If editing an existing feature, unsuppress it so replay restores the original geometry
    if (extrudeTool_.editingFeatureID != NullFeatureID) {
        featureHistory_.unsuppressFeature(extrudeTool_.editingFeatureID);
    }

    extrudeTool_.reset();
    finishFeatureTool(true);
}

} // namespace shitcad
