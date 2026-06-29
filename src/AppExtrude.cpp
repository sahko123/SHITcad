#include "App.h"
#include "ProfileDetector.h"
#include "Extrude.h"
#include "FacePicker.h"
#include "ExtrudeTool.h"
#include "FeatureReplay.h"
#include <BRep_Builder.hxx>
#include <TopoDS_Compound.hxx>
#include <BRepAlgoAPI_Cut.hxx>
#include <BRepAlgoAPI_Fuse.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Solid.hxx>

#include <glad/gl.h>
#include <GLFW/glfw3.h>
#include <imgui.h>
#include <imgui_internal.h>
#include <imgui_impl_glfw.h>
#include <imgui_impl_opengl3.h>
#include <cstdio>
#include <cmath>
#include <algorithm>
#include <string>

namespace shitcad {

static bool worldToScreen(const float world[3], const float view[16], const float proj[16],
                          float vpW, float vpH, float& sx, float& sy) {
    float vx = view[0]*world[0] + view[4]*world[1] + view[8]*world[2]  + view[12];
    float vy = view[1]*world[0] + view[5]*world[1] + view[9]*world[2]  + view[13];
    float vz = view[2]*world[0] + view[6]*world[1] + view[10]*world[2] + view[14];
    float vw = view[3]*world[0] + view[7]*world[1] + view[11]*world[2] + view[15];
    float cx = proj[0]*vx + proj[4]*vy + proj[8]*vz  + proj[12]*vw;
    float cy = proj[1]*vx + proj[5]*vy + proj[9]*vz  + proj[13]*vw;
    float cw = proj[3]*vx + proj[7]*vy + proj[11]*vz + proj[15]*vw;
    if (std::fabs(cw) < 1e-6f) return false;
    if (cw < 0) return false;
    float ndcX = cx / cw;
    float ndcY = cy / cw;
    sx = (ndcX * 0.5f + 0.5f) * vpW;
    sy = (1.0f - (ndcY * 0.5f + 0.5f)) * vpH;
    return true;
}

void App::enterExtrudeMode() {
    int planeIdx = -1;
    std::vector<ClosedProfile> profiles;

    if (activeSketchPlane_ >= 0) {
        // In sketch mode: use current sketch
        planeIdx = activeSketchPlane_;
        profiles = detectClosedProfiles(activeSketch(), activePlane());
    } else {
        // In navigate mode: find the first sketch plane with closed profiles
        for (int i = 0; i < (int)sketchPlanes_.size(); i++) {
            auto& sp = sketchPlanes_[i];
            if (sp.sketch.points.empty() && sp.sketch.lines.empty() && sp.sketch.circles.empty())
                continue;
            auto p = detectClosedProfiles(sp.sketch, sp);
            if (!p.empty()) {
                if (profiles.empty() || p.size() > profiles.size()) {
                    planeIdx = i;
                    profiles = std::move(p);
                }
            }
        }
    }

    if (planeIdx < 0 || profiles.empty()) {
        fprintf(stderr, "No closed profiles found in any sketch\n");
        return;
    }

    tool_.type = ToolType::Extrude;
    tool_.reset();
    selection_.clear();

    extrudeTool_.reset();
    extrudeTool_.sketchPlaneIndex = planeIdx;
    extrudeTool_.allProfiles = std::move(profiles);

    // Pre-compute tessellation + ear-clipping for all profiles (done once)
    buildProfileRenderCaches(
        sketchPlanes_[planeIdx].sketch,
        extrudeTool_.allProfiles,
        extrudeTool_.renderCache);

    // Auto-select all profiles if there's only one
    if (extrudeTool_.allProfiles.size() == 1) {
        extrudeTool_.selectedProfileIndices.insert(0);
    }

    extrudeTool_.previewDirty = true;
    extrudeTool_.lastPreviewTime = std::chrono::steady_clock::now();
}

void App::handleExtrudeInput(float vpW, float vpH) {
    if (!hasExtrudeSketch()) return;
    ImGuiIO& io = ImGui::GetIO();

    const SketchPlane& plane = extrudePlane();

    // Project mouse to sketch plane
    if (!io.WantCaptureMouse) {
        int w, h;
        glfwGetFramebufferSize(window_, &w, &h);
        float view[16], proj[16];
        getViewProj(w, h, view, proj);

        float rayOrig[3], rayDir[3];
        screenToRay(io.MousePos.x, io.MousePos.y, 0, 0, vpW, vpH, view, proj, rayOrig, rayDir);

        float lx, ly, t;
        if (plane.rayIntersect(rayOrig, rayDir, lx, ly, t)) {
            cursorLocal_ = {lx, ly};
        }
    }

    // Keyboard
    if (!io.WantCaptureKeyboard) {
        if (ImGui::IsKeyPressed(ImGuiKey_Escape)) {
            cancelExtrude();
            return;
        }
        if (ImGui::IsKeyPressed(ImGuiKey_Enter) || ImGui::IsKeyPressed(ImGuiKey_KeypadEnter)) {
            if (extrudeTool_.hasSelectedProfiles()) {
                commitExtrude();
                return;
            }
        }
    }

    if (io.WantCaptureMouse) return;

    // Profile selection: click to toggle (disabled during handle drag)
    if (!extrudeTool_.isDragging) {
        if (ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
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
        extrudeTool_.handleVisible && ImGui::IsMouseClicked(ImGuiMouseButton_Left)) {
        // Compute handle tip position in screen space
        int w, h;
        glfwGetFramebufferSize(window_, &w, &h);
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
            float dx = io.MousePos.x - tipSx;
            float dy = io.MousePos.y - tipSy;
            float handleRadius = 20.0f; // pixels
            if (dx*dx + dy*dy < handleRadius * handleRadius) {
                extrudeTool_.isDragging = true;
                extrudeTool_.dragStartMouseX = io.MousePos.x;
                extrudeTool_.dragStartMouseY = io.MousePos.y;
                extrudeTool_.dragStartHeight = extrudeTool_.height;
            }
        }
    }

    // During drag: update height (throttled preview rebuild)
    if (extrudeTool_.isDragging) {
        // Compute pixels-per-unit along the extrude normal for 1:1 screen mapping
        int w, h;
        glfwGetFramebufferSize(window_, &w, &h);
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
        float mouseDx = io.MousePos.x - extrudeTool_.dragStartMouseX;
        float mouseDy = io.MousePos.y - extrudeTool_.dragStartMouseY;

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

        // Throttle: only rebuild preview every 50ms during drag
        auto now = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            now - extrudeTool_.lastPreviewTime).count();
        if (elapsed >= 50) {
            extrudeTool_.previewDirty = true;
            extrudeTool_.lastPreviewTime = now;
        }
    }

    // End drag — force final preview rebuild
    if (extrudeTool_.isDragging && ImGui::IsMouseReleased(ImGuiMouseButton_Left)) {
        extrudeTool_.isDragging = false;
        extrudeTool_.previewDirty = true;
    }

    // Right click: cancel
    if (ImGui::IsMouseClicked(ImGuiMouseButton_Right)) {
        cancelExtrude();
    }
}

void App::drawExtrudePanel() {
    ImGuiIO& io = ImGui::GetIO();
    float panelW = 220.0f;

    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, {10, 10});
    ImGui::SetNextWindowPos({io.DisplaySize.x - panelW, 30});
    ImGui::SetNextWindowSize({panelW, 0});
    ImGui::Begin("Extrude", nullptr,
        ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove |
        ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_AlwaysAutoResize);

    // Operation
    ImGui::Text("Operation:");
    const char* opNames[] = {"New Body", "Cut"};
    int opIdx = (int)extrudeTool_.operation;
    bool cutDisabled = (extrudeTool_.operation != ExtrudeOperation::Cut && scene_.empty());
    if (cutDisabled) {
        // Can't switch to Cut if no bodies exist — but allow if already Cut
    }
    if (ImGui::Combo("##op", &opIdx, opNames, 2)) {
        if (opIdx == (int)ExtrudeOperation::Cut && scene_.empty()) {
            // Don't allow Cut when no bodies
        } else {
            extrudeTool_.operation = (ExtrudeOperation)opIdx;
            extrudeTool_.previewDirty = true;
        }
    }

    // Distance
    ImGui::Text("Distance:");
    if (ImGui::InputText("##dist", extrudeTool_.heightBuf, sizeof(extrudeTool_.heightBuf),
            ImGuiInputTextFlags_EnterReturnsTrue)) {
        float val = (float)atof(extrudeTool_.heightBuf);
        if (val > 0.001f) {
            extrudeTool_.height = val;
            extrudeTool_.previewDirty = true;
        } else {
            // Revert to last valid value
            snprintf(extrudeTool_.heightBuf, sizeof(extrudeTool_.heightBuf), "%.3f", extrudeTool_.height);
        }
    }
    // Also sync on deactivation (user tabs away or clicks elsewhere)
    if (ImGui::IsItemDeactivatedAfterEdit()) {
        float val = (float)atof(extrudeTool_.heightBuf);
        if (val > 0.001f) {
            extrudeTool_.height = val;
            extrudeTool_.previewDirty = true;
        } else {
            snprintf(extrudeTool_.heightBuf, sizeof(extrudeTool_.heightBuf), "%.3f", extrudeTool_.height);
        }
    }

    // Direction
    ImGui::Text("Direction:");
    const char* dirNames[] = {"One Side", "Other Side", "Both Sides", "Symmetric"};
    int dirIdx = (int)extrudeTool_.direction;
    if (ImGui::Combo("##dir", &dirIdx, dirNames, 4)) {
        extrudeTool_.direction = (ExtrudeDirection)dirIdx;
        extrudeTool_.previewDirty = true;
    }

    // Offset
    ImGui::Text("Offset:");
    if (ImGui::InputText("##offset", extrudeTool_.offsetBuf, sizeof(extrudeTool_.offsetBuf),
            ImGuiInputTextFlags_EnterReturnsTrue)) {
        extrudeTool_.offset = (float)atof(extrudeTool_.offsetBuf);
        extrudeTool_.previewDirty = true;
    }
    if (ImGui::IsItemDeactivatedAfterEdit()) {
        extrudeTool_.offset = (float)atof(extrudeTool_.offsetBuf);
        extrudeTool_.previewDirty = true;
    }

    // Profile info
    ImGui::Separator();
    ImGui::Text("Profiles: %d / %d",
                (int)extrudeTool_.selectedProfileIndices.size(),
                (int)extrudeTool_.allProfiles.size());

    // OK / Cancel
    ImGui::Separator();
    bool canCommit = extrudeTool_.hasSelectedProfiles();
    if (!canCommit) ImGui::BeginDisabled();
    if (ImGui::Button("OK [Enter]", {95, 0})) {
        commitExtrude();
    }
    if (!canCommit) ImGui::EndDisabled();
    ImGui::SameLine();
    if (ImGui::Button("Cancel [Esc]", {95, 0})) {
        cancelExtrude();
    }

    ImGui::End();
    ImGui::PopStyleVar();
}

// Helper: build the tool shape from selected profiles for current extrude settings
// buildExtrudeToolShape and enumerateSolids are now in Extrude.h/cpp


void App::updateExtrudePreview() {
    extrudeTool_.previewDirty = false;

    // Restore body visibility from any previous preview
    if (extrudeTool_.hidingBodiesForPreview) {
        for (size_t i = 0; i < scene_.bodyCount(); i++)
            scene_.getBodyMut((int)i).visible = true;
        extrudeTool_.hidingBodiesForPreview = false;
    }

    // Clean up old preview resources
    if (extrudeTool_.previewBody.vao) {
        glDeleteVertexArrays(1, &extrudeTool_.previewBody.vao);
        extrudeTool_.previewBody.vao = 0;
    }
    if (extrudeTool_.previewBody.vbo) {
        glDeleteBuffers(1, &extrudeTool_.previewBody.vbo);
        extrudeTool_.previewBody.vbo = 0;
    }
    if (extrudeTool_.previewBody.edgeVAO) {
        glDeleteVertexArrays(1, &extrudeTool_.previewBody.edgeVAO);
        extrudeTool_.previewBody.edgeVAO = 0;
    }
    if (extrudeTool_.previewBody.edgeVBO) {
        glDeleteBuffers(1, &extrudeTool_.previewBody.edgeVBO);
        extrudeTool_.previewBody.edgeVBO = 0;
    }
    extrudeTool_.previewBody.vertices.clear();
    extrudeTool_.previewBody.vertexCount = 0;
    extrudeTool_.previewBody.edgeVertexCount = 0;

    for (auto& b : extrudeTool_.cutPreviewBodies) {
        if (b.vao) glDeleteVertexArrays(1, &b.vao);
        if (b.vbo) glDeleteBuffers(1, &b.vbo);
        if (b.edgeVAO) glDeleteVertexArrays(1, &b.edgeVAO);
        if (b.edgeVBO) glDeleteBuffers(1, &b.edgeVBO);
    }
    extrudeTool_.cutPreviewBodies.clear();

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
    const Sketch& sketch = extrudeSketch();
    TopoDS_Shape previewShape = buildExtrudeToolShape(extrudeTool_, sketch, plane);

    extrudeTool_.previewBody = Body3D{};
    if (!previewShape.IsNull()) {
        Scene3D::triangulateShape(previewShape, extrudeTool_.previewBody.vertices);
    }

    if (extrudeTool_.operation == ExtrudeOperation::Cut) {
        extrudeTool_.previewBody.colorR = 0.9f;
        extrudeTool_.previewBody.colorG = 0.3f;
        extrudeTool_.previewBody.colorB = 0.3f;
    } else {
        extrudeTool_.previewBody.colorR = 0.4f;
        extrudeTool_.previewBody.colorG = 0.6f;
        extrudeTool_.previewBody.colorB = 0.9f;
    }
    Scene3D::uploadMesh(extrudeTool_.previewBody);
}

void App::renderExtrudePreview(const float* view, const float* proj, const float* eyePos) {
    auto& body = extrudeTool_.previewBody;
    if (body.vao == 0 || body.vertexCount == 0) return;

    auto& shader = viewport3D_.meshShader();
    shader.use();
    shader.setMat4("uView", view);
    shader.setMat4("uProj", proj);
    shader.setVec3("uEyePos", eyePos[0], eyePos[1], eyePos[2]);
    shader.setVec3("uLightDir", 0.3f, 0.8f, 0.5f);

    float alpha = (extrudeTool_.operation == ExtrudeOperation::Cut) ? 0.4f : 0.5f;
    shader.setVec3("uColor", body.colorR, body.colorG, body.colorB);
    shader.setFloat("uAlpha", alpha);

    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glDepthMask(GL_FALSE);

    glBindVertexArray(body.vao);
    glDrawArrays(GL_TRIANGLES, 0, body.vertexCount);
    glBindVertexArray(0);

    glDepthMask(GL_TRUE);
    glDisable(GL_BLEND);
    shader.setFloat("uAlpha", 1.0f);
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

    ImDrawList* dl = ImGui::GetForegroundDrawList();
    ImU32 handleColor = IM_COL32(255, 180, 0, 220);
    ImU32 tipColor = IM_COL32(255, 200, 50, 255);

    // Line from base to tip
    dl->AddLine({baseSx, baseSy}, {tipSx, tipSy}, handleColor, 2.5f);

    // Arrowhead at tip
    float dx = tipSx - baseSx, dy = tipSy - baseSy;
    float len = std::sqrt(dx*dx + dy*dy);
    if (len > 5.0f) {
        float ux = dx / len, uy = dy / len;
        float arrowLen = 12.0f;
        float arrowW = 6.0f;
        ImVec2 p1 = {tipSx - ux * arrowLen + uy * arrowW, tipSy - uy * arrowLen - ux * arrowW};
        ImVec2 p2 = {tipSx - ux * arrowLen - uy * arrowW, tipSy - uy * arrowLen + ux * arrowW};
        dl->AddTriangleFilled({tipSx, tipSy}, p1, p2, handleColor);
    }

    // Drag handle circle at tip
    float handleRadius = 8.0f;
    dl->AddCircleFilled({tipSx, tipSy}, handleRadius, tipColor);
    dl->AddCircle({tipSx, tipSy}, handleRadius, handleColor, 0, 2.0f);

    // Small circle at base
    dl->AddCircleFilled({baseSx, baseSy}, 4.0f, handleColor);

    // Height label near tip
    char label[32];
    snprintf(label, sizeof(label), "%.2f", extrudeTool_.height);
    dl->AddText({tipSx + 14.0f, tipSy - 8.0f}, IM_COL32(255, 220, 100, 255), label);
}

void App::editExtrudeFeature(FeatureID id) {
    const Feature* feat = featureHistory_.findFeature(id);
    if (!feat || feat->type != FeatureType::Extrude) return;

    const auto& ed = std::get<ExtrudeFeatureData>(feat->data);

    // Find the source sketch plane index from the source sketch feature
    const Feature* srcFeat = featureHistory_.findFeature(ed.sourceSketchFeature);
    if (!srcFeat || srcFeat->type != FeatureType::Sketch) return;
    const auto& sd = std::get<SketchFeatureData>(srcFeat->data);
    int planeIdx = sd.sketchPlaneIndex;
    if (planeIdx < 0 || planeIdx >= (int)sketchPlanes_.size()) return;

    // If currently sketching, finish the sketch first
    if (mode_ == InteractionMode::Sketching) {
        finishSketch();
    }

    // Detect profiles from the sketch
    auto profiles = detectClosedProfiles(sketchPlanes_[planeIdx].sketch, sketchPlanes_[planeIdx]);
    if (profiles.empty()) return;

    // Enter extrude mode
    tool_.type = ToolType::Extrude;
    tool_.reset();
    selection_.clear();

    extrudeTool_.reset();
    extrudeTool_.sketchPlaneIndex = planeIdx;
    extrudeTool_.allProfiles = std::move(profiles);
    extrudeTool_.editingFeatureID = id;

    // Restore saved parameters
    extrudeTool_.height = ed.height;
    extrudeTool_.offset = ed.offset;
    extrudeTool_.operation = ed.operation;
    extrudeTool_.direction = ed.direction;
    snprintf(extrudeTool_.heightBuf, sizeof(extrudeTool_.heightBuf), "%.3f", ed.height);
    snprintf(extrudeTool_.offsetBuf, sizeof(extrudeTool_.offsetBuf), "%.3f", ed.offset);

    // Build render caches
    buildProfileRenderCaches(
        sketchPlanes_[planeIdx].sketch,
        extrudeTool_.allProfiles,
        extrudeTool_.renderCache);

    // Match saved profile signatures to current detected profiles (closest centroid)
    auto matched = matchProfiles(ed.profileSigs, ed.profileIndicesFallback, extrudeTool_.allProfiles, sketchPlanes_[planeIdx].sketch);
    extrudeTool_.selectedProfileIndices = matched;

    // Suppress the feature being edited so replay removes its geometry
    // (the transparent preview will show instead)
    featureHistory_.suppressFeature(id);
    replayAllFeatures();

    extrudeTool_.previewDirty = true;
    extrudeTool_.lastPreviewTime = std::chrono::steady_clock::now();
}

void App::commitExtrude() {
    if (!extrudeTool_.hasSelectedProfiles() || !hasExtrudeSketch()) return;

    // Restore body visibility before modifying bodies
    if (extrudeTool_.hidingBodiesForPreview) {
        for (size_t i = 0; i < scene_.bodyCount(); i++)
            scene_.getBodyMut((int)i).visible = true;
        extrudeTool_.hidingBodiesForPreview = false;
    }

    // Ensure sketch feature exists for the source sketch plane
    FeatureID srcSketch = featureHistory_.findSketchFeatureForPlane(extrudeTool_.sketchPlaneIndex);
    if (srcSketch == NullFeatureID) {
        srcSketch = featureHistory_.addSketchFeature(extrudeTool_.sketchPlaneIndex, extrudeSketch(), extrudePlane().planeID);
        // Push undo for the implicitly created sketch feature
        UndoCommand skCmd;
        skCmd.type = UndoActionType::AddFeature;
        skCmd.addedFeature = *featureHistory_.findFeature(srcSketch);
        globalUndo_.push(std::move(skCmd)); markDirty();
    } else if (mode_ == InteractionMode::Sketching) {
        // Currently sketching — the live sketch may have unsaved edits.
        // Update the snapshot so replay uses the current geometry.
        const Sketch& liveSketch = extrudeSketch();
        const Sketch& oldSnap = std::get<SketchFeatureData>(
            featureHistory_.findFeature(srcSketch)->data).sketchSnapshot;
        // Only push undo if actually changed
        if (oldSnap.nextID != liveSketch.nextID ||
            oldSnap.points.size() != liveSketch.points.size() ||
            oldSnap.lines.size() != liveSketch.lines.size() ||
            oldSnap.circles.size() != liveSketch.circles.size() ||
            oldSnap.arcs.size() != liveSketch.arcs.size()) {
            UndoCommand skCmd;
            skCmd.type = UndoActionType::ModifySketch;
            skCmd.featureID = srcSketch;
            skCmd.oldSketch = oldSnap;
            skCmd.newSketch = liveSketch;
            globalUndo_.push(std::move(skCmd)); markDirty();
        }
        featureHistory_.updateSketchSnapshot(srcSketch, liveSketch);
    }

    // Build extrude feature data with profile signatures
    ExtrudeFeatureData ed;
    ed.sourceSketchFeature = srcSketch;
    for (int idx : extrudeTool_.selectedProfileIndices) {
        if (idx >= 0 && idx < (int)extrudeTool_.allProfiles.size()) {
            ed.profileSigs.push_back(ProfileSignature::fromProfile(extrudeTool_.allProfiles[idx], extrudeSketch()));
            ed.profileIndicesFallback.push_back(idx);
        }
    }
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
    tool_.type = ToolType::None;
    tool_.reset();

    // If we were in sketch mode, finish the sketch after extrude
    // Skip feature recording since commitExtrude already handled it
    if (mode_ == InteractionMode::Sketching) {
        finishSketch(false);
    }
}

void App::cancelExtrude() {
    // Restore body visibility before reset clears the flag
    if (extrudeTool_.hidingBodiesForPreview) {
        for (size_t i = 0; i < scene_.bodyCount(); i++)
            scene_.getBodyMut((int)i).visible = true;
    }

    // If editing an existing feature, unsuppress it and replay to restore original geometry
    if (extrudeTool_.editingFeatureID != NullFeatureID) {
        featureHistory_.unsuppressFeature(extrudeTool_.editingFeatureID);
    }

    extrudeTool_.reset();
    tool_.type = ToolType::None;
    tool_.reset();

    replayAllFeatures();
}

} // namespace shitcad
