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
#include <imgui.h>
#include <imgui_internal.h>
#include <cstdio>
#include <cmath>
#include <algorithm>
#include <string>

namespace shitcad {

void App::enterRevolveMode() {
    int planeIdx = -1;
    std::vector<ClosedProfile> profiles;

    if (activeSketchPlane_ >= 0) {
        planeIdx = activeSketchPlane_;
        profiles = detectClosedProfiles(activeSketch(), activePlane());
    } else {
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
        fprintf(stderr, "No closed profiles found for revolve\n");
        return;
    }

    // Need at least one line in the sketch to use as axis
    if (sketchPlanes_[planeIdx].sketch.lines.empty()) {
        fprintf(stderr, "No lines available as revolve axis\n");
        return;
    }

    tool_.type = ToolType::Revolve;
    tool_.reset();
    selection_.clear();

    revolveTool_.reset();
    revolveTool_.sketchPlaneIndex = planeIdx;
    revolveTool_.allProfiles = std::move(profiles);

    buildProfileRenderCaches(
        sketchPlanes_[planeIdx].sketch,
        revolveTool_.allProfiles,
        revolveTool_.renderCache);

    if (revolveTool_.allProfiles.size() == 1) {
        revolveTool_.selectedProfileIndices.insert(0);
    }

    revolveTool_.previewDirty = true;
    revolveTool_.lastPreviewTime = std::chrono::steady_clock::now();
}

void App::handleRevolveInput(float vpW, float vpH) {
    if (!hasRevolveSketch()) return;

    const SketchPlane& plane = revolvePlane();

    if (!in_.uiWantsMouse) {
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

    if (!in_.uiWantsKeyboard) {
        if (in_.keyPressed(Key::Escape)) {
            cancelRevolve();
            return;
        }
        if (in_.keyPressed(Key::Enter) || in_.keyPressed(Key::KeypadEnter)) {
            if (revolveTool_.hasSelectedProfiles() && revolveTool_.axisLineID != NullID) {
                commitRevolve();
                return;
            }
        }
    }

    if (in_.uiWantsMouse) return;

    if (in_.mouseClicked(MouseButton::Left)) {
        if (revolveTool_.phase == RevolvePhase::SelectingProfiles) {
            int hitIdx = hitTestProfile(revolveSketch(), revolveTool_.allProfiles, cursorLocal_,
                                        revolveTool_.renderCache);
            if (hitIdx >= 0) {
                if (revolveTool_.selectedProfileIndices.count(hitIdx))
                    revolveTool_.selectedProfileIndices.erase(hitIdx);
                else
                    revolveTool_.selectedProfileIndices.insert(hitIdx);
                revolveTool_.previewDirty = true;
            }
        } else if (revolveTool_.phase == RevolvePhase::SelectingAxis) {
            // Hit test lines in sketch
            const Sketch& sketch = revolveSketch();
            int w, h;
            framebufferSize(w, h);
            float view[16], proj[16];
            getViewProj(w, h, view, proj);
            float apparentScale = computeApparentScale(revolvePlane(), view, proj, (float)w, (float)h);

            EntityID bestLine = NullID;
            float bestDist = 15.0f; // pixel tolerance
            for (const auto& line : sketch.lines) {
                if (line.projected) continue;
                Point2D a = sketch.getPointPos(line.startPt);
                Point2D b = sketch.getPointPos(line.endPt);
                // Point-to-segment distance
                double dx = b.x - a.x, dy = b.y - a.y;
                double lenSq = dx*dx + dy*dy;
                if (lenSq < 1e-10) continue;
                double t2 = std::clamp(((cursorLocal_.x - a.x)*dx + (cursorLocal_.y - a.y)*dy) / lenSq, 0.0, 1.0);
                double px = a.x + t2*dx, py = a.y + t2*dy;
                float dist = (float)std::sqrt((cursorLocal_.x-px)*(cursorLocal_.x-px) + (cursorLocal_.y-py)*(cursorLocal_.y-py));
                dist *= apparentScale;
                if (dist < bestDist) {
                    bestDist = dist;
                    bestLine = line.id;
                }
            }
            if (bestLine != NullID) {
                revolveTool_.axisLineID = bestLine;
                revolveTool_.phase = RevolvePhase::Adjusting;
                revolveTool_.previewDirty = true;
            }
        }
    }
}

void App::drawRevolvePanel() {
    ImGuiIO& io = ImGui::GetIO();
    float panelW = 220.0f;

    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, {10, 10});
    ImGui::SetNextWindowPos({io.DisplaySize.x - panelW, 30});
    ImGui::SetNextWindowSize({panelW, 0});
    ImGui::Begin("Revolve", nullptr,
        ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove |
        ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_AlwaysAutoResize);

    // Operation
    ImGui::Text("Operation:");
    const char* opNames[] = {"New Body", "Cut"};
    int opIdx = (int)revolveTool_.operation;
    if (ImGui::Combo("##revOp", &opIdx, opNames, 2)) {
        if (opIdx == (int)ExtrudeOperation::Cut && scene_.empty()) {
            // Don't allow
        } else {
            revolveTool_.operation = (ExtrudeOperation)opIdx;
            revolveTool_.previewDirty = true;
        }
    }

    // Angle
    ImGui::Text("Angle (deg):");
    if (ImGui::InputText("##revAngle", revolveTool_.angleBuf, sizeof(revolveTool_.angleBuf),
            ImGuiInputTextFlags_EnterReturnsTrue)) {
        float val = (float)atof(revolveTool_.angleBuf);
        if (std::fabs(val) > 0.01f) {
            revolveTool_.angleDeg = val;
            revolveTool_.previewDirty = true;
        } else {
            snprintf(revolveTool_.angleBuf, sizeof(revolveTool_.angleBuf), "%.1f", revolveTool_.angleDeg);
        }
    }
    if (ImGui::IsItemDeactivatedAfterEdit()) {
        float val = (float)atof(revolveTool_.angleBuf);
        if (std::fabs(val) > 0.01f) {
            revolveTool_.angleDeg = val;
            revolveTool_.previewDirty = true;
        } else {
            snprintf(revolveTool_.angleBuf, sizeof(revolveTool_.angleBuf), "%.1f", revolveTool_.angleDeg);
        }
    }

    // Axis info
    ImGui::Separator();
    if (revolveTool_.axisLineID != NullID) {
        ImGui::Text("Axis: Line %u", revolveTool_.axisLineID);
        if (ImGui::Button("Change Axis")) {
            revolveTool_.phase = RevolvePhase::SelectingAxis;
            revolveTool_.axisLineID = NullID;
            revolveTool_.previewDirty = true;
        }
    } else {
        if (revolveTool_.phase == RevolvePhase::SelectingAxis) {
            ImGui::TextColored(ImVec4(1,0.8f,0,1), "Click a line for axis");
        } else {
            if (ImGui::Button("Select Axis")) {
                revolveTool_.phase = RevolvePhase::SelectingAxis;
            }
        }
    }

    // Profile info
    ImGui::Separator();
    ImGui::Text("Profiles: %d / %d",
                (int)revolveTool_.selectedProfileIndices.size(),
                (int)revolveTool_.allProfiles.size());

    if (revolveTool_.phase != RevolvePhase::SelectingAxis) {
        if (ImGui::Button("Select Profiles")) {
            revolveTool_.phase = RevolvePhase::SelectingProfiles;
        }
    }

    // OK / Cancel
    ImGui::Separator();
    bool canCommit = revolveTool_.hasSelectedProfiles() && revolveTool_.axisLineID != NullID;
    if (!canCommit) ImGui::BeginDisabled();
    if (ImGui::Button("OK [Enter]", {95, 0})) {
        commitRevolve();
    }
    if (!canCommit) ImGui::EndDisabled();
    ImGui::SameLine();
    if (ImGui::Button("Cancel [Esc]", {95, 0})) {
        cancelRevolve();
    }

    ImGui::End();
    ImGui::PopStyleVar();
}

void App::updateRevolvePreview() {
    revolveTool_.previewDirty = false;

    // Restore body visibility from any previous preview
    if (revolveTool_.hidingBodiesForPreview) {
        for (size_t i = 0; i < scene_.bodyCount(); i++)
            scene_.getBodyMut((int)i).visible = true;
        revolveTool_.hidingBodiesForPreview = false;
    }

    // Clean up old preview resources
    if (revolveTool_.previewBody.vao) {
        glDeleteVertexArrays(1, &revolveTool_.previewBody.vao);
        revolveTool_.previewBody.vao = 0;
    }
    if (revolveTool_.previewBody.vbo) {
        glDeleteBuffers(1, &revolveTool_.previewBody.vbo);
        revolveTool_.previewBody.vbo = 0;
    }
    if (revolveTool_.previewBody.edgeVAO) {
        glDeleteVertexArrays(1, &revolveTool_.previewBody.edgeVAO);
        revolveTool_.previewBody.edgeVAO = 0;
    }
    if (revolveTool_.previewBody.edgeVBO) {
        glDeleteBuffers(1, &revolveTool_.previewBody.edgeVBO);
        revolveTool_.previewBody.edgeVBO = 0;
    }
    revolveTool_.previewBody.vertices.clear();
    revolveTool_.previewBody.vertexCount = 0;
    revolveTool_.previewBody.edgeVertexCount = 0;

    for (auto& b : revolveTool_.cutPreviewBodies) {
        if (b.vao) glDeleteVertexArrays(1, &b.vao);
        if (b.vbo) glDeleteBuffers(1, &b.vbo);
        if (b.edgeVAO) glDeleteVertexArrays(1, &b.edgeVAO);
        if (b.edgeVBO) glDeleteBuffers(1, &b.edgeVBO);
    }
    revolveTool_.cutPreviewBodies.clear();

    if (!revolveTool_.hasSelectedProfiles() || revolveTool_.axisLineID == NullID) {
        return;
    }

    const Sketch& sketch = revolveSketch();
    const SketchPlane& plane = revolvePlane();

    TopoDS_Shape previewShape = buildRevolveToolShape(revolveTool_, sketch, plane);

    revolveTool_.previewBody = Body3D{};
    if (!previewShape.IsNull()) {
        Scene3D::triangulateShape(previewShape, revolveTool_.previewBody.vertices);
    }

    if (revolveTool_.operation == ExtrudeOperation::Cut) {
        revolveTool_.previewBody.colorR = 0.9f;
        revolveTool_.previewBody.colorG = 0.3f;
        revolveTool_.previewBody.colorB = 0.3f;
    } else {
        revolveTool_.previewBody.colorR = 0.4f;
        revolveTool_.previewBody.colorG = 0.6f;
        revolveTool_.previewBody.colorB = 0.9f;
    }
    Scene3D::uploadMesh(revolveTool_.previewBody);

    revolveTool_.lastPreviewTime = std::chrono::steady_clock::now();
}

void App::renderRevolvePreview(const float* view, const float* proj, const float* eyePos) {
    auto& body = revolveTool_.previewBody;
    if (body.vao == 0 || body.vertexCount == 0) return;

    auto& shader = viewport3D_.meshShader();
    shader.use();
    shader.setMat4("uView", view);
    shader.setMat4("uProj", proj);
    shader.setVec3("uEyePos", eyePos[0], eyePos[1], eyePos[2]);
    shader.setVec3("uLightDir", 0.3f, 0.8f, 0.5f);
    applyClip(shader, nullptr); // tool previews are never sectioned

    float alpha = (revolveTool_.operation == ExtrudeOperation::Cut) ? 0.4f : 0.5f;
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

void App::commitRevolve() {
    if (!revolveTool_.hasSelectedProfiles() || !hasRevolveSketch() || revolveTool_.axisLineID == NullID)
        return;

    if (revolveTool_.hidingBodiesForPreview) {
        for (size_t i = 0; i < scene_.bodyCount(); i++)
            scene_.getBodyMut((int)i).visible = true;
        revolveTool_.hidingBodiesForPreview = false;
    }

    FeatureID srcSketch = featureHistory_.findSketchFeatureForPlane(revolveTool_.sketchPlaneIndex);
    if (srcSketch == NullFeatureID) {
        srcSketch = featureHistory_.addSketchFeature(revolveTool_.sketchPlaneIndex, revolveSketch(), revolvePlane().planeID);
        UndoCommand skCmd;
        skCmd.type = UndoActionType::AddFeature;
        skCmd.addedFeature = *featureHistory_.findFeature(srcSketch);
        globalUndo_.push(std::move(skCmd)); markDirty();
    } else if (mode_ == InteractionMode::Sketching) {
        const Sketch& liveSketch = revolveSketch();
        const Sketch& oldSnap = std::get<SketchFeatureData>(
            featureHistory_.findFeature(srcSketch)->data).sketchSnapshot;
        if (oldSnap.nextID != liveSketch.nextID ||
            oldSnap.points.size() != liveSketch.points.size()) {
            UndoCommand skCmd;
            skCmd.type = UndoActionType::ModifySketch;
            skCmd.featureID = srcSketch;
            skCmd.oldSketch = oldSnap;
            skCmd.newSketch = liveSketch;
            globalUndo_.push(std::move(skCmd)); markDirty();
        }
        featureHistory_.updateSketchSnapshot(srcSketch, liveSketch);
    }

    RevolveFeatureData rd;
    rd.sourceSketchFeature = srcSketch;
    for (int idx : revolveTool_.selectedProfileIndices) {
        if (idx >= 0 && idx < (int)revolveTool_.allProfiles.size()) {
            rd.profileSigs.push_back(ProfileSignature::fromProfile(revolveTool_.allProfiles[idx], sketchPlanes_[revolveTool_.sketchPlaneIndex].sketch));
            rd.profileIndicesFallback.push_back(idx);
        }
    }
    rd.axisLineID = revolveTool_.axisLineID;
    rd.angleDeg = revolveTool_.angleDeg;
    rd.operation = revolveTool_.operation;

    if (revolveTool_.editingFeatureID != NullFeatureID) {
        featureHistory_.unsuppressFeature(revolveTool_.editingFeatureID);
        Feature* existing = featureHistory_.findFeature(revolveTool_.editingFeatureID);
        if (existing) {
            UndoCommand cmd;
            cmd.type = UndoActionType::ModifyRevolve;
            cmd.featureID = revolveTool_.editingFeatureID;
            cmd.oldRevolve = std::get<RevolveFeatureData>(existing->data);
            cmd.newRevolve = rd;
            globalUndo_.push(std::move(cmd)); markDirty();
            featureHistory_.updateRevolveData(revolveTool_.editingFeatureID, rd);
        }
    } else {
        FeatureID revFID = featureHistory_.addRevolveFeature(rd);
        UndoCommand cmd;
        cmd.type = UndoActionType::AddFeature;
        cmd.addedFeature = *featureHistory_.findFeature(revFID);
        globalUndo_.push(std::move(cmd)); markDirty();
    }

    replayAllFeatures();

    revolveTool_.reset();
    tool_.type = ToolType::None;
    tool_.reset();

    if (mode_ == InteractionMode::Sketching) {
        finishSketch(false);
    }
}

void App::cancelRevolve() {
    if (revolveTool_.hidingBodiesForPreview) {
        for (size_t i = 0; i < scene_.bodyCount(); i++)
            scene_.getBodyMut((int)i).visible = true;
    }

    if (revolveTool_.editingFeatureID != NullFeatureID) {
        featureHistory_.unsuppressFeature(revolveTool_.editingFeatureID);
    }

    revolveTool_.reset();
    tool_.type = ToolType::None;
    tool_.reset();

    replayAllFeatures();
}

void App::editRevolveFeature(FeatureID id) {
    const Feature* feat = featureHistory_.findFeature(id);
    if (!feat || feat->type != FeatureType::Revolve) return;

    const auto& rd = std::get<RevolveFeatureData>(feat->data);

    // Find the source sketch plane
    const Feature* srcFeat = featureHistory_.findFeature(rd.sourceSketchFeature);
    if (!srcFeat || srcFeat->type != FeatureType::Sketch) return;
    const auto& sd = std::get<SketchFeatureData>(srcFeat->data);
    int planeIdx = sd.sketchPlaneIndex;
    if (planeIdx < 0 || planeIdx >= (int)sketchPlanes_.size()) return;

    if (mode_ == InteractionMode::Sketching) {
        finishSketch();
    }

    auto profiles = detectClosedProfiles(sketchPlanes_[planeIdx].sketch, sketchPlanes_[planeIdx]);
    if (profiles.empty()) return;

    // Need at least one line for axis
    if (sketchPlanes_[planeIdx].sketch.lines.empty()) return;

    tool_.type = ToolType::Revolve;
    tool_.reset();
    selection_.clear();

    revolveTool_.reset();
    revolveTool_.sketchPlaneIndex = planeIdx;
    revolveTool_.allProfiles = std::move(profiles);
    revolveTool_.editingFeatureID = id;

    // Restore saved parameters
    revolveTool_.angleDeg = rd.angleDeg;
    revolveTool_.operation = rd.operation;
    revolveTool_.axisLineID = rd.axisLineID;
    snprintf(revolveTool_.angleBuf, sizeof(revolveTool_.angleBuf), "%.1f", rd.angleDeg);

    // If axis line is set, skip to Adjusting phase
    if (rd.axisLineID != NullID) {
        revolveTool_.phase = RevolvePhase::Adjusting;
    }

    buildProfileRenderCaches(
        sketchPlanes_[planeIdx].sketch,
        revolveTool_.allProfiles,
        revolveTool_.renderCache);

    // Match saved profile signatures to current detected profiles
    auto matched = matchProfiles(rd.profileSigs, rd.profileIndicesFallback, revolveTool_.allProfiles, sketchPlanes_[planeIdx].sketch);
    revolveTool_.selectedProfileIndices = matched;

    // If profiles matched, skip past profile selection phase
    if (!matched.empty() && rd.axisLineID != NullID) {
        revolveTool_.phase = RevolvePhase::Adjusting;
    } else if (!matched.empty()) {
        revolveTool_.phase = RevolvePhase::SelectingAxis;
    }

    // Suppress the feature being edited
    featureHistory_.suppressFeature(id);
    replayAllFeatures();

    revolveTool_.previewDirty = true;
    revolveTool_.lastPreviewTime = std::chrono::steady_clock::now();
}

} // namespace shitcad
