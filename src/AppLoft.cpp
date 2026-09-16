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

void App::enterLoftMode() {
    tool_.type = ToolType::Loft;
    tool_.reset();
    selection_.clear();
    loftTool_.reset();
    loftTool_.previewDirty = true;
}

void App::handleLoftInput(float vpW, float vpH) {
    (void)vpW; (void)vpH;
    ImGuiIO& io = ImGui::GetIO();

    if (!io.WantCaptureKeyboard) {
        if (ImGui::IsKeyPressed(ImGuiKey_Escape)) {
            cancelLoft();
            return;
        }
        if (ImGui::IsKeyPressed(ImGuiKey_Enter) || ImGui::IsKeyPressed(ImGuiKey_KeypadEnter)) {
            if (loftTool_.canCommit()) {
                commitLoft();
                return;
            }
        }
    }
}

void App::drawLoftPanel() {
    ImGuiIO& io = ImGui::GetIO();
    float panelW = 250.0f;

    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, {10, 10});
    ImGui::SetNextWindowPos({io.DisplaySize.x - panelW, 30});
    ImGui::SetNextWindowSize({panelW, 0});
    ImGui::Begin("Loft", nullptr,
        ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove |
        ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_AlwaysAutoResize);

    ImGui::Text("Select profiles on different");
    ImGui::Text("sketch planes to loft between.");
    ImGui::Separator();

    // List current sections
    for (int i = 0; i < (int)loftTool_.sections.size(); i++) {
        const auto& sec = loftTool_.sections[i];
        char label[64];
        snprintf(label, sizeof(label), "Section %d: Plane %d, Profile %d", i+1, sec.sketchPlaneIndex, sec.profileIndex);
        ImGui::TextUnformatted(label);
        ImGui::SameLine();
        ImGui::PushID(i);
        if (ImGui::SmallButton("X")) {
            loftTool_.sections.erase(loftTool_.sections.begin() + i);
            loftTool_.previewDirty = true;
            ImGui::PopID();
            break; // invalidated iterator
        }
        ImGui::PopID();
    }

    ImGui::Separator();

    // Add section: pick a sketch plane that has profiles
    ImGui::Text("Add section:");
    for (int pi = 0; pi < (int)sketchPlanes_.size(); pi++) {
        auto& sp = sketchPlanes_[pi];
        if (sp.sketch.points.empty() && sp.sketch.lines.empty() && sp.sketch.circles.empty())
            continue;

        auto profiles = detectClosedProfiles(sp.sketch, sp);
        if (profiles.empty()) continue;

        // Check not already added
        bool alreadyUsed = false;
        for (const auto& sec : loftTool_.sections) {
            if (sec.sketchPlaneIndex == pi) { alreadyUsed = true; break; }
        }
        if (alreadyUsed) continue;

        char btnLabel[64];
        snprintf(btnLabel, sizeof(btnLabel), "%s (%d profiles)", sp.name.c_str(), (int)profiles.size());
        if (ImGui::Button(btnLabel)) {
            LoftToolSection sec;
            sec.sketchPlaneIndex = pi;
            sec.detectedProfiles = profiles;
            // Auto-select first profile
            sec.profileIndex = 0;
            buildProfileRenderCaches(sp.sketch, sec.detectedProfiles, sec.renderCache);
            loftTool_.sections.push_back(std::move(sec));
            loftTool_.previewDirty = true;
        }
    }

    // Profile selection per section
    if (!loftTool_.sections.empty()) {
        ImGui::Separator();
        ImGui::Text("Profile selection:");
        for (int i = 0; i < (int)loftTool_.sections.size(); i++) {
            auto& sec = loftTool_.sections[i];
            int numProfiles = (int)sec.detectedProfiles.size();
            if (numProfiles <= 1) continue;

            char label[64];
            snprintf(label, sizeof(label), "Section %d profile", i+1);
            ImGui::PushID(100 + i);
            if (ImGui::SliderInt(label, &sec.profileIndex, 0, numProfiles - 1)) {
                loftTool_.previewDirty = true;
            }
            ImGui::PopID();
        }
    }

    // Solid toggle
    ImGui::Separator();
    if (ImGui::Checkbox("Solid", &loftTool_.solid)) {
        loftTool_.previewDirty = true;
    }

    // OK / Cancel
    ImGui::Separator();
    bool canCommit = loftTool_.canCommit();
    if (!canCommit) ImGui::BeginDisabled();
    if (ImGui::Button("OK [Enter]", {95, 0})) {
        commitLoft();
    }
    if (!canCommit) ImGui::EndDisabled();
    ImGui::SameLine();
    if (ImGui::Button("Cancel [Esc]", {95, 0})) {
        cancelLoft();
    }

    ImGui::End();
    ImGui::PopStyleVar();
}

void App::updateLoftPreview() {
    loftTool_.previewDirty = false;

    // Restore body visibility from any previous preview
    if (loftTool_.hidingBodiesForPreview) {
        for (size_t i = 0; i < scene_.bodyCount(); i++)
            scene_.getBodyMut((int)i).visible = true;
        loftTool_.hidingBodiesForPreview = false;
    }

    // Clean up old preview resources
    if (loftTool_.previewBody.vao) {
        glDeleteVertexArrays(1, &loftTool_.previewBody.vao);
        loftTool_.previewBody.vao = 0;
    }
    if (loftTool_.previewBody.vbo) {
        glDeleteBuffers(1, &loftTool_.previewBody.vbo);
        loftTool_.previewBody.vbo = 0;
    }
    if (loftTool_.previewBody.edgeVAO) {
        glDeleteVertexArrays(1, &loftTool_.previewBody.edgeVAO);
        loftTool_.previewBody.edgeVAO = 0;
    }
    if (loftTool_.previewBody.edgeVBO) {
        glDeleteBuffers(1, &loftTool_.previewBody.edgeVBO);
        loftTool_.previewBody.edgeVBO = 0;
    }
    loftTool_.previewBody.vertices.clear();
    loftTool_.previewBody.vertexCount = 0;
    loftTool_.previewBody.edgeVertexCount = 0;

    for (auto& b : loftTool_.cutPreviewBodies) {
        if (b.vao) glDeleteVertexArrays(1, &b.vao);
        if (b.vbo) glDeleteBuffers(1, &b.vbo);
        if (b.edgeVAO) glDeleteVertexArrays(1, &b.edgeVAO);
        if (b.edgeVBO) glDeleteBuffers(1, &b.edgeVBO);
    }
    loftTool_.cutPreviewBodies.clear();

    if (!loftTool_.canCommit()) {
        return;
    }

    std::vector<LoftWireInput> inputs;
    for (const auto& sec : loftTool_.sections) {
        if (sec.profileIndex < 0 || sec.profileIndex >= (int)sec.detectedProfiles.size()) continue;
        if (sec.sketchPlaneIndex < 0 || sec.sketchPlaneIndex >= (int)sketchPlanes_.size()) continue;
        LoftWireInput wi;
        wi.sketch = &sketchPlanes_[sec.sketchPlaneIndex].sketch;
        wi.profile = &sec.detectedProfiles[sec.profileIndex];
        wi.plane = &sketchPlanes_[sec.sketchPlaneIndex];
        inputs.push_back(wi);
    }

    if (inputs.size() < 2) {
        return;
    }

    TopoDS_Shape previewShape = loftProfiles(inputs, loftTool_.solid);

    loftTool_.previewBody = Body3D{};
    if (!previewShape.IsNull()) {
        Scene3D::triangulateShape(previewShape, loftTool_.previewBody.vertices);
    }

    if (loftTool_.operation == ExtrudeOperation::Cut) {
        loftTool_.previewBody.colorR = 0.9f;
        loftTool_.previewBody.colorG = 0.3f;
        loftTool_.previewBody.colorB = 0.3f;
    } else {
        loftTool_.previewBody.colorR = 0.4f;
        loftTool_.previewBody.colorG = 0.6f;
        loftTool_.previewBody.colorB = 0.9f;
    }
    Scene3D::uploadMesh(loftTool_.previewBody);
}

void App::renderLoftPreview(const float* view, const float* proj, const float* eyePos) {
    auto& body = loftTool_.previewBody;
    if (body.vao == 0 || body.vertexCount == 0) return;

    auto& shader = viewport3D_.meshShader();
    shader.use();
    shader.setMat4("uView", view);
    shader.setMat4("uProj", proj);
    shader.setVec3("uEyePos", eyePos[0], eyePos[1], eyePos[2]);
    shader.setVec3("uLightDir", 0.3f, 0.8f, 0.5f);
    applyClip(shader, nullptr); // tool previews are never sectioned

    float alpha = (loftTool_.operation == ExtrudeOperation::Cut) ? 0.4f : 0.5f;
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

void App::commitLoft() {
    if (!loftTool_.canCommit()) return;

    // Ensure all source sketches are saved
    for (const auto& sec : loftTool_.sections) {
        FeatureID srcSketch = featureHistory_.findSketchFeatureForPlane(sec.sketchPlaneIndex);
        if (srcSketch == NullFeatureID) {
            srcSketch = featureHistory_.addSketchFeature(sec.sketchPlaneIndex,
                sketchPlanes_[sec.sketchPlaneIndex].sketch,
                sketchPlanes_[sec.sketchPlaneIndex].planeID);
            UndoCommand skCmd;
            skCmd.type = UndoActionType::AddFeature;
            skCmd.addedFeature = *featureHistory_.findFeature(srcSketch);
            globalUndo_.push(std::move(skCmd)); markDirty();
        }
    }

    LoftFeatureData ld;
    ld.operation = loftTool_.operation;
    ld.solid = loftTool_.solid;

    for (const auto& sec : loftTool_.sections) {
        LoftSection ls;
        ls.sourceSketchFeature = featureHistory_.findSketchFeatureForPlane(sec.sketchPlaneIndex);
        if (sec.profileIndex >= 0 && sec.profileIndex < (int)sec.detectedProfiles.size()) {
            ls.profileSig = ProfileSignature::fromProfile(sec.detectedProfiles[sec.profileIndex], sketchPlanes_[sec.sketchPlaneIndex].sketch);
            ls.profileIndexFallback = sec.profileIndex;
        }
        ld.sections.push_back(ls);
    }

    if (loftTool_.editingFeatureID != NullFeatureID) {
        featureHistory_.unsuppressFeature(loftTool_.editingFeatureID);
        Feature* existing = featureHistory_.findFeature(loftTool_.editingFeatureID);
        if (existing) {
            UndoCommand cmd;
            cmd.type = UndoActionType::ModifyLoft;
            cmd.featureID = loftTool_.editingFeatureID;
            cmd.oldLoft = std::get<LoftFeatureData>(existing->data);
            cmd.newLoft = ld;
            globalUndo_.push(std::move(cmd)); markDirty();
            featureHistory_.updateLoftData(loftTool_.editingFeatureID, ld);
        }
    } else {
        FeatureID loftFID = featureHistory_.addLoftFeature(ld);
        UndoCommand cmd;
        cmd.type = UndoActionType::AddFeature;
        cmd.addedFeature = *featureHistory_.findFeature(loftFID);
        globalUndo_.push(std::move(cmd)); markDirty();
    }

    replayAllFeatures();

    loftTool_.reset();
    tool_.type = ToolType::None;
    tool_.reset();

    if (mode_ == InteractionMode::Sketching) {
        finishSketch(false);
    }
}

void App::cancelLoft() {
    if (loftTool_.hidingBodiesForPreview) {
        for (size_t i = 0; i < scene_.bodyCount(); i++)
            scene_.getBodyMut((int)i).visible = true;
    }

    if (loftTool_.editingFeatureID != NullFeatureID) {
        featureHistory_.unsuppressFeature(loftTool_.editingFeatureID);
    }

    loftTool_.reset();
    tool_.type = ToolType::None;
    tool_.reset();
    replayAllFeatures();
}

void App::editLoftFeature(FeatureID id) {
    const Feature* feat = featureHistory_.findFeature(id);
    if (!feat || feat->type != FeatureType::Loft) return;

    const auto& ld = std::get<LoftFeatureData>(feat->data);

    if (mode_ == InteractionMode::Sketching) {
        finishSketch();
    }

    tool_.type = ToolType::Loft;
    tool_.reset();
    selection_.clear();

    loftTool_.reset();
    loftTool_.editingFeatureID = id;
    loftTool_.operation = ld.operation;
    loftTool_.solid = ld.solid;

    // Reconstruct sections from saved data
    for (const auto& ls : ld.sections) {
        const Feature* srcFeat = featureHistory_.findFeature(ls.sourceSketchFeature);
        if (!srcFeat || srcFeat->type != FeatureType::Sketch) continue;
        const auto& sd = std::get<SketchFeatureData>(srcFeat->data);
        int planeIdx = sd.sketchPlaneIndex;
        if (planeIdx < 0 || planeIdx >= (int)sketchPlanes_.size()) continue;

        auto profiles = detectClosedProfiles(sketchPlanes_[planeIdx].sketch, sketchPlanes_[planeIdx]);
        if (profiles.empty()) continue;

        LoftToolSection sec;
        sec.sketchPlaneIndex = planeIdx;
        sec.detectedProfiles = profiles;

        // Match the saved profile signature to current detected profiles
        std::vector<ProfileSignature> sigVec = {ls.profileSig};
        std::vector<int> fbVec = {ls.profileIndexFallback};
        auto matched = matchProfiles(sigVec, fbVec, profiles, sketchPlanes_[planeIdx].sketch);
        if (!matched.empty()) {
            sec.profileIndex = *matched.begin();
        } else {
            sec.profileIndex = std::min(ls.profileIndexFallback, (int)profiles.size() - 1);
        }

        buildProfileRenderCaches(sketchPlanes_[planeIdx].sketch, sec.detectedProfiles, sec.renderCache);
        loftTool_.sections.push_back(std::move(sec));
    }

    // Suppress the feature being edited
    featureHistory_.suppressFeature(id);
    replayAllFeatures();

    loftTool_.previewDirty = true;
}

} // namespace shitcad
