#include "App.h"
#include "FacePicker.h"
#include "Extrude.h"
#include "FeatureReplay.h"
#include <BRepAlgoAPI_Cut.hxx>
#include <BRepAlgoAPI_Fuse.hxx>
#include <imgui.h>

namespace shitcad {

void App::enterBooleanMode(BooleanOperation op) {
    if (scene_.bodyCount() < 2) return;

    tool_.type = (op == BooleanOperation::Union) ? ToolType::BooleanUnion : ToolType::BooleanSubtract;
    tool_.reset();
    booleanTool_.reset();
    booleanTool_.operation = op;
}

void App::handleBooleanInput(float vpW, float vpH) {
    if (ImGui::IsKeyPressed(ImGuiKey_Escape)) {
        cancelBoolean();
        return;
    }
    if (ImGui::IsKeyPressed(ImGuiKey_Enter) && booleanTool_.canCommit()) {
        commitBoolean();
        return;
    }

    // Click to pick bodies
    if (ImGui::IsMouseClicked(ImGuiMouseButton_Left) && !ImGui::GetIO().WantCaptureMouse) {
        float mx = ImGui::GetMousePos().x;
        float my = ImGui::GetMousePos().y;
        ImVec2 vMin = ImGui::GetMainViewport()->WorkPos;

        float view[16], proj[16];
        getViewProj((int)vpW, (int)vpH, view, proj);

        float rayOrig[3], rayDir[3];
        screenToRay(mx, my, vMin.x, vMin.y, vpW, vpH, view, proj, rayOrig, rayDir);

        FacePickResult hit = pickFace(scene_, rayOrig, rayDir, &section_);
        if (hit.hit) {
            int bodyIdx = hit.bodyIndex;

            if (!booleanTool_.hasTarget()) {
                booleanTool_.targetBodyIndex = bodyIdx;
                booleanTool_.previewDirty = true;
            } else if (!booleanTool_.hasTool()) {
                if (bodyIdx != booleanTool_.targetBodyIndex) {
                    booleanTool_.toolBodyIndex = bodyIdx;
                    booleanTool_.previewDirty = true;
                }
            } else {
                // Both selected — click to change tool body
                if (bodyIdx != booleanTool_.targetBodyIndex) {
                    booleanTool_.toolBodyIndex = bodyIdx;
                    booleanTool_.previewDirty = true;
                }
            }
        }
    }
}

void App::drawBooleanPanel() {
    ImGui::SetNextWindowPos({ImGui::GetMainViewport()->WorkPos.x + ImGui::GetMainViewport()->WorkSize.x - 250,
                             ImGui::GetMainViewport()->WorkPos.y + 60}, ImGuiCond_Always);
    ImGui::SetNextWindowSize({240, 0}, ImGuiCond_Always);

    const char* title = (booleanTool_.operation == BooleanOperation::Union) ? "Union Bodies" : "Subtract Bodies";
    ImGui::Begin(title, nullptr, ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse);

    if (booleanTool_.operation == BooleanOperation::Union) {
        ImGui::TextWrapped("Pick two bodies to combine into one.");
    } else {
        ImGui::TextWrapped("Pick target body, then tool body to cut away.");
    }

    ImGui::Separator();

    // Target body
    if (booleanTool_.hasTarget()) {
        ImGui::Text("Target: Body %d", booleanTool_.targetBodyIndex);
        ImGui::SameLine();
        if (ImGui::SmallButton("Clear##target")) {
            booleanTool_.targetBodyIndex = -1;
            booleanTool_.toolBodyIndex = -1;
            booleanTool_.previewValid = false;
        }
    } else {
        ImGui::TextColored({1, 1, 0, 1}, "Click to select target body");
    }

    // Tool body
    if (booleanTool_.hasTool()) {
        if (booleanTool_.operation == BooleanOperation::Union)
            ImGui::Text("Other: Body %d", booleanTool_.toolBodyIndex);
        else
            ImGui::Text("Tool:  Body %d", booleanTool_.toolBodyIndex);
        ImGui::SameLine();
        if (ImGui::SmallButton("Clear##tool")) {
            booleanTool_.toolBodyIndex = -1;
            booleanTool_.previewValid = false;
        }
    } else if (booleanTool_.hasTarget()) {
        if (booleanTool_.operation == BooleanOperation::Union)
            ImGui::TextColored({1, 1, 0, 1}, "Click another body to combine");
        else
            ImGui::TextColored({1, 1, 0, 1}, "Click body to subtract");
    }

    ImGui::Separator();

    bool canCommit = booleanTool_.canCommit();
    if (!canCommit) ImGui::BeginDisabled();
    if (ImGui::Button("Apply [Enter]", {-1, 0})) {
        commitBoolean();
    }
    if (!canCommit) ImGui::EndDisabled();

    if (ImGui::Button("Cancel [Esc]", {-1, 0})) {
        cancelBoolean();
    }

    if (booleanTool_.previewValid) {
        ImGui::Separator();
        ImGui::TextColored({0, 1, 0, 1}, "Preview ready");
    }

    ImGui::End();
}

void App::updateBooleanPreview() {
    booleanTool_.previewDirty = false;
    booleanTool_.previewValid = false;

    if (!booleanTool_.canCommit()) return;
    if (booleanTool_.targetBodyIndex >= (int)scene_.bodyCount()) return;
    if (booleanTool_.toolBodyIndex >= (int)scene_.bodyCount()) return;

    const auto& targetShape = scene_.getBody(booleanTool_.targetBodyIndex).shape;
    const auto& toolShape = scene_.getBody(booleanTool_.toolBodyIndex).shape;

    TopoDS_Shape result;
    if (booleanTool_.operation == BooleanOperation::Union) {
        BRepAlgoAPI_Fuse fuser(targetShape, toolShape);
        if (!fuser.IsDone() || fuser.HasErrors()) return;
        result = fuser.Shape();
    } else {
        BRepAlgoAPI_Cut cutter(targetShape, toolShape);
        if (!cutter.IsDone() || cutter.HasErrors()) return;
        result = cutter.Shape();
    }

    if (result.IsNull()) return;

    // Triangulate for preview
    Body3D body;
    body.shape = result;
    Scene3D::triangulateShape(result, body.vertices);
    if (body.vertices.empty()) return;

    booleanTool_.previewBody = std::move(body);
    booleanTool_.previewValid = true;
}

void App::renderBooleanPreview(const float* view, const float* proj, const float* eyePos) {
    (void)view; (void)proj; (void)eyePos;
    if (!booleanTool_.previewValid) return;

    // Highlight target body in green, tool body in red/blue
    if (booleanTool_.hasTarget() && booleanTool_.targetBodyIndex < (int)scene_.bodyCount()) {
        scene_.getBodyMut(booleanTool_.targetBodyIndex).colorR = 0.2f;
        scene_.getBodyMut(booleanTool_.targetBodyIndex).colorG = 0.8f;
        scene_.getBodyMut(booleanTool_.targetBodyIndex).colorB = 0.2f;
    }
    if (booleanTool_.hasTool() && booleanTool_.toolBodyIndex < (int)scene_.bodyCount()) {
        if (booleanTool_.operation == BooleanOperation::Subtract) {
            scene_.getBodyMut(booleanTool_.toolBodyIndex).colorR = 0.8f;
            scene_.getBodyMut(booleanTool_.toolBodyIndex).colorG = 0.2f;
            scene_.getBodyMut(booleanTool_.toolBodyIndex).colorB = 0.2f;
        } else {
            scene_.getBodyMut(booleanTool_.toolBodyIndex).colorR = 0.2f;
            scene_.getBodyMut(booleanTool_.toolBodyIndex).colorG = 0.4f;
            scene_.getBodyMut(booleanTool_.toolBodyIndex).colorB = 0.8f;
        }
    }
}

void App::commitBoolean() {
    if (!booleanTool_.canCommit()) return;
    if (booleanTool_.targetBodyIndex >= (int)scene_.bodyCount()) return;
    if (booleanTool_.toolBodyIndex >= (int)scene_.bodyCount()) return;

    BooleanFeatureData bd;
    bd.operation = booleanTool_.operation;
    bd.targetBodyIndex = booleanTool_.targetBodyIndex;
    bd.toolBodyIndex = booleanTool_.toolBodyIndex;

    FeatureID fid = featureHistory_.addBooleanFeature(bd);

    UndoCommand cmd;
    cmd.type = UndoActionType::AddFeature;
    cmd.addedFeature = *featureHistory_.findFeature(fid);
    globalUndo_.push(std::move(cmd));
    markDirty();

    // Reset body colors before replay
    for (size_t i = 0; i < scene_.bodyCount(); i++) {
        scene_.getBodyMut((int)i).colorR = 0.6f;
        scene_.getBodyMut((int)i).colorG = 0.6f;
        scene_.getBodyMut((int)i).colorB = 0.65f;
    }

    replayAllFeatures();

    booleanTool_.reset();
    tool_.type = ToolType::None;
    tool_.reset();
}

void App::cancelBoolean() {
    // Reset body colors
    for (size_t i = 0; i < scene_.bodyCount(); i++) {
        scene_.getBodyMut((int)i).colorR = 0.6f;
        scene_.getBodyMut((int)i).colorG = 0.6f;
        scene_.getBodyMut((int)i).colorB = 0.65f;
    }

    booleanTool_.reset();
    tool_.type = ToolType::None;
    tool_.reset();
}

} // namespace shitcad
