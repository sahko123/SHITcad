#include "App.h"
#include "ProfileDetector.h"
#include "Extrude.h"
#include "AutoConstraint.h"
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
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <algorithm>
#include <string>

namespace shitcad {

void App::drawPreferencesWindow() {
    ImGui::SetNextWindowSize({220, 0}, ImGuiCond_FirstUseEver);
    ImGui::Begin("Preferences", &prefsOpen_);

    bool changed = false;
    if (ImGui::Checkbox("Light Mode", &prefs_.lightMode)) {
        changed = true;
    }
    ImGui::Checkbox("Show Edges", &prefs_.showWireframe);

    if (changed) {
        prefs_.applyTheme();
        viewport3D_.rebuildGrid();
    }

    ImGui::Separator();
    ImGui::Text("Sketch Lines");
    ImGui::ColorEdit3("Line Color", prefs_.sketchLineColor, ImGuiColorEditFlags_NoInputs);
    ImGui::SliderFloat("Line Width", &prefs_.sketchLineThickness, 0.5f, 5.0f, "%.1f");

    ImGui::Separator();
    ImGui::Text("Body Edges");
    ImGui::ColorEdit3("Edge Color", prefs_.edgeColor, ImGuiColorEditFlags_NoInputs);
    ImGui::SliderFloat("Edge Width", &prefs_.edgeThickness, 0.5f, 5.0f, "%.1f");

    ImGui::Separator();
    ImGui::Text("Dimension Labels");
    bool dimChanged = false;
    dimChanged |= ImGui::ColorEdit4("Dim Line", prefs_.dimLineCol, ImGuiColorEditFlags_NoInputs | ImGuiColorEditFlags_AlphaBar);
    dimChanged |= ImGui::ColorEdit4("Dim Text", prefs_.dimTextCol, ImGuiColorEditFlags_NoInputs | ImGuiColorEditFlags_AlphaBar);
    dimChanged |= ImGui::ColorEdit4("Dim Bg", prefs_.dimBgCol, ImGuiColorEditFlags_NoInputs | ImGuiColorEditFlags_AlphaBar);
    if (dimChanged) {
        auto toU32 = [](const float c[4]) -> ImU32 {
            return IM_COL32((int)(c[0]*255), (int)(c[1]*255), (int)(c[2]*255), (int)(c[3]*255));
        };
        auto& tm = activeThemeMut();
        tm.dimLineColor = toU32(prefs_.dimLineCol);
        tm.dimTextColor = toU32(prefs_.dimTextCol);
        tm.dimBgColor = toU32(prefs_.dimBgCol);
    }

    ImGui::Separator();
    ImGui::Text("Constraint Labels");
    ImGui::ColorEdit4("Con Text", prefs_.conTextCol, ImGuiColorEditFlags_NoInputs | ImGuiColorEditFlags_AlphaBar);
    ImGui::ColorEdit4("Con Bg", prefs_.conBgCol, ImGuiColorEditFlags_NoInputs | ImGuiColorEditFlags_AlphaBar);

    ImGui::Separator();
    ImGui::Text("Snapping");
    ImGui::SliderFloat("Tangent Snap (px)", &prefs_.tangentSnapPx, 5.0f, 40.0f, "%.0f");

    ImGui::End();
}

void App::replayAllFeatures() {
    shitcad::replayFeatures(featureHistory_, sketchPlanes_, scene_);
}

void App::globalUndo() {
    UndoCommand cmd;
    if (!globalUndo_.undoStep(cmd)) return;

    switch (cmd.type) {
        case UndoActionType::AddFeature:
            featureHistory_.removeFeature(cmd.addedFeature.id);
            break;
        case UndoActionType::DeleteFeature:
            for (const auto& [idx, feat] : cmd.deletedFeatures) {
                featureHistory_.insertFeatureAt(idx, feat);
            }
            break;
        case UndoActionType::SuppressFeature:
            featureHistory_.unsuppressFeature(cmd.featureID);
            break;
        case UndoActionType::UnsuppressFeature:
            featureHistory_.suppressFeature(cmd.featureID);
            break;
        case UndoActionType::RenameFeature:
            featureHistory_.renameFeature(cmd.featureID, cmd.oldName);
            break;
        case UndoActionType::SetRollbackPos:
            featureHistory_.setRollbackPos(cmd.oldRollbackPos);
            break;
        case UndoActionType::ModifySketch:
            featureHistory_.updateSketchSnapshot(cmd.featureID, cmd.oldSketch);
            break;
        case UndoActionType::ModifyExtrude:
            featureHistory_.updateExtrudeData(cmd.featureID, cmd.oldExtrude);
            break;
        case UndoActionType::ModifyRevolve:
            featureHistory_.updateRevolveData(cmd.featureID, cmd.oldRevolve);
            break;
        case UndoActionType::ModifyLoft:
            featureHistory_.updateLoftData(cmd.featureID, cmd.oldLoft);
            break;
        case UndoActionType::ModifyBoolean:
            featureHistory_.updateBooleanData(cmd.featureID, cmd.oldBoolean);
            break;
    }
    replayAllFeatures();
    markDirty();
}

void App::globalRedo() {
    UndoCommand cmd;
    if (!globalUndo_.redoStep(cmd)) return;

    switch (cmd.type) {
        case UndoActionType::AddFeature:
            featureHistory_.insertFeatureAt((int)featureHistory_.size(), cmd.addedFeature);
            break;
        case UndoActionType::DeleteFeature:
            for (int i = (int)cmd.deletedFeatures.size() - 1; i >= 0; i--) {
                featureHistory_.removeFeature(cmd.deletedFeatures[i].second.id);
            }
            break;
        case UndoActionType::SuppressFeature:
            featureHistory_.suppressFeature(cmd.featureID);
            break;
        case UndoActionType::UnsuppressFeature:
            featureHistory_.unsuppressFeature(cmd.featureID);
            break;
        case UndoActionType::RenameFeature:
            featureHistory_.renameFeature(cmd.featureID, cmd.newName);
            break;
        case UndoActionType::SetRollbackPos:
            featureHistory_.setRollbackPos(cmd.newRollbackPos);
            break;
        case UndoActionType::ModifySketch:
            featureHistory_.updateSketchSnapshot(cmd.featureID, cmd.newSketch);
            break;
        case UndoActionType::ModifyExtrude:
            featureHistory_.updateExtrudeData(cmd.featureID, cmd.newExtrude);
            break;
        case UndoActionType::ModifyRevolve:
            featureHistory_.updateRevolveData(cmd.featureID, cmd.newRevolve);
            break;
        case UndoActionType::ModifyLoft:
            featureHistory_.updateLoftData(cmd.featureID, cmd.newLoft);
            break;
        case UndoActionType::ModifyBoolean:
            featureHistory_.updateBooleanData(cmd.featureID, cmd.newBoolean);
            break;
    }
    replayAllFeatures();
    markDirty();
}

void App::saveProjectDialog() {
    if (currentFilePath_.empty()) {
        std::string path = openNativeSaveDialog();
        if (path.empty()) return;
        currentFilePath_ = path;
    }
    if (!saveProject(currentFilePath_, featureHistory_, sketchPlanes_)) {
        fprintf(stderr, "Save failed: %s\n", lastLoadError().c_str());
    } else {
        unsavedChanges_ = false;
        updateWindowTitle();
    }
}

void App::openProjectDialog() {
    std::string path = openNativeOpenDialog();
    if (path.empty()) return;

    FeatureHistory newHistory;
    std::vector<SketchPlane> newPlanes;

    if (!loadProject(path, newHistory, newPlanes)) {
        fprintf(stderr, "Load failed: %s\n", lastLoadError().c_str());
        return;
    }

    // Reset state
    if (mode_ == InteractionMode::Sketching) {
        finishSketch(false);
    }
    tool_ = {};
    arcTool_.reset();
    selection_.clear();
    dimTool_.reset();
    globalUndo_.clear();
    history_.clear();
    activeSketchPlane_ = -1;
    extrudeTool_.reset();
    mode_ = InteractionMode::Navigate;

    // Apply loaded data
    featureHistory_ = std::move(newHistory);
    sketchPlanes_ = std::move(newPlanes);
    currentFilePath_ = path;

    // Restore nextPlaneID_ from loaded planes
    nextPlaneID_ = 1;
    for (const auto& sp : sketchPlanes_) {
        if (sp.planeID >= nextPlaneID_) nextPlaneID_ = sp.planeID + 1;
    }

    // Rebuild 3D
    replayAllFeatures();
    unsavedChanges_ = false;
    updateWindowTitle();
}

void App::exportStlDialog() {
    if (scene_.empty()) return;
    std::string path = openNativeStlSaveDialog();
    if (path.empty()) return;
    if (!exportSTL(path, scene_)) {
        fprintf(stderr, "STL export failed: %s\n", lastLoadError().c_str());
    }
}

void App::importStlDialog() {
    std::string path = openNativeStlOpenDialog();
    if (path.empty()) return;
    if (!importSTL(path, scene_)) {
        fprintf(stderr, "STL import failed: %s\n", lastLoadError().c_str());
    }
}

void App::exportStepDialog() {
    if (scene_.empty()) return;
    std::string path = openNativeStepSaveDialog();
    if (path.empty()) return;
    if (!exportSTEP(path, scene_)) {
        fprintf(stderr, "STEP export failed: %s\n", lastLoadError().c_str());
    }
}

void App::importStepDialog() {
    std::string path = openNativeStepOpenDialog();
    if (path.empty()) return;
    if (!importSTEP(path, scene_)) {
        fprintf(stderr, "STEP import failed: %s\n", lastLoadError().c_str());
    }
}

void App::exportIgesDialog() {
    if (scene_.empty()) return;
    std::string path = openNativeIgesSaveDialog();
    if (path.empty()) return;
    if (!exportIGES(path, scene_)) {
        fprintf(stderr, "IGES export failed: %s\n", lastLoadError().c_str());
    }
}

void App::importIgesDialog() {
    std::string path = openNativeIgesOpenDialog();
    if (path.empty()) return;
    if (!importIGES(path, scene_)) {
        fprintf(stderr, "IGES import failed: %s\n", lastLoadError().c_str());
    }
}

void App::exportObjDialog() {
    if (scene_.empty()) return;
    std::string path = openNativeObjSaveDialog();
    if (path.empty()) return;
    if (!exportOBJ(path, scene_)) {
        fprintf(stderr, "OBJ export failed: %s\n", lastLoadError().c_str());
    }
}

void App::exportDxfDialog() {
    if (!hasActiveSketch()) return;
    std::string path = openNativeDxfSaveDialog();
    if (path.empty()) return;
    if (!exportDXF(path, activeSketch())) {
        fprintf(stderr, "DXF export failed: %s\n", lastLoadError().c_str());
    }
}

void App::importModelDialog() {
    std::string path = openNativeImportDialog();
    if (path.empty()) return;

    // Determine format from extension
    std::string ext;
    auto dot = path.rfind('.');
    if (dot != std::string::npos) {
        ext = path.substr(dot);
        for (auto& ch : ext) ch = (char)tolower((unsigned char)ch);
    }

    bool ok = false;
    if (ext == ".step" || ext == ".stp") {
        ok = importSTEP(path, scene_);
    } else if (ext == ".igs" || ext == ".iges") {
        ok = importIGES(path, scene_);
    } else if (ext == ".stl") {
        ok = importSTL(path, scene_);
    } else {
        fprintf(stderr, "Unsupported import format: %s\n", ext.c_str());
        return;
    }

    if (!ok) {
        fprintf(stderr, "Import failed: %s\n", lastLoadError().c_str());
    }
}

void App::updateWindowTitle() {
    std::string title = "SHITcad";
    if (!currentFilePath_.empty()) {
        auto pos = currentFilePath_.find_last_of("/\\");
        title += " - " + (pos != std::string::npos ? currentFilePath_.substr(pos + 1) : currentFilePath_);
    }
    if (unsavedChanges_) {
        title += " *";
    }
    glfwSetWindowTitle(window_, title.c_str());
}

void App::markDirty() {
    if (!unsavedChanges_) {
        unsavedChanges_ = true;
        updateWindowTitle();
    }
}

void App::drawTimeline(float panelW) {
    if (featureHistory_.empty()) return;

    ImGuiIO& io = ImGui::GetIO();
    float vpW = io.DisplaySize.x;
    float vpH = io.DisplaySize.y;
    float timelineH = ImGui::GetFrameHeight() + ImGui::GetStyle().WindowPadding.y * 2.0f;

    ImGui::SetNextWindowPos({panelW, vpH - timelineH});
    ImGui::SetNextWindowSize({vpW - panelW, timelineH});
    ImGui::Begin("##timeline", nullptr,
        ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize |
        ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoCollapse |
        ImGuiWindowFlags_NoScrollbar |
        ImGuiWindowFlags_NoBringToFrontOnFocus | ImGuiWindowFlags_NoFocusOnAppearing |
        ImGuiWindowFlags_HorizontalScrollbar);

    const auto& features = featureHistory_.features();
    int rollbackPos = featureHistory_.rollbackPos();
    int numFeatures = (int)features.size();

    // First pass: draw buttons and record their right edges (screen X)
    std::vector<float> buttonRightEdges(numFeatures);
    float firstButtonLeft = 0.0f;

    for (int i = 0; i < numFeatures; i++) {
        const Feature& feat = features[i];

        if (i > 0) ImGui::SameLine();

        // Color based on type and state
        ImVec4 col;
        bool rolledBack = featureHistory_.isRolledBack(i);
        bool grayed = feat.suppressed || rolledBack;

        if (grayed) {
            col = ImVec4(0.4f, 0.4f, 0.4f, 1.0f);
        } else if (feat.hasError) {
            col = ImVec4(0.8f, 0.2f, 0.2f, 1.0f);
        } else if (feat.type == FeatureType::Sketch) {
            col = ImVec4(0.3f, 0.5f, 0.9f, 1.0f);
        } else if (feat.type == FeatureType::Extrude) {
            const auto& ed = std::get<ExtrudeFeatureData>(feat.data);
            if (ed.operation == ExtrudeOperation::Cut)
                col = ImVec4(0.9f, 0.4f, 0.2f, 1.0f);
            else
                col = ImVec4(0.9f, 0.6f, 0.2f, 1.0f);
        } else if (feat.type == FeatureType::Revolve) {
            const auto& rd = std::get<RevolveFeatureData>(feat.data);
            if (rd.operation == ExtrudeOperation::Cut)
                col = ImVec4(0.9f, 0.3f, 0.5f, 1.0f);
            else
                col = ImVec4(0.7f, 0.4f, 0.9f, 1.0f);
        } else if (feat.type == FeatureType::Loft) {
            col = ImVec4(0.3f, 0.8f, 0.5f, 1.0f);
        } else if (feat.type == FeatureType::Boolean) {
            const auto& bd = std::get<BooleanFeatureData>(feat.data);
            if (bd.operation == BooleanOperation::Subtract)
                col = ImVec4(0.9f, 0.3f, 0.3f, 1.0f);
            else
                col = ImVec4(0.2f, 0.7f, 0.9f, 1.0f);
        }

        ImGui::PushStyleColor(ImGuiCol_Button, col);
        ImGui::PushStyleColor(ImGuiCol_ButtonHovered, ImVec4(col.x * 1.2f, col.y * 1.2f, col.z * 1.2f, 1.0f));
        ImGui::PushStyleColor(ImGuiCol_ButtonActive, ImVec4(col.x * 0.8f, col.y * 0.8f, col.z * 0.8f, 1.0f));

        if (feat.hasError && !grayed) {
            ImGui::PushStyleColor(ImGuiCol_Border, ImVec4(1.0f, 0.0f, 0.0f, 1.0f));
            ImGui::PushStyleVar(ImGuiStyleVar_FrameBorderSize, 2.0f);
        }

        ImGui::PushID(i);
        ImGui::Button(feat.name.c_str(), ImVec2(0, ImGui::GetFrameHeight()));

        if (i == 0) firstButtonLeft = ImGui::GetItemRectMin().x;
        buttonRightEdges[i] = ImGui::GetItemRectMax().x;

        // Click to select, double-click to edit
        if (ImGui::IsItemClicked(0)) {
            selectedFeatureID_ = feat.id;
        }
        if (ImGui::IsItemHovered() && ImGui::IsMouseDoubleClicked(0)) {
            if (feat.type == FeatureType::Sketch && !feat.suppressed && !rolledBack) {
                const auto& sd = std::get<SketchFeatureData>(feat.data);
                if (mode_ != InteractionMode::Sketching) {
                    enterSketchMode(sd.sketchPlaneIndex);
                    selectedFeatureID_ = NullFeatureID; // deselect so Delete targets sketch geometry
                }
            } else if (feat.type == FeatureType::Extrude && !feat.suppressed && !rolledBack) {
                editExtrudeFeature(feat.id);
                selectedFeatureID_ = NullFeatureID;
            } else if (feat.type == FeatureType::Revolve && !feat.suppressed && !rolledBack) {
                editRevolveFeature(feat.id);
                selectedFeatureID_ = NullFeatureID;
            } else if (feat.type == FeatureType::Loft && !feat.suppressed && !rolledBack) {
                editLoftFeature(feat.id);
                selectedFeatureID_ = NullFeatureID;
            }
        }

        // Draw selection highlight
        if (feat.id == selectedFeatureID_) {
            ImDrawList* dl = ImGui::GetWindowDrawList();
            ImVec2 rMin = ImGui::GetItemRectMin();
            ImVec2 rMax = ImGui::GetItemRectMax();
            dl->AddRect(ImVec2(rMin.x - 2, rMin.y - 2), ImVec2(rMax.x + 2, rMax.y + 2),
                        IM_COL32(0, 200, 255, 255), 2.0f, 0, 3.0f);
        }

        if (feat.hasError && ImGui::IsItemHovered()) {
            ImGui::SetTooltip("%s", feat.errorMsg.c_str());
        }

        // Right-click context menu
        if (ImGui::BeginPopupContextItem()) {
            static char renameBuf[128] = {};
            if (ImGui::IsWindowAppearing()) {
                snprintf(renameBuf, sizeof(renameBuf), "%s", feat.name.c_str());
            }
            ImGui::SetNextItemWidth(120);
            if (ImGui::InputText("Name", renameBuf, sizeof(renameBuf), ImGuiInputTextFlags_EnterReturnsTrue)) {
                UndoCommand cmd;
                cmd.type = UndoActionType::RenameFeature;
                cmd.featureID = feat.id;
                cmd.oldName = feat.name;
                cmd.newName = renameBuf;
                globalUndo_.push(std::move(cmd)); markDirty();
                featureHistory_.renameFeature(feat.id, renameBuf);
                ImGui::CloseCurrentPopup();
            }
            ImGui::Separator();

            if (feat.suppressed) {
                if (ImGui::MenuItem("Unsuppress")) {
                    UndoCommand cmd;
                    cmd.type = UndoActionType::UnsuppressFeature;
                    cmd.featureID = feat.id;
                    globalUndo_.push(std::move(cmd)); markDirty();
                    featureHistory_.unsuppressFeature(feat.id);
                    replayAllFeatures();
                }
            } else {
                if (ImGui::MenuItem("Suppress")) {
                    UndoCommand cmd;
                    cmd.type = UndoActionType::SuppressFeature;
                    cmd.featureID = feat.id;
                    globalUndo_.push(std::move(cmd)); markDirty();
                    featureHistory_.suppressFeature(feat.id);
                    replayAllFeatures();
                }
            }

            if (ImGui::MenuItem("Delete")) {
                // Collect feature + dependents with their indices before deleting
                UndoCommand cmd;
                cmd.type = UndoActionType::DeleteFeature;
                cmd.featureID = feat.id;
                // Save the feature itself
                int idx = featureHistory_.featureIndex(feat.id);
                cmd.deletedFeatures.push_back({idx, feat});
                // Save dependents
                auto deps = featureHistory_.getDependents(feat.id);
                for (auto depID : deps) {
                    int depIdx = featureHistory_.featureIndex(depID);
                    const Feature* depFeat = featureHistory_.findFeature(depID);
                    if (depFeat) cmd.deletedFeatures.push_back({depIdx, *depFeat});
                }
                // Sort by index ascending for proper re-insertion on undo
                std::sort(cmd.deletedFeatures.begin(), cmd.deletedFeatures.end(),
                    [](const auto& a, const auto& b) { return a.first < b.first; });
                globalUndo_.push(std::move(cmd)); markDirty();

                featureHistory_.deleteFeature(feat.id);
                replayAllFeatures();
                ImGui::EndPopup();
                ImGui::PopID();
                if (feat.hasError && !grayed) {
                    ImGui::PopStyleVar();
                    ImGui::PopStyleColor();
                }
                ImGui::PopStyleColor(3);
                goto timeline_end;
            }

            ImGui::EndPopup();
        }

        ImGui::PopID();

        if (feat.hasError && !grayed) {
            ImGui::PopStyleVar();
            ImGui::PopStyleColor();
        }
        ImGui::PopStyleColor(3);
    }

    // Delete key on selected feature
    if (selectedFeatureID_ != NullFeatureID && ImGui::IsKeyPressed(ImGuiKey_Delete) && !ImGui::IsAnyItemActive()) {
        const Feature* selFeat = featureHistory_.findFeature(selectedFeatureID_);
        if (selFeat) {
            UndoCommand cmd;
            cmd.type = UndoActionType::DeleteFeature;
            cmd.featureID = selectedFeatureID_;
            int idx = featureHistory_.featureIndex(selectedFeatureID_);
            cmd.deletedFeatures.push_back({idx, *selFeat});
            auto deps = featureHistory_.getDependents(selectedFeatureID_);
            for (auto depID : deps) {
                int depIdx = featureHistory_.featureIndex(depID);
                const Feature* depFeat = featureHistory_.findFeature(depID);
                if (depFeat) cmd.deletedFeatures.push_back({depIdx, *depFeat});
            }
            std::sort(cmd.deletedFeatures.begin(), cmd.deletedFeatures.end(),
                [](const auto& a, const auto& b) { return a.first < b.first; });
            globalUndo_.push(std::move(cmd)); markDirty();
            featureHistory_.deleteFeature(selectedFeatureID_);
            selectedFeatureID_ = NullFeatureID;
            replayAllFeatures();
            goto timeline_end;
        }
    }

    // Draw draggable playhead
    {
        ImDrawList* dl = ImGui::GetWindowDrawList();
        float contentTop = ImGui::GetWindowPos().y + ImGui::GetStyle().WindowPadding.y;
        float barH = ImGui::GetFrameHeight();
        float handleW = 6.0f;

        // Playhead X position: after the rollback feature, or after last feature if not rolled back
        int headIdx = (rollbackPos >= 0) ? rollbackPos : numFeatures - 1;
        float headX = buttonRightEdges[headIdx] + 2.0f;

        // Draw the playhead bar
        ImVec2 pMin(headX - handleW * 0.5f, contentTop);
        ImVec2 pMax(headX + handleW * 0.5f, contentTop + barH);

        // Invisible button for drag interaction (overlay on top of the bar)
        ImGui::SetCursorScreenPos(ImVec2(pMin.x - 2, pMin.y));
        ImGui::InvisibleButton("##playhead", ImVec2(handleW + 4, barH));

        bool dragging = ImGui::IsItemActive();
        bool hovered = ImGui::IsItemHovered();
        bool justActivated = ImGui::IsItemActivated();

        // Record drag start position for undo
        if (justActivated) {
            dragStartRollbackPos_ = rollbackPos;
        }

        // While dragging, snap to nearest feature boundary
        if (dragging) {
            float mouseX = ImGui::GetIO().MousePos.x;
            int bestIdx = numFeatures - 1;
            float bestDist = 1e9f;
            for (int i = 0; i < numFeatures; i++) {
                float dist = std::fabs(mouseX - buttonRightEdges[i]);
                if (dist < bestDist) {
                    bestDist = dist;
                    bestIdx = i;
                }
            }

            int newPos = (bestIdx >= numFeatures - 1) ? -1 : bestIdx;
            if (newPos != rollbackPos) {
                featureHistory_.setRollbackPos(newPos);
                rollbackPos = newPos;
                // Defer replay until mouse stops moving (150ms idle)
                playheadReplayPending_ = true;
                playheadLastMoveTime_ = std::chrono::steady_clock::now();
            }

            // Replay after mouse idle for 150ms during drag
            if (playheadReplayPending_) {
                auto elapsed = std::chrono::steady_clock::now() - playheadLastMoveTime_;
                if (elapsed >= std::chrono::milliseconds(150)) {
                    replayAllFeatures();
                    playheadReplayPending_ = false;
                }
            }

            // Recompute headX based on new position
            headIdx = (rollbackPos >= 0) ? rollbackPos : numFeatures - 1;
            headX = buttonRightEdges[headIdx] + 2.0f;
            pMin = ImVec2(headX - handleW * 0.5f, contentTop);
            pMax = ImVec2(headX + handleW * 0.5f, contentTop + barH);
        }

        // Replay on drag release if still pending
        if (ImGui::IsItemDeactivated()) {
            if (playheadReplayPending_) {
                replayAllFeatures();
                playheadReplayPending_ = false;
            }
        }

        // Push undo command when drag ends
        if (ImGui::IsItemDeactivated() && dragStartRollbackPos_ != rollbackPos) {
            UndoCommand cmd;
            cmd.type = UndoActionType::SetRollbackPos;
            cmd.oldRollbackPos = dragStartRollbackPos_;
            cmd.newRollbackPos = rollbackPos;
            globalUndo_.push(std::move(cmd)); markDirty();
        }

        // Color: brighter when hovered/dragging
        ImU32 headCol = (hovered || dragging)
            ? IM_COL32(255, 60, 40, 255)
            : IM_COL32(220, 50, 30, 255);

        dl->AddRectFilled(pMin, pMax, headCol, 2.0f);

        // Draw small triangle at top as a grip indicator
        float triSize = 4.0f;
        dl->AddTriangleFilled(
            ImVec2(headX - triSize, contentTop),
            ImVec2(headX + triSize, contentTop),
            ImVec2(headX, contentTop + triSize * 1.2f),
            headCol);

        if (hovered || dragging) {
            ImGui::SetMouseCursor(ImGuiMouseCursor_ResizeEW);
        }
    }

timeline_end:
    ImGui::End();
}

void App::drawObjectTree() {
    ImGuiIO& io = ImGui::GetIO();
    float panelW = 200.0f;
    float toolbarH = ImGui::GetFrameHeight() + ImGui::GetStyle().WindowPadding.y * 2.0f;
    float panelH = io.DisplaySize.y - toolbarH;

    ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, {8, 8});
    ImGui::SetNextWindowPos({0, toolbarH});
    ImGui::SetNextWindowSize({panelW, panelH});
    ImGui::Begin("##objecttree", nullptr,
        ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoResize |
        ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoCollapse |
        ImGuiWindowFlags_NoBringToFrontOnFocus | ImGuiWindowFlags_NoFocusOnAppearing);

    ImGui::TextUnformatted("Object Tree");
    ImGui::Separator();

    // Reference Planes section
    if (ImGui::TreeNodeEx("Reference Planes", ImGuiTreeNodeFlags_DefaultOpen)) {
        // Show all reference planes (built-in + user-created)
        for (int i = 0; i < (int)sketchPlanes_.size(); i++) {
            if (!sketchPlanes_[i].isReferencePlane) continue;
            ImGui::PushID(i);
            ImGui::Checkbox("##vis", &sketchPlanes_[i].visible);
            ImGui::SameLine();
            if (ImGui::Selectable(sketchPlanes_[i].name.c_str(), false,
                    ImGuiSelectableFlags_AllowDoubleClick)) {
                if (ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left)) {
                    enterSketchMode(i);
                }
            }
            ImGui::PopID();
        }

        // Add Offset Plane button
        if (ImGui::SmallButton("+ Add Plane")) {
            addPlaneDialogOpen_ = true;
            addPlaneSourceIndex_ = 0;
            addPlaneOffset_ = 10.0f;
            snprintf(addPlaneOffsetBuf_, sizeof(addPlaneOffsetBuf_), "%.1f", addPlaneOffset_);
            snprintf(addPlaneNameBuf_, sizeof(addPlaneNameBuf_), "");
            addPlaneFromFace_ = false;
            addPlaneWaitingFace_ = false;
        }

        ImGui::TreePop();
    }

    // Sketches section (planes with geometry, index >= kRefPlaneCount)
    bool hasUserSketches = false;
    for (int i = kRefPlaneCount; i < (int)sketchPlanes_.size(); i++) {
        const auto& sp = sketchPlanes_[i];
        if (!sp.sketch.points.empty() || !sp.sketch.lines.empty() || !sp.sketch.circles.empty()) {
            hasUserSketches = true;
            break;
        }
    }
    // Also check reference planes that have sketch geometry
    bool hasAnySketch = hasUserSketches;
    for (int i = 0; i < kRefPlaneCount && !hasAnySketch; i++) {
        const auto& sp = sketchPlanes_[i];
        if (!sp.sketch.points.empty() || !sp.sketch.lines.empty() || !sp.sketch.circles.empty())
            hasAnySketch = true;
    }

    if (hasAnySketch && ImGui::TreeNodeEx("Sketches", ImGuiTreeNodeFlags_DefaultOpen)) {
        for (int i = 0; i < (int)sketchPlanes_.size(); i++) {
            const auto& sp = sketchPlanes_[i];
            if (sp.sketch.points.empty() && sp.sketch.lines.empty() && sp.sketch.circles.empty())
                continue;

            ImGui::PushID(100 + i);
            ImGui::Checkbox("##vis", &sketchPlanes_[i].sketchVisible);
            ImGui::SameLine();

            char label[64];
            if (i < kRefPlaneCount)
                snprintf(label, sizeof(label), "Sketch on %s", sp.name.c_str());
            else
                snprintf(label, sizeof(label), "%s", sp.name.c_str());

            if (ImGui::Selectable(label, false,
                    ImGuiSelectableFlags_AllowDoubleClick)) {
                if (ImGui::IsMouseDoubleClicked(ImGuiMouseButton_Left)) {
                    enterSketchMode(i);
                }
            }
            ImGui::PopID();
        }
        ImGui::TreePop();
    }

    // Bodies section
    if (!scene_.empty() && ImGui::TreeNodeEx("Bodies", ImGuiTreeNodeFlags_DefaultOpen)) {
        for (int i = 0; i < (int)scene_.bodyCount(); i++) {
            ImGui::PushID(200 + i);
            ImGui::Checkbox("##vis", &scene_.getBodyMut(i).visible);
            ImGui::SameLine();

            char label[32];
            snprintf(label, sizeof(label), "Body %d", i + 1);
            ImGui::TextUnformatted(label);
            ImGui::PopID();
        }
        ImGui::TreePop();
    }

    ImGui::End();
    ImGui::PopStyleVar();

    // Add Plane dialog
    if (addPlaneDialogOpen_) {
        ImGui::SetNextWindowSize({280, 0});
        ImGui::Begin("Add Reference Plane", &addPlaneDialogOpen_,
            ImGuiWindowFlags_AlwaysAutoResize | ImGuiWindowFlags_NoCollapse);

        // Source selection
        ImGui::Text("Source:");

        // Build list of reference planes + "Pick Face" option
        if (ImGui::RadioButton("From Plane", !addPlaneFromFace_)) {
            addPlaneFromFace_ = false;
            addPlaneWaitingFace_ = false;
        }
        if (!addPlaneFromFace_) {
            ImGui::Indent();
            for (int i = 0; i < (int)sketchPlanes_.size(); i++) {
                if (!sketchPlanes_[i].isReferencePlane) continue;
                if (ImGui::RadioButton(sketchPlanes_[i].name.c_str(), addPlaneSourceIndex_ == i)) {
                    addPlaneSourceIndex_ = i;
                }
            }
            ImGui::Unindent();
        }

        if (ImGui::RadioButton("From Face", addPlaneFromFace_)) {
            addPlaneFromFace_ = true;
            addPlaneWaitingFace_ = true;
        }
        if (addPlaneFromFace_) {
            ImGui::Indent();
            if (addPlaneWaitingFace_) {
                ImGui::TextColored({1.0f, 0.8f, 0.2f, 1.0f}, "Click a face in the viewport");
            } else {
                ImGui::TextColored({0.3f, 1.0f, 0.3f, 1.0f}, "Face selected");
            }
            ImGui::Unindent();
        }

        ImGui::Separator();

        // Offset
        ImGui::Text("Offset distance:");
        ImGui::SetNextItemWidth(120);
        if (ImGui::InputText("##offset", addPlaneOffsetBuf_, sizeof(addPlaneOffsetBuf_),
                ImGuiInputTextFlags_EnterReturnsTrue)) {
            addPlaneOffset_ = (float)atof(addPlaneOffsetBuf_);
        }

        // Name (optional)
        ImGui::Text("Name (optional):");
        ImGui::SetNextItemWidth(180);
        ImGui::InputText("##name", addPlaneNameBuf_, sizeof(addPlaneNameBuf_));

        ImGui::Separator();

        // Create button
        bool canCreate = !addPlaneFromFace_ || !addPlaneWaitingFace_;
        if (!canCreate) ImGui::BeginDisabled();
        if (ImGui::Button("Create", {80, 0})) {
            addPlaneOffset_ = (float)atof(addPlaneOffsetBuf_);

            const SketchPlane& src = addPlaneFromFace_
                ? addPlaneFaceSource_
                : sketchPlanes_[addPlaneSourceIndex_];
            SketchPlane newPlane;
            std::memcpy(newPlane.origin, src.origin, sizeof(src.origin));
            std::memcpy(newPlane.normal, src.normal, sizeof(src.normal));
            std::memcpy(newPlane.uAxis, src.uAxis, sizeof(src.uAxis));
            std::memcpy(newPlane.vAxis, src.vAxis, sizeof(src.vAxis));

            // Apply offset along normal
            newPlane.origin[0] += newPlane.normal[0] * addPlaneOffset_;
            newPlane.origin[1] += newPlane.normal[1] * addPlaneOffset_;
            newPlane.origin[2] += newPlane.normal[2] * addPlaneOffset_;

            // Set name
            if (addPlaneNameBuf_[0] != '\0') {
                newPlane.name = addPlaneNameBuf_;
            } else {
                // Auto-generate name
                const char* srcName = addPlaneFromFace_ ? "Face" : src.name.c_str();
                char autoName[128];
                snprintf(autoName, sizeof(autoName), "%s + %.1f", srcName, addPlaneOffset_);
                newPlane.name = autoName;
            }

            newPlane.isReferencePlane = true;
            newPlane.visible = true;
            newPlane.color[0] = 0.5f; newPlane.color[1] = 0.5f;
            newPlane.color[2] = 0.5f; newPlane.color[3] = 0.10f;

            newPlane.planeID = nextPlaneID_++;
            sketchPlanes_.push_back(std::move(newPlane));
            addPlaneDialogOpen_ = false;
            markDirty();
        }
        if (!canCreate) ImGui::EndDisabled();

        ImGui::SameLine();
        if (ImGui::Button("Cancel", {80, 0})) {
            addPlaneDialogOpen_ = false;
            addPlaneWaitingFace_ = false;
        }

        ImGui::End();
    }
}

void App::drawToolbar() {
    ImGui::PushStyleVar(ImGuiStyleVar_ItemSpacing, {4, 0});
    ImGui::PushStyleVar(ImGuiStyleVar_FramePadding, {8, 4});

    if (mode_ == InteractionMode::Sketching && activeSketchPlane_ >= 0) {
        // Sketch mode toolbar
        if (ImGui::Button("Finish Sketch [Esc]")) {
            finishSketch();
            ImGui::PopStyleVar(2);
            return;
        }
        ImGui::SameLine();
        ImGui::TextDisabled("|");
        ImGui::SameLine();

        const char* toolNames[] = {"[None]", "[P]oint", "[L]ine", "[C]ircle", "[R]ect", "[A]rc 3pt", "Arc Ctr", "Ctr Rect", "[D]im"};
        ToolType toolTypes[] = {ToolType::None, ToolType::Point, ToolType::Line, ToolType::Circle, ToolType::Rectangle, ToolType::Arc3Point, ToolType::ArcCenter, ToolType::CenterRect, ToolType::Dimension};
        for (int i = 0; i < 9; i++) {
            ImGui::SameLine();
            bool selected = (tool_.type == toolTypes[i]);
            if (selected) ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.3f, 0.5f, 0.8f, 1.0f));
            if (ImGui::Button(toolNames[i])) {
                switchTool(toolTypes[i]);
            }
            if (selected) ImGui::PopStyleColor();
        }

        ImGui::SameLine();
        ImGui::TextDisabled("|");
        ImGui::SameLine();
        bool extrudeActive = (tool_.type == ToolType::Extrude);
        if (extrudeActive)
            ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.3f, 0.5f, 0.8f, 1.0f));
        else
            ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.5f, 0.3f, 0.1f, 1.0f));
        if (ImGui::Button("[E]xtrude")) {
            if (extrudeActive)
                cancelExtrude();
            else
                enterExtrudeMode();
        }
        ImGui::PopStyleColor();

        ImGui::SameLine();
        bool revolveActive = (tool_.type == ToolType::Revolve);
        if (revolveActive)
            ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.3f, 0.5f, 0.8f, 1.0f));
        else
            ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.5f, 0.3f, 0.1f, 1.0f));
        if (ImGui::Button("Re[v]olve")) {
            if (revolveActive)
                cancelRevolve();
            else
                enterRevolveMode();
        }
        ImGui::PopStyleColor();

        ImGui::SameLine();
        bool loftActive = (tool_.type == ToolType::Loft);
        if (loftActive)
            ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.3f, 0.5f, 0.8f, 1.0f));
        else
            ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.5f, 0.3f, 0.1f, 1.0f));
        if (ImGui::Button("Loft")) {
            if (loftActive)
                cancelLoft();
            else
                enterLoftMode();
        }
        ImGui::PopStyleColor();

        ImGui::SameLine();
        ImGui::TextDisabled("|");
        ImGui::SameLine();
        if (ImGui::Button("[N] Snap View")) {
            orientCameraToPlane(activePlane());
        }

        // Constraint buttons
        ImGui::SameLine();
        ImGui::TextDisabled("|");
        ImGui::SameLine();
        {
            auto& sel = selection_.selected;
            auto countType = [&](HitType t) {
                int n = 0;
                for (auto& s : sel) if (s.type == t) n++;
                return n;
            };
            int nLines = countType(HitType::Line);
            int nPoints = countType(HitType::Point);
            int nCircles = countType(HitType::Circle);
            int nArcs = countType(HitType::Arc);

            struct CBtn { const char* label; ConstraintType type; bool valid; };
            CBtn cbtns[] = {
                {"Perp",    ConstraintType::Perpendicular, nLines == 2 && sel.size() == 2},
                {"Para",    ConstraintType::Parallel,      nLines == 2 && sel.size() == 2},
                {"Equal",   ConstraintType::EqualLength,   nLines == 2 && sel.size() == 2},
                {"Tang",    ConstraintType::Tangent,        nLines == 1 && (nCircles + nArcs) == 1 && sel.size() == 2},
                {"OnLine",  ConstraintType::PointOnLine,   nPoints == 1 && nLines == 1 && sel.size() == 2},
                {"Mid",     ConstraintType::Midpoint,      nPoints == 1 && nLines == 1 && sel.size() == 2},
                {"Sym",     ConstraintType::Symmetric,     nPoints == 2 && nLines == 1 && sel.size() == 3},
                {"Conc",    ConstraintType::Concentric,    (nCircles + nArcs) == 2 && sel.size() == 2},
            };
            Sketch& sk = activeSketch();
            for (auto& cb : cbtns) {
                if (!cb.valid) {
                    ImGui::PushStyleVar(ImGuiStyleVar_Alpha, 0.4f);
                    ImGui::Button(cb.label);
                    ImGui::PopStyleVar();
                } else if (ImGui::Button(cb.label)) {
                    applyGeometricConstraint(sk, cb.type);
                }
                ImGui::SameLine();
            }
        }

        ImGui::TextDisabled("|");
        ImGui::SameLine();

        char label[64];
        snprintf(label, sizeof(label), "Sketching: %s", sketchPlanes_[activeSketchPlane_].name.c_str());
        ImGui::TextUnformatted(label);

        ImGui::SameLine();
        ImGui::TextDisabled("|");
        ImGui::SameLine();
        {
            bool& ortho = viewport3D_.camera().orthographic;
            if (ImGui::Button(ortho ? "[O]rtho" : "[O] Persp")) ortho = !ortho;
        }
        ImGui::SameLine();
        if (ImGui::Button("Export DXF")) exportDxfDialog();
        ImGui::SameLine();
        if (ImGui::Button("Prefs")) prefsOpen_ = !prefsOpen_;
    } else {
        // Navigate mode toolbar
        if (ImGui::Button("Save")) saveProjectDialog();
        ImGui::SameLine();
        if (ImGui::Button("Open")) openProjectDialog();
        ImGui::SameLine();
        if (ImGui::Button("Import")) ImGui::OpenPopup("ImportPopup");
        if (ImGui::BeginPopup("ImportPopup")) {
            if (ImGui::MenuItem("STEP (.step/.stp)")) importStepDialog();
            if (ImGui::MenuItem("IGES (.igs/.iges)")) importIgesDialog();
            if (ImGui::MenuItem("STL (.stl)"))        importStlDialog();
            ImGui::EndPopup();
        }
        ImGui::SameLine();
        if (ImGui::Button("Export")) ImGui::OpenPopup("ExportPopup");
        if (ImGui::BeginPopup("ExportPopup")) {
            if (ImGui::MenuItem("STEP (.step)")) exportStepDialog();
            if (ImGui::MenuItem("IGES (.igs)"))  exportIgesDialog();
            if (ImGui::MenuItem("STL (.stl)"))   exportStlDialog();
            if (ImGui::MenuItem("OBJ (.obj)"))   exportObjDialog();
            ImGui::EndPopup();
        }
        ImGui::SameLine();
        ImGui::TextDisabled("|");
        ImGui::SameLine();
        ImGui::TextUnformatted("Navigate");
        ImGui::SameLine();
        ImGui::TextDisabled("|");
        ImGui::SameLine();
        ImGui::TextDisabled("Click a plane or face to sketch");
        ImGui::SameLine();
        ImGui::TextDisabled("|");
        ImGui::SameLine();

        char bodyText[64];
        snprintf(bodyText, sizeof(bodyText), "Bodies: %zu", scene_.bodyCount());
        ImGui::TextUnformatted(bodyText);

        ImGui::SameLine();
        ImGui::TextDisabled("|");
        ImGui::SameLine();
        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.5f, 0.3f, 0.1f, 1.0f));
        if (ImGui::Button("[E]xtrude")) {
            enterExtrudeMode();
        }
        ImGui::PopStyleColor();

        ImGui::SameLine();
        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.5f, 0.3f, 0.1f, 1.0f));
        if (ImGui::Button("Re[v]olve")) {
            enterRevolveMode();
        }
        ImGui::PopStyleColor();

        ImGui::SameLine();
        ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.5f, 0.3f, 0.1f, 1.0f));
        if (ImGui::Button("Loft")) {
            enterLoftMode();
        }
        ImGui::PopStyleColor();

        ImGui::SameLine();
        ImGui::TextDisabled("|");
        ImGui::SameLine();
        {
            bool unionActive = (tool_.type == ToolType::BooleanUnion);
            if (unionActive)
                ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.3f, 0.5f, 0.8f, 1.0f));
            else
                ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.1f, 0.4f, 0.5f, 1.0f));
            if (ImGui::Button("Union")) {
                if (unionActive) cancelBoolean();
                else enterBooleanMode(BooleanOperation::Union);
            }
            ImGui::PopStyleColor();

            ImGui::SameLine();
            bool subActive = (tool_.type == ToolType::BooleanSubtract);
            if (subActive)
                ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.3f, 0.5f, 0.8f, 1.0f));
            else
                ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.1f, 0.4f, 0.5f, 1.0f));
            if (ImGui::Button("Subtract")) {
                if (subActive) cancelBoolean();
                else enterBooleanMode(BooleanOperation::Subtract);
            }
            ImGui::PopStyleColor();
        }

        ImGui::SameLine();
        ImGui::TextDisabled("|");
        ImGui::SameLine();
        {
            bool& ortho = viewport3D_.camera().orthographic;
            if (ImGui::Button(ortho ? "[O]rtho" : "[O] Persp")) ortho = !ortho;
        }
        ImGui::SameLine();
        if (ImGui::Button("Prefs")) prefsOpen_ = !prefsOpen_;
    }

    ImGui::PopStyleVar(2);
}

} // namespace shitcad
