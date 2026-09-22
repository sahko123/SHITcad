#include "App.h"
#include "ProfileDetector.h"
#include "Extrude.h"
#include "AutoConstraint.h"
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

    ImGui::Separator();
    ImGui::Text("Profile Detection");
    const char* backendNames[] = { "Custom (half-edge tracer)", "OCCT (exact geometry)" };
    int backendIdx = (int)prefs_.profileBackend;
    if (ImGui::Combo("Backend", &backendIdx, backendNames, 2)) {
        prefs_.profileBackend = (ProfileDetectorBackend)backendIdx;
        setActiveProfileBackend(prefs_.profileBackend);
    }
    if (prefs_.profileBackend == ProfileDetectorBackend::OCCT) {
        ImGui::TextWrapped("Uses OCCT's BOPAlgo_BuilderFace for exact curve intersections. "
                           "Ellipses and splines produce smooth edges instead of polyline approximations.");
    }

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
        case UndoActionType::ModifyMeshImport:
            featureHistory_.updateMeshImportData(cmd.featureID, cmd.oldMeshImport);
            break;
        case UndoActionType::ModifySimulation:
            simulation_ = cmd.oldSimulation;
            break;
    }
    simUndoBase_ = simulation_;
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
        case UndoActionType::ModifyMeshImport:
            featureHistory_.updateMeshImportData(cmd.featureID, cmd.newMeshImport);
            break;
        case UndoActionType::ModifySimulation:
            simulation_ = cmd.newSimulation;
            break;
    }
    simUndoBase_ = simulation_;
    replayAllFeatures();
    markDirty();
}

void App::saveProjectDialog() {
    if (currentFilePath_.empty()) {
        std::string path = openNativeSaveDialog();
        if (path.empty()) return;
        currentFilePath_ = path;
    }
    if (!saveProject(currentFilePath_, featureHistory_, sketchPlanes_, &simulation_)) {
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
    SimulationSetup newSimulation;

    if (!loadProject(path, newHistory, newPlanes, &newSimulation)) {
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
    simulation_ = std::move(newSimulation);
    simUndoBase_ = simulation_;
    simUi_ = {};
    meshPlace_.reset();
    // Results belong to the project that produced them: without this, the
    // previous project's coverage table and coloured mesh stayed on screen
    // over the new project's geometry.
    clearSimulationRun();
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
    beginMeshImport(path);
}

void App::beginMeshImport(const std::string& path) {
    auto& d = meshImportDialog_;
    d.reset();
    d.path = path;

    // Default the feature name to the file stem: it becomes the surface name
    // when the model is handed to a simulation.
    auto slash = path.find_last_of("/\\");
    std::string base = (slash == std::string::npos) ? path : path.substr(slash + 1);
    auto dot = base.rfind('.');
    std::string stem = (dot == std::string::npos) ? base : base.substr(0, dot);
    snprintf(d.nameBuf, sizeof(d.nameBuf), "%s", stem.c_str());

    if (!probeMeshFile(path, d.info, d.error)) {
        fprintf(stderr, "Mesh import failed: %s\n", d.error.c_str());
    }
    d.open = true;
}

void App::drawMeshImportDialog() {
    auto& d = meshImportDialog_;
    if (!d.open) return;

    ImGui::SetNextWindowSize({380, 0}, ImGuiCond_Always);
    ImGui::SetNextWindowPos(ImGui::GetMainViewport()->GetCenter(), ImGuiCond_Appearing, {0.5f, 0.5f});
    bool keepOpen = true;
    ImGui::Begin("Import Mesh", &keepOpen,
        ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_AlwaysAutoResize);

    ImGui::TextWrapped("%s", d.path.c_str());
    ImGui::Separator();

    if (!d.error.empty()) {
        ImGui::TextColored({1.0f, 0.4f, 0.4f, 1.0f}, "%s", d.error.c_str());
        if (ImGui::Button("Close", {-1, 0})) keepOpen = false;
        ImGui::End();
        if (!keepOpen) d.reset();
        return;
    }

    ImGui::Text("%zu triangles", d.info.triangleCount);

    ImGui::Text("Name:");
    ImGui::SetNextItemWidth(-1);
    ImGui::InputText("##meshname", d.nameBuf, sizeof(d.nameBuf));

    // STL stores bare numbers. The unit is whatever the exporting program was
    // set to (Onshape asks at export time), so it has to be stated here.
    ImGui::Text("Unit of the numbers in this file:");
    ImGui::SetNextItemWidth(-1);
    ImGui::Combo("##meshunit", &d.unitIndex,
        [](void*, int i) { return kUnits[i].name; }, nullptr, kUnitCount);

    const float toMm = kUnits[d.unitIndex].toMm;
    float ext[3];
    float maxExt = 0.0f;
    for (int k = 0; k < 3; k++) {
        ext[k] = (d.info.rawMax[k] - d.info.rawMin[k]) * toMm;
        maxExt = std::max(maxExt, ext[k]);
    }
    if (maxExt >= 1000.0f)
        ImGui::Text("Size: %.4g x %.4g x %.4g m", ext[0] / 1000.0f, ext[1] / 1000.0f, ext[2] / 1000.0f);
    else
        ImGui::Text("Size: %.4g x %.4g x %.4g mm", ext[0], ext[1], ext[2]);

    // Same bounds cip-sim refuses to trace outside of: unit mix-ups are factors
    // of 1000 or 25.4, so a size check catches them where nothing else can.
    if (maxExt < 20.0f || maxExt > 100000.0f) {
        ImGui::PushTextWrapPos(ImGui::GetCursorPosX() + 360.0f);
        ImGui::TextColored({1.0f, 0.85f, 0.2f, 1.0f},
            "That is %.4g mm across. If the real part is not that size, the unit "
            "above is wrong.", maxExt);
        ImGui::PopTextWrapPos();
    }

    ImGui::Separator();
    if (ImGui::Button("Import", {-1, 0})) {
        MeshImportFeatureData md;
        md.sourcePath = d.path;
        md.unit = kUnits[d.unitIndex].name;
        std::string name = d.nameBuf[0] ? d.nameBuf : "Mesh";

        FeatureID fid = featureHistory_.addMeshImportFeature(md, name);
        UndoCommand cmd;
        cmd.type = UndoActionType::AddFeature;
        cmd.addedFeature = *featureHistory_.findFeature(fid);
        globalUndo_.push(std::move(cmd));
        markDirty();
        replayAllFeatures();
        keepOpen = false;
        // Straight into placement: an export is rarely the right way up.
        editMeshImportFeature(fid);
    }
    if (ImGui::Button("Cancel", {-1, 0})) keepOpen = false;

    ImGui::End();
    if (!keepOpen) d.reset();
}

static bool sameMeshImportData(const MeshImportFeatureData& a, const MeshImportFeatureData& b) {
    if (a.sourcePath != b.sourcePath || a.unit != b.unit) return false;
    for (int i = 0; i < 9; i++) if (a.transform.r[i] != b.transform.r[i]) return false;
    for (int i = 0; i < 3; i++) if (a.transform.t[i] != b.transform.t[i]) return false;
    return true;
}

void App::editMeshImportFeature(FeatureID id) {
    if (meshPlace_.active()) finishMeshPlace(true);
    const Feature* f = featureHistory_.findFeature(id);
    if (!f || f->type != FeatureType::MeshImport) return;
    meshPlace_.reset();
    meshPlace_.featureID = id;
    meshPlace_.original = std::get<MeshImportFeatureData>(f->data);
    for (int i = 0; i < 3; i++) meshPlace_.posBuf[i] = meshPlace_.original.transform.t[i];
}

void App::setMeshImportData(const MeshImportFeatureData& data) {
    // A nozzle is stored in its host's scaled frame. Changing the file's unit
    // rescales the mesh, so the nozzles must scale with it or they are left
    // stranded: a 2 m vessel corrected from cm to in became 5.08 m with its
    // nozzle still 1.98 m up - 3.1 m below the roof, and inside every guard
    // cip-sim has, so the run looked entirely normal.
    const Feature* before = featureHistory_.findFeature(meshPlace_.featureID);
    if (before && before->type == FeatureType::MeshImport) {
        const auto& old = std::get<MeshImportFeatureData>(before->data);
        const UnitInfo* from = findLengthUnit(old.unit);
        const UnitInfo* to = findLengthUnit(data.unit);
        if (from && to && from->toMm != to->toMm) {
            const double k = (double)to->toMm / (double)from->toMm;
            for (auto& n : simulation_.nozzles) {
                if (n.hostFeature != meshPlace_.featureID) continue;
                for (int i = 0; i < 3; i++) n.position[i] *= k;   // direction is unchanged
            }
            commitSimulationEdit();
        }
    }
    featureHistory_.updateMeshImportData(meshPlace_.featureID, data);
    for (int i = 0; i < 3; i++) meshPlace_.posBuf[i] = data.transform.t[i];
    replayAllFeatures();
}

void App::finishMeshPlace(bool keep) {
    if (!meshPlace_.active()) return;
    const Feature* f = featureHistory_.findFeature(meshPlace_.featureID);
    if (f && f->type == FeatureType::MeshImport) {
        const auto& current = std::get<MeshImportFeatureData>(f->data);
        if (!sameMeshImportData(current, meshPlace_.original)) {
            if (keep) {
                UndoCommand cmd;
                cmd.type = UndoActionType::ModifyMeshImport;
                cmd.featureID = meshPlace_.featureID;
                cmd.oldMeshImport = meshPlace_.original;
                cmd.newMeshImport = current;
                globalUndo_.push(std::move(cmd));
                markDirty();
            } else {
                featureHistory_.updateMeshImportData(meshPlace_.featureID, meshPlace_.original);
                replayAllFeatures();
            }
        }
    }
    meshPlace_.reset();
}

void App::drawMeshPlacePanel() {
    if (!meshPlace_.active()) return;
    const Feature* f = featureHistory_.findFeature(meshPlace_.featureID);
    if (!f || f->type != FeatureType::MeshImport) { // deleted or undone underneath us
        meshPlace_.reset();
        return;
    }
    MeshImportFeatureData data = std::get<MeshImportFeatureData>(f->data);

    ImGuiViewport* vp = ImGui::GetMainViewport();
    ImGui::SetNextWindowPos({vp->WorkPos.x + vp->WorkSize.x - 290, vp->WorkPos.y + 60}, ImGuiCond_Appearing);
    ImGui::SetNextWindowSize({280, 0}, ImGuiCond_Always);
    char title[160];
    snprintf(title, sizeof(title), "Place: %s###meshplace", f->name.c_str());
    bool open = true;
    ImGui::Begin(title, &open, ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_AlwaysAutoResize);

    MeshFileInfo info;
    std::string err;
    const UnitInfo* unit = findLengthUnit(data.unit);
    if (!probeMeshFile(data.sourcePath, info, err) || !unit) {
        ImGui::TextColored({1.0f, 0.4f, 0.4f, 1.0f}, "%s", unit ? err.c_str() : "Unknown unit");
        if (ImGui::Button("Close", {-1, 0})) open = false;
        ImGui::End();
        if (!open) finishMeshPlace(false);
        return;
    }

    // ---- unit (fixable here if the import dialog got it wrong)
    int unitIndex = (int)(unit - kUnits);
    ImGui::Text("File unit");
    ImGui::SameLine(90);
    ImGui::SetNextItemWidth(-1);
    if (ImGui::Combo("##placeunit", &unitIndex, [](void*, int i) { return kUnits[i].name; }, nullptr, kUnitCount)) {
        data.unit = kUnits[unitIndex].name;
        setMeshImportData(data);
    }

    // Exact bounds from the triangles actually drawn, when they are on hand.
    // placedBounds transforms the raw bounding box, which is exact only for
    // quarter turns: a 500 mm sphere turned 45 degrees twice reported its
    // bottom 353 mm too low, so "Drop to ground" left it floating.
    double lo[3], hi[3];
    bool exact = false;
    for (int i = 0; i < (int)scene_.bodyCount() && !exact; i++) {
        const Body3D& b = scene_.getBody(i);
        if (b.sourceFeature != meshPlace_.featureID || b.vertices.empty()) continue;
        for (int k = 0; k < 3; k++) { lo[k] = 1e300; hi[k] = -1e300; }
        for (const auto& v : b.vertices) {
            const double p[3] = {v.px, v.py, v.pz};
            for (int k = 0; k < 3; k++) { lo[k] = std::min(lo[k], p[k]); hi[k] = std::max(hi[k], p[k]); }
        }
        exact = true;
    }
    if (!exact) placedBounds(info, kUnits[unitIndex].toMm, data.transform, lo, hi);
    const double pivot[3] = {(lo[0] + hi[0]) * 0.5, (lo[1] + hi[1]) * 0.5, (lo[2] + hi[2]) * 0.5};
    const double big = std::max({hi[0] - lo[0], hi[1] - lo[1], hi[2] - lo[2]});
    const double s = big >= 1000.0 ? 0.001 : 1.0;
    const char* su = big >= 1000.0 ? "m" : "mm";
    ImGui::TextDisabled("Size  X %.4g  Y %.4g  Z %.4g %s", (hi[0] - lo[0]) * s, (hi[1] - lo[1]) * s, (hi[2] - lo[2]) * s, su);
    // The viewport is Y-up (ground grid in XZ), so "bottom" is along Y.
    ImGui::TextDisabled("Bottom at Y = %.4g %s", lo[1] * s, su);

    // ---- rotation: always about the mesh's own centre
    ImGui::Separator();
    ImGui::Text("Rotate 90 deg about its centre");
    auto quarterTurn = [&](int axis, double deg) {
        double q[9];
        axisRotation(axis, deg, q);
        rotateAbout(data.transform, q, pivot);
        setMeshImportData(data);
    };
    const char* axisNames[3] = {"X", "Y", "Z"};
    for (int a = 0; a < 3; a++) {
        ImGui::PushID(a);
        ImGui::AlignTextToFramePadding();
        ImGui::Text("%s", axisNames[a]);
        ImGui::SameLine(30);
        if (ImGui::Button("-90", {60, 0})) quarterTurn(a, -90.0);
        ImGui::SameLine();
        if (ImGui::Button("+90", {60, 0})) quarterTurn(a, 90.0);
        ImGui::SameLine();
        if (ImGui::Button("180", {60, 0})) quarterTurn(a, 180.0);
        ImGui::PopID();
    }
    // This viewport is Y-up. CAD packages including Onshape export Z-up, which
    // lands on its side here; Rx(-90) takes +Z to +Y.
    if (ImGui::Button("Z-up file (Onshape) -> stand upright", {-1, 0})) quarterTurn(0, -90.0);

    ImGui::SetNextItemWidth(70);
    ImGui::InputFloat("deg##angle", &meshPlace_.angleDeg, 0, 0, "%.2f");
    ImGui::SameLine();
    ImGui::SetNextItemWidth(50);
    ImGui::Combo("##angleaxis", &meshPlace_.angleAxis, axisNames, 3);
    ImGui::SameLine();
    if (ImGui::Button("Rotate", {-1, 0})) quarterTurn(meshPlace_.angleAxis, meshPlace_.angleDeg);

    // ---- position
    ImGui::Separator();
    ImGui::Text("Move (mm, applied after rotation)");
    bool posEdited = false;
    for (int a = 0; a < 3; a++) {
        ImGui::SetNextItemWidth(-1);
        char label[16];
        snprintf(label, sizeof(label), "##pos%d", a);
        ImGui::PushStyleColor(ImGuiCol_Text, a == 0 ? ImVec4(1.0f, 0.55f, 0.55f, 1.0f)
                                         : a == 1 ? ImVec4(0.55f, 1.0f, 0.55f, 1.0f)
                                                  : ImVec4(0.6f, 0.7f, 1.0f, 1.0f));
        char fmt[16];
        snprintf(fmt, sizeof(fmt), "%s %%.3f", axisNames[a]);
        ImGui::InputDouble(label, &meshPlace_.posBuf[a], 0, 0, fmt);
        ImGui::PopStyleColor();
        // Apply once editing finishes, not per keystroke: each apply replays
        // the whole model.
        if (ImGui::IsItemDeactivatedAfterEdit()) posEdited = true;
    }
    if (posEdited) {
        for (int a = 0; a < 3; a++) data.transform.t[a] = meshPlace_.posBuf[a];
        setMeshImportData(data);
    }
    if (ImGui::Button("Drop to ground (Y = 0)", {-1, 0})) {
        data.transform.t[1] -= lo[1];
        setMeshImportData(data);
    }
    if (ImGui::Button("Centre on origin (X, Z)", {-1, 0})) {
        data.transform.t[0] -= pivot[0];
        data.transform.t[2] -= pivot[2];
        setMeshImportData(data);
    }
    if (ImGui::Button("Reset placement", {-1, 0})) {
        data.transform = MeshTransform{};
        setMeshImportData(data);
    }

    ImGui::Separator();
    if (ImGui::Button("Done", {-1, 0})) {
        ImGui::End();
        finishMeshPlace(true);
        return;
    }
    if (ImGui::Button("Cancel", {-1, 0})) open = false;
    ImGui::End();
    if (!open) finishMeshPlace(false);
}

void App::updateMeshHover(float vpW, float vpH) {

    bool anyMesh = false;
    for (int i = 0; i < (int)scene_.bodyCount(); i++) {
        const auto& b = scene_.getBody(i);
        if (b.visible && b.isMeshOnly()) { anyMesh = true; break; }
    }
    if (!anyMesh || in_.uiWantsMouse || in_.mouseY < in_.viewY) {
        meshHover_ = {};
        return;
    }

    // Brute-force pick, so only redo it when the view or cursor could have
    // changed: large exports are hundreds of thousands of triangles.
    bool moved = in_.mouseX != meshHoverMouse_[0] || in_.mouseY != meshHoverMouse_[1];
    bool viewChanging = in_.wheel != 0.0f || in_.down[0] || in_.down[1] || in_.down[2];
    if (!moved && !viewChanging && meshHover_.bodyIndex < (int)scene_.bodyCount()) return;
    meshHoverMouse_[0] = in_.mouseX;
    meshHoverMouse_[1] = in_.mouseY;

    int w, h;
    glfwGetFramebufferSize(window_, &w, &h);
    float view[16], proj[16];
    getViewProj(w, h, view, proj);
    float rayOrig[3], rayDir[3];
    screenToRay(in_.mouseX, in_.mouseY, 0, 0, vpW, vpH, view, proj, rayOrig, rayDir);
    meshHover_ = pickMesh(scene_, rayOrig, rayDir, &section_);
}

void App::drawMeshHoverReadout() {
    if (!meshHover_.hit || meshHover_.bodyIndex >= (int)scene_.bodyCount()) return;

    ImGuiIO& io = ImGui::GetIO();
    const Body3D& body = scene_.getBody(meshHover_.bodyIndex);
    const Feature* feat = featureHistory_.findFeature(body.sourceFeature);

    int w, h;
    glfwGetFramebufferSize(window_, &w, &h);
    float view[16], proj[16];
    getViewProj(w, h, view, proj);

    // Normal as a short line from the hit point, so the file's winding - which
    // decides which way a nozzle placed here would point by default - is visible.
    ImDrawList* dl = ImGui::GetForegroundDrawList();
    const float* p = meshHover_.hitWorld;
    const float len = std::max(meshHover_.t * 0.08f, 1.0f);
    const float tip[3] = {p[0] + meshHover_.normal[0] * len,
                          p[1] + meshHover_.normal[1] * len,
                          p[2] + meshHover_.normal[2] * len};
    float sx0, sy0, sx1, sy1;
    if (worldToScreen(p, view, proj, io.DisplaySize.x, io.DisplaySize.y, sx0, sy0) &&
        worldToScreen(tip, view, proj, io.DisplaySize.x, io.DisplaySize.y, sx1, sy1)) {
        ImU32 col = meshHover_.frontFacing ? IM_COL32(80, 220, 120, 255) : IM_COL32(230, 120, 60, 255);
        dl->AddLine({sx0, sy0}, {sx1, sy1}, col, 2.0f);
        dl->AddCircleFilled({sx0, sy0}, 3.0f, col);
    }

    char text[256];
    snprintf(text, sizeof(text), "%s\n(%.2f, %.2f, %.2f) mm\nnormal (%.3f, %.3f, %.3f)  %s",
             feat ? feat->name.c_str() : "mesh",
             p[0], p[1], p[2],
             meshHover_.normal[0], meshHover_.normal[1], meshHover_.normal[2],
             meshHover_.frontFacing ? "front" : "back");
    ImVec2 pos(io.MousePos.x + 16.0f, io.MousePos.y + 16.0f);
    ImVec2 ts = ImGui::CalcTextSize(text);
    dl->AddRectFilled({pos.x - 4, pos.y - 2}, {pos.x + ts.x + 4, pos.y + ts.y + 2}, IM_COL32(0, 0, 0, 170), 4.0f);
    dl->AddText(pos, IM_COL32(220, 220, 220, 255), text);
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
        beginMeshImport(path); // unit is confirmed in the dialog
        return;
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
        } else if (feat.type == FeatureType::MeshImport) {
            col = ImVec4(0.45f, 0.55f, 0.55f, 1.0f);
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
            } else if (feat.type == FeatureType::MeshImport && !feat.suppressed && !rolledBack) {
                editMeshImportFeature(feat.id);
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

            if (feat.type == FeatureType::MeshImport && !feat.suppressed && !rolledBack) {
                if (ImGui::MenuItem("Rotate / Move...")) editMeshImportFeature(feat.id);
            }

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

            char label[160];
            const Body3D& body = scene_.getBody(i);
            const Feature* src = body.isMeshOnly() ? featureHistory_.findFeature(body.sourceFeature) : nullptr;
            if (src)
                snprintf(label, sizeof(label), "%s (mesh)", src->name.c_str());
            else
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

        const char* toolNames[] = {"[None]", "[P]oint", "[L]ine", "[C]ircle", "[R]ect", "[A]rc 3pt", "Arc Ctr", "Ctr Rect", "[D]im", "[F]illet"};
        ToolType toolTypes[] = {ToolType::None, ToolType::Point, ToolType::Line, ToolType::Circle, ToolType::Rectangle, ToolType::Arc3Point, ToolType::ArcCenter, ToolType::CenterRect, ToolType::Dimension, ToolType::Fillet};
        for (int i = 0; i < 10; i++) {
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
                {"Colin",   ConstraintType::Collinear,     nLines == 2 && sel.size() == 2},
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
        // Workspace tabs: Model (CAD) | Simulation
        {
            bool canSwitch = canSwitchWorkspace();
            auto tab = [&](const char* label, Workspace w) {
                bool on = workspace_ == w;
                ImGui::PushStyleColor(ImGuiCol_Button, on ? ImVec4(0.25f, 0.45f, 0.75f, 1.0f)
                                                         : ImVec4(0.35f, 0.35f, 0.38f, 1.0f));
                if (!canSwitch && !on) ImGui::BeginDisabled();
                if (ImGui::Button(label)) setWorkspace(w);
                if (!canSwitch && !on) ImGui::EndDisabled();
                ImGui::PopStyleColor();
            };
            tab("Model", Workspace::Model);
            ImGui::SameLine();
            tab("Simulation", Workspace::Simulation);
            ImGui::SameLine();
            ImGui::TextDisabled("|");
            ImGui::SameLine();
        }

        if (workspace_ == Workspace::Simulation) {
            if (ImGui::Button("Save")) saveProjectDialog();
            ImGui::SameLine();
            if (ImGui::Button("Open")) openProjectDialog();
            ImGui::SameLine();
            if (ImGui::Button("Import STL")) importStlDialog();
            ImGui::SameLine();
            ImGui::TextDisabled("|");
            ImGui::SameLine();
            ImGui::TextDisabled("Set up surfaces and nozzles in the Simulation panel");
            ImGui::SameLine();
            ImGui::TextDisabled("|");
            ImGui::SameLine();
            {
                bool& ortho = viewport3D_.camera().orthographic;
                if (ImGui::Button(ortho ? "[O]rtho" : "[O] Persp")) ortho = !ortho;
            }
            ImGui::SameLine();
            if (ImGui::Button("Prefs")) prefsOpen_ = !prefsOpen_;
            ImGui::PopStyleVar(2);
            return;
        }

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
        {
            bool on = sectionWindowOpen_ || section_.enabled;
            if (on) ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.25f, 0.45f, 0.75f, 1.0f));
            if (ImGui::Button("Section")) sectionWindowOpen_ = !sectionWindowOpen_;
            if (on) ImGui::PopStyleColor();
        }
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
