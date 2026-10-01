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
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <cmath>
#include <algorithm>
#include <string>

namespace shitcad {

void App::setPreferences(const Preferences& p) {
    const bool themeChanged = p.lightMode != prefs_.lightMode;
    const bool backendChanged = p.profileBackend != prefs_.profileBackend;
    prefs_ = p;
    if (themeChanged) {
        // Resets the user-adjustable colours to the new theme's defaults.
        prefs_.applyTheme();
    }
    auto toU32 = [](const float c[4]) -> Color32 {
        return rgba32((int)(c[0]*255), (int)(c[1]*255), (int)(c[2]*255), (int)(c[3]*255));
    };
    auto& tm = activeThemeMut();
    tm.dimLineColor = toU32(prefs_.dimLineCol);
    tm.dimTextColor = toU32(prefs_.dimTextCol);
    tm.dimBgColor = toU32(prefs_.dimBgCol);
    if (backendChanged) setActiveProfileBackend(prefs_.profileBackend);
}

void App::replayAllFeatures() {
    shitcad::replayFeatures(featureHistory_, sketchPlanes_, scene_);
    // Replay is where a re-exported import is re-read; the Place panel's
    // bounds are keyed on the placement only, so they must be dropped here.
    cadBoundsCache_.valid = false;
}

// Copy a UTF-8 name into a fixed buffer without cutting a character in half:
// a long accented file name truncated at 127 bytes left a stray lead byte.
static void copyUtf8(char* buf, size_t size, const std::string& s) {
    size_t n = std::min(s.size(), size - 1);
    while (n > 0 && n < s.size() && ((unsigned char)s[n] & 0xC0) == 0x80) n--; // s[n] continues a character
    std::memcpy(buf, s.data(), n);
    buf[n] = '\0';
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
        case UndoActionType::ModifyCadImport:
            featureHistory_.updateCadImportData(cmd.featureID, cmd.oldCadImport);
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
        case UndoActionType::ModifyCadImport:
            featureHistory_.updateCadImportData(cmd.featureID, cmd.newCadImport);
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
        std::string path = chooseFile(FileDialog::SaveProject);
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
    std::string path = chooseFile(FileDialog::OpenProject);
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
    cadImportDialog_.reset();
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
    std::string path = chooseFile(FileDialog::SaveStl);
    if (path.empty()) return;
    if (!exportSTL(path, scene_)) {
        fprintf(stderr, "STL export failed: %s\n", lastLoadError().c_str());
    }
}

void App::importStlDialog() {
    std::string path = chooseFile(FileDialog::OpenStl);
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
    copyUtf8(d.nameBuf, sizeof(d.nameBuf), stem);

    if (!probeMeshFile(path, d.info, d.error)) {
        fprintf(stderr, "Mesh import failed: %s\n", d.error.c_str());
    }
    d.open = true;
}

// ---- Mesh import dialog model ---------------------------------------------------

App::MeshImportModel App::meshImportModel() const {
    const auto& d = meshImportDialog_;
    MeshImportModel m;
    m.open = d.open;
    if (!d.open) return m;
    m.path = d.path;
    m.error = d.error;
    m.name = d.nameBuf;
    m.triangles = d.info.triangleCount;
    m.unitIndex = d.unitIndex;
    const float toMm = kUnits[d.unitIndex].toMm;
    for (int k = 0; k < 3; k++) {
        m.extMm[k] = (d.info.rawMax[k] - d.info.rawMin[k]) * toMm;
        m.maxExtMm = std::max(m.maxExtMm, m.extMm[k]);
    }
    // Same bounds cip-sim refuses to trace outside of: unit mix-ups are factors
    // of 1000 or 25.4, so a size check catches them where nothing else can.
    m.sizeSuspicious = m.maxExtMm < 20.0f || m.maxExtMm > 100000.0f;
    if (!canImport()) m.blocked = "Finish the sketch or the active tool first.";
    return m;
}

void App::setMeshImportName(const std::string& name) {
    copyUtf8(meshImportDialog_.nameBuf, sizeof(meshImportDialog_.nameBuf), name);
}

void App::setMeshImportUnit(int unitIndex) {
    if (unitIndex >= 0 && unitIndex < kUnitCount) meshImportDialog_.unitIndex = unitIndex;
}

void App::confirmMeshImport() {
    auto& d = meshImportDialog_;
    if (!d.open || !d.error.empty() || !canImport()) return;
    MeshImportFeatureData md;
    md.sourcePath = d.path;
    md.unit = kUnits[d.unitIndex].name;
    std::string name = d.nameBuf[0] ? d.nameBuf : "Mesh";

    rollForwardForNewFeature();
    FeatureID fid = featureHistory_.addMeshImportFeature(md, name);
    UndoCommand cmd;
    cmd.type = UndoActionType::AddFeature;
    cmd.addedFeature = *featureHistory_.findFeature(fid);
    globalUndo_.push(std::move(cmd));
    markDirty();
    replayAllFeatures();
    // Straight into placement: an export is rarely the right way up.
    editImportPlacement(fid);
    MeshTransform xf;
    double lo[3], hi[3];
    if (placeTransform(xf, lo, hi, nullptr)) frameBounds(lo, hi);
    d.reset();
}

// ---- STEP / IGES import ---------------------------------------------------------

void App::beginCadImport(const std::string& path) {
    auto& d = cadImportDialog_;
    d.reset();
    d.path = path;
    auto slash = path.find_last_of("/\\");
    std::string base = (slash == std::string::npos) ? path : path.substr(slash + 1);
    auto dot = base.rfind('.');
    std::string stem = (dot == std::string::npos) ? base : base.substr(0, dot);
    copyUtf8(d.nameBuf, sizeof(d.nameBuf), stem);

    // Read now (it is cached for the replay that follows), so the dialog can
    // say what the file holds - or why it could not be read.
    std::vector<CadPart> parts;
    if (!loadCadFile(path, parts, d.info, d.error)) {
        fprintf(stderr, "Import failed: %s\n", d.error.c_str());
    }
    d.open = true;
}

App::CadImportModel App::cadImportModel() const {
    const auto& d = cadImportDialog_;
    CadImportModel m;
    m.open = d.open;
    if (!d.open) return m;
    m.path = d.path;
    m.error = d.error;
    m.name = d.nameBuf;
    m.format = d.info.format;
    m.fileUnit = d.info.fileUnit;
    m.solids = d.info.solids;
    m.surfaces = d.info.surfaces;
    m.skippedWires = d.info.skippedWires;
    for (int k = 0; k < 3; k++) m.extMm[k] = d.info.hi[k] - d.info.lo[k];
    m.zUp = d.zUp;
    if (!canImport()) m.blocked = "Finish the sketch or the active tool first.";
    return m;
}

void App::setCadImportName(const std::string& name) {
    copyUtf8(cadImportDialog_.nameBuf, sizeof(cadImportDialog_.nameBuf), name);
}

void App::setCadImportZUp(bool zUp) { cadImportDialog_.zUp = zUp; }

void App::cancelCadImport() { cadImportDialog_.reset(); }

void App::confirmCadImport() {
    auto& d = cadImportDialog_;
    if (!d.open || !d.error.empty() || !canImport()) return;
    CadImportFeatureData cd;
    cd.sourcePath = d.path;
    // This viewport is Y-up; most CAD (Onshape, Fusion, Inventor) is Z-up, and
    // a Z-up file lies on its side here. Rx(-90) takes +Z to +Y. About the
    // origin, not the part's centre, so parts from separate files that share
    // a frame still line up.
    if (d.zUp) axisRotation(0, -90.0, cd.transform.r);
    std::string name = d.nameBuf[0] ? d.nameBuf : "Import";

    rollForwardForNewFeature();
    FeatureID fid = featureHistory_.addCadImportFeature(cd, name);
    UndoCommand cmd;
    cmd.type = UndoActionType::AddFeature;
    cmd.addedFeature = *featureHistory_.findFeature(fid);
    globalUndo_.push(std::move(cmd));
    markDirty();
    replayAllFeatures();
    d.reset();

    // The part is wherever the file put it, often nowhere near the origin the
    // camera is looking at: show it, then open placement.
    editImportPlacement(fid);
    MeshTransform xf;
    double lo[3], hi[3];
    if (placeTransform(xf, lo, hi, nullptr)) frameBounds(lo, hi);
}

void App::rollForwardForNewFeature() {
    const int pos = featureHistory_.rollbackPos();
    if (pos < 0) return;
    UndoCommand cmd;
    cmd.type = UndoActionType::SetRollbackPos;
    cmd.oldRollbackPos = pos;
    cmd.newRollbackPos = -1;
    featureHistory_.setRollbackPos(-1);
    globalUndo_.push(std::move(cmd));
}

void App::importFile(const std::string& path) {
    if (!canImport()) {
        fprintf(stderr, "Import refused: finish the sketch or the active tool first (%s)\n", path.c_str());
        return;
    }
    std::string ext;
    auto dot = path.rfind('.');
    if (dot != std::string::npos) {
        ext = path.substr(dot);
        for (auto& ch : ext) ch = (char)tolower((unsigned char)ch);
    }
    if (isCadFile(path)) {
        beginCadImport(path);
    } else if (ext == ".stl") {
        beginMeshImport(path); // unit is confirmed in the dialog
    } else {
        fprintf(stderr, "Unsupported import format: %s\n", ext.c_str());
    }
}

void App::replaceImportFile(FeatureID id) {
    const Feature* f = featureHistory_.findFeature(id);
    if (!f || (f->type != FeatureType::MeshImport && f->type != FeatureType::CadImport)) return;
    const bool mesh = f->type == FeatureType::MeshImport;
    std::string path = chooseFile(mesh ? FileDialog::OpenStl : FileDialog::OpenCad);
    if (path.empty()) return;
    f = featureHistory_.findFeature(id); // the dialog ran a nested event loop
    if (!f) return;
    if (!mesh && !isCadFile(path)) {
        fprintf(stderr, "Not a STEP or IGES file: %s\n", path.c_str());
        return;
    }
    // A placement edit in progress becomes its own undo step first.
    if (meshPlace_.featureID == id) finishMeshPlace(true);
    f = featureHistory_.findFeature(id);

    UndoCommand cmd;
    cmd.featureID = id;
    if (mesh) {
        cmd.type = UndoActionType::ModifyMeshImport;
        cmd.oldMeshImport = std::get<MeshImportFeatureData>(f->data);
        cmd.newMeshImport = cmd.oldMeshImport;
        cmd.newMeshImport.sourcePath = path;
        featureHistory_.updateMeshImportData(id, cmd.newMeshImport);
    } else {
        cmd.type = UndoActionType::ModifyCadImport;
        cmd.oldCadImport = std::get<CadImportFeatureData>(f->data);
        cmd.newCadImport = cmd.oldCadImport;
        cmd.newCadImport.sourcePath = path;
        featureHistory_.updateCadImportData(id, cmd.newCadImport);
    }
    globalUndo_.push(std::move(cmd));
    markDirty();
    replayAllFeatures();
}

void App::cancelMeshImport() { meshImportDialog_.reset(); }


static bool sameMeshImportData(const MeshImportFeatureData& a, const MeshImportFeatureData& b) {
    if (a.sourcePath != b.sourcePath || a.unit != b.unit) return false;
    for (int i = 0; i < 9; i++) if (a.transform.r[i] != b.transform.r[i]) return false;
    for (int i = 0; i < 3; i++) if (a.transform.t[i] != b.transform.t[i]) return false;
    return true;
}

void App::editImportPlacement(FeatureID id) {
    if (meshPlace_.active()) finishMeshPlace(true);
    const Feature* f = featureHistory_.findFeature(id);
    if (!f || (f->type != FeatureType::MeshImport && f->type != FeatureType::CadImport)) return;
    meshPlace_.reset();
    meshPlace_.featureID = id;
    meshPlace_.type = f->type;
    const MeshTransform* xf;
    if (f->type == FeatureType::MeshImport) {
        meshPlace_.original = std::get<MeshImportFeatureData>(f->data);
        xf = &meshPlace_.original.transform;
    } else {
        meshPlace_.originalCad = std::get<CadImportFeatureData>(f->data);
        xf = &meshPlace_.originalCad.transform;
    }
    for (int i = 0; i < 3; i++) meshPlace_.posBuf[i] = xf->t[i];
}

static bool sameTransform(const MeshTransform& a, const MeshTransform& b) {
    for (int i = 0; i < 9; i++) if (a.r[i] != b.r[i]) return false;
    for (int i = 0; i < 3; i++) if (a.t[i] != b.t[i]) return false;
    return true;
}

bool App::placeTransform(MeshTransform& xf, double lo[3], double hi[3], std::string* error) const {
    const Feature* f = featureHistory_.findFeature(meshPlace_.featureID);
    if (!f || f->type != meshPlace_.type) return false;
    if (f->type == FeatureType::MeshImport) {
        MeshImportFeatureData data;
        if (!meshPlaceBounds(data, lo, hi, error)) return false;
        xf = data.transform;
        return true;
    }
    const auto& cd = std::get<CadImportFeatureData>(f->data);
    xf = cd.transform;
    auto& c = cadBoundsCache_;
    if (!c.valid || c.path != cd.sourcePath || !sameTransform(c.xf, cd.transform)) {
        std::vector<CadPart> parts;
        CadFileInfo info;
        std::string err;
        if (!loadCadFile(cd.sourcePath, parts, info, err) || !cadPlacedBounds(parts, cd.transform, c.lo, c.hi)) {
            c.valid = false;
            if (error) *error = err.empty() ? "The file has no geometry" : err;
            return false;
        }
        c.path = cd.sourcePath;
        c.xf = cd.transform;
        c.valid = true;
    }
    for (int k = 0; k < 3; k++) { lo[k] = c.lo[k]; hi[k] = c.hi[k]; }
    return true;
}

void App::setPlaceTransform(const MeshTransform& xf) {
    const Feature* f = featureHistory_.findFeature(meshPlace_.featureID);
    if (!f || f->type != meshPlace_.type) return;
    if (f->type == FeatureType::MeshImport) {
        MeshImportFeatureData data = std::get<MeshImportFeatureData>(f->data);
        data.transform = xf;
        setMeshImportData(data);
        return;
    }
    CadImportFeatureData data = std::get<CadImportFeatureData>(f->data);
    data.transform = xf;
    featureHistory_.updateCadImportData(meshPlace_.featureID, data);
    for (int i = 0; i < 3; i++) meshPlace_.posBuf[i] = xf.t[i];
    replayAllFeatures();
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
    if (f && f->type == FeatureType::CadImport && meshPlace_.type == FeatureType::CadImport) {
        const auto& current = std::get<CadImportFeatureData>(f->data);
        const auto& original = meshPlace_.originalCad;
        if (current.sourcePath != original.sourcePath || !sameTransform(current.transform, original.transform)) {
            if (keep) {
                UndoCommand cmd;
                cmd.type = UndoActionType::ModifyCadImport;
                cmd.featureID = meshPlace_.featureID;
                cmd.oldCadImport = original;
                cmd.newCadImport = current;
                globalUndo_.push(std::move(cmd));
                markDirty();
            } else {
                featureHistory_.updateCadImportData(meshPlace_.featureID, original);
                replayAllFeatures();
            }
        }
    } else if (f && f->type == FeatureType::MeshImport && meshPlace_.type == FeatureType::MeshImport) {
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

// ---- Mesh placement model -------------------------------------------------------

void App::validateMeshPlace() {
    if (!meshPlace_.active()) return;
    const Feature* f = featureHistory_.findFeature(meshPlace_.featureID);
    if (!f || f->type != meshPlace_.type) meshPlace_.reset(); // deleted or undone underneath us
}

bool App::meshPlaceBounds(MeshImportFeatureData& data, double lo[3], double hi[3], std::string* error) const {
    const Feature* f = featureHistory_.findFeature(meshPlace_.featureID);
    if (!f || f->type != FeatureType::MeshImport) return false;
    data = std::get<MeshImportFeatureData>(f->data);

    MeshFileInfo info;
    std::string err;
    const UnitInfo* unit = findLengthUnit(data.unit);
    if (!probeMeshFile(data.sourcePath, info, err) || !unit) {
        if (error) *error = unit ? err : "Unknown unit";
        return false;
    }

    // Exact bounds from the triangles actually drawn, when they are on hand.
    // placedBounds transforms the raw bounding box, which is exact only for
    // quarter turns: a 500 mm sphere turned 45 degrees twice reported its
    // bottom 353 mm too low, so "Drop to ground" left it floating.
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
    if (!exact) placedBounds(info, unit->toMm, data.transform, lo, hi);
    return true;
}

App::MeshPlaceModel App::meshPlaceModel() const {
    MeshPlaceModel m;
    if (!meshPlace_.active()) return m;
    const Feature* f = featureHistory_.findFeature(meshPlace_.featureID);
    if (!f || f->type != meshPlace_.type) return m;
    m.active = true;
    m.name = f->name;
    m.hasUnit = f->type == FeatureType::MeshImport;
    MeshTransform xf;
    if (!placeTransform(xf, m.lo, m.hi, &m.error)) {
        if (m.error.empty()) m.error = "File not found";
        return m;
    }
    if (m.hasUnit)
        m.unitIndex = (int)(findLengthUnit(std::get<MeshImportFeatureData>(f->data).unit) - kUnits);
    for (int k = 0; k < 3; k++) m.pos[k] = meshPlace_.posBuf[k];
    m.angleDeg = meshPlace_.angleDeg;
    m.angleAxis = meshPlace_.angleAxis;
    return m;
}

void App::meshPlaceSetUnit(int unitIndex) {
    MeshImportFeatureData data;
    double lo[3], hi[3];
    if (meshPlace_.type != FeatureType::MeshImport) return; // STEP/IGES declare their unit
    if (unitIndex < 0 || unitIndex >= kUnitCount || !meshPlaceBounds(data, lo, hi, nullptr)) return;
    data.unit = kUnits[unitIndex].name;
    setMeshImportData(data);
}

void App::meshPlaceRotate(int axis, double degrees) {
    MeshTransform xf;
    double lo[3], hi[3];
    if (axis < 0 || axis > 2 || !placeTransform(xf, lo, hi, nullptr)) return;
    const double pivot[3] = {(lo[0] + hi[0]) * 0.5, (lo[1] + hi[1]) * 0.5, (lo[2] + hi[2]) * 0.5};
    double q[9];
    axisRotation(axis, degrees, q);
    rotateAbout(xf, q, pivot);
    setPlaceTransform(xf);
}

void App::meshPlaceSetPosition(const double pos[3]) {
    MeshTransform xf;
    double lo[3], hi[3];
    if (!placeTransform(xf, lo, hi, nullptr)) return;
    for (int a = 0; a < 3; a++) xf.t[a] = pos[a];
    setPlaceTransform(xf);
}

void App::meshPlaceDropToGround() {
    MeshTransform xf;
    double lo[3], hi[3];
    if (!placeTransform(xf, lo, hi, nullptr)) return;
    xf.t[1] -= lo[1];   // the viewport is Y-up, so "bottom" is along Y
    setPlaceTransform(xf);
}

void App::meshPlaceCentreOnOrigin() {
    MeshTransform xf;
    double lo[3], hi[3];
    if (!placeTransform(xf, lo, hi, nullptr)) return;
    xf.t[0] -= (lo[0] + hi[0]) * 0.5;
    xf.t[2] -= (lo[2] + hi[2]) * 0.5;
    setPlaceTransform(xf);
}

void App::meshPlaceResetPlacement() {
    MeshTransform xf;
    double lo[3], hi[3];
    if (!placeTransform(xf, lo, hi, nullptr)) return;
    setPlaceTransform(MeshTransform{});
}

// ---- Camera framing ---------------------------------------------------------------

void App::frameBounds(const double lo[3], const double hi[3]) {
    const OrbitCamera& cam = viewport3D_.camera();
    OrbitCamera to = cameraAnimating_ ? cameraTo_ : cam;
    to.targetX = (float)((lo[0] + hi[0]) * 0.5);
    to.targetY = (float)((lo[1] + hi[1]) * 0.5);
    to.targetZ = (float)((lo[2] + hi[2]) * 0.5);
    const double dx = hi[0] - lo[0], dy = hi[1] - lo[1], dz = hi[2] - lo[2];
    const float radius = std::max(1.0f, (float)(0.5 * std::sqrt(dx * dx + dy * dy + dz * dz)));
    // The bounding sphere fits the 45 degree field of view (perspective) or
    // the half-height (orthographic), with a margin; a narrow window fits width.
    int w, h;
    framebufferSize(w, h);
    const float aspect = (w > 0 && h > 0) ? std::min(1.0f, (float)w / (float)h) : 1.0f;
    const float halfFov = 22.5f * kDegToRad;
    to.distance = cam.orthographic ? radius * 1.15f / aspect
                                   : radius * 1.15f / (std::sin(halfFov) * aspect);
    to.orthographic = cam.orthographic;
    to.autoOrtho = cam.autoOrtho;
    cameraFrom_ = cam;
    cameraTo_ = to;
    cameraAnimating_ = true;
    cameraAnimT_ = 0.0f;
}

void App::frameScene() {
    if (scene_.empty()) return;
    float l[3], h[3];
    sceneBounds(l, h);
    const double lo[3] = {l[0], l[1], l[2]}, hi[3] = {h[0], h[1], h[2]};
    frameBounds(lo, hi);
}

void App::updateMeshHover(float vpW, float vpH) {

    bool anyMesh = false;
    for (int i = 0; i < (int)scene_.bodyCount(); i++) {
        const auto& b = scene_.getBody(i);
        if (b.visible && b.isMeshOnly()) { anyMesh = true; break; }
    }
    if (!anyMesh || in_.mouseY < in_.viewY) {
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
    framebufferSize(w, h);
    float view[16], proj[16];
    getViewProj(w, h, view, proj);
    float rayOrig[3], rayDir[3];
    screenToRay(in_.mouseX, in_.mouseY, 0, 0, vpW, vpH, view, proj, rayOrig, rayDir);
    meshHover_ = pickMesh(scene_, rayOrig, rayDir, &section_);
}

void App::drawMeshHoverReadout() {
    if (!meshHover_.hit || meshHover_.bodyIndex >= (int)scene_.bodyCount()) return;

    const Body3D& body = scene_.getBody(meshHover_.bodyIndex);
    const Feature* feat = featureHistory_.findFeature(body.sourceFeature);

    int w, h;
    framebufferSize(w, h);
    float view[16], proj[16];
    getViewProj(w, h, view, proj);

    // Normal as a short line from the hit point, so the file's winding - which
    // decides which way a nozzle placed here would point by default - is visible.
    Overlay2D& ov = overlay_;
    const float* p = meshHover_.hitWorld;
    const float len = std::max(meshHover_.t * 0.08f, 1.0f);
    const float tip[3] = {p[0] + meshHover_.normal[0] * len,
                          p[1] + meshHover_.normal[1] * len,
                          p[2] + meshHover_.normal[2] * len};
    float sx0, sy0, sx1, sy1;
    if (worldToScreen(p, view, proj, in_.screenW, in_.screenH, sx0, sy0) &&
        worldToScreen(tip, view, proj, in_.screenW, in_.screenH, sx1, sy1)) {
        Color32 col = meshHover_.frontFacing ? rgba32(80, 220, 120, 255) : rgba32(230, 120, 60, 255);
        ov.addLine({sx0, sy0}, {sx1, sy1}, col, 2.0f);
        ov.addCircleFilled({sx0, sy0}, 3.0f, col);
    }

    char text[256];
    snprintf(text, sizeof(text), "%s\n(%.2f, %.2f, %.2f) mm\nnormal (%.3f, %.3f, %.3f)  %s",
             feat ? feat->name.c_str() : "mesh",
             p[0], p[1], p[2],
             meshHover_.normal[0], meshHover_.normal[1], meshHover_.normal[2],
             meshHover_.frontFacing ? "front" : "back");
    OvVec2 pos(in_.mouseX + 16.0f, in_.mouseY + 16.0f);
    OvVec2 ts = ov.textSize(text);
    ov.addRectFilled({pos.x - 4, pos.y - 2}, {pos.x + ts.x + 4, pos.y + ts.y + 2}, rgba32(0, 0, 0, 170), 4.0f);
    ov.addText(pos, rgba32(220, 220, 220, 255), text);
}

void App::exportStepDialog() {
    if (scene_.empty()) return;
    std::string path = chooseFile(FileDialog::SaveStep);
    if (path.empty()) return;
    if (!exportSTEP(path, scene_)) {
        fprintf(stderr, "STEP export failed: %s\n", lastLoadError().c_str());
    }
}

void App::importStepDialog() {
    std::string path = chooseFile(FileDialog::OpenStep);
    if (path.empty()) return;
    beginCadImport(path);
}

void App::exportIgesDialog() {
    if (scene_.empty()) return;
    std::string path = chooseFile(FileDialog::SaveIges);
    if (path.empty()) return;
    if (!exportIGES(path, scene_)) {
        fprintf(stderr, "IGES export failed: %s\n", lastLoadError().c_str());
    }
}

void App::importIgesDialog() {
    std::string path = chooseFile(FileDialog::OpenIges);
    if (path.empty()) return;
    beginCadImport(path);
}

void App::exportObjDialog() {
    if (scene_.empty()) return;
    std::string path = chooseFile(FileDialog::SaveObj);
    if (path.empty()) return;
    if (!exportOBJ(path, scene_)) {
        fprintf(stderr, "OBJ export failed: %s\n", lastLoadError().c_str());
    }
}

void App::exportDxfDialog() {
    if (!hasActiveSketch()) return;
    std::string path = chooseFile(FileDialog::SaveDxf);
    if (path.empty()) return;
    if (!exportDXF(path, activeSketch())) {
        fprintf(stderr, "DXF export failed: %s\n", lastLoadError().c_str());
    }
}

void App::importModelDialog() {
    std::string path = chooseFile(FileDialog::OpenImport);
    if (path.empty()) return;
    importFile(path);
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
    if (host_) host_->setWindowTitle(title);
}

void App::markDirty() {
    if (!unsavedChanges_) {
        unsavedChanges_ = true;
        updateWindowTitle();
    }
}

// ---- Timeline -------------------------------------------------------------------

// Button colour by feature type and state.
static void featureColor(const Feature& feat, bool grayed, float rgb[3]) {
    auto set = [rgb](float r, float g, float b) { rgb[0] = r; rgb[1] = g; rgb[2] = b; };
    set(0.0f, 0.0f, 0.0f);
    if (grayed) {
        set(0.4f, 0.4f, 0.4f);
    } else if (feat.hasError) {
        set(0.8f, 0.2f, 0.2f);
    } else if (feat.type == FeatureType::Sketch) {
        set(0.3f, 0.5f, 0.9f);
    } else if (feat.type == FeatureType::Extrude) {
        const auto& ed = std::get<ExtrudeFeatureData>(feat.data);
        if (ed.operation == ExtrudeOperation::Cut) set(0.9f, 0.4f, 0.2f);
        else set(0.9f, 0.6f, 0.2f);
    } else if (feat.type == FeatureType::Revolve) {
        const auto& rd = std::get<RevolveFeatureData>(feat.data);
        if (rd.operation == ExtrudeOperation::Cut) set(0.9f, 0.3f, 0.5f);
        else set(0.7f, 0.4f, 0.9f);
    } else if (feat.type == FeatureType::Loft) {
        set(0.3f, 0.8f, 0.5f);
    } else if (feat.type == FeatureType::Boolean) {
        const auto& bd = std::get<BooleanFeatureData>(feat.data);
        if (bd.operation == BooleanOperation::Subtract) set(0.9f, 0.3f, 0.3f);
        else set(0.2f, 0.7f, 0.9f);
    } else if (feat.type == FeatureType::MeshImport) {
        set(0.45f, 0.55f, 0.55f);
    } else if (feat.type == FeatureType::CadImport) {
        set(0.55f, 0.55f, 0.45f);
    }
}

App::TimelineModel App::timelineModel() const {
    TimelineModel m;
    m.visible = !featureHistory_.empty();
    if (!m.visible) return m;
    const auto& features = featureHistory_.features();
    for (int i = 0; i < (int)features.size(); i++) {
        const Feature& feat = features[i];
        const bool rolledBack = featureHistory_.isRolledBack(i);
        TimelineModel::Item item;
        item.id = feat.id;
        item.name = feat.name;
        item.grayed = feat.suppressed || rolledBack;
        featureColor(feat, item.grayed, item.rgb);
        item.error = feat.hasError;
        item.errorMsg = feat.errorMsg;
        item.selected = feat.id == selectedFeatureID_;
        item.suppressed = feat.suppressed;
        const bool isImport = feat.type == FeatureType::MeshImport || feat.type == FeatureType::CadImport;
        item.canPlace = isImport && !feat.suppressed && !rolledBack;
        item.canReplaceFile = isImport;
        m.items.push_back(std::move(item));
    }
    const int rollbackPos = featureHistory_.rollbackPos();
    m.playhead = rollbackPos >= 0 ? rollbackPos : (int)features.size() - 1;
    m.dragging = playheadDragging_;
    return m;
}

void App::editFeature(FeatureID id) {
    const Feature* found = featureHistory_.findFeature(id);
    if (!found || found->suppressed) return;
    if (featureHistory_.isRolledBack(featureHistory_.featureIndex(id))) return;
    const Feature feat = *found;   // editing may change the history
    if (feat.type == FeatureType::Sketch) {
        const auto& sd = std::get<SketchFeatureData>(feat.data);
        if (mode_ != InteractionMode::Sketching) {
            enterSketchMode(sd.sketchPlaneIndex);
            selectedFeatureID_ = NullFeatureID; // deselect so Delete targets sketch geometry
        }
    } else if (feat.type == FeatureType::Extrude) {
        editExtrudeFeature(feat.id);
        selectedFeatureID_ = NullFeatureID;
    } else if (feat.type == FeatureType::Revolve) {
        editRevolveFeature(feat.id);
        selectedFeatureID_ = NullFeatureID;
    } else if (feat.type == FeatureType::Loft) {
        editLoftFeature(feat.id);
        selectedFeatureID_ = NullFeatureID;
    } else if (feat.type == FeatureType::MeshImport || feat.type == FeatureType::CadImport) {
        editImportPlacement(feat.id);
    }
}

void App::renameFeature(FeatureID id, const std::string& name) {
    const Feature* feat = featureHistory_.findFeature(id);
    if (!feat) return;
    UndoCommand cmd;
    cmd.type = UndoActionType::RenameFeature;
    cmd.featureID = id;
    cmd.oldName = feat->name;
    cmd.newName = name;
    globalUndo_.push(std::move(cmd)); markDirty();
    featureHistory_.renameFeature(id, name);
}

void App::setFeatureSuppressed(FeatureID id, bool suppressed) {
    const Feature* feat = featureHistory_.findFeature(id);
    if (!feat || feat->suppressed == suppressed) return;
    UndoCommand cmd;
    cmd.type = suppressed ? UndoActionType::SuppressFeature : UndoActionType::UnsuppressFeature;
    cmd.featureID = id;
    globalUndo_.push(std::move(cmd)); markDirty();
    if (suppressed) featureHistory_.suppressFeature(id);
    else featureHistory_.unsuppressFeature(id);
    replayAllFeatures();
}

void App::deleteFeature(FeatureID id) {
    const Feature* feat = featureHistory_.findFeature(id);
    if (!feat) return;
    // Collect the feature and its dependents with their indices before deleting
    UndoCommand cmd;
    cmd.type = UndoActionType::DeleteFeature;
    cmd.featureID = id;
    cmd.deletedFeatures.push_back({featureHistory_.featureIndex(id), *feat});
    for (auto depID : featureHistory_.getDependents(id)) {
        const Feature* depFeat = featureHistory_.findFeature(depID);
        if (depFeat) cmd.deletedFeatures.push_back({featureHistory_.featureIndex(depID), *depFeat});
    }
    // Sort by index ascending for proper re-insertion on undo
    std::sort(cmd.deletedFeatures.begin(), cmd.deletedFeatures.end(),
        [](const auto& a, const auto& b) { return a.first < b.first; });
    globalUndo_.push(std::move(cmd)); markDirty();
    featureHistory_.deleteFeature(id);
    replayAllFeatures();
}

bool App::deleteSelectedFeature() {
    if (selectedFeatureID_ == NullFeatureID || !featureHistory_.findFeature(selectedFeatureID_)) return false;
    deleteFeature(selectedFeatureID_);
    selectedFeatureID_ = NullFeatureID;
    return true;
}

void App::beginPlayheadDrag() {
    dragStartRollbackPos_ = featureHistory_.rollbackPos();
    playheadDragging_ = true;
}

void App::movePlayhead(int pos) {
    if (pos != featureHistory_.rollbackPos()) {
        featureHistory_.setRollbackPos(pos);
        // Defer replay until mouse stops moving (150ms idle)
        playheadReplayPending_ = true;
        playheadLastMoveTime_ = std::chrono::steady_clock::now();
    }
}

void App::tickPlayhead() {
    if (!playheadReplayPending_) return;
    auto elapsed = std::chrono::steady_clock::now() - playheadLastMoveTime_;
    if (elapsed >= std::chrono::milliseconds(150)) {
        replayAllFeatures();
        playheadReplayPending_ = false;
    }
}

void App::endPlayheadDrag() {
    if (!playheadDragging_) return;
    playheadDragging_ = false;
    // Replay on drag release if still pending
    if (playheadReplayPending_) {
        replayAllFeatures();
        playheadReplayPending_ = false;
    }
    // One undo step for the whole drag
    const int rollbackPos = featureHistory_.rollbackPos();
    if (dragStartRollbackPos_ != rollbackPos) {
        UndoCommand cmd;
        cmd.type = UndoActionType::SetRollbackPos;
        cmd.oldRollbackPos = dragStartRollbackPos_;
        cmd.newRollbackPos = rollbackPos;
        globalUndo_.push(std::move(cmd)); markDirty();
    }
}

static bool planeHasSketch(const SketchPlane& sp) {
    return !sp.sketch.points.empty() || !sp.sketch.lines.empty() || !sp.sketch.circles.empty();
}

App::ObjectTreeModel App::objectTreeModel() const {
    ObjectTreeModel m;
    m.open = objectTreeOpen_;
    auto planeKey = [&](int i) { return ((uint64_t)sketchPlanes_[i].planeID << 32) | (uint32_t)i; };
    for (int i = 0; i < (int)sketchPlanes_.size(); i++) {
        const SketchPlane& sp = sketchPlanes_[i];
        if (sp.isReferencePlane) m.planes.push_back({planeKey(i), i, sp.name, sp.visible});
    }
    for (int i = 0; i < (int)sketchPlanes_.size(); i++) {
        const SketchPlane& sp = sketchPlanes_[i];
        if (!planeHasSketch(sp)) continue;
        char label[64];
        if (i < kRefPlaneCount)
            snprintf(label, sizeof(label), "Sketch on %s", sp.name.c_str());
        else
            snprintf(label, sizeof(label), "%s", sp.name.c_str());
        m.sketches.push_back({planeKey(i), i, label, sp.sketchVisible});
    }
    for (int i = 0; i < (int)scene_.bodyCount(); i++) {
        const Body3D& body = scene_.getBody(i);
        // Every body has a source feature now; only imports are named after it.
        const Feature* src = body.sourceFeature ? featureHistory_.findFeature(body.sourceFeature) : nullptr;
        if (src && src->type != FeatureType::MeshImport && src->type != FeatureType::CadImport) src = nullptr;
        char label[160];
        if (src && body.isMeshOnly())
            snprintf(label, sizeof(label), "%s (mesh)", src->name.c_str());
        else if (src && !body.name.empty() && body.name != src->name)
            snprintf(label, sizeof(label), "%s: %s", src->name.c_str(), body.name.c_str());
        else if (src)
            snprintf(label, sizeof(label), "%s", src->name.c_str());
        else
            snprintf(label, sizeof(label), "Body %d", i + 1);
        m.bodies.push_back({(uint64_t)i, i, label, body.visible});
    }
    return m;
}

void App::setPlaneVisible(int planeIndex, bool visible) {
    if (planeIndex >= 0 && planeIndex < (int)sketchPlanes_.size()) sketchPlanes_[planeIndex].visible = visible;
}

void App::setSketchVisible(int planeIndex, bool visible) {
    if (planeIndex >= 0 && planeIndex < (int)sketchPlanes_.size()) sketchPlanes_[planeIndex].sketchVisible = visible;
}

void App::setBodyVisible(int bodyIndex, bool visible) {
    if (bodyIndex >= 0 && bodyIndex < (int)scene_.bodyCount()) scene_.getBodyMut(bodyIndex).visible = visible;
}

void App::editPlaneSketch(int planeIndex) {
    if (planeIndex >= 0 && planeIndex < (int)sketchPlanes_.size()) enterSketchMode(planeIndex);
}

// ---- Toolbar model ------------------------------------------------------------

const App::ToolbarConstraint App::kToolbarConstraints[App::kToolbarConstraintCount] = {
    {"Perp",   ConstraintType::Perpendicular},
    {"Para",   ConstraintType::Parallel},
    {"Colin",  ConstraintType::Collinear},
    {"Equal",  ConstraintType::EqualLength},
    {"Tang",   ConstraintType::Tangent},
    {"OnLine", ConstraintType::PointOnLine},
    {"Mid",    ConstraintType::Midpoint},
    {"Sym",    ConstraintType::Symmetric},
    {"Conc",   ConstraintType::Concentric},
};

const ToolType App::kSketchTools[App::kSketchToolCount] = {
    ToolType::None, ToolType::Point, ToolType::Line, ToolType::Circle, ToolType::Rectangle,
    ToolType::Arc3Point, ToolType::ArcCenter, ToolType::CenterRect, ToolType::Dimension, ToolType::Fillet};
const char* const App::kSketchToolLabels[App::kSketchToolCount] = {
    "[None]", "[P]oint", "[L]ine", "[C]ircle", "[R]ect", "[A]rc 3pt", "Arc Ctr", "Ctr Rect", "[D]im", "[F]illet"};

bool ToolbarModel::operator==(const ToolbarModel& o) const {
    if (variant != o.variant || tool != o.tool || workspace != o.workspace ||
        canSwitchWorkspace != o.canSwitchWorkspace || extrudeActive != o.extrudeActive ||
        revolveActive != o.revolveActive || loftActive != o.loftActive ||
        unionActive != o.unionActive || subtractActive != o.subtractActive ||
        ortho != o.ortho || sectionOn != o.sectionOn || bodyCount != o.bodyCount ||
        sketchPlaneName != o.sketchPlaneName)
        return false;
    for (int i = 0; i < App::kToolbarConstraintCount; i++)
        if (constraintValid[i] != o.constraintValid[i]) return false;
    return true;
}

ToolbarModel App::toolbarModel() const {
    ToolbarModel m;
    if (mode_ == InteractionMode::Sketching && activeSketchPlane_ >= 0)
        m.variant = ToolbarModel::Variant::Sketch;
    else
        m.variant = workspace_ == Workspace::Simulation ? ToolbarModel::Variant::Simulation
                                                        : ToolbarModel::Variant::Model;
    m.tool = tool_.type;
    m.workspace = workspace_;
    m.canSwitchWorkspace = canSwitchWorkspace();
    m.extrudeActive = tool_.type == ToolType::Extrude;
    m.revolveActive = tool_.type == ToolType::Revolve;
    m.loftActive = tool_.type == ToolType::Loft;
    m.unionActive = tool_.type == ToolType::BooleanUnion;
    m.subtractActive = tool_.type == ToolType::BooleanSubtract;
    m.ortho = viewport3D_.camera().orthographic;
    m.sectionOn = sectionWindowOpen_ || section_.enabled;
    m.bodyCount = scene_.bodyCount();
    if (m.variant == ToolbarModel::Variant::Sketch) {
        m.sketchPlaneName = sketchPlanes_[activeSketchPlane_].name;

        const auto& sel = selection_.selected;
        auto countType = [&](HitType t) {
            int n = 0;
            for (auto& s : sel) if (s.type == t) n++;
            return n;
        };
        int nLines = countType(HitType::Line);
        int nPoints = countType(HitType::Point);
        int nCircles = countType(HitType::Circle);
        int nArcs = countType(HitType::Arc);
        const bool valid[kToolbarConstraintCount] = {
            nLines == 2 && sel.size() == 2,                              // Perp
            nLines == 2 && sel.size() == 2,                              // Para
            nLines == 2 && sel.size() == 2,                              // Colin
            nLines == 2 && sel.size() == 2,                              // Equal
            nLines == 1 && (nCircles + nArcs) == 1 && sel.size() == 2,   // Tang
            nPoints == 1 && nLines == 1 && sel.size() == 2,              // OnLine
            nPoints == 1 && nLines == 1 && sel.size() == 2,              // Mid
            nPoints == 2 && nLines == 1 && sel.size() == 3,              // Sym
            (nCircles + nArcs) == 2 && sel.size() == 2,                  // Conc
        };
        for (int i = 0; i < kToolbarConstraintCount; i++) m.constraintValid[i] = valid[i];
    }
    return m;
}

void App::perform(UiAction action, int arg) {
    const bool sketchBar = mode_ == InteractionMode::Sketching && activeSketchPlane_ >= 0;
    switch (action) {
        case UiAction::FinishSketch: if (sketchBar) finishSketch(); break;
        case UiAction::SelectTool: if (sketchBar) switchTool((ToolType)arg); break;
        // On the sketch toolbar these toggle; on the model toolbar they only enter.
        case UiAction::Extrude:
            if (sketchBar && tool_.type == ToolType::Extrude) cancelExtrude(); else enterExtrudeMode();
            break;
        case UiAction::Revolve:
            if (sketchBar && tool_.type == ToolType::Revolve) cancelRevolve(); else enterRevolveMode();
            break;
        case UiAction::Loft:
            if (sketchBar && tool_.type == ToolType::Loft) cancelLoft(); else enterLoftMode();
            break;
        case UiAction::SnapView: if (sketchBar) orientCameraToPlane(activePlane()); break;
        case UiAction::ApplyConstraint:
            if (sketchBar && arg >= 0 && arg < kToolbarConstraintCount && toolbarModel().constraintValid[arg])
                applyGeometricConstraint(activeSketch(), kToolbarConstraints[arg].type);
            break;
        case UiAction::ExportDxf: exportDxfDialog(); break;
        case UiAction::SetWorkspace:
            if (canSwitchWorkspace() || workspace_ == (Workspace)arg) setWorkspace((Workspace)arg);
            break;
        case UiAction::Save: saveProjectDialog(); break;
        case UiAction::Open: openProjectDialog(); break;
        case UiAction::ImportStep: importStepDialog(); break;
        case UiAction::ImportIges: importIgesDialog(); break;
        case UiAction::ImportStl: importStlDialog(); break;
        case UiAction::ExportStep: exportStepDialog(); break;
        case UiAction::ExportIges: exportIgesDialog(); break;
        case UiAction::ExportStl: exportStlDialog(); break;
        case UiAction::ExportObj: exportObjDialog(); break;
        case UiAction::ToggleSection: sectionWindowOpen_ = !sectionWindowOpen_; break;
        case UiAction::Union:
            if (tool_.type == ToolType::BooleanUnion) cancelBoolean(); else enterBooleanMode(BooleanOperation::Union);
            break;
        case UiAction::Subtract:
            if (tool_.type == ToolType::BooleanSubtract) cancelBoolean(); else enterBooleanMode(BooleanOperation::Subtract);
            break;
        case UiAction::ToggleOrtho: setOrthographic(!viewport3D_.camera().orthographic); break;
        case UiAction::TogglePrefs: prefsOpen_ = !prefsOpen_; break;
    }
}

// ---- Add Reference Plane --------------------------------------------------------

App::AddPlaneModel App::addPlaneModel() const {
    AddPlaneModel m;
    m.open = addPlaneDialogOpen_;
    if (!m.open) return m;
    m.fromFace = addPlaneFromFace_;
    m.waitingFace = addPlaneWaitingFace_;
    m.sourceIndex = addPlaneSourceIndex_;
    m.offsetText = addPlaneOffsetBuf_;
    m.name = addPlaneNameBuf_;
    for (int i = 0; i < (int)sketchPlanes_.size(); i++)
        if (sketchPlanes_[i].isReferencePlane) m.sources.push_back({i, sketchPlanes_[i].name});
    m.canCreate = !addPlaneFromFace_ || !addPlaneWaitingFace_;
    return m;
}

void App::openAddPlaneDialog() {
    addPlaneDialogOpen_ = true;
    addPlaneSourceIndex_ = 0;
    addPlaneOffset_ = 10.0f;
    snprintf(addPlaneOffsetBuf_, sizeof(addPlaneOffsetBuf_), "%.1f", addPlaneOffset_);
    snprintf(addPlaneNameBuf_, sizeof(addPlaneNameBuf_), "");
    addPlaneFromFace_ = false;
    addPlaneWaitingFace_ = false;
}

void App::setAddPlaneSource(bool fromFace, int planeIndex) {
    addPlaneFromFace_ = fromFace;
    addPlaneWaitingFace_ = fromFace;   // a face still has to be clicked
    if (!fromFace && planeIndex >= 0 && planeIndex < (int)sketchPlanes_.size())
        addPlaneSourceIndex_ = planeIndex;
}

void App::setAddPlaneOffsetText(const std::string& text) {
    snprintf(addPlaneOffsetBuf_, sizeof(addPlaneOffsetBuf_), "%s", text.c_str());
    addPlaneOffset_ = (float)atof(addPlaneOffsetBuf_);
}

void App::setAddPlaneName(const std::string& name) {
    snprintf(addPlaneNameBuf_, sizeof(addPlaneNameBuf_), "%s", name.c_str());
}

void App::createOffsetPlane() {
    if (!addPlaneDialogOpen_ || (addPlaneFromFace_ && addPlaneWaitingFace_)) return;
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

void App::cancelAddPlane() {
    addPlaneDialogOpen_ = false;
    addPlaneWaitingFace_ = false;
}

// ---- Tangent plane on a cylinder ------------------------------------------------

App::TangentPlaneModel App::tangentPlaneModel() const {
    TangentPlaneModel m;
    m.open = cylPlaneDialogOpen_;
    if (!m.open) return m;
    m.angleDeg = cylPlaneAngle_;
    m.name = cylPlaneNameBuf_;
    return m;
}

void App::setTangentPlaneAngle(float degrees) {
    cylPlaneAngle_ = degrees;
}

void App::setTangentPlaneName(const std::string& name) {
    snprintf(cylPlaneNameBuf_, sizeof(cylPlaneNameBuf_), "%s", name.c_str());
}

void App::createTangentPlane() {
    if (!cylPlaneDialogOpen_) return;
    SketchPlane newPlane;
    if (!buildCylinderTangentPlane(cylPlaneFace_, cylPlaneAngle_, cylPlaneHitWorld_, newPlane)) return;
    newPlane.sourceBodyIndex = cylPlaneBodyIndex_;
    newPlane.name = strlen(cylPlaneNameBuf_) > 0 ? cylPlaneNameBuf_ : "CylPlane";
    newPlane.isReferencePlane = true;
    newPlane.color[0] = 0.2f; newPlane.color[1] = 0.7f;
    newPlane.color[2] = 0.5f; newPlane.color[3] = 0.15f;
    projectFaceOntoSketch(cylPlaneFace_, newPlane, newPlane.sketch);
    newPlane.planeID = nextPlaneID_++;
    sketchPlanes_.push_back(std::move(newPlane));
    cylPlaneDialogOpen_ = false;
    enterSketchMode((int)sketchPlanes_.size() - 1);
}

void App::cancelTangentPlane() { cylPlaneDialogOpen_ = false; }

} // namespace shitcad
