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

    if (in_.keyPressed(Key::Escape)) {
        cancelLoft();
        return;
    }
    if (in_.keyPressed(Key::Enter) || in_.keyPressed(Key::KeypadEnter)) {
        if (loftTool_.canCommit()) {
            commitLoft();
            return;
        }
    }
}

App::LoftPanelModel App::loftPanelModel() const {
    LoftPanelModel m;
    m.open = tool_.type == ToolType::Loft;
    if (!m.open) return m;
    for (const auto& sec : loftTool_.sections)
        m.sections.push_back({sec.sketchPlaneIndex, sec.profileIndex, (int)sec.detectedProfiles.size()});
    // Sketch planes with closed profiles that are not a section yet
    for (int pi = 0; pi < (int)sketchPlanes_.size(); pi++) {
        const auto& sp = sketchPlanes_[pi];
        if (sp.sketch.points.empty() && sp.sketch.lines.empty() && sp.sketch.circles.empty())
            continue;
        bool alreadyUsed = false;
        for (const auto& sec : loftTool_.sections) {
            if (sec.sketchPlaneIndex == pi) { alreadyUsed = true; break; }
        }
        if (alreadyUsed) continue;
        auto profiles = detectClosedProfiles(sp.sketch, sp);
        if (profiles.empty()) continue;
        char label[64];
        snprintf(label, sizeof(label), "%s (%d profiles)", sp.name.c_str(), (int)profiles.size());
        m.candidates.push_back({pi, label});
    }
    m.solid = loftTool_.solid;
    m.canCommit = loftTool_.canCommit();
    return m;
}

void App::addLoftSection(int planeIndex) {
    if (planeIndex < 0 || planeIndex >= (int)sketchPlanes_.size()) return;
    for (const auto& sec : loftTool_.sections)
        if (sec.sketchPlaneIndex == planeIndex) return;
    auto& sp = sketchPlanes_[planeIndex];
    auto profiles = detectClosedProfiles(sp.sketch, sp);
    if (profiles.empty()) return;
    LoftToolSection sec;
    sec.sketchPlaneIndex = planeIndex;
    sec.detectedProfiles = std::move(profiles);
    // Auto-select first profile
    sec.profileIndex = 0;
    buildProfileRenderCaches(sp.sketch, sec.detectedProfiles, sec.renderCache);
    loftTool_.sections.push_back(std::move(sec));
    loftTool_.previewDirty = true;
}

void App::removeLoftSection(int section) {
    if (section < 0 || section >= (int)loftTool_.sections.size()) return;
    loftTool_.sections.erase(loftTool_.sections.begin() + section);
    loftTool_.previewDirty = true;
}

void App::setLoftSectionProfile(int section, int profile) {
    if (section < 0 || section >= (int)loftTool_.sections.size()) return;
    auto& sec = loftTool_.sections[section];
    if (profile < 0 || profile >= (int)sec.detectedProfiles.size()) return;
    sec.profileIndex = profile;
    loftTool_.previewDirty = true;
}

void App::setLoftSolid(bool solid) {
    loftTool_.solid = solid;
    loftTool_.previewDirty = true;
}

void App::updateLoftPreview() {
    loftTool_.previewDirty = false;

    loftTool_.previewBody = Body3D{};

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

    setToolPreview(loftTool_, loftProfiles(inputs, loftTool_.solid));
}

void App::commitLoft() {
    if (!loftTool_.canCommit()) return;

    LoftFeatureData ld;
    ld.operation = loftTool_.operation;
    ld.solid = loftTool_.solid;

    for (const auto& sec : loftTool_.sections) {
        LoftSection ls;
        ls.sourceSketchFeature = ensureSketchFeature(sec.sketchPlaneIndex);
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
    finishFeatureTool(false);
}

void App::cancelLoft() {
    if (loftTool_.editingFeatureID != NullFeatureID) {
        featureHistory_.unsuppressFeature(loftTool_.editingFeatureID);
    }

    loftTool_.reset();
    finishFeatureTool(true);
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
