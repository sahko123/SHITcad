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

void App::enterRevolveMode() {
    int planeIdx = -1;
    std::vector<ClosedProfile> profiles;
    if (!findProfilePlane(planeIdx, profiles)) {
        fprintf(stderr, "No closed profiles found for revolve\n");
        return;
    }

    // Need at least one line in the sketch to use as axis
    if (sketchPlanes_[planeIdx].sketch.lines.empty()) {
        fprintf(stderr, "No lines available as revolve axis\n");
        return;
    }

    revolveTool_.reset();
    beginProfileTool(ToolType::Revolve, revolveTool_, planeIdx, std::move(profiles));

    if (revolveTool_.allProfiles.size() == 1) {
        revolveTool_.selectedProfileIndices.insert(0);
    }
}

void App::handleRevolveInput(float vpW, float vpH) {
    if (!hasRevolveSketch()) return;

    const SketchPlane& plane = revolvePlane();

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

App::RevolvePanelModel App::revolvePanelModel() const {
    RevolvePanelModel m;
    m.open = tool_.type == ToolType::Revolve && hasRevolveSketch();
    if (!m.open) return m;
    m.operation = (int)revolveTool_.operation;
    m.angleText = revolveTool_.angleBuf;
    m.axisLine = revolveTool_.axisLineID;
    m.selectingAxis = revolveTool_.phase == RevolvePhase::SelectingAxis;
    m.selectedProfiles = (int)revolveTool_.selectedProfileIndices.size();
    m.totalProfiles = (int)revolveTool_.allProfiles.size();
    m.canCommit = revolveTool_.hasSelectedProfiles() && revolveTool_.axisLineID != NullID;
    return m;
}

void App::setRevolveOperation(int op) {
    if (op == (int)ExtrudeOperation::Cut && scene_.empty()) return;   // nothing to cut
    revolveTool_.operation = (ExtrudeOperation)op;
    revolveTool_.previewDirty = true;
}

void App::setRevolveAngleText(const std::string& text) {
    snprintf(revolveTool_.angleBuf, sizeof(revolveTool_.angleBuf), "%s", text.c_str());
    float val = (float)atof(revolveTool_.angleBuf);
    if (std::fabs(val) > 0.01f) {
        revolveTool_.angleDeg = val;
        revolveTool_.previewDirty = true;
    } else {
        snprintf(revolveTool_.angleBuf, sizeof(revolveTool_.angleBuf), "%.1f", revolveTool_.angleDeg);
    }
}

void App::pickRevolveAxis(bool clearCurrent) {
    revolveTool_.phase = RevolvePhase::SelectingAxis;
    if (clearCurrent) {
        revolveTool_.axisLineID = NullID;
        revolveTool_.previewDirty = true;
    }
}

void App::pickRevolveProfiles() { revolveTool_.phase = RevolvePhase::SelectingProfiles; }

void App::updateRevolvePreview() {
    revolveTool_.previewDirty = false;

    revolveTool_.previewBody = Body3D{};

    if (!revolveTool_.hasSelectedProfiles() || revolveTool_.axisLineID == NullID) {
        return;
    }

    setToolPreview(revolveTool_, buildRevolveToolShape(revolveTool_, revolveSketch(), revolvePlane()));

    revolveTool_.lastPreviewTime = std::chrono::steady_clock::now();
}

void App::commitRevolve() {
    if (!revolveTool_.hasSelectedProfiles() || !hasRevolveSketch() || revolveTool_.axisLineID == NullID)
        return;

    RevolveFeatureData rd;
    rd.sourceSketchFeature = ensureSketchFeature(revolveTool_.sketchPlaneIndex);
    recordProfileSelection(revolveTool_.selectedProfileIndices, revolveTool_.allProfiles,
                           revolveSketch(), rd.profileSigs, rd.profileIndicesFallback);
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
    finishFeatureTool(false);
}

void App::cancelRevolve() {
    if (revolveTool_.editingFeatureID != NullFeatureID) {
        featureHistory_.unsuppressFeature(revolveTool_.editingFeatureID);
    }

    revolveTool_.reset();
    finishFeatureTool(true);
}

void App::editRevolveFeature(FeatureID id) {
    const Feature* feat = featureHistory_.findFeature(id);
    if (!feat || feat->type != FeatureType::Revolve) return;

    const auto& rd = std::get<RevolveFeatureData>(feat->data);

    int planeIdx = -1;
    std::vector<ClosedProfile> profiles;
    if (!loadSourceSketch(rd.sourceSketchFeature, planeIdx, profiles)) return;

    // Need at least one line for axis
    if (sketchPlanes_[planeIdx].sketch.lines.empty()) return;

    revolveTool_.reset();
    beginProfileTool(ToolType::Revolve, revolveTool_, planeIdx, std::move(profiles));
    revolveTool_.editingFeatureID = id;

    // Restore saved parameters
    revolveTool_.angleDeg = rd.angleDeg;
    revolveTool_.operation = rd.operation;
    revolveTool_.axisLineID = rd.axisLineID;
    snprintf(revolveTool_.angleBuf, sizeof(revolveTool_.angleBuf), "%.1f", rd.angleDeg);

    // Match saved profile signatures to current detected profiles
    auto matched = matchProfiles(rd.profileSigs, rd.profileIndicesFallback, revolveTool_.allProfiles,
                                 sketchPlanes_[planeIdx].sketch);
    revolveTool_.selectedProfileIndices = matched;

    // Skip past the phases whose input is already known
    if (!matched.empty() && rd.axisLineID != NullID) {
        revolveTool_.phase = RevolvePhase::Adjusting;
    } else if (!matched.empty()) {
        revolveTool_.phase = RevolvePhase::SelectingAxis;
    } else if (rd.axisLineID != NullID) {
        revolveTool_.phase = RevolvePhase::Adjusting;
    }

    // Suppress the feature being edited
    featureHistory_.suppressFeature(id);
    replayAllFeatures();
}

} // namespace shitcad
