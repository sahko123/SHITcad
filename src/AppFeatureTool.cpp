// What the Extrude, Revolve and Loft tools have in common: finding the sketch to work from,
// saving it as a feature, drawing the translucent preview, and leaving the tool.
#include "App.h"
#include "ProfileDetector.h"
#include "Scene3D.h"
#include <glad/gl.h>
#include <algorithm>

namespace shitcad {

// Entering Extrude or Revolve from the toolbar: the sketch being edited if there is one,
// otherwise the plane whose sketch has the most closed profiles. False if there are none.
bool App::findProfilePlane(int& planeIdx, std::vector<ClosedProfile>& profiles) {
    planeIdx = -1;
    profiles.clear();

    if (activeSketchPlane_ >= 0) {
        planeIdx = activeSketchPlane_;
        profiles = detectClosedProfiles(activeSketch(), activePlane());
    } else {
        for (int i = 0; i < (int)sketchPlanes_.size(); i++) {
            auto& sp = sketchPlanes_[i];
            if (sp.sketch.points.empty() && sp.sketch.lines.empty() && sp.sketch.circles.empty())
                continue;
            auto p = detectClosedProfiles(sp.sketch, sp);
            if (!p.empty() && p.size() > profiles.size()) {
                planeIdx = i;
                profiles = std::move(p);
            }
        }
    }
    return planeIdx >= 0 && !profiles.empty();
}

// Editing an existing feature: the plane its source sketch lives on and that sketch's closed
// profiles. Finishes any sketch in progress first. False if the sketch or its profiles are gone.
bool App::loadSourceSketch(FeatureID srcSketch, int& planeIdx, std::vector<ClosedProfile>& profiles) {
    const Feature* src = featureHistory_.findFeature(srcSketch);
    if (!src || src->type != FeatureType::Sketch) return false;
    planeIdx = std::get<SketchFeatureData>(src->data).sketchPlaneIndex;
    if (planeIdx < 0 || planeIdx >= (int)sketchPlanes_.size()) return false;

    if (mode_ == InteractionMode::Sketching) {
        finishSketch();
    }

    profiles = detectClosedProfiles(sketchPlanes_[planeIdx].sketch, sketchPlanes_[planeIdx]);
    return !profiles.empty();
}

// Switch to a profile-based tool working on `profiles` from sketch plane `planeIdx`.
// The caller resets the tool's own state first and fills in whatever else it needs after.
void App::beginProfileTool(ToolType type, ProfileToolBase& t, int planeIdx,
                           std::vector<ClosedProfile> profiles) {
    tool_.type = type;
    tool_.reset();
    selection_.clear();

    t.sketchPlaneIndex = planeIdx;
    t.allProfiles = std::move(profiles);
    buildProfileRenderCaches(sketchPlanes_[planeIdx].sketch, t.allProfiles, t.renderCache);

    t.previewDirty = true;
    t.lastPreviewTime = std::chrono::steady_clock::now();
}

// The sketch feature a 3D feature is built from, created (with its undo step) if the plane has
// none yet. While sketching, the live sketch may hold edits the snapshot does not: record them.
FeatureID App::ensureSketchFeature(int planeIdx) {
    const SketchPlane& plane = sketchPlanes_[planeIdx];
    const Sketch& liveSketch = plane.sketch;

    FeatureID srcSketch = featureHistory_.findSketchFeatureForPlane(planeIdx);
    if (srcSketch == NullFeatureID) {
        srcSketch = featureHistory_.addSketchFeature(planeIdx, liveSketch, plane.planeID);
        UndoCommand skCmd;
        skCmd.type = UndoActionType::AddFeature;
        skCmd.addedFeature = *featureHistory_.findFeature(srcSketch);
        globalUndo_.push(std::move(skCmd)); markDirty();
    } else if (mode_ == InteractionMode::Sketching) {
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
    return srcSketch;
}

// Replace the tool's preview mesh with `shape` (null clears it), tinted red for a cut and
// blue for a new body.
void App::setToolPreview(FeatureToolBase& t, const TopoDS_Shape& shape) {
    t.previewBody = Body3D{}; // frees the old GL buffers
    if (!shape.IsNull()) {
        Scene3D::triangulateShape(shape, t.previewBody.vertices);
    }

    if (t.operation == ExtrudeOperation::Cut) {
        t.previewBody.colorR = 0.9f;
        t.previewBody.colorG = 0.3f;
        t.previewBody.colorB = 0.3f;
    } else {
        t.previewBody.colorR = 0.4f;
        t.previewBody.colorG = 0.6f;
        t.previewBody.colorB = 0.9f;
    }
    Scene3D::uploadMesh(t.previewBody);
}

void App::renderToolPreview(const FeatureToolBase& t, const float* view, const float* proj,
                            const float* eyePos) {
    const Body3D& body = t.previewBody;
    if (body.vao == 0 || body.vertexCount == 0) return;

    auto& shader = viewport3D_.meshShader();
    shader.use();
    shader.setMat4("uView", view);
    shader.setMat4("uProj", proj);
    shader.setVec3("uEyePos", eyePos[0], eyePos[1], eyePos[2]);
    shader.setVec3("uLightDir", 0.3f, 0.8f, 0.5f);
    applyClip(shader, nullptr); // tool previews are never sectioned

    float alpha = (t.operation == ExtrudeOperation::Cut) ? 0.4f : 0.5f;
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

// Leave the tool once its state is reset. A committed tool finishes the sketch it was started
// from (the feature was already recorded); a cancelled one replays so an edited feature that
// was suppressed for its preview comes back.
void App::finishFeatureTool(bool cancelled) {
    tool_.type = ToolType::None;
    tool_.reset();

    if (cancelled) {
        replayAllFeatures();
    } else if (mode_ == InteractionMode::Sketching) {
        finishSketch(false);
    }
}

} // namespace shitcad
