#include "App.h"

#include <imgui.h>

#include <algorithm>
#include <cmath>
#include <cstring>

namespace shitcad {

// Bounds of everything drawable, in mm: bodies plus loaded results. Cached on a
// cheap key so a 500k-triangle import is not walked every frame.
void App::sceneBounds(float lo[3], float hi[3]) {
    // The key has to change when a body MOVES, not only when one is added or
    // re-tessellated: replay reloads the same file and re-applies the transform,
    // so vertexCount alone is bit-identical after a rotate, and the section
    // slider kept the pre-move range.
    size_t key = scene_.bodyCount() * 1315423911u + (simView_.loaded ? simView_.mesh.triangles : 0);
    for (int i = 0; i < (int)scene_.bodyCount(); i++) {
        const Body3D& b = scene_.getBody(i);
        key = key * 31 + (size_t)b.vertexCount;
        if (!b.vertices.empty()) {
            const MeshVertex& v = b.vertices.front();
            const MeshVertex& w = b.vertices.back();
            for (float f : {v.px, v.py, v.pz, w.px, w.py, w.pz}) {
                uint32_t bits;
                std::memcpy(&bits, &f, sizeof(bits));
                key = key * 1099511628211u + bits;
            }
        }
    }

    if (key != sceneBoundsKey_) {
        sceneBoundsKey_ = key;
        float l[3] = {1e30f, 1e30f, 1e30f}, h[3] = {-1e30f, -1e30f, -1e30f};
        for (int i = 0; i < (int)scene_.bodyCount(); i++) {
            for (const auto& v : scene_.getBody(i).vertices) {
                const float p[3] = {v.px, v.py, v.pz};
                for (int k = 0; k < 3; k++) { l[k] = std::min(l[k], p[k]); h[k] = std::max(h[k], p[k]); }
            }
        }
        if (simView_.loaded) {
            for (int k = 0; k < 3; k++) {
                l[k] = std::min(l[k], simView_.mesh.boundsMin[k]);
                h[k] = std::max(h[k], simView_.mesh.boundsMax[k]);
            }
        }
        if (l[0] > h[0]) { // nothing in the scene yet
            for (int k = 0; k < 3; k++) { l[k] = -100.0f; h[k] = 100.0f; }
        }
        for (int k = 0; k < 3; k++) { sceneLo_[k] = l[k]; sceneHi_[k] = h[k]; }
    }
    for (int k = 0; k < 3; k++) { lo[k] = sceneLo_[k]; hi[k] = sceneHi_[k]; }
}

void App::drawSectionControls() {
    float lo[3], hi[3];
    sceneBounds(lo, hi);

    const bool was = section_.enabled;
    ImGui::Checkbox("Section view", &section_.enabled);
    if (section_.enabled && !was)
        section_.position = (lo[section_.axis] + hi[section_.axis]) * 0.5f;
    if (!section_.enabled) {
        ImGui::TextDisabled("Cut the model open to see inside.");
        return;
    }

    const int before = section_.axis;
    ImGui::AlignTextToFramePadding();
    ImGui::TextUnformatted("Cut along");
    ImGui::SameLine();
    ImGui::RadioButton("X", &section_.axis, 0); ImGui::SameLine();
    ImGui::RadioButton("Y", &section_.axis, 1); ImGui::SameLine();
    ImGui::RadioButton("Z", &section_.axis, 2); ImGui::SameLine();
    ImGui::Checkbox("Flip", &section_.flip);
    if (before != section_.axis)
        section_.position = (lo[section_.axis] + hi[section_.axis]) * 0.5f;

    // Clamped to the model, so the slider cannot be dragged somewhere that
    // shows nothing at all.
    section_.position = std::clamp(section_.position, lo[section_.axis], hi[section_.axis]);
    ImGui::SetNextItemWidth(-60);
    ImGui::SliderFloat("Position", &section_.position, lo[section_.axis], hi[section_.axis], "%.0f mm");
    if (ImGui::Button("Centre")) section_.position = (lo[section_.axis] + hi[section_.axis]) * 0.5f;
    ImGui::SameLine();
    ImGui::Checkbox("Cap the cut", &section_.cap);
    ImGui::TextDisabled("Cuts geometry and results. Spray cones stay whole.");

    if (section_.cap) {
        int open = 0, closed = 0;
        for (int i = 0; i < (int)scene_.bodyCount(); i++) {
            if (!scene_.getBody(i).visible) continue;
            (scene_.getBody(i).closed ? closed : open)++;
        }
        if (open > 0)
            ImGui::TextDisabled("%d open surface%s cannot be capped and are left hollow.",
                                open, open == 1 ? "" : "s");
        if (closed == 0 && open > 0)
            ImGui::TextDisabled("(A cap needs a watertight surface.)");
    }
}

// Fill the opening left by the cut so the model reads as solid.
//
// Stencil capping: with depth testing off, draw the (already clipped) closed
// meshes counting front faces up and back faces down. Anywhere the count is
// non-zero the camera is looking into the inside of a solid, which is exactly
// where the cut face belongs - so a quad on the plane is drawn through that
// stencil.
void App::renderSectionCap(const float* view, const float* proj, bool resultsShown) {
    if (!section_.enabled || !section_.cap) return;

    // Only CLOSED surfaces can be capped. The front/back face counts of an open
    // surface never cancel, so the stencil ends up non-zero across its whole
    // projected area and the cap quad paints a slab over the very thing being
    // looked at - including the coverage colours. The panel recommends one file
    // per surface, so open surfaces are the normal case, not an edge case.
    const bool haveResults = resultsShown && simView_.vao && simView_.vertexCount &&
                             simView_.closed && resultShader_.id() != 0;
    std::vector<int> cappable;
    for (int i = 0; i < (int)scene_.bodyCount(); i++) {
        const Body3D& b = scene_.getBody(i);
        if (!b.visible || !b.closed || b.vao == 0 || b.vertexCount == 0) continue;
        if (resultsShown && b.isMeshOnly() && b.sourceFeature &&
            std::find(simView_.coveredFeatures.begin(), simView_.coveredFeatures.end(),
                      b.sourceFeature) != simView_.coveredFeatures.end())
            continue;   // drawn as results instead
        cappable.push_back(i);
    }
    if (cappable.empty() && !haveResults) return;

    float eye[3];
    viewport3D_.camera().getEyePosition(eye);

    const GLboolean cullWas = glIsEnabled(GL_CULL_FACE);
    glEnable(GL_STENCIL_TEST);
    glClear(GL_STENCIL_BUFFER_BIT);
    glColorMask(GL_FALSE, GL_FALSE, GL_FALSE, GL_FALSE);
    glDepthMask(GL_FALSE);
    glDisable(GL_DEPTH_TEST);
    glDisable(GL_CULL_FACE);
    glStencilFunc(GL_ALWAYS, 0, 0xFF);
    glStencilOpSeparate(GL_FRONT, GL_KEEP, GL_KEEP, GL_INCR_WRAP);
    glStencilOpSeparate(GL_BACK, GL_KEEP, GL_KEEP, GL_DECR_WRAP);

    if (!cappable.empty()) {
        auto& mesh = viewport3D_.meshShader();
        mesh.use();
        mesh.setMat4("uView", view);
        mesh.setMat4("uProj", proj);
        mesh.setVec3("uEyePos", eye[0], eye[1], eye[2]);
        mesh.setVec3("uLightDir", 0.3f, 0.8f, 0.5f);
        mesh.setFloat("uAlpha", 1.0f);
        applyClip(mesh, &section_);
        for (int i : cappable) {
            const Body3D& b = scene_.getBody(i);
            glBindVertexArray(b.vao);
            glDrawArrays(GL_TRIANGLES, 0, b.vertexCount);
        }
        glBindVertexArray(0);
    }
    if (haveResults) {
        resultShader_.use();
        resultShader_.setMat4("uView", view);
        resultShader_.setMat4("uProj", proj);
        applyClip(resultShader_, &section_);
        glBindVertexArray(simView_.vao);
        glDrawArrays(GL_TRIANGLES, 0, simView_.vertexCount);
        glBindVertexArray(0);
    }

    // The cap quad itself, on the plane, only where the stencil says "inside".
    float lo[3], hi[3];
    sceneBounds(lo, hi);
    float n[3];
    section_.normal(n);
    const float centre[3] = {(lo[0] + hi[0]) * 0.5f, (lo[1] + hi[1]) * 0.5f, (lo[2] + hi[2]) * 0.5f};
    const float diag = std::sqrt((hi[0] - lo[0]) * (hi[0] - lo[0]) + (hi[1] - lo[1]) * (hi[1] - lo[1]) +
                                 (hi[2] - lo[2]) * (hi[2] - lo[2]));
    const float half = std::max(diag, 1.0f) * 0.75f;

    // Nudge towards the camera so the quad wins against geometry lying exactly
    // on the plane. Measured against the PLANE, not the model centre: with the
    // eye inside the kept half the centre test picked the wrong sign and pushed
    // the cap away from the camera instead.
    const float eyeSide = eye[0] * n[0] + eye[1] * n[1] + eye[2] * n[2] - section_.offset();
    const float bias = std::max(diag * 1e-4f, 1e-3f) * (eyeSide >= 0.0f ? 1.0f : -1.0f);
    const float d = section_.offset() + bias;

    const int a0 = (section_.axis + 1) % 3, a1 = (section_.axis + 2) % 3;
    float u[3] = {0, 0, 0}, v[3] = {0, 0, 0};
    u[a0] = 1.0f;
    v[a1] = 1.0f;
    float origin[3];
    const float centreDot = centre[0] * n[0] + centre[1] * n[1] + centre[2] * n[2];
    for (int k = 0; k < 3; k++) origin[k] = centre[k] - n[k] * (centreDot - d);

    const auto& bc = activeTheme().bodyColor;
    const float capCol[3] = {bc[0] * 0.72f, bc[1] * 0.72f, bc[2] * 0.75f};
    struct CapVert { float x, y, z, r, g, b; };
    CapVert quad[6];
    const float su[4] = {-1, 1, 1, -1}, sv[4] = {-1, -1, 1, 1};
    const int order[6] = {0, 1, 2, 0, 2, 3};
    for (int i = 0; i < 6; i++) {
        const int c = order[i];
        quad[i] = {origin[0] + (u[0] * su[c] + v[0] * sv[c]) * half,
                   origin[1] + (u[1] * su[c] + v[1] * sv[c]) * half,
                   origin[2] + (u[2] * su[c] + v[2] * sv[c]) * half,
                   capCol[0], capCol[1], capCol[2]};
    }

    if (!capVAO_) {
        glGenVertexArrays(1, &capVAO_);
        glGenBuffers(1, &capVBO_);
        glBindVertexArray(capVAO_);
        glBindBuffer(GL_ARRAY_BUFFER, capVBO_);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(CapVert), (void*)0);
        glEnableVertexAttribArray(1);
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(CapVert), (void*)(3 * sizeof(float)));
        glBindVertexArray(0);
    }
    glBindBuffer(GL_ARRAY_BUFFER, capVBO_);
    glBufferData(GL_ARRAY_BUFFER, sizeof(quad), quad, GL_DYNAMIC_DRAW);

    glColorMask(GL_TRUE, GL_TRUE, GL_TRUE, GL_TRUE);
    glDepthMask(GL_TRUE);
    glEnable(GL_DEPTH_TEST);
    glStencilFunc(GL_NOTEQUAL, 0, 0xFF);
    glStencilOp(GL_KEEP, GL_KEEP, GL_KEEP);

    auto& shader = viewport3D_.gridShader();  // position + colour, no lighting
    shader.use();
    shader.setMat4("uView", view);
    shader.setMat4("uProj", proj);
    applyClip(shader, nullptr); // the cap is the cut face: never cut it
    glBindVertexArray(capVAO_);
    glDrawArrays(GL_TRIANGLES, 0, 6);
    glBindVertexArray(0);

    glDisable(GL_STENCIL_TEST);
    if (cullWas) glEnable(GL_CULL_FACE);   // leave GL as it was found
}

} // namespace shitcad
