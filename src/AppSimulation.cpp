#include "App.h"
#include "FacePicker.h"
#include "UnitUtils.h"

#include <GLFW/glfw3.h>
#include <imgui.h>
#include <imgui_internal.h> // GetActiveID, for the name field's edit buffer

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>

namespace shitcad {

// ---- workspace --------------------------------------------------------------

bool App::canSwitchWorkspace() const {
    return mode_ == InteractionMode::Navigate &&
           tool_.type != ToolType::Extrude && tool_.type != ToolType::Revolve &&
           tool_.type != ToolType::Loft &&
           tool_.type != ToolType::BooleanUnion && tool_.type != ToolType::BooleanSubtract;
}

void App::setWorkspace(Workspace w) {
    if (w == workspace_ || !canSwitchWorkspace()) return;
    simUi_.placing = false;
    workspace_ = w;
}

void App::commitSimulationEdit() {
    if (sameSimulation(simulation_, simUndoBase_)) return;
    UndoCommand cmd;
    cmd.type = UndoActionType::ModifySimulation;
    cmd.oldSimulation = simUndoBase_;
    cmd.newSimulation = simulation_;
    globalUndo_.push(std::move(cmd));
    simUndoBase_ = simulation_;
    markDirty();
}

float App::simulationSceneExtent() {
    // Same trap as sceneBounds: vertexCount does not change when a mesh is
    // moved, so fold a couple of actual vertices into the key.
    size_t key = scene_.bodyCount();
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
    if (key == simUi_.sceneExtentKey) return simUi_.sceneExtentMm;
    simUi_.sceneExtentKey = key;

    float lo[3] = {1e30f, 1e30f, 1e30f}, hi[3] = {-1e30f, -1e30f, -1e30f};
    for (int i = 0; i < (int)scene_.bodyCount(); i++) {
        for (const auto& v : scene_.getBody(i).vertices) {
            lo[0] = std::min(lo[0], v.px); hi[0] = std::max(hi[0], v.px);
            lo[1] = std::min(lo[1], v.py); hi[1] = std::max(hi[1], v.py);
            lo[2] = std::min(lo[2], v.pz); hi[2] = std::max(hi[2], v.pz);
        }
    }
    float ext = std::max({hi[0] - lo[0], hi[1] - lo[1], hi[2] - lo[2]});
    simUi_.sceneExtentMm = (ext > 1.0f && ext < 1e8f) ? ext : 1000.0f;
    return simUi_.sceneExtentMm;
}

// ---- input --------------------------------------------------------------------

void App::handleSimulationInput(float vpW, float vpH) {
    ImGuiIO& io = ImGui::GetIO();

    if (!io.WantCaptureKeyboard) {
        if (io.KeyCtrl && ImGui::IsKeyPressed(ImGuiKey_Z) && !io.KeyShift) globalUndo();
        if (io.KeyCtrl && ImGui::IsKeyPressed(ImGuiKey_Y)) globalRedo();
        if (io.KeyCtrl && ImGui::IsKeyPressed(ImGuiKey_Z) && io.KeyShift) globalRedo();
        if (io.KeyCtrl && ImGui::IsKeyPressed(ImGuiKey_S)) saveProjectDialog();
        if (io.KeyCtrl && ImGui::IsKeyPressed(ImGuiKey_O)) openProjectDialog();
        if (ImGui::IsKeyPressed(ImGuiKey_Escape)) {
            if (simUi_.placing) simUi_.placing = false;
            else simUi_.selectedNozzle = 0;
        }
        if (ImGui::IsKeyPressed(ImGuiKey_Delete) && simUi_.selectedNozzle &&
            simulation_.findNozzle(simUi_.selectedNozzle)) {
            simulation_.removeNozzle(simUi_.selectedNozzle);
            simUi_.selectedNozzle = 0;
            commitSimulationEdit();
        }
    }

    if (io.MousePos.y < 30.0f || io.WantCaptureMouse) return;
    if (!ImGui::IsMouseClicked(ImGuiMouseButton_Left)) return;

    if (simUi_.placing) {
        if (!meshHover_.hit || meshHover_.bodyIndex >= (int)scene_.bodyCount()) return;
        const Body3D& body = scene_.getBody(meshHover_.bodyIndex);

        // Spray into the solid the STL encloses: an exported fluid cavity has
        // outward normals, so the fluid side is -normal whichever side the
        // user happens to be looking from. Flip in the panel if the file's
        // winding is the other way.
        const double inward[3] = {-meshHover_.normal[0], -meshHover_.normal[1], -meshHover_.normal[2]};
        const double pos[3] = {
            meshHover_.hitWorld[0] + inward[0] * simUi_.standoffMm,
            meshHover_.hitWorld[1] + inward[1] * simUi_.standoffMm,
            meshHover_.hitWorld[2] + inward[2] * simUi_.standoffMm,
        };
        SimNozzle n;
        n.hostFeature = body.sourceFeature;
        setNozzleWorld(n, featureHistory_, pos, inward);
        simUi_.selectedNozzle = simulation_.addNozzle(n);
        if (!io.KeyShift) simUi_.placing = false;
        commitSimulationEdit();
        return;
    }

    // Click near a nozzle to select it; anywhere else clears the selection.
    int w, h;
    glfwGetFramebufferSize(window_, &w, &h);
    float view[16], proj[16];
    getViewProj(w, h, view, proj);
    float bestD = 14.0f * dpiScale_;
    uint32_t best = 0;
    for (const auto& n : simulation_.nozzles) {
        double p[3], a[3];
        if (!nozzleWorld(n, featureHistory_, p, a)) continue;
        const float pf[3] = {(float)p[0], (float)p[1], (float)p[2]};
        float sx, sy;
        if (!worldToScreen(pf, view, proj, vpW, vpH, sx, sy)) continue;
        float d = std::hypot(sx - io.MousePos.x, sy - io.MousePos.y);
        if (d < bestD) { bestD = d; best = n.id; }
    }
    simUi_.selectedNozzle = best;
}

// ---- overlay ------------------------------------------------------------------

namespace {

struct LineVert { float x, y, z, r, g, b; };

void addLine(std::vector<LineVert>& out, const double a[3], const double b[3], const float c[3]) {
    out.push_back({(float)a[0], (float)a[1], (float)a[2], c[0], c[1], c[2]});
    out.push_back({(float)b[0], (float)b[1], (float)b[2], c[0], c[1], c[2]});
}

// Rings at a third, two thirds and the full half-angle, plus generatrices.
// Works for narrow cones and for spray balls (half-angle up to 180).
void addCone(std::vector<LineVert>& out, const double p[3], const double axisIn[3],
             double halfDeg, double len, const float c[3]) {
    double a[3] = {axisIn[0], axisIn[1], axisIn[2]};
    double m = std::sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
    if (m < 1e-12) return;
    for (double& v : a) v /= m;
    const double helper[3] = {std::fabs(a[1]) < 0.9 ? 0.0 : 1.0, std::fabs(a[1]) < 0.9 ? 1.0 : 0.0, 0.0};
    double u[3] = {a[1] * helper[2] - a[2] * helper[1], a[2] * helper[0] - a[0] * helper[2],
                   a[0] * helper[1] - a[1] * helper[0]};
    m = std::sqrt(u[0] * u[0] + u[1] * u[1] + u[2] * u[2]);
    for (double& v : u) v /= m;
    const double vv[3] = {a[1] * u[2] - a[2] * u[1], a[2] * u[0] - a[0] * u[2], a[0] * u[1] - a[1] * u[0]};

    const double pi = 3.14159265358979323846;
    const double half = std::min(std::max(halfDeg, 0.5), 180.0) * pi / 180.0;
    auto point = [&](double polar, double az, double out3[3]) {
        const double ca = std::cos(polar), sa = std::sin(polar);
        for (int k = 0; k < 3; k++)
            out3[k] = p[k] + len * (a[k] * ca + sa * (u[k] * std::cos(az) + vv[k] * std::sin(az)));
    };

    const int seg = 48;
    for (int ring = 1; ring <= 3; ring++) {
        const double polar = std::min(half * ring / 3.0, pi - 1e-3);
        for (int s = 0; s < seg; s++) {
            double q0[3], q1[3];
            point(polar, 2 * pi * s / seg, q0);
            point(polar, 2 * pi * (s + 1) / seg, q1);
            addLine(out, q0, q1, c);
        }
    }
    for (int g = 0; g < 12; g++) {
        double q[3];
        point(std::min(half, pi - 1e-3), 2 * pi * g / 12, q);
        addLine(out, p, q, c);
    }
    const double tip[3] = {p[0] + a[0] * len * 0.35, p[1] + a[1] * len * 0.35, p[2] + a[2] * len * 0.35};
    const float axisCol[3] = {1.0f, 1.0f, 1.0f};
    addLine(out, p, tip, axisCol);
}

} // namespace

void App::renderSimulationOverlay(const float* view, const float* proj) {
    if (workspace_ != Workspace::Simulation) return;

    std::vector<LineVert> verts;
    const double len = simulationSceneExtent() * 0.25;
    const float normalCol[3] = {0.10f, 0.70f, 0.95f};
    const float selectedCol[3] = {1.00f, 0.55f, 0.10f};
    const float previewCol[3] = {0.25f, 0.90f, 0.35f};

    for (const auto& n : simulation_.nozzles) {
        double p[3], a[3];
        if (!nozzleWorld(n, featureHistory_, p, a)) continue;
        addCone(verts, p, a, n.halfAngleDeg, len, n.id == simUi_.selectedNozzle ? selectedCol : normalCol);
    }
    if (simUi_.placing && meshHover_.hit) {
        const double in[3] = {-meshHover_.normal[0], -meshHover_.normal[1], -meshHover_.normal[2]};
        const double p[3] = {meshHover_.hitWorld[0] + in[0] * simUi_.standoffMm,
                             meshHover_.hitWorld[1] + in[1] * simUi_.standoffMm,
                             meshHover_.hitWorld[2] + in[2] * simUi_.standoffMm};
        addCone(verts, p, in, 65.0, len, previewCol);
    }
    if (verts.empty()) return;

    if (!simLineVAO_) {
        glGenVertexArrays(1, &simLineVAO_);
        glGenBuffers(1, &simLineVBO_);
        glBindVertexArray(simLineVAO_);
        glBindBuffer(GL_ARRAY_BUFFER, simLineVBO_);
        glEnableVertexAttribArray(0);
        glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(LineVert), (void*)0);
        glEnableVertexAttribArray(1);
        glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(LineVert), (void*)(3 * sizeof(float)));
        glBindVertexArray(0);
    }
    glBindBuffer(GL_ARRAY_BUFFER, simLineVBO_);
    glBufferData(GL_ARRAY_BUFFER, verts.size() * sizeof(LineVert), verts.data(), GL_DYNAMIC_DRAW);

    // Nozzles sit inside a closed vessel, so the cones are drawn over the
    // geometry rather than depth-tested - otherwise the wall hides them.
    GLboolean depthWas = glIsEnabled(GL_DEPTH_TEST);
    glDisable(GL_DEPTH_TEST);
    glLineWidth(1.5f);
    auto& shader = viewport3D_.gridShader();
    shader.use();
    shader.setMat4("uView", view);
    shader.setMat4("uProj", proj);
    applyClip(shader, nullptr); // spray cones stay whole, so a section still shows them
    glBindVertexArray(simLineVAO_);
    glDrawArrays(GL_LINES, 0, (GLsizei)verts.size());
    glBindVertexArray(0);
    glLineWidth(1.0f);
    if (depthWas) glEnable(GL_DEPTH_TEST);
}

// ---- export -------------------------------------------------------------------

void App::exportSimulationSpecDialog() {
    std::string json;
    simUi_.warnings.clear();
    std::string err;
    if (!buildTier1Spec(simulation_, featureHistory_, json, simUi_.warnings, err)) {
        simUi_.message = err;
        simUi_.messageIsError = true;
        return;
    }
    std::string path = openNativeJsonSaveDialog();
    if (path.empty()) return;
    std::ofstream out(path, std::ios::binary);
    out << json << "\n";
    if (!out) {
        simUi_.message = "Could not write " + path;
        simUi_.messageIsError = true;
        return;
    }
    simUi_.message = "Wrote " + path;
    simUi_.messageIsError = false;
}

// ---- panel ----------------------------------------------------------------------

void App::drawSimulationPanel() {
    if (workspace_ != Workspace::Simulation) return;
    ImGuiIO& io = ImGui::GetIO();
    ImGuiViewport* vp = ImGui::GetMainViewport();

    if (simUi_.selectedNozzle && !simulation_.findNozzle(simUi_.selectedNozzle))
        simUi_.selectedNozzle = 0; // removed by undo

    const float panelW = 330.0f;
    ImGui::SetNextWindowPos({vp->WorkPos.x + vp->WorkSize.x - panelW - 10, vp->WorkPos.y + 45}, ImGuiCond_Appearing);
    ImGui::SetNextWindowSize({panelW, vp->WorkSize.y * 0.75f}, ImGuiCond_Appearing);
    ImGui::Begin("Simulation###simpanel", nullptr, ImGuiWindowFlags_NoCollapse);

    ImGui::TextDisabled("Tier 1: spray line of sight + splash");

    // ---- surfaces
    if (ImGui::CollapsingHeader("Surfaces", ImGuiTreeNodeFlags_DefaultOpen)) {
        int count = 0;
        const auto& feats = featureHistory_.features();
        for (int i = 0; i < (int)feats.size(); i++) {
            const Feature& f = feats[i];
            if (f.type != FeatureType::MeshImport) continue;
            count++;
            ImGui::PushID((int)f.id);
            bool active = !f.suppressed && !featureHistory_.isRolledBack(i) && !f.hasError;
            ImGui::AlignTextToFramePadding();
            if (active) ImGui::TextUnformatted(f.name.c_str());
            else ImGui::TextDisabled("%s", f.name.c_str());
            ImGui::SameLine(130);
            ImGui::SetNextItemWidth(-1);
            int role = (int)simulation_.roleFor(f.id);
            if (ImGui::Combo("##role", &role,
                    [](void*, int r) { return surfaceRoleLabel((SurfaceRole)r); }, nullptr, kSurfaceRoleCount)) {
                simulation_.setRole(f.id, (SurfaceRole)role);
                commitSimulationEdit();
            }
            if (f.hasError) ImGui::TextColored({1, 0.45f, 0.45f, 1}, "  left out: %s", f.errorMsg.c_str());
            else if (!active) ImGui::TextDisabled("  left out: suppressed or rolled back");
            ImGui::PopID();
        }
        if (count == 0)
            ImGui::TextWrapped("No surfaces yet. Import the vessel with Import > STL, one file per surface "
                               "(wall, inlet cap, drain cap...).");
        else
            ImGui::TextDisabled("Walls count toward coverage; caps and obstructions only block spray.");
    }

    // ---- nozzles
    if (ImGui::CollapsingHeader("Nozzles", ImGuiTreeNodeFlags_DefaultOpen)) {
        if (simUi_.placing) {
            ImGui::PushStyleColor(ImGuiCol_Button, ImVec4(0.2f, 0.6f, 0.3f, 1.0f));
            if (ImGui::Button("Click a surface to place... (Esc)", {-1, 0})) simUi_.placing = false;
            ImGui::PopStyleColor();
        } else if (ImGui::Button("+ Place nozzle", {-1, 0})) {
            simUi_.placing = true;
        }
        ImGui::SetNextItemWidth(90);
        ImGui::InputFloat("Standoff from surface (mm)", &simUi_.standoffMm, 0, 0, "%.1f");
        simUi_.standoffMm = std::max(0.0f, simUi_.standoffMm);
        ImGui::TextDisabled("Shift-click places several. Click a cone's apex to select.");

        for (const auto& n : simulation_.nozzles) {
            ImGui::PushID((int)n.id);
            double p[3], a[3];
            bool ok = nozzleWorld(n, featureHistory_, p, a);
            char label[128];
            snprintf(label, sizeof(label), "%s%s", n.name.c_str(), ok ? "" : "  (surface deleted)");
            if (ImGui::Selectable(label, simUi_.selectedNozzle == n.id)) simUi_.selectedNozzle = n.id;
            ImGui::PopID();
        }

        SimNozzle* n = simulation_.findNozzle(simUi_.selectedNozzle);
        if (n) {
            ImGui::Separator();
            // Mirror the name into the edit buffer except while it is being
            // typed in, so undo and selection changes show up.
            static char nameBuf[64];
            if (ImGui::GetActiveID() != ImGui::GetID("Name"))
                snprintf(nameBuf, sizeof(nameBuf), "%s", n->name.c_str());
            ImGui::SetNextItemWidth(-60);
            ImGui::InputText("Name", nameBuf, sizeof(nameBuf));
            if (ImGui::IsItemDeactivatedAfterEdit() && nameBuf[0]) { n->name = nameBuf; commitSimulationEdit(); }

            double p[3], a[3];
            if (nozzleWorld(*n, featureHistory_, p, a)) {
                ImGui::SetNextItemWidth(-60);
                if (ImGui::InputScalarN("Pos mm", ImGuiDataType_Double, p, 3, nullptr, nullptr, "%.1f"))
                    setNozzleWorld(*n, featureHistory_, p, a);
                if (ImGui::IsItemDeactivatedAfterEdit()) commitSimulationEdit();
                ImGui::SetNextItemWidth(-60);
                double aEdit[3] = {a[0], a[1], a[2]};
                if (ImGui::InputScalarN("Axis", ImGuiDataType_Double, aEdit, 3, nullptr, nullptr, "%.3f")) {
                    if (std::sqrt(aEdit[0] * aEdit[0] + aEdit[1] * aEdit[1] + aEdit[2] * aEdit[2]) > 1e-9)
                        setNozzleWorld(*n, featureHistory_, p, aEdit);
                }
                if (ImGui::IsItemDeactivatedAfterEdit()) commitSimulationEdit();
                if (ImGui::Button("Flip")) {
                    const double f[3] = {-a[0], -a[1], -a[2]};
                    setNozzleWorld(*n, featureHistory_, p, f);
                    commitSimulationEdit();
                }
                ImGui::SameLine();
                if (ImGui::Button("Point down (-Y)")) {
                    const double d[3] = {0, -1, 0};
                    setNozzleWorld(*n, featureHistory_, p, d);
                    commitSimulationEdit();
                }
            } else {
                ImGui::TextColored({1, 0.45f, 0.45f, 1}, "Its surface was deleted.");
            }

            ImGui::SetNextItemWidth(-60);
            ImGui::SliderFloat("Half-angle", &n->halfAngleDeg, 1.0f, 180.0f, "%.0f deg");
            if (ImGui::IsItemDeactivatedAfterEdit()) commitSimulationEdit();
            ImGui::SetNextItemWidth(-60);
            ImGui::InputFloat("Flow kg/s", &n->mdotKgS, 0, 0, "%.3f");
            n->mdotKgS = std::max(0.001f, n->mdotKgS);
            if (ImGui::IsItemDeactivatedAfterEdit()) commitSimulationEdit();
            ImGui::SetNextItemWidth(-60);
            ImGui::InputFloat("Press. bar", &n->pressureBar, 0, 0, "%.2f");
            if (ImGui::IsItemDeactivatedAfterEdit()) commitSimulationEdit();
            ImGui::TextDisabled("Half-angle 180 = full spray ball. Pressure is for CFD; Tier 1 uses flow.");

            if (ImGui::Button("Delete nozzle", {-1, 0})) {
                simulation_.removeNozzle(n->id);
                simUi_.selectedNozzle = 0;
                commitSimulationEdit();
            }
        }
    }

    // ---- run settings
    if (ImGui::CollapsingHeader("Run settings")) {
        ImGui::SetNextItemWidth(120);
        ImGui::InputInt("Rays per nozzle", &simulation_.rays, 10000, 100000);
        simulation_.rays = std::clamp(simulation_.rays, 1000, 5000000);
        if (ImGui::IsItemDeactivatedAfterEdit()) commitSimulationEdit();
        ImGui::SetNextItemWidth(120);
        ImGui::SliderInt("Splash bounces", &simulation_.bounces, 0, 4);
        if (ImGui::IsItemDeactivatedAfterEdit()) commitSimulationEdit();
        ImGui::TextDisabled("Coverage is exact at any ray count; rays only sharpen the flux map.");
    }

    // Section view sits above the run: cutting the model open is useful
    // while placing nozzles, not only when looking at results.
    if (ImGui::CollapsingHeader("View")) drawSectionControls();

    drawSimulationRunSection();
    drawSimulationResultsSection();

    ImGui::Separator();
    if (ImGui::Button("Export spec...", {-1, 0})) exportSimulationSpecDialog();
    if (!simUi_.message.empty()) {
        ImGui::PushTextWrapPos(0.0f);
        if (simUi_.messageIsError) ImGui::TextColored({1, 0.45f, 0.45f, 1}, "%s", simUi_.message.c_str());
        else ImGui::TextColored({0.4f, 0.9f, 0.5f, 1}, "%s", simUi_.message.c_str());
        for (const auto& w : simUi_.warnings) ImGui::TextColored({1, 0.85f, 0.3f, 1}, "%s", w.c_str());
        ImGui::PopTextWrapPos();
    }
    ImGui::End();

    // Nozzle names beside their apexes (drawn here: labels must be emitted
    // during the ImGui frame, and the 3D overlay renders after it).
    int w, h;
    glfwGetFramebufferSize(window_, &w, &h);
    float view[16], proj[16];
    getViewProj(w, h, view, proj);
    ImDrawList* dl = ImGui::GetForegroundDrawList();
    for (const auto& nz : simulation_.nozzles) {
        double p[3], a[3];
        if (!nozzleWorld(nz, featureHistory_, p, a)) continue;
        const float pf[3] = {(float)p[0], (float)p[1], (float)p[2]};
        float sx, sy;
        if (!worldToScreen(pf, view, proj, io.DisplaySize.x, io.DisplaySize.y, sx, sy)) continue;
        ImU32 col = nz.id == simUi_.selectedNozzle ? IM_COL32(255, 150, 40, 255) : IM_COL32(40, 180, 240, 255);
        dl->AddCircleFilled({sx, sy}, 4.0f, col);
        dl->AddText({sx + 7, sy - 7}, col, nz.name.c_str());
    }
}

} // namespace shitcad
