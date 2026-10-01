#include "App.h"
#include "FacePicker.h"
#include "UnitUtils.h"
#include "Utf8Path.h"


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

    if (in_.ctrl && in_.keyPressed(Key::Z) && !in_.shift) globalUndo();
    if (in_.ctrl && in_.keyPressed(Key::Y)) globalRedo();
    if (in_.ctrl && in_.keyPressed(Key::Z) && in_.shift) globalRedo();
    if (in_.ctrl && in_.keyPressed(Key::S)) saveProjectDialog();
    if (in_.ctrl && in_.keyPressed(Key::O)) openProjectDialog();
    if (in_.keyPressed(Key::Escape)) {
        if (simUi_.placing) simUi_.placing = false;
        else simUi_.selectedNozzle = 0;
    }
    if (in_.keyPressed(Key::Delete) && simUi_.selectedNozzle &&
        simulation_.findNozzle(simUi_.selectedNozzle)) {
        simulation_.removeNozzle(simUi_.selectedNozzle);
        simUi_.selectedNozzle = 0;
        commitSimulationEdit();
    }

    if (in_.mouseY < in_.viewY) return;
    if (!in_.mouseClicked(MouseButton::Left)) return;

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
        if (!in_.shift) simUi_.placing = false;
        commitSimulationEdit();
        return;
    }

    // Click near a nozzle to select it; anywhere else clears the selection.
    int w, h;
    framebufferSize(w, h);
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
        float d = std::hypot(sx - in_.mouseX, sy - in_.mouseY);
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

    const double half = std::min(std::max(halfDeg, 0.5), 180.0) * kDegToRadD;
    auto point = [&](double polar, double az, double out3[3]) {
        const double ca = std::cos(polar), sa = std::sin(polar);
        for (int k = 0; k < 3; k++)
            out3[k] = p[k] + len * (a[k] * ca + sa * (u[k] * std::cos(az) + vv[k] * std::sin(az)));
    };

    const int seg = 48;
    for (int ring = 1; ring <= 3; ring++) {
        const double polar = std::min(half * ring / 3.0, kPiD - 1e-3);
        for (int s = 0; s < seg; s++) {
            double q0[3], q1[3];
            point(polar, kTwoPiD * s / seg, q0);
            point(polar, kTwoPiD * (s + 1) / seg, q1);
            addLine(out, q0, q1, c);
        }
    }
    for (int g = 0; g < 12; g++) {
        double q[3];
        point(std::min(half, kPiD - 1e-3), kTwoPiD * g / 12, q);
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
    std::string path = chooseFile(FileDialog::SaveJson);
    if (path.empty()) return;
    std::ofstream out(fsPath(path), std::ios::binary);
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

App::SimSetupModel App::simSetupModel() const {
    SimSetupModel m;
    m.open = workspace_ == Workspace::Simulation;
    if (!m.open) return m;
    const auto& feats = featureHistory_.features();
    for (int i = 0; i < (int)feats.size(); i++) {
        const Feature& f = feats[i];
        if (f.type != FeatureType::MeshImport) continue;
        SimSetupModel::Surface s;
        s.feature = f.id;
        s.name = f.name;
        s.role = (int)simulation_.roleFor(f.id);
        s.active = !f.suppressed && !featureHistory_.isRolledBack(i) && !f.hasError;
        s.failed = f.hasError;
        s.errorMsg = f.errorMsg;
        m.surfaces.push_back(std::move(s));
    }
    m.placing = simUi_.placing;
    m.standoffMm = simUi_.standoffMm;
    for (const auto& n : simulation_.nozzles) {
        double p[3], a[3];
        const bool ok = nozzleWorld(n, featureHistory_, p, a);
        m.nozzles.push_back({n.id, n.name + (ok ? "" : "  (surface deleted)")});
    }
    if (const SimNozzle* n = simulation_.findNozzle(simUi_.selectedNozzle)) {
        m.selected = n->id;
        m.name = n->name;
        m.hosted = nozzleWorld(*n, featureHistory_, m.pos, m.axis);
        m.halfAngleDeg = n->halfAngleDeg;
        m.flowKgS = n->mdotKgS;
        m.pressureBar = n->pressureBar;
    }
    m.rays = simulation_.rays;
    m.bounces = simulation_.bounces;
    m.message = simUi_.message;
    m.messageIsError = simUi_.messageIsError;
    m.warnings = simUi_.warnings;
    return m;
}

void App::setSurfaceRole(uint32_t meshFeature, int role) {
    if (role < 0 || role >= kSurfaceRoleCount) return;
    simulation_.setRole(meshFeature, (SurfaceRole)role);
    commitSimulationEdit();
}

void App::setNozzleName(uint32_t id, const std::string& name) {
    SimNozzle* n = simulation_.findNozzle(id);
    if (!n || name.empty()) return;
    n->name = name;
    commitSimulationEdit();
}

void App::setNozzlePosition(uint32_t id, const double pos[3]) {
    SimNozzle* n = simulation_.findNozzle(id);
    double p[3], a[3];
    if (n && nozzleWorld(*n, featureHistory_, p, a)) setNozzleWorld(*n, featureHistory_, pos, a);
}

void App::setNozzleAxis(uint32_t id, const double axis[3]) {
    SimNozzle* n = simulation_.findNozzle(id);
    double p[3], a[3];
    if (!n || !nozzleWorld(*n, featureHistory_, p, a)) return;
    if (std::sqrt(axis[0] * axis[0] + axis[1] * axis[1] + axis[2] * axis[2]) > 1e-9)
        setNozzleWorld(*n, featureHistory_, p, axis);
}

void App::flipNozzle(uint32_t id) {
    SimNozzle* n = simulation_.findNozzle(id);
    double p[3], a[3];
    if (!n || !nozzleWorld(*n, featureHistory_, p, a)) return;
    const double f[3] = {-a[0], -a[1], -a[2]};
    setNozzleWorld(*n, featureHistory_, p, f);
    commitSimulationEdit();
}

void App::pointNozzleDown(uint32_t id) {
    SimNozzle* n = simulation_.findNozzle(id);
    double p[3], a[3];
    if (!n || !nozzleWorld(*n, featureHistory_, p, a)) return;
    const double d[3] = {0, -1, 0};
    setNozzleWorld(*n, featureHistory_, p, d);
    commitSimulationEdit();
}

void App::setNozzleHalfAngle(uint32_t id, float deg) {
    if (SimNozzle* n = simulation_.findNozzle(id)) n->halfAngleDeg = std::clamp(deg, 1.0f, 180.0f);
}

void App::setNozzleFlow(uint32_t id, float kgS) {
    if (SimNozzle* n = simulation_.findNozzle(id)) n->mdotKgS = std::max(0.001f, kgS);
}

void App::setNozzlePressure(uint32_t id, float bar) {
    if (SimNozzle* n = simulation_.findNozzle(id)) n->pressureBar = bar;
}

void App::deleteNozzle(uint32_t id) {
    if (!simulation_.findNozzle(id)) return;
    simulation_.removeNozzle(id);
    if (simUi_.selectedNozzle == id) simUi_.selectedNozzle = 0;
    commitSimulationEdit();
}

void App::setSimulationRays(int rays) { simulation_.rays = std::clamp(rays, 1000, 5000000); }
void App::setSimulationBounces(int bounces) { simulation_.bounces = std::clamp(bounces, 0, 4); }

void App::validateSimulationSelection() {
    if (simUi_.selectedNozzle && !simulation_.findNozzle(simUi_.selectedNozzle))
        simUi_.selectedNozzle = 0; // removed by undo
}

// Nozzle names beside their apexes, in the Simulation workspace.
void App::drawNozzleLabels() {
    if (workspace_ != Workspace::Simulation) return;
    int w, h;
    framebufferSize(w, h);
    float view[16], proj[16];
    getViewProj(w, h, view, proj);
    Overlay2D& ov = overlay_;
    for (const auto& nz : simulation_.nozzles) {
        double p[3], a[3];
        if (!nozzleWorld(nz, featureHistory_, p, a)) continue;
        const float pf[3] = {(float)p[0], (float)p[1], (float)p[2]};
        float sx, sy;
        if (!worldToScreen(pf, view, proj, in_.screenW, in_.screenH, sx, sy)) continue;
        Color32 col = nz.id == simUi_.selectedNozzle ? rgba32(255, 150, 40, 255) : rgba32(40, 180, 240, 255);
        ov.addCircleFilled({sx, sy}, 4.0f, col);
        ov.addText({sx + 7, sy - 7}, col, nz.name.c_str());
    }
}

} // namespace shitcad
