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
        else if (simUi_.placingOpening >= 0) simUi_.placingOpening = -1;
        else { simUi_.selectedNozzle = 0; simUi_.selectedOpening = 0; }
    }
    if (in_.keyPressed(Key::Delete) && simUi_.selectedNozzle &&
        simulation_.findNozzle(simUi_.selectedNozzle)) {
        simulation_.removeNozzle(simUi_.selectedNozzle);
        simUi_.selectedNozzle = 0;
        commitSimulationEdit();
    }
    if (in_.keyPressed(Key::Delete) && simUi_.selectedOpening &&
        simulation_.findOpening(simUi_.selectedOpening)) {
        simulation_.removeOpening(simUi_.selectedOpening);
        simUi_.selectedOpening = 0;
        commitSimulationEdit();
    }

    if (in_.mouseY < in_.viewY) return;
    if (!in_.mouseClicked(MouseButton::Left)) return;

    if (simUi_.placingOpening >= 0) {
        placeOpeningAtHover(in_.shift);
        return;
    }

    if (simUi_.placing) {
        if (!meshHover_.hit || meshHover_.bodyIndex >= (int)scene_.bodyCount()) return;
        const Body3D& body = scene_.getBody(meshHover_.bodyIndex);

        // Spray to the fluid side, -normal, whichever side the user happens to
        // be looking from. Mesh normals point away from the fluid: a fluid
        // cavity export winds that way already, and the inside skin of a
        // solid-wall export is turned round on load (MeshSkinChoice). A file
        // that is wound the other way is fixed with Flip on its skin.
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
        simUi_.selectedOpening = 0;
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
    uint32_t bestOpening = 0;
    for (const auto& o : simulation_.openings) {
        double c[3], ax[3];
        if (!openingWorld(o, featureHistory_, c, ax)) continue;
        const float cf[3] = {(float)c[0], (float)c[1], (float)c[2]};
        float sx, sy;
        if (!worldToScreen(cf, view, proj, vpW, vpH, sx, sy)) continue;
        float d = std::hypot(sx - in_.mouseX, sy - in_.mouseY);
        if (d < bestD) { bestD = d; bestOpening = o.id; best = 0; }
    }
    simUi_.selectedNozzle = best;
    simUi_.selectedOpening = bestOpening;
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

// A circle of `radius` about `axis` at `c`, plus a tick along the axis so a
// disc seen edge-on still shows where it is.
void addRing(std::vector<LineVert>& out, const double c[3], const double axisIn[3], double radius, const float col[3]) {
    double a[3] = {axisIn[0], axisIn[1], axisIn[2]};
    double m = std::sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
    if (m < 1e-12 || radius <= 0) return;
    for (double& v : a) v /= m;
    const double helper[3] = {std::fabs(a[1]) < 0.9 ? 0.0 : 1.0, std::fabs(a[1]) < 0.9 ? 1.0 : 0.0, 0.0};
    double u[3] = {a[1] * helper[2] - a[2] * helper[1], a[2] * helper[0] - a[0] * helper[2],
                   a[0] * helper[1] - a[1] * helper[0]};
    m = std::sqrt(u[0] * u[0] + u[1] * u[1] + u[2] * u[2]);
    for (double& v : u) v /= m;
    const double w[3] = {a[1] * u[2] - a[2] * u[1], a[2] * u[0] - a[0] * u[2], a[0] * u[1] - a[1] * u[0]};
    const int seg = 48;
    for (int s = 0; s < seg; s++) {
        const double t0 = kTwoPiD * s / seg, t1 = kTwoPiD * (s + 1) / seg;
        double p0[3], p1[3];
        for (int k = 0; k < 3; k++) {
            p0[k] = c[k] + radius * (u[k] * std::cos(t0) + w[k] * std::sin(t0));
            p1[k] = c[k] + radius * (u[k] * std::cos(t1) + w[k] * std::sin(t1));
        }
        addLine(out, p0, p1, col);
    }
    const double tip[3] = {c[0] + a[0] * radius, c[1] + a[1] * radius, c[2] + a[2] * radius};
    const double tail[3] = {c[0] - a[0] * radius, c[1] - a[1] * radius, c[2] - a[2] * radius};
    addLine(out, tail, tip, col);
}

// About `count` arrows spread by area over a triangle list, each along its
// triangle's winding normal with a two-line head. Area-weighted and seeded, so
// a cylinder of long slivers still gets arrows all the way up, and the same
// mesh gets the same arrows every frame.
void addNormalArrows(std::vector<LineVert>& out, const std::vector<MeshVertex>& v, double len,
                     const float c[3], int count = 300) {
    const size_t nt = v.size() / 3;
    if (nt == 0 || len <= 0) return;
    std::vector<double> cum(nt);
    double total = 0;
    auto corner = [&](size_t i, double p[3]) { p[0] = v[i].px; p[1] = v[i].py; p[2] = v[i].pz; };
    for (size_t t = 0; t < nt; t++) {
        double a[3], b[3], d[3];
        corner(t * 3, a), corner(t * 3 + 1, b), corner(t * 3 + 2, d);
        const double e1[3] = {b[0] - a[0], b[1] - a[1], b[2] - a[2]}, e2[3] = {d[0] - a[0], d[1] - a[1], d[2] - a[2]};
        const double n[3] = {e1[1] * e2[2] - e1[2] * e2[1], e1[2] * e2[0] - e1[0] * e2[2], e1[0] * e2[1] - e1[1] * e2[0]};
        total += 0.5 * std::sqrt(n[0] * n[0] + n[1] * n[1] + n[2] * n[2]);
        cum[t] = total;
    }
    if (total <= 0) return;
    uint32_t seed = 12345u;
    auto rnd = [&] { seed = seed * 1664525u + 1013904223u; return (seed >> 8) * (1.0 / 16777216.0); };
    for (int k = 0; k < count; k++) {
        const size_t t = std::min(nt - 1, (size_t)(std::lower_bound(cum.begin(), cum.end(), (k + rnd()) / count * total) - cum.begin()));
        double a[3], b[3], d[3];
        corner(t * 3, a), corner(t * 3 + 1, b), corner(t * 3 + 2, d);
        double r1 = rnd(), r2 = rnd();
        if (r1 + r2 > 1) { r1 = 1 - r1; r2 = 1 - r2; }
        double e1[3], e2[3], n[3], base[3];
        for (int i = 0; i < 3; i++) {
            e1[i] = b[i] - a[i];
            e2[i] = d[i] - a[i];
            base[i] = a[i] + r1 * e1[i] + r2 * e2[i];
        }
        n[0] = e1[1] * e2[2] - e1[2] * e2[1];
        n[1] = e1[2] * e2[0] - e1[0] * e2[2];
        n[2] = e1[0] * e2[1] - e1[1] * e2[0];
        const double nl = std::sqrt(n[0] * n[0] + n[1] * n[1] + n[2] * n[2]);
        const double el = std::sqrt(e1[0] * e1[0] + e1[1] * e1[1] + e1[2] * e1[2]);
        if (nl <= 0 || el <= 0) continue;
        double tip[3], h1[3], h2[3];
        for (int i = 0; i < 3; i++) {
            n[i] /= nl;
            const double side = e1[i] / el;   // in the triangle's plane, so square to the arrow
            tip[i] = base[i] + n[i] * len;
            h1[i] = tip[i] - n[i] * len * 0.3 + side * len * 0.15;
            h2[i] = tip[i] - n[i] * len * 0.3 - side * len * 0.15;
        }
        addLine(out, base, tip, c);
        addLine(out, tip, h1, c);
        addLine(out, tip, h2, c);
    }
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
    // Openings: a ring at the radius, and a short tick along the axis.
    const float drainCol[3] = {0.85f, 0.45f, 0.20f};
    const float inletCol[3] = {0.30f, 0.85f, 0.45f};
    for (const auto& o : simulation_.openings) {
        double c[3], ax[3];
        if (!openingWorld(o, featureHistory_, c, ax)) continue;
        const float* col = o.id == simUi_.selectedOpening ? selectedCol
                         : o.kind == OpeningKind::Inlet ? inletCol : drainCol;
        addRing(verts, c, ax, o.radiusMm, col);
    }
    if (simUi_.placingOpening >= 0 && meshHover_.hit) {
        const double c[3] = {meshHover_.hitWorld[0], meshHover_.hitWorld[1], meshHover_.hitWorld[2]};
        const double ax[3] = {meshHover_.normal[0], meshHover_.normal[1], meshHover_.normal[2]};
        addRing(verts, c, ax, 25.0, previewCol);
    }

    // Normal arrows on the kept skins of the imports that ask for them. Drawn
    // from the body's own triangles, so they show the facing that picking and
    // nozzle placement actually use.
    std::vector<LineVert> arrows;
    if (!simUi_.normalsShown.empty()) {
        const float arrowCol[3] = {0.95f, 0.85f, 0.15f};
        for (int i = 0; i < (int)scene_.bodyCount(); i++) {
            const Body3D& b = scene_.getBody(i);
            if (b.visible && b.isMeshOnly() && simUi_.normalsShown.count(b.sourceFeature))
                addNormalArrows(arrows, b.vertices, len * 0.16, arrowCol);
        }
    }
    if (verts.empty() && arrows.empty()) return;

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
    GLboolean depthWas = glIsEnabled(GL_DEPTH_TEST);
    glLineWidth(1.5f);
    auto& shader = viewport3D_.gridShader();
    shader.use();
    shader.setMat4("uView", view);
    shader.setMat4("uProj", proj);
    glBindVertexArray(simLineVAO_);
    glBindBuffer(GL_ARRAY_BUFFER, simLineVBO_);

    // Arrows sit on the walls: depth-tested, so the far side's are hidden, and
    // cut with the section like the walls they belong to.
    if (!arrows.empty()) {
        glBufferData(GL_ARRAY_BUFFER, arrows.size() * sizeof(LineVert), arrows.data(), GL_DYNAMIC_DRAW);
        glEnable(GL_DEPTH_TEST);
        applyClip(shader, &section_);
        glDrawArrays(GL_LINES, 0, (GLsizei)arrows.size());
    }

    // Nozzles sit inside a closed vessel, so the cones are drawn over the
    // geometry rather than depth-tested - otherwise the wall hides them.
    if (!verts.empty()) {
        glBufferData(GL_ARRAY_BUFFER, verts.size() * sizeof(LineVert), verts.data(), GL_DYNAMIC_DRAW);
        glDisable(GL_DEPTH_TEST);
        applyClip(shader, nullptr); // spray cones stay whole, so a section still shows them
        glDrawArrays(GL_LINES, 0, (GLsizei)verts.size());
    }
    glBindVertexArray(0);
    glLineWidth(1.0f);
    if (depthWas) glEnable(GL_DEPTH_TEST);
    else glDisable(GL_DEPTH_TEST);
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
        s.showNormals = simUi_.normalsShown.count(f.id) > 0;
        const auto& md = std::get<MeshImportFeatureData>(f.data);
        s.hasSkinChoice = !md.skins.empty();
        MeshFileInfo info;
        std::string err;
        const UnitInfo* unit = findLengthUnit(md.unit);
        if (unit && probeMeshFile(md.sourcePath, info, err)) {
            std::vector<MeshSkinState> st;
            if (!skinStates(md.sourcePath, md.skins, st, err)) s.skinError = err;
            const bool anyTurned = std::any_of(info.skins.begin(), info.skins.end(), skinAutoFlipped);
            if (info.skins.size() > 1 || anyTurned || s.hasSkinChoice) {
                for (size_t k = 0; k < info.skins.size(); k++) {
                    const MeshSkinInfo& si = info.skins[k];
                    SimSetupModel::Surface::Skin row;
                    row.triangles = si.triangleCount;
                    for (int a = 0; a < 3; a++) row.sizeMm[a] = (si.rawMax[a] - si.rawMin[a]) * unit->toMm;
                    row.closed = si.closed;
                    row.cavity = si.isCavity();
                    row.wallOutside = isWallOutside(info, k);
                    if (k < st.size()) {
                        row.kept = st[k].kept;
                        row.flipped = st[k].flipped;
                        row.userFlipped = st[k].userFlipped;
                    }
                    s.skins.push_back(row);
                }
            }
        }
        m.surfaces.push_back(std::move(s));
    }
    m.placing = simUi_.placing;
    m.standoffMm = simUi_.standoffMm;
    m.skinRevision = simUi_.skinRevision;
    m.placingOpening = simUi_.placingOpening;
    for (const auto& o : simulation_.openings) {
        double c[3], ax[3];
        const bool ok = openingWorld(o, featureHistory_, c, ax);
        m.openings.push_back({o.id, o.name + " (" + openingKindLabel(o.kind) + ")" + (ok ? "" : "  (surface deleted)")});
    }
    if (const SimOpening* o = simulation_.findOpening(simUi_.selectedOpening)) {
        m.selectedOpening = o->id;
        m.openingName = o->name;
        m.openingKind = (int)o->kind;
        m.openingRadiusMm = o->radiusMm;
        double c[3], ax[3];
        m.openingHosted = o->hostFeature != NullFeatureID && openingWorld(*o, featureHistory_, c, ax);
    }
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

// ---- skins ------------------------------------------------------------------

void App::setMeshSkinChoice(uint32_t meshFeature, const MeshSkinChoice& skins) {
    const Feature* f = featureHistory_.findFeature(meshFeature);
    if (!f || f->type != FeatureType::MeshImport) return;
    if (std::get<MeshImportFeatureData>(f->data).skins == skins) return;
    // A placement edit in progress becomes its own undo step first.
    if (meshPlace_.featureID == meshFeature) finishMeshPlace(true);
    f = featureHistory_.findFeature(meshFeature);
    UndoCommand cmd;
    cmd.type = UndoActionType::ModifyMeshImport;
    cmd.featureID = meshFeature;
    cmd.oldMeshImport = std::get<MeshImportFeatureData>(f->data);
    cmd.newMeshImport = cmd.oldMeshImport;
    cmd.newMeshImport.skins = skins;
    featureHistory_.updateMeshImportData(meshFeature, cmd.newMeshImport);
    globalUndo_.push(std::move(cmd));
    markDirty();
    replayAllFeatures();
}

namespace {

// The current skins of an import, and a choice built back from edited states:
// every kept skin named by its point (or none, if all are kept) and every
// user-flipped one. Rebuilding rather than appending keeps the lists from
// collecting stale points with every click.
bool currentSkins(const FeatureHistory& history, uint32_t feature, MeshFileInfo& info,
                  std::vector<MeshSkinState>& st) {
    const Feature* f = history.findFeature(feature);
    if (!f || f->type != FeatureType::MeshImport) return false;
    const auto& md = std::get<MeshImportFeatureData>(f->data);
    std::string err;
    return probeMeshFile(md.sourcePath, info, err) && skinStates(md.sourcePath, md.skins, st, err);
}

MeshSkinChoice choiceFrom(const MeshFileInfo& info, const std::vector<MeshSkinState>& st) {
    MeshSkinChoice c;
    const bool all = std::all_of(st.begin(), st.end(), [](const MeshSkinState& s) { return s.kept; });
    for (size_t k = 0; k < st.size(); k++) {
        const std::array<double, 3> p{info.skins[k].point[0], info.skins[k].point[1], info.skins[k].point[2]};
        if (!all && st[k].kept) c.keep.push_back(p);
        if (st[k].userFlipped) c.flip.push_back(p);
    }
    return c;
}

} // namespace

void App::setSkinKept(uint32_t meshFeature, int skin, bool keep) {
    MeshFileInfo info;
    std::vector<MeshSkinState> st;
    if (!currentSkins(featureHistory_, meshFeature, info, st) || skin < 0 || skin >= (int)st.size()) return;
    st[skin].kept = keep;
    if (std::none_of(st.begin(), st.end(), [](const MeshSkinState& s) { return s.kept; })) {
        simUi_.message = "At least one skin has to stay. Suppress the import to leave it out altogether.";
        simUi_.messageIsError = true;
        simUi_.skinRevision++;   // the unticked box has to snap back
        return;
    }
    setMeshSkinChoice(meshFeature, choiceFrom(info, st));
}

void App::flipSkin(uint32_t meshFeature, int skin) {
    MeshFileInfo info;
    std::vector<MeshSkinState> st;
    if (!currentSkins(featureHistory_, meshFeature, info, st) || skin < 0 || skin >= (int)st.size()) return;
    st[skin].userFlipped = !st[skin].userFlipped;
    setMeshSkinChoice(meshFeature, choiceFrom(info, st));
}

void App::keepInsideOnly(uint32_t meshFeature) {
    MeshFileInfo info;
    std::vector<MeshSkinState> st;
    if (!currentSkins(featureHistory_, meshFeature, info, st)) return;
    bool any = false;
    for (size_t k = 0; k < st.size(); k++) any = any || isWallOutside(info, k);
    if (!any) return;
    for (size_t k = 0; k < st.size(); k++) st[k].kept = !isWallOutside(info, k);
    setMeshSkinChoice(meshFeature, choiceFrom(info, st));
}

void App::resetSkins(uint32_t meshFeature) { setMeshSkinChoice(meshFeature, MeshSkinChoice{}); }

void App::setNormalsShown(uint32_t meshFeature, bool show) {
    if (show) simUi_.normalsShown.insert(meshFeature);
    else simUi_.normalsShown.erase(meshFeature);
}

// ---- openings ------------------------------------------------------------------

bool App::placeOpeningAtHover(bool keepPlacing) {
    if (simUi_.placingOpening < 0 || !meshHover_.hit || meshHover_.bodyIndex >= (int)scene_.bodyCount())
        return false;
    const Body3D& body = scene_.getBody(meshHover_.bodyIndex);
    // Flush with the wall: the centre is the hit itself, the axis the surface
    // normal. The disc's sign does not matter to the cut.
    const double c[3] = {meshHover_.hitWorld[0], meshHover_.hitWorld[1], meshHover_.hitWorld[2]};
    const double ax[3] = {meshHover_.normal[0], meshHover_.normal[1], meshHover_.normal[2]};
    SimOpening o;
    o.kind = (OpeningKind)simUi_.placingOpening;
    o.hostFeature = body.sourceFeature;
    setOpeningWorld(o, featureHistory_, c, ax);
    simUi_.selectedOpening = simulation_.addOpening(o);
    simUi_.selectedNozzle = 0;
    if (!keepPlacing) simUi_.placingOpening = -1;
    commitSimulationEdit();
    return true;
}

void App::setOpeningPlacing(int kind) {
    simUi_.placingOpening = (kind >= 0 && kind < kOpeningKindCount) ? kind : -1;
    if (simUi_.placingOpening >= 0) simUi_.placing = false;
}

void App::setOpeningName(uint32_t id, const std::string& name) {
    SimOpening* o = simulation_.findOpening(id);
    if (!o || name.empty()) return;
    o->name = name;
    commitSimulationEdit();
}

void App::setOpeningKind(uint32_t id, int kind) {
    SimOpening* o = simulation_.findOpening(id);
    if (!o || kind < 0 || kind >= kOpeningKindCount) return;
    o->kind = (OpeningKind)kind;
    commitSimulationEdit();
}

void App::setOpeningRadius(uint32_t id, float mm) {
    if (SimOpening* o = simulation_.findOpening(id)) o->radiusMm = std::max(0.1f, mm);
}

void App::deleteOpening(uint32_t id) {
    if (!simulation_.findOpening(id)) return;
    simulation_.removeOpening(id);
    if (simUi_.selectedOpening == id) simUi_.selectedOpening = 0;
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
    if (simUi_.selectedOpening && !simulation_.findOpening(simUi_.selectedOpening))
        simUi_.selectedOpening = 0;
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
    for (const auto& o : simulation_.openings) {
        double c[3], ax[3];
        if (!openingWorld(o, featureHistory_, c, ax)) continue;
        const float cf[3] = {(float)c[0], (float)c[1], (float)c[2]};
        float sx, sy;
        if (!worldToScreen(cf, view, proj, in_.screenW, in_.screenH, sx, sy)) continue;
        Color32 col = o.id == simUi_.selectedOpening ? rgba32(255, 150, 40, 255)
                    : o.kind == OpeningKind::Inlet ? rgba32(80, 215, 115, 255) : rgba32(215, 115, 50, 255);
        ov.addText({sx + 7, sy - 7}, col, o.name.c_str());
    }
}

} // namespace shitcad
