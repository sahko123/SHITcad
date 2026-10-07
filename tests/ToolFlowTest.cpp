// Drives App's 3D feature tools end to end, headless: enter the tool, set parameters, commit,
// edit, cancel, undo and redo, in Navigate mode and while sketching. ReplayTest covers what
// replay builds from a history; this covers how the tools put that history together.
//
// App is exercised through its own entry points and its private state is read directly (the
// tools' results live there): the tests are static members of AppTestAccess, App's friend.
#include "TestUtil.h"

#include <glad/gl.h>

#include <QGuiApplication>
#include <QOffscreenSurface>
#include <QOpenGLContext>
#include <QSurfaceFormat>

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>

#include <algorithm>
#include <bitset>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <functional>
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <thread>
#include <unordered_map>
#include <variant>
#include <vector>

#include "App.h"

using namespace shitcad;

namespace shitcad {

struct AppTestAccess {

struct Host : AppHost {
    void setWindowTitle(const std::string&) override {}
    bool chooseFile(FileDialog, const char*, std::string&) override { return false; }
};

struct Fixture {
    Host host;
    App app;
    Fixture() { app.init(&host, 1.0f); }
    ~Fixture() { app.shutdown(); }
};

static void addRect(Sketch& sk, double x0, double y0, double x1, double y1) {
    EntityID p0 = sk.addPoint(x0, y0), p1 = sk.addPoint(x1, y0);
    EntityID p2 = sk.addPoint(x1, y1), p3 = sk.addPoint(x0, y1);
    sk.addLine(p0, p1);
    sk.addLine(p1, p2);
    sk.addLine(p2, p3);
    sk.addLine(p3, p0);
}

static double volume(const Scene3D& scene) {
    double v = 0;
    for (size_t i = 0; i < scene.bodyCount(); i++) {
        const Body3D& b = scene.getBody((int)i);
        if (b.isMeshOnly()) continue;
        GProp_GProps props;
        BRepGProp::VolumeProperties(b.shape, props);
        v += props.Mass();
    }
    return v;
}

static int countFeatures(App& app, FeatureType t) {
    int n = 0;
    for (const auto& f : app.featureHistory_.features()) n += f.type == t;
    return n;
}

static FeatureID firstFeature(App& app, FeatureType t) {
    for (const auto& f : app.featureHistory_.features())
        if (f.type == t) return f.id;
    return NullFeatureID;
}

static bool idle(App& app) { return app.tool_.type == ToolType::None; }

// A frame with the tool active exercises the preview upload and draw.
static void drawAFrame(App& app) {
    InputFrame in;
    app.frame(0.016f, 640, 480, in);
    app.paint();
}

static void testExtrudeNavigate() {
    std::printf("extrude from Navigate: create, undo, edit, cancel\n");
    Fixture fx; App& a = fx.app;
    addRect(a.sketchPlanes_[0].sketch, 0, 0, 20, 10);

    a.enterExtrudeMode();
    CHECK(a.tool_.type == ToolType::Extrude, "tool did not start");
    CHECK(a.extrudeTool_.allProfiles.size() == 1 && a.extrudeTool_.selectedProfileIndices.size() == 1,
          "the single profile should be auto-selected");
    a.setExtrudeDistanceText("10");
    a.updateExtrudePreview();
    CHECK(a.extrudeTool_.previewBody.vertexCount > 0 && a.extrudeTool_.previewBody.vao != 0, "no preview uploaded");
    drawAFrame(a);

    a.commitExtrude();
    CHECK(idle(a), "tool should end on commit");
    CHECK(a.scene_.bodyCount() == 1, "bodies=%zu", a.scene_.bodyCount());
    CHECK(near(volume(a.scene_), 2000, 1), "volume %.1f", volume(a.scene_));
    CHECK(countFeatures(a, FeatureType::Sketch) == 1 && countFeatures(a, FeatureType::Extrude) == 1,
          "the source sketch and the extrude should both be recorded");

    a.globalUndo();
    CHECK(a.scene_.bodyCount() == 0 && countFeatures(a, FeatureType::Extrude) == 0, "undo of the extrude");
    CHECK(countFeatures(a, FeatureType::Sketch) == 1, "the sketch is a separate undo step");
    a.globalUndo();
    CHECK(countFeatures(a, FeatureType::Sketch) == 0, "undo of the implicit sketch");
    a.globalRedo();
    a.globalRedo();
    CHECK(a.scene_.bodyCount() == 1 && near(volume(a.scene_), 2000, 1), "redo restores the body");

    // Edit: the feature is suppressed while the tool shows its preview.
    FeatureID ext = firstFeature(a, FeatureType::Extrude);
    a.editExtrudeFeature(ext);
    CHECK(a.tool_.type == ToolType::Extrude && a.extrudeTool_.editingFeatureID == ext, "edit did not start");
    CHECK(a.scene_.bodyCount() == 0, "the edited feature should be suppressed");
    CHECK(near(a.extrudeTool_.height, 10, 1e-4) && a.extrudeTool_.selectedProfileIndices.size() == 1,
          "saved parameters / profile not restored");
    a.setExtrudeDistanceText("25");
    a.commitExtrude();
    CHECK(idle(a) && countFeatures(a, FeatureType::Extrude) == 1, "edit should modify in place");
    CHECK(near(volume(a.scene_), 5000, 1), "edited volume %.1f", volume(a.scene_));
    a.globalUndo();
    CHECK(near(volume(a.scene_), 2000, 1), "undo of the edit: %.1f", volume(a.scene_));
    a.globalRedo();
    CHECK(near(volume(a.scene_), 5000, 1), "redo of the edit: %.1f", volume(a.scene_));

    a.editExtrudeFeature(ext);
    a.setExtrudeDistanceText("99");
    a.cancelExtrude();
    CHECK(idle(a) && a.extrudeTool_.editingFeatureID == 0, "cancel should end the tool");
    CHECK(a.scene_.bodyCount() == 1 && near(volume(a.scene_), 5000, 1),
          "cancel must restore the suppressed feature unchanged: %.1f", volume(a.scene_));
}

static void testExtrudeCancelWhileCreating() {
    std::printf("extrude: cancel while creating leaves nothing behind\n");
    Fixture fx; App& a = fx.app;
    addRect(a.sketchPlanes_[0].sketch, 0, 0, 20, 10);
    a.enterExtrudeMode();
    a.cancelExtrude();
    CHECK(idle(a) && a.scene_.bodyCount() == 0 && a.featureHistory_.features().empty(), "cancel left state behind");

    // No profiles anywhere: the tool does not start.
    Fixture empty;
    empty.app.enterExtrudeMode();
    CHECK(empty.app.tool_.type != ToolType::Extrude, "extrude started with nothing to extrude");
}

static void testExtrudeWhileSketching() {
    std::printf("extrude while sketching finishes the sketch\n");
    Fixture fx; App& a = fx.app;
    a.enterSketchMode(0);
    addRect(a.activeSketch(), 5, 5, 25, 15); // clear of the projected origin anchor
    a.enterExtrudeMode();
    CHECK(a.tool_.type == ToolType::Extrude && a.extrudeTool_.sketchPlaneIndex == 0, "should use the active sketch");
    a.setExtrudeDistanceText("10");
    a.commitExtrude();
    CHECK(a.mode_ == InteractionMode::Navigate, "commit should leave sketch mode");
    CHECK(a.scene_.bodyCount() == 1 && near(volume(a.scene_), 2000, 1), "volume %.1f", volume(a.scene_));
    CHECK(countFeatures(a, FeatureType::Sketch) == 1, "sketch features=%d", countFeatures(a, FeatureType::Sketch));

    // Sketching again, then editing the extrude: the sketch in progress is saved first.
    a.enterSketchMode(0);
    addRect(a.activeSketch(), 40, 5, 50, 15);
    FeatureID ext = firstFeature(a, FeatureType::Extrude);
    a.editExtrudeFeature(ext);
    CHECK(a.mode_ == InteractionMode::Navigate && a.tool_.type == ToolType::Extrude, "edit while sketching");
    a.cancelExtrude();
    CHECK(a.mode_ == InteractionMode::Navigate && near(volume(a.scene_), 2000, 1), "volume after %.1f", volume(a.scene_));
}

static void testExtrudeCut() {
    std::printf("extrude cut\n");
    Fixture fx; App& a = fx.app;
    addRect(a.sketchPlanes_[0].sketch, 0, 0, 20, 10);
    a.enterExtrudeMode();
    a.setExtrudeDistanceText("10");
    a.commitExtrude();
    CHECK(near(volume(a.scene_), 2000, 1), "base volume");

    // A 4 x 4 window on the YZ plane (u = y, v = z), cut 5 deep along +X into the box.
    a.enterSketchMode(2);
    addRect(a.activeSketch(), 2, 3, 6, 7);
    a.enterExtrudeMode();
    CHECK(a.tool_.type == ToolType::Extrude && a.extrudeTool_.sketchPlaneIndex == 2, "plane %d", a.extrudeTool_.sketchPlaneIndex);
    a.setExtrudeOperation((int)ExtrudeOperation::Cut);
    a.setExtrudeDirection((int)ExtrudeDirection::OtherSide);
    a.setExtrudeDistanceText("5");
    a.updateExtrudePreview();
    drawAFrame(a);
    a.commitExtrude();
    CHECK(near(volume(a.scene_), 2000 - 80, 1), "cut volume %.1f", volume(a.scene_));
    a.globalUndo();
    CHECK(near(volume(a.scene_), 2000, 1), "undo of the cut %.1f", volume(a.scene_));
}

static void testRevolve() {
    std::printf("revolve: create, edit, cancel, undo\n");
    Fixture fx; App& a = fx.app;
    Sketch& sk = a.sketchPlanes_[0].sketch;
    addRect(sk, 5, 0, 10, 4);
    EntityID axisA = sk.addPoint(0, 0), axisB = sk.addPoint(0, 10);
    EntityID axis = sk.addLine(axisA, axisB);

    a.enterRevolveMode();
    CHECK(a.tool_.type == ToolType::Revolve, "tool did not start");
    CHECK(a.revolveTool_.selectedProfileIndices.size() == 1, "profile not auto-selected");
    a.revolveTool_.axisLineID = axis;
    a.revolveTool_.phase = RevolvePhase::Adjusting;
    a.updateRevolvePreview();
    CHECK(a.revolveTool_.previewBody.vertexCount > 0, "no preview");
    drawAFrame(a);
    a.commitRevolve();
    const double full = 3.14159265358979 * (100 - 25) * 4;
    CHECK(idle(a) && a.scene_.bodyCount() == 1, "bodies=%zu", a.scene_.bodyCount());
    CHECK(near(volume(a.scene_), full, full * 0.01), "volume %.1f vs %.1f", volume(a.scene_), full);
    CHECK(countFeatures(a, FeatureType::Sketch) == 1 && countFeatures(a, FeatureType::Revolve) == 1, "features");

    FeatureID rev = firstFeature(a, FeatureType::Revolve);
    a.editRevolveFeature(rev);
    CHECK(a.tool_.type == ToolType::Revolve && a.revolveTool_.editingFeatureID == rev, "edit did not start");
    CHECK(a.revolveTool_.axisLineID == axis && a.revolveTool_.phase == RevolvePhase::Adjusting, "axis / phase not restored");
    CHECK(a.scene_.bodyCount() == 0, "edited feature should be suppressed");
    a.setRevolveAngleText("180");
    a.commitRevolve();
    CHECK(near(volume(a.scene_), full / 2, full * 0.01), "half revolve %.1f", volume(a.scene_));
    CHECK(countFeatures(a, FeatureType::Revolve) == 1, "edit should modify in place");

    a.editRevolveFeature(rev);
    a.cancelRevolve();
    CHECK(idle(a) && near(volume(a.scene_), full / 2, full * 0.01), "cancel: %.1f", volume(a.scene_));
    a.globalUndo();
    a.globalUndo();
    CHECK(a.scene_.bodyCount() == 0 && countFeatures(a, FeatureType::Revolve) == 0, "undo");
}

static void testLoft() {
    std::printf("loft: create, edit, cancel, undo\n");
    Fixture fx; App& a = fx.app;
    SketchPlane upper = a.sketchPlanes_[0];
    upper.sketch.clear();
    upper.planeID = a.nextPlaneID_++;
    upper.origin[2] = 20;
    upper.isReferencePlane = false;
    upper.name = "Upper";
    a.sketchPlanes_.push_back(upper);
    addRect(a.sketchPlanes_[0].sketch, 0, 0, 10, 10);
    addRect(a.sketchPlanes_[3].sketch, 0, 0, 10, 10);

    a.enterLoftMode();
    CHECK(a.tool_.type == ToolType::Loft, "tool did not start");
    a.addLoftSection(0);
    CHECK(!a.loftTool_.canCommit(), "one section is not enough");
    a.commitLoft();
    CHECK(a.scene_.bodyCount() == 0, "committed with one section");
    a.addLoftSection(3);
    CHECK(a.loftTool_.canCommit(), "two sections should be enough");
    a.updateLoftPreview();
    CHECK(a.loftTool_.previewBody.vertexCount > 0, "no preview");
    drawAFrame(a);
    a.commitLoft();
    CHECK(idle(a) && a.scene_.bodyCount() == 1, "bodies=%zu", a.scene_.bodyCount());
    CHECK(near(volume(a.scene_), 2000, 2), "volume %.1f", volume(a.scene_));
    CHECK(countFeatures(a, FeatureType::Sketch) == 2 && countFeatures(a, FeatureType::Loft) == 1,
          "every section's sketch should be recorded");

    FeatureID loft = firstFeature(a, FeatureType::Loft);
    a.editLoftFeature(loft);
    CHECK(a.tool_.type == ToolType::Loft && a.loftTool_.sections.size() == 2 && a.loftTool_.editingFeatureID == loft,
          "edit did not restore the sections");
    CHECK(a.scene_.bodyCount() == 0, "edited feature should be suppressed");
    a.cancelLoft();
    CHECK(idle(a) && near(volume(a.scene_), 2000, 2), "cancel restores: %.1f", volume(a.scene_));
    a.editLoftFeature(loft);
    a.commitLoft();
    CHECK(countFeatures(a, FeatureType::Loft) == 1 && near(volume(a.scene_), 2000, 2), "re-commit of an unchanged edit");
    a.globalUndo();
    CHECK(a.scene_.bodyCount() == 1, "undo of an unchanged edit keeps the body");
}

// "Generate CFD case" end to end: the synthetic vessel's three STLs with their
// roles, one nozzle, then cip-sim's case generator as the button runs it.
// Written to a Windows folder and not meshed, so it needs Python but not WSL.
static void testCfdCase(const std::string& cipSim, const std::string& python) {
    std::printf("generate CFD case\n");
    namespace fs = std::filesystem;
    const fs::path geom = fs::path(cipSim) / "geometry" / "synthetic";
    if (!fs::exists(geom / "vessel_wall.stl")) {
        std::printf("  skipped: no synthetic geometry in %s\n", geom.string().c_str());
        return;
    }
    Fixture fx; App& a = fx.app;
    std::map<std::string, FeatureID> ids;
    for (const char* n : {"vessel_wall", "nozzle_inlet", "drain_outlet"}) {
        MeshImportFeatureData md;
        md.sourcePath = (geom / (std::string(n) + ".stl")).string();
        md.unit = "m";
        ids[n] = a.featureHistory_.addMeshImportFeature(md, n);
    }
    a.replayAllFeatures();
    a.simulation_.setRole(ids["nozzle_inlet"], SurfaceRole::Inlet);
    a.simulation_.setRole(ids["drain_outlet"], SurfaceRole::Drain);
    SimNozzle nz;
    nz.hostFeature = ids["vessel_wall"];
    nz.position[2] = 980;            // mm, on the axis under the inlet
    nz.axis[1] = 0;
    nz.axis[2] = -1;
    a.simulation_.addNozzle(nz);

    const fs::path cases = fs::temp_directory_path() / "shitcad_cfd_test";
    fs::remove_all(cases);
    a.simEngine_.cipSimPath = cipSim;
    a.simEngine_.python = python;
    a.simEngine_.cfdCasesDir = cases.generic_string();
    a.simEngine_.loaded = true;
    a.setCfdMesh(false);

    auto runToEnd = [&a] {
        a.startCfdCase();
        const auto t0 = std::chrono::steady_clock::now();
        while (a.cfd_.phase == App::SimPhase::Running &&
               std::chrono::steady_clock::now() - t0 < std::chrono::seconds(180)) {
            a.pollCfdCase();
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }
    };

    runToEnd();
    App::CfdCaseModel m = a.cfdCaseModel();
    CHECK(m.done, "not done: %s", m.error.c_str());
    CHECK(m.summary.find("3 surfaces") != std::string::npos, "summary: %s", m.summary.c_str());
    const bool roles = m.summary.find("nozzle_inlet (inlet)") != std::string::npos &&
                       m.summary.find("drain_outlet (drain)") != std::string::npos &&
                       m.summary.find("vessel_wall (wall)") != std::string::npos;
    CHECK(roles, "roles: %s", m.summary.c_str());
    CHECK(m.summary.find("Not meshed") != std::string::npos, "summary: %s", m.summary.c_str());
    const fs::path out = fs::path(m.caseDir);
    const bool files = fs::exists(out / "system" / "snappyHexMeshDict") && fs::exists(out / "Allmesh") &&
                       fs::exists(out / "constant" / "triSurface" / "drain_outlet.stl") &&
                       fs::exists(out / "constant" / "parcelInjectionProperties");
    CHECK(files, "case files missing in %s", out.string().c_str());
    std::error_code ec;
    CHECK(!m.caseDir.empty() && fs::equivalent(out.parent_path(), cases, ec), "case in '%s', not under the cases folder",
          out.string().c_str());

    // No way out for liquid or air: refused, with the reason shown.
    a.simulation_.setRole(ids["nozzle_inlet"], SurfaceRole::Obstruction);
    a.simulation_.setRole(ids["drain_outlet"], SurfaceRole::Obstruction);
    runToEnd();
    m = a.cfdCaseModel();
    CHECK(!m.done && m.error.find("no inlet or drain") != std::string::npos, "closed vessel: done=%d error=%s",
          (int)m.done, m.error.c_str());
    fs::remove_all(cases);
}

};

} // namespace shitcad

int main(int argc, char** argv) {
    std::setvbuf(stdout, nullptr, _IONBF, 0);   // a crash must not swallow the progress lines
    int qtArgc = 1;
    char* qtArgv[] = {argv[0], nullptr};
    QGuiApplication qapp(qtArgc, qtArgv);
    QSurfaceFormat fmt;
    fmt.setVersion(3, 3);
    fmt.setProfile(QSurfaceFormat::CoreProfile);
    QOpenGLContext ctx;
    ctx.setFormat(fmt);
    QOffscreenSurface surface;
    surface.setFormat(fmt);
    surface.create();
    if (!ctx.create() || !ctx.makeCurrent(&surface)) {
        std::printf("no GL context\n");
        return 1;
    }
    gladLoadGL([](const char* name) -> GLADapiproc {
        return (GLADapiproc)QOpenGLContext::currentContext()->getProcAddress(name);
    });

    AppTestAccess::testExtrudeNavigate();
    AppTestAccess::testExtrudeCancelWhileCreating();
    AppTestAccess::testExtrudeWhileSketching();
    AppTestAccess::testExtrudeCut();
    AppTestAccess::testRevolve();
    AppTestAccess::testLoft();
    // ToolFlowTest.exe <cip-sim dir> [python]: also the CFD case button.
    if (argc >= 2) AppTestAccess::testCfdCase(argv[1], argc >= 3 ? argv[2] : "python");

    ctx.doneCurrent();
    std::printf("\n%d checks, %d failed\n", g_checks, g_failures);
    return g_failures == 0 ? 0 : 1;
}
