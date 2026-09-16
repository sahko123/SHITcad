// Tests for the simulation set-up: surface roles, nozzles attached to their
// host mesh, cip-sim spec export, and save/load.
//
// Also writes a cross-check bundle for cip-sim to verify from the Python side:
//   SimulationTest.exe [out_dir]
//   (cip-sim) CIPSIM_SHITCAD_EXPORT=<out_dir> python -m unittest tests.test_shitcad_export

#include "FacePicker.h"
#include "FeatureHistory.h"
#include "FeatureReplay.h"
#include "MeshImport.h"
#include "Scene3D.h"
#include "Serialization.h"
#include "Section.h"
#include "ShaderProgram.h"
#include "Viewport3D.h"
#include "SimProcess.h"
#include "SimResults.h"
#include "Simulation.h"
#include "SketchPlane.h"

#include <chrono>
#include <thread>

#include <glad/gl.h>
#include <GLFW/glfw3.h>
#include <nlohmann/json.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

using namespace shitcad;
using json = nlohmann::json;
namespace fs = std::filesystem;

static int g_failures = 0;
static int g_checks = 0;

#define CHECK(cond, ...)                                                    \
    do {                                                                    \
        g_checks++;                                                         \
        if (!(cond)) {                                                      \
            g_failures++;                                                   \
            std::printf("  FAIL %s:%d: %s -- ", __FILE__, __LINE__, #cond); \
            std::printf(__VA_ARGS__);                                       \
            std::printf("\n");                                              \
        }                                                                   \
    } while (0)

static bool near(double a, double b, double tol) { return std::fabs(a - b) <= tol; }

struct Tri { float v[9]; };

static std::vector<Tri> boxTriangles(float x0, float y0, float z0, float x1, float y1, float z1) {
    const float c[8][3] = {{x0, y0, z0}, {x1, y0, z0}, {x1, y1, z0}, {x0, y1, z0},
                           {x0, y0, z1}, {x1, y0, z1}, {x1, y1, z1}, {x0, y1, z1}};
    const int q[6][4] = {{0, 3, 2, 1}, {4, 5, 6, 7}, {0, 1, 5, 4}, {2, 3, 7, 6}, {1, 2, 6, 5}, {3, 0, 4, 7}};
    std::vector<Tri> out;
    for (const auto& f : q) {
        for (const auto& t : {std::array<int, 3>{f[0], f[1], f[2]}, std::array<int, 3>{f[0], f[2], f[3]}}) {
            Tri tri;
            for (int k = 0; k < 3; k++)
                for (int a = 0; a < 3; a++) tri.v[k * 3 + a] = c[t[k]][a];
            out.push_back(tri);
        }
    }
    return out;
}

static void writeBinaryStl(const fs::path& path, const std::vector<Tri>& tris) {
    std::ofstream f(path, std::ios::binary);
    char header[80] = {};
    f.write(header, 80);
    uint32_t n = (uint32_t)tris.size();
    f.write((const char*)&n, 4);
    for (const auto& t : tris) {
        const float zero[3] = {0, 0, 0};
        f.write((const char*)zero, 12);
        f.write((const char*)t.v, 36);
        uint16_t attr = 0;
        f.write((const char*)&attr, 2);
    }
}

static std::vector<SketchPlane> referencePlanes() {
    std::vector<SketchPlane> planes(3);
    for (int i = 0; i < 3; i++) planes[i].planeID = (PlaneID)(i + 1);
    return planes;
}

static json parse(const std::string& s) { return json::parse(s); }

// ---- fixture: an Onshape-style Z-up vessel stood upright in this Y-up app --

struct Fixture {
    fs::path dir;
    FeatureHistory history;
    FeatureID wall = 0, drain = 0;
    MeshImportFeatureData wallData;
};

static Fixture makeFixture() {
    Fixture fx;
    fx.dir = fs::temp_directory_path() / "shitcad_simulation_test";
    fs::create_directories(fx.dir);
    // 0.6 x 0.6 x 1.3 m, Z-up, in metres - as an Onshape export would be.
    writeBinaryStl(fx.dir / "vessel_wall.stl", boxTriangles(-0.3f, -0.3f, 0.0f, 0.3f, 0.3f, 1.3f));
    // Small drain cap box, in millimetres.
    writeBinaryStl(fx.dir / "drain_outlet.stl", boxTriangles(-50, -50, 0, 50, 50, 10));

    fx.wallData.sourcePath = (fx.dir / "vessel_wall.stl").string();
    fx.wallData.unit = "m";
    // Stand it upright (Z-up file -> Y-up viewport) and move it off the origin,
    // exactly what the Place panel's buttons produce.
    double q[9];
    axisRotation(0, -90.0, q);
    const double origin[3] = {0, 0, 0};
    rotateAbout(fx.wallData.transform, q, origin);
    fx.wallData.transform.t[0] = 250.0;
    fx.wallData.transform.t[2] = -100.0;
    fx.wall = fx.history.addMeshImportFeature(fx.wallData, "vessel_wall");

    MeshImportFeatureData dd;
    dd.sourcePath = (fx.dir / "drain_outlet.stl").string();
    dd.unit = "mm";
    axisRotation(0, -90.0, q);
    rotateAbout(dd.transform, q, origin);
    dd.transform.t[0] = 250.0;
    dd.transform.t[2] = -100.0;
    fx.drain = fx.history.addMeshImportFeature(dd, "drain_outlet");
    return fx;
}

// ---- tests -----------------------------------------------------------------

static void testRoles() {
    std::printf("roles\n");
    SimulationSetup s;
    CHECK(s.roleFor(7) == SurfaceRole::Wall, "default role is not Wall");
    s.setRole(7, SurfaceRole::Drain);
    s.setRole(7, SurfaceRole::Inlet);
    CHECK(s.roleFor(7) == SurfaceRole::Inlet && s.roles.size() == 1, "setRole did not replace");
    CHECK(surfaceRoleScored(SurfaceRole::Wall) && surfaceRoleScored(SurfaceRole::InternalWall), "walls not scored");
    CHECK(!surfaceRoleScored(SurfaceRole::Inlet) && !surfaceRoleScored(SurfaceRole::Drain) &&
          !surfaceRoleScored(SurfaceRole::Obstruction), "caps scored");
    for (int i = 0; i < kSurfaceRoleCount; i++) {
        SurfaceRole r;
        CHECK(surfaceRoleFromName(surfaceRoleName((SurfaceRole)i), r) && r == (SurfaceRole)i, "role name %d", i);
    }
}

static void testNozzleFollowsHost() {
    std::printf("nozzle attached to its surface\n");
    Fixture fx = makeFixture();
    SimNozzle n;
    n.hostFeature = fx.wall;
    const double wp[3] = {250.0, 1280.0, -100.0}, wa[3] = {0.0, -2.0, 0.0};
    setNozzleWorld(n, fx.history, wp, wa);

    double p[3], a[3];
    CHECK(nozzleWorld(n, fx.history, p, a), "host not found");
    CHECK(near(p[0], 250, 1e-9) && near(p[1], 1280, 1e-9) && near(p[2], -100, 1e-9),
          "world round trip (%g, %g, %g)", p[0], p[1], p[2]);
    CHECK(near(a[1], -1.0, 1e-12), "axis not normalised: %g", a[1]);

    // Move the vessel: the nozzle moves with it, keeping its place on the wall.
    MeshImportFeatureData moved = fx.wallData;
    moved.transform.t[1] += 500.0;
    fx.history.updateMeshImportData(fx.wall, moved);
    CHECK(nozzleWorld(n, fx.history, p, a) && near(p[1], 1780, 1e-9), "nozzle did not follow a move: y=%g", p[1]);

    // Rotate the vessel 90 deg about Z through its origin: nozzle and axis rotate too.
    double q[9];
    axisRotation(2, 90.0, q);
    const double pivot[3] = {moved.transform.t[0], moved.transform.t[1], moved.transform.t[2]};
    rotateAbout(moved.transform, q, pivot);
    fx.history.updateMeshImportData(fx.wall, moved);
    CHECK(nozzleWorld(n, fx.history, p, a), "host lost");
    CHECK(near(a[0], 1.0, 1e-12) && near(a[1], 0.0, 1e-12), "axis did not rotate: (%g, %g, %g)", a[0], a[1], a[2]);

    // Host deleted: nozzleWorld reports it rather than inventing a position.
    fx.history.removeFeature(fx.wall);
    CHECK(!nozzleWorld(n, fx.history, p, a), "deleted host not reported");
}

static void testSpecExportErrors() {
    std::printf("spec export errors\n");
    Fixture fx = makeFixture();
    SimulationSetup s;
    std::string out, err;
    std::vector<std::string> warn;

    CHECK(!buildTier1Spec(s, fx.history, out, warn, err) && err.find("No nozzles") != std::string::npos,
          "no nozzles: %s", err.c_str());

    SimNozzle n;
    n.hostFeature = fx.wall;
    const double wp[3] = {250, 1280, -100}, wa[3] = {0, -1, 0};
    setNozzleWorld(n, fx.history, wp, wa);
    s.addNozzle(n);

    SimulationSetup capsOnly = s;
    capsOnly.setRole(fx.wall, SurfaceRole::Inlet);
    capsOnly.setRole(fx.drain, SurfaceRole::Drain);
    err.clear();
    CHECK(!buildTier1Spec(capsOnly, fx.history, out, warn, err) && err.find("No surface is a wall") != std::string::npos,
          "no scored: %s", err.c_str());

    FeatureHistory dup = fx.history;
    dup.renameFeature(fx.drain, "vessel_wall");
    err.clear();
    CHECK(!buildTier1Spec(s, dup, out, warn, err) && err.find("Two surfaces are named") != std::string::npos,
          "duplicate: %s", err.c_str());

    FeatureHistory yards = fx.history;
    MeshImportFeatureData y = std::get<MeshImportFeatureData>(yards.findFeature(fx.drain)->data);
    y.unit = "yd";
    yards.updateMeshImportData(fx.drain, y);
    err.clear();
    CHECK(!buildTier1Spec(s, yards, out, warn, err) && err.find("'yd'") != std::string::npos,
          "unsupported unit: %s", err.c_str());

    FeatureHistory gone = fx.history;
    gone.removeFeature(fx.wall);
    err.clear();
    CHECK(!buildTier1Spec(s, gone, out, warn, err) && err.find("deleted") != std::string::npos,
          "host deleted: %s", err.c_str());

    SimulationSetup badAngle = s;
    badAngle.nozzles[0].halfAngleDeg = 0.0f;
    err.clear();
    CHECK(!buildTier1Spec(badAngle, fx.history, out, warn, err) && err.find("half-angle") != std::string::npos,
          "half-angle: %s", err.c_str());

    // Suppressed surface: left out with a warning, not an error.
    FeatureHistory sup = fx.history;
    sup.suppressFeature(fx.drain);
    warn.clear();
    err.clear();
    CHECK(buildTier1Spec(s, sup, out, warn, err), "suppressed drain should still export: %s", err.c_str());
    CHECK(parse(out)["surfaces"].size() == 1 && !warn.empty(), "suppressed surface not left out with a warning");
}

static void testSpecContent() {
    std::printf("spec content\n");
    Fixture fx = makeFixture();
    SimulationSetup s;
    s.setRole(fx.drain, SurfaceRole::Drain);
    SimNozzle n;
    n.name = "top_spray";
    n.hostFeature = fx.wall;
    n.halfAngleDeg = 70.0f;
    n.mdotKgS = 0.25f;
    const double wp[3] = {250, 1280, -100}, wa[3] = {0, -1, 0};
    setNozzleWorld(n, fx.history, wp, wa);
    s.addNozzle(n);

    std::string out, err;
    std::vector<std::string> warn;
    CHECK(buildTier1Spec(s, fx.history, out, warn, err), "%s", err.c_str());
    json j = parse(out);
    CHECK(j["units"] == "mm", "units");
    CHECK(j["up"] == json({0.0, 1.0, 0.0}), "up=%s", j["up"].dump().c_str());
    CHECK(j["surfaces"].size() == 2, "surfaces=%zu", j["surfaces"].size());
    const json& w = j["surfaces"][0];
    CHECK(w["name"] == "vessel_wall" && w["scored"] == true && w["units"] == "m", "wall entry %s", w.dump().c_str());
    CHECK(w["file"].get<std::string>().find('\\') == std::string::npos, "backslash in path");
    CHECK(w["transform"]["rotation"].size() == 9 && w["transform"]["translation"][0] == 250.0, "wall transform");
    const json& d = j["surfaces"][1];
    CHECK(d["scored"] == false && d["meta"]["role"] == "drain" && d["units"] == "mm", "drain entry %s", d.dump().c_str());
    const json& nz = j["nozzles"][0];
    CHECK(nz["name"] == "top_spray" && near(nz["position"][1].get<double>(), 1280, 1e-9), "nozzle %s", nz.dump().c_str());
    CHECK(nz["half_angle_deg"].get<double>() == 70.0 && nz["mdot_kg_s"].get<double>() == 0.25, "nozzle params");
    CHECK(nz["pressure_bar"].get<double>() == 2.0, "pressure");
    SimulationSetup thirty = s;
    thirty.nozzles[0].mdotKgS = 0.3f;
    CHECK(buildTier1Spec(thirty, fx.history, out, warn, err) && parse(out)["nozzles"][0]["mdot_kg_s"].get<double>() == 0.3,
          "0.3f written as %s", parse(out)["nozzles"][0]["mdot_kg_s"].dump().c_str());

    // Identity placement writes no transform at all.
    FeatureHistory plain;
    MeshImportFeatureData pd;
    pd.sourcePath = (fx.dir / "vessel_wall.stl").string();
    pd.unit = "m";
    plain.addMeshImportFeature(pd, "w");
    SimulationSetup ps;
    SimNozzle pn;
    pn.position[2] = 1000;
    ps.addNozzle(pn);
    CHECK(buildTier1Spec(ps, plain, out, warn, err) && !parse(out)["surfaces"][0].contains("transform"),
          "identity transform written");
    CHECK(parse(out)["nozzles"][0]["name"] == "nozzle1", "default nozzle name");
}

static void testSaveLoad() {
    std::printf("save / load\n");
    Fixture fx = makeFixture();
    auto planes = referencePlanes();
    SimulationSetup s;
    s.setRole(fx.drain, SurfaceRole::Drain);
    s.rays = 123456;
    s.bounces = 3;
    SimNozzle n;
    n.name = "a";
    n.hostFeature = fx.wall;
    n.position[0] = 1.25;
    n.axis[2] = 1.0;
    n.axis[1] = 0.0;
    n.halfAngleDeg = 180.0f;
    s.addNozzle(n);
    s.addNozzle(SimNozzle{});

    const fs::path p = fx.dir / "sim.shitcad";
    CHECK(saveProject(p.string(), fx.history, planes, &s), "%s", lastLoadError().c_str());
    FeatureHistory h2;
    std::vector<SketchPlane> pl2;
    SimulationSetup s2;
    CHECK(loadProject(p.string(), h2, pl2, &s2), "%s", lastLoadError().c_str());
    CHECK(sameSimulation(s, s2), "simulation changed across save/load");
    CHECK(s2.nextNozzleID == 3, "nextNozzleID=%u", s2.nextNozzleID);

    // A simulation alone (no imports) still forces version 2.
    FeatureHistory empty;
    SimulationSetup only;
    only.addNozzle(SimNozzle{});
    const fs::path p2 = fx.dir / "only.shitcad";
    CHECK(saveProject(p2.string(), empty, planes, &only), "%s", lastLoadError().c_str());
    {
        std::ifstream in(p2);
        json doc = json::parse(in);
        CHECK(doc["version"] == 2 && doc.contains("simulation"), "sim-only project version=%d", doc["version"].get<int>());
    }
    // Empty set-up: no block, version 1, loads as empty.
    const fs::path p3 = fx.dir / "none.shitcad";
    SimulationSetup none;
    CHECK(saveProject(p3.string(), empty, planes, &none), "%s", lastLoadError().c_str());
    {
        std::ifstream in(p3);
        json doc = json::parse(in);
        CHECK(doc["version"] == 1 && !doc.contains("simulation"), "empty set-up written");
    }
    SimulationSetup loadedNone;
    loadedNone.addNozzle(SimNozzle{}); // must be overwritten with empty
    FeatureHistory h3;
    std::vector<SketchPlane> pl3;
    CHECK(loadProject(p3.string(), h3, pl3, &loadedNone) && loadedNone.empty(), "stale set-up survived loading");

    // Bad role name is rejected.
    {
        std::ifstream in(p);
        json doc = json::parse(in);
        doc["simulation"]["roles"][0]["role"] = "gutter";
        const fs::path pb = fx.dir / "badrole.shitcad";
        std::ofstream(pb) << doc.dump();
        SimulationSetup sb;
        FeatureHistory hb;
        std::vector<SketchPlane> plb;
        CHECK(!loadProject(pb.string(), hb, plb, &sb), "unknown role accepted");
    }
}

// Place a nozzle the way a click does, export, and leave files for cip-sim.
static void writeCrossCheck(const fs::path& outDir) {
    std::printf("cross-check bundle for cip-sim\n");
    Fixture fx = makeFixture();
    Scene3D scene;
    auto planes = referencePlanes();
    replayFeatures(fx.history, planes, scene);
    CHECK(scene.bodyCount() == 2, "bodies=%zu", scene.bodyCount());

    // Click from above onto the top of the upright vessel.
    const float origin[3] = {250.0f, 5000.0f, -100.0f}, down[3] = {0, -1, 0};
    MeshPickResult hit = pickMesh(scene, origin, down);
    CHECK(hit.hit && near(hit.hitWorld[1], 1300, 1e-2), "pick top: hit=%d y=%g", hit.hit, hit.hitWorld[1]);
    CHECK(hit.frontFacing && near(hit.normal[1], 1.0, 1e-5), "top normal (%g, %g, %g)",
          hit.normal[0], hit.normal[1], hit.normal[2]);

    const double standoff = 20.0;
    const double inward[3] = {-hit.normal[0], -hit.normal[1], -hit.normal[2]};
    const double pos[3] = {hit.hitWorld[0] + inward[0] * standoff, hit.hitWorld[1] + inward[1] * standoff,
                           hit.hitWorld[2] + inward[2] * standoff};
    SimulationSetup s;
    s.setRole(fx.drain, SurfaceRole::Drain);
    SimNozzle n;
    n.name = "top_spray";
    n.hostFeature = scene.getBody(hit.bodyIndex).sourceFeature;
    setNozzleWorld(n, fx.history, pos, inward);
    s.addNozzle(n);
    s.rays = 20000;
    s.bounces = 1;

    std::string spec, err;
    std::vector<std::string> warn;
    CHECK(buildTier1Spec(s, fx.history, spec, warn, err), "%s", err.c_str());

    // Expected placement, from what SHITcad actually draws.
    json expected;
    expected["surfaces"] = json::object();
    for (int i = 0; i < (int)scene.bodyCount(); i++) {
        const Body3D& b = scene.getBody(i);
        float lo[3] = {1e30f, 1e30f, 1e30f}, hi[3] = {-1e30f, -1e30f, -1e30f};
        for (const auto& v : b.vertices) {
            const float p[3] = {v.px, v.py, v.pz};
            for (int k = 0; k < 3; k++) { lo[k] = std::min(lo[k], p[k]); hi[k] = std::max(hi[k], p[k]); }
        }
        const Feature* f = fx.history.findFeature(b.sourceFeature);
        expected["surfaces"][f->name] = {{"min_mm", {lo[0], lo[1], lo[2]}}, {"max_mm", {hi[0], hi[1], hi[2]}}};
    }
    expected["nozzle_mm"] = {pos[0], pos[1], pos[2]};
    expected["axis"] = {inward[0], inward[1], inward[2]};

    fs::create_directories(outDir);
    std::ofstream(outDir / "spec.json") << spec << "\n";
    std::ofstream(outDir / "expected.json") << expected.dump(2) << "\n";
    std::printf("  wrote %s\n", (outDir / "spec.json").string().c_str());
}

// ---- running the engine -------------------------------------------------------

static bool runToCompletion(ProcessRunner& r, std::vector<std::string>& lines, double timeoutS) {
    auto t0 = std::chrono::steady_clock::now();
    while (!r.finished()) {
        r.poll(lines);
        if (std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count() > timeoutS) return false;
        std::this_thread::sleep_for(std::chrono::milliseconds(15)); // a frame, roughly
    }
    return true;
}

static void testProcessRunner(const std::string& python) {
    std::printf("process runner\n");
    CHECK(ProcessRunner::quoteArg("plain") == "plain", "plain");
    CHECK(ProcessRunner::quoteArg("a b") == "\"a b\"", "space");
    CHECK(ProcessRunner::quoteArg("C:\\dir with space\\") == "\"C:\\dir with space\\\\\"", "trailing backslash: %s",
          ProcessRunner::quoteArg("C:\\dir with space\\").c_str());
    CHECK(ProcessRunner::quoteArg("say \"hi\"") == "\"say \\\"hi\\\"\"", "quote: %s", ProcessRunner::quoteArg("say \"hi\"").c_str());

    // Lines arrive while running, a flood of stderr does not deadlock, the exit
    // code comes back, and awkward arguments survive quoting.
    const std::string awkward = "C:\\path with space\\ and \"quotes\"\\";
    const std::string script =
        "import sys, json, time\n"
        "for i in range(3):\n"
        "    print(json.dumps({'event': 'progress', 'i': i}), flush=True); time.sleep(0.05)\n"
        "sys.stderr.write('x' * (1 << 20)); sys.stderr.flush()\n"
        "print(json.dumps({'event': 'result', 'arg': sys.argv[1]}))\n"
        "sys.exit(3)\n";
    ProcessRunner r;
    std::string err;
    CHECK(r.start({python, "-c", script, awkward}, "", true, err), "start: %s", err.c_str());
    std::vector<std::string> lines;
    CHECK(runToCompletion(r, lines, 60), "did not finish (deadlocked on stderr?)");
    CHECK(lines.size() == 4, "lines=%zu", lines.size());
    CHECK(r.exitCode() == 3, "exit=%d", r.exitCode());
    CHECK(r.stderrTail().size() <= 16 * 1024 && r.stderrTail().size() > 1000, "stderr tail=%zu", r.stderrTail().size());
    if (lines.size() == 4) {
        CHECK(eventKind(lines[0]) == "progress", "first event %s", lines[0].c_str());
        CHECK(json::parse(lines[3])["arg"] == awkward, "argument mangled: %s", lines[3].c_str());
    }

    ProcessRunner sleeper;
    CHECK(sleeper.start({python, "-c", "import time; time.sleep(60)"}, "", true, err), "%s", err.c_str());
    std::vector<std::string> none;
    sleeper.poll(none);
    CHECK(sleeper.running(), "sleeper not running");
    auto t0 = std::chrono::steady_clock::now();
    sleeper.cancel();
    CHECK(sleeper.finished() && std::chrono::steady_clock::now() - t0 < std::chrono::seconds(5), "cancel did not stop it");

    ProcessRunner missing;
    err.clear();
    CHECK(!missing.start({"definitely_not_a_program_xyz.exe"}, "", true, err) && err.find("not found") != std::string::npos,
          "missing exe: %s", err.c_str());
}

static void testResultLoaderErrors(const fs::path& dir) {
    std::printf("result loader errors\n");
    ResultMesh m;
    std::string err;
    std::ofstream(dir / "notours.json") << R"({"format": "something-else", "version": 1})";
    CHECK(!loadResultMesh((dir / "notours.json").string(), m, err) && err.find("Not a cip-sim") != std::string::npos,
          "%s", err.c_str());
    std::ofstream(dir / "trunc.bin", std::ios::binary) << "abcd";
    std::ofstream(dir / "trunc.json") << R"({"format":"cipsim-trimesh","version":1,"units":"m","triangles":1,
        "positions":{"offset":0,"count":9},"fields":[],"bin":"trunc.bin","bytes":36})";
    err.clear();
    CHECK(!loadResultMesh((dir / "trunc.json").string(), m, err) && err.find("truncated") != std::string::npos,
          "%s", err.c_str());
}

// Run Tier 1 on the cross-check spec exactly as the Run button does.
static void testEndToEnd(const fs::path& bundle, const std::string& cipSim, const std::string& python) {
    std::printf("end to end: SHITcad spec -> cip-sim -> results on the geometry\n");
    const fs::path out = bundle / "run";
    fs::remove_all(out);
    ProcessRunner r;
    std::string err;
    CHECK(r.start({python, "-u", "-m", "cipsim.cli", "tier1", "--spec", (bundle / "spec.json").string(),
                   "--out", out.string(), "--rays", "20000", "--bounces", "1"}, cipSim, true, err),
          "start: %s", err.c_str());
    std::vector<std::string> lines;
    CHECK(runToCompletion(r, lines, 600), "tier1 did not finish");
    CHECK(r.exitCode() == 0, "exit=%d stderr=%s", r.exitCode(), r.stderrTail().c_str());

    int progress = 0;
    RunSummary summary;
    bool got = false;
    for (const auto& l : lines) {
        const std::string k = eventKind(l);
        CHECK(!k.empty(), "non-JSON stdout line: %s", l.c_str());
        if (k == "progress") progress++;
        if (k == "result") got = parseResultEvent(l, summary, err);
    }
    CHECK(progress > 0, "no progress events");
    CHECK(got, "no result: %s", err.c_str());
    if (!got) return;
    CHECK(summary.surfaces.size() == 2 && summary.overall.directPct > 50.0, "summary direct=%g", summary.overall.directPct);

    ResultMesh mesh;
    CHECK(loadResultMesh(summary.viewerJson, mesh, err), "%s", err.c_str());
    CHECK(mesh.triangles == 24, "triangles=%zu (two 12-triangle boxes)", mesh.triangles);

    // The results must sit exactly on the geometry SHITcad drew.
    std::ifstream ef(bundle / "expected.json");
    json expected = json::parse(ef);
    float lo[3] = {1e30f, 1e30f, 1e30f}, hi[3] = {-1e30f, -1e30f, -1e30f};
    for (const auto& [name, s] : expected["surfaces"].items()) {
        for (int k = 0; k < 3; k++) {
            lo[k] = std::min(lo[k], s["min_mm"][k].get<float>());
            hi[k] = std::max(hi[k], s["max_mm"][k].get<float>());
        }
    }
    for (int k = 0; k < 3; k++) {
        CHECK(near(mesh.boundsMin[k], lo[k], 0.01) && near(mesh.boundsMax[k], hi[k], 0.01),
              "axis %d: results [%g, %g] vs geometry [%g, %g]", k, mesh.boundsMin[k], mesh.boundsMax[k], lo[k], hi[k]);
    }

    int reach = mesh.fieldIndex("reach");
    CHECK(reach >= 0, "no reach field");
    if (reach < 0) return;
    const ResultField& f = mesh.fields[reach];
    CHECK(f.categorical && f.colours.size() == 3 && f.labels.size() == 3, "reach metadata");
    std::vector<float> rgb;
    colourByField(mesh, reach, rgb);
    bool coloursMatch = true;
    for (size_t t = 0; t < mesh.triangles; t++) {
        int k = (int)std::lround(f.values[t]);
        coloursMatch = coloursMatch && k >= 0 && k <= 2 && rgb[t * 3] == f.colours[k][0] && rgb[t * 3 + 2] == f.colours[k][2];
    }
    CHECK(coloursMatch, "triangle colours do not follow the reach categories");
    int flux = mesh.fieldIndex("total_flux");
    CHECK(flux >= 0 && !mesh.fields[flux].categorical && mesh.fields[flux].p95 > 0, "flux field");
}


static void testSectionPlane() {
    std::printf("section plane\n");
    SectionPlane sp;
    const float inside[3] = {0, -10, 0}, outside[3] = {0, 10, 0};
    CHECK(!sp.cuts(inside) && !sp.cuts(outside), "disabled plane cuts");

    sp.enabled = true;
    sp.axis = 1;      // Y, the up axis here
    sp.position = 0.0f;
    CHECK(!sp.cuts(inside) && sp.cuts(outside), "Y plane keeps the wrong half");

    sp.flip = true;
    CHECK(sp.cuts(inside) && !sp.cuts(outside), "flip did not swap the halves");
    float n[3];
    sp.normal(n);
    CHECK(n[1] == -1.0f && sp.offset() == 0.0f, "flipped normal (%g, %g, %g) offset %g", n[0], n[1], n[2], sp.offset());

    // Offset plane: a point exactly on it is kept, past it is cut.
    sp.flip = false;
    sp.position = 500.0f;
    const float on[3] = {0, 500, 0}, past[3] = {0, 500.1f, 0};
    CHECK(!sp.cuts(on) && sp.cuts(past), "offset plane boundary");

    sp.axis = 0;
    const float x[3] = {600, 0, 0};
    CHECK(sp.cuts(x), "X axis ignored");
    sp.axis = 2;
    CHECK(!sp.cuts(x), "Z plane cut on X");
}

int main(int argc, char** argv) {
    testSectionPlane();
    testRoles();
    testNozzleFollowsHost();
    testSpecExportErrors();
    testSpecContent();
    testSaveLoad();

    const fs::path crossDir = argc > 1 ? fs::path(argv[1]) : fs::temp_directory_path() / "shitcad_sim_crosscheck";
    if (!glfwInit()) {
        std::printf("glfwInit failed\n");
        g_failures++;
    } else {
        glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);
        glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
        glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
        glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
        GLFWwindow* win = glfwCreateWindow(64, 64, "test", nullptr, nullptr);
        if (!win) {
            std::printf("no GL context\n");
            g_failures++;
        } else {
            glfwMakeContextCurrent(win);
            gladLoadGL(glfwGetProcAddress);
            writeCrossCheck(crossDir);
            {
                // The mesh and line shaders gained clip uniforms; a typo there
                // would only show as a blank viewport at run time.
                std::printf("viewport shaders compile\n");
                Viewport3D vp;
                CHECK(vp.init(), "viewport shaders failed to compile");
                vp.shutdown();
            }
            {
                std::printf("result shader compiles\n");
                ShaderProgram sp;
                CHECK(sp.compile(kResultVertSrc, kResultFragSrc) && sp.id() != 0, "result shader failed to compile");
            }
            glfwDestroyWindow(win);
        }
        glfwTerminate();
    }

    // Engine tests need Python (and, for end to end, the cip-sim repo):
    //   SimulationTest.exe <bundle_dir> <cip-sim dir> [python]
    const std::string python = argc > 3 ? argv[3] : "python";
    testResultLoaderErrors(fs::temp_directory_path() / "shitcad_simulation_test");
    if (argc > 2) {
        testProcessRunner(python);
        testEndToEnd(crossDir, argv[2], python);
    } else {
        std::printf("(engine tests skipped: pass <bundle_dir> <cip-sim dir> [python])\n");
    }

    std::printf("\n%d checks, %d failed\n", g_checks, g_failures);
    return g_failures == 0 ? 0 : 1;
}
