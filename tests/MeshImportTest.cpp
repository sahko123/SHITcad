// Tests for STL reference import: unit scaling, caching, picking, save/load,
// and replay alongside OCCT features.
//
// Build with -DSHITCAD_BUILD_TESTS=ON, run build/Release/MeshImportTest.exe.
// Runs without a GL context on purpose: Scene3D uploads lazily at render
// time, so replay must not call OpenGL (a stray call would crash here).

#include "FacePicker.h"
#include "FeatureHistory.h"
#include "FeatureReplay.h"
#include "MeshImport.h"
#include "ProfileDetector.h"
#include "Scene3D.h"
#include "Serialization.h"
#include "SketchPlane.h"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

using namespace shitcad;
namespace fs = std::filesystem;

static int g_failures = 0;
static int g_checks = 0;

#define CHECK(cond, ...)                                              \
    do {                                                              \
        g_checks++;                                                   \
        if (!(cond)) {                                                \
            g_failures++;                                             \
            std::printf("  FAIL %s:%d: %s -- ", __FILE__, __LINE__, #cond); \
            std::printf(__VA_ARGS__);                                 \
            std::printf("\n");                                        \
        }                                                             \
    } while (0)

static bool near(double a, double b, double tol) { return std::fabs(a - b) <= tol; }

// ---- fixtures ---------------------------------------------------------------

struct Tri { float v[9]; };

// Closed box with outward-facing (counter-clockwise from outside) triangles.
static std::vector<Tri> boxTriangles(float x0, float y0, float z0, float x1, float y1, float z1) {
    const float c[8][3] = {
        {x0, y0, z0}, {x1, y0, z0}, {x1, y1, z0}, {x0, y1, z0},
        {x0, y0, z1}, {x1, y0, z1}, {x1, y1, z1}, {x0, y1, z1},
    };
    const int q[6][4] = {
        {0, 3, 2, 1}, // bottom  -z
        {4, 5, 6, 7}, // top     +z
        {0, 1, 5, 4}, // front   -y
        {2, 3, 7, 6}, // back    +y
        {1, 2, 6, 5}, // right   +x
        {3, 0, 4, 7}, // left    -x
    };
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
    std::snprintf(header, sizeof(header), "MeshImportTest");
    f.write(header, 80);
    uint32_t n = (uint32_t)tris.size();
    f.write((const char*)&n, 4);
    for (const auto& t : tris) {
        const float zero[3] = {0, 0, 0}; // readers recompute normals
        f.write((const char*)zero, 12);
        f.write((const char*)t.v, 36);
        uint16_t attr = 0;
        f.write((const char*)&attr, 2);
    }
}

static std::vector<SketchPlane> referencePlanes() {
    std::vector<SketchPlane> planes(3);
    const float normals[3][3] = {{0, 0, 1}, {0, 1, 0}, {1, 0, 0}};
    const float us[3][3] = {{1, 0, 0}, {1, 0, 0}, {0, 1, 0}};
    const float vs[3][3] = {{0, 1, 0}, {0, 0, 1}, {0, 0, 1}};
    for (int i = 0; i < 3; i++) {
        planes[i].planeID = (PlaneID)(i + 1);
        std::memcpy(planes[i].normal, normals[i], sizeof(float) * 3);
        std::memcpy(planes[i].uAxis, us[i], sizeof(float) * 3);
        std::memcpy(planes[i].vAxis, vs[i], sizeof(float) * 3);
        planes[i].isReferencePlane = true;
    }
    return planes;
}

static Sketch rectangleSketch(double x0, double y0, double x1, double y1) {
    Sketch s;
    EntityID a = s.addPoint(x0, y0), b = s.addPoint(x1, y0);
    EntityID c = s.addPoint(x1, y1), d = s.addPoint(x0, y1);
    s.addLine(a, b); s.addLine(b, c); s.addLine(c, d); s.addLine(d, a);
    return s;
}

static int countMeshBodies(const Scene3D& scene) {
    int n = 0;
    for (int i = 0; i < (int)scene.bodyCount(); i++) n += scene.getBody(i).isMeshOnly() ? 1 : 0;
    return n;
}

// ---- tests ------------------------------------------------------------------

static void testLoadAndUnits(const fs::path& box) {
    std::printf("load and units\n");
    MeshFileInfo info;
    std::string err;
    CHECK(probeMeshFile(box.string(), info, err), "%s", err.c_str());
    CHECK(info.triangleCount == 12, "triangles=%zu", info.triangleCount);
    CHECK(near(info.rawMax[2] - info.rawMin[2], 1.3, 1e-6), "raw height=%g", info.rawMax[2] - info.rawMin[2]);

    std::vector<MeshVertex> mm;
    CHECK(loadMeshFile(box.string(), "m", mm, info, err), "%s", err.c_str());
    float zmax = -1e30f;
    for (const auto& v : mm) zmax = std::max(zmax, v.pz);
    CHECK(near(zmax, 1300.0, 1e-3), "m -> mm: zmax=%g", zmax);

    CHECK(loadMeshFile(box.string(), "in", mm, info, err), "%s", err.c_str());
    zmax = -1e30f;
    for (const auto& v : mm) zmax = std::max(zmax, v.pz);
    CHECK(near(zmax, 1.3 * 25.4, 1e-3), "in -> mm: zmax=%g", zmax);

    // Normals must survive scaling unchanged: the top face points +z.
    bool sawUp = false;
    for (const auto& v : mm) if (near(v.nz, 1.0, 1e-6) && near(v.pz, 1.3 * 25.4, 1e-3)) sawUp = true;
    CHECK(sawUp, "top face normal lost");

    err.clear();
    CHECK(!loadMeshFile(box.string(), "furlong", mm, info, err), "unknown unit accepted");
    CHECK(err.find("Unknown unit") != std::string::npos, "err=%s", err.c_str());

    err.clear();
    CHECK(!loadMeshFile((box.parent_path() / "missing.stl").string(), "mm", mm, info, err), "missing file accepted");
    CHECK(err.find("not found") != std::string::npos, "err=%s", err.c_str());
}

static void testCacheInvalidation(const fs::path& dir) {
    std::printf("cache picks up a re-exported file\n");
    const fs::path p = dir / "reexport.stl";
    writeBinaryStl(p, boxTriangles(0, 0, 0, 1, 1, 1));

    MeshFileInfo info;
    std::string err;
    CHECK(probeMeshFile(p.string(), info, err), "%s", err.c_str());
    CHECK(near(info.rawMax[0], 1.0, 1e-6), "first read xmax=%g", info.rawMax[0]);

    // Simulate a re-export: new geometry, later timestamp.
    auto before = fs::last_write_time(p);
    writeBinaryStl(p, boxTriangles(0, 0, 0, 2, 1, 1));
    fs::last_write_time(p, before + std::chrono::seconds(5));

    CHECK(probeMeshFile(p.string(), info, err), "%s", err.c_str());
    CHECK(near(info.rawMax[0], 2.0, 1e-6), "stale cache: xmax=%g after re-export", info.rawMax[0]);
}

static void testPick(const fs::path& box) {
    std::printf("pick\n");
    Scene3D scene;
    Body3D body;
    MeshFileInfo info;
    std::string err;
    CHECK(loadMeshFile(box.string(), "m", body.vertices, info, err), "%s", err.c_str());
    body.sourceFeature = 42;
    scene.addMeshBody(std::move(body)); // no upload: picking reads CPU vertices only

    // From above, straight down onto the top face: outside, so front-facing.
    const float o1[3] = {100, 50, 5000}, d1[3] = {0, 0, -1};
    MeshPickResult r = pickMesh(scene, o1, d1);
    CHECK(r.hit, "missed the top face");
    CHECK(near(r.hitWorld[2], 1300.0, 1e-2), "hit z=%g", r.hitWorld[2]);
    CHECK(near(r.hitWorld[0], 100.0, 1e-2) && near(r.hitWorld[1], 50.0, 1e-2), "hit xy=(%g,%g)", r.hitWorld[0], r.hitWorld[1]);
    CHECK(near(r.normal[2], 1.0, 1e-5), "normal z=%g", r.normal[2]);
    CHECK(r.frontFacing, "outside hit reported as back-facing");
    CHECK(near(r.t, 3700.0, 1e-2), "t=%g", r.t);

    // From inside the vessel, upward: same face, seen from the back.
    const float o2[3] = {100, 50, 500}, d2[3] = {0, 0, 1};
    r = pickMesh(scene, o2, d2);
    CHECK(r.hit && near(r.hitWorld[2], 1300.0, 1e-2), "inside-up hit z=%g", r.hitWorld[2]);
    CHECK(!r.frontFacing, "inside hit reported as front-facing");

    // Nearest hit wins: horizontal ray through both side walls.
    const float o3[3] = {-5000, 0, 650}, d3[3] = {1, 0, 0};
    r = pickMesh(scene, o3, d3);
    CHECK(r.hit && near(r.hitWorld[0], -300.0, 1e-2), "nearest wall x=%g", r.hitWorld[0]);

    const float o4[3] = {1000, 1000, 5000};
    r = pickMesh(scene, o4, d1);
    CHECK(!r.hit, "hit outside the footprint");

    scene.getBodyMut(0).visible = false;
    r = pickMesh(scene, o1, d1);
    CHECK(!r.hit, "hidden body was picked");
}

static void testPlacement(const fs::path& box) {
    std::printf("placement\n");
    MeshFileInfo info;
    std::string err;
    CHECK(probeMeshFile(box.string(), info, err), "%s", err.c_str());

    // Quarter turns are exact: no 6e-17 residue anywhere in the matrix.
    double q[9];
    axisRotation(0, 90.0, q);
    const double rx90[9] = {1, 0, 0, 0, 0, -1, 0, 1, 0};
    bool exact = true;
    for (int i = 0; i < 9; i++) exact = exact && q[i] == rx90[i];
    CHECK(exact, "Rx(90) not exact: %g %g %g / %g %g %g / %g %g %g", q[0], q[1], q[2], q[3], q[4], q[5], q[6], q[7], q[8]);
    axisRotation(2, -270.0, q);
    const double rz90[9] = {0, -1, 0, 1, 0, 0, 0, 0, 1};
    exact = true;
    for (int i = 0; i < 9; i++) exact = exact && q[i] == rz90[i];
    CHECK(exact, "Rz(-270) should equal Rz(90) exactly");

    // Box 600 x 600 x 1300 mm standing on z = 0. Lay it on its side about its
    // centre: the 1300 dimension moves to Y and the centre does not move.
    MeshTransform xf;
    double lo[3], hi[3];
    placedBounds(info, 1000.0f, xf, lo, hi);
    const double c0[3] = {(lo[0] + hi[0]) / 2, (lo[1] + hi[1]) / 2, (lo[2] + hi[2]) / 2};
    axisRotation(0, 90.0, q);
    rotateAbout(xf, q, c0);
    placedBounds(info, 1000.0f, xf, lo, hi);
    CHECK(near(hi[1] - lo[1], 1300.0, 1e-3) && near(hi[2] - lo[2], 600.0, 1e-3),
          "after Rx90 size Y=%g Z=%g", hi[1] - lo[1], hi[2] - lo[2]);
    CHECK(near((lo[2] + hi[2]) / 2, c0[2], 1e-3) && near((lo[1] + hi[1]) / 2, c0[1], 1e-3),
          "rotation moved the centre to (%g, %g)", (lo[1] + hi[1]) / 2, (lo[2] + hi[2]) / 2);

    // Four quarter turns about the same axis return to identity exactly.
    MeshTransform round;
    for (int i = 0; i < 4; i++) rotateAbout(round, rx90, c0);
    CHECK(round.isIdentity(), "4 x Rx90 is not identity: t=(%g, %g, %g)", round.t[0], round.t[1], round.t[2]);

    // Order matters and is preserved: X then Z differs from Z then X.
    MeshTransform xz, zx;
    const double origin[3] = {0, 0, 0};
    rotateAbout(xz, rx90, origin); rotateAbout(xz, rz90, origin);
    rotateAbout(zx, rz90, origin); rotateAbout(zx, rx90, origin);
    bool differ = false;
    for (int i = 0; i < 9; i++) differ = differ || xz.r[i] != zx.r[i];
    CHECK(differ, "rotation order was lost");

    // Drop to ground puts the lowest point at z = 0.
    xf.t[2] -= lo[2];
    placedBounds(info, 1000.0f, xf, lo, hi);
    CHECK(near(lo[2], 0.0, 1e-6), "bottom after drop = %g", lo[2]);

    // Vertices and normals: the top face (+z normal) now faces -y.
    std::vector<MeshVertex> v;
    CHECK(loadMeshFile(box.string(), "m", v, info, err), "%s", err.c_str());
    applyMeshTransform(v, xf);
    bool sawMinusY = false;
    float vmin = 1e30f;
    for (const auto& p : v) {
        if (near(p.ny, -1.0, 1e-6) && near(p.nz, 0.0, 1e-6)) sawMinusY = true;
        vmin = std::min(vmin, p.pz);
    }
    CHECK(sawMinusY, "top-face normal was not rotated to -y");
    CHECK(near(vmin, 0.0, 1e-3), "vertex min z = %g", vmin);
}

static void testSaveLoad(const fs::path& dir, const fs::path& box) {
    std::printf("save / load\n");
    auto planes = referencePlanes();

    FeatureHistory plain;
    plain.addSketchFeature(0, rectangleSketch(0, 0, 10, 10), 1);
    const fs::path p1 = dir / "plain.shitcad";
    CHECK(saveProject(p1.string(), plain, planes), "%s", lastLoadError().c_str());
    {
        std::ifstream in(p1);
        auto doc = nlohmann::json::parse(in);
        CHECK(doc["version"] == 1, "project without imports must stay version 1, got %d", doc["version"].get<int>());
    }

    FeatureHistory h;
    MeshImportFeatureData md;
    md.sourcePath = box.string();
    md.unit = "m";
    FeatureID fid = h.addMeshImportFeature(md, "vessel_wall");
    const fs::path p2 = dir / "withimport.shitcad";
    CHECK(saveProject(p2.string(), h, planes), "%s", lastLoadError().c_str());
    {
        std::ifstream in(p2);
        auto doc = nlohmann::json::parse(in);
        CHECK(doc["version"] == 2, "project with an import must be version 2, got %d", doc["version"].get<int>());
    }

    FeatureHistory loaded;
    std::vector<SketchPlane> loadedPlanes;
    CHECK(loadProject(p2.string(), loaded, loadedPlanes), "%s", lastLoadError().c_str());
    const Feature* f = loaded.findFeature(fid);
    CHECK(f && f->type == FeatureType::MeshImport, "import feature lost on load");
    if (f && f->type == FeatureType::MeshImport) {
        const auto& lmd = std::get<MeshImportFeatureData>(f->data);
        CHECK(lmd.sourcePath == md.sourcePath, "path=%s", lmd.sourcePath.c_str());
        CHECK(lmd.unit == "m", "unit=%s", lmd.unit.c_str());
        CHECK(f->name == "vessel_wall", "name=%s", f->name.c_str());
    }

    // Imports saved before placement existed have no "transform": identity.
    CHECK(f && std::get<MeshImportFeatureData>(f->data).transform.isIdentity(), "untransformed import not identity");
    {
        std::ifstream in(p2);
        auto doc = nlohmann::json::parse(in);
        CHECK(!doc["featureHistory"]["features"][0]["data"].contains("transform"),
              "identity transform should not be written");
    }

    // Placement round-trips bit-exactly.
    FeatureHistory hx;
    MeshImportFeatureData mdx = md;
    const double rx90[9] = {1, 0, 0, 0, 0, -1, 0, 1, 0};
    for (int i = 0; i < 9; i++) mdx.transform.r[i] = rx90[i];
    mdx.transform.t[0] = 12.5; mdx.transform.t[1] = -300.0; mdx.transform.t[2] = 0.1;
    FeatureID fx = hx.addMeshImportFeature(mdx, "placed");
    const fs::path px = dir / "placed.shitcad";
    CHECK(saveProject(px.string(), hx, planes), "%s", lastLoadError().c_str());
    FeatureHistory lx;
    std::vector<SketchPlane> lxPlanes;
    CHECK(loadProject(px.string(), lx, lxPlanes), "%s", lastLoadError().c_str());
    const Feature* fxl = lx.findFeature(fx);
    if (fxl) {
        const auto& t = std::get<MeshImportFeatureData>(fxl->data).transform;
        bool same = true;
        for (int i = 0; i < 9; i++) same = same && t.r[i] == rx90[i];
        same = same && t.t[0] == 12.5 && t.t[1] == -300.0 && t.t[2] == 0.1;
        CHECK(same, "placement changed across save/load");
    }

    // A malformed transform is rejected, not half-applied.
    {
        std::ifstream in(px);
        auto doc = nlohmann::json::parse(in);
        doc["featureHistory"]["features"][0]["data"]["transform"]["rotation"] = {1, 0, 0};
        const fs::path pb = dir / "badxf.shitcad";
        std::ofstream(pb) << doc.dump(2);
        FeatureHistory bad;
        std::vector<SketchPlane> badPlanes;
        CHECK(!loadProject(pb.string(), bad, badPlanes), "3-element rotation was accepted");
    }

    // A unit-less import entry is rejected rather than defaulted.
    {
        std::ifstream in(p2);
        auto doc = nlohmann::json::parse(in);
        doc["featureHistory"]["features"][0]["data"].erase("unit");
        const fs::path p3 = dir / "nounit.shitcad";
        std::ofstream(p3) << doc.dump(2);
        FeatureHistory bad;
        std::vector<SketchPlane> badPlanes;
        CHECK(!loadProject(p3.string(), bad, badPlanes), "import without a unit was accepted");
    }
}

static void testReplay(const fs::path& dir, const fs::path& box) {
    std::printf("replay\n");
    auto planes = referencePlanes();
    Scene3D scene;
    FeatureHistory h;

    MeshImportFeatureData md;
    md.sourcePath = box.string();
    md.unit = "m";
    FeatureID meshID = h.addMeshImportFeature(md, "vessel_wall");

    // An OCCT solid created after the mesh: the NewBody auto-fuse loop walks
    // every existing body, including the shape-less mesh.
    Sketch rect = rectangleSketch(2000, 2000, 2040, 2020);
    FeatureID sk = h.addSketchFeature(0, rect, planes[0].planeID);
    planes[0].sketch = rect;
    auto profiles = detectClosedProfiles(rect, planes[0]);
    CHECK(profiles.size() == 1, "profiles=%zu", profiles.size());

    ExtrudeFeatureData ed;
    ed.sourceSketchFeature = sk;
    if (!profiles.empty()) {
        ed.profileSigs.push_back(ProfileSignature::fromProfile(profiles[0], rect));
        ed.profileIndicesFallback.push_back(0);
    }
    ed.height = 50.0f;
    h.addExtrudeFeature(ed);

    replayFeatures(h, planes, scene);
    CHECK(scene.bodyCount() == 2, "bodies=%zu", scene.bodyCount());
    CHECK(countMeshBodies(scene) == 1, "mesh bodies=%d", countMeshBodies(scene));
    for (const auto& f : h.features()) CHECK(!f.hasError, "%s: %s", f.name.c_str(), f.errorMsg.c_str());
    if (scene.bodyCount() >= 1) {
        CHECK(scene.getBody(0).sourceFeature == meshID, "mesh body not tagged with its feature");
        CHECK(scene.getBody(0).vertexCount == 36, "vertexCount=%d", scene.getBody(0).vertexCount);
    }

    // Placement is applied on replay: stand the box on its side and lift it.
    {
        FeatureHistory hp;
        MeshImportFeatureData mp = md;
        axisRotation(1, 90.0, mp.transform.r); // about origin: x' = z, z' = -x
        mp.transform.t[2] = 1000.0;
        hp.addMeshImportFeature(mp, "placed");
        Scene3D sp;
        auto planesP = referencePlanes();
        replayFeatures(hp, planesP, sp);
        CHECK(sp.bodyCount() == 1, "bodies=%zu", sp.bodyCount());
        if (sp.bodyCount() == 1) {
            float lo[3] = {1e30f, 1e30f, 1e30f}, hi[3] = {-1e30f, -1e30f, -1e30f};
            for (const auto& v : sp.getBody(0).vertices) {
                const float p[3] = {v.px, v.py, v.pz};
                for (int k = 0; k < 3; k++) { lo[k] = std::min(lo[k], p[k]); hi[k] = std::max(hi[k], p[k]); }
            }
            CHECK(near(lo[0], 0.0, 1e-2) && near(hi[0], 1300.0, 1e-2), "replayed x range [%g, %g]", lo[0], hi[0]);
            CHECK(near(lo[2], 700.0, 1e-2) && near(hi[2], 1300.0, 1e-2), "replayed z range [%g, %g]", lo[2], hi[2]);
        }
    }

    // Replaying again (what every edit does) must not duplicate or drop it.
    replayFeatures(h, planes, scene);
    CHECK(scene.bodyCount() == 2 && countMeshBodies(scene) == 1, "second replay: bodies=%zu", scene.bodyCount());

    // A cut walks every body too. The invariant is not a particular OCCT
    // result (a cut sharing the solid's side faces is left to OCCT) but that
    // an imported mesh changes nothing about the solids: same history without
    // the mesh must give the same solid bodies, and the mesh must survive.
    ExtrudeFeatureData cut = ed;
    cut.operation = ExtrudeOperation::Cut;
    cut.height = 60.0f;
    h.addExtrudeFeature(cut);
    replayFeatures(h, planes, scene);

    FeatureHistory noMesh;
    FeatureID sk2 = noMesh.addSketchFeature(0, rect, planes[0].planeID);
    ExtrudeFeatureData e2 = ed;  e2.sourceSketchFeature = sk2;  noMesh.addExtrudeFeature(e2);
    ExtrudeFeatureData c2 = cut; c2.sourceSketchFeature = sk2;  noMesh.addExtrudeFeature(c2);
    auto planes2 = referencePlanes();
    Scene3D baseline;
    replayFeatures(noMesh, planes2, baseline);

    CHECK(countMeshBodies(scene) == 1, "cut removed the mesh");
    CHECK(scene.bodyCount() - 1 == baseline.bodyCount(),
          "solids with mesh=%zu, without=%zu", scene.bodyCount() - 1, baseline.bodyCount());
    for (int i = 0; i < (int)baseline.bodyCount() && i + 1 < (int)scene.bodyCount(); i++) {
        CHECK(scene.getBody(i + 1).vertexCount == baseline.getBody(i).vertexCount,
              "solid %d differs: %d vs %d vertices", i, scene.getBody(i + 1).vertexCount,
              baseline.getBody(i).vertexCount);
    }
    for (const auto& f : h.features()) CHECK(!f.hasError, "%s: %s", f.name.c_str(), f.errorMsg.c_str());

    // Boolean pointing at the mesh: an error on that feature, not a crash.
    FeatureHistory hb;
    hb.addMeshImportFeature(md, "a");
    hb.addMeshImportFeature(md, "b");
    BooleanFeatureData bd;
    bd.targetBodyIndex = 0;
    bd.toolBodyIndex = 1;
    FeatureID boolID = hb.addBooleanFeature(bd);
    Scene3D sb;
    auto planesB = referencePlanes();
    replayFeatures(hb, planesB, sb);
    const Feature* bf = hb.findFeature(boolID);
    CHECK(bf && bf->hasError, "boolean on meshes did not report an error");
    CHECK(sb.bodyCount() == 2, "boolean on meshes changed the scene: bodies=%zu", sb.bodyCount());

    // A moved/deleted file is an error on that feature; the rest still builds.
    FeatureHistory hm;
    MeshImportFeatureData gone = md;
    gone.sourcePath = (dir / "deleted.stl").string();
    FeatureID goneID = hm.addMeshImportFeature(gone, "gone");
    hm.addMeshImportFeature(md, "still_here");
    Scene3D sm;
    auto planesM = referencePlanes();
    replayFeatures(hm, planesM, sm);
    const Feature* gf = hm.findFeature(goneID);
    CHECK(gf && gf->hasError, "missing file not reported");
    CHECK(sm.bodyCount() == 1, "bodies=%zu", sm.bodyCount());
}

int main() {
    const fs::path dir = fs::temp_directory_path() / "shitcad_mesh_import_test";
    fs::create_directories(dir);
    const fs::path box = dir / "box_metres.stl";
    writeBinaryStl(box, boxTriangles(-0.3f, -0.3f, 0.0f, 0.3f, 0.3f, 1.3f));

    testLoadAndUnits(box);
    testCacheInvalidation(dir);
    testPick(box);
    testPlacement(box);
    testSaveLoad(dir, box);

    testReplay(dir, box);

    std::printf("\n%d checks, %d failed\n", g_checks, g_failures);
    return g_failures == 0 ? 0 : 1;
}
