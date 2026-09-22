// Regression net for everything under the UI: fixture histories built in code
// (one per feature type), replayed and checked against analytic volumes and
// bounds, plus every constraint type through the solver and save/load.
//
// The point is to catch behaviour changes while the UI layer is replaced
// (docs/qt-migration-plan.md, Phase 0), so the checks are on results a user
// would see: bodies, volumes, sizes, errors, solved sketches, saved files.
//
// Build with -DSHITCAD_BUILD_TESTS=ON, run build/Release/ReplayTest.exe.
// Uses a hidden GLFW window only because replay uploads meshes to OpenGL.

#include "FeatureHistory.h"
#include "FeatureReplay.h"
#include "MeshImport.h"
#include "ProfileDetector.h"
#include "Scene3D.h"
#include "Serialization.h"
#include "Simulation.h"
#include "SketchPlane.h"
#include "Solver.h"
#include "Constants.h"

#include <BRepGProp.hxx>
#include <GProp_GProps.hxx>

#include <glad/gl.h>
#include <GLFW/glfw3.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <functional>
#include <sstream>
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
static bool nearRel(double a, double b, double rel) { return std::fabs(a - b) <= rel * std::fabs(b); }

// ---- fixture helpers ---------------------------------------------------------

// The three reference planes as App::init() makes them, plus offset XY planes
// at z = 40 (index 3, for the loft) and z = 10 (index 4, the top of the box).
static std::vector<SketchPlane> fixturePlanes() {
    std::vector<SketchPlane> planes(5);
    const float normals[5][3] = {{0, 0, 1}, {0, 1, 0}, {1, 0, 0}, {0, 0, 1}, {0, 0, 1}};
    const float us[5][3] = {{1, 0, 0}, {1, 0, 0}, {0, 1, 0}, {1, 0, 0}, {1, 0, 0}};
    const float vs[5][3] = {{0, 1, 0}, {0, 0, 1}, {0, 0, 1}, {0, 1, 0}, {0, 1, 0}};
    const char* names[5] = {"XY Plane", "XZ Plane", "YZ Plane", "Offset XY", "Box top"};
    for (int i = 0; i < 5; i++) {
        planes[i].planeID = (PlaneID)(i + 1);
        std::memcpy(planes[i].normal, normals[i], sizeof(float) * 3);
        std::memcpy(planes[i].uAxis, us[i], sizeof(float) * 3);
        std::memcpy(planes[i].vAxis, vs[i], sizeof(float) * 3);
        planes[i].name = names[i];
        planes[i].isReferencePlane = true;
    }
    planes[3].origin[2] = 40.0f;
    planes[4].origin[2] = 10.0f;
    return planes;
}

static void addRect(Sketch& s, double x0, double y0, double x1, double y1) {
    EntityID a = s.addPoint(x0, y0), b = s.addPoint(x1, y0);
    EntityID c = s.addPoint(x1, y1), d = s.addPoint(x0, y1);
    s.addLine(a, b); s.addLine(b, c); s.addLine(c, d); s.addLine(d, a);
}

static Sketch rectSketch(double x0, double y0, double x1, double y1) {
    Sketch s;
    addRect(s, x0, y0, x1, y1);
    return s;
}

// Adds a sketch feature and returns its ID; also puts the sketch on the plane,
// as finishSketch() leaves it, so profiles can be signed against it.
static FeatureID addSketch(FeatureHistory& h, std::vector<SketchPlane>& planes, int planeIndex,
                           const Sketch& s) {
    planes[planeIndex].sketch = s;
    return h.addSketchFeature(planeIndex, s, planes[planeIndex].planeID);
}

// Signs profile `index` of the sketch on `planeIndex`, the way commit*() does.
static bool signProfile(const std::vector<SketchPlane>& planes, int planeIndex, int index,
                        ProfileSignature& sig) {
    const auto& plane = planes[planeIndex];
    auto profiles = detectClosedProfiles(plane.sketch, plane);
    if (index >= (int)profiles.size()) return false;
    sig = ProfileSignature::fromProfile(profiles[index], plane.sketch);
    return true;
}

static FeatureID addExtrude(FeatureHistory& h, const std::vector<SketchPlane>& planes,
                            FeatureID sketch, int planeIndex, float height,
                            ExtrudeOperation op = ExtrudeOperation::NewBody) {
    ExtrudeFeatureData ed;
    ed.sourceSketchFeature = sketch;
    ProfileSignature sig;
    CHECK(signProfile(planes, planeIndex, 0, sig), "no profile to extrude on plane %d", planeIndex);
    ed.profileSigs.push_back(sig);
    ed.profileIndicesFallback.push_back(0);
    ed.height = height;
    ed.operation = op;
    return h.addExtrudeFeature(ed);
}

struct Fixture {
    std::string name;
    FeatureHistory history;
    std::vector<SketchPlane> planes = fixturePlanes();
    SimulationSetup simulation;
    bool hasSimulation = false;
};

// What replay produced, reduced to what a user would notice.
struct Outcome {
    size_t bodies = 0;
    size_t meshBodies = 0;
    std::vector<double> volumes;         // per solid body, in body order
    double lo[3] = {1e30, 1e30, 1e30};   // over all bodies' tessellation
    double hi[3] = {-1e30, -1e30, -1e30};
    std::vector<std::string> errors;     // "name: message" for every feature with an error
};

static Outcome replayAndMeasure(FeatureHistory& h, std::vector<SketchPlane>& planes) {
    Scene3D scene;
    replayFeatures(h, planes, scene);
    Outcome o;
    o.bodies = scene.bodyCount();
    for (int i = 0; i < (int)scene.bodyCount(); i++) {
        const Body3D& b = scene.getBody(i);
        if (b.isMeshOnly()) {
            o.meshBodies++;
        } else {
            GProp_GProps props;
            BRepGProp::VolumeProperties(b.shape, props);
            o.volumes.push_back(props.Mass());
        }
        for (const auto& v : b.vertices) {
            const double p[3] = {v.px, v.py, v.pz};
            for (int k = 0; k < 3; k++) {
                o.lo[k] = std::min(o.lo[k], p[k]);
                o.hi[k] = std::max(o.hi[k], p[k]);
            }
        }
    }
    for (const auto& f : h.features())
        if (f.hasError) o.errors.push_back(f.name + ": " + f.errorMsg);
    return o;
}

static double totalVolume(const Outcome& o) {
    double v = 0;
    for (double x : o.volumes) v += x;
    return v;
}

static std::string joinErrors(const Outcome& o) {
    std::string s;
    for (const auto& e : o.errors) s += (s.empty() ? "" : "; ") + e;
    return s.empty() ? "none" : s;
}

static bool sameBounds(const Outcome& a, const Outcome& b, double tol) {
    for (int k = 0; k < 3; k++)
        if (!near(a.lo[k], b.lo[k], tol) || !near(a.hi[k], b.hi[k], tol)) return false;
    return true;
}

static void checkBounds(const Outcome& o, const double lo[3], const double hi[3], double tol,
                        const char* what) {
    bool ok = true;
    for (int k = 0; k < 3; k++) ok = ok && near(o.lo[k], lo[k], tol) && near(o.hi[k], hi[k], tol);
    CHECK(ok, "%s bounds (%g %g %g)-(%g %g %g), expected (%g %g %g)-(%g %g %g)", what,
          o.lo[0], o.lo[1], o.lo[2], o.hi[0], o.hi[1], o.hi[2],
          lo[0], lo[1], lo[2], hi[0], hi[1], hi[2]);
}

static std::string readFile(const fs::path& p) {
    std::ifstream in(p, std::ios::binary);
    std::stringstream ss;
    ss << in.rdbuf();
    return ss.str();
}

// ---- fixtures ----------------------------------------------------------------

// 40 x 20 x 10 box, then a r5 through-hole cut from its top face. A cut
// extrudes along -normal, into the material, so it is sketched on the top.
static Fixture extrudeJoinCut() {
    Fixture f;
    f.name = "extrude_join_cut";
    FeatureID s1 = addSketch(f.history, f.planes, 0, rectSketch(0, 0, 40, 20));
    addExtrude(f.history, f.planes, s1, 0, 10.0f);

    Sketch hole;
    hole.addCircle(hole.addPoint(20, 10), 5.0);
    FeatureID s2 = addSketch(f.history, f.planes, 4, hole);
    addExtrude(f.history, f.planes, s2, 4, 10.0f, ExtrudeOperation::Cut);
    return f;
}

// 10 x 30 rectangle 10 mm off a vertical axis line, revolved 360 degrees:
// a tube, inner radius 10, outer 20, 30 tall along Y.
static Fixture revolveTube() {
    Fixture f;
    f.name = "revolve_tube";
    Sketch s = rectSketch(10, 0, 20, 30);
    EntityID axis = s.addLine(s.addPoint(0, 0), s.addPoint(0, 30));
    FeatureID sk = addSketch(f.history, f.planes, 0, s);

    RevolveFeatureData rd;
    rd.sourceSketchFeature = sk;
    ProfileSignature sig;
    CHECK(signProfile(f.planes, 0, 0, sig), "no profile to revolve");
    rd.profileSigs.push_back(sig);
    rd.profileIndicesFallback.push_back(0);
    rd.axisLineID = axis;
    rd.angleDeg = 360.0f;
    f.history.addRevolveFeature(rd);
    return f;
}

// 20 x 20 square at z = 0 lofted to a 10 x 10 square at z = 40: a frustum.
static Fixture loftFrustum() {
    Fixture f;
    f.name = "loft_frustum";
    FeatureID a = addSketch(f.history, f.planes, 0, rectSketch(-10, -10, 10, 10));
    FeatureID b = addSketch(f.history, f.planes, 3, rectSketch(-5, -5, 5, 5));

    LoftFeatureData ld;
    LoftSection sa, sb;
    sa.sourceSketchFeature = a;
    sb.sourceSketchFeature = b;
    CHECK(signProfile(f.planes, 0, 0, sa.profileSig), "no bottom loft profile");
    CHECK(signProfile(f.planes, 3, 0, sb.profileSig), "no top loft profile");
    ld.sections = {sa, sb};
    f.history.addLoftFeature(ld);
    return f;
}

// Two separate 20 mm cubes (a NewBody extrude fuses anything it overlaps, so
// the bodies cannot overlap), then a boolean subtract of the second from the first.
static Fixture booleanSubtract() {
    Fixture f;
    f.name = "boolean_subtract";
    FeatureID a = addSketch(f.history, f.planes, 0, rectSketch(0, 0, 20, 20));
    FeatureID ea = addExtrude(f.history, f.planes, a, 0, 20.0f);
    FeatureID b = addSketch(f.history, f.planes, 0, rectSketch(30, 0, 50, 20));
    FeatureID eb = addExtrude(f.history, f.planes, b, 0, 20.0f);

    BooleanFeatureData bd;
    bd.operation = BooleanOperation::Subtract;
    bd.targetBodyFeatures = {ea};
    bd.toolBodyFeatures = {eb};
    bd.targetBodyIndex = 0;
    bd.toolBodyIndex = 1;
    f.history.addBooleanFeature(bd);
    return f;
}

// The join/cut history with the cut suppressed.
static Fixture suppressedCut() {
    Fixture f = extrudeJoinCut();
    f.name = "suppressed_cut";
    f.history.suppressFeature(f.history.features().back().id);
    return f;
}

// The join/cut history rolled back to just after the first extrude.
static Fixture rolledBack() {
    Fixture f = extrudeJoinCut();
    f.name = "rolled_back";
    f.history.setRollbackPos(1);
    return f;
}

struct Tri { float v[9]; };

static void writeBoxStl(const fs::path& path, float x0, float y0, float z0, float x1, float y1, float z1) {
    const float c[8][3] = {
        {x0, y0, z0}, {x1, y0, z0}, {x1, y1, z0}, {x0, y1, z0},
        {x0, y0, z1}, {x1, y0, z1}, {x1, y1, z1}, {x0, y1, z1},
    };
    const int q[6][4] = {{0, 3, 2, 1}, {4, 5, 6, 7}, {0, 1, 5, 4}, {2, 3, 7, 6}, {1, 2, 6, 5}, {3, 0, 4, 7}};
    std::vector<Tri> tris;
    for (const auto& fq : q) {
        const int t[2][3] = {{fq[0], fq[1], fq[2]}, {fq[0], fq[2], fq[3]}};
        for (const auto& tt : t) {
            Tri tri;
            for (int k = 0; k < 3; k++)
                for (int a = 0; a < 3; a++) tri.v[k * 3 + a] = c[tt[k]][a];
            tris.push_back(tri);
        }
    }
    std::ofstream out(path, std::ios::binary);
    char header[80] = {};
    std::snprintf(header, sizeof(header), "ReplayTest");
    out.write(header, 80);
    uint32_t n = (uint32_t)tris.size();
    out.write((const char*)&n, 4);
    for (const auto& t : tris) {
        const float zero[3] = {0, 0, 0};
        out.write((const char*)zero, 12);
        out.write((const char*)t.v, 36);
        uint16_t attr = 0;
        out.write((const char*)&attr, 2);
    }
}

// A 0.6 x 0.6 x 1.3 m vessel in metres, laid on its side and lifted, with a
// surface role and a nozzle on it.
static Fixture meshWithSimulation(const fs::path& stl) {
    Fixture f;
    f.name = "mesh_simulation";
    MeshImportFeatureData md;
    md.sourcePath = stl.string();
    md.unit = "m";
    axisRotation(0, 90.0, md.transform.r); // Rx90 about the origin: y' = -z, z' = y
    md.transform.t[2] = 500.0;
    FeatureID mesh = f.history.addMeshImportFeature(md, "vessel");

    f.simulation.setRole(mesh, SurfaceRole::Wall);
    SimNozzle n;
    n.hostFeature = mesh;
    n.position[0] = 0; n.position[1] = 0; n.position[2] = 1200;
    n.axis[0] = 0; n.axis[1] = 0; n.axis[2] = -1;
    n.halfAngleDeg = 90.0f;
    f.simulation.addNozzle(n);
    f.simulation.rays = 12345;
    f.simulation.bounces = 1;
    f.hasSimulation = true;
    return f;
}

// ---- tests -------------------------------------------------------------------

static void testExtrudeJoinCut() {
    std::printf("extrude join + cut\n");
    Fixture f = extrudeJoinCut();
    Outcome o = replayAndMeasure(f.history, f.planes);
    CHECK(o.errors.empty(), "errors: %s", joinErrors(o).c_str());
    CHECK(o.bodies == 1, "bodies=%zu", o.bodies);
    CHECK(nearRel(totalVolume(o), 8000.0 - 250.0 * kPiD, 1e-3), "volume=%g", totalVolume(o));
    const double lo[3] = {0, 0, 0}, hi[3] = {40, 20, 10};
    checkBounds(o, lo, hi, 1e-3, "box");
}

static void testRevolve() {
    std::printf("revolve\n");
    Fixture f = revolveTube();
    Outcome o = replayAndMeasure(f.history, f.planes);
    CHECK(o.errors.empty(), "errors: %s", joinErrors(o).c_str());
    CHECK(o.bodies == 1, "bodies=%zu", o.bodies);
    CHECK(nearRel(totalVolume(o), kPiD * (400.0 - 100.0) * 30.0, 1e-3), "volume=%g", totalVolume(o));
    // Tessellation sits on or inside the true surface, so allow for chord error.
    const double lo[3] = {-20, 0, -20}, hi[3] = {20, 30, 20};
    checkBounds(o, lo, hi, 0.5, "tube");
}

static void testLoft() {
    std::printf("loft\n");
    Fixture f = loftFrustum();
    Outcome o = replayAndMeasure(f.history, f.planes);
    CHECK(o.errors.empty(), "errors: %s", joinErrors(o).c_str());
    CHECK(o.bodies == 1, "bodies=%zu", o.bodies);
    const double frustum = 40.0 / 3.0 * (400.0 + 100.0 + std::sqrt(400.0 * 100.0));
    CHECK(nearRel(totalVolume(o), frustum, 1e-2), "volume=%g, frustum=%g", totalVolume(o), frustum);
    const double lo[3] = {-10, -10, 0}, hi[3] = {10, 10, 40};
    checkBounds(o, lo, hi, 1e-3, "frustum");
}

static void testBoolean() {
    std::printf("boolean subtract\n");
    Fixture f = booleanSubtract();
    Outcome o = replayAndMeasure(f.history, f.planes);
    CHECK(o.errors.empty(), "errors: %s", joinErrors(o).c_str());
    CHECK(o.bodies == 1, "bodies=%zu (the tool body is consumed)", o.bodies);
    CHECK(nearRel(totalVolume(o), 8000.0, 1e-3), "volume=%g", totalVolume(o));
    const double lo[3] = {0, 0, 0}, hi[3] = {20, 20, 20};
    checkBounds(o, lo, hi, 1e-3, "target");
}

static void testSuppressAndRollback() {
    std::printf("suppress and rollback\n");
    Fixture s = suppressedCut();
    Outcome os = replayAndMeasure(s.history, s.planes);
    CHECK(os.errors.empty(), "errors: %s", joinErrors(os).c_str());
    CHECK(os.bodies == 1 && nearRel(totalVolume(os), 8000.0, 1e-3),
          "suppressed cut still applied: bodies=%zu volume=%g", os.bodies, totalVolume(os));

    Fixture r = rolledBack();
    Outcome orb = replayAndMeasure(r.history, r.planes);
    CHECK(orb.errors.empty(), "errors: %s", joinErrors(orb).c_str());
    CHECK(orb.bodies == 1 && nearRel(totalVolume(orb), 8000.0, 1e-3),
          "rolled-back cut still applied: bodies=%zu volume=%g", orb.bodies, totalVolume(orb));

    // Rolling forward again restores the cut.
    r.history.setRollbackPos(-1);
    Outcome all = replayAndMeasure(r.history, r.planes);
    CHECK(nearRel(totalVolume(all), 8000.0 - 250.0 * kPiD, 1e-3), "roll forward volume=%g", totalVolume(all));
}

static void testMeshAndSimulation(const fs::path& stl) {
    std::printf("mesh import + simulation\n");
    Fixture f = meshWithSimulation(stl);
    Outcome o = replayAndMeasure(f.history, f.planes);
    CHECK(o.errors.empty(), "errors: %s", joinErrors(o).c_str());
    CHECK(o.bodies == 1 && o.meshBodies == 1, "bodies=%zu mesh=%zu", o.bodies, o.meshBodies);
    // Box x,y in [-300, 300], z in [0, 1300] mm; Rx90 sends (x, y, z) to (x, -z, y); then +500 z.
    const double lo[3] = {-300, -1300, 200}, hi[3] = {300, 0, 800};
    checkBounds(o, lo, hi, 1e-2, "placed vessel");

    // The nozzle follows its host's placement.
    double pos[3], axis[3];
    CHECK(nozzleWorld(f.simulation.nozzles[0], f.history, pos, axis), "nozzle lost its host");
    CHECK(near(pos[0], 0, 1e-6) && near(pos[1], -1200, 1e-6) && near(pos[2], 500, 1e-6),
          "nozzle world (%g, %g, %g)", pos[0], pos[1], pos[2]);
    CHECK(near(axis[1], 1.0, 1e-9), "nozzle axis (%g, %g, %g)", axis[0], axis[1], axis[2]);
}

// Every entity type, and every constraint type on geometry that already
// satisfies it: the solver must accept all of them and leave the geometry put.
static Sketch kitchenSinkSketch() {
    Sketch s;
    // Rectangle: H/V and distances.
    EntityID p1 = s.addPoint(0, 0), p2 = s.addPoint(40, 0);
    EntityID p3 = s.addPoint(40, 20), p4 = s.addPoint(0, 20);
    EntityID l1 = s.addLine(p1, p2), l2 = s.addLine(p2, p3);
    EntityID l3 = s.addLine(p3, p4), l4 = s.addLine(p4, p1);
    s.addConstraint(ConstraintType::Horizontal, l1, NullID);
    s.addConstraint(ConstraintType::Horizontal, l3, NullID);
    s.addConstraint(ConstraintType::Vertical, l2, NullID);
    s.addConstraint(ConstraintType::Vertical, l4, NullID);
    s.addConstraint(ConstraintType::Distance, l1, NullID, 40.0);
    s.addConstraint(ConstraintType::Distance, l2, NullID, 20.0);

    // Points against the rectangle.
    s.addConstraint(ConstraintType::Coincident, s.addPoint(40, 20), p3);
    s.addConstraint(ConstraintType::PointOnLine, s.addPoint(10, 0), l1);
    EntityID pld = s.addConstraint(ConstraintType::PointLineDistance, s.addPoint(20, 5), l1, 5.0);
    s.findConstraint(pld)->negativeSide = true; // left of p1->p2 is +y: cross < 0
    s.addConstraint(ConstraintType::Midpoint, s.addPoint(20, 20), l3);
    EntityID axis = s.addLine(s.addPoint(20, -5), s.addPoint(20, 25));
    EntityID sym = s.addConstraint(ConstraintType::Symmetric, s.addPoint(10, 10), s.addPoint(30, 10));
    s.findConstraint(sym)->entityC = axis;
    s.addConstraint(ConstraintType::PointDistance, s.addPoint(0, 40), s.addPoint(30, 40), 30.0);

    // Line pairs.
    EntityID a = s.addLine(s.addPoint(60, 0), s.addPoint(80, 0));
    EntityID b = s.addLine(s.addPoint(60, 10), s.addPoint(80, 10));
    s.addConstraint(ConstraintType::Parallel, a, b);
    s.addConstraint(ConstraintType::EqualLength, a, b);
    s.addConstraint(ConstraintType::Perpendicular, a, s.addLine(s.addPoint(100, 0), s.addPoint(100, 20)));
    s.addConstraint(ConstraintType::Collinear, a, s.addLine(s.addPoint(85, 0), s.addPoint(95, 0)));
    EntityID vtx = s.addPoint(60, 40);
    EntityID fLine = s.addLine(vtx, s.addPoint(80, 40));
    EntityID gLine = s.addLine(vtx, s.addPoint(60 + 10 * std::cos(kPiD / 6), 40 + 10 * std::sin(kPiD / 6)));
    s.addConstraint(ConstraintType::Angle, fLine, gLine, 30.0);

    // Circles and an arc.
    EntityID c0 = s.addCircle(s.addPoint(0, -40), 10.0);
    EntityID c1 = s.addCircle(s.addPoint(0, -40), 5.0);
    s.addConstraint(ConstraintType::Radius, c0, NullID, 10.0);
    s.addConstraint(ConstraintType::Diameter, c1, NullID, 10.0);
    s.addConstraint(ConstraintType::Concentric, c0, c1);
    s.addConstraint(ConstraintType::PointOnCircle, s.addPoint(10, -40), c0);
    s.addConstraint(ConstraintType::Tangent, s.addLine(s.addPoint(-20, -30), s.addPoint(20, -30)), c0);
    EntityID arc = s.addArc(s.addPoint(60, -40), s.addPoint(70, -40), s.addPoint(60, -30));
    s.addConstraint(ConstraintType::Radius, arc, NullID, 10.0);

    // Remaining entity types (no constraint types apply to them).
    s.addEllipse(s.addPoint(100, -40), 12.0, 6.0, 0.0);
    s.addEllipseArc(s.addPoint(130, -40), s.addPoint(142, -40), s.addPoint(130, -34), 12.0, 6.0, 0.0);
    s.addSpline({s.addPoint(0, 60), s.addPoint(10, 70), s.addPoint(20, 60), s.addPoint(30, 70)});
    return s;
}

static void testEveryConstraintType(const fs::path& dir) {
    std::printf("every constraint type\n");
    Sketch s = kitchenSinkSketch();

    const int kTypes = (int)ConstraintType::PointOnCircle + 1;
    std::vector<int> seen(kTypes, 0);
    for (const auto& c : s.constraints) seen[(int)c.type]++;
    for (int t = 0; t < kTypes; t++) CHECK(seen[t] > 0, "constraint type %d has no fixture", t);

    Sketch solved = s;
    Solver solver;
    SolveResult r = solver.solve(solved);
    CHECK(r.ok && r.converged, "satisfied sketch did not solve: ok=%d converged=%d error=%g",
          r.ok, r.converged, r.totalError);
    double maxMove = 0;
    for (size_t i = 0; i < s.points.size(); i++)
        maxMove = std::max(maxMove, distance({s.points[i].x, s.points[i].y},
                                             {solved.points[i].x, solved.points[i].y}));
    CHECK(maxMove < 1e-4, "solver moved already-satisfied geometry by %g mm", maxMove);

    // Knock two free points off their constraints; the solver must pull them back.
    Sketch nudged = s;
    nudged.points[4].x += 0.7;  // the Coincident point
    nudged.points[5].y += 0.4;  // the PointOnLine point
    nudged.rebuildIndices();
    r = solver.solve(nudged);
    CHECK(r.ok && r.converged, "nudged sketch did not solve: ok=%d converged=%d error=%g",
          r.ok, r.converged, r.totalError);

    // Every entity and constraint survives save/load unchanged.
    FeatureHistory h;
    auto planes = fixturePlanes();
    addSketch(h, planes, 0, s);
    const fs::path p = dir / "kitchen_sink.shitcad";
    CHECK(saveProject(p.string(), h, planes), "%s", lastLoadError().c_str());
    FeatureHistory lh;
    std::vector<SketchPlane> lp;
    CHECK(loadProject(p.string(), lh, lp), "%s", lastLoadError().c_str());
    if (lh.empty()) return;
    const Sketch& l = std::get<SketchFeatureData>(lh.features()[0].data).sketchSnapshot;
    CHECK(l.points.size() == s.points.size() && l.lines.size() == s.lines.size() &&
          l.circles.size() == s.circles.size() && l.arcs.size() == s.arcs.size() &&
          l.ellipses.size() == s.ellipses.size() && l.ellipseArcs.size() == s.ellipseArcs.size() &&
          l.splines.size() == s.splines.size(),
          "entity counts changed across save/load");
    CHECK(l.constraints.size() == s.constraints.size(), "constraints %zu -> %zu",
          s.constraints.size(), l.constraints.size());
    for (size_t i = 0; i < std::min(l.constraints.size(), s.constraints.size()); i++) {
        const Constraint& a = s.constraints[i];
        const Constraint& b = l.constraints[i];
        CHECK(a.id == b.id && a.type == b.type && a.entityA == b.entityA && a.entityB == b.entityB &&
              a.entityC == b.entityC && a.value == b.value && a.negativeSide == b.negativeSide &&
              a.angleCW == b.angleCW,
              "constraint %u (type %d) changed across save/load", a.id, (int)a.type);
    }
}

// Every fixture: save, load, save again. The two files must be identical and
// the loaded history must replay to the same result as the original.
static void testRoundTrips(const fs::path& dir, const fs::path& stl) {
    std::printf("save / load round trips\n");
    std::vector<std::function<Fixture()>> makers = {
        extrudeJoinCut, revolveTube, loftFrustum, booleanSubtract, suppressedCut, rolledBack,
        [&] { return meshWithSimulation(stl); },
    };
    for (const auto& make : makers) {
        Fixture f = make();
        Outcome before = replayAndMeasure(f.history, f.planes);

        const fs::path p1 = dir / (f.name + "_1.shitcad");
        const fs::path p2 = dir / (f.name + "_2.shitcad");
        const SimulationSetup* sim = f.hasSimulation ? &f.simulation : nullptr;
        CHECK(saveProject(p1.string(), f.history, f.planes, sim), "%s: %s", f.name.c_str(), lastLoadError().c_str());

        FeatureHistory lh;
        std::vector<SketchPlane> lp;
        SimulationSetup ls;
        CHECK(loadProject(p1.string(), lh, lp, &ls), "%s: %s", f.name.c_str(), lastLoadError().c_str());
        CHECK(saveProject(p2.string(), lh, lp, f.hasSimulation ? &ls : nullptr), "%s: %s", f.name.c_str(),
              lastLoadError().c_str());
        CHECK(readFile(p1) == readFile(p2), "%s: save -> load -> save changed the file", f.name.c_str());
        if (f.hasSimulation)
            CHECK(sameSimulation(f.simulation, ls), "%s: simulation setup changed across save/load", f.name.c_str());
        CHECK(lh.rollbackPos() == f.history.rollbackPos(), "%s: rollback %d -> %d", f.name.c_str(),
              f.history.rollbackPos(), lh.rollbackPos());

        Outcome after = replayAndMeasure(lh, lp);
        CHECK(after.errors == before.errors, "%s: errors after load: %s", f.name.c_str(), joinErrors(after).c_str());
        CHECK(after.bodies == before.bodies && after.meshBodies == before.meshBodies,
              "%s: bodies %zu -> %zu", f.name.c_str(), before.bodies, after.bodies);
        CHECK(nearRel(totalVolume(after), totalVolume(before), 1e-9) || totalVolume(before) == 0,
              "%s: volume %g -> %g", f.name.c_str(), totalVolume(before), totalVolume(after));
        CHECK(sameBounds(before, after, 1e-6), "%s: bounds changed after load", f.name.c_str());
    }
}

int main() {
    const fs::path dir = fs::temp_directory_path() / "shitcad_replay_test";
    fs::create_directories(dir);
    const fs::path stl = dir / "vessel_metres.stl";
    writeBoxStl(stl, -0.3f, -0.3f, 0.0f, 0.3f, 0.3f, 1.3f);

    testEveryConstraintType(dir);

    if (!glfwInit()) {
        std::printf("glfwInit failed; replay tests skipped\n");
        g_failures++;
    } else {
        glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);
        glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
        glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
        glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
        GLFWwindow* win = glfwCreateWindow(64, 64, "test", nullptr, nullptr);
        if (!win) {
            std::printf("no GL context; replay tests skipped\n");
            g_failures++;
        } else {
            glfwMakeContextCurrent(win);
            gladLoadGL(glfwGetProcAddress);
            testExtrudeJoinCut();
            testRevolve();
            testLoft();
            testBoolean();
            testSuppressAndRollback();
            testMeshAndSimulation(stl);
            testRoundTrips(dir, stl);
            glfwDestroyWindow(win);
        }
        glfwTerminate();
    }

    std::printf("\n%d checks, %d failed\n", g_checks, g_failures);
    return g_failures == 0 ? 0 : 1;
}
