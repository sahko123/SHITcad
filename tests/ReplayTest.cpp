// Regression net for everything under the UI: fixture histories built in code
// (one per feature type), replayed and checked against analytic volumes and
// bounds, plus every constraint type through the solver and save/load.
//
// The point is to catch behaviour changes while the UI layer is replaced
// (docs/qt-migration-plan.md, Phase 0), so the checks are on results a user
// would see: bodies, volumes, sizes, errors, solved sketches, saved files.
//
// Build with -DSHITCAD_BUILD_TESTS=ON, run build/Release/ReplayTest.exe.
// Runs without a GL context on purpose: Scene3D uploads lazily at render
// time, so replay must not call OpenGL (a stray call would crash here).

#include "CadImport.h"
#include "FeatureHistory.h"
#include "FeatureReplay.h"
#include "MeshImport.h"
#include "ProfileDetector.h"
#include "Scene3D.h"
#include "Serialization.h"
#include "Simulation.h"
#include "SketchPlane.h"
#include "Solver.h"
#include "Utf8Path.h"
#include "Constants.h"

#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepGProp.hxx>
#include <BRep_Builder.hxx>
#include <TopoDS_Compound.hxx>
#include <BRepPrimAPI_MakeBox.hxx>
#include <BRepPrimAPI_MakeCylinder.hxx>
#include <GProp_GProps.hxx>
#include <IGESControl_Writer.hxx>
#include <Quantity_Color.hxx>
#include <STEPCAFControl_Writer.hxx>
#include <STEPControl_Writer.hxx>
#include <StepData_StepModel.hxx>
#include <TDataStd_Name.hxx>
#include <TDocStd_Document.hxx>
#include <XCAFApp_Application.hxx>
#include <XCAFDoc_ColorTool.hxx>
#include <XCAFDoc_DocumentTool.hxx>
#include <XCAFDoc_ShapeTool.hxx>
#include <gp_Pln.hxx>
#include <gp_Trsf.hxx>

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

#include "TestUtil.h"

using namespace shitcad;
namespace fs = std::filesystem;




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
        CHECK(b.vao == 0 && b.gpuDirty, "body %d was uploaded during replay", i);
        CHECK(b.vertexCount == (int)b.vertices.size(), "body %d vertexCount %d != %zu", i, b.vertexCount, b.vertices.size());
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
    bd.targetBody = {ea, 0};
    bd.toolBody = {eb, 0};
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


// ---- STEP / IGES fixtures -----------------------------------------------------

// A box written as STEP in inches, so reading it back in mm proves the unit
// conversion. Returns false (and the test fails) if the file is not in inches.
static bool writeBoxStepInches(const fs::path& path, double x0, double y0, double z0,
                               double dx, double dy, double dz) {
    TopoDS_Shape box = BRepPrimAPI_MakeBox(gp_Pnt(x0, y0, z0), dx, dy, dz).Shape();
    STEPControl_Writer writer;
    DESTEP_Parameters params; // OCCT 8 ignores Interface_Static("write.step.unit")
    params.WriteUnit = UnitsMethods_LengthUnit_Inch;
    if (writer.Transfer(box, STEPControl_AsIs, params) != IFSelect_RetDone) return false;
    if (writer.Write(utf8(path).c_str()) != IFSelect_RetDone) return false;
    return readFile(path).find("INCH") != std::string::npos;
}

static bool writeBoxIges(const fs::path& path, double dx, double dy, double dz) {
    IGESControl_Writer writer("IN", 0);
    writer.AddShape(BRepPrimAPI_MakeBox(dx, dy, dz).Shape());
    writer.ComputeModel();
    return writer.Write(utf8(path).c_str());
}

// An assembly: one named, red r5 x 20 pin used twice, at x = 0 and x = 30.
static bool writePinAssembly(const fs::path& path) {
    Handle(XCAFApp_Application) app = XCAFApp_Application::GetApplication();
    Handle(TDocStd_Document) doc;
    app->NewDocument("MDTV-XCAF", doc);
    XCAFDoc_DocumentTool::SetLengthUnit(doc, 1.0, UnitsMethods_LengthUnit_Millimeter);
    Handle(XCAFDoc_ShapeTool) shapes = XCAFDoc_DocumentTool::ShapeTool(doc->Main());
    Handle(XCAFDoc_ColorTool) colors = XCAFDoc_DocumentTool::ColorTool(doc->Main());

    TDF_Label pin = shapes->AddShape(BRepPrimAPI_MakeCylinder(5.0, 20.0).Shape(), false);
    TDataStd_Name::Set(pin, "Pin");
    colors->SetColor(pin, Quantity_Color(1.0, 0.0, 0.0, Quantity_TOC_sRGB), XCAFDoc_ColorSurf);
    TDF_Label assembly = shapes->NewShape();
    TDataStd_Name::Set(assembly, "Pins");
    for (double x : {0.0, 30.0}) {
        gp_Trsf t;
        t.SetTranslation(gp_Vec(x, 0, 0));
        shapes->AddComponent(assembly, pin, TopLoc_Location(t));
    }
    shapes->UpdateAssemblies();

    STEPCAFControl_Writer writer;
    const bool ok = writer.Transfer(doc, STEPControl_AsIs) && writer.Write(utf8(path).c_str()) == IFSelect_RetDone;
    app->Close(doc);
    return ok;
}

static bool writeStepShape(const fs::path& path, const TopoDS_Shape& shape) {
    STEPControl_Writer writer;
    return writer.Transfer(shape, STEPControl_AsIs) == IFSelect_RetDone &&
           writer.Write(utf8(path).c_str()) == IFSelect_RetDone;
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

// A STEP box (inches in the file) imported Z-up: stood upright, then moved.
static Fixture cadImportFixture(const fs::path& step) {
    Fixture f;
    f.name = "cad_import";
    CadImportFeatureData cd;
    cd.sourcePath = utf8(step);
    axisRotation(0, -90.0, cd.transform.r); // Rx-90: (x, y, z) -> (x, z, -y)
    cd.transform.t[0] = 100.0;
    f.history.addCadImportFeature(cd, "bracket");
    return f;
}

static void testCadImport(const fs::path& dir) {
    std::printf("STEP / IGES import\n");

    // 2 x 1 x 0.5 in box at (1, 1, 0) in, written in inches.
    const fs::path step = dir / "box_inches.step";
    CHECK(writeBoxStepInches(step, 25.4, 25.4, 0.0, 50.8, 25.4, 12.7), "fixture STEP is not in inches");

    std::vector<CadPart> parts;
    CadFileInfo info;
    std::string err;
    CHECK(loadCadFile(utf8(step), parts, info, err), "load: %s", err.c_str());
    CHECK(info.format == "STEP" && info.fileUnit == "inch", "format '%s' unit '%s'", info.format.c_str(), info.fileUnit.c_str());
    CHECK(info.solids == 1 && info.surfaces == 0 && parts.size() == 1, "solids %d surfaces %d parts %zu",
          info.solids, info.surfaces, parts.size());
    {
        const double lo[3] = {25.4, 25.4, 0.0}, hi[3] = {76.2, 50.8, 12.7};
        bool ok = true;
        for (int k = 0; k < 3; k++) ok = ok && near(info.lo[k], lo[k], 1e-3) && near(info.hi[k], hi[k], 1e-3);
        CHECK(ok, "inch file not read in mm: (%g %g %g)-(%g %g %g)", info.lo[0], info.lo[1], info.lo[2],
              info.hi[0], info.hi[1], info.hi[2]);
    }

    // Replayed as a feature: a real solid (not mesh-only), in mm, placed.
    Fixture f = cadImportFixture(step);
    Outcome o = replayAndMeasure(f.history, f.planes);
    CHECK(o.errors.empty(), "errors: %s", joinErrors(o).c_str());
    CHECK(o.bodies == 1 && o.meshBodies == 0, "bodies=%zu mesh=%zu", o.bodies, o.meshBodies);
    CHECK(nearRel(totalVolume(o), 50.8 * 25.4 * 12.7, 1e-6), "volume=%g", totalVolume(o));
    {
        // Rx-90 sends (x, y, z) to (x, z, -y); then +100 in x.
        const double lo[3] = {125.4, 0.0, -50.8}, hi[3] = {176.2, 12.7, -25.4};
        checkBounds(o, lo, hi, 1e-3, "placed STEP box");
    }

    // An imported solid is B-rep like any other body: a cut extrude cuts it.
    {
        Fixture b = cadImportFixture(step);
        auto& cd = std::get<CadImportFeatureData>(b.history.features()[0].data);
        cd.transform = MeshTransform{};
        cd.transform.t[0] = 140.0 - 25.4 - 10.0; // box x 130..180.8
        cd.transform.t[1] = -25.4 - 5.0;         // box y -5..20.4
        cd.transform.t[2] = -17.7;               // box z -17.7..-5
        // A 20 x 20 cut from the XY plane, 20 deep into -z: x 140..160, y -10..10.
        FeatureID sk = addSketch(b.history, b.planes, 0, rectSketch(140, -10, 160, 10));
        addExtrude(b.history, b.planes, sk, 0, 20.0f, ExtrudeOperation::Cut);
        Outcome ob = replayAndMeasure(b.history, b.planes);
        CHECK(ob.errors.empty(), "cut through an import: %s", joinErrors(ob).c_str());
        // Removed: x 20 x y -5..10 (15) x z 12.7
        const double expect = 50.8 * 25.4 * 12.7 - 20.0 * 15.0 * 12.7;
        CHECK(ob.bodies == 1 && nearRel(totalVolume(ob), expect, 1e-6), "import after cut: bodies=%zu volume=%g, expected %g",
              ob.bodies, totalVolume(ob), expect);
    }

    // Placement is a pure rotation even after turns that are not quarter turns.
    {
        MeshTransform xf;
        double q[9];
        const double pivot[3] = {0, 0, 0};
        for (int i = 0; i < 7; i++) {
            axisRotation(i % 3, 37.0, q);
            rotateAbout(xf, q, pivot);
        }
        TopoDS_Shape placed = placeCadShape(parts[0].shape, xf);
        const double sf = placed.Location().Transformation().ScaleFactor();
        CHECK(sf == 1.0, "placement has a scale factor %.17g", sf);
        GProp_GProps props;
        BRepGProp::VolumeProperties(placed, props);
        CHECK(nearRel(props.Mass(), 50.8 * 25.4 * 12.7, 1e-9), "rotated volume=%g", props.Mass());
    }

    // IGES, also in inches.
    {
        const fs::path iges = dir / "box_inches.igs";
        CHECK(writeBoxIges(iges, 25.4, 50.8, 76.2), "could not write the IGES fixture");
        CHECK(loadCadFile(utf8(iges), parts, info, err), "IGES: %s", err.c_str());
        CHECK(info.format == "IGES" && info.fileUnit == "inch", "IGES format '%s' unit '%s'", info.format.c_str(), info.fileUnit.c_str());
        CHECK(!parts.empty(), "IGES produced no parts");
        CHECK(near(info.hi[0] - info.lo[0], 25.4, 1e-3) && near(info.hi[2] - info.lo[2], 76.2, 1e-3),
              "IGES size %g x %g x %g", info.hi[0] - info.lo[0], info.hi[1] - info.lo[1], info.hi[2] - info.lo[2]);
    }

    // An assembly: two instances of one named, coloured part.
    {
        const fs::path asmPath = dir / "pins.step";
        CHECK(writePinAssembly(asmPath), "could not write the assembly fixture");
        FeatureHistory h;
        std::vector<SketchPlane> planes = fixturePlanes();
        CadImportFeatureData cd;
        cd.sourcePath = utf8(asmPath);
        h.addCadImportFeature(cd, "pins");
        Scene3D scene;
        replayFeatures(h, planes, scene);
        CHECK(!h.features()[0].hasError, "assembly: %s", h.features()[0].errorMsg.c_str());
        CHECK(scene.bodyCount() == 2, "assembly bodies=%zu", scene.bodyCount());
        double xs[2] = {0, 0};
        for (int i = 0; i < (int)scene.bodyCount() && i < 2; i++) {
            const Body3D& b = scene.getBody(i);
            // Two instances of one part: told apart in the object tree.
            CHECK(b.name == (i == 0 ? "Pin (1)" : "Pin (2)"), "part name '%s'", b.name.c_str());
            CHECK(b.hasFileColor && near(b.fileColor[0], 1.0, 1e-3) && near(b.fileColor[1], 0.0, 1e-3),
                  "part colour (%g %g %g) set=%d", b.fileColor[0], b.fileColor[1], b.fileColor[2], (int)b.hasFileColor);
            CHECK(b.sourceFeature == h.features()[0].id, "body %d not tied to its feature", i);
            float lo = 1e30f, hi = -1e30f;
            for (const auto& v : b.vertices) { lo = std::min(lo, v.px); hi = std::max(hi, v.px); }
            xs[i] = 0.5 * (lo + hi);
        }
        if (xs[0] > xs[1]) std::swap(xs[0], xs[1]);
        CHECK(near(xs[0], 0.0, 0.01) && near(xs[1], 30.0, 0.01), "instances at x=%g, %g", xs[0], xs[1]);
        // A boolean tint is undone back to the file's colour, not the theme's.
        scene.getBodyMut(0).colorR = 0.2f;
        scene.resetBodyColors();
        CHECK(near(scene.getBody(0).colorR, 1.0, 1e-3), "colour after reset %g", scene.getBody(0).colorR);
    }

    // A surface model makes an open surface body; curves alone make an error.
    {
        const fs::path face = dir / "face.step";
        CHECK(writeStepShape(face, BRepBuilderAPI_MakeFace(gp_Pln(), -10, 10, -10, 10).Shape()), "face fixture");
        CHECK(loadCadFile(utf8(face), parts, info, err), "surface: %s", err.c_str());
        CHECK(info.solids == 0 && info.surfaces == 1 && parts.size() == 1, "surface: solids %d surfaces %d",
              info.solids, info.surfaces);
        FeatureHistory h;
        std::vector<SketchPlane> planes = fixturePlanes();
        CadImportFeatureData cd;
        cd.sourcePath = utf8(face);
        h.addCadImportFeature(cd, "sheet");
        Scene3D scene;
        replayFeatures(h, planes, scene);
        CHECK(scene.bodyCount() == 1 && !scene.getBody(0).closed, "a surface must not be section-capped");

        const fs::path wire = dir / "wire.step";
        CHECK(writeStepShape(wire, BRepBuilderAPI_MakeEdge(gp_Pnt(0, 0, 0), gp_Pnt(10, 0, 0)).Shape()), "wire fixture");
        CHECK(!loadCadFile(utf8(wire), parts, info, err) && !err.empty(), "curves-only file should fail");
    }

    // A missing file is an error on the feature, not a crash or a silent gap.
    {
        Fixture m = cadImportFixture(dir / "does_not_exist.step");
        Outcome om = replayAndMeasure(m.history, m.planes);
        CHECK(om.bodies == 0 && om.errors.size() == 1, "missing file: bodies=%zu errors=%s", om.bodies, joinErrors(om).c_str());
        CHECK(!loadCadFile(utf8(dir / "not_a_step.step"), parts, info, err), "a missing file loaded");
        {
            std::ofstream junk(dir / "garbage.step");
            junk << "this is not ISO-10303-21\n";
        }
        CHECK(!loadCadFile(utf8(dir / "garbage.step"), parts, info, err) && !err.empty(), "garbage loaded");
        // The failure is cached (not re-parsed every frame) but only until the
        // file changes: fixing the file must be picked up.
        std::string err2;
        CHECK(!loadCadFile(utf8(dir / "garbage.step"), parts, info, err2) && err2 == err, "cached failure '%s'", err2.c_str());
        CHECK(writeBoxStepInches(dir / "garbage.step", 0, 0, 0, 25.4, 25.4, 25.4), "rewrite garbage.step");
        CHECK(loadCadFile(utf8(dir / "garbage.step"), parts, info, err), "a fixed file stayed failed: %s", err.c_str());
    }

    // A re-exported file is picked up by the next replay.
    {
        const fs::path live = dir / "live.step";
        CHECK(writeBoxStepInches(live, 0, 0, 0, 25.4, 25.4, 25.4), "live fixture");
        Fixture a = cadImportFixture(live);
        Outcome o1 = replayAndMeasure(a.history, a.planes);
        CHECK(writeBoxStepInches(live, 0, 0, 0, 50.8, 25.4, 25.4), "live fixture rewrite");
        Outcome o2 = replayAndMeasure(a.history, a.planes);
        CHECK(nearRel(totalVolume(o2), 2.0 * totalVolume(o1), 1e-6), "re-export not picked up: %g -> %g",
              totalVolume(o1), totalVolume(o2));
    }

    // Non-ASCII paths (stored as UTF-8).
    {
        const fs::path uni = dir / fs::u8path(u8"Ünïcødé 箱") / fs::u8path(u8"größe.step");
        fs::create_directories(uni.parent_path());
        CHECK(writeBoxStepInches(uni, 0, 0, 0, 25.4, 25.4, 25.4), "could not write a non-ASCII STEP path");
        CHECK(loadCadFile(utf8(uni), parts, info, err), "non-ASCII path: %s", err.c_str());
    }

    // A large part is tessellated coarsely enough to stay interactive: a
    // 770 mm cover took 1.7 million vertices at the old fixed 0.01 mm.
    {
        const fs::path big = dir / "big_disc.step";
        CHECK(writeStepShape(big, BRepPrimAPI_MakeCylinder(385.0, 12.0).Shape()), "big fixture");
        FeatureHistory h;
        std::vector<SketchPlane> planes = fixturePlanes();
        CadImportFeatureData cd;
        cd.sourcePath = utf8(big);
        h.addCadImportFeature(cd, "disc");
        Scene3D scene;
        replayFeatures(h, planes, scene);
        CHECK(scene.bodyCount() == 1, "disc bodies=%zu", scene.bodyCount());
        if (scene.bodyCount() == 1) {
            const Body3D& b = scene.getBody(0);
            CHECK(b.vertexCount > 0 && b.vertexCount < 60000, "disc tessellated into %d vertices", b.vertexCount);
            CHECK(b.edgeVertexCount < 20000, "disc edges %d vertices", b.edgeVertexCount);
        }
    }
}

// Booleans find their bodies by identity (source feature + index), not by
// position: a re-exported import with one more solid shifted every later body
// and made a Boolean subtract the wrong one, silently.
static void testBooleanBodyIdentity(const fs::path& dir) {
    std::printf("boolean body identity\n");
    // A: an import that later gains a solid. B: a 20 x 20 x 10 box that
    // overlaps the extruded cube by 10 x 20 x 10.
    const fs::path a = dir / "identity_a.step", b = dir / "identity_b.step";
    auto writeBoxes = [](const fs::path& path, int count) {
        BRep_Builder builder;
        TopoDS_Compound c;
        builder.MakeCompound(c);
        for (int i = 0; i < count; i++)
            builder.Add(c, BRepPrimAPI_MakeBox(gp_Pnt(500.0 + 100.0 * i, 0, 0), 10, 10, 10).Shape());
        return writeStepShape(path, c);
    };
    CHECK(writeBoxes(a, 1), "identity fixture A");
    CHECK(writeStepShape(b, BRepPrimAPI_MakeBox(gp_Pnt(10, 0, 0), 20, 20, 10).Shape()), "identity fixture B");

    auto build = [&](bool legacy, bool targetIsCube) {
        Fixture f;
        f.name = "identity";
        CadImportFeatureData ca;
        ca.sourcePath = utf8(a);
        FeatureID ia = f.history.addCadImportFeature(ca, "A");
        FeatureID sk = addSketch(f.history, f.planes, 0, rectSketch(0, 0, 20, 20));
        FeatureID ex = addExtrude(f.history, f.planes, sk, 0, 20.0f);   // cube: body 1
        CadImportFeatureData cb;
        cb.sourcePath = utf8(b);
        FeatureID ib = f.history.addCadImportFeature(cb, "B");          // body 2
        (void)ia;
        BooleanFeatureData bd;
        bd.operation = BooleanOperation::Subtract;
        const BodyRef cube{ex, 0}, box{ib, 0};
        if (!legacy) {
            bd.targetBody = targetIsCube ? cube : box;
            bd.toolBody = targetIsCube ? box : cube;
        }
        bd.targetBodyIndex = targetIsCube ? 1 : 2;
        bd.toolBodyIndex = targetIsCube ? 2 : 1;
        f.history.addBooleanFeature(bd);
        return f;
    };
    const double cubeMinusBox = 8000.0 - 10.0 * 20.0 * 10.0;   // 6000
    const double boxMinusCube = 4000.0 - 10.0 * 20.0 * 10.0;   // 2000
    const double aBox = 1000.0;

    for (bool legacy : {false, true}) {
        CHECK(writeBoxes(a, 1), "reset A");
        Fixture f = build(legacy, true);
        Outcome o1 = replayAndMeasure(f.history, f.planes);
        CHECK(o1.errors.empty(), "%s: %s", legacy ? "legacy" : "refs", joinErrors(o1).c_str());
        CHECK(nearRel(totalVolume(o1), aBox + cubeMinusBox, 1e-6), "%s: volume %g", legacy ? "legacy" : "refs", totalVolume(o1));
        const auto& bd = std::get<BooleanFeatureData>(f.history.features().back().data);
        CHECK(bd.targetBody.isSet() && bd.toolBody.isSet(), "%s: replay did not record the body identities",
              legacy ? "legacy" : "refs");

        // A is re-exported with a second solid: the cube and B move down the list.
        CHECK(writeBoxes(a, 2), "re-export A");
        Outcome o2 = replayAndMeasure(f.history, f.planes);
        CHECK(o2.errors.empty(), "%s after re-export: %s", legacy ? "legacy" : "refs", joinErrors(o2).c_str());
        CHECK(nearRel(totalVolume(o2), 2 * aBox + cubeMinusBox, 1e-6),
              "%s: after A gained a solid the Boolean hit the wrong bodies (volume %g, expected %g)",
              legacy ? "legacy" : "refs", totalVolume(o2), 2 * aBox + cubeMinusBox);
        const auto& bd2 = std::get<BooleanFeatureData>(f.history.features().back().data);
        CHECK(bd2.targetBodyIndex == 2 && bd2.toolBodyIndex == 3, "indices not kept current: %d, %d",
              bd2.targetBodyIndex, bd2.toolBodyIndex);
    }

    // The result is the target, even when the tool comes first in the list.
    {
        CHECK(writeBoxes(a, 1), "reset A");
        Fixture f = build(false, false); // target B (body 2), tool cube (body 1)
        Scene3D scene;
        replayFeatures(f.history, f.planes, scene);
        CHECK(!f.history.features().back().hasError, "box minus cube: %s", f.history.features().back().errorMsg.c_str());
        const FeatureID ib = f.history.features()[3].id;
        const int r = scene.findBody(ib, 0);
        CHECK(r >= 0, "the result lost the target's identity");
        if (r >= 0) {
            GProp_GProps props;
            BRepGProp::VolumeProperties(scene.getBody(r).shape, props);
            CHECK(nearRel(props.Mass(), boxMinusCube, 1e-6), "target volume %g", props.Mass());
        }
        CHECK(scene.findBody(f.history.features()[2].id, 0) < 0, "the tool body survived the Boolean");
    }

    // A body that is gone is an error, not a stand-in.
    {
        CHECK(writeBoxes(a, 1), "reset A");
        Fixture f = build(false, true);
        f.history.suppressFeature(f.history.features()[3].id); // B
        Outcome o = replayAndMeasure(f.history, f.planes);
        CHECK(o.errors.size() == 1 && o.errors[0].find("no longer exists") != std::string::npos,
              "missing tool body: %s", joinErrors(o).c_str());
    }

    // Identities survive save and load.
    {
        CHECK(writeBoxes(a, 1), "reset A");
        Fixture f = build(false, true);
        const fs::path pj = dir / "identity.shitcad";
        CHECK(saveProject(utf8(pj), f.history, f.planes, nullptr), "%s", lastLoadError().c_str());
        FeatureHistory lh;
        std::vector<SketchPlane> lp;
        CHECK(loadProject(utf8(pj), lh, lp, nullptr), "%s", lastLoadError().c_str());
        const auto& lbd = std::get<BooleanFeatureData>(lh.features().back().data);
        const auto& obd = std::get<BooleanFeatureData>(f.history.features().back().data);
        CHECK(lbd.targetBody == obd.targetBody && lbd.toolBody == obd.toolBody, "body identities lost on save/load");
    }
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
    const fs::path step = dir / "round_trip.step";
    CHECK(writeBoxStepInches(step, 0, 0, 0, 25.4, 50.8, 12.7), "round-trip STEP fixture");
    std::vector<std::function<Fixture()>> makers = {
        extrudeJoinCut, revolveTube, loftFrustum, booleanSubtract, suppressedCut, rolledBack,
        [&] { return meshWithSimulation(stl); },
        [&] { return cadImportFixture(step); },
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
        if (f.name == "cad_import")
            CHECK(readFile(p1).find("\"version\": 3") != std::string::npos, "a STEP import must save as version 3");
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

    testExtrudeJoinCut();
    testRevolve();
    testLoft();
    testBoolean();
    testSuppressAndRollback();
    testMeshAndSimulation(stl);
    testCadImport(dir);
    testBooleanBodyIdentity(dir);
    testRoundTrips(dir, stl);

    std::printf("\n%d checks, %d failed\n", g_checks, g_failures);
    return g_failures == 0 ? 0 : 1;
}
