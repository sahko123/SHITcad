#define NOMINMAX
#include <windows.h>
#include <commdlg.h>
#include <shlobj.h>

#include "Serialization.h"
#include "Utf8Path.h"
#include <nlohmann/json.hpp>
#include <fstream>
#include <sstream>
#include <cmath>
#include <StlAPI_Writer.hxx>
#include <Poly_Triangulation.hxx>
#include <gp_Pnt.hxx>
#include <BRep_Builder.hxx>
#include <TopoDS_Compound.hxx>
#include <STEPControl_Writer.hxx>
#include <STEPControl_Reader.hxx>
#include <IGESControl_Writer.hxx>
#include <IGESControl_Reader.hxx>
#include <Interface_Static.hxx>
#include <XSControl_WorkSession.hxx>

using json = nlohmann::json;

namespace shitcad {

static std::string s_lastError;

const std::string& lastLoadError() { return s_lastError; }

// ─── Enum conversions ───────────────────────────────────────────────

static const char* constraintTypeToStr(ConstraintType t) {
    switch (t) {
        case ConstraintType::Coincident:    return "Coincident";
        case ConstraintType::Horizontal:    return "Horizontal";
        case ConstraintType::Vertical:      return "Vertical";
        case ConstraintType::Distance:      return "Distance";
        case ConstraintType::Radius:        return "Radius";
        case ConstraintType::Diameter:      return "Diameter";
        case ConstraintType::PointDistance:  return "PointDistance";
        case ConstraintType::PointOnLine:   return "PointOnLine";
        case ConstraintType::EqualLength:   return "EqualLength";
        case ConstraintType::Perpendicular: return "Perpendicular";
        case ConstraintType::Parallel:      return "Parallel";
        case ConstraintType::Collinear:     return "Collinear";
        case ConstraintType::Tangent:       return "Tangent";
        case ConstraintType::Angle:         return "Angle";
        case ConstraintType::Symmetric:     return "Symmetric";
        case ConstraintType::Concentric:    return "Concentric";
        case ConstraintType::Midpoint:      return "Midpoint";
        case ConstraintType::PointLineDistance: return "PointLineDistance";
        case ConstraintType::PointOnCircle:     return "PointOnCircle";
    }
    return "Coincident";
}

static ConstraintType constraintTypeFromStr(const std::string& s) {
    if (s == "Coincident")    return ConstraintType::Coincident;
    if (s == "Horizontal")    return ConstraintType::Horizontal;
    if (s == "Vertical")      return ConstraintType::Vertical;
    if (s == "Distance")      return ConstraintType::Distance;
    if (s == "Radius")        return ConstraintType::Radius;
    if (s == "Diameter")      return ConstraintType::Diameter;
    if (s == "PointDistance")  return ConstraintType::PointDistance;
    if (s == "PointOnLine")   return ConstraintType::PointOnLine;
    if (s == "EqualLength")   return ConstraintType::EqualLength;
    if (s == "Perpendicular") return ConstraintType::Perpendicular;
    if (s == "Parallel")      return ConstraintType::Parallel;
    if (s == "Collinear")     return ConstraintType::Collinear;
    if (s == "Tangent")       return ConstraintType::Tangent;
    if (s == "Angle")         return ConstraintType::Angle;
    if (s == "Symmetric")     return ConstraintType::Symmetric;
    if (s == "Concentric")    return ConstraintType::Concentric;
    if (s == "Midpoint")      return ConstraintType::Midpoint;
    if (s == "PointLineDistance") return ConstraintType::PointLineDistance;
    if (s == "PointOnCircle")     return ConstraintType::PointOnCircle;
    return ConstraintType::Coincident;
}

static const char* featureTypeToStr(FeatureType t) {
    switch (t) {
        case FeatureType::Sketch:  return "Sketch";
        case FeatureType::Extrude: return "Extrude";
        case FeatureType::Revolve: return "Revolve";
        case FeatureType::Loft:    return "Loft";
        case FeatureType::Boolean: return "Boolean";
        case FeatureType::MeshImport: return "MeshImport";
    }
    return "Sketch";
}

static FeatureType featureTypeFromStr(const std::string& s) {
    if (s == "Extrude") return FeatureType::Extrude;
    if (s == "Revolve") return FeatureType::Revolve;
    if (s == "Loft")    return FeatureType::Loft;
    if (s == "Boolean") return FeatureType::Boolean;
    if (s == "MeshImport") return FeatureType::MeshImport;
    return FeatureType::Sketch;
}

static const char* extrudeOpToStr(ExtrudeOperation op) {
    switch (op) {
        case ExtrudeOperation::NewBody: return "NewBody";
        case ExtrudeOperation::Cut:     return "Cut";
    }
    return "NewBody";
}

static ExtrudeOperation extrudeOpFromStr(const std::string& s) {
    if (s == "Cut") return ExtrudeOperation::Cut;
    return ExtrudeOperation::NewBody;
}

static const char* extrudeDirToStr(ExtrudeDirection d) {
    switch (d) {
        case ExtrudeDirection::OneSide:   return "OneSide";
        case ExtrudeDirection::OtherSide: return "OtherSide";
        case ExtrudeDirection::BothSides: return "BothSides";
        case ExtrudeDirection::Symmetric: return "Symmetric";
    }
    return "OneSide";
}

static ExtrudeDirection extrudeDirFromStr(const std::string& s) {
    if (s == "OtherSide") return ExtrudeDirection::OtherSide;
    if (s == "BothSides") return ExtrudeDirection::BothSides;
    if (s == "Symmetric") return ExtrudeDirection::Symmetric;
    return ExtrudeDirection::OneSide;
}

// ─── Sketch entity serialization ────────────────────────────────────

static json pointToJson(const PointEntity& p) {
    json j = {{"id", p.id}, {"x", p.x}, {"y", p.y}};
    if (p.projected) j["projected"] = true;
    return j;
}

static PointEntity pointFromJson(const json& j) {
    PointEntity p;
    p.id = j.at("id").get<EntityID>();
    p.x = j.at("x").get<double>();
    p.y = j.at("y").get<double>();
    p.projected = j.value("projected", false);
    return p;
}

static json lineToJson(const LineEntity& l) {
    json j = {{"id", l.id}, {"startPt", l.startPt}, {"endPt", l.endPt}};
    if (l.projected) j["projected"] = true;
    return j;
}

static LineEntity lineFromJson(const json& j) {
    LineEntity l;
    l.id = j.at("id").get<EntityID>();
    l.startPt = j.at("startPt").get<EntityID>();
    l.endPt = j.at("endPt").get<EntityID>();
    l.projected = j.value("projected", false);
    return l;
}

static json circleToJson(const CircleEntity& c) {
    json j = {{"id", c.id}, {"centerPt", c.centerPt}, {"radius", c.radius}};
    if (c.projected) j["projected"] = true;
    return j;
}

static CircleEntity circleFromJson(const json& j) {
    CircleEntity c;
    c.id = j.at("id").get<EntityID>();
    c.centerPt = j.at("centerPt").get<EntityID>();
    c.radius = j.at("radius").get<double>();
    c.projected = j.value("projected", false);
    return c;
}

static json arcToJson(const ArcEntity& a) {
    json j = {{"id", a.id}, {"centerPt", a.centerPt}, {"startPt", a.startPt},
              {"endPt", a.endPt}, {"startAngle", a.startAngle}, {"endAngle", a.endAngle}};
    if (a.projected) j["projected"] = true;
    return j;
}

static ArcEntity arcFromJson(const json& j) {
    ArcEntity a;
    a.id = j.at("id").get<EntityID>();
    a.centerPt = j.at("centerPt").get<EntityID>();
    a.startPt = j.at("startPt").get<EntityID>();
    a.endPt = j.at("endPt").get<EntityID>();
    a.startAngle = j.at("startAngle").get<double>();
    a.endAngle = j.at("endAngle").get<double>();
    a.projected = j.value("projected", false);
    return a;
}

static json ellipseToJson(const EllipseEntity& e) {
    json j = {{"id", e.id}, {"centerPt", e.centerPt},
              {"semiMajor", e.semiMajor}, {"semiMinor", e.semiMinor},
              {"rotation", e.rotation}};
    if (e.projected) j["projected"] = true;
    return j;
}

static EllipseEntity ellipseFromJson(const json& j) {
    EllipseEntity e;
    e.id = j.at("id").get<EntityID>();
    e.centerPt = j.at("centerPt").get<EntityID>();
    e.semiMajor = j.at("semiMajor").get<double>();
    e.semiMinor = j.at("semiMinor").get<double>();
    e.rotation = j.at("rotation").get<double>();
    e.projected = j.value("projected", false);
    return e;
}

static json ellipseArcToJson(const EllipseArcEntity& e) {
    json j = {{"id", e.id}, {"centerPt", e.centerPt},
              {"startPt", e.startPt}, {"endPt", e.endPt},
              {"semiMajor", e.semiMajor}, {"semiMinor", e.semiMinor},
              {"rotation", e.rotation},
              {"startAngle", e.startAngle}, {"endAngle", e.endAngle}};
    if (e.projected) j["projected"] = true;
    return j;
}

static EllipseArcEntity ellipseArcFromJson(const json& j) {
    EllipseArcEntity e;
    e.id = j.at("id").get<EntityID>();
    e.centerPt = j.at("centerPt").get<EntityID>();
    e.startPt = j.at("startPt").get<EntityID>();
    e.endPt = j.at("endPt").get<EntityID>();
    e.semiMajor = j.at("semiMajor").get<double>();
    e.semiMinor = j.at("semiMinor").get<double>();
    e.rotation = j.at("rotation").get<double>();
    e.startAngle = j.at("startAngle").get<double>();
    e.endAngle = j.at("endAngle").get<double>();
    e.projected = j.value("projected", false);
    return e;
}

static json splineToJson(const SplineEntity& s) {
    json j = {{"id", s.id}, {"degree", s.degree}, {"periodic", s.periodic}};
    j["controlPtIDs"] = json::array();
    for (auto id : s.controlPtIDs) j["controlPtIDs"].push_back(id);
    if (!s.knots.empty()) j["knots"] = s.knots;
    if (!s.weights.empty()) j["weights"] = s.weights;
    if (s.projected) j["projected"] = true;
    return j;
}

static SplineEntity splineFromJson(const json& j) {
    SplineEntity s;
    s.id = j.at("id").get<EntityID>();
    s.degree = j.at("degree").get<int>();
    s.periodic = j.at("periodic").get<bool>();
    for (auto& id : j.at("controlPtIDs")) s.controlPtIDs.push_back(id.get<EntityID>());
    if (j.contains("knots")) s.knots = j["knots"].get<std::vector<double>>();
    if (j.contains("weights")) s.weights = j["weights"].get<std::vector<double>>();
    s.projected = j.value("projected", false);
    return s;
}

static json constraintToJson(const Constraint& c) {
    json j;
    j["id"] = c.id;
    j["type"] = constraintTypeToStr(c.type);
    j["entityA"] = c.entityA;
    j["entityB"] = c.entityB;
    j["value"] = c.value;
    j["isAuto"] = c.isAuto;
    j["inputUnit"] = c.inputUnit;
    j["inputValue"] = c.inputValue;
    j["dimOffsetX"] = c.dimOffsetX;
    j["dimOffsetY"] = c.dimOffsetY;
    j["driven"] = c.driven;
    j["angleCW"] = c.angleCW;
    j["negativeSide"] = c.negativeSide;
    if (c.entityC != NullID) j["entityC"] = c.entityC;
    return j;
}

static Constraint constraintFromJson(const json& j) {
    Constraint c;
    c.id = j.at("id").get<EntityID>();
    c.type = constraintTypeFromStr(j.at("type").get<std::string>());
    c.entityA = j.at("entityA").get<EntityID>();
    c.entityB = j.at("entityB").get<EntityID>();
    c.value = j.at("value").get<double>();
    c.isAuto = j.value("isAuto", false);
    c.inputUnit = j.value("inputUnit", std::string{});
    c.inputValue = j.value("inputValue", 0.0);
    c.dimOffsetX = j.value("dimOffsetX", 0.0);
    c.dimOffsetY = j.value("dimOffsetY", 0.0);
    c.driven = j.value("driven", false);
    c.angleCW = j.value("angleCW", false);
    c.negativeSide = j.value("negativeSide", false);
    // Backwards compat: old files stored PointLineDistance side in angleCW
    if (c.type == ConstraintType::PointLineDistance && !j.contains("negativeSide")) {
        c.negativeSide = c.angleCW;
    }
    c.entityC = j.value("entityC", (EntityID)NullID);
    return c;
}

// ─── Sketch ─────────────────────────────────────────────────────────

static json sketchToJson(const Sketch& s) {
    json j;
    j["nextID"] = s.nextID;

    j["points"] = json::array();
    for (auto& p : s.points) j["points"].push_back(pointToJson(p));

    j["lines"] = json::array();
    for (auto& l : s.lines) j["lines"].push_back(lineToJson(l));

    j["circles"] = json::array();
    for (auto& c : s.circles) j["circles"].push_back(circleToJson(c));

    j["arcs"] = json::array();
    for (auto& a : s.arcs) j["arcs"].push_back(arcToJson(a));

    j["ellipses"] = json::array();
    for (auto& e : s.ellipses) j["ellipses"].push_back(ellipseToJson(e));

    j["ellipseArcs"] = json::array();
    for (auto& e : s.ellipseArcs) j["ellipseArcs"].push_back(ellipseArcToJson(e));

    j["splines"] = json::array();
    for (auto& sp : s.splines) j["splines"].push_back(splineToJson(sp));

    j["constraints"] = json::array();
    for (auto& c : s.constraints) j["constraints"].push_back(constraintToJson(c));

    return j;
}

static Sketch sketchFromJson(const json& j) {
    Sketch s;
    s.nextID = j.at("nextID").get<EntityID>();

    for (auto& p : j.at("points")) s.points.push_back(pointFromJson(p));
    for (auto& l : j.at("lines")) s.lines.push_back(lineFromJson(l));
    for (auto& c : j.at("circles")) s.circles.push_back(circleFromJson(c));
    for (auto& a : j.at("arcs")) s.arcs.push_back(arcFromJson(a));
    if (j.contains("ellipses"))
        for (auto& e : j["ellipses"]) s.ellipses.push_back(ellipseFromJson(e));
    if (j.contains("ellipseArcs"))
        for (auto& e : j["ellipseArcs"]) s.ellipseArcs.push_back(ellipseArcFromJson(e));
    if (j.contains("splines"))
        for (auto& sp : j["splines"]) s.splines.push_back(splineFromJson(sp));
    for (auto& c : j.at("constraints")) s.constraints.push_back(constraintFromJson(c));

    return s;
}

// ─── ProfileSignature ───────────────────────────────────────────────

static json profileSigToJson(const ProfileSignature& ps) {
    json j;
    j["lineIDs"] = json::array();
    for (auto id : ps.lineIDs) j["lineIDs"].push_back(id);
    j["arcIDs"] = json::array();
    for (auto id : ps.arcIDs) j["arcIDs"].push_back(id);
    j["circleID"] = ps.circleID;
    j["centroidX"] = ps.centroidX;
    j["centroidY"] = ps.centroidY;
    if (!ps.ellipseIDs.empty()) {
        j["ellipseIDs"] = json::array();
        for (auto id : ps.ellipseIDs) j["ellipseIDs"].push_back(id);
    }
    if (!ps.splineIDs.empty()) {
        j["splineIDs"] = json::array();
        for (auto id : ps.splineIDs) j["splineIDs"].push_back(id);
    }
    if (ps.ellipseID != NullID) j["ellipseID"] = ps.ellipseID;
    return j;
}

static ProfileSignature profileSigFromJson(const json& j) {
    ProfileSignature ps;
    for (auto& id : j.at("lineIDs")) ps.lineIDs.insert(id.get<EntityID>());
    for (auto& id : j.at("arcIDs")) ps.arcIDs.insert(id.get<EntityID>());
    ps.circleID = j.at("circleID").get<EntityID>();
    if (j.contains("centroidX")) ps.centroidX = j["centroidX"].get<double>();
    if (j.contains("centroidY")) ps.centroidY = j["centroidY"].get<double>();
    if (j.contains("ellipseIDs"))
        for (auto& id : j["ellipseIDs"]) ps.ellipseIDs.insert(id.get<EntityID>());
    if (j.contains("splineIDs"))
        for (auto& id : j["splineIDs"]) ps.splineIDs.insert(id.get<EntityID>());
    ps.ellipseID = j.value("ellipseID", (EntityID)NullID);
    return ps;
}

// ─── Feature data ───────────────────────────────────────────────────

static json sketchFeatureDataToJson(const SketchFeatureData& sd) {
    json j;
    j["sketchPlaneIndex"] = sd.sketchPlaneIndex;
    j["sketchPlaneID"] = sd.sketchPlaneID;
    j["sketchSnapshot"] = sketchToJson(sd.sketchSnapshot);
    return j;
}

static SketchFeatureData sketchFeatureDataFromJson(const json& j) {
    SketchFeatureData sd;
    sd.sketchPlaneIndex = j.at("sketchPlaneIndex").get<int>();
    sd.sketchPlaneID = j.value("sketchPlaneID", (PlaneID)NullPlaneID);
    sd.sketchSnapshot = sketchFromJson(j.at("sketchSnapshot"));
    return sd;
}

static json extrudeFeatureDataToJson(const ExtrudeFeatureData& ed) {
    json j;
    j["sourceSketchFeature"] = ed.sourceSketchFeature;
    j["height"] = ed.height;
    j["offset"] = ed.offset;
    j["operation"] = extrudeOpToStr(ed.operation);
    j["direction"] = extrudeDirToStr(ed.direction);

    j["profileSigs"] = json::array();
    for (auto& ps : ed.profileSigs) j["profileSigs"].push_back(profileSigToJson(ps));

    j["profileIndicesFallback"] = ed.profileIndicesFallback;
    return j;
}

static ExtrudeFeatureData extrudeFeatureDataFromJson(const json& j) {
    ExtrudeFeatureData ed;
    ed.sourceSketchFeature = j.at("sourceSketchFeature").get<FeatureID>();
    ed.height = j.at("height").get<float>();
    ed.offset = j.at("offset").get<float>();
    ed.operation = extrudeOpFromStr(j.at("operation").get<std::string>());
    ed.direction = extrudeDirFromStr(j.at("direction").get<std::string>());

    for (auto& ps : j.at("profileSigs")) ed.profileSigs.push_back(profileSigFromJson(ps));
    ed.profileIndicesFallback = j.at("profileIndicesFallback").get<std::vector<int>>();
    return ed;
}

// ─── Revolve feature data ────────────────────────────────────────────

static json revolveFeatureDataToJson(const RevolveFeatureData& rd) {
    json j;
    j["sourceSketchFeature"] = rd.sourceSketchFeature;
    j["axisLineID"] = rd.axisLineID;
    j["angleDeg"] = rd.angleDeg;
    j["operation"] = extrudeOpToStr(rd.operation);

    j["profileSigs"] = json::array();
    for (auto& ps : rd.profileSigs) j["profileSigs"].push_back(profileSigToJson(ps));

    j["profileIndicesFallback"] = rd.profileIndicesFallback;
    return j;
}

static RevolveFeatureData revolveFeatureDataFromJson(const json& j) {
    RevolveFeatureData rd;
    rd.sourceSketchFeature = j.at("sourceSketchFeature").get<FeatureID>();
    rd.axisLineID = j.at("axisLineID").get<EntityID>();
    rd.angleDeg = j.at("angleDeg").get<float>();
    rd.operation = extrudeOpFromStr(j.at("operation").get<std::string>());

    for (auto& ps : j.at("profileSigs")) rd.profileSigs.push_back(profileSigFromJson(ps));
    rd.profileIndicesFallback = j.at("profileIndicesFallback").get<std::vector<int>>();
    return rd;
}

// ─── Loft feature data ──────────────────────────────────────────────

static json loftSectionToJson(const LoftSection& ls) {
    json j;
    j["sourceSketchFeature"] = ls.sourceSketchFeature;
    j["profileSig"] = profileSigToJson(ls.profileSig);
    j["profileIndexFallback"] = ls.profileIndexFallback;
    return j;
}

static LoftSection loftSectionFromJson(const json& j) {
    LoftSection ls;
    ls.sourceSketchFeature = j.at("sourceSketchFeature").get<FeatureID>();
    ls.profileSig = profileSigFromJson(j.at("profileSig"));
    ls.profileIndexFallback = j.at("profileIndexFallback").get<int>();
    return ls;
}

static json loftFeatureDataToJson(const LoftFeatureData& ld) {
    json j;
    j["operation"] = extrudeOpToStr(ld.operation);
    j["solid"] = ld.solid;
    j["sections"] = json::array();
    for (const auto& sec : ld.sections) j["sections"].push_back(loftSectionToJson(sec));
    return j;
}

static LoftFeatureData loftFeatureDataFromJson(const json& j) {
    LoftFeatureData ld;
    ld.operation = extrudeOpFromStr(j.at("operation").get<std::string>());
    ld.solid = j.value("solid", true);
    for (const auto& sj : j.at("sections")) ld.sections.push_back(loftSectionFromJson(sj));
    return ld;
}

static json booleanFeatureDataToJson(const BooleanFeatureData& bd) {
    json j;
    j["operation"] = (bd.operation == BooleanOperation::Subtract) ? "Subtract" : "Union";
    j["targetBodyIndex"] = bd.targetBodyIndex;
    j["toolBodyIndex"] = bd.toolBodyIndex;
    return j;
}

static BooleanFeatureData booleanFeatureDataFromJson(const json& j) {
    BooleanFeatureData bd;
    std::string op = j.value("operation", "Union");
    bd.operation = (op == "Subtract") ? BooleanOperation::Subtract : BooleanOperation::Union;
    bd.targetBodyIndex = j.value("targetBodyIndex", -1);
    bd.toolBodyIndex = j.value("toolBodyIndex", -1);
    return bd;
}

static json meshImportFeatureDataToJson(const MeshImportFeatureData& md) {
    json j;
    j["sourcePath"] = md.sourcePath;
    j["unit"] = md.unit;
    if (!md.transform.isIdentity()) {
        j["transform"] = {
            {"rotation", std::vector<double>(md.transform.r, md.transform.r + 9)},
            {"translationMm", std::vector<double>(md.transform.t, md.transform.t + 3)},
        };
    }
    return j;
}

static MeshImportFeatureData meshImportFeatureDataFromJson(const json& j) {
    MeshImportFeatureData md;
    md.sourcePath = j.at("sourcePath").get<std::string>();
    // No default: guessing the unit of a mesh is exactly the mistake this
    // field exists to prevent. A file without it is invalid.
    md.unit = j.at("unit").get<std::string>();
    // Optional: imports saved before placement existed load as identity.
    if (j.contains("transform")) {
        const auto& xj = j.at("transform");
        auto rot = xj.at("rotation").get<std::vector<double>>();
        auto tr = xj.at("translationMm").get<std::vector<double>>();
        if (rot.size() != 9 || tr.size() != 3)
            throw json::other_error::create(501, "MeshImport transform must have 9 rotation and 3 translation values", &xj);
        for (int i = 0; i < 9; i++) md.transform.r[i] = rot[i];
        for (int i = 0; i < 3; i++) md.transform.t[i] = tr[i];
    }
    return md;
}

// ─── Feature ────────────────────────────────────────────────────────

static json featureToJson(const Feature& f) {
    json j;
    j["id"] = f.id;
    j["type"] = featureTypeToStr(f.type);
    j["name"] = f.name;
    j["suppressed"] = f.suppressed;

    if (f.type == FeatureType::Sketch) {
        j["data"] = sketchFeatureDataToJson(std::get<SketchFeatureData>(f.data));
    } else if (f.type == FeatureType::Extrude) {
        j["data"] = extrudeFeatureDataToJson(std::get<ExtrudeFeatureData>(f.data));
    } else if (f.type == FeatureType::Revolve) {
        j["data"] = revolveFeatureDataToJson(std::get<RevolveFeatureData>(f.data));
    } else if (f.type == FeatureType::Loft) {
        j["data"] = loftFeatureDataToJson(std::get<LoftFeatureData>(f.data));
    } else if (f.type == FeatureType::Boolean) {
        j["data"] = booleanFeatureDataToJson(std::get<BooleanFeatureData>(f.data));
    } else if (f.type == FeatureType::MeshImport) {
        j["data"] = meshImportFeatureDataToJson(std::get<MeshImportFeatureData>(f.data));
    }
    return j;
}

static Feature featureFromJson(const json& j) {
    Feature f;
    f.id = j.at("id").get<FeatureID>();
    f.type = featureTypeFromStr(j.at("type").get<std::string>());
    f.name = j.at("name").get<std::string>();
    f.suppressed = j.value("suppressed", false);

    if (f.type == FeatureType::Sketch) {
        f.data = sketchFeatureDataFromJson(j.at("data"));
    } else if (f.type == FeatureType::Extrude) {
        f.data = extrudeFeatureDataFromJson(j.at("data"));
    } else if (f.type == FeatureType::Revolve) {
        f.data = revolveFeatureDataFromJson(j.at("data"));
    } else if (f.type == FeatureType::Loft) {
        f.data = loftFeatureDataFromJson(j.at("data"));
    } else if (f.type == FeatureType::Boolean) {
        f.data = booleanFeatureDataFromJson(j.at("data"));
    } else if (f.type == FeatureType::MeshImport) {
        f.data = meshImportFeatureDataFromJson(j.at("data"));
    }
    return f;
}

// ─── SketchPlane ────────────────────────────────────────────────────

static json sketchPlaneToJson(const SketchPlane& sp) {
    json j;
    j["origin"] = {sp.origin[0], sp.origin[1], sp.origin[2]};
    j["normal"] = {sp.normal[0], sp.normal[1], sp.normal[2]};
    j["uAxis"] = {sp.uAxis[0], sp.uAxis[1], sp.uAxis[2]};
    j["vAxis"] = {sp.vAxis[0], sp.vAxis[1], sp.vAxis[2]};
    j["name"] = sp.name;
    j["color"] = {sp.color[0], sp.color[1], sp.color[2], sp.color[3]};
    j["sourceBodyIndex"] = sp.sourceBodyIndex;
    j["visible"] = sp.visible;
    j["sketchVisible"] = sp.sketchVisible;
    j["isReferencePlane"] = sp.isReferencePlane;
    j["planeID"] = sp.planeID;
    // Note: sketch data is NOT saved here; it's rebuilt by replayFeatures
    return j;
}

static SketchPlane sketchPlaneFromJson(const json& j) {
    SketchPlane sp;
    auto o = j.at("origin");
    sp.origin[0] = o[0]; sp.origin[1] = o[1]; sp.origin[2] = o[2];
    auto n = j.at("normal");
    sp.normal[0] = n[0]; sp.normal[1] = n[1]; sp.normal[2] = n[2];
    auto u = j.at("uAxis");
    sp.uAxis[0] = u[0]; sp.uAxis[1] = u[1]; sp.uAxis[2] = u[2];
    auto v = j.at("vAxis");
    sp.vAxis[0] = v[0]; sp.vAxis[1] = v[1]; sp.vAxis[2] = v[2];
    sp.name = j.at("name").get<std::string>();
    auto c = j.at("color");
    sp.color[0] = c[0]; sp.color[1] = c[1]; sp.color[2] = c[2]; sp.color[3] = c[3];
    sp.sourceBodyIndex = j.value("sourceBodyIndex", -1);
    sp.visible = j.value("visible", true);
    sp.sketchVisible = j.value("sketchVisible", true);
    sp.isReferencePlane = j.value("isReferencePlane", false);
    sp.planeID = j.value("planeID", (PlaneID)NullPlaneID);
    return sp;
}

// ─── FeatureHistory ─────────────────────────────────────────────────

static json featureHistoryToJson(const FeatureHistory& h) {
    json j;
    j["nextID"] = h.nextID();
    j["rollbackPos"] = h.rollbackPos();
    j["sketchCounter"] = h.sketchCounter();
    j["extrudeCounter"] = h.extrudeCounter();
    j["revolveCounter"] = h.revolveCounter();
    j["loftCounter"] = h.loftCounter();
    j["booleanCounter"] = h.booleanCounter();

    j["features"] = json::array();
    for (auto& f : h.features()) j["features"].push_back(featureToJson(f));

    return j;
}

static void featureHistoryFromJson(const json& j, FeatureHistory& h) {
    h.clear();
    h.setNextID(j.at("nextID").get<FeatureID>());
    h.setRollbackPos(j.value("rollbackPos", -1));
    h.setSketchCounter(j.at("sketchCounter").get<int>());
    h.setExtrudeCounter(j.at("extrudeCounter").get<int>());
    h.setRevolveCounter(j.value("revolveCounter", 0));
    h.setLoftCounter(j.value("loftCounter", 0));
    h.setBooleanCounter(j.value("booleanCounter", 0));

    for (auto& fj : j.at("features")) {
        Feature f = featureFromJson(fj);
        h.insertFeatureAt((int)h.size(), f);
    }
}

// ─── Top-level save/load ────────────────────────────────────────────

bool saveProject(const std::string& filepath,
                 const FeatureHistory& history,
                 const std::vector<SketchPlane>& planes,
                 const SimulationSetup* simulation) {
    // Version 2 = may contain MeshImport features or a simulation set-up. Only
    // written when it does, so projects without either still open in older
    // builds. An older build refuses a v2 file with a clear message, rather
    // than failing on an unknown feature type or - worse, for the simulation
    // block it would not read - silently dropping it on the next save.
    bool needsV2 = simulation && !simulation->empty();
    for (const auto& f : history.features()) {
        if (f.type == FeatureType::MeshImport) { needsV2 = true; break; }
    }

    json doc;
    doc["version"] = needsV2 ? 2 : 1;
    doc["app"] = "SHITcad";
    doc["featureHistory"] = featureHistoryToJson(history);
    if (simulation && !simulation->empty()) {
        json sj;
        simulationToJson(*simulation, sj);
        doc["simulation"] = sj;
    }

    doc["sketchPlanes"] = json::array();
    for (auto& sp : planes) doc["sketchPlanes"].push_back(sketchPlaneToJson(sp));

    std::ofstream out(fsPath(filepath));
    if (!out.is_open()) {
        s_lastError = "Could not open file for writing: " + filepath;
        return false;
    }

    out << doc.dump(2, ' ', false, json::error_handler_t::replace);
    if (out.fail()) {
        s_lastError = "Write error";
        return false;
    }

    return true;
}

bool loadProject(const std::string& filepath,
                 FeatureHistory& history,
                 std::vector<SketchPlane>& planes,
                 SimulationSetup* simulation) {
    std::ifstream in(fsPath(filepath));
    if (!in.is_open()) {
        s_lastError = "Could not open file: " + filepath;
        return false;
    }

    json doc;
    try {
        doc = json::parse(in);
    } catch (const json::parse_error& e) {
        s_lastError = std::string("JSON parse error: ") + e.what();
        return false;
    }

    try {
        int version = doc.at("version").get<int>();
        if (version > 2) {
            s_lastError = "File was created with a newer version of SHITcad (version " +
                          std::to_string(version) + ")";
            return false;
        }

        // Deserialize into temporaries
        FeatureHistory tempHistory;
        featureHistoryFromJson(doc.at("featureHistory"), tempHistory);

        std::vector<SketchPlane> tempPlanes;
        for (auto& sp : doc.at("sketchPlanes")) {
            tempPlanes.push_back(sketchPlaneFromJson(sp));
        }

        SimulationSetup tempSim;
        if (doc.contains("simulation")) {
            std::string simErr;
            if (!simulationFromJson(doc.at("simulation"), tempSim, simErr)) {
                s_lastError = "Invalid simulation set-up: " + simErr;
                return false;
            }
        }

        // All succeeded — move into output
        history = std::move(tempHistory);
        planes = std::move(tempPlanes);
        if (simulation) *simulation = std::move(tempSim);

    } catch (const json::exception& e) {
        s_lastError = std::string("Invalid file format: ") + e.what();
        return false;
    }

    return true;
}

// ─── Native file dialogs ────────────────────────────────────────────

std::string openNativeOpenDialog() {
    char filename[MAX_PATH] = {};
    OPENFILENAMEA ofn = {};
    ofn.lStructSize = sizeof(ofn);
    ofn.lpstrFilter = "SHITcad Files (*.shitcad)\0*.shitcad\0All Files\0*.*\0";
    ofn.lpstrFile = filename;
    ofn.nMaxFile = MAX_PATH;
    ofn.Flags = OFN_FILEMUSTEXIST | OFN_NOCHANGEDIR;
    if (GetOpenFileNameA(&ofn)) return ansiToUtf8(filename);
    return {};
}

std::string openNativeSaveDialog() {
    char filename[MAX_PATH] = {};
    OPENFILENAMEA ofn = {};
    ofn.lStructSize = sizeof(ofn);
    ofn.lpstrFilter = "SHITcad Files (*.shitcad)\0*.shitcad\0All Files\0*.*\0";
    ofn.lpstrFile = filename;
    ofn.nMaxFile = MAX_PATH;
    ofn.lpstrDefExt = "shitcad";
    ofn.Flags = OFN_OVERWRITEPROMPT | OFN_NOCHANGEDIR;
    if (GetSaveFileNameA(&ofn)) return ansiToUtf8(filename);
    return {};
}

std::string openNativeStlSaveDialog() {
    char filename[MAX_PATH] = {};
    OPENFILENAMEA ofn = {};
    ofn.lStructSize = sizeof(ofn);
    ofn.lpstrFilter = "STL Files (*.stl)\0*.stl\0All Files\0*.*\0";
    ofn.lpstrFile = filename;
    ofn.nMaxFile = MAX_PATH;
    ofn.lpstrDefExt = "stl";
    ofn.Flags = OFN_OVERWRITEPROMPT | OFN_NOCHANGEDIR;
    if (GetSaveFileNameA(&ofn)) return ansiToUtf8(filename);
    return {};
}

bool exportSTL(const std::string& filepath, const Scene3D& scene) {
    if (scene.bodyCount() == 0) {
        s_lastError = "No bodies to export";
        return false;
    }

    // Combine all visible bodies into a compound
    BRep_Builder builder;
    TopoDS_Compound compound;
    builder.MakeCompound(compound);
    int count = 0;
    for (size_t i = 0; i < scene.bodyCount(); i++) {
        const auto& body = scene.getBody((int)i);
        if (!body.visible || body.shape.IsNull()) continue;
        builder.Add(compound, body.shape);
        count++;
    }
    if (count == 0) {
        s_lastError = "No visible bodies to export";
        return false;
    }

    StlAPI_Writer writer;
    writer.ASCIIMode() = false; // binary STL (smaller files)
    if (!writer.Write(compound, filepath.c_str())) {
        s_lastError = "StlAPI_Writer failed to write file";
        return false;
    }
    return true;
}

// Helper: build compound of all visible bodies
static bool buildVisibleCompound(const Scene3D& scene, TopoDS_Compound& compound) {
    BRep_Builder builder;
    builder.MakeCompound(compound);
    int count = 0;
    for (size_t i = 0; i < scene.bodyCount(); i++) {
        const auto& body = scene.getBody((int)i);
        if (!body.visible || body.shape.IsNull()) continue;
        builder.Add(compound, body.shape);
        count++;
    }
    if (count == 0) {
        s_lastError = "No visible bodies to export";
        return false;
    }
    return true;
}

// ─── STEP export/import ─────────────────────────────────────────────

bool exportSTEP(const std::string& filepath, const Scene3D& scene) {
    if (scene.bodyCount() == 0) {
        s_lastError = "No bodies to export";
        return false;
    }

    TopoDS_Compound compound;
    if (!buildVisibleCompound(scene, compound)) return false;

    Interface_Static::SetCVal("xstep.cascade.unit", "MM");
    Interface_Static::SetCVal("write.step.unit", "MM");

    STEPControl_Writer writer;
    IFSelect_ReturnStatus status = writer.Transfer(compound, STEPControl_AsIs);
    if (status != IFSelect_RetDone) {
        s_lastError = "STEP transfer failed";
        return false;
    }
    status = writer.Write(filepath.c_str());
    if (status != IFSelect_RetDone) {
        s_lastError = "STEP write failed";
        return false;
    }
    return true;
}

bool importSTEP(const std::string& filepath, Scene3D& scene) {
    STEPControl_Reader reader;
    IFSelect_ReturnStatus status = reader.ReadFile(filepath.c_str());
    if (status != IFSelect_RetDone) {
        s_lastError = "Failed to read STEP file";
        return false;
    }

    reader.TransferRoots();
    int nbShapes = reader.NbShapes();
    if (nbShapes == 0) {
        s_lastError = "STEP file contains no shapes";
        return false;
    }

    for (int i = 1; i <= nbShapes; i++) {
        TopoDS_Shape shape = reader.Shape(i);
        if (!shape.IsNull()) {
            scene.addBody(shape);
        }
    }
    return true;
}

// ─── IGES export/import ─────────────────────────────────────────────

bool exportIGES(const std::string& filepath, const Scene3D& scene) {
    if (scene.bodyCount() == 0) {
        s_lastError = "No bodies to export";
        return false;
    }

    TopoDS_Compound compound;
    if (!buildVisibleCompound(scene, compound)) return false;

    IGESControl_Writer writer("MM", 0);
    writer.AddShape(compound);
    writer.ComputeModel();
    if (!writer.Write(filepath.c_str())) {
        s_lastError = "IGES write failed";
        return false;
    }
    return true;
}

bool importIGES(const std::string& filepath, Scene3D& scene) {
    IGESControl_Reader reader;
    IFSelect_ReturnStatus status = reader.ReadFile(filepath.c_str());
    if (status != IFSelect_RetDone) {
        s_lastError = "Failed to read IGES file";
        return false;
    }

    reader.TransferRoots();
    int nbShapes = reader.NbShapes();
    if (nbShapes == 0) {
        s_lastError = "IGES file contains no shapes";
        return false;
    }

    for (int i = 1; i <= nbShapes; i++) {
        TopoDS_Shape shape = reader.Shape(i);
        if (!shape.IsNull()) {
            scene.addBody(shape);
        }
    }
    return true;
}

// ─── OBJ export ─────────────────────────────────────────────────────

bool exportOBJ(const std::string& filepath, const Scene3D& scene) {
    if (scene.bodyCount() == 0) {
        s_lastError = "No bodies to export";
        return false;
    }

    std::ofstream out(fsPath(filepath));
    if (!out.is_open()) {
        s_lastError = "Cannot open file for writing";
        return false;
    }

    out << "# Exported by SHITcad\n";

    int vertexOffset = 0;
    int normalOffset = 0;
    int bodyIdx = 0;

    for (size_t i = 0; i < scene.bodyCount(); i++) {
        const auto& body = scene.getBody((int)i);
        if (!body.visible || body.vertices.empty()) continue;

        out << "g body_" << bodyIdx++ << "\n";

        // Write vertices and normals
        for (const auto& v : body.vertices) {
            out << "v " << v.px << " " << v.py << " " << v.pz << "\n";
        }
        for (const auto& v : body.vertices) {
            out << "vn " << v.nx << " " << v.ny << " " << v.nz << "\n";
        }

        // Write faces (every 3 vertices = 1 triangle, OBJ is 1-based)
        int numVerts = (int)body.vertices.size();
        for (int j = 0; j + 2 < numVerts; j += 3) {
            int v1 = vertexOffset + j + 1;
            int v2 = vertexOffset + j + 2;
            int v3 = vertexOffset + j + 3;
            int n1 = normalOffset + j + 1;
            int n2 = normalOffset + j + 2;
            int n3 = normalOffset + j + 3;
            out << "f " << v1 << "//" << n1 << " "
                        << v2 << "//" << n2 << " "
                        << v3 << "//" << n3 << "\n";
        }

        vertexOffset += numVerts;
        normalOffset += numVerts;
    }

    return true;
}

// ─── DXF export (2D sketch) ─────────────────────────────────────────

bool exportDXF(const std::string& filepath, const Sketch& sketch) {
    std::ofstream out(fsPath(filepath));
    if (!out.is_open()) {
        s_lastError = "Cannot open file for writing";
        return false;
    }

    // Minimal DXF: just ENTITIES section
    out << "0\nSECTION\n2\nENTITIES\n";

    // Lines
    for (const auto& line : sketch.lines) {
        Point2D a = sketch.getPointPos(line.startPt);
        Point2D b = sketch.getPointPos(line.endPt);
        out << "0\nLINE\n8\n0\n"  // layer 0
            << "10\n" << a.x << "\n20\n" << a.y << "\n30\n0.0\n"
            << "11\n" << b.x << "\n21\n" << b.y << "\n31\n0.0\n";
    }

    // Circles
    for (const auto& circ : sketch.circles) {
        Point2D c = sketch.getPointPos(circ.centerPt);
        out << "0\nCIRCLE\n8\n0\n"
            << "10\n" << c.x << "\n20\n" << c.y << "\n30\n0.0\n"
            << "40\n" << circ.radius << "\n";
    }

    // Arcs
    for (const auto& arc : sketch.arcs) {
        Point2D c = sketch.getPointPos(arc.centerPt);
        Point2D sp = sketch.getPointPos(arc.startPt);
        double radius = std::sqrt((sp.x - c.x) * (sp.x - c.x) + (sp.y - c.y) * (sp.y - c.y));
        double startDeg = arc.startAngle * 180.0 / 3.14159265358979;
        double endDeg = arc.endAngle * 180.0 / 3.14159265358979;
        out << "0\nARC\n8\n0\n"
            << "10\n" << c.x << "\n20\n" << c.y << "\n30\n0.0\n"
            << "40\n" << radius << "\n"
            << "50\n" << startDeg << "\n51\n" << endDeg << "\n";
    }

    // Ellipses (DXF ELLIPSE entity)
    for (const auto& ell : sketch.ellipses) {
        Point2D c = sketch.getPointPos(ell.centerPt);
        // Major axis endpoint relative to center
        double majX = ell.semiMajor * std::cos(ell.rotation);
        double majY = ell.semiMajor * std::sin(ell.rotation);
        double ratio = (ell.semiMajor > 1e-10) ? ell.semiMinor / ell.semiMajor : 1.0;
        out << "0\nELLIPSE\n8\n0\n"
            << "10\n" << c.x << "\n20\n" << c.y << "\n30\n0.0\n"    // center
            << "11\n" << majX << "\n21\n" << majY << "\n31\n0.0\n"  // major axis endpoint (relative)
            << "40\n" << ratio << "\n"   // ratio minor/major
            << "41\n0.0\n42\n6.283185307\n";  // full ellipse (0 to 2pi)
    }

    // Splines: export as polyline approximation
    for (const auto& sp : sketch.splines) {
        if (sp.controlPtIDs.size() < 2) continue;
        auto pts = sampleSpline(sp, sketch, 64);
        if (pts.size() < 2) continue;
        out << "0\nPOLYLINE\n8\n0\n66\n1\n70\n0\n";
        for (const auto& p : pts) {
            out << "0\nVERTEX\n8\n0\n"
                << "10\n" << p.x << "\n20\n" << p.y << "\n30\n0.0\n";
        }
        out << "0\nSEQEND\n8\n0\n";
    }

    out << "0\nENDSEC\n0\nEOF\n";
    return true;
}

// ─── Additional file dialogs ────────────────────────────────────────

std::string openNativeStepSaveDialog() {
    char filename[MAX_PATH] = {};
    OPENFILENAMEA ofn = {};
    ofn.lStructSize = sizeof(ofn);
    ofn.lpstrFilter = "STEP Files (*.step;*.stp)\0*.step;*.stp\0All Files\0*.*\0";
    ofn.lpstrFile = filename;
    ofn.nMaxFile = MAX_PATH;
    ofn.lpstrDefExt = "step";
    ofn.Flags = OFN_OVERWRITEPROMPT | OFN_NOCHANGEDIR;
    if (GetSaveFileNameA(&ofn)) return ansiToUtf8(filename);
    return {};
}

std::string openNativeStepOpenDialog() {
    char filename[MAX_PATH] = {};
    OPENFILENAMEA ofn = {};
    ofn.lStructSize = sizeof(ofn);
    ofn.lpstrFilter = "STEP Files (*.step;*.stp)\0*.step;*.stp\0All Files\0*.*\0";
    ofn.lpstrFile = filename;
    ofn.nMaxFile = MAX_PATH;
    ofn.Flags = OFN_FILEMUSTEXIST | OFN_NOCHANGEDIR;
    if (GetOpenFileNameA(&ofn)) return ansiToUtf8(filename);
    return {};
}

std::string openNativeIgesSaveDialog() {
    char filename[MAX_PATH] = {};
    OPENFILENAMEA ofn = {};
    ofn.lStructSize = sizeof(ofn);
    ofn.lpstrFilter = "IGES Files (*.igs;*.iges)\0*.igs;*.iges\0All Files\0*.*\0";
    ofn.lpstrFile = filename;
    ofn.nMaxFile = MAX_PATH;
    ofn.lpstrDefExt = "igs";
    ofn.Flags = OFN_OVERWRITEPROMPT | OFN_NOCHANGEDIR;
    if (GetSaveFileNameA(&ofn)) return ansiToUtf8(filename);
    return {};
}

std::string openNativeIgesOpenDialog() {
    char filename[MAX_PATH] = {};
    OPENFILENAMEA ofn = {};
    ofn.lStructSize = sizeof(ofn);
    ofn.lpstrFilter = "IGES Files (*.igs;*.iges)\0*.igs;*.iges\0All Files\0*.*\0";
    ofn.lpstrFile = filename;
    ofn.nMaxFile = MAX_PATH;
    ofn.Flags = OFN_FILEMUSTEXIST | OFN_NOCHANGEDIR;
    if (GetOpenFileNameA(&ofn)) return ansiToUtf8(filename);
    return {};
}

std::string openNativeObjSaveDialog() {
    char filename[MAX_PATH] = {};
    OPENFILENAMEA ofn = {};
    ofn.lStructSize = sizeof(ofn);
    ofn.lpstrFilter = "OBJ Files (*.obj)\0*.obj\0All Files\0*.*\0";
    ofn.lpstrFile = filename;
    ofn.nMaxFile = MAX_PATH;
    ofn.lpstrDefExt = "obj";
    ofn.Flags = OFN_OVERWRITEPROMPT | OFN_NOCHANGEDIR;
    if (GetSaveFileNameA(&ofn)) return ansiToUtf8(filename);
    return {};
}

std::string ansiToUtf8(const std::string& ansi) {
    if (ansi.empty()) return ansi;
    const int wide = MultiByteToWideChar(CP_ACP, 0, ansi.c_str(), (int)ansi.size(), nullptr, 0);
    if (wide <= 0) return ansi;
    std::wstring w((size_t)wide, L'\0');
    MultiByteToWideChar(CP_ACP, 0, ansi.c_str(), (int)ansi.size(), &w[0], wide);
    const int utf8 = WideCharToMultiByte(CP_UTF8, 0, w.c_str(), wide, nullptr, 0, nullptr, nullptr);
    if (utf8 <= 0) return ansi;
    std::string out((size_t)utf8, '\0');
    WideCharToMultiByte(CP_UTF8, 0, w.c_str(), wide, &out[0], utf8, nullptr, nullptr);
    return out;
}

std::string openNativeFolderDialog(const char* title) {
    char path[MAX_PATH] = {};
    BROWSEINFOA bi = {};
    bi.lpszTitle = title;
    bi.ulFlags = BIF_RETURNONLYFSDIRS | BIF_NEWDIALOGSTYLE;
    LPITEMIDLIST pidl = SHBrowseForFolderA(&bi);
    if (!pidl) return {};
    std::string out;
    if (SHGetPathFromIDListA(pidl, path)) out = ansiToUtf8(path);
    CoTaskMemFree(pidl);
    return out;
}

std::string openNativeJsonSaveDialog() {
    char filename[MAX_PATH] = {};
    OPENFILENAMEA ofn = {};
    ofn.lStructSize = sizeof(ofn);
    ofn.lpstrFilter = "JSON Files (*.json)\0*.json\0All Files\0*.*\0";
    ofn.lpstrFile = filename;
    ofn.nMaxFile = MAX_PATH;
    ofn.lpstrDefExt = "json";
    ofn.Flags = OFN_OVERWRITEPROMPT | OFN_NOCHANGEDIR;
    if (GetSaveFileNameA(&ofn)) return ansiToUtf8(filename);
    return {};
}

std::string openNativeDxfSaveDialog() {
    char filename[MAX_PATH] = {};
    OPENFILENAMEA ofn = {};
    ofn.lStructSize = sizeof(ofn);
    ofn.lpstrFilter = "DXF Files (*.dxf)\0*.dxf\0All Files\0*.*\0";
    ofn.lpstrFile = filename;
    ofn.nMaxFile = MAX_PATH;
    ofn.lpstrDefExt = "dxf";
    ofn.Flags = OFN_OVERWRITEPROMPT | OFN_NOCHANGEDIR;
    if (GetSaveFileNameA(&ofn)) return ansiToUtf8(filename);
    return {};
}

std::string openNativeStlOpenDialog() {
    char filename[MAX_PATH] = {};
    OPENFILENAMEA ofn = {};
    ofn.lStructSize = sizeof(ofn);
    ofn.lpstrFilter = "STL Files (*.stl)\0*.stl\0All Files\0*.*\0";
    ofn.lpstrFile = filename;
    ofn.nMaxFile = MAX_PATH;
    ofn.Flags = OFN_FILEMUSTEXIST | OFN_NOCHANGEDIR;
    if (GetOpenFileNameA(&ofn)) return ansiToUtf8(filename);
    return {};
}

std::string openNativeImportDialog() {
    char filename[MAX_PATH] = {};
    OPENFILENAMEA ofn = {};
    ofn.lStructSize = sizeof(ofn);
    ofn.lpstrFilter = "All Supported (*.step;*.stp;*.igs;*.iges;*.stl)\0*.step;*.stp;*.igs;*.iges;*.stl\0"
                      "STEP Files (*.step;*.stp)\0*.step;*.stp\0"
                      "IGES Files (*.igs;*.iges)\0*.igs;*.iges\0"
                      "STL Files (*.stl)\0*.stl\0"
                      "All Files\0*.*\0";
    ofn.lpstrFile = filename;
    ofn.nMaxFile = MAX_PATH;
    ofn.Flags = OFN_FILEMUSTEXIST | OFN_NOCHANGEDIR;
    if (GetOpenFileNameA(&ofn)) return ansiToUtf8(filename);
    return {};
}

} // namespace shitcad
