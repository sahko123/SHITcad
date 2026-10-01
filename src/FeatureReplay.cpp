#include "FeatureReplay.h"
#include "Extrude.h"
#include "ExtrudeTool.h"
#include "ProfileDetector.h"
#include "FacePicker.h"
#include "MeshImport.h"
#include "CadImport.h"
#include <BRepAlgoAPI_Cut.hxx>
#include <BRepAlgoAPI_Fuse.hxx>
#include <BRepCheck_Analyzer.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <Bnd_Box.hxx>
#include <BRepBndLib.hxx>
#include <cstdio>
#include <ctime>
#include <map>
#include <sstream>
#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <Windows.h>
#endif

namespace shitcad {

// ---- Profile diagnostics logging ----

static FILE* openDiagLog() {
    static char path[512] = {};
    if (path[0] == '\0') {
#ifdef _WIN32
        // Place log next to executable
        GetModuleFileNameA(nullptr, path, sizeof(path));
        char* slash = strrchr(path, '\\');
        if (slash) *(slash + 1) = '\0';
        else path[0] = '\0';
        strncat(path, "SHITcad_profile_diag.log", sizeof(path) - strlen(path) - 1);
#else
        strcpy(path, "SHITcad_profile_diag.log");
#endif
    }
    return fopen(path, "a");
}

static void logTimestamp(FILE* f) {
    time_t now = time(nullptr);
    char buf[64];
    strftime(buf, sizeof(buf), "%Y-%m-%d %H:%M:%S", localtime(&now));
    fprintf(f, "\n===== %s =====\n", buf);
}

static void logSignature(FILE* f, const char* label, int idx, const ProfileSignature& sig) {
    fprintf(f, "  %s[%d]: circleID=%u, centroid=(%.4f, %.4f), lineIDs={",
            label, idx, sig.circleID, sig.centroidX, sig.centroidY);
    for (auto id : sig.lineIDs) fprintf(f, "%u,", id);
    fprintf(f, "}, arcIDs={");
    for (auto id : sig.arcIDs) fprintf(f, "%u,", id);
    fprintf(f, "}\n");
}

static void logDetectedProfile(FILE* f, int idx, const ClosedProfile& prof, const Sketch& sketch) {
    Point2D c = polygonCentroid(tessellateProfile(sketch, prof));
    fprintf(f, "  detected[%d]: circleID=%u, centroid=(%.4f, %.4f), %d pts, %d lines, %d segs, lineIDs={",
            idx, prof.circleID, c.x, c.y,
            (int)prof.pointIDs.size(), (int)prof.lineIDs.size(), (int)prof.segments.size());
    for (auto id : prof.lineIDs) fprintf(f, "%u,", id);
    fprintf(f, "}, arcCircleIDs={");
    for (const auto& seg : prof.segments) {
        if (seg.type == SegmentType::Arc && seg.origCircleID != NullID)
            fprintf(f, "%u,", seg.origCircleID);
    }
    fprintf(f, "}, %d holes, %d resolvedPts\n",
            (int)prof.holes.size(), (int)prof.resolvedPoints.size());
}

static void logSketchSummary(FILE* f, const Sketch& sketch, const SketchPlane& plane) {
    int projPts = 0, projLines = 0, projCircles = 0, projArcs = 0;
    for (const auto& p : sketch.points) if (p.projected) projPts++;
    for (const auto& l : sketch.lines) if (l.projected) projLines++;
    for (const auto& c : sketch.circles) if (c.projected) projCircles++;
    for (const auto& a : sketch.arcs) if (a.projected) projArcs++;

    fprintf(f, "  Sketch: %d pts (%d proj), %d lines (%d proj), %d circles (%d proj), %d arcs (%d proj)\n",
            (int)sketch.points.size(), projPts,
            (int)sketch.lines.size(), projLines,
            (int)sketch.circles.size(), projCircles,
            (int)sketch.arcs.size(), projArcs);
    fprintf(f, "  Plane: origin=(%.3f,%.3f,%.3f), normal=(%.3f,%.3f,%.3f), srcBody=%d\n",
            plane.origin[0], plane.origin[1], plane.origin[2],
            plane.normal[0], plane.normal[1], plane.normal[2],
            plane.sourceBodyIndex);
}

static void logMatchAttempt(FILE* f, int sigIdx, const ProfileSignature& sig,
                            const std::vector<ClosedProfile>& detected, const Sketch& sketch) {
    fprintf(f, "  Match attempt for sig[%d]:\n", sigIdx);

    std::set<EntityID> sigEdges = sig.lineIDs;
    sigEdges.insert(sig.arcIDs.begin(), sig.arcIDs.end());
    sigEdges.insert(sig.ellipseIDs.begin(), sig.ellipseIDs.end());
    sigEdges.insert(sig.splineIDs.begin(), sig.splineIDs.end());

    for (int d = 0; d < (int)detected.size(); d++) {
        const auto& prof = detected[d];

        // Circle check
        if (sig.circleID != NullID || prof.circleID != NullID) {
            fprintf(f, "    vs detected[%d]: circle check sig=%u prof=%u -> %s\n",
                    d, sig.circleID, prof.circleID,
                    sig.circleID == prof.circleID ? "MATCH" : "FAIL");
            continue;
        }

        // Jaccard
        std::set<EntityID> otherEdges(prof.lineIDs.begin(), prof.lineIDs.end());
        for (const auto& seg : prof.segments) {
            if (seg.type == SegmentType::Arc && seg.origCircleID != NullID)
                otherEdges.insert(seg.origCircleID);
        }

        double jaccard = 0;
        if (!sigEdges.empty() && !otherEdges.empty()) {
            std::set<EntityID> inter, uni;
            std::set_intersection(sigEdges.begin(), sigEdges.end(),
                                  otherEdges.begin(), otherEdges.end(),
                                  std::inserter(inter, inter.begin()));
            std::set_union(sigEdges.begin(), sigEdges.end(),
                          otherEdges.begin(), otherEdges.end(),
                          std::inserter(uni, uni.begin()));
            if (!uni.empty()) jaccard = (double)inter.size() / uni.size();
        }

        // Centroid distance
        Point2D c = polygonCentroid(tessellateProfile(sketch, prof));
        double dx = c.x - sig.centroidX, dy = c.y - sig.centroidY;
        double dist = std::sqrt(dx * dx + dy * dy);

        // Segment count ratio
        int mySeg = (int)(sig.lineIDs.size() + sig.arcIDs.size());
        int otherSeg = (int)(prof.lineIDs.size() + prof.segments.size());
        double ratio = (mySeg > 0 && otherSeg > 0)
            ? (double)std::min(mySeg, otherSeg) / std::max(mySeg, otherSeg) : 0;

        fprintf(f, "    vs detected[%d]: jaccard=%.3f (>0.7?%s), centroidDist=%.4f (<0.5?%s), "
                   "segRatio=%.2f (>0.5?%s), sigEdges=%d, detEdges=%d\n",
                d, jaccard, jaccard > 0.7 ? "Y" : "N",
                dist, dist < 0.5 ? "Y" : "N",
                ratio, ratio > 0.5 ? "Y" : "N",
                (int)sigEdges.size(), (int)otherEdges.size());
    }
}

void logProfileDiagnostics(const char* featureName, const char* errorType,
                           const std::vector<ProfileSignature>& sigs,
                           const std::vector<int>& fallback,
                           const std::vector<ClosedProfile>& detected,
                           const Sketch& sketch, const SketchPlane& plane) {
    FILE* f = openDiagLog();
    if (!f) return;

    logTimestamp(f);
    fprintf(f, "PROFILE ERROR: feature='%s', error='%s'\n", featureName, errorType);
    logSketchSummary(f, sketch, plane);

    fprintf(f, "  Saved signatures (%d):\n", (int)sigs.size());
    for (int i = 0; i < (int)sigs.size(); i++)
        logSignature(f, "sig", i, sigs[i]);

    fprintf(f, "  Fallback indices (%d): {", (int)fallback.size());
    for (int i = 0; i < (int)fallback.size(); i++)
        fprintf(f, "%d%s", fallback[i], i + 1 < (int)fallback.size() ? "," : "");
    fprintf(f, "}\n");

    fprintf(f, "  Detected profiles (%d):\n", (int)detected.size());
    for (int i = 0; i < (int)detected.size(); i++)
        logDetectedProfile(f, i, detected[i], sketch);

    if (!detected.empty() && !sigs.empty()) {
        fprintf(f, "  Detailed match analysis:\n");
        for (int s = 0; s < (int)sigs.size(); s++)
            logMatchAttempt(f, s, sigs[s], detected, sketch);
    }

    fclose(f);
}

// Overload for "no profiles detected" (no sigs to compare)
void logNoProfilesDiagnostics(const char* featureName,
                              const Sketch& sketch, const SketchPlane& plane) {
    FILE* f = openDiagLog();
    if (!f) return;

    logTimestamp(f);
    fprintf(f, "PROFILE ERROR: feature='%s', error='No profiles detected'\n", featureName);
    logSketchSummary(f, sketch, plane);

    // Dump all entity IDs for debugging
    fprintf(f, "  Line entities (%d):\n", (int)sketch.lines.size());
    for (const auto& l : sketch.lines) {
        fprintf(f, "    id=%u, startPt=%u, endPt=%u, proj=%d\n", l.id, l.startPt, l.endPt, l.projected);
    }
    fprintf(f, "  Circle entities (%d):\n", (int)sketch.circles.size());
    for (const auto& c : sketch.circles) {
        fprintf(f, "    id=%u, center=%u, r=%.4f, proj=%d\n", c.id, c.centerPt, c.radius, c.projected);
    }
    fprintf(f, "  Point entities (%d):\n", (int)sketch.points.size());
    for (const auto& p : sketch.points) {
        Point2D pos = {p.x, p.y};
        fprintf(f, "    id=%u, pos=(%.4f,%.4f), proj=%d\n", p.id, pos.x, pos.y, p.projected);
    }

    fclose(f);
}

// ---- OCCT tool error logging ----

static const char* shapeTypeName(TopAbs_ShapeEnum t) {
    switch (t) {
        case TopAbs_COMPOUND:  return "COMPOUND";
        case TopAbs_COMPSOLID: return "COMPSOLID";
        case TopAbs_SOLID:     return "SOLID";
        case TopAbs_SHELL:     return "SHELL";
        case TopAbs_FACE:      return "FACE";
        case TopAbs_WIRE:      return "WIRE";
        case TopAbs_EDGE:      return "EDGE";
        case TopAbs_VERTEX:    return "VERTEX";
        case TopAbs_SHAPE:     return "SHAPE";
    }
    return "UNKNOWN";
}

static void logShapeInfo(FILE* f, const char* label, const TopoDS_Shape& shape) {
    if (shape.IsNull()) {
        fprintf(f, "  %s: <null>\n", label);
        return;
    }
    fprintf(f, "  %s: type=%s, orientation=%d",
            label, shapeTypeName(shape.ShapeType()), (int)shape.Orientation());

    // Count sub-shapes
    int nSolids = 0, nShells = 0, nFaces = 0, nEdges = 0, nVertices = 0;
    for (TopExp_Explorer ex(shape, TopAbs_SOLID); ex.More(); ex.Next()) nSolids++;
    for (TopExp_Explorer ex(shape, TopAbs_SHELL); ex.More(); ex.Next()) nShells++;
    for (TopExp_Explorer ex(shape, TopAbs_FACE); ex.More(); ex.Next()) nFaces++;
    for (TopExp_Explorer ex(shape, TopAbs_EDGE); ex.More(); ex.Next()) nEdges++;
    for (TopExp_Explorer ex(shape, TopAbs_VERTEX); ex.More(); ex.Next()) nVertices++;
    fprintf(f, ", solids=%d, shells=%d, faces=%d, edges=%d, verts=%d\n",
            nSolids, nShells, nFaces, nEdges, nVertices);

    // Bounding box
    Bnd_Box bbox;
    BRepBndLib::Add(shape, bbox);
    if (!bbox.IsVoid()) {
        double xmin, ymin, zmin, xmax, ymax, zmax;
        bbox.Get(xmin, ymin, zmin, xmax, ymax, zmax);
        fprintf(f, "    bbox: (%.4f,%.4f,%.4f) - (%.4f,%.4f,%.4f)\n",
                xmin, ymin, zmin, xmax, ymax, zmax);
    }

    // Validity check
    BRepCheck_Analyzer checker(shape);
    fprintf(f, "    valid=%s\n", checker.IsValid() ? "yes" : "NO");
}

template<typename TAlgo>
static void logBooleanError(const char* featureName, const char* opName,
                            const TopoDS_Shape& targetShape, const TopoDS_Shape& toolShape,
                            int targetIdx, int toolIdx, TAlgo& algo) {
    FILE* f = openDiagLog();
    if (!f) return;

    logTimestamp(f);
    fprintf(f, "BOOLEAN ERROR: feature='%s', op='%s'\n", featureName, opName);
    fprintf(f, "  targetBodyIdx=%d, toolBodyIdx=%d\n", targetIdx, toolIdx);
    fprintf(f, "  IsDone=%d, HasErrors=%d, HasWarnings=%d\n",
            (int)algo.IsDone(), (int)algo.HasErrors(), (int)algo.HasWarnings());

    // Dump OCCT error/warning messages
    std::ostringstream oss;
    algo.DumpErrors(oss);
    std::string errors = oss.str();
    if (!errors.empty()) {
        fprintf(f, "  OCCT errors:\n%s\n", errors.c_str());
    }

    oss.str("");
    oss.clear();
    algo.DumpWarnings(oss);
    std::string warnings = oss.str();
    if (!warnings.empty()) {
        fprintf(f, "  OCCT warnings:\n%s\n", warnings.c_str());
    }

    logShapeInfo(f, "target", targetShape);
    logShapeInfo(f, "tool", toolShape);

    // Check if result shape exists even though HasErrors
    TopoDS_Shape result = algo.Shape();
    if (!result.IsNull()) {
        logShapeInfo(f, "result (despite error)", result);
        auto solids = enumerateSolids(result);
        fprintf(f, "  enumerateSolids returned %d solid(s)\n", (int)solids.size());
    }

    fclose(f);
}

static void logExtrudeToolError(const char* featureName, const char* errorType,
                                const TopoDS_Shape& toolShape,
                                const Scene3D& scene) {
    FILE* f = openDiagLog();
    if (!f) return;

    logTimestamp(f);
    fprintf(f, "TOOL SHAPE ERROR: feature='%s', error='%s'\n", featureName, errorType);
    logShapeInfo(f, "toolShape", toolShape);
    fprintf(f, "  scene bodies: %d\n", (int)scene.bodyCount());
    for (int i = 0; i < (int)scene.bodyCount(); i++) {
        char label[32];
        snprintf(label, sizeof(label), "body[%d]", i);
        logShapeInfo(f, label, scene.getBody(i).shape);
    }
    fclose(f);
}

// ---- matchProfiles ----

std::set<int> matchProfiles(const std::vector<ProfileSignature>& sigs,
                            const std::vector<int>& fallback,
                            const std::vector<ClosedProfile>& detected,
                            const Sketch& sketch) {
    std::set<int> matched;

    for (size_t s = 0; s < sigs.size(); s++) {
        // Find the best topological match by closest centroid
        int bestIdx = -1;
        float bestDist = 1e9f;
        for (int d = 0; d < (int)detected.size(); d++) {
            if (matched.count(d)) continue;
            if (sigs[s].matches(detected[d], sketch)) {
                float dist = sigs[s].centroidDistTo(detected[d], sketch);
                if (dist < bestDist) {
                    bestDist = dist;
                    bestIdx = d;
                }
            }
        }
        if (bestIdx >= 0) {
            matched.insert(bestIdx);
        } else if (s < fallback.size()) {
            // Fallback to index
            int idx = fallback[s];
            if (idx >= 0 && idx < (int)detected.size() && !matched.count(idx)) {
                matched.insert(idx);
            }
        }
    }
    return matched;
}

void recordProfileSelection(const std::set<int>& selected,
                            const std::vector<ClosedProfile>& all,
                            const Sketch& sketch,
                            std::vector<ProfileSignature>& sigs,
                            std::vector<int>& fallback) {
    for (int idx : selected) {
        if (idx < 0 || idx >= (int)all.size()) continue;
        sigs.push_back(ProfileSignature::fromProfile(all[idx], sketch));
        fallback.push_back(idx);
    }
}

// Give the body at `index` its identity: made by `feature`, as its n-th body.
static void tagBody(Scene3D& scene, int index, FeatureID feature, int n) {
    Body3D& b = scene.getBodyMut(index);
    b.sourceFeature = feature;
    b.sourceIndex = n;
}

// The sketch plane a feature's source sketch lives on; null, with the reason in `why`, if
// the sketch is gone or its plane is invalid.
static const SketchPlane* findSourcePlane(const FeatureHistory& history,
                                          const std::vector<SketchPlane>& planes,
                                          FeatureID sourceSketch, const char*& why) {
    const Feature* src = history.findFeature(sourceSketch);
    if (!src || src->type != FeatureType::Sketch) {
        why = "Source sketch not found";
        return nullptr;
    }
    const auto& sd = std::get<SketchFeatureData>(src->data);
    if (sd.sketchPlaneIndex < 0 || sd.sketchPlaneIndex >= (int)planes.size()) {
        why = "Invalid sketch plane";
        return nullptr;
    }
    return &planes[sd.sketchPlaneIndex];
}

struct SourceProfiles {
    const SketchPlane* plane = nullptr;
    std::vector<ClosedProfile> detected;
    std::set<int> matched; // indices into `detected`
};

// Extrude and Revolve start the same way: find the sketch the feature was made from, detect
// its closed profiles and match the recorded signatures against them. On failure the
// feature is marked with the reason and this returns false.
static bool resolveSourceProfiles(const FeatureHistory& history,
                                  const std::vector<SketchPlane>& planes,
                                  FeatureID sourceSketch,
                                  const std::vector<ProfileSignature>& sigs,
                                  const std::vector<int>& fallback,
                                  Feature& feat, SourceProfiles& out) {
    const char* why = nullptr;
    out.plane = findSourcePlane(history, planes, sourceSketch, why);
    if (!out.plane) {
        feat.hasError = true;
        feat.errorMsg = why;
        return false;
    }
    const SketchPlane& plane = *out.plane;
    const Sketch& sketch = plane.sketch;

    out.detected = detectClosedProfiles(sketch, plane);
    if (out.detected.empty()) {
        feat.hasError = true;
        feat.errorMsg = "No profiles detected";
        logNoProfilesDiagnostics(feat.name.c_str(), sketch, plane);
        return false;
    }

    out.matched = matchProfiles(sigs, fallback, out.detected, sketch);
    if (out.matched.empty()) {
        feat.hasError = true;
        feat.errorMsg = "Could not match profiles";
        logProfileDiagnostics(feat.name.c_str(), "Could not match profiles",
                              sigs, fallback, out.detected, sketch, plane);
        return false;
    }
    return true;
}

// Puts a feature's tool shape into the scene. A Cut subtracts it from every B-rep body
// (a body that splits keeps its first piece in place and appends the rest); anything else
// adds it as a new body and fuses it into the first existing body it joins. `label` names
// the feature type in the diagnostics.
static void applyToolShape(Scene3D& scene, const TopoDS_Shape& toolShape, ExtrudeOperation op,
                           Feature& feat, int& created, const char* label) {
    if (op == ExtrudeOperation::Cut) {
        const std::string opName = std::string(label) + " Cut";
        bool anyCut = false;
        for (int i = (int)scene.bodyCount() - 1; i >= 0; i--) {
            const auto& body = scene.getBody(i);
            if (body.isMeshOnly()) continue; // no B-rep to cut
            BRepAlgoAPI_Cut cutter(body.shape, toolShape);
            if (!cutter.IsDone() || cutter.HasErrors()) {
                logBooleanError(feat.name.c_str(), opName.c_str(), body.shape, toolShape, i, -1, cutter);
                continue;
            }

            auto solids = enumerateSolids(cutter.Shape());
            if (solids.empty()) {
                scene.removeBody(i);
            } else {
                scene.replaceBody(i, solids[0]);
                for (size_t j = 1; j < solids.size(); j++) {
                    scene.addBody(solids[j]);
                    tagBody(scene, (int)scene.bodyCount() - 1, feat.id, created++);
                }
            }
            anyCut = true;
        }
        if (!anyCut) {
            feat.hasError = true;
            feat.errorMsg = "Cut boolean failed on all bodies";
            logExtrudeToolError(feat.name.c_str(), (opName + " failed on all bodies").c_str(), toolShape, scene);
        }
        return;
    }

    scene.addBody(toolShape);
    int newIdx = (int)scene.bodyCount() - 1;
    tagBody(scene, newIdx, feat.id, created++);

    for (int i = newIdx - 1; i >= 0; i--) {
        const auto& existing = scene.getBody(i);
        if (existing.isMeshOnly()) continue; // never fuse into a reference mesh
        BRepAlgoAPI_Fuse fuser(existing.shape, scene.getBody(newIdx).shape);
        if (!fuser.IsDone() || fuser.HasErrors()) continue;

        auto solids = enumerateSolids(fuser.Shape());
        if (solids.size() == 1) {
            scene.replaceBody(i, solids[0]);
            scene.removeBody(newIdx);
            break;
        }
    }
}

void replayFeatures(FeatureHistory& history,
                    std::vector<SketchPlane>& planes,
                    Scene3D& scene) {
    scene.clear();

    // Clear all sketch plane geometry so undone sketches don't persist
    for (auto& plane : planes) {
        plane.sketch.clear();
    }

    const auto& features = history.features();

    for (int fi = 0; fi < (int)features.size(); fi++) {
        const Feature& feat = features[fi];

        if (feat.suppressed || history.isRolledBack(fi)) continue;

        // Clear any previous error
        Feature& mutableFeat = history.features()[fi];
        mutableFeat.hasError = false;
        mutableFeat.errorMsg.clear();
        int created = 0; // bodies this feature has created, for their identities

        if (feat.type == FeatureType::Sketch) {
            const auto& sd = std::get<SketchFeatureData>(feat.data);
            if (sd.sketchPlaneIndex >= 0 && sd.sketchPlaneIndex < (int)planes.size()) {
                planes[sd.sketchPlaneIndex].sketch = sd.sketchSnapshot;

                // Face-based sketch planes: keep projected geometry from the
                // snapshot as-is.  Re-projecting during replay would assign new
                // IDs to projected points, orphaning any user-drawn lines that
                // were snapped to the old projected points and breaking profile
                // detection.  The snapshot already contains the correct projected
                // edges from when the sketch was created / last edited.
            }
        } else if (feat.type == FeatureType::Extrude) {
            const auto& ed = std::get<ExtrudeFeatureData>(feat.data);

            SourceProfiles src;
            if (!resolveSourceProfiles(history, planes, ed.sourceSketchFeature, ed.profileSigs,
                                       ed.profileIndicesFallback, mutableFeat, src)) continue;
            const SketchPlane& plane = *src.plane;
            const Sketch& sketch = plane.sketch;

            ExtrudeToolState tempState;
            tempState.allProfiles = std::move(src.detected);
            tempState.selectedProfileIndices = std::move(src.matched);
            tempState.height = ed.height;
            tempState.offset = ed.offset;
            tempState.operation = ed.operation;
            tempState.direction = ed.direction;

            TopoDS_Shape toolShape = buildExtrudeToolShape(tempState, sketch, plane);
            if (toolShape.IsNull()) {
                mutableFeat.hasError = true;
                mutableFeat.errorMsg = "Extrude failed (OCCT)";
                logExtrudeToolError(feat.name.c_str(), "buildExtrudeToolShape returned null", toolShape, scene);
                continue;
            }
            applyToolShape(scene, toolShape, ed.operation, mutableFeat, created, "Extrude");
        } else if (feat.type == FeatureType::Revolve) {
            const auto& rd = std::get<RevolveFeatureData>(feat.data);

            SourceProfiles src;
            if (!resolveSourceProfiles(history, planes, rd.sourceSketchFeature, rd.profileSigs,
                                       rd.profileIndicesFallback, mutableFeat, src)) continue;
            const SketchPlane& plane = *src.plane;
            const Sketch& sketch = plane.sketch;

            RevolveToolState tempState;
            tempState.allProfiles = std::move(src.detected);
            tempState.selectedProfileIndices = std::move(src.matched);
            tempState.axisLineID = rd.axisLineID;
            tempState.angleDeg = rd.angleDeg;
            tempState.operation = rd.operation;

            TopoDS_Shape toolShape = buildRevolveToolShape(tempState, sketch, plane);
            if (toolShape.IsNull()) {
                mutableFeat.hasError = true;
                mutableFeat.errorMsg = "Revolve failed (OCCT)";
                logExtrudeToolError(feat.name.c_str(), "buildRevolveToolShape returned null", toolShape, scene);
                continue;
            }
            applyToolShape(scene, toolShape, rd.operation, mutableFeat, created, "Revolve");
        } else if (feat.type == FeatureType::Loft) {
            const auto& ld = std::get<LoftFeatureData>(feat.data);
            if (ld.sections.size() < 2) {
                mutableFeat.hasError = true;
                mutableFeat.errorMsg = "Loft needs at least 2 sections";
                continue;
            }

            struct LoftData {
                Sketch const* sketch;
                SketchPlane const* plane;
                std::vector<ClosedProfile> detected;
                int matchedIdx;
            };
            std::vector<LoftData> loftDatas;
            const char* sectionError = nullptr;

            for (const auto& sec : ld.sections) {
                const SketchPlane* plane = findSourcePlane(history, planes, sec.sourceSketchFeature, sectionError);
                if (!plane) break;
                const Sketch& sketch = plane->sketch;

                LoftData d;
                d.sketch = &sketch;
                d.plane = plane;
                d.detected = detectClosedProfiles(sketch, *plane);
                if (d.detected.empty()) {
                    logNoProfilesDiagnostics(feat.name.c_str(), sketch, *plane);
                    sectionError = "No profiles detected";
                    break;
                }

                std::vector<ProfileSignature> sigVec = {sec.profileSig};
                std::vector<int> fbVec = {sec.profileIndexFallback};
                auto matched = matchProfiles(sigVec, fbVec, d.detected, sketch);
                if (matched.empty()) {
                    logProfileDiagnostics(feat.name.c_str(), "Loft section match failed",
                                          sigVec, fbVec, d.detected, sketch, *plane);
                    sectionError = "Could not match loft section profile";
                    break;
                }
                d.matchedIdx = *matched.begin();
                loftDatas.push_back(std::move(d));
            }

            if (sectionError) {
                mutableFeat.hasError = true;
                mutableFeat.errorMsg = sectionError;
                continue;
            }

            std::vector<LoftWireInput> realInputs;
            for (auto& d : loftDatas) {
                LoftWireInput wi;
                wi.sketch = d.sketch;
                wi.plane = d.plane;
                wi.profile = &d.detected[d.matchedIdx];
                realInputs.push_back(wi);
            }

            TopoDS_Shape toolShape = loftProfiles(realInputs, ld.solid);
            if (toolShape.IsNull()) {
                mutableFeat.hasError = true;
                mutableFeat.errorMsg = "Loft failed (OCCT)";
                logExtrudeToolError(feat.name.c_str(), "loftProfiles returned null", toolShape, scene);
                continue;
            }
            applyToolShape(scene, toolShape, ld.operation, mutableFeat, created, "Loft");
        } else if (feat.type == FeatureType::MeshImport) {
            const auto& md = std::get<MeshImportFeatureData>(feat.data);

            Body3D body;
            MeshFileInfo info;
            std::string err;
            if (!loadMeshFile(md.sourcePath, md.unit, body.vertices, info, err)) {
                // Referenced, not embedded: a moved or deleted file is an error on
                // this feature, not a silently missing body.
                mutableFeat.hasError = true;
                mutableFeat.errorMsg = err;
                continue;
            }
            applyMeshTransform(body.vertices, md.transform);
            body.sourceFeature = feat.id;
            body.sourceIndex = created++;
            body.closed = info.closed;
            scene.addMeshBody(std::move(body)); // uploaded when next rendered
        } else if (feat.type == FeatureType::CadImport) {
            const auto& cd = std::get<CadImportFeatureData>(feat.data);

            std::vector<CadPart> parts;
            CadFileInfo info;
            std::string err;
            if (!loadCadFile(cd.sourcePath, parts, info, err)) {
                // Referenced, not embedded, like a mesh import.
                mutableFeat.hasError = true;
                mutableFeat.errorMsg = err;
                continue;
            }
            // Names that tell the bodies apart in the object tree: an unnamed
            // part gets a number, and a name used twice (instances of one part
            // in an assembly) a suffix.
            std::vector<std::string> names(parts.size());
            std::map<std::string, int> uses, seen;
            for (size_t i = 0; i < parts.size(); i++) {
                names[i] = parts[i].name.empty() && parts.size() > 1 ? "Body " + std::to_string(i + 1) : parts[i].name;
                uses[names[i]]++;
            }
            for (auto& n : names)
                if (!n.empty() && uses[n] > 1) n += " (" + std::to_string(++seen[n]) + ")";
            for (size_t pi = 0; pi < parts.size(); pi++) {
                const CadPart& part = parts[pi];
                // Placement is only a location, so the cached tessellation is reused.
                scene.addBody(placeCadShape(part.shape, cd.transform), info.deflection, true);
                tagBody(scene, (int)scene.bodyCount() - 1, feat.id, created++); // solid i of the file
                Body3D& body = scene.getBodyMut((int)scene.bodyCount() - 1);
                body.name = names[pi];
                // A shell or face group can be open; only a solid is surely closed.
                body.closed = part.shape.ShapeType() == TopAbs_SOLID;
                if (part.hasColor) {
                    body.hasFileColor = true;
                    for (int k = 0; k < 3; k++) body.fileColor[k] = part.color[k];
                    body.colorR = part.color[0]; body.colorG = part.color[1]; body.colorB = part.color[2];
                }
            }
        } else if (feat.type == FeatureType::Boolean) {
            auto& bd = std::get<BooleanFeatureData>(mutableFeat.data);

            // By identity. A project saved before identities has only indices:
            // resolve those once and record who they pointed at, so from here
            // on the Boolean follows its bodies rather than their positions.
            const bool byRef = bd.targetBody.isSet() && bd.toolBody.isSet();
            int targetIdx = byRef ? scene.findBody(bd.targetBody.feature, bd.targetBody.index) : bd.targetBodyIndex;
            int toolIdx = byRef ? scene.findBody(bd.toolBody.feature, bd.toolBody.index) : bd.toolBodyIndex;

            if (byRef && (targetIdx < 0 || toolIdx < 0)) {
                // Its feature was deleted or suppressed, or a re-exported import
                // now has fewer solids. Refuse rather than guess at another body.
                mutableFeat.hasError = true;
                mutableFeat.errorMsg = targetIdx < 0 ? "The target body no longer exists"
                                                     : "The tool body no longer exists";
                continue;
            }
            if (targetIdx < 0 || targetIdx >= (int)scene.bodyCount() ||
                toolIdx < 0 || toolIdx >= (int)scene.bodyCount() ||
                targetIdx == toolIdx) {
                mutableFeat.hasError = true;
                char msg[128];
                snprintf(msg, sizeof(msg), "Invalid body indices (target=%d, tool=%d, bodies=%d)",
                         targetIdx, toolIdx, (int)scene.bodyCount());
                mutableFeat.errorMsg = msg;
                continue;
            }
            if (!byRef) {
                const Body3D& t = scene.getBody(targetIdx);
                const Body3D& u = scene.getBody(toolIdx);
                bd.targetBody = {t.sourceFeature, t.sourceIndex};
                bd.toolBody = {u.sourceFeature, u.sourceIndex};
            }
            // Kept current for builds that only read indices.
            bd.targetBodyIndex = targetIdx;
            bd.toolBodyIndex = toolIdx;

            if (scene.getBody(targetIdx).isMeshOnly() || scene.getBody(toolIdx).isMeshOnly()) {
                mutableFeat.hasError = true;
                mutableFeat.errorMsg = "Boolean body is an imported mesh (no solid geometry)";
                continue;
            }

            const TopoDS_Shape targetShape = scene.getBody(targetIdx).shape;
            const TopoDS_Shape toolShape = scene.getBody(toolIdx).shape;

            std::vector<TopoDS_Shape> solids;
            if (bd.operation == BooleanOperation::Union) {
                BRepAlgoAPI_Fuse fuser(targetShape, toolShape);
                if (!fuser.IsDone() || fuser.HasErrors()) {
                    mutableFeat.hasError = true;
                    mutableFeat.errorMsg = "Union boolean failed";
                    logBooleanError(feat.name.c_str(), "Union (Fuse)", targetShape, toolShape, targetIdx, toolIdx, fuser);
                    continue;
                }
                solids = enumerateSolids(fuser.Shape());
                if (solids.empty()) {
                    mutableFeat.hasError = true;
                    mutableFeat.errorMsg = "Union produced no solids";
                    logBooleanError(feat.name.c_str(), "Union produced 0 solids", targetShape, toolShape, targetIdx, toolIdx, fuser);
                    continue;
                }
            } else {
                BRepAlgoAPI_Cut cutter(targetShape, toolShape);
                if (!cutter.IsDone() || cutter.HasErrors()) {
                    mutableFeat.hasError = true;
                    mutableFeat.errorMsg = "Subtract boolean failed";
                    logBooleanError(feat.name.c_str(), "Subtract (Cut)", targetShape, toolShape, targetIdx, toolIdx, cutter);
                    continue;
                }
                solids = enumerateSolids(cutter.Shape());
            }

            // The result is the target (it keeps the target's identity, name and
            // colour, whichever comes first in the list); the tool is used up.
            if (solids.empty()) {
                scene.removeBody(std::max(targetIdx, toolIdx));
                scene.removeBody(std::min(targetIdx, toolIdx));
            } else {
                scene.replaceBody(targetIdx, solids[0]);
                scene.removeBody(toolIdx);
                for (size_t j = 1; j < solids.size(); j++) {
                    scene.addBody(solids[j]);
                    tagBody(scene, (int)scene.bodyCount() - 1, feat.id, created++);
                }
            }
        }
    }
}

} // namespace shitcad
