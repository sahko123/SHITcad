#include "Extrude.h"
#include "ExtrudeTool.h"

#include <BRepBuilderAPI_MakeWire.hxx>
#include <BRepBuilderAPI_MakeEdge.hxx>
#include <BRepBuilderAPI_MakeFace.hxx>
#include <BRepBuilderAPI_Transform.hxx>
#include <BRepPrimAPI_MakePrism.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>
#include <gp_Circ.hxx>
#include <gp_Ax2.hxx>
#include <gp_Trsf.hxx>
#include <gp_Pln.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Wire.hxx>
#include <TopoDS_Face.hxx>
#include <ShapeFix_Face.hxx>
#include <ShapeFix_Wire.hxx>
#include <GC_MakeArcOfCircle.hxx>
#include <Geom_Ellipse.hxx>
#include <Geom_BSplineCurve.hxx>
#include <gp_Elips.hxx>
#include <TColgp_Array1OfPnt.hxx>
#include <TColStd_Array1OfReal.hxx>
#include <TColStd_Array1OfInteger.hxx>
#include <BRepAlgoAPI_Fuse.hxx>
#include <BRepPrimAPI_MakeRevol.hxx>
#include <BRepOffsetAPI_ThruSections.hxx>
#include <BRep_Builder.hxx>
#include <TopoDS_Compound.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <cmath>

#include "Constants.h"

namespace shitcad {

static gp_Pnt localToGpPnt(const SketchPlane& plane, float lx, float ly) {
    float wx, wy, wz;
    plane.localToWorld(lx, ly, wx, wy, wz);
    return gp_Pnt(wx, wy, wz);
}

// Build an OCCT wire from a ClosedProfile (circle or polygon)
static TopoDS_Wire buildProfileWire(const Sketch& sketch, const ClosedProfile& profile,
                                     const SketchPlane& plane) {
    if (profile.isCircle()) {
        const CircleEntity* circle = sketch.findCircle(profile.circleID);
        if (!circle || circle->radius < 1e-6f) return TopoDS_Wire();

        Point2D center = sketch.getPointPos(circle->centerPt);
        gp_Pnt centerPt = localToGpPnt(plane, center.x, center.y);
        gp_Dir normalDir(plane.normal[0], plane.normal[1], plane.normal[2]);
        gp_Ax2 axis(centerPt, normalDir);
        gp_Circ circ(axis, circle->radius);

        BRepBuilderAPI_MakeEdge edgeBuilder(circ);
        if (!edgeBuilder.IsDone()) return TopoDS_Wire();

        BRepBuilderAPI_MakeWire wireBuilder(edgeBuilder.Edge());
        if (!wireBuilder.IsDone()) return TopoDS_Wire();
        return wireBuilder.Wire();
    } else {
        // Use resolvedPoints if available (subdivided profiles), else look up pointIDs
        const bool useResolved = !profile.resolvedPoints.empty();
        size_t numPts = useResolved ? profile.resolvedPoints.size() : profile.pointIDs.size();
        if (numPts < 2) return TopoDS_Wire();

        BRepBuilderAPI_MakeWire wireBuilder;
        for (size_t i = 0; i < numPts; i++) {
            size_t next = (i + 1) % numPts;
            Point2D a = useResolved ? profile.resolvedPoints[i] : sketch.getPointPos(profile.pointIDs[i]);
            Point2D b = useResolved ? profile.resolvedPoints[next] : sketch.getPointPos(profile.pointIDs[next]);

            gp_Pnt pa = localToGpPnt(plane, a.x, a.y);
            gp_Pnt pb = localToGpPnt(plane, b.x, b.y);
            if (pa.Distance(pb) < 1e-6) continue;

            // Build edge based on segment type
            SegmentType segType = SegmentType::Line;
            if (!profile.segments.empty() && i < profile.segments.size())
                segType = profile.segments[i].type;

            if (segType == SegmentType::Arc) {
                const auto& seg = profile.segments[i];
                double span = seg.arcEndAngle - seg.arcStartAngle;
                double midAngle = seg.arcStartAngle + span * 0.5;
                Point2D midLocal = {
                    seg.arcCenter.x + seg.arcRadius * std::cos(midAngle),
                    seg.arcCenter.y + seg.arcRadius * std::sin(midAngle)
                };
                gp_Pnt pm = localToGpPnt(plane, midLocal.x, midLocal.y);

                GC_MakeArcOfCircle arcMaker(pa, pm, pb);
                if (!arcMaker.IsDone()) return TopoDS_Wire();

                BRepBuilderAPI_MakeEdge edgeBuilder(arcMaker.Value());
                if (!edgeBuilder.IsDone()) return TopoDS_Wire();
                wireBuilder.Add(edgeBuilder.Edge());

            } else if (segType == SegmentType::Ellipse || segType == SegmentType::EllipseArc) {
                const auto& seg = profile.segments[i];
                // Build OCCT ellipse in 3D on the sketch plane
                gp_Pnt centerPt = localToGpPnt(plane, seg.ellipseCenter.x, seg.ellipseCenter.y);
                gp_Dir normalDir(plane.normal[0], plane.normal[1], plane.normal[2]);

                // Major axis direction: rotate plane's U axis by ellipseRotation
                double cosR = std::cos(seg.ellipseRotation);
                double sinR = std::sin(seg.ellipseRotation);
                gp_Dir majorDir(
                    plane.uAxis[0] * cosR + plane.vAxis[0] * sinR,
                    plane.uAxis[1] * cosR + plane.vAxis[1] * sinR,
                    plane.uAxis[2] * cosR + plane.vAxis[2] * sinR
                );

                gp_Ax2 ax(centerPt, normalDir, majorDir);
                gp_Elips elips(ax, seg.semiMajor, seg.semiMinor);
                Handle(Geom_Ellipse) geomEllipse = new Geom_Ellipse(elips);

                // Build edge between the parametric angles
                BRepBuilderAPI_MakeEdge edgeBuilder(geomEllipse, seg.ellipseStartAngle, seg.ellipseEndAngle);
                if (!edgeBuilder.IsDone()) return TopoDS_Wire();
                wireBuilder.Add(edgeBuilder.Edge());

            } else if (segType == SegmentType::Spline) {
                const auto& seg = profile.segments[i];
                int nCtrl = (int)seg.splineControlPts.size();
                if (nCtrl < 2) return TopoDS_Wire();

                // Convert control points to 3D on the sketch plane
                TColgp_Array1OfPnt poles(1, nCtrl);
                for (int ci = 0; ci < nCtrl; ci++) {
                    poles.SetValue(ci + 1, localToGpPnt(plane,
                        seg.splineControlPts[ci].x, seg.splineControlPts[ci].y));
                }

                // Build knot vector: OCCT wants unique knots + multiplicities
                // Our format stores the full knot vector with repeats
                std::vector<double> uniqueKnots;
                std::vector<int> mults;
                for (int ki = 0; ki < (int)seg.splineKnots.size(); ki++) {
                    if (uniqueKnots.empty() || std::fabs(seg.splineKnots[ki] - uniqueKnots.back()) > 1e-10) {
                        uniqueKnots.push_back(seg.splineKnots[ki]);
                        mults.push_back(1);
                    } else {
                        mults.back()++;
                    }
                }

                int nKnots = (int)uniqueKnots.size();
                TColStd_Array1OfReal knotsArr(1, nKnots);
                TColStd_Array1OfInteger multsArr(1, nKnots);
                for (int ki = 0; ki < nKnots; ki++) {
                    knotsArr.SetValue(ki + 1, uniqueKnots[ki]);
                    multsArr.SetValue(ki + 1, mults[ki]);
                }

                // Weights (optional — uniform if empty)
                bool hasWeights = !seg.splineWeights.empty() && (int)seg.splineWeights.size() == nCtrl;
                TColStd_Array1OfReal weightsArr(1, nCtrl);
                for (int ci = 0; ci < nCtrl; ci++)
                    weightsArr.SetValue(ci + 1, hasWeights ? seg.splineWeights[ci] : 1.0);

                Handle(Geom_BSplineCurve) bspline;
                try {
                    bspline = new Geom_BSplineCurve(poles, weightsArr, knotsArr, multsArr, seg.splineDegree);
                } catch (...) {
                    return TopoDS_Wire(); // invalid spline data
                }

                BRepBuilderAPI_MakeEdge edgeBuilder(bspline, seg.splineParamStart, seg.splineParamEnd);
                if (!edgeBuilder.IsDone()) return TopoDS_Wire();
                wireBuilder.Add(edgeBuilder.Edge());

            } else {
                // Line segment
                BRepBuilderAPI_MakeEdge edgeBuilder(pa, pb);
                if (!edgeBuilder.IsDone()) return TopoDS_Wire();
                wireBuilder.Add(edgeBuilder.Edge());
            }
        }

        TopoDS_Wire wire;
        if (!wireBuilder.IsDone()) {
            // Try fixing with ShapeFix_Wire
            ShapeFix_Wire fixer;
            fixer.Load(wireBuilder.Wire());
            fixer.SetPrecision(1e-3);
            fixer.FixConnected();
            fixer.FixClosed();
            fixer.Perform();
            wire = fixer.Wire();
        } else {
            wire = wireBuilder.Wire();
        }

        if (wire.IsNull()) return TopoDS_Wire();
        int edgeCount = 0;
        for (TopExp_Explorer exp(wire, TopAbs_EDGE); exp.More(); exp.Next())
            edgeCount++;
        if (edgeCount < 2) return TopoDS_Wire(); // degenerate
        return wire;
    }
}

// Build a face from the outer wire, then add inner hole wires
static TopoDS_Face buildFaceWithHoles(const Sketch& sketch, const ClosedProfile& profile,
                                       const SketchPlane& plane) {
    TopoDS_Wire outerWire = buildProfileWire(sketch, profile, plane);
    if (outerWire.IsNull()) return TopoDS_Face();

    // Build face on an explicit plane so all wires are guaranteed coplanar
    gp_Pnt origin(plane.origin[0], plane.origin[1], plane.origin[2]);
    gp_Dir normal(plane.normal[0], plane.normal[1], plane.normal[2]);
    gp_Pln gpPlane(origin, normal);

    BRepBuilderAPI_MakeFace faceBuilder(gpPlane, outerWire);
    if (!faceBuilder.IsDone()) return TopoDS_Face();

    // Add inner hole wires — let ShapeFix handle orientation
    for (const auto& hole : profile.holes) {
        TopoDS_Wire holeWire = buildProfileWire(sketch, hole, plane);
        if (holeWire.IsNull()) continue;
        faceBuilder.Add(holeWire);
    }

    TopoDS_Face resultFace = faceBuilder.Face();

    // Fix wire orientations automatically (outer=CCW, holes=CW relative to face normal)
    ShapeFix_Face fixer(resultFace);
    fixer.SetPrecision(1e-4);
    fixer.FixOrientation();
    fixer.Perform();
    return fixer.Face();
}

TopoDS_Shape extrudeProfile(const Sketch& sketch, const ClosedProfile& profile,
                            float height, const SketchPlane& plane) {
    if (std::fabs(height) < 1e-6f) return TopoDS_Shape();

    TopoDS_Face face = buildFaceWithHoles(sketch, profile, plane);
    if (face.IsNull()) return TopoDS_Shape();

    gp_Vec extrudeVec(plane.normal[0] * height, plane.normal[1] * height, plane.normal[2] * height);
    BRepPrimAPI_MakePrism prism(face, extrudeVec);
    if (!prism.IsDone()) return TopoDS_Shape();

    return prism.Shape();
}

TopoDS_Shape extrudeProfileEx(const Sketch& sketch, const ClosedProfile& profile,
                               float height, float offset, const SketchPlane& plane) {
    if (std::fabs(height) < 1e-6f) return TopoDS_Shape();

    TopoDS_Face face = buildFaceWithHoles(sketch, profile, plane);
    if (face.IsNull()) return TopoDS_Shape();

    // Apply offset: translate face along normal
    if (std::fabs(offset) > 1e-6f) {
        gp_Trsf trsf;
        gp_Vec offsetVec(plane.normal[0] * offset, plane.normal[1] * offset, plane.normal[2] * offset);
        trsf.SetTranslation(offsetVec);
        BRepBuilderAPI_Transform xform(face, trsf);
        if (!xform.IsDone()) return TopoDS_Shape();
        face = TopoDS::Face(xform.Shape());
    }

    gp_Vec extrudeVec(plane.normal[0] * height, plane.normal[1] * height, plane.normal[2] * height);
    BRepPrimAPI_MakePrism prism(face, extrudeVec);
    if (!prism.IsDone()) return TopoDS_Shape();

    return prism.Shape();
}

TopoDS_Shape buildExtrudeToolShape(const ExtrudeToolState& state,
                                          const Sketch& sketch,
                                          const SketchPlane& plane) {
    float h = state.height;
    float off = state.offset;

    // For Cut operations, flip direction: "One Side" should cut INTO the body
    bool flipDir = (state.operation == ExtrudeOperation::Cut);

    std::vector<TopoDS_Shape> shapes;

    for (int idx : state.selectedProfileIndices) {
        if (idx < 0 || idx >= (int)state.allProfiles.size()) continue;
        const auto& profile = state.allProfiles[idx];

        switch (state.direction) {
            case ExtrudeDirection::OneSide: {
                TopoDS_Shape s = extrudeProfileEx(sketch, profile, flipDir ? -h : h, off, plane);
                if (!s.IsNull()) shapes.push_back(s);
                break;
            }
            case ExtrudeDirection::OtherSide: {
                TopoDS_Shape s = extrudeProfileEx(sketch, profile, flipDir ? h : -h, off, plane);
                if (!s.IsNull()) shapes.push_back(s);
                break;
            }
            case ExtrudeDirection::Symmetric: {
                TopoDS_Shape s = extrudeProfileEx(sketch, profile, h, off - h * 0.5f, plane);
                if (!s.IsNull()) shapes.push_back(s);
                break;
            }
            case ExtrudeDirection::BothSides: {
                TopoDS_Shape s1 = extrudeProfileEx(sketch, profile, h, off, plane);
                TopoDS_Shape s2 = extrudeProfileEx(sketch, profile, -h, off, plane);
                if (!s1.IsNull()) shapes.push_back(s1);
                if (!s2.IsNull()) shapes.push_back(s2);
                break;
            }
        }
    }

    if (shapes.empty()) return TopoDS_Shape();
    if (shapes.size() == 1) return shapes[0];

    // Group shapes into a compound (instant, no boolean computation).
    // Actual fusing happens at commit time in replayFeatures().
    BRep_Builder builder;
    TopoDS_Compound compound;
    builder.MakeCompound(compound);
    for (auto& s : shapes)
        builder.Add(compound, s);
    return compound;
}

TopoDS_Shape revolveProfile(const Sketch& sketch, const ClosedProfile& profile,
                            Point2D axisA, Point2D axisB,
                            float angleDeg, const SketchPlane& plane) {
    if (std::fabs(angleDeg) < 0.01f) return TopoDS_Shape();

    TopoDS_Face face = buildFaceWithHoles(sketch, profile, plane);
    if (face.IsNull()) return TopoDS_Shape();

    // Build axis in 3D
    gp_Pnt pA = localToGpPnt(plane, axisA.x, axisA.y);
    gp_Pnt pB = localToGpPnt(plane, axisB.x, axisB.y);
    gp_Dir axisDir(pB.X() - pA.X(), pB.Y() - pA.Y(), pB.Z() - pA.Z());
    gp_Ax1 axis(pA, axisDir);

    // For full revolution (>= 360), use the no-angle overload which creates
    // a properly closed solid. The angle overload creates a degenerate shape at 2π.
    if (std::fabs(angleDeg) >= 360.0f) {
        BRepPrimAPI_MakeRevol revol(face, axis);
        if (!revol.IsDone()) return TopoDS_Shape();
        return revol.Shape();
    }

    float angleRad = angleDeg * kDegToRad;
    BRepPrimAPI_MakeRevol revol(face, axis, angleRad);
    if (!revol.IsDone()) return TopoDS_Shape();
    return revol.Shape();
}

TopoDS_Shape buildRevolveToolShape(const RevolveToolState& state,
                                   const Sketch& sketch,
                                   const SketchPlane& plane) {
    if (state.axisLineID == NullID) return TopoDS_Shape();
    const LineEntity* axisLine = sketch.findLine(state.axisLineID);
    if (!axisLine) return TopoDS_Shape();

    Point2D axisA = sketch.getPointPos(axisLine->startPt);
    Point2D axisB = sketch.getPointPos(axisLine->endPt);
    if (distance(axisA, axisB) < 1e-6f) return TopoDS_Shape();

    std::vector<TopoDS_Shape> shapes;

    for (int idx : state.selectedProfileIndices) {
        if (idx < 0 || idx >= (int)state.allProfiles.size()) continue;
        const auto& profile = state.allProfiles[idx];
        TopoDS_Shape s = revolveProfile(sketch, profile, axisA, axisB, state.angleDeg, plane);
        if (!s.IsNull()) shapes.push_back(s);
    }

    if (shapes.empty()) return TopoDS_Shape();
    if (shapes.size() == 1) return shapes[0];

    BRep_Builder builder;
    TopoDS_Compound compound;
    builder.MakeCompound(compound);
    for (auto& s : shapes)
        builder.Add(compound, s);
    return compound;
}

TopoDS_Shape loftProfiles(const std::vector<LoftWireInput>& sections, bool solid) {
    if (sections.size() < 2) return TopoDS_Shape();

    BRepOffsetAPI_ThruSections lofter(solid ? Standard_True : Standard_False);
    lofter.SetSmoothing(Standard_True);

    for (const auto& sec : sections) {
        TopoDS_Wire wire = buildProfileWire(*sec.sketch, *sec.profile, *sec.plane);
        if (wire.IsNull()) return TopoDS_Shape();
        lofter.AddWire(wire);
    }

    lofter.Build();
    if (!lofter.IsDone()) return TopoDS_Shape();
    return lofter.Shape();
}

std::vector<TopoDS_Shape> enumerateSolids(const TopoDS_Shape& shape) {
    std::vector<TopoDS_Shape> solids;
    for (TopExp_Explorer exp(shape, TopAbs_SOLID); exp.More(); exp.Next()) {
        solids.push_back(exp.Current());
    }
    return solids;
}

// ─── Error-reporting variants ──────────────────────────────────────

ShapeResult buildExtrudeToolShapeEx(const ExtrudeToolState& state,
                                    const Sketch& sketch,
                                    const SketchPlane& plane) {
    ShapeResult result;
    if (state.selectedProfileIndices.empty()) {
        result.error = "No profiles selected";
        return result;
    }

    float h = state.height;
    float off = state.offset;
    bool flipDir = (state.operation == ExtrudeOperation::Cut);
    std::vector<TopoDS_Shape> shapes;

    for (int idx : state.selectedProfileIndices) {
        if (idx < 0 || idx >= (int)state.allProfiles.size()) {
            result.error = "Profile index out of range";
            return result;
        }
        const auto& profile = state.allProfiles[idx];
        TopoDS_Shape s;

        switch (state.direction) {
            case ExtrudeDirection::OneSide:
                s = extrudeProfileEx(sketch, profile, flipDir ? -h : h, off, plane); break;
            case ExtrudeDirection::OtherSide:
                s = extrudeProfileEx(sketch, profile, flipDir ? h : -h, off, plane); break;
            case ExtrudeDirection::Symmetric:
                s = extrudeProfileEx(sketch, profile, h, off - h * 0.5f, plane); break;
            case ExtrudeDirection::BothSides: {
                TopoDS_Shape s1 = extrudeProfileEx(sketch, profile, h, off, plane);
                TopoDS_Shape s2 = extrudeProfileEx(sketch, profile, -h, off, plane);
                if (!s1.IsNull()) shapes.push_back(s1);
                if (!s2.IsNull()) shapes.push_back(s2);
                if (s1.IsNull() && s2.IsNull()) {
                    result.error = "Failed to build profile wire or face (self-intersecting or degenerate profile?)";
                    return result;
                }
                continue;
            }
        }
        if (s.IsNull()) {
            result.error = "Failed to build profile wire or face (self-intersecting or degenerate profile?)";
            return result;
        }
        shapes.push_back(s);
    }

    if (shapes.empty()) {
        result.error = "No valid shapes produced";
        return result;
    }
    if (shapes.size() == 1) { result.shape = shapes[0]; return result; }

    BRep_Builder builder;
    TopoDS_Compound compound;
    builder.MakeCompound(compound);
    for (auto& s : shapes) builder.Add(compound, s);
    result.shape = compound;
    return result;
}

ShapeResult buildRevolveToolShapeEx(const RevolveToolState& state,
                                    const Sketch& sketch,
                                    const SketchPlane& plane) {
    ShapeResult result;
    if (state.axisLineID == NullID) { result.error = "No axis line selected"; return result; }
    const LineEntity* axisLine = sketch.findLine(state.axisLineID);
    if (!axisLine) { result.error = "Axis line not found"; return result; }

    Point2D axisA = sketch.getPointPos(axisLine->startPt);
    Point2D axisB = sketch.getPointPos(axisLine->endPt);
    if (distance(axisA, axisB) < 1e-6f) { result.error = "Axis line has zero length"; return result; }

    std::vector<TopoDS_Shape> shapes;
    for (int idx : state.selectedProfileIndices) {
        if (idx < 0 || idx >= (int)state.allProfiles.size()) continue;
        TopoDS_Shape s = revolveProfile(sketch, state.allProfiles[idx], axisA, axisB, state.angleDeg, plane);
        if (s.IsNull()) {
            result.error = "Failed to revolve profile (may overlap with axis or be self-intersecting)";
            return result;
        }
        shapes.push_back(s);
    }

    if (shapes.empty()) { result.error = "No valid shapes produced"; return result; }
    if (shapes.size() == 1) { result.shape = shapes[0]; return result; }

    BRep_Builder builder;
    TopoDS_Compound compound;
    builder.MakeCompound(compound);
    for (auto& s : shapes) builder.Add(compound, s);
    result.shape = compound;
    return result;
}

ShapeResult loftProfilesEx(const std::vector<LoftWireInput>& sections, bool solid) {
    ShapeResult result;
    if (sections.size() < 2) { result.error = "Need at least 2 sections for loft"; return result; }

    BRepOffsetAPI_ThruSections lofter(solid ? Standard_True : Standard_False);
    lofter.SetSmoothing(Standard_True);

    for (size_t i = 0; i < sections.size(); i++) {
        TopoDS_Wire wire = buildProfileWire(*sections[i].sketch, *sections[i].profile, *sections[i].plane);
        if (wire.IsNull()) {
            result.error = "Failed to build wire for section " + std::to_string(i + 1);
            return result;
        }
        lofter.AddWire(wire);
    }

    lofter.Build();
    if (!lofter.IsDone()) { result.error = "Loft operation failed (incompatible profiles or topologies)"; return result; }
    result.shape = lofter.Shape();
    return result;
}

} // namespace shitcad
