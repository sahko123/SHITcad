#include "FacePicker.h"

#include <BRep_Tool.hxx>
#include <BRepAdaptor_Surface.hxx>
#include <BRepAdaptor_Curve.hxx>
#include <GCPnts_TangentialDeflection.hxx>
#include <GeomAbs_SurfaceType.hxx>
#include <GeomAbs_CurveType.hxx>
#include <gp_Pln.hxx>
#include <gp_Lin.hxx>
#include <gp_Pnt.hxx>
#include <gp_Dir.hxx>
#include <gp_Circ.hxx>
#include <gp_Elips.hxx>
#include <Geom_BSplineCurve.hxx>
#include <IntCurvesFace_Intersector.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <cstdio>
#include <cmath>

namespace shitcad {

FacePickResult pickFace(const Scene3D& scene, const float rayOrigin[3], const float rayDir[3]) {
    FacePickResult best;

    gp_Pnt origin(rayOrigin[0], rayOrigin[1], rayOrigin[2]);
    gp_Dir direction(rayDir[0], rayDir[1], rayDir[2]);
    gp_Lin line(origin, direction);

    for (int bi = 0; bi < (int)scene.bodyCount(); bi++) {
        const Body3D& body = scene.getBody(bi);

        for (TopExp_Explorer exp(body.shape, TopAbs_FACE); exp.More(); exp.Next()) {
            const TopoDS_Face& face = TopoDS::Face(exp.Current());

            IntCurvesFace_Intersector inter(face, 1e-4);
            inter.Perform(line, 0.0, 1e10);

            for (int i = 1; i <= inter.NbPnt(); i++) {
                float t = (float)inter.WParameter(i);
                if (t > 0.001f && t < best.t) {
                    best.hit = true;
                    best.bodyIndex = bi;
                    best.face = face;
                    gp_Pnt p = inter.Pnt(i);
                    best.hitWorld[0] = (float)p.X();
                    best.hitWorld[1] = (float)p.Y();
                    best.hitWorld[2] = (float)p.Z();
                    best.t = t;
                }
            }
        }
    }

    return best;
}

MeshPickResult pickMesh(const Scene3D& scene, const float rayOrigin[3], const float rayDir[3]) {
    MeshPickResult best;

    // Double precision: vessel-scale coordinates in mm (~1e3) with float
    // cross products lose the small-triangle determinants.
    const double o[3] = {rayOrigin[0], rayOrigin[1], rayOrigin[2]};
    const double d[3] = {rayDir[0], rayDir[1], rayDir[2]};

    for (int bi = 0; bi < (int)scene.bodyCount(); bi++) {
        const Body3D& body = scene.getBody(bi);
        if (!body.visible || !body.isMeshOnly()) continue;

        const auto& v = body.vertices;
        for (size_t i = 0; i + 2 < v.size(); i += 3) {
            // Moller-Trumbore, two-sided: a nozzle is placed on whichever side
            // of the wall the user is looking at.
            const double p0[3] = {v[i].px, v[i].py, v[i].pz};
            const double e1[3] = {v[i + 1].px - p0[0], v[i + 1].py - p0[1], v[i + 1].pz - p0[2]};
            const double e2[3] = {v[i + 2].px - p0[0], v[i + 2].py - p0[1], v[i + 2].pz - p0[2]};

            const double pv[3] = {d[1] * e2[2] - d[2] * e2[1],
                                  d[2] * e2[0] - d[0] * e2[2],
                                  d[0] * e2[1] - d[1] * e2[0]};
            const double det = e1[0] * pv[0] + e1[1] * pv[1] + e1[2] * pv[2];
            if (std::fabs(det) < 1e-12) continue;
            const double inv = 1.0 / det;

            const double tv[3] = {o[0] - p0[0], o[1] - p0[1], o[2] - p0[2]};
            const double u = (tv[0] * pv[0] + tv[1] * pv[1] + tv[2] * pv[2]) * inv;
            if (u < 0.0 || u > 1.0) continue;

            const double qv[3] = {tv[1] * e1[2] - tv[2] * e1[1],
                                  tv[2] * e1[0] - tv[0] * e1[2],
                                  tv[0] * e1[1] - tv[1] * e1[0]};
            const double w = (d[0] * qv[0] + d[1] * qv[1] + d[2] * qv[2]) * inv;
            if (w < 0.0 || u + w > 1.0) continue;

            const double t = (e2[0] * qv[0] + e2[1] * qv[1] + e2[2] * qv[2]) * inv;
            if (t <= 1e-6 || t >= best.t) continue;

            double nx = e1[1] * e2[2] - e1[2] * e2[1];
            double ny = e1[2] * e2[0] - e1[0] * e2[2];
            double nz = e1[0] * e2[1] - e1[1] * e2[0];
            const double len = std::sqrt(nx * nx + ny * ny + nz * nz);
            if (len <= 0.0) continue;
            nx /= len; ny /= len; nz /= len;

            best.hit = true;
            best.bodyIndex = bi;
            best.triangleIndex = (int)(i / 3);
            best.t = (float)t;
            best.hitWorld[0] = (float)(o[0] + d[0] * t);
            best.hitWorld[1] = (float)(o[1] + d[1] * t);
            best.hitWorld[2] = (float)(o[2] + d[2] * t);
            best.normal[0] = (float)nx;
            best.normal[1] = (float)ny;
            best.normal[2] = (float)nz;
            best.frontFacing = (nx * d[0] + ny * d[1] + nz * d[2]) < 0.0;
        }
    }

    return best;
}

bool extractPlaneFromFace(const TopoDS_Face& face, SketchPlane& out,
                          const float* hitWorld) {
    BRepAdaptor_Surface adaptor(face);

    if (adaptor.GetType() == GeomAbs_Plane) {
        // Planar face: use the exact plane
        gp_Pln plane = adaptor.Plane();
        gp_Pnt loc = plane.Location();
        gp_Dir norm = plane.Axis().Direction();
        gp_Dir xDir = plane.XAxis().Direction();
        gp_Dir yDir = plane.YAxis().Direction();

        if (face.Orientation() == TopAbs_REVERSED)
            norm.Reverse();

        out.origin[0] = (float)loc.X();
        out.origin[1] = (float)loc.Y();
        out.origin[2] = (float)loc.Z();
        out.normal[0] = (float)norm.X();
        out.normal[1] = (float)norm.Y();
        out.normal[2] = (float)norm.Z();
        out.uAxis[0] = (float)xDir.X();
        out.uAxis[1] = (float)xDir.Y();
        out.uAxis[2] = (float)xDir.Z();
        out.vAxis[0] = (float)yDir.X();
        out.vAxis[1] = (float)yDir.Y();
        out.vAxis[2] = (float)yDir.Z();
        return true;
    }

    // Non-planar face: compute tangent plane at the hit point
    if (!hitWorld) {
        fprintf(stderr, "FacePicker: face is not planar and no hit point provided\n");
        return false;
    }

    // Find UV parameters closest to the hit point using ShapeAnalysis
    gp_Pnt hitPt(hitWorld[0], hitWorld[1], hitWorld[2]);

    // Project hit point onto the surface to get (u,v) parameters
    double u1 = adaptor.FirstUParameter(), u2 = adaptor.LastUParameter();
    double v1 = adaptor.FirstVParameter(), v2 = adaptor.LastVParameter();

    // Simple closest-point search on the parameter domain
    double bestU = (u1 + u2) * 0.5, bestV = (v1 + v2) * 0.5;
    double bestDist2 = 1e30;
    const int N = 32;
    for (int i = 0; i <= N; i++) {
        double u = u1 + (u2 - u1) * i / N;
        for (int j = 0; j <= N; j++) {
            double v = v1 + (v2 - v1) * j / N;
            gp_Pnt p = adaptor.Value(u, v);
            double d2 = p.SquareDistance(hitPt);
            if (d2 < bestDist2) {
                bestDist2 = d2;
                bestU = u;
                bestV = v;
            }
        }
    }

    // Refine with a few iterations of local search
    for (int iter = 0; iter < 4; iter++) {
        double du = (u2 - u1) / (N * std::pow(4.0, iter + 1));
        double dv = (v2 - v1) / (N * std::pow(4.0, iter + 1));
        double searchU = bestU, searchV = bestV;
        for (int i = -4; i <= 4; i++) {
            double u = searchU + du * i;
            for (int j = -4; j <= 4; j++) {
                double v = searchV + dv * j;
                if (u < u1 || u > u2 || v < v1 || v > v2) continue;
                gp_Pnt p = adaptor.Value(u, v);
                double d2 = p.SquareDistance(hitPt);
                if (d2 < bestDist2) {
                    bestDist2 = d2;
                    bestU = u;
                    bestV = v;
                }
            }
        }
    }

    // Evaluate surface normal and tangent vectors at the found (u,v)
    gp_Pnt surfPt;
    gp_Vec dU, dV;
    adaptor.D1(bestU, bestV, surfPt, dU, dV);

    if (dU.Magnitude() < 1e-10 || dV.Magnitude() < 1e-10) {
        fprintf(stderr, "FacePicker: degenerate surface tangent at hit point\n");
        return false;
    }

    gp_Vec norm = dU.Crossed(dV);
    if (norm.Magnitude() < 1e-10) {
        fprintf(stderr, "FacePicker: degenerate surface normal at hit point\n");
        return false;
    }
    norm.Normalize();

    if (face.Orientation() == TopAbs_REVERSED)
        norm.Reverse();

    // Build orthonormal frame: normalize dU, then compute dV as norm x dU
    dU.Normalize();
    gp_Vec uDir = dU;
    gp_Vec vDir = gp_Vec(norm).Crossed(uDir);
    vDir.Normalize();

    out.origin[0] = (float)surfPt.X();
    out.origin[1] = (float)surfPt.Y();
    out.origin[2] = (float)surfPt.Z();
    out.normal[0] = (float)norm.X();
    out.normal[1] = (float)norm.Y();
    out.normal[2] = (float)norm.Z();
    out.uAxis[0] = (float)uDir.X();
    out.uAxis[1] = (float)uDir.Y();
    out.uAxis[2] = (float)uDir.Z();
    out.vAxis[0] = (float)vDir.X();
    out.vAxis[1] = (float)vDir.Y();
    out.vAxis[2] = (float)vDir.Z();
    return true;
}

// Helper: project a 3D point to 2D sketch coords and add as a projected point
static EntityID addProjectedPoint(const SketchPlane& plane, Sketch& sketch,
                                   float wx, float wy, float wz) {
    float lx, ly;
    plane.worldToLocal(wx, wy, wz, lx, ly);
    EntityID ptID = sketch.genID();
    PointEntity pt;
    pt.id = ptID;
    pt.x = lx;
    pt.y = ly;
    pt.projected = true;
    sketch.points.push_back(pt);
    return ptID;
}

void projectFaceOntoSketch(const TopoDS_Face& face, const SketchPlane& plane, Sketch& sketch) {
    // For cylindrical/conical faces, replace the seam edge with silhouette lines
    BRepAdaptor_Surface surfAdaptor(face);
    bool isCylinder = (surfAdaptor.GetType() == GeomAbs_Cylinder);
    bool isCone = (surfAdaptor.GetType() == GeomAbs_Cone);

    if (isCylinder) {
        gp_Cylinder cyl = surfAdaptor.Cylinder();
        gp_Dir axisDir = cyl.Axis().Direction();
        double radius = cyl.Radius();

        // Compute the radial direction from cylinder axis toward the sketch plane
        gp_Vec planeN(plane.normal[0], plane.normal[1], plane.normal[2]);
        gp_Vec axisV(axisDir);
        gp_Vec radialDir = planeN - axisV * planeN.Dot(axisV);
        if (radialDir.Magnitude() > 1e-10) {
            radialDir.Normalize();

            // Find the height range by collecting all edge endpoints and projecting onto axis
            double hMin = 1e30, hMax = -1e30;
            gp_Pnt refPt; // a point on the axis
            bool foundRef = false;
            for (TopExp_Explorer edgeExp(face, TopAbs_EDGE); edgeExp.More(); edgeExp.Next()) {
                BRepAdaptor_Curve c(TopoDS::Edge(edgeExp.Current()));
                for (int ep = 0; ep < 2; ep++) {
                    gp_Pnt p = c.Value(ep == 0 ? c.FirstParameter() : c.LastParameter());
                    if (!foundRef) { refPt = p; foundRef = true; }
                    double h = gp_Vec(refPt, p).Dot(axisV);
                    if (h < hMin) hMin = h;
                    if (h > hMax) hMax = h;
                }
            }
            if (!foundRef || std::fabs(hMax - hMin) < 1e-10) goto skipSilhouette;

            // Compute the axis point at hMin (bottom) by projecting refPt onto axis
            {
                gp_Pnt axisLoc = cyl.Axis().Location();
                double t0 = gp_Vec(axisLoc, refPt).Dot(axisV); // refPt's axis parameter
                gp_Pnt bottomAxis(
                    axisLoc.X() + axisDir.X() * (t0 + hMin),
                    axisLoc.Y() + axisDir.Y() * (t0 + hMin),
                    axisLoc.Z() + axisDir.Z() * (t0 + hMin));
                gp_Pnt topAxis(
                    axisLoc.X() + axisDir.X() * (t0 + hMax),
                    axisLoc.Y() + axisDir.Y() * (t0 + hMax),
                    axisLoc.Z() + axisDir.Z() * (t0 + hMax));

                // Silhouette direction: perpendicular to both axis and view direction
                // These are the two lines where the cylinder surface is tangent to viewing rays
                gp_Vec silhouetteDir = axisV.Crossed(radialDir);
                if (silhouetteDir.Magnitude() < 1e-10) goto skipSilhouette;
                silhouetteDir.Normalize();

                // Two silhouette lines: left and right edges of the cylinder
                for (int side = 0; side < 2; side++) {
                    double sign = (side == 0) ? 1.0 : -1.0;
                    float bx = (float)(bottomAxis.X() + silhouetteDir.X() * radius * sign);
                    float by = (float)(bottomAxis.Y() + silhouetteDir.Y() * radius * sign);
                    float bz = (float)(bottomAxis.Z() + silhouetteDir.Z() * radius * sign);
                    float tx = (float)(topAxis.X() + silhouetteDir.X() * radius * sign);
                    float ty = (float)(topAxis.Y() + silhouetteDir.Y() * radius * sign);
                    float tz = (float)(topAxis.Z() + silhouetteDir.Z() * radius * sign);

                    EntityID startPt = addProjectedPoint(plane, sketch, bx, by, bz);
                    EntityID endPt = addProjectedPoint(plane, sketch, tx, ty, tz);

                    EntityID lineID = sketch.genID();
                    LineEntity line;
                    line.id = lineID;
                    line.startPt = startPt;
                    line.endPt = endPt;
                    line.projected = true;
                    sketch.lines.push_back(line);
                }
            }
        }
    }
    skipSilhouette:

    for (TopExp_Explorer exp(face, TopAbs_EDGE); exp.More(); exp.Next()) {
        const TopoDS_Edge& edge = TopoDS::Edge(exp.Current());
        BRepAdaptor_Curve curve(edge);
        GeomAbs_CurveType curveType = curve.GetType();

        if (curveType == GeomAbs_Line) {
            // Skip seam edges on cylindrical/conical faces — we generate silhouette lines instead
            if (isCylinder || isCone) {
                // Check if this line is a seam (same start and end vertex, or along the axis direction)
                gp_Pnt p1 = curve.Value(curve.FirstParameter());
                gp_Pnt p2 = curve.Value(curve.LastParameter());
                if (isCylinder) {
                    gp_Dir axisDir = surfAdaptor.Cylinder().Axis().Direction();
                    gp_Vec lineDir(p1, p2);
                    if (lineDir.Magnitude() > 1e-10) {
                        lineDir.Normalize();
                        double dotAxis = std::fabs(lineDir.Dot(gp_Vec(axisDir)));
                        if (dotAxis > 0.99) continue; // skip seam edge parallel to axis
                    }
                }
            }

            // Project as a native line entity
            gp_Pnt p1 = curve.Value(curve.FirstParameter());
            gp_Pnt p2 = curve.Value(curve.LastParameter());

            EntityID startPt = addProjectedPoint(plane, sketch,
                (float)p1.X(), (float)p1.Y(), (float)p1.Z());
            EntityID endPt = addProjectedPoint(plane, sketch,
                (float)p2.X(), (float)p2.Y(), (float)p2.Z());

            EntityID lineID = sketch.genID();
            LineEntity line;
            line.id = lineID;
            line.startPt = startPt;
            line.endPt = endPt;
            line.projected = true;
            sketch.lines.push_back(line);

        } else if (curveType == GeomAbs_Circle) {
            gp_Circ circ = curve.Circle();
            gp_Pnt center3D = circ.Location();
            float radius = (float)circ.Radius();

            // Check angle between circle's normal and sketch plane normal
            // to determine if the circle projects as a circle, ellipse, or line
            gp_Dir circNormal = circ.Axis().Direction();
            float dot = (float)(circNormal.X() * plane.normal[0] +
                                circNormal.Y() * plane.normal[1] +
                                circNormal.Z() * plane.normal[2]);
            float absDot = std::fabs(dot);

            float clx, cly;
            plane.worldToLocal((float)center3D.X(), (float)center3D.Y(), (float)center3D.Z(), clx, cly);

            // Check if this is a full circle or an arc
            double uFirst = curve.FirstParameter();
            double uLast = curve.LastParameter();
            double span = uLast - uFirst;
            bool isFullCircle = (std::fabs(span - 2.0 * 3.14159265358979) < 1e-3);

            if (absDot < 0.05f) {
                // Circle is nearly perpendicular to sketch plane — projects as a line segment
                // The line direction = circle normal × plane normal (intersection of the two planes)
                gp_Vec planeN(plane.normal[0], plane.normal[1], plane.normal[2]);
                gp_Vec lineDir3D = gp_Vec(circNormal).Crossed(planeN);
                if (lineDir3D.Magnitude() < 1e-10) continue; // degenerate
                lineDir3D.Normalize();

                // Endpoints: center ± radius along lineDir3D
                gp_Pnt p1(center3D.X() + lineDir3D.X() * radius,
                          center3D.Y() + lineDir3D.Y() * radius,
                          center3D.Z() + lineDir3D.Z() * radius);
                gp_Pnt p2(center3D.X() - lineDir3D.X() * radius,
                          center3D.Y() - lineDir3D.Y() * radius,
                          center3D.Z() - lineDir3D.Z() * radius);

                EntityID startPt = addProjectedPoint(plane, sketch,
                    (float)p1.X(), (float)p1.Y(), (float)p1.Z());
                EntityID endPt = addProjectedPoint(plane, sketch,
                    (float)p2.X(), (float)p2.Y(), (float)p2.Z());

                EntityID lineID = sketch.genID();
                LineEntity line;
                line.id = lineID;
                line.startPt = startPt;
                line.endPt = endPt;
                line.projected = true;
                sketch.lines.push_back(line);

            } else if (absDot > 0.99f) {
                // Circle is nearly parallel to sketch plane — projects as a circle
                if (isFullCircle) {
                    EntityID centerPtID = addProjectedPoint(plane, sketch,
                        (float)center3D.X(), (float)center3D.Y(), (float)center3D.Z());

                    EntityID circID = sketch.genID();
                    CircleEntity ce;
                    ce.id = circID;
                    ce.centerPt = centerPtID;
                    ce.radius = radius;
                    ce.projected = true;
                    sketch.circles.push_back(ce);
                } else {
                    gp_Pnt startPt3D = curve.Value(uFirst);
                    gp_Pnt endPt3D = curve.Value(uLast);

                    EntityID centerPtID = addProjectedPoint(plane, sketch,
                        (float)center3D.X(), (float)center3D.Y(), (float)center3D.Z());
                    EntityID startPtID = addProjectedPoint(plane, sketch,
                        (float)startPt3D.X(), (float)startPt3D.Y(), (float)startPt3D.Z());
                    EntityID endPtID = addProjectedPoint(plane, sketch,
                        (float)endPt3D.X(), (float)endPt3D.Y(), (float)endPt3D.Z());

                    EntityID arcID = sketch.genID();
                    ArcEntity arc;
                    arc.id = arcID;
                    arc.centerPt = centerPtID;
                    arc.startPt = startPtID;
                    arc.endPt = endPtID;
                    arc.projected = true;
                    float slx, sly, elx, ely;
                    plane.worldToLocal((float)startPt3D.X(), (float)startPt3D.Y(), (float)startPt3D.Z(), slx, sly);
                    plane.worldToLocal((float)endPt3D.X(), (float)endPt3D.Y(), (float)endPt3D.Z(), elx, ely);
                    arc.startAngle = std::atan2(sly - cly, slx - clx);
                    arc.endAngle = std::atan2(ely - cly, elx - clx);
                    sketch.arcs.push_back(arc);
                }

            } else {
                // Circle at an angle — projects as an ellipse
                // Semi-major = radius (unchanged), semi-minor = radius * |dot|
                float semiMajor = radius;
                float semiMinor = radius * absDot;

                // Compute the rotation of the projected ellipse
                // The major axis direction is the circle normal crossed with the plane normal,
                // projected onto the sketch plane
                gp_Vec circN(circNormal);
                gp_Vec planeN(plane.normal[0], plane.normal[1], plane.normal[2]);
                gp_Vec majorAxis3D = circN.Crossed(planeN);
                if (majorAxis3D.Magnitude() < 1e-10) {
                    majorAxis3D = gp_Vec(plane.uAxis[0], plane.uAxis[1], plane.uAxis[2]);
                } else {
                    majorAxis3D.Normalize();
                }
                float uComp = (float)(majorAxis3D.X() * plane.uAxis[0] +
                                       majorAxis3D.Y() * plane.uAxis[1] +
                                       majorAxis3D.Z() * plane.uAxis[2]);
                float vComp = (float)(majorAxis3D.X() * plane.vAxis[0] +
                                       majorAxis3D.Y() * plane.vAxis[1] +
                                       majorAxis3D.Z() * plane.vAxis[2]);
                float rotation = std::atan2(vComp, uComp);

                if (isFullCircle) {
                    EntityID centerPtID = addProjectedPoint(plane, sketch,
                        (float)center3D.X(), (float)center3D.Y(), (float)center3D.Z());

                    EntityID ellipseID = sketch.genID();
                    EllipseEntity ee;
                    ee.id = ellipseID;
                    ee.centerPt = centerPtID;
                    ee.semiMajor = semiMajor;
                    ee.semiMinor = semiMinor;
                    ee.rotation = rotation;
                    ee.projected = true;
                    sketch.ellipses.push_back(ee);
                } else {
                    // Sample the arc endpoints to get projected positions
                    gp_Pnt startPt3D = curve.Value(uFirst);
                    gp_Pnt endPt3D = curve.Value(uLast);

                    EntityID centerPtID = addProjectedPoint(plane, sketch,
                        (float)center3D.X(), (float)center3D.Y(), (float)center3D.Z());
                    EntityID startPtID = addProjectedPoint(plane, sketch,
                        (float)startPt3D.X(), (float)startPt3D.Y(), (float)startPt3D.Z());
                    EntityID endPtID = addProjectedPoint(plane, sketch,
                        (float)endPt3D.X(), (float)endPt3D.Y(), (float)endPt3D.Z());

                    EntityID eaID = sketch.genID();
                    EllipseArcEntity ea;
                    ea.id = eaID;
                    ea.centerPt = centerPtID;
                    ea.startPt = startPtID;
                    ea.endPt = endPtID;
                    ea.semiMajor = semiMajor;
                    ea.semiMinor = semiMinor;
                    ea.rotation = rotation;
                    ea.projected = true;

                    float slx, sly, elx, ely;
                    plane.worldToLocal((float)startPt3D.X(), (float)startPt3D.Y(), (float)startPt3D.Z(), slx, sly);
                    plane.worldToLocal((float)endPt3D.X(), (float)endPt3D.Y(), (float)endPt3D.Z(), elx, ely);
                    ea.startAngle = std::atan2(sly - cly, slx - clx);
                    ea.endAngle = std::atan2(ely - cly, elx - clx);
                    sketch.ellipseArcs.push_back(ea);
                }
            }

        } else if (curveType == GeomAbs_Ellipse) {
            gp_Elips elips = curve.Ellipse();
            gp_Pnt center3D = elips.Location();
            float semiMajor = (float)elips.MajorRadius();
            float semiMinor = (float)elips.MinorRadius();

            // Compute rotation angle of major axis projected onto sketch plane
            gp_Dir majorDir = elips.XAxis().Direction();
            float mx = (float)majorDir.X(), my = (float)majorDir.Y(), mz = (float)majorDir.Z();
            // Project major axis direction onto sketch plane's u/v axes
            float uComp = mx * plane.uAxis[0] + my * plane.uAxis[1] + mz * plane.uAxis[2];
            float vComp = mx * plane.vAxis[0] + my * plane.vAxis[1] + mz * plane.vAxis[2];
            float rotation = std::atan2(vComp, uComp);

            double uFirst = curve.FirstParameter();
            double uLast = curve.LastParameter();
            double span = uLast - uFirst;
            bool isFullEllipse = (std::fabs(span - 2.0 * 3.14159265358979) < 1e-3);

            if (isFullEllipse) {
                EntityID centerPtID = addProjectedPoint(plane, sketch,
                    (float)center3D.X(), (float)center3D.Y(), (float)center3D.Z());

                EntityID ellipseID = sketch.genID();
                EllipseEntity ee;
                ee.id = ellipseID;
                ee.centerPt = centerPtID;
                ee.semiMajor = semiMajor;
                ee.semiMinor = semiMinor;
                ee.rotation = rotation;
                ee.projected = true;
                sketch.ellipses.push_back(ee);
            } else {
                // Ellipse arc
                gp_Pnt startPt3D = curve.Value(uFirst);
                gp_Pnt endPt3D = curve.Value(uLast);

                EntityID centerPtID = addProjectedPoint(plane, sketch,
                    (float)center3D.X(), (float)center3D.Y(), (float)center3D.Z());
                EntityID startPtID = addProjectedPoint(plane, sketch,
                    (float)startPt3D.X(), (float)startPt3D.Y(), (float)startPt3D.Z());
                EntityID endPtID = addProjectedPoint(plane, sketch,
                    (float)endPt3D.X(), (float)endPt3D.Y(), (float)endPt3D.Z());

                EntityID eaID = sketch.genID();
                EllipseArcEntity ea;
                ea.id = eaID;
                ea.centerPt = centerPtID;
                ea.startPt = startPtID;
                ea.endPt = endPtID;
                ea.semiMajor = semiMajor;
                ea.semiMinor = semiMinor;
                ea.rotation = rotation;
                ea.projected = true;

                // Compute angles from projected 2D positions
                float clx, cly, slx, sly, elx, ely;
                plane.worldToLocal((float)center3D.X(), (float)center3D.Y(), (float)center3D.Z(), clx, cly);
                plane.worldToLocal((float)startPt3D.X(), (float)startPt3D.Y(), (float)startPt3D.Z(), slx, sly);
                plane.worldToLocal((float)endPt3D.X(), (float)endPt3D.Y(), (float)endPt3D.Z(), elx, ely);
                ea.startAngle = std::atan2(sly - cly, slx - clx);
                ea.endAngle = std::atan2(ely - cly, elx - clx);
                sketch.ellipseArcs.push_back(ea);
            }

        } else if (curveType == GeomAbs_BSplineCurve) {
            Handle(Geom_BSplineCurve) bspline = curve.BSpline();
            int nbPoles = bspline->NbPoles();

            SplineEntity sp;
            sp.id = sketch.genID();
            sp.degree = bspline->Degree();
            sp.periodic = bspline->IsPeriodic() ? true : false;
            sp.projected = true;

            // Project control points
            for (int i = 1; i <= nbPoles; i++) {
                gp_Pnt pole = bspline->Pole(i);
                EntityID ptID = addProjectedPoint(plane, sketch,
                    (float)pole.X(), (float)pole.Y(), (float)pole.Z());
                sp.controlPtIDs.push_back(ptID);
            }

            // Copy knots (flat knot vector)
            for (int i = 1; i <= bspline->NbKnots(); i++) {
                int mult = bspline->Multiplicity(i);
                for (int m = 0; m < mult; m++)
                    sp.knots.push_back((float)bspline->Knot(i));
            }

            // Copy weights if rational
            if (bspline->IsRational()) {
                for (int i = 1; i <= nbPoles; i++)
                    sp.weights.push_back((float)bspline->Weight(i));
            }

            sketch.splines.push_back(sp);

        } else {
            // Other curves: discretize into polyline
            GCPnts_TangentialDeflection discretizer(curve, 0.5, 0.3);
            int nbPts = discretizer.NbPoints();
            if (nbPts < 2) continue;

            EntityID prevPtID = NullID;
            for (int i = 1; i <= nbPts; i++) {
                gp_Pnt p = discretizer.Value(i);
                EntityID ptID = addProjectedPoint(plane, sketch,
                    (float)p.X(), (float)p.Y(), (float)p.Z());

                if (prevPtID != NullID) {
                    EntityID lineID = sketch.genID();
                    LineEntity line;
                    line.id = lineID;
                    line.startPt = prevPtID;
                    line.endPt = ptID;
                    line.projected = true;
                    sketch.lines.push_back(line);
                }
                prevPtID = ptID;
            }
        }
    }
}

TopoDS_Face findCoplanarFace(const Scene3D& scene, const SketchPlane& plane) {
    const float tol = 0.1f; // position tolerance
    const float angTol = 0.01f; // normal dot product tolerance (cos(~0.6°))

    for (int bi = 0; bi < (int)scene.bodyCount(); bi++) {
        const Body3D& body = scene.getBody(bi);
        for (TopExp_Explorer exp(body.shape, TopAbs_FACE); exp.More(); exp.Next()) {
            const TopoDS_Face& face = TopoDS::Face(exp.Current());
            BRepAdaptor_Surface adaptor(face);
            if (adaptor.GetType() != GeomAbs_Plane) continue;

            gp_Pln facePlane = adaptor.Plane();
            gp_Dir faceNorm = facePlane.Axis().Direction();
            if (face.Orientation() == TopAbs_REVERSED)
                faceNorm.Reverse();

            // Check normal alignment
            float dot = (float)(faceNorm.X() * plane.normal[0] +
                                faceNorm.Y() * plane.normal[1] +
                                faceNorm.Z() * plane.normal[2]);
            if (std::fabs(std::fabs(dot) - 1.0f) > angTol) continue;

            // Check that face plane passes through sketch origin
            gp_Pnt faceLoc = facePlane.Location();
            float dx = plane.origin[0] - (float)faceLoc.X();
            float dy = plane.origin[1] - (float)faceLoc.Y();
            float dz = plane.origin[2] - (float)faceLoc.Z();
            float distAlongNormal = std::fabs(dx * plane.normal[0] +
                                               dy * plane.normal[1] +
                                               dz * plane.normal[2]);
            if (distAlongNormal < tol) {
                return face;
            }
        }
    }
    return TopoDS_Face(); // null
}

bool isCylindricalFace(const TopoDS_Face& face) {
    if (face.IsNull()) return false;
    BRepAdaptor_Surface adaptor(face);
    return adaptor.GetType() == GeomAbs_Cylinder;
}

bool buildCylinderTangentPlane(const TopoDS_Face& face, float angleDeg,
                                const float* hitWorld, SketchPlane& out) {
    (void)hitWorld;
    if (face.IsNull()) return false;
    BRepAdaptor_Surface adaptor(face);
    if (adaptor.GetType() != GeomAbs_Cylinder) return false;

    gp_Cylinder cyl = adaptor.Cylinder();
    gp_Ax1 axis = cyl.Axis();
    gp_Dir axisDir = axis.Direction();
    gp_Pnt axisLoc = axis.Location();
    double radius = cyl.Radius();

    // Reference radial direction: +X axis projected onto the cylinder's cross-section
    // (so 0° always faces the global X direction)
    gp_Vec axisV(axisDir);
    gp_Vec worldX(1.0, 0.0, 0.0);
    gp_Vec radial = worldX - axisV * worldX.Dot(axisV);
    if (radial.Magnitude() < 1e-6) {
        // Cylinder axis is along X — fall back to Y axis
        gp_Vec worldY(0.0, 1.0, 0.0);
        radial = worldY - axisV * worldY.Dot(axisV);
    }
    radial.Normalize();

    // Rotate radial direction by angleDeg around the cylinder axis
    double angleRad = angleDeg * 3.14159265358979 / 180.0;
    gp_Vec perpToRadial = axisV.Crossed(radial);
    gp_Vec rotatedRadial = radial * std::cos(angleRad) + perpToRadial * std::sin(angleRad);
    rotatedRadial.Normalize();

    // Find the cylinder's actual height range from edge endpoints
    double hMin = 1e30, hMax = -1e30;
    for (TopExp_Explorer edgeExp(face, TopAbs_EDGE); edgeExp.More(); edgeExp.Next()) {
        BRepAdaptor_Curve c(TopoDS::Edge(edgeExp.Current()));
        for (int ep = 0; ep < 2; ep++) {
            gp_Pnt p = c.Value(ep == 0 ? c.FirstParameter() : c.LastParameter());
            double h = gp_Vec(axisLoc, p).Dot(axisV);
            if (h < hMin) hMin = h;
            if (h > hMax) hMax = h;
        }
    }
    double midH = (hMin + hMax) * 0.5;

    // Plane origin: on the cylinder surface at the rotated angle, at mid-height
    gp_Pnt origin(
        axisLoc.X() + axisDir.X() * midH + rotatedRadial.X() * radius,
        axisLoc.Y() + axisDir.Y() * midH + rotatedRadial.Y() * radius,
        axisLoc.Z() + axisDir.Z() * midH + rotatedRadial.Z() * radius);

    // Normal: radially outward (the rotated radial direction)
    gp_Vec normal = rotatedRadial;

    // U axis: along the cylinder axis
    gp_Vec uDir = axisV;

    // V axis: tangent to the cylinder (perpendicular to both normal and axis)
    gp_Vec vDir = normal.Crossed(uDir);
    vDir.Normalize();

    out.origin[0] = (float)origin.X();
    out.origin[1] = (float)origin.Y();
    out.origin[2] = (float)origin.Z();
    out.normal[0] = (float)normal.X();
    out.normal[1] = (float)normal.Y();
    out.normal[2] = (float)normal.Z();
    out.uAxis[0] = (float)uDir.X();
    out.uAxis[1] = (float)uDir.Y();
    out.uAxis[2] = (float)uDir.Z();
    out.vAxis[0] = (float)vDir.X();
    out.vAxis[1] = (float)vDir.Y();
    out.vAxis[2] = (float)vDir.Z();

    return true;
}

} // namespace shitcad
