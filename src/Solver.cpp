#include "Solver.h"
#include "Constants.h"
#include <cmath>
#include <algorithm>
#include <unordered_set>
#include <unordered_map>

namespace shitcad {

// Sentinel returned by the pair-weight helpers: neither side of the constraint may move.
static constexpr double kBothLocked = -1.0;

SolveResult Solver::solvePass(Sketch& sketch, EntityID draggedPoint) {
    SolveResult result;
    result.ok = true;
    result.dof = 0;
    result.converged = false;
    result.totalError = 0.0f;
    result.iterations = 0;

    // Simple iterative constraint solver.
    // Handles: Horizontal, Vertical, Coincident, Distance.
    // Run multiple iterations to converge (constraints can chain).

    sketch.rebuildIndices();

    // --- Topology-derived reference counts ------------------------------------------------
    // How constrained a point is: entity connections + constraints referencing it. Points with
    // more references get moved less. Topology cannot change during a solve, so this is
    // loop-invariant and is now computed once. It used to be an O(entities + constraints) scan
    // re-run for every reference of every constraint on every one of 40 iterations, while the
    // solve itself runs on every frame of a drag.
    std::unordered_map<EntityID, int> refCount;
    refCount.reserve(sketch.points.size() * 2 + 1);
    auto bump = [&](EntityID id) { if (id != NullID) refCount[id]++; };
    for (const auto& l  : sketch.lines)       { if (!l.projected)  { bump(l.startPt); bump(l.endPt); } }
    for (const auto& ci : sketch.circles)     { if (!ci.projected) { bump(ci.centerPt); } }
    for (const auto& ar : sketch.arcs)        { if (!ar.projected) { bump(ar.centerPt); bump(ar.startPt); bump(ar.endPt); } }
    for (const auto& el : sketch.ellipses)    { if (!el.projected) { bump(el.centerPt); } }
    for (const auto& ea : sketch.ellipseArcs) { if (!ea.projected) { bump(ea.centerPt); bump(ea.startPt); bump(ea.endPt); } }
    for (const auto& sp : sketch.splines) {
        if (sp.projected) continue;
        std::unordered_set<EntityID> seen;               // one count per spline, as before
        for (auto cpID : sp.controlPtIDs) if (seen.insert(cpID).second) bump(cpID);
    }
    for (const auto& c : sketch.constraints) {
        if (c.entityA != NullID) refCount[c.entityA]++;
        if (c.entityB != NullID && c.entityB != c.entityA) refCount[c.entityB]++;
    }

    // --- Immovability ---------------------------------------------------------------------
    // A point is locked if it is projected reference geometry, or if it is the point the user is
    // currently dragging. Locked points are never written: the whole correction goes to the other
    // side of the constraint. Previously "locked" was only a very large ref-count weight, which
    // still let projected geometry creep by ~1/10000 of each correction per iteration, and let
    // the dragged point be averaged straight back off the cursor.
    std::unordered_set<EntityID> lockedSet;
    for (const auto& pt : sketch.points)
        if (pt.projected || (draggedPoint != NullID && pt.id == draggedPoint))
            lockedSet.insert(pt.id);

    auto locked = [&](EntityID ptID) -> bool {
        return ptID != NullID && lockedSet.count(ptID) != 0;
    };
    auto countRefs = [&](EntityID ptID) -> int {
        if (locked(ptID)) return kProjectedPointRefCount;
        auto it = refCount.find(ptID);
        return (it == refCount.end()) ? 0 : it->second;
    };
    auto countLineRefs = [&](const LineEntity* line) -> int {
        return countRefs(line->startPt) + countRefs(line->endPt);
    };
    auto lineLocked = [&](const LineEntity* line) -> bool {
        return locked(line->startPt) && locked(line->endPt);
    };

    // Weight of A within a pair: the fraction of the pair's immobility owned by A. wA == 1 means
    // A is held still and B absorbs the whole correction. kBothLocked means neither side may
    // move, in which case the constraint is skipped for this pass.
    auto blendWeight = [](int ra, int rb) -> double {
        double t = (double)(ra + rb);
        if (t < 1.0) t = 1.0;
        return (double)ra / t;
    };
    auto weightPP = [&](EntityID aID, EntityID bID) -> double {            // point vs point
        bool la = locked(aID), lb = locked(bID);
        if (la && lb) return kBothLocked;
        if (la) return 1.0;
        if (lb) return 0.0;
        return blendWeight(countRefs(aID), countRefs(bID));
    };
    auto weightPL = [&](EntityID ptID, const LineEntity* line) -> double {  // point vs line
        bool kp = locked(ptID), kl = lineLocked(line);
        if (kp && kl) return kBothLocked;
        if (kp) return 1.0;
        if (kl) return 0.0;
        return blendWeight(countRefs(ptID), countLineRefs(line));
    };
    auto weightLL = [&](const LineEntity* l1, const LineEntity* l2) -> double { // line vs line
        bool k1 = lineLocked(l1), k2 = lineLocked(l2);
        if (k1 && k2) return kBothLocked;
        if (k1) return 1.0;
        if (k2) return 0.0;
        return blendWeight(countLineRefs(l1), countLineRefs(l2));
    };

    // --- Vertex identity ------------------------------------------------------------------
    // Points joined by a Coincident constraint are geometrically one vertex. Both the
    // shared-vertex tests and the DOF count need to treat them as one. Union-find, built once.
    std::unordered_map<EntityID, EntityID> ufParent;
    for (const auto& pt : sketch.points) ufParent[pt.id] = pt.id;
    auto ufFind = [&](EntityID x) -> EntityID {
        for (int guard = 0; guard < 4096; guard++) {
            auto it = ufParent.find(x);
            if (it == ufParent.end() || it->second == x) break;
            x = it->second;
        }
        return x;
    };
    for (const auto& c : sketch.constraints) {
        if (c.driven || c.type != ConstraintType::Coincident) continue;
        if (!sketch.findPoint(c.entityA) || !sketch.findPoint(c.entityB)) continue;
        EntityID ra = ufFind(c.entityA), rb = ufFind(c.entityB);
        if (ra != rb) ufParent[rb] = ra;
    }
    auto sameVertex = [&](EntityID a, EntityID b) -> bool {
        return a != NullID && b != NullID && (a == b || ufFind(a) == ufFind(b));
    };
    // Shared vertex between two lines, by entity identity. This used to compare positions with a
    // 1e-3 epsilon, which both missed genuinely-shared vertices that had not yet converged to
    // within it (tearing the corner apart on the next rotation) and fused distinct points that
    // happened to be less than a micron apart.
    auto sharedVertexOf = [&](const LineEntity* l1, const LineEntity* l2) -> EntityID {
        if (sameVertex(l1->startPt, l2->startPt) || sameVertex(l1->startPt, l2->endPt)) return l1->startPt;
        if (sameVertex(l1->endPt,   l2->startPt) || sameVertex(l1->endPt,   l2->endPt)) return l1->endPt;
        return NullID;
    };

    // --- Fillet arcs ----------------------------------------------------------------------
    // A fillet is the arc the fillet tool creates: a Radius constraint plus a Tangent to each of
    // the two adjacent lines, endpoints shared with those lines. Detecting it from the constraint
    // set rather than from endpoint reference counts matters: a ref-count test also matches any
    // ordinary arc chained into a profile, and the fillet special-casing below would then
    // silently drop that arc's equal-radius invariant, which every consumer relies on when it
    // derives the arc radius from the start point alone.
    struct FilletInfo { EntityID lineA = NullID, lineB = NullID; };
    std::unordered_map<EntityID, FilletInfo> filletArcs;
    {
        std::unordered_set<EntityID> hasRadius;
        for (const auto& c : sketch.constraints) {
            if (c.driven) continue;
            if ((c.type == ConstraintType::Radius || c.type == ConstraintType::Diameter) &&
                c.entityA != NullID)
                hasRadius.insert(c.entityA);
        }
        std::unordered_map<EntityID, FilletInfo> tangents;
        std::unordered_map<EntityID, int> tangentCount;
        for (const auto& c : sketch.constraints) {
            if (c.driven || c.type != ConstraintType::Tangent) continue;
            if (c.entityB == NullID || !sketch.findArc(c.entityB)) continue;
            if (!sketch.findLine(c.entityA)) continue;
            FilletInfo& fi = tangents[c.entityB];
            if      (fi.lineA == NullID) fi.lineA = c.entityA;
            else if (fi.lineB == NullID) fi.lineB = c.entityA;
            tangentCount[c.entityB]++;
        }
        for (const auto& ar : sketch.arcs) {
            auto it = tangents.find(ar.id);
            if (it == tangents.end() || it->second.lineB == NullID) continue;
            if (tangentCount[ar.id] != 2 || !hasRadius.count(ar.id)) continue;
            filletArcs[ar.id] = it->second;
        }
    }

    // --- Motion helpers -------------------------------------------------------------------
    // Every point write goes through these: they refuse to move locked points, apply the current
    // under-relaxation factor, and record the largest correction so that convergence is judged on
    // correction magnitude rather than on "did anything at all change".
    double relax = 1.0;
    double maxDelta = 0.0;
    double prevMaxDelta = 0.0;
    auto noteDelta = [&](double d) { if (d > maxDelta) maxDelta = d; };
    auto moveBy = [&](PointEntity* p, double dx, double dy) {
        if (!p || lockedSet.count(p->id)) return;
        dx *= relax; dy *= relax;
        noteDelta(std::fabs(dx) + std::fabs(dy));
        p->x += dx; p->y += dy;
    };
    auto moveTo = [&](PointEntity* p, double tx, double ty) {
        if (!p || lockedSet.count(p->id)) return;
        moveBy(p, tx - p->x, ty - p->y);
    };

    for (int iter = 0; iter < kSolverMaxIterations; iter++) {
        bool changed = false;
        maxDelta = 0.0;

        for (const auto& c : sketch.constraints) {
            if (c.driven) continue; // driven dimensions are reference-only
            switch (c.type) {
                case ConstraintType::Horizontal: {
                    LineEntity* line = sketch.findLine(c.entityA);
                    if (!line) break;
                    PointEntity* a = sketch.findPoint(line->startPt);
                    PointEntity* b = sketch.findPoint(line->endPt);
                    if (!a || !b) break;

                    if (std::fabs(a->y - b->y) > 1e-6) {
                        double wA = weightPP(line->startPt, line->endPt);
                        if (wA == kBothLocked) break;
                        double avg = a->y * wA + b->y * (1.0 - wA);
                        moveTo(a, a->x, avg);
                        moveTo(b, b->x, avg);
                        changed = true;
                    }
                    break;
                }

                case ConstraintType::Vertical: {
                    LineEntity* line = sketch.findLine(c.entityA);
                    if (!line) break;
                    PointEntity* a = sketch.findPoint(line->startPt);
                    PointEntity* b = sketch.findPoint(line->endPt);
                    if (!a || !b) break;

                    if (std::fabs(a->x - b->x) > 1e-6) {
                        double wA = weightPP(line->startPt, line->endPt);
                        if (wA == kBothLocked) break;
                        double avg = a->x * wA + b->x * (1.0 - wA);
                        moveTo(a, avg, a->y);
                        moveTo(b, avg, b->y);
                        changed = true;
                    }
                    break;
                }

                case ConstraintType::Coincident: {
                    PointEntity* a = sketch.findPoint(c.entityA);
                    PointEntity* b = sketch.findPoint(c.entityB);
                    if (!a || !b) break;

                    if (std::fabs(a->x - b->x) > 1e-6 || std::fabs(a->y - b->y) > 1e-6) {
                        double wA = weightPP(c.entityA, c.entityB);
                        if (wA == kBothLocked) break;
                        double tX = a->x * wA + b->x * (1.0 - wA);
                        double tY = a->y * wA + b->y * (1.0 - wA);
                        moveTo(a, tX, tY);
                        moveTo(b, tX, tY);
                        changed = true;
                    }
                    break;
                }

                case ConstraintType::Distance: {
                    // Distance constraint on a line: set the line to the target length
                    LineEntity* line = sketch.findLine(c.entityA);
                    if (!line) break;
                    PointEntity* a = sketch.findPoint(line->startPt);
                    PointEntity* b = sketch.findPoint(line->endPt);
                    if (!a || !b) break;

                    double dx = b->x - a->x;
                    double dy = b->y - a->y;
                    double currentLen = std::sqrt(dx * dx + dy * dy);
                    double targetLen = c.value;

                    if (currentLen < 1e-6) break;
                    if (std::fabs(currentLen - targetLen) < 1e-6) break;

                    // Weighted pivot: more-constrained endpoint moves less
                    double wA = weightPP(line->startPt, line->endPt);
                    if (wA == kBothLocked) break;
                    // Pivot biased toward more-constrained end
                    double pivotX = a->x * wA + b->x * (1.0 - wA);
                    double pivotY = a->y * wA + b->y * (1.0 - wA);
                    double ux = dx / currentLen, uy = dy / currentLen;
                    // Place endpoints at target distance from pivot, preserving direction
                    double distA = currentLen * (1.0 - wA); // a's distance from pivot
                    double distB = currentLen * wA;          // b's distance from pivot
                    double scale = targetLen / currentLen;
                    moveTo(a, pivotX - ux * distA * scale, pivotY - uy * distA * scale);
                    moveTo(b, pivotX + ux * distB * scale, pivotY + uy * distB * scale);
                    changed = true;
                    break;
                }

                case ConstraintType::PointDistance: {
                    PointEntity* a = sketch.findPoint(c.entityA);
                    PointEntity* b = sketch.findPoint(c.entityB);
                    if (!a || !b) break;

                    double dx = b->x - a->x;
                    double dy = b->y - a->y;
                    double currentLen = std::sqrt(dx * dx + dy * dy);
                    double targetLen = c.value;

                    if (currentLen < 1e-6) break;
                    if (std::fabs(currentLen - targetLen) < 1e-6) break;

                    double wA = weightPP(c.entityA, c.entityB);
                    if (wA == kBothLocked) break;
                    double pivotX = a->x * wA + b->x * (1.0 - wA);
                    double pivotY = a->y * wA + b->y * (1.0 - wA);
                    double ux = dx / currentLen, uy = dy / currentLen;
                    double distA = currentLen * (1.0 - wA);
                    double distB = currentLen * wA;
                    double scale = targetLen / currentLen;
                    moveTo(a, pivotX - ux * distA * scale, pivotY - uy * distA * scale);
                    moveTo(b, pivotX + ux * distB * scale, pivotY + uy * distB * scale);
                    changed = true;
                    break;
                }

                case ConstraintType::Radius: {
                    CircleEntity* circle = sketch.findCircle(c.entityA);
                    if (circle) {
                        if (std::fabs(circle->radius - c.value) > 1e-6) {
                            noteDelta(std::fabs(circle->radius - c.value));
                            circle->radius = c.value;
                            changed = true;
                        }
                    } else {
                        ArcEntity* arc = sketch.findArc(c.entityA);
                        if (arc) {
                            PointEntity* cp = sketch.findPoint(arc->centerPt);
                            PointEntity* sp = sketch.findPoint(arc->startPt);
                            PointEntity* ep = sketch.findPoint(arc->endPt);
                            if (cp && sp && ep) {
                                double targetR = c.value;
                                // A fillet's endpoints are the tangent trim points on its two
                                // adjacent lines, so its radius is set by moving the centre --
                                // scaling the endpoints would drag those trim points off the
                                // lines they sit on. A standalone arc's centre is the stable
                                // part, so its radius is set by scaling the endpoints out from
                                // it. This used to be decided by comparing endpoint reference
                                // counts, which also classified ordinary chained arcs as fillets.
                                if (filletArcs.count(arc->id)) {
                                    double dx = sp->x - cp->x, dy = sp->y - cp->y;
                                    double curR = std::sqrt(dx*dx + dy*dy);
                                    if (curR > 1e-7 && std::fabs(curR - targetR) > 1e-6) {
                                        moveTo(cp, sp->x - targetR * dx / curR,
                                                   sp->y - targetR * dy / curR);
                                        sketch.recomputeArcAngles(*arc);
                                        changed = true;
                                    }
                                } else {
                                    // Standalone arc: scale endpoints radially from center
                                    auto scalePoint = [&](PointEntity* pt) {
                                        double dx = pt->x - cp->x, dy = pt->y - cp->y;
                                        double curR = std::sqrt(dx*dx + dy*dy);
                                        if (curR > 1e-7) {
                                            double scale = targetR / curR;
                                            moveTo(pt, cp->x + dx * scale, cp->y + dy * scale);
                                        }
                                    };
                                    scalePoint(sp);
                                    scalePoint(ep);
                                    sketch.recomputeArcAngles(*arc);
                                    changed = true;
                                }
                            }
                        }
                    }
                    break;
                }

                case ConstraintType::Diameter: {
                    CircleEntity* circle = sketch.findCircle(c.entityA);
                    if (!circle) break;
                    double targetR = c.value * 0.5;
                    if (std::fabs(circle->radius - targetR) > 1e-6) {
                        noteDelta(std::fabs(circle->radius - targetR));
                        circle->radius = targetR;
                        changed = true;
                    }
                    break;
                }

                case ConstraintType::Angle: {
                    // Angle between two lines (value in degrees)
                    LineEntity* line1 = sketch.findLine(c.entityA);
                    LineEntity* line2 = sketch.findLine(c.entityB);
                    if (!line1 || !line2) break;

                    PointEntity* a1 = sketch.findPoint(line1->startPt);
                    PointEntity* b1 = sketch.findPoint(line1->endPt);
                    PointEntity* a2 = sketch.findPoint(line2->startPt);
                    PointEntity* b2 = sketch.findPoint(line2->endPt);
                    if (!a1 || !b1 || !a2 || !b2) break;

                    // Find vertex: shared endpoint by identity, else infinite-line intersection.
                    double vx = 0.0, vy = 0.0;
                    bool sharedVertex = false;
                    if (EntityID svID = sharedVertexOf(line1, line2)) {
                        if (const PointEntity* sv = sketch.findPoint(svID)) {
                            vx = sv->x; vy = sv->y; sharedVertex = true;
                        }
                    }
                    if (!sharedVertex) {
                        // Compute intersection of infinite lines
                        double ldx1 = b1->x-a1->x, ldy1 = b1->y-a1->y;
                        double ldx2 = b2->x-a2->x, ldy2 = b2->y-a2->y;
                        double denom = ldx1*ldy2 - ldy1*ldx2;
                        if (std::fabs(denom) < 1e-6) break; // parallel
                        double t = ((a2->x-a1->x)*ldy2 - (a2->y-a1->y)*ldx2) / denom;
                        vx = a1->x + t*ldx1; vy = a1->y + t*ldy1;
                    }

                    // Direction from vertex toward farther endpoint of each line
                    auto dist2 = [](double ax, double ay, double bx, double by) {
                        double dx = ax-bx, dy = ay-by; return dx*dx + dy*dy;
                    };
                    PointEntity* far1 = (dist2(vx,vy,b1->x,b1->y) >= dist2(vx,vy,a1->x,a1->y)) ? b1 : a1;
                    PointEntity* far2 = (dist2(vx,vy,b2->x,b2->y) >= dist2(vx,vy,a2->x,a2->y)) ? b2 : a2;

                    double dx1 = far1->x - vx, dy1 = far1->y - vy;
                    double dx2 = far2->x - vx, dy2 = far2->y - vy;
                    double len1 = std::sqrt(dx1*dx1 + dy1*dy1);
                    double len2 = std::sqrt(dx2*dx2 + dy2*dy2);
                    if (len1 < 1e-6 || len2 < 1e-6) break;

                    double dot = dx1*dx2 + dy1*dy2;
                    double cross = dx1*dy2 - dy1*dx2;
                    double ccwRad = std::atan2(cross, dot);
                    if (ccwRad < 0) ccwRad += kTwoPiD;
                    double cwRad = kTwoPiD - ccwRad;
                    // Use the stored sector (CW or CCW) from when the constraint was created
                    double currentAngle, targetAngle;
                    bool cwSector = c.angleCW;
                    if (!cwSector) {
                        currentAngle = ccwRad;
                        targetAngle = c.value * kDegToRadD;
                    } else {
                        currentAngle = cwRad;
                        targetAngle = c.value * kDegToRadD;
                    }

                    double diff = targetAngle - currentAngle;
                    if (diff > kPiD) diff -= kTwoPiD;
                    if (diff < -kPiD) diff += kTwoPiD;
                    if (std::fabs(diff) < 1e-6) break;

                    // CW sector: rotation direction is inverted
                    double totalRot = cwSector ? -diff : diff;

                    // Weight: more-constrained line rotates less
                    double w1 = weightLL(line1, line2);
                    if (w1 == kBothLocked) break;
                    double rot2 = totalRot * w1;           // line2's rotation
                    double rot1 = -totalRot * (1.0 - w1); // line1's rotation (opposite)

                    auto rotatePt = [&](PointEntity* pt, double ox, double oy, double cosR, double sinR) {
                        double rx = pt->x-ox, ry = pt->y-oy;
                        moveTo(pt, ox + rx*cosR - ry*sinR, oy + rx*sinR + ry*cosR);
                    };

                    if (sharedVertex) {
                        // Rotate far endpoints around shared vertex
                        PointEntity* far1_ = (dist2(vx,vy,b1->x,b1->y) >= dist2(vx,vy,a1->x,a1->y)) ? b1 : a1;
                        double cos1 = std::cos(rot1), sin1 = std::sin(rot1);
                        double cos2 = std::cos(rot2), sin2 = std::sin(rot2);
                        rotatePt(far1_, vx, vy, cos1, sin1);
                        rotatePt(far2, vx, vy, cos2, sin2);
                    } else {
                        // Rotate each line around the intersection point
                        double cos1 = std::cos(rot1), sin1 = std::sin(rot1);
                        double cos2 = std::cos(rot2), sin2 = std::sin(rot2);
                        rotatePt(a1, vx, vy, cos1, sin1);
                        rotatePt(b1, vx, vy, cos1, sin1);
                        rotatePt(a2, vx, vy, cos2, sin2);
                        rotatePt(b2, vx, vy, cos2, sin2);
                    }
                    changed = true;
                    break;
                }

                case ConstraintType::PointLineDistance: {
                    // entityA = point, entityB = line
                    // value = target distance (negative flips side)
                    // negativeSide = initial side (false = positive, true = negative)
                    PointEntity* pt = sketch.findPoint(c.entityA);
                    LineEntity* line = sketch.findLine(c.entityB);
                    if (!pt || !line) break;
                    PointEntity* la = sketch.findPoint(line->startPt);
                    PointEntity* lb = sketch.findPoint(line->endPt);
                    if (!la || !lb) break;

                    double ldx = lb->x - la->x, ldy = lb->y - la->y;
                    double lineLen2 = ldx*ldx + ldy*ldy;
                    if (lineLen2 < 1e-12) break;
                    double lineLen = std::sqrt(lineLen2);

                    // Signed perpendicular distance from point to line
                    double cross = (pt->x - la->x)*ldy - (pt->y - la->y)*ldx;
                    double dist = cross / lineLen;

                    // Target signed distance: initial side * value
                    // Negative value flips the side
                    double sideSign = c.negativeSide ? -1.0 : 1.0;
                    double targetSigned = sideSign * c.value;

                    if (std::fabs(dist - targetSigned) < 1e-6) break;

                    double nx = -ldy / lineLen;
                    double ny = ldx / lineLen;
                    double error = targetSigned - dist;

                    // Weight: more constrained entity moves less.
                    // lineFrac = how much point moves (high when line is constrained)
                    // ptFrac   = how much line moves (high when point is constrained)
                    double wPt = weightPL(c.entityA, line);
                    if (wPt == kBothLocked) break;
                    double lineFrac = 1.0 - wPt;
                    double ptFrac   = wPt;

                    // Point target: project onto line, then offset to target distance
                    // pt = proj - normal * targetSigned (derived from sign convention)
                    double t = ((pt->x - la->x)*ldx + (pt->y - la->y)*ldy) / lineLen2;
                    double projX = la->x + t*ldx;
                    double projY = la->y + t*ldy;
                    double ptTargetX = projX - nx * targetSigned;
                    double ptTargetY = projY - ny * targetSigned;
                    double ptMoveX = ptTargetX - pt->x;
                    double ptMoveY = ptTargetY - pt->y;

                    // Line shift: moving line by (nx*s) changes dist by +s
                    // We need dist to change by +error, so s = error
                    double lineShift = error;

                    // Apply weighted
                    moveBy(pt, ptMoveX * lineFrac, ptMoveY * lineFrac);
                    double ls = lineShift * ptFrac;
                    moveBy(la, nx * ls, ny * ls);
                    moveBy(lb, nx * ls, ny * ls);
                    changed = true;
                    break;
                }

                case ConstraintType::PointOnLine: {
                    // Move point onto line, weighted by ref counts
                    PointEntity* pt = sketch.findPoint(c.entityA);
                    LineEntity* line = sketch.findLine(c.entityB);
                    if (!pt || !line) break;
                    PointEntity* la = sketch.findPoint(line->startPt);
                    PointEntity* lb = sketch.findPoint(line->endPt);
                    if (!la || !lb) break;

                    double ldx = lb->x - la->x, ldy = lb->y - la->y;
                    double len2 = ldx*ldx + ldy*ldy;
                    if (len2 < 1e-12) break;
                    double lineLen = std::sqrt(len2);

                    // Signed perpendicular distance from point to line
                    double cross = (pt->x - la->x)*ldy - (pt->y - la->y)*ldx;
                    double dist = cross / lineLen;
                    if (std::fabs(dist) < 1e-6) break;

                    // Normal direction (perpendicular to line)
                    double nx = -ldy / lineLen;
                    double ny = ldx / lineLen;

                    // Weight: point vs line
                    double wPt = weightPL(c.entityA, line);
                    if (wPt == kBothLocked) break;
                    double lineFrac = 1.0 - wPt; // how much point moves
                    double ptFrac   = wPt;       // how much line moves

                    // Move point toward line
                    moveBy(pt, nx * dist * lineFrac, ny * dist * lineFrac);
                    // Shift line toward point
                    double ls = -dist * ptFrac;
                    moveBy(la, nx * ls, ny * ls);
                    moveBy(lb, nx * ls, ny * ls);
                    changed = true;
                    break;
                }

                case ConstraintType::PointOnCircle: {
                    // entityA = point ID, entityB = circle or arc ID
                    PointEntity* pt = sketch.findPoint(c.entityA);
                    if (!pt) break;
                    // A projected or dragged point is immovable, so the curve centre absorbs the
                    // correction -- which is what "this point lies on that circle" means when the
                    // point is the fixed reference. This used to skip the constraint outright.

                    EntityID centerPtID = NullID;
                    double radius = 0.0;
                    CircleEntity* circ = sketch.findCircle(c.entityB);
                    if (circ) {
                        centerPtID = circ->centerPt;
                        radius = circ->radius;
                    } else {
                        ArcEntity* arc = sketch.findArc(c.entityB);
                        if (arc) {
                            centerPtID = arc->centerPt;
                            PointEntity* sp = sketch.findPoint(arc->startPt);
                            PointEntity* cp = sketch.findPoint(arc->centerPt);
                            if (sp && cp) {
                                double dx = sp->x - cp->x, dy = sp->y - cp->y;
                                radius = std::sqrt(dx*dx + dy*dy);
                            }
                        }
                    }
                    if (centerPtID == NullID || radius < 1e-7) break;
                    PointEntity* cen = sketch.findPoint(centerPtID);
                    if (!cen) break;

                    double dx = pt->x - cen->x, dy = pt->y - cen->y;
                    double dist = std::sqrt(dx*dx + dy*dy);
                    if (dist < 1e-7) break; // point at center, can't determine direction
                    double err = dist - radius;
                    if (std::fabs(err) < 1e-6) break;

                    double nx = dx / dist, ny = dy / dist;

                    double wPt = weightPP(c.entityA, centerPtID);
                    if (wPt == kBothLocked) break;
                    double cenFrac = 1.0 - wPt;
                    double ptFrac  = wPt;

                    moveBy(pt,  -nx * err * cenFrac, -ny * err * cenFrac);
                    moveBy(cen,  nx * err * ptFrac,   ny * err * ptFrac);
                    changed = true;
                    break;
                }

                case ConstraintType::EqualLength: {
                    LineEntity* l1 = sketch.findLine(c.entityA);
                    LineEntity* l2 = sketch.findLine(c.entityB);
                    if (!l1 || !l2) break;
                    PointEntity* a1 = sketch.findPoint(l1->startPt);
                    PointEntity* b1 = sketch.findPoint(l1->endPt);
                    PointEntity* a2 = sketch.findPoint(l2->startPt);
                    PointEntity* b2 = sketch.findPoint(l2->endPt);
                    if (!a1 || !b1 || !a2 || !b2) break;

                    double dx1 = b1->x-a1->x, dy1 = b1->y-a1->y;
                    double dx2 = b2->x-a2->x, dy2 = b2->y-a2->y;
                    double len1 = std::sqrt(dx1*dx1 + dy1*dy1);
                    double len2 = std::sqrt(dx2*dx2 + dy2*dy2);
                    if (len1 < 1e-6 || len2 < 1e-6) break;

                    if (std::fabs(len1 - len2) < 1e-6) break;

                    // Weighted target: more-constrained line changes less
                    double w1 = weightLL(l1, l2);
                    if (w1 == kBothLocked) break;
                    // Target length biased toward more-constrained line
                    double target = len1 * w1 + len2 * (1.0 - w1);

                    // Scale each line from weighted pivot
                    auto scaleLine = [&](PointEntity* a, PointEntity* b, double dx, double dy, double curLen,
                                         EntityID startPt, EntityID endPt) {
                        double wa = weightPP(startPt, endPt);
                        if (wa == kBothLocked) return;
                        double pivX = a->x * wa + b->x * (1.0 - wa);
                        double pivY = a->y * wa + b->y * (1.0 - wa);
                        double ux = dx / curLen, uy = dy / curLen;
                        double dA = curLen * (1.0 - wa);
                        double dB = curLen * wa;
                        double s = target / curLen;
                        moveTo(a, pivX - ux * dA * s, pivY - uy * dA * s);
                        moveTo(b, pivX + ux * dB * s, pivY + uy * dB * s);
                    };
                    scaleLine(a1, b1, dx1, dy1, len1, l1->startPt, l1->endPt);
                    scaleLine(a2, b2, dx2, dy2, len2, l2->startPt, l2->endPt);
                    changed = true;
                    break;
                }

                case ConstraintType::Perpendicular: {
                    LineEntity* l1 = sketch.findLine(c.entityA);
                    LineEntity* l2 = sketch.findLine(c.entityB);
                    if (!l1 || !l2) break;
                    PointEntity* a1 = sketch.findPoint(l1->startPt);
                    PointEntity* b1 = sketch.findPoint(l1->endPt);
                    PointEntity* a2 = sketch.findPoint(l2->startPt);
                    PointEntity* b2 = sketch.findPoint(l2->endPt);
                    if (!a1 || !b1 || !a2 || !b2) break;

                    double dx1 = b1->x-a1->x, dy1 = b1->y-a1->y;
                    double dx2 = b2->x-a2->x, dy2 = b2->y-a2->y;
                    double len1 = std::sqrt(dx1*dx1 + dy1*dy1);
                    double len2 = std::sqrt(dx2*dx2 + dy2*dy2);
                    if (len1 < 1e-6 || len2 < 1e-6) break;

                    double dot = (dx1*dx2 + dy1*dy2) / (len1*len2);
                    if (std::fabs(dot) < 1e-6) break; // already perpendicular

                    // Total rotation needed to make lines perpendicular
                    double cross = dx1*dy2 - dy1*dx2;
                    double curAngle = std::atan2(cross, dx1*dx2 + dy1*dy2);
                    double targetAngle = (curAngle >= 0) ? kPiD * 0.5 : -kPiD * 0.5;
                    double totalRot = targetAngle - curAngle;
                    while (totalRot > kPiD) totalRot -= kTwoPiD;
                    while (totalRot < -kPiD) totalRot += kTwoPiD;
                    if (std::fabs(totalRot) < 1e-6) break;

                    // Weight: more-constrained line rotates less
                    double w1 = weightLL(l1, l2);
                    if (w1 == kBothLocked) break;
                    // line2 rotates by totalRot * w1, line1 rotates by -totalRot * w2
                    double rot2 = totalRot * w1;
                    double rot1 = -totalRot * (1.0 - w1);

                    auto dist2 = [](double ax,double ay,double bx,double by){ double ddx=ax-bx,ddy=ay-by; return ddx*ddx+ddy*ddy; };
                    // Shared vertex by identity, so a corner that has not yet converged to within
                    // a positional epsilon is still recognised as a corner and rotated about
                    // rather than torn apart.
                    double cx = 0, cy = 0;
                    bool shared = false;
                    if (EntityID svID = sharedVertexOf(l1, l2)) {
                        if (const PointEntity* sv = sketch.findPoint(svID)) {
                            cx = sv->x; cy = sv->y; shared = true;
                        }
                    }

                    auto rotatePt = [&](PointEntity* pt, double ox, double oy, double cosR, double sinR) {
                        double rx = pt->x-ox, ry = pt->y-oy;
                        moveTo(pt, ox + rx*cosR - ry*sinR, oy + rx*sinR + ry*cosR);
                    };

                    if (shared) {
                        // Rotate far endpoints around shared vertex
                        PointEntity* far1 = (dist2(cx,cy,b1->x,b1->y)>=dist2(cx,cy,a1->x,a1->y)) ? b1 : a1;
                        PointEntity* far2 = (dist2(cx,cy,b2->x,b2->y)>=dist2(cx,cy,a2->x,a2->y)) ? b2 : a2;
                        double cos1 = std::cos(rot1), sin1 = std::sin(rot1);
                        double cos2 = std::cos(rot2), sin2 = std::sin(rot2);
                        rotatePt(far1, cx, cy, cos1, sin1);
                        rotatePt(far2, cx, cy, cos2, sin2);
                    } else {
                        // Rotate each line around its midpoint
                        double cos1 = std::cos(rot1), sin1 = std::sin(rot1);
                        double cos2 = std::cos(rot2), sin2 = std::sin(rot2);
                        double mx1 = (a1->x+b1->x)*0.5, my1 = (a1->y+b1->y)*0.5;
                        rotatePt(a1, mx1, my1, cos1, sin1);
                        rotatePt(b1, mx1, my1, cos1, sin1);
                        double mx2 = (a2->x+b2->x)*0.5, my2 = (a2->y+b2->y)*0.5;
                        rotatePt(a2, mx2, my2, cos2, sin2);
                        rotatePt(b2, mx2, my2, cos2, sin2);
                    }
                    changed = true;
                    break;
                }

                case ConstraintType::Parallel: {
                    LineEntity* l1 = sketch.findLine(c.entityA);
                    LineEntity* l2 = sketch.findLine(c.entityB);
                    if (!l1 || !l2) break;
                    PointEntity* a1 = sketch.findPoint(l1->startPt);
                    PointEntity* b1 = sketch.findPoint(l1->endPt);
                    PointEntity* a2 = sketch.findPoint(l2->startPt);
                    PointEntity* b2 = sketch.findPoint(l2->endPt);
                    if (!a1 || !b1 || !a2 || !b2) break;

                    double dx1 = b1->x-a1->x, dy1 = b1->y-a1->y;
                    double dx2 = b2->x-a2->x, dy2 = b2->y-a2->y;
                    double len1 = std::sqrt(dx1*dx1 + dy1*dy1);
                    double len2 = std::sqrt(dx2*dx2 + dy2*dy2);
                    if (len1 < 1e-6 || len2 < 1e-6) break;

                    double cross = dx1*dy2 - dy1*dx2;
                    if (std::fabs(cross) < 1e-6 * len1 * len2) break; // already parallel

                    double dot = dx1*dx2 + dy1*dy2;
                    double totalAngle = std::atan2(cross, dot); // angle from parallel

                    // Weight: more-constrained line rotates less
                    double w1 = weightLL(l1, l2);
                    if (w1 == kBothLocked) break;
                    double rot2 = -totalAngle * w1;         // line2 rotates toward line1
                    double rot1 = totalAngle * (1.0 - w1); // line1 rotates toward line2

                    auto rotatePt = [&](PointEntity* p, double ox, double oy, double cosR, double sinR) {
                        double rx = p->x-ox, ry = p->y-oy;
                        moveTo(p, ox + rx*cosR - ry*sinR, oy + rx*sinR + ry*cosR);
                    };

                    double cos1 = std::cos(rot1), sin1 = std::sin(rot1);
                    double cos2 = std::cos(rot2), sin2 = std::sin(rot2);

                    // If the two lines meet at a shared vertex, rotate about it. Rotating each
                    // line about its own midpoint (the only thing this did before) pulls the
                    // shared corner apart every pass, which the Coincident constraint then drags
                    // back -- the two constraints fight, and the sketch visibly thrashes.
                    // Perpendicular has always handled this; Parallel did not.
                    EntityID svID = sharedVertexOf(l1, l2);
                    const PointEntity* sv = (svID != NullID) ? sketch.findPoint(svID) : nullptr;
                    if (sv) {
                        auto dist2 = [](double ax,double ay,double bx,double by){ double ddx=ax-bx,ddy=ay-by; return ddx*ddx+ddy*ddy; };
                        double vx = sv->x, vy = sv->y;
                        PointEntity* far1 = (dist2(vx,vy,b1->x,b1->y) >= dist2(vx,vy,a1->x,a1->y)) ? b1 : a1;
                        PointEntity* far2 = (dist2(vx,vy,b2->x,b2->y) >= dist2(vx,vy,a2->x,a2->y)) ? b2 : a2;
                        rotatePt(far1, vx, vy, cos1, sin1);
                        rotatePt(far2, vx, vy, cos2, sin2);
                    } else {
                        double mx1 = (a1->x+b1->x)*0.5, my1 = (a1->y+b1->y)*0.5;
                        rotatePt(a1, mx1, my1, cos1, sin1);
                        rotatePt(b1, mx1, my1, cos1, sin1);
                        double mx2 = (a2->x+b2->x)*0.5, my2 = (a2->y+b2->y)*0.5;
                        rotatePt(a2, mx2, my2, cos2, sin2);
                        rotatePt(b2, mx2, my2, cos2, sin2);
                    }
                    changed = true;
                    break;
                }

                case ConstraintType::Collinear: {
                    LineEntity* l1 = sketch.findLine(c.entityA);
                    LineEntity* l2 = sketch.findLine(c.entityB);
                    if (!l1 || !l2) break;
                    PointEntity* a1 = sketch.findPoint(l1->startPt);
                    PointEntity* b1 = sketch.findPoint(l1->endPt);
                    PointEntity* a2 = sketch.findPoint(l2->startPt);
                    PointEntity* b2 = sketch.findPoint(l2->endPt);
                    if (!a1 || !b1 || !a2 || !b2) break;

                    double dx1 = b1->x-a1->x, dy1 = b1->y-a1->y;
                    double dx2 = b2->x-a2->x, dy2 = b2->y-a2->y;
                    double len1 = std::sqrt(dx1*dx1 + dy1*dy1);
                    double len2 = std::sqrt(dx2*dx2 + dy2*dy2);
                    if (len1 < 1e-6 || len2 < 1e-6) break;

                    double w1 = weightLL(l1, l2);
                    if (w1 == kBothLocked) break;

                    // Step 1: make directions parallel (same as Parallel constraint)
                    double cross = dx1*dy2 - dy1*dx2;
                    if (std::fabs(cross) >= 1e-9) {
                        double dot = dx1*dx2 + dy1*dy2;
                        double totalAngle = std::atan2(cross, dot);
                        double rot2 = -totalAngle * w1;
                        double rot1 =  totalAngle * (1.0 - w1);
                        auto rotatePt = [&](PointEntity* p, double ox, double oy, double cosR, double sinR) {
                            double rx = p->x-ox, ry = p->y-oy;
                            moveTo(p, ox + rx*cosR - ry*sinR, oy + rx*sinR + ry*cosR);
                        };
                        double cos1 = std::cos(rot1), sin1 = std::sin(rot1);
                        double cos2 = std::cos(rot2), sin2 = std::sin(rot2);
                        // Rotate about a shared vertex when there is one, as Parallel does.
                        EntityID svID = sharedVertexOf(l1, l2);
                        const PointEntity* sv = (svID != NullID) ? sketch.findPoint(svID) : nullptr;
                        if (sv) {
                            auto d2 = [](double ax,double ay,double bx,double by){ double ex=ax-bx,ey=ay-by; return ex*ex+ey*ey; };
                            double vx = sv->x, vy = sv->y;
                            rotatePt((d2(vx,vy,b1->x,b1->y) >= d2(vx,vy,a1->x,a1->y)) ? b1 : a1, vx, vy, cos1, sin1);
                            rotatePt((d2(vx,vy,b2->x,b2->y) >= d2(vx,vy,a2->x,a2->y)) ? b2 : a2, vx, vy, cos2, sin2);
                        } else {
                            double mx1 = (a1->x+b1->x)*0.5, my1 = (a1->y+b1->y)*0.5;
                            rotatePt(a1, mx1, my1, cos1, sin1);
                            rotatePt(b1, mx1, my1, cos1, sin1);
                            double mx2 = (a2->x+b2->x)*0.5, my2 = (a2->y+b2->y)*0.5;
                            rotatePt(a2, mx2, my2, cos2, sin2);
                            rotatePt(b2, mx2, my2, cos2, sin2);
                        }
                        // Refresh direction after rotation
                        dx1 = b1->x-a1->x; dy1 = b1->y-a1->y;
                        len1 = std::sqrt(dx1*dx1 + dy1*dy1);
                        changed = true;
                    }

                    // Step 2: translate to make lines colinear.
                    // Perpendicular distance from a2 to the infinite line through a1,b1:
                    // dist = ((a2-a1) × d1) / len1  where d1=(dx1,dy1), × is 2D cross product
                    if (len1 > 1e-9) {
                        double dist = ((a2->x - a1->x)*dy1 - (a2->y - a1->y)*dx1) / len1;
                        if (std::fabs(dist) > 1e-9) {
                            double nx = -dy1 / len1, ny = dx1 / len1; // unit normal to l1
                            // Distribute the correction: l2 moves by -dist*w1, l1 by +dist*(1-w1)
                            double move2 = -dist * w1;
                            double move1 =  dist * (1.0 - w1);
                            moveBy(a2, nx * move2, ny * move2);
                            moveBy(b2, nx * move2, ny * move2);
                            moveBy(a1, nx * move1, ny * move1);
                            moveBy(b1, nx * move1, ny * move1);
                            changed = true;
                        }
                    }
                    break;
                }

                case ConstraintType::Tangent: {
                    // A=line, B=circle or arc
                    LineEntity* line = sketch.findLine(c.entityA);
                    if (!line) break;
                    PointEntity* la = sketch.findPoint(line->startPt);
                    PointEntity* lb = sketch.findPoint(line->endPt);
                    if (!la || !lb) break;

                    double cx, cy, radius;
                    CircleEntity* circle = sketch.findCircle(c.entityB);
                    ArcEntity* arc = sketch.findArc(c.entityB);
                    PointEntity* curveCenterPt = nullptr;
                    if (circle) {
                        curveCenterPt = sketch.findPoint(circle->centerPt);
                        if (!curveCenterPt) break;
                        cx = curveCenterPt->x; cy = curveCenterPt->y; radius = circle->radius;
                    } else if (arc) {
                        curveCenterPt = sketch.findPoint(arc->centerPt);
                        PointEntity* sp = sketch.findPoint(arc->startPt);
                        if (!curveCenterPt || !sp) break;
                        cx = curveCenterPt->x; cy = curveCenterPt->y;
                        double dx = sp->x - curveCenterPt->x, dy = sp->y - curveCenterPt->y;
                        radius = std::sqrt(dx*dx + dy*dy);
                    } else break;

                    // Contact point. For a fillet the contact is the endpoint the line and the
                    // arc genuinely share, so resolve it by identity first; only fall back to
                    // "whichever endpoint happens to sit nearest the curve" when there is no
                    // shared vertex, since proximity alone can pick the wrong end and flip the
                    // line around.
                    PointEntity* contact = nullptr;
                    bool isFillet = false;
                    if (arc) {
                        if (sameVertex(line->startPt, arc->startPt) || sameVertex(line->startPt, arc->endPt)) {
                            contact = la; isFillet = true;
                        } else if (sameVertex(line->endPt, arc->startPt) || sameVertex(line->endPt, arc->endPt)) {
                            contact = lb; isFillet = true;
                        }
                    }
                    if (!contact) {
                        double distA = std::fabs(distance({la->x, la->y}, {cx, cy}) - radius);
                        double distB = std::fabs(distance({lb->x, lb->y}, {cx, cy}) - radius);
                        contact = (distA < distB) ? la : lb;
                    }
                    PointEntity* far = (contact == la) ? lb : la;
                    (void)far;

                    if (isFillet && curveCenterPt) {
                        // Fillet case: move the arc CENTER to maintain tangency, not the trim point.
                        // The trim point is owned by the adjacent line and likely has Distance
                        // constraints; moving it here would conflict with Distance, causing oscillation.
                        // Instead, project the center's component along the line direction onto the
                        // perpendicular through the contact point: (center - contact) · lineDir = 0.
                        double fullDx = lb->x - la->x, fullDy = lb->y - la->y;
                        double fullLen = std::sqrt(fullDx*fullDx + fullDy*fullDy);
                        if (fullLen < 1e-9) break;
                        double unitDx = fullDx / fullLen, unitDy = fullDy / fullLen;
                        double err = (cx - contact->x) * unitDx + (cy - contact->y) * unitDy;
                        if (std::fabs(err) > 1e-6) {
                            moveBy(curveCenterPt, -err * unitDx, -err * unitDy);
                            sketch.recomputeArcAngles(*arc);
                            changed = true;
                        }
                        break;
                    }

                    // Non-fillet: constrain the line's INFINITE extension to be tangent, i.e. the
                    // perpendicular distance from the curve centre to the line equals the radius.
                    // This removes exactly the 1 DOF that Tangent is defined (and DOF-counted) to
                    // remove. The previous implementation snapped a line endpoint onto the curve
                    // and then rotated the far endpoint about it, which removes 3 DOF and
                    // teleports a distant line onto the circle instead of just rotating it --
                    // Fusion does not require a tangent line to touch at an endpoint at all.
                    double ldx = lb->x - la->x, ldy = lb->y - la->y;
                    double lineLen = std::sqrt(ldx*ldx + ldy*ldy);
                    if (lineLen < 1e-9) break;
                    double nx = -ldy / lineLen, ny = ldx / lineLen;      // unit normal
                    double signedDist = (cx - la->x) * nx + (cy - la->y) * ny;
                    // Keep whichever side the curve is currently on; flipping it would jump the
                    // line across the circle.
                    double targetDist = (signedDist >= 0.0) ? radius : -radius;
                    double tErr = signedDist - targetDist;
                    if (std::fabs(tErr) < 1e-9) break;

                    // Split the correction between translating the line and translating the
                    // centre. For an arc the "radius" is derived from its start point, so moving
                    // the centre would change the arc's own shape -- move the line instead
                    // whenever it is free to move.
                    bool lineIsLocked = lineLocked(line);
                    bool cenIsLocked  = locked(curveCenterPt->id);
                    if (lineIsLocked && cenIsLocked) break;
                    double wLine;                       // share of the fix taken by the line
                    if (cenIsLocked)      wLine = 1.0;
                    else if (lineIsLocked) wLine = 0.0;
                    else if (arc)          wLine = 1.0;
                    else                   wLine = blendWeight(countRefs(curveCenterPt->id),
                                                               countLineRefs(line));
                    // Moving the line by +n*s decreases signedDist by s; moving the centre by
                    // +n*t increases it by t. Together they must remove exactly tErr.
                    double lineShift = tErr * wLine;
                    double cenShift  = -tErr * (1.0 - wLine);
                    moveBy(la, nx * lineShift, ny * lineShift);
                    moveBy(lb, nx * lineShift, ny * lineShift);
                    moveBy(curveCenterPt, nx * cenShift, ny * cenShift);
                    if (arc) sketch.recomputeArcAngles(*arc);
                    changed = true;
                    break;
                }

                case ConstraintType::Midpoint: {
                    PointEntity* pt = sketch.findPoint(c.entityA);
                    LineEntity* line = sketch.findLine(c.entityB);
                    if (!pt || !line) break;
                    PointEntity* la = sketch.findPoint(line->startPt);
                    PointEntity* lb = sketch.findPoint(line->endPt);
                    if (!la || !lb) break;

                    double mx = (la->x + lb->x) * 0.5;
                    double my = (la->y + lb->y) * 0.5;
                    double errX = pt->x - mx, errY = pt->y - my;
                    if (std::fabs(errX) > 1e-6 || std::fabs(errY) > 1e-6) {
                        // Weight: point vs line endpoints
                        double wPt = weightPL(c.entityA, line);
                        if (wPt == kBothLocked) break;
                        double lineFrac = 1.0 - wPt; // how much point moves
                        double ptFrac   = wPt;       // how much line moves

                        // Move point toward midpoint
                        moveBy(pt, -errX * lineFrac, -errY * lineFrac);
                        // Shift line endpoints so midpoint moves toward point
                        moveBy(la, errX * ptFrac, errY * ptFrac);
                        moveBy(lb, errX * ptFrac, errY * ptFrac);
                        changed = true;
                    }
                    break;
                }

                case ConstraintType::Symmetric: {
                    // A=pt1, B=pt2, C=axis line
                    PointEntity* p1 = sketch.findPoint(c.entityA);
                    PointEntity* p2 = sketch.findPoint(c.entityB);
                    LineEntity* axis = sketch.findLine(c.entityC);
                    if (!p1 || !p2 || !axis) break;
                    PointEntity* aa = sketch.findPoint(axis->startPt);
                    PointEntity* ab = sketch.findPoint(axis->endPt);
                    if (!aa || !ab) break;

                    // Reflect p1 across axis line
                    double adx = ab->x-aa->x, ady = ab->y-aa->y;
                    double alen2 = adx*adx + ady*ady;
                    if (alen2 < 1e-12) break;

                    // Reflection of p1: r = 2*proj(p1-aa, axis) + aa - (p1-aa) + aa
                    double px = p1->x-aa->x, py = p1->y-aa->y;
                    double t = (px*adx + py*ady) / alen2;
                    double reflX = 2*(aa->x + t*adx) - p1->x;
                    double reflY = 2*(aa->y + t*ady) - p1->y;

                    // Also reflect p2 across axis
                    double px2 = p2->x-aa->x, py2 = p2->y-aa->y;
                    double t2 = (px2*adx + py2*ady) / alen2;
                    double refl2X = 2*(aa->x + t2*adx) - p2->x;
                    double refl2Y = 2*(aa->y + t2*ady) - p2->y;

                    // Target: midpoint of both reflections (symmetric convergence)
                    double targ1X = (p1->x + refl2X) * 0.5;
                    double targ1Y = (p1->y + refl2Y) * 0.5;
                    double targ2X = (p2->x + reflX) * 0.5;
                    double targ2Y = (p2->y + reflY) * 0.5;

                    if (std::fabs(p1->x-targ1X) > 1e-6 || std::fabs(p1->y-targ1Y) > 1e-6 ||
                        std::fabs(p2->x-targ2X) > 1e-6 || std::fabs(p2->y-targ2Y) > 1e-6) {
                        // moveTo refuses locked points, so a symmetric pair with one projected or
                        // dragged side reflects the free side about the axis instead of dragging
                        // the fixed one halfway.
                        moveTo(p1, targ1X, targ1Y);
                        moveTo(p2, targ2X, targ2Y);
                        changed = true;
                    }
                    break;
                }

                case ConstraintType::Concentric: {
                    // Find center points of both entities (circles or arcs)
                    EntityID cp1ID = NullID, cp2ID = NullID;
                    CircleEntity* c1 = sketch.findCircle(c.entityA);
                    ArcEntity* a1 = sketch.findArc(c.entityA);
                    if (c1) cp1ID = c1->centerPt;
                    else if (a1) cp1ID = a1->centerPt;

                    CircleEntity* c2 = sketch.findCircle(c.entityB);
                    ArcEntity* a2 = sketch.findArc(c.entityB);
                    if (c2) cp2ID = c2->centerPt;
                    else if (a2) cp2ID = a2->centerPt;

                    if (cp1ID == NullID || cp2ID == NullID) break;
                    PointEntity* p1 = sketch.findPoint(cp1ID);
                    PointEntity* p2 = sketch.findPoint(cp2ID);
                    if (!p1 || !p2) break;

                    if (std::fabs(p1->x - p2->x) > 1e-6 || std::fabs(p1->y - p2->y) > 1e-6) {
                        double w1 = weightPP(cp1ID, cp2ID);
                        if (w1 == kBothLocked) break;
                        double tX = p1->x * w1 + p2->x * (1.0 - w1);
                        double tY = p1->y * w1 + p2->y * (1.0 - w1);
                        moveTo(p1, tX, tY);
                        moveTo(p2, tX, tY);
                        changed = true;
                    }
                    break;
                }

                default:
                    break;
            }
        }

        // Fillet arcs: put each endpoint at the foot of the perpendicular from the centre onto
        // its adjacent line, which is exactly what "tangent trim point" means. The foot lies on
        // the line, so the line keeps its direction through its other endpoint and simply gets
        // trimmed. Doing this explicitly keeps |start-centre| == |end-centre| -- the invariant
        // every consumer relies on when it derives an arc's radius from the start point alone --
        // without the generic equal-radius pass below fighting the Tangent constraints over
        // where the centre belongs.
        for (auto& arc : sketch.arcs) {
            auto fit = filletArcs.find(arc.id);
            if (fit == filletArcs.end()) continue;
            PointEntity* cp = sketch.findPoint(arc.centerPt);
            if (!cp) continue;
            auto footOnto = [&](EntityID lineID, EntityID ptID) {
                const LineEntity* ln = sketch.findLine(lineID);
                PointEntity* pe = sketch.findPoint(ptID);
                if (!ln || !pe) return;
                const PointEntity* la = sketch.findPoint(ln->startPt);
                const PointEntity* lb = sketch.findPoint(ln->endPt);
                if (!la || !lb) return;
                double dx = lb->x - la->x, dy = lb->y - la->y;
                double L2 = dx*dx + dy*dy;
                if (L2 < 1e-18) return;
                double t = ((cp->x - la->x)*dx + (cp->y - la->y)*dy) / L2;
                moveTo(pe, la->x + t*dx, la->y + t*dy);
            };
            const LineEntity* lA = sketch.findLine(fit->second.lineA);
            if (!lA) continue;
            bool aOwnsStart = sameVertex(lA->startPt, arc.startPt) || sameVertex(lA->endPt, arc.startPt);
            footOnto(aOwnsStart ? fit->second.lineA : fit->second.lineB, arc.startPt);
            footOnto(aOwnsStart ? fit->second.lineB : fit->second.lineA, arc.endPt);
            sketch.recomputeArcAngles(arc);
        }

        // Enforce arc geometry inside the loop: both endpoints equidistant from center
        for (auto& arc : sketch.arcs) {
            PointEntity* cp = sketch.findPoint(arc.centerPt);
            PointEntity* sp = sketch.findPoint(arc.startPt);
            PointEntity* ep = sketch.findPoint(arc.endPt);
            if (!cp || !sp || !ep) continue;

            // Fillets are handled by the tangent-foot pass above, which already guarantees equal
            // radii; running this as well would fight the Tangent constraints. Keyed on the real
            // fillet signature (Radius + two Tangents) rather than on endpoint reference counts,
            // which also matched any ordinary arc chained into a profile and silently dropped
            // that arc's equal-radius invariant.
            if (filletArcs.count(arc.id)) continue;

            double dsx = sp->x - cp->x, dsy = sp->y - cp->y;
            double dex = ep->x - cp->x, dey = ep->y - cp->y;
            double rStart = std::sqrt(dsx*dsx + dsy*dsy);
            double rEnd   = std::sqrt(dex*dex + dey*dey);

            if (rStart < 1e-7 || rEnd < 1e-7) continue;
            if (std::fabs(rStart - rEnd) < 1e-6) continue;

            double targetR;
            if (draggedPoint == arc.startPt)
                targetR = rStart;
            else if (draggedPoint == arc.endPt)
                targetR = rEnd;
            else
                targetR = (rStart + rEnd) * 0.5;

            // Choose whether to move center or endpoints based on which is less constrained.
            // Fillet arcs share their endpoints with lines (high ref count on endpoints),
            // so we move the center rather than pulling line endpoints off their angles.
            int refC = countRefs(arc.centerPt);
            int refS = countRefs(arc.startPt);
            int refE = countRefs(arc.endPt);
            bool moveCenter = (refC <= refS && refC <= refE);

            if (moveCenter) {
                // Move center along the perpendicular bisector of sp-ep to sit at targetR from both.
                double midX = (sp->x + ep->x) * 0.5, midY = (sp->y + ep->y) * 0.5;
                double halfSepX = (ep->x - sp->x) * 0.5, halfSepY = (ep->y - sp->y) * 0.5;
                double halfSep2 = halfSepX*halfSepX + halfSepY*halfSepY;
                double newH2 = targetR*targetR - halfSep2;
                if (newH2 < 0.0) continue; // endpoints too far apart for this radius
                double newH = std::sqrt(newH2);
                // Perpendicular to (ep-sp), same side as current center
                double perpX = -(ep->y - sp->y), perpY = ep->x - sp->x;
                double perpLen = std::sqrt(perpX*perpX + perpY*perpY);
                if (perpLen < 1e-9) continue;
                perpX /= perpLen; perpY /= perpLen;
                if ((cp->x - midX)*perpX + (cp->y - midY)*perpY < 0.0) { perpX = -perpX; perpY = -perpY; }
                double newCX = midX + perpX * newH;
                double newCY = midY + perpY * newH;
                if (std::fabs(cp->x - newCX) > 1e-9 || std::fabs(cp->y - newCY) > 1e-9) {
                    moveTo(cp, newCX, newCY);
                    changed = true;
                }
            } else {
                moveTo(sp, cp->x + dsx / rStart * targetR, cp->y + dsy / rStart * targetR);
                moveTo(ep, cp->x + dex / rEnd   * targetR, cp->y + dey / rEnd   * targetR);
                changed = true;
            }
        }

        result.iterations = iter + 1;
        // Convergence is judged on the size of the largest correction, not on "did anything
        // change at all": with under-relaxation the corrections never fall to exactly zero, and a
        // sketch oscillating below the reporting tolerance would otherwise spin to the iteration
        // cap every frame while still claiming to be fine.
        if (!changed || maxDelta < (double)kSolverConvergenceTol) {
            result.converged = true;
            break;
        }

        // Adaptive under-relaxation. Full Gauss-Seidel steps for as long as the residual keeps
        // shrinking, so well-behaved sketches converge exactly as fast as they did before. When a
        // pass makes things worse -- the signature of two constraints fighting over the same
        // points -- damp the step so they settle on a compromise instead of ping-ponging until
        // the iteration cap and leaving the sketch wherever iteration 40 happened to land.
        if (iter > 0) {
            if (maxDelta > prevMaxDelta) relax = std::max(kSolverMinRelax, relax * 0.5);
            else                         relax = std::min(1.0, relax * 1.1);
        }
        prevMaxDelta = maxDelta;
    }

    // Estimate degrees of freedom: entity free params minus constraint removals.
    {
        // Count vertex GROUPS, not raw points. Two points joined by a Coincident constraint are
        // one vertex worth 2 DOF, not two points worth 4 with a -2 correction; counting groups
        // makes the two spellings of a joined corner (a shared point ID, or two points plus a
        // Coincident) agree, and stops a chain of Coincidents through one vertex from
        // over-subtracting. A group containing any projected point is pinned and contributes 0.
        int rawDof = 0;
        {
            std::unordered_set<EntityID> pinnedGroups;
            for (const auto& pt : sketch.points)
                if (pt.projected) pinnedGroups.insert(ufFind(pt.id));
            std::unordered_set<EntityID> counted;
            for (const auto& pt : sketch.points) {
                EntityID root = ufFind(pt.id);
                if (pinnedGroups.count(root)) continue;
                if (counted.insert(root).second) rawDof += 2;
            }
        }
        // Each non-projected circle adds 1 extra DOF for radius
        for (const auto& ci : sketch.circles) {
            if (!ci.projected) rawDof += 1;
        }
        // Each non-projected arc internally enforces equal radius for start/end point (-1 DOF)
        for (const auto& ar : sketch.arcs) {
            if (!ar.projected) rawDof -= 1;
        }
        // Subtract DOF removed by each active constraint
        for (const auto& c : sketch.constraints) {
            if (c.driven) continue;
            switch (c.type) {
                // Coincident is deliberately absent: it is already accounted for by merging the
                // two points into one vertex group above, and subtracting here as well would
                // double-count it.
                case ConstraintType::Symmetric:
                case ConstraintType::Concentric:
                case ConstraintType::Midpoint:
                case ConstraintType::Collinear:
                    rawDof -= 2; break;
                case ConstraintType::Horizontal:
                case ConstraintType::Vertical:
                case ConstraintType::Distance:
                case ConstraintType::Radius:
                case ConstraintType::Diameter:
                case ConstraintType::PointDistance:
                case ConstraintType::PointOnLine:
                case ConstraintType::PointLineDistance:
                case ConstraintType::EqualLength:
                case ConstraintType::Perpendicular:
                case ConstraintType::Parallel:
                case ConstraintType::Tangent:
                case ConstraintType::Angle:
                case ConstraintType::PointOnCircle:
                    rawDof -= 1; break;
                default: break;
            }
        }
        result.dof = std::max(0, rawDof);
        // Report over-constraint instead of silently clamping it away. Note this is a parameter
        // count, not a rank analysis: it cannot tell a genuinely redundant constraint from an
        // independent one, so it under-reports DOF when redundant constraints are present.
        result.excessDof = (rawDof < 0) ? -rawDof : 0;
        result.overConstrained = (rawDof < 0);
    }

    // Check if all constraints are satisfied after solving
    for (const auto& c : sketch.constraints) {
        if (c.driven) continue;
        double err = 0;
        switch (c.type) {
            case ConstraintType::Distance: {
                LineEntity* line = sketch.findLine(c.entityA);
                if (!line) break;
                PointEntity* a = sketch.findPoint(line->startPt);
                PointEntity* b = sketch.findPoint(line->endPt);
                if (!a || !b) break;
                double dx = b->x - a->x, dy = b->y - a->y;
                err = std::fabs(std::sqrt(dx*dx + dy*dy) - c.value);
                break;
            }
            case ConstraintType::PointDistance: {
                PointEntity* a = sketch.findPoint(c.entityA);
                PointEntity* b = sketch.findPoint(c.entityB);
                if (!a || !b) break;
                double dx = b->x - a->x, dy = b->y - a->y;
                err = std::fabs(std::sqrt(dx*dx + dy*dy) - c.value);
                break;
            }
            case ConstraintType::Horizontal: {
                LineEntity* line = sketch.findLine(c.entityA);
                if (!line) break;
                PointEntity* a = sketch.findPoint(line->startPt);
                PointEntity* b = sketch.findPoint(line->endPt);
                if (a && b) err = std::fabs(a->y - b->y);
                break;
            }
            case ConstraintType::Vertical: {
                LineEntity* line = sketch.findLine(c.entityA);
                if (!line) break;
                PointEntity* a = sketch.findPoint(line->startPt);
                PointEntity* b = sketch.findPoint(line->endPt);
                if (a && b) err = std::fabs(a->x - b->x);
                break;
            }
            case ConstraintType::Coincident: {
                PointEntity* a = sketch.findPoint(c.entityA);
                PointEntity* b = sketch.findPoint(c.entityB);
                if (a && b) err = std::fabs(a->x - b->x) + std::fabs(a->y - b->y);
                break;
            }
            case ConstraintType::Radius: {
                CircleEntity* circle = sketch.findCircle(c.entityA);
                if (circle) {
                    err = std::fabs(circle->radius - c.value);
                } else {
                    ArcEntity* arc = sketch.findArc(c.entityA);
                    if (arc) {
                        Point2D cp = sketch.getPointPos(arc->centerPt);
                        Point2D sp = sketch.getPointPos(arc->startPt);
                        err = std::fabs(distance(cp, sp) - c.value);
                    }
                }
                break;
            }
            case ConstraintType::Diameter: {
                CircleEntity* circle = sketch.findCircle(c.entityA);
                if (circle) err = std::fabs(circle->radius - c.value * 0.5);
                break;
            }
            case ConstraintType::Angle: {
                LineEntity* line1 = sketch.findLine(c.entityA);
                LineEntity* line2 = sketch.findLine(c.entityB);
                if (!line1 || !line2) break;
                PointEntity* a1 = sketch.findPoint(line1->startPt);
                PointEntity* b1 = sketch.findPoint(line1->endPt);
                PointEntity* a2 = sketch.findPoint(line2->startPt);
                PointEntity* b2 = sketch.findPoint(line2->endPt);
                if (!a1 || !b1 || !a2 || !b2) break;
                // Find vertex (shared or intersection)
                double eps = 1e-3;
                auto pEq = [eps](PointEntity* p, PointEntity* q) {
                    return std::fabs(p->x - q->x) < eps && std::fabs(p->y - q->y) < eps;
                };
                double vx, vy;
                if (pEq(a1, a2) || pEq(a1, b2))      { vx = a1->x; vy = a1->y; }
                else if (pEq(b1, a2) || pEq(b1, b2)) { vx = b1->x; vy = b1->y; }
                else {
                    double ldx1 = b1->x-a1->x, ldy1 = b1->y-a1->y;
                    double ldx2 = b2->x-a2->x, ldy2 = b2->y-a2->y;
                    double denom = ldx1*ldy2 - ldy1*ldx2;
                    if (std::fabs(denom) < 1e-6) break;
                    double t = ((a2->x-a1->x)*ldy2 - (a2->y-a1->y)*ldx2) / denom;
                    vx = a1->x + t*ldx1; vy = a1->y + t*ldy1;
                }
                auto dist2v = [](double ax, double ay, double bx, double by) {
                    double dx = ax-bx, dy = ay-by; return dx*dx + dy*dy;
                };
                PointEntity* f1 = (dist2v(vx,vy,b1->x,b1->y)>=dist2v(vx,vy,a1->x,a1->y)) ? b1 : a1;
                PointEntity* f2 = (dist2v(vx,vy,b2->x,b2->y)>=dist2v(vx,vy,a2->x,a2->y)) ? b2 : a2;
                double dx1 = f1->x-vx, dy1 = f1->y-vy;
                double dx2 = f2->x-vx, dy2 = f2->y-vy;
                double dot = dx1*dx2 + dy1*dy2;
                double cross = dx1*dy2 - dy1*dx2;
                double ccwRad = std::atan2(cross, dot);
                if (ccwRad < 0) ccwRad += kTwoPiD;
                double ccwDeg = ccwRad * kRadToDegD;
                double cwDeg = 360.0 - ccwDeg;
                // Use the stored sector
                double currentDeg = c.angleCW ? cwDeg : ccwDeg;
                err = std::fabs(currentDeg - c.value);
                break;
            }
            case ConstraintType::PointLineDistance: {
                PointEntity* pt = sketch.findPoint(c.entityA);
                LineEntity* line = sketch.findLine(c.entityB);
                if (!pt || !line) break;
                PointEntity* la = sketch.findPoint(line->startPt);
                PointEntity* lb = sketch.findPoint(line->endPt);
                if (!la || !lb) break;
                double ldx = lb->x-la->x, ldy = lb->y-la->y;
                double lineLen = std::sqrt(ldx*ldx+ldy*ldy);
                if (lineLen < 1e-6) break;
                double cross = (pt->x-la->x)*ldy - (pt->y-la->y)*ldx;
                double dist = cross / lineLen;
                double sideSign = c.negativeSide ? -1.0 : 1.0;
                double targetSigned = sideSign * c.value;
                err = std::fabs(dist - targetSigned);
                break;
            }
            case ConstraintType::PointOnLine: {
                PointEntity* pt = sketch.findPoint(c.entityA);
                LineEntity* line = sketch.findLine(c.entityB);
                if (!pt || !line) break;
                PointEntity* la = sketch.findPoint(line->startPt);
                PointEntity* lb = sketch.findPoint(line->endPt);
                if (!la || !lb) break;
                double ldx = lb->x-la->x, ldy = lb->y-la->y;
                double lineLen = std::sqrt(ldx*ldx+ldy*ldy);
                if (lineLen < 1e-6) break;
                double cross = (pt->x-la->x)*ldy - (pt->y-la->y)*ldx;
                err = std::fabs(cross) / lineLen;
                break;
            }
            case ConstraintType::PointOnCircle: {
                PointEntity* pt = sketch.findPoint(c.entityA);
                if (!pt) break;
                EntityID centerPtID = NullID;
                double radius = 0.0;
                CircleEntity* circ = sketch.findCircle(c.entityB);
                if (circ) { centerPtID = circ->centerPt; radius = circ->radius; }
                else {
                    ArcEntity* arc = sketch.findArc(c.entityB);
                    if (arc) {
                        centerPtID = arc->centerPt;
                        PointEntity* sp = sketch.findPoint(arc->startPt);
                        PointEntity* cp = sketch.findPoint(arc->centerPt);
                        if (sp && cp) { double dx=sp->x-cp->x, dy=sp->y-cp->y; radius=std::sqrt(dx*dx+dy*dy); }
                    }
                }
                if (centerPtID == NullID) break;
                PointEntity* cen = sketch.findPoint(centerPtID);
                if (!cen) break;
                double dx = pt->x-cen->x, dy = pt->y-cen->y;
                err = std::fabs(std::sqrt(dx*dx+dy*dy) - radius);
                break;
            }
            case ConstraintType::EqualLength: {
                LineEntity* l1 = sketch.findLine(c.entityA);
                LineEntity* l2 = sketch.findLine(c.entityB);
                if (!l1 || !l2) break;
                PointEntity* a1 = sketch.findPoint(l1->startPt);
                PointEntity* b1 = sketch.findPoint(l1->endPt);
                PointEntity* a2 = sketch.findPoint(l2->startPt);
                PointEntity* b2 = sketch.findPoint(l2->endPt);
                if (!a1||!b1||!a2||!b2) break;
                double dx1=b1->x-a1->x, dy1=b1->y-a1->y;
                double dx2=b2->x-a2->x, dy2=b2->y-a2->y;
                err = std::fabs(std::sqrt(dx1*dx1+dy1*dy1) - std::sqrt(dx2*dx2+dy2*dy2));
                break;
            }
            case ConstraintType::Perpendicular: {
                LineEntity* l1 = sketch.findLine(c.entityA);
                LineEntity* l2 = sketch.findLine(c.entityB);
                if (!l1 || !l2) break;
                PointEntity* a1 = sketch.findPoint(l1->startPt);
                PointEntity* b1 = sketch.findPoint(l1->endPt);
                PointEntity* a2 = sketch.findPoint(l2->startPt);
                PointEntity* b2 = sketch.findPoint(l2->endPt);
                if (!a1||!b1||!a2||!b2) break;
                double dx1=b1->x-a1->x, dy1=b1->y-a1->y;
                double dx2=b2->x-a2->x, dy2=b2->y-a2->y;
                double len1=std::sqrt(dx1*dx1+dy1*dy1), len2=std::sqrt(dx2*dx2+dy2*dy2);
                if (len1>1e-6 && len2>1e-6)
                    err = std::fabs(dx1*dx2+dy1*dy2) / (len1*len2);
                break;
            }
            case ConstraintType::Parallel: {
                LineEntity* l1 = sketch.findLine(c.entityA);
                LineEntity* l2 = sketch.findLine(c.entityB);
                if (!l1 || !l2) break;
                PointEntity* a1 = sketch.findPoint(l1->startPt);
                PointEntity* b1 = sketch.findPoint(l1->endPt);
                PointEntity* a2 = sketch.findPoint(l2->startPt);
                PointEntity* b2 = sketch.findPoint(l2->endPt);
                if (!a1||!b1||!a2||!b2) break;
                double dx1=b1->x-a1->x, dy1=b1->y-a1->y;
                double dx2=b2->x-a2->x, dy2=b2->y-a2->y;
                double len1=std::sqrt(dx1*dx1+dy1*dy1), len2=std::sqrt(dx2*dx2+dy2*dy2);
                if (len1>1e-6 && len2>1e-6)
                    err = std::fabs(dx1*dy2-dy1*dx2) / (len1*len2);
                break;
            }
            case ConstraintType::Collinear: {
                LineEntity* l1 = sketch.findLine(c.entityA);
                LineEntity* l2 = sketch.findLine(c.entityB);
                if (!l1 || !l2) break;
                PointEntity* a1 = sketch.findPoint(l1->startPt);
                PointEntity* b1 = sketch.findPoint(l1->endPt);
                PointEntity* a2 = sketch.findPoint(l2->startPt);
                if (!a1||!b1||!a2) break;
                double dx1=b1->x-a1->x, dy1=b1->y-a1->y;
                double len1=std::sqrt(dx1*dx1+dy1*dy1);
                if (len1 > 1e-6) {
                    // Max of: angle error + distance error
                    LineEntity* l2b = sketch.findLine(c.entityB);
                    PointEntity* b2 = sketch.findPoint(l2b->endPt);
                    double dx2=b2->x-a2->x, dy2=b2->y-a2->y;
                    double len2=std::sqrt(dx2*dx2+dy2*dy2);
                    double angleErr = len2>1e-6 ? std::fabs(dx1*dy2-dy1*dx2)/(len1*len2) : 0.0;
                    double distErr  = std::fabs((a2->x-a1->x)*dy1 - (a2->y-a1->y)*dx1) / len1;
                    err = angleErr + distErr;
                }
                break;
            }
            case ConstraintType::Tangent: {
                LineEntity* line = sketch.findLine(c.entityA);
                if (!line) break;
                PointEntity* la = sketch.findPoint(line->startPt);
                PointEntity* lb = sketch.findPoint(line->endPt);
                if (!la || !lb) break;
                double cx, cy, radius;
                CircleEntity* circ = sketch.findCircle(c.entityB);
                ArcEntity* arc = sketch.findArc(c.entityB);
                if (circ) {
                    PointEntity* cp = sketch.findPoint(circ->centerPt);
                    if (!cp) break;
                    cx=cp->x; cy=cp->y; radius=circ->radius;
                } else if (arc) {
                    PointEntity* cp = sketch.findPoint(arc->centerPt);
                    PointEntity* sp = sketch.findPoint(arc->startPt);
                    if (!cp || !sp) break;
                    cx=cp->x; cy=cp->y;
                    double dx=sp->x-cp->x, dy=sp->y-cp->y;
                    radius=std::sqrt(dx*dx+dy*dy);
                } else break;
                double ldx=lb->x-la->x, ldy=lb->y-la->y;
                double lineLen=std::sqrt(ldx*ldx+ldy*ldy);
                if (lineLen<1e-6) break;
                double cross=(cx-la->x)*ldy-(cy-la->y)*ldx;
                err = std::fabs(std::fabs(cross)/lineLen - radius);
                break;
            }
            case ConstraintType::Midpoint: {
                PointEntity* pt = sketch.findPoint(c.entityA);
                LineEntity* line = sketch.findLine(c.entityB);
                if (!pt || !line) break;
                PointEntity* la = sketch.findPoint(line->startPt);
                PointEntity* lb = sketch.findPoint(line->endPt);
                if (!la || !lb) break;
                double mx=(la->x+lb->x)*0.5, my=(la->y+lb->y)*0.5;
                err = std::fabs(pt->x-mx) + std::fabs(pt->y-my);
                break;
            }
            case ConstraintType::Symmetric: {
                PointEntity* p1 = sketch.findPoint(c.entityA);
                PointEntity* p2 = sketch.findPoint(c.entityB);
                LineEntity* axis = sketch.findLine(c.entityC);
                if (!p1 || !p2 || !axis) break;
                PointEntity* aa = sketch.findPoint(axis->startPt);
                PointEntity* ab = sketch.findPoint(axis->endPt);
                if (!aa || !ab) break;
                double adx=ab->x-aa->x, ady=ab->y-aa->y;
                double alen2=adx*adx+ady*ady;
                if (alen2<1e-12) break;
                double px=p1->x-aa->x, py=p1->y-aa->y;
                double t=(px*adx+py*ady)/alen2;
                double reflX=2*(aa->x+t*adx)-p1->x;
                double reflY=2*(aa->y+t*ady)-p1->y;
                err = std::fabs(p2->x-reflX) + std::fabs(p2->y-reflY);
                break;
            }
            case ConstraintType::Concentric: {
                EntityID cp1ID=NullID, cp2ID=NullID;
                CircleEntity* c1=sketch.findCircle(c.entityA);
                ArcEntity* a1_=sketch.findArc(c.entityA);
                if (c1) cp1ID=c1->centerPt; else if (a1_) cp1ID=a1_->centerPt;
                CircleEntity* c2=sketch.findCircle(c.entityB);
                ArcEntity* a2_=sketch.findArc(c.entityB);
                if (c2) cp2ID=c2->centerPt; else if (a2_) cp2ID=a2_->centerPt;
                if (cp1ID!=NullID && cp2ID!=NullID) {
                    PointEntity* p1=sketch.findPoint(cp1ID);
                    PointEntity* p2=sketch.findPoint(cp2ID);
                    if (p1 && p2) err = std::fabs(p1->x-p2->x)+std::fabs(p1->y-p2->y);
                }
                break;
            }
            default: break;
        }
        result.totalError += f(err);
        // Use type-appropriate tolerance: degrees for angles, mm for distances
        float tol = (c.type == ConstraintType::Angle) ? 0.5f : 0.01f;
        if (err > tol) {
            result.ok = false;
        }
    }

    // Recompute arc angles from point positions after solving
    for (auto& arc : sketch.arcs) {
        sketch.recomputeArcAngles(arc);
    }

    sketch.dirty = false;
    return result;
}

SolveResult Solver::solve(Sketch& sketch, EntityID draggedPoint) {
    SolveResult r = solvePass(sketch, draggedPoint);
    // The dragged point is pinned to the cursor as a hard constraint so that it tracks the
    // mouse exactly instead of being averaged back off it by the constraints around it. When
    // that pin makes the sketch unsatisfiable, redo the pass with the point free: it is then
    // pulled back onto the constraint manifold and slides along it, which is how a constrained
    // drag is supposed to feel -- the point follows as closely as the constraints allow rather
    // than either escaping them or refusing to move.
    if (draggedPoint != NullID && !r.ok) {
        return solvePass(sketch, NullID);
    }
    return r;
}

} // namespace shitcad
