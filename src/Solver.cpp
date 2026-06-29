#include "Solver.h"
#include "Constants.h"
#include <cmath>
#include <algorithm>
#include <unordered_set>

namespace shitcad {

SolveResult Solver::solve(Sketch& sketch, EntityID draggedPoint) {
    SolveResult result;
    result.ok = true;
    result.dof = 0;
    result.converged = false;
    result.totalError = 0.0f;
    result.iterations = 0;

    // Simple iterative constraint solver.
    // Handles: Horizontal, Vertical, Coincident, Distance.
    // Run multiple iterations to converge (constraints can chain).

    // Count how constrained a point is: entity connections + constraints referencing it.
    // Points with more connections should be moved less by constraints.
    auto countRefs = [&](EntityID ptID) -> int {
        // Projected points are locked in place — treat as immovable
        const PointEntity* pt = sketch.findPoint(ptID);
        if (pt && pt->projected) return kProjectedPointRefCount;
        int n = 0;
        for (const auto& l : sketch.lines) {
            if (!l.projected && (l.startPt == ptID || l.endPt == ptID)) n++;
        }
        for (const auto& ci : sketch.circles) {
            if (!ci.projected && ci.centerPt == ptID) n++;
        }
        for (const auto& ar : sketch.arcs) {
            if (!ar.projected && (ar.centerPt == ptID || ar.startPt == ptID || ar.endPt == ptID)) n++;
        }
        for (const auto& el : sketch.ellipses) {
            if (!el.projected && el.centerPt == ptID) n++;
        }
        for (const auto& ea : sketch.ellipseArcs) {
            if (!ea.projected && (ea.centerPt == ptID || ea.startPt == ptID || ea.endPt == ptID)) n++;
        }
        for (const auto& sp : sketch.splines) {
            if (sp.projected) continue;
            for (auto cpID : sp.controlPtIDs)
                if (cpID == ptID) { n++; break; }
        }
        // Also count constraints that directly reference this point
        for (const auto& c : sketch.constraints) {
            if (c.entityA == ptID || c.entityB == ptID) n++;
        }
        return n;
    };

    // Count how constrained a line is (sum of its endpoint refs).
    auto countLineRefs = [&](const LineEntity* line) -> int {
        return countRefs(line->startPt) + countRefs(line->endPt);
    };

    sketch.rebuildIndices();

    for (int iter = 0; iter < kSolverMaxIterations; iter++) {
        bool changed = false;

        for (const auto& c : sketch.constraints) {
            if (c.driven) continue; // driven dimensions are reference-only
            switch (c.type) {
                case ConstraintType::Horizontal: {
                    LineEntity* line = sketch.findLine(c.entityA);
                    if (!line) break;
                    PointEntity* a = sketch.findPoint(line->startPt);
                    PointEntity* b = sketch.findPoint(line->endPt);
                    if (!a || !b) break;

                    if (std::fabs(a->y - b->y) > 1e-6f) {
                        int ra = countRefs(line->startPt);
                        int rb = countRefs(line->endPt);
                        double totalR = (double)(ra + rb);
                        if (totalR < 1.0) totalR = 1.0;
                        // Weight: point with more refs moves less
                        double wA = (double)ra / totalR; // fraction of correction applied to b
                        double wB = (double)rb / totalR; // fraction of correction applied to a
                        double avg = a->y * wA + b->y * wB;
                        a->y = avg;
                        b->y = avg;
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

                    if (std::fabs(a->x - b->x) > 1e-6f) {
                        int ra = countRefs(line->startPt);
                        int rb = countRefs(line->endPt);
                        double totalR = (double)(ra + rb);
                        if (totalR < 1.0) totalR = 1.0;
                        double wA = (double)ra / totalR;
                        double wB = (double)rb / totalR;
                        double avg = a->x * wA + b->x * wB;
                        a->x = avg;
                        b->x = avg;
                        changed = true;
                    }
                    break;
                }

                case ConstraintType::Coincident: {
                    PointEntity* a = sketch.findPoint(c.entityA);
                    PointEntity* b = sketch.findPoint(c.entityB);
                    if (!a || !b) break;

                    if (std::fabs(a->x - b->x) > 1e-6f || std::fabs(a->y - b->y) > 1e-6f) {
                        int ra = countRefs(c.entityA);
                        int rb = countRefs(c.entityB);
                        double totalR = (double)(ra + rb);
                        if (totalR < 1.0) totalR = 1.0;
                        double wA = (double)ra / totalR; // a's weight — more refs = moves less
                        double wB = (double)rb / totalR;
                        double tX = a->x * wA + b->x * wB;
                        double tY = a->y * wA + b->y * wB;
                        a->x = tX; a->y = tY;
                        b->x = tX; b->y = tY;
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
                    int ra = countRefs(line->startPt);
                    int rb = countRefs(line->endPt);
                    double totalR = (double)(ra + rb);
                    if (totalR < 1.0) totalR = 1.0;
                    double wA = (double)ra / totalR; // a's fraction — high = a moves less
                    // Pivot biased toward more-constrained end
                    double pivotX = a->x * wA + b->x * (1.0 - wA);
                    double pivotY = a->y * wA + b->y * (1.0 - wA);
                    double ux = dx / currentLen, uy = dy / currentLen;
                    // Place endpoints at target distance from pivot, preserving direction
                    double distA = currentLen * (1.0 - wA); // a's distance from pivot
                    double distB = currentLen * wA;          // b's distance from pivot
                    double scale = targetLen / currentLen;
                    a->x = pivotX - ux * distA * scale;
                    a->y = pivotY - uy * distA * scale;
                    b->x = pivotX + ux * distB * scale;
                    b->y = pivotY + uy * distB * scale;
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

                    int ra = countRefs(c.entityA);
                    int rb = countRefs(c.entityB);
                    double totalR = (double)(ra + rb);
                    if (totalR < 1.0) totalR = 1.0;
                    double wA = (double)ra / totalR;
                    double pivotX = a->x * wA + b->x * (1.0 - wA);
                    double pivotY = a->y * wA + b->y * (1.0 - wA);
                    double ux = dx / currentLen, uy = dy / currentLen;
                    double distA = currentLen * (1.0 - wA);
                    double distB = currentLen * wA;
                    double scale = targetLen / currentLen;
                    a->x = pivotX - ux * distA * scale;
                    a->y = pivotY - uy * distA * scale;
                    b->x = pivotX + ux * distB * scale;
                    b->y = pivotY + uy * distB * scale;
                    changed = true;
                    break;
                }

                case ConstraintType::Radius: {
                    CircleEntity* circle = sketch.findCircle(c.entityA);
                    if (circle) {
                        if (std::fabs(circle->radius - c.value) > 1e-6f) {
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
                                // Scale start and end points radially from center
                                auto scalePoint = [&](PointEntity* pt) {
                                    double dx = pt->x - cp->x, dy = pt->y - cp->y;
                                    double curR = std::sqrt(dx*dx + dy*dy);
                                    if (curR > 1e-7) {
                                        double scale = targetR / curR;
                                        pt->x = cp->x + dx * scale;
                                        pt->y = cp->y + dy * scale;
                                    }
                                };
                                scalePoint(sp);
                                scalePoint(ep);
                                sketch.recomputeArcAngles(*arc);
                                changed = true;
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

                    double eps = 1e-3;
                    auto pEq = [eps](PointEntity* p, PointEntity* q) {
                        return std::fabs(p->x - q->x) < eps && std::fabs(p->y - q->y) < eps;
                    };

                    // Find vertex (shared endpoint or line intersection)
                    double vx, vy;
                    bool sharedVertex = false;
                    if (pEq(a1, a2) || pEq(a1, b2))      { vx = a1->x; vy = a1->y; sharedVertex = true; }
                    else if (pEq(b1, a2) || pEq(b1, b2)) { vx = b1->x; vy = b1->y; sharedVertex = true; }
                    else {
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
                    if (ccwRad < 0) ccwRad += kTwoPi;
                    double cwRad = kTwoPi - ccwRad;
                    // Use the stored sector (CW or CCW) from when the constraint was created
                    double currentAngle, targetAngle;
                    bool cwSector = c.angleCW;
                    if (!cwSector) {
                        currentAngle = ccwRad;
                        targetAngle = c.value * kPi / 180.0;
                    } else {
                        currentAngle = cwRad;
                        targetAngle = c.value * kPi / 180.0;
                    }

                    double diff = targetAngle - currentAngle;
                    if (diff > kPi) diff -= kTwoPi;
                    if (diff < -kPi) diff += kTwoPi;
                    if (std::fabs(diff) < 1e-6) break;

                    // CW sector: rotation direction is inverted
                    double totalRot = cwSector ? -diff : diff;

                    // Weight: more-constrained line rotates less
                    int r1 = countLineRefs(line1);
                    int r2 = countLineRefs(line2);
                    double totalR = (double)(r1 + r2);
                    if (totalR < 1.0) totalR = 1.0;
                    double w1 = (double)r1 / totalR;
                    double rot2 = totalRot * w1;           // line2's rotation
                    double rot1 = -totalRot * (1.0 - w1); // line1's rotation (opposite)

                    auto rotatePt = [](PointEntity* pt, double ox, double oy, double cosR, double sinR) {
                        double rx = pt->x-ox, ry = pt->y-oy;
                        pt->x = ox + rx*cosR - ry*sinR;
                        pt->y = oy + rx*sinR + ry*cosR;
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

                    // Weight: more constrained entity moves less
                    int rPt = countRefs(c.entityA);
                    int rLine = countLineRefs(line);
                    double totalR = (double)(rPt + rLine);
                    if (totalR < 1.0) totalR = 1.0;
                    // lineFrac = how much point moves (high when line is constrained)
                    // ptFrac = how much line moves (high when point is constrained)
                    double lineFrac = (double)rLine / totalR;
                    double ptFrac = (double)rPt / totalR;

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
                    pt->x += ptMoveX * lineFrac;
                    pt->y += ptMoveY * lineFrac;
                    double ls = lineShift * ptFrac;
                    la->x += nx * ls;
                    la->y += ny * ls;
                    lb->x += nx * ls;
                    lb->y += ny * ls;
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
                    int rPt = countRefs(c.entityA);
                    int rLine = countLineRefs(line);
                    double totalR = (double)(rPt + rLine);
                    if (totalR < 1.0) totalR = 1.0;
                    double lineFrac = (double)rLine / totalR; // how much point moves
                    double ptFrac = (double)rPt / totalR;     // how much line moves

                    // Move point toward line
                    pt->x += nx * dist * lineFrac;
                    pt->y += ny * dist * lineFrac;
                    // Shift line toward point
                    double ls = -dist * ptFrac;
                    la->x += nx * ls;
                    la->y += ny * ls;
                    lb->x += nx * ls;
                    lb->y += ny * ls;
                    changed = true;
                    break;
                }

                case ConstraintType::PointOnCircle: {
                    // entityA = point ID, entityB = circle or arc ID
                    PointEntity* pt = sketch.findPoint(c.entityA);
                    if (!pt) break;
                    // If the point is projected (pinned as a reference), skip — moving the
                    // circle center to compensate would shift the entire circle system.
                    if (pt->projected) break;

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

                    int rPt  = countRefs(c.entityA);
                    int rCen = countRefs(centerPtID);
                    double totalR = (double)(rPt + rCen);
                    if (totalR < 1.0) totalR = 1.0;
                    double cenFrac = (double)rCen / totalR;
                    double ptFrac  = (double)rPt  / totalR;

                    pt->x  -= nx * err * cenFrac;
                    pt->y  -= ny * err * cenFrac;
                    cen->x += nx * err * ptFrac;
                    cen->y += ny * err * ptFrac;
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
                    int r1 = countLineRefs(l1);
                    int r2 = countLineRefs(l2);
                    double totalR = (double)(r1 + r2);
                    if (totalR < 1.0) totalR = 1.0;
                    double w1 = (double)r1 / totalR; // line1's weight — high = changes less
                    // Target length biased toward more-constrained line
                    double target = len1 * w1 + len2 * (1.0 - w1);

                    // Scale each line from weighted pivot
                    auto scaleLine = [&](PointEntity* a, PointEntity* b, double dx, double dy, double curLen,
                                         EntityID startPt, EntityID endPt) {
                        int ra = countRefs(startPt);
                        int rb = countRefs(endPt);
                        double tr = (double)(ra + rb);
                        if (tr < 1.0) tr = 1.0;
                        double wa = (double)ra / tr;
                        double pivX = a->x * wa + b->x * (1.0 - wa);
                        double pivY = a->y * wa + b->y * (1.0 - wa);
                        double ux = dx / curLen, uy = dy / curLen;
                        double dA = curLen * (1.0 - wa);
                        double dB = curLen * wa;
                        double s = target / curLen;
                        a->x = pivX - ux * dA * s; a->y = pivY - uy * dA * s;
                        b->x = pivX + ux * dB * s; b->y = pivY + uy * dB * s;
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
                    double targetAngle = (curAngle >= 0) ? kPi * 0.5 : -kPi * 0.5;
                    double totalRot = targetAngle - curAngle;
                    while (totalRot > kPi) totalRot -= 2*kPi;
                    while (totalRot < -kPi) totalRot += 2*kPi;
                    if (std::fabs(totalRot) < 1e-6) break;

                    // Weight: more-constrained line rotates less
                    int r1 = countLineRefs(l1);
                    int r2 = countLineRefs(l2);
                    double totalR = (double)(r1 + r2);
                    if (totalR < 1.0) totalR = 1.0;
                    double w1 = (double)r1 / totalR; // line1's weight
                    // line2 rotates by totalRot * w1, line1 rotates by -totalRot * w2
                    double rot2 = totalRot * w1;
                    double rot1 = -totalRot * (1.0 - w1);

                    // Check for shared vertex
                    double eps = 1e-3;
                    auto pEq = [eps](PointEntity* p, PointEntity* q) {
                        return std::fabs(p->x-q->x)<eps && std::fabs(p->y-q->y)<eps;
                    };
                    auto dist2 = [](double ax,double ay,double bx,double by){ double ddx=ax-bx,ddy=ay-by; return ddx*ddx+ddy*ddy; };
                    double cx = 0, cy = 0;
                    bool shared = false;
                    if (pEq(a1,a2)||pEq(a1,b2))      { cx=a1->x; cy=a1->y; shared=true; }
                    else if (pEq(b1,a2)||pEq(b1,b2)) { cx=b1->x; cy=b1->y; shared=true; }

                    auto rotatePt = [](PointEntity* pt, double ox, double oy, double cosR, double sinR) {
                        double rx = pt->x-ox, ry = pt->y-oy;
                        pt->x = ox + rx*cosR - ry*sinR;
                        pt->y = oy + rx*sinR + ry*cosR;
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
                    int r1 = countLineRefs(l1);
                    int r2 = countLineRefs(l2);
                    double totalR = (double)(r1 + r2);
                    if (totalR < 1.0) totalR = 1.0;
                    double w1 = (double)r1 / totalR;
                    double rot2 = -totalAngle * w1;         // line2 rotates toward line1
                    double rot1 = totalAngle * (1.0 - w1); // line1 rotates toward line2

                    auto rotatePt = [](PointEntity* p, double ox, double oy, double cosR, double sinR) {
                        double rx = p->x-ox, ry = p->y-oy;
                        p->x = ox + rx*cosR - ry*sinR;
                        p->y = oy + rx*sinR + ry*cosR;
                    };

                    double cos1 = std::cos(rot1), sin1 = std::sin(rot1);
                    double cos2 = std::cos(rot2), sin2 = std::sin(rot2);
                    double mx1 = (a1->x+b1->x)*0.5, my1 = (a1->y+b1->y)*0.5;
                    rotatePt(a1, mx1, my1, cos1, sin1);
                    rotatePt(b1, mx1, my1, cos1, sin1);
                    double mx2 = (a2->x+b2->x)*0.5, my2 = (a2->y+b2->y)*0.5;
                    rotatePt(a2, mx2, my2, cos2, sin2);
                    rotatePt(b2, mx2, my2, cos2, sin2);
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

                    int r1 = countLineRefs(l1);
                    int r2 = countLineRefs(l2);
                    double totalR = (double)(r1 + r2);
                    if (totalR < 1.0) totalR = 1.0;
                    double w1 = (double)r1 / totalR; // fraction of motion absorbed by l2

                    // Step 1: make directions parallel (same as Parallel constraint)
                    double cross = dx1*dy2 - dy1*dx2;
                    if (std::fabs(cross) >= 1e-9) {
                        double dot = dx1*dx2 + dy1*dy2;
                        double totalAngle = std::atan2(cross, dot);
                        double rot2 = -totalAngle * w1;
                        double rot1 =  totalAngle * (1.0 - w1);
                        auto rotatePt = [](PointEntity* p, double ox, double oy, double cosR, double sinR) {
                            double rx = p->x-ox, ry = p->y-oy;
                            p->x = ox + rx*cosR - ry*sinR;
                            p->y = oy + rx*sinR + ry*cosR;
                        };
                        double cos1 = std::cos(rot1), sin1 = std::sin(rot1);
                        double cos2 = std::cos(rot2), sin2 = std::sin(rot2);
                        double mx1 = (a1->x+b1->x)*0.5, my1 = (a1->y+b1->y)*0.5;
                        rotatePt(a1, mx1, my1, cos1, sin1);
                        rotatePt(b1, mx1, my1, cos1, sin1);
                        double mx2 = (a2->x+b2->x)*0.5, my2 = (a2->y+b2->y)*0.5;
                        rotatePt(a2, mx2, my2, cos2, sin2);
                        rotatePt(b2, mx2, my2, cos2, sin2);
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
                            a2->x += nx * move2; a2->y += ny * move2;
                            b2->x += nx * move2; b2->y += ny * move2;
                            a1->x += nx * move1; a1->y += ny * move1;
                            b1->x += nx * move1; b1->y += ny * move1;
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

                    // Determine which line endpoint is on/near the circle (contact point)
                    double distA = std::fabs(distance({la->x, la->y}, {cx, cy}) - radius);
                    double distB = std::fabs(distance({lb->x, lb->y}, {cx, cy}) - radius);
                    PointEntity* contact = (distA < distB) ? la : lb;
                    PointEntity* far = (contact == la) ? lb : la;

                    // Fillet detection: contact point is a shared endpoint of the arc.
                    EntityID contactPtID = (contact == la) ? line->startPt : line->endPt;
                    bool isFillet = arc && (contactPtID == arc->startPt || contactPtID == arc->endPt);

                    if (isFillet && curveCenterPt) {
                        // Fillet case: slide the contact point (line trim endpoint) along the line
                        // to the foot of perpendicular from the arc center. This makes the arc
                        // tangent to the line at the contact point without moving C or the far endpoint.
                        // The arc geometry enforcement (move-center path) will equalise the radius.
                        double fullDx = lb->x - la->x, fullDy = lb->y - la->y;
                        double fullLen = std::sqrt(fullDx*fullDx + fullDy*fullDy);
                        if (fullLen < 1e-9) break;
                        double unitDx = fullDx / fullLen, unitDy = fullDy / fullLen;
                        double t = (cx - la->x) * unitDx + (cy - la->y) * unitDy;
                        double footX = la->x + t * unitDx;
                        double footY = la->y + t * unitDy;
                        if (std::fabs(contact->x - footX) > 1e-6 || std::fabs(contact->y - footY) > 1e-6) {
                            contact->x = footX;
                            contact->y = footY;
                            sketch.recomputeArcAngles(*arc);
                            changed = true;
                        }
                        break;
                    }

                    // Non-fillet: Step 1 — snap contact point onto the circle
                    double dcx = contact->x - cx, dcy = contact->y - cy;
                    double dcLen = std::sqrt(dcx*dcx + dcy*dcy);
                    if (dcLen < 1e-7) break;
                    double newCx2 = cx + dcx / dcLen * radius;
                    double newCy2 = cy + dcy / dcLen * radius;
                    if (std::fabs(contact->x - newCx2) > 1e-6 || std::fabs(contact->y - newCy2) > 1e-6)
                        changed = true;
                    contact->x = newCx2;
                    contact->y = newCy2;

                    double ldx = far->x - contact->x, ldy = far->y - contact->y;
                    double lineLen = std::sqrt(ldx*ldx + ldy*ldy);
                    if (lineLen < 1e-6) break;

                    // Non-fillet Step 2: rotate far endpoint to make line perpendicular to radius.
                    {
                        // Rotate far endpoint around contact so line is perpendicular to radius
                        double rx = contact->x - cx, ry = contact->y - cy;
                        double dot1 = ldx * ry + ldy * (-rx);
                        double dot2 = ldx * (-ry) + ldy * rx;
                        double tx, ty;
                        if (dot1 >= dot2) { tx = ry; ty = -rx; }
                        else              { tx = -ry; ty = rx; }
                        double tLen = std::sqrt(tx*tx + ty*ty);
                        if (tLen < 1e-7) break;
                        double newFarX = contact->x + (tx / tLen) * lineLen;
                        double newFarY = contact->y + (ty / tLen) * lineLen;
                        if (std::fabs(far->x - newFarX) > 1e-6 || std::fabs(far->y - newFarY) > 1e-6) {
                            far->x = newFarX;
                            far->y = newFarY;
                            changed = true;
                        }
                    }
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
                        int rPt = countRefs(c.entityA);
                        int rLine = countLineRefs(line);
                        double totalR = (double)(rPt + rLine);
                        if (totalR < 1.0) totalR = 1.0;
                        double lineFrac = (double)rLine / totalR; // how much point moves
                        double ptFrac = (double)rPt / totalR;     // how much line moves

                        // Move point toward midpoint
                        pt->x -= errX * lineFrac;
                        pt->y -= errY * lineFrac;
                        // Shift line endpoints so midpoint moves toward point
                        la->x += errX * ptFrac;
                        la->y += errY * ptFrac;
                        lb->x += errX * ptFrac;
                        lb->y += errY * ptFrac;
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

                    if (std::fabs(p1->x-targ1X) > 1e-6f || std::fabs(p1->y-targ1Y) > 1e-6f ||
                        std::fabs(p2->x-targ2X) > 1e-6f || std::fabs(p2->y-targ2Y) > 1e-6f) {
                        p1->x = targ1X; p1->y = targ1Y;
                        p2->x = targ2X; p2->y = targ2Y;
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
                        int r1 = countRefs(cp1ID);
                        int r2 = countRefs(cp2ID);
                        double totalR = (double)(r1 + r2);
                        if (totalR < 1.0) totalR = 1.0;
                        double w1 = (double)r1 / totalR;
                        double tX = p1->x * w1 + p2->x * (1.0 - w1);
                        double tY = p1->y * w1 + p2->y * (1.0 - w1);
                        p1->x = tX; p1->y = tY;
                        p2->x = tX; p2->y = tY;
                        changed = true;
                    }
                    break;
                }

                default:
                    break;
            }
        }

        // Enforce arc geometry inside the loop: both endpoints equidistant from center
        for (auto& arc : sketch.arcs) {
            PointEntity* cp = sketch.findPoint(arc.centerPt);
            PointEntity* sp = sketch.findPoint(arc.startPt);
            PointEntity* ep = sketch.findPoint(arc.endPt);
            if (!cp || !sp || !ep) continue;

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
                    cp->x = newCX; cp->y = newCY;
                    changed = true;
                }
            } else {
                sp->x = cp->x + dsx / rStart * targetR;
                sp->y = cp->y + dsy / rStart * targetR;
                ep->x = cp->x + dex / rEnd   * targetR;
                ep->y = cp->y + dey / rEnd   * targetR;
                changed = true;
            }
        }

        result.iterations = iter + 1;
        if (!changed) {
            result.converged = true;
            break;
        }
    }

    // Estimate degrees of freedom: entity free params minus constraint removals.
    {
        // Count all non-projected points (each contributes 2 DOF: x, y)
        int rawDof = 0;
        for (const auto& pt : sketch.points) {
            if (!pt.projected) rawDof += 2;
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
                case ConstraintType::Coincident:
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
                if (ccwRad < 0) ccwRad += 2.0 * 3.14159265358979;
                double ccwDeg = ccwRad * 180.0 / 3.14159265358979;
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

} // namespace shitcad
