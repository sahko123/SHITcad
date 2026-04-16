#include "Solver.h"
#include "Constants.h"
#include <cmath>
#include <algorithm>

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
                        float totalR = (float)(ra + rb);
                        if (totalR < 1.0f) totalR = 1.0f;
                        // Weight: point with more refs moves less
                        float wA = (float)ra / totalR; // fraction of correction applied to b
                        float wB = (float)rb / totalR; // fraction of correction applied to a
                        float avg = a->y * wA + b->y * wB;
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
                        float totalR = (float)(ra + rb);
                        if (totalR < 1.0f) totalR = 1.0f;
                        float wA = (float)ra / totalR;
                        float wB = (float)rb / totalR;
                        float avg = a->x * wA + b->x * wB;
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
                        float totalR = (float)(ra + rb);
                        if (totalR < 1.0f) totalR = 1.0f;
                        float wA = (float)ra / totalR; // a's weight — more refs = moves less
                        float wB = (float)rb / totalR;
                        float tX = a->x * wA + b->x * wB;
                        float tY = a->y * wA + b->y * wB;
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

                    float dx = b->x - a->x;
                    float dy = b->y - a->y;
                    float currentLen = std::sqrt(dx * dx + dy * dy);
                    float targetLen = c.value;

                    if (currentLen < 1e-6f) break;
                    if (std::fabs(currentLen - targetLen) < 1e-6f) break;

                    // Weighted pivot: more-constrained endpoint moves less
                    int ra = countRefs(line->startPt);
                    int rb = countRefs(line->endPt);
                    float totalR = (float)(ra + rb);
                    if (totalR < 1.0f) totalR = 1.0f;
                    float wA = (float)ra / totalR; // a's fraction — high = a moves less
                    // Pivot biased toward more-constrained end
                    float pivotX = a->x * wA + b->x * (1.0f - wA);
                    float pivotY = a->y * wA + b->y * (1.0f - wA);
                    float ux = dx / currentLen, uy = dy / currentLen;
                    // Place endpoints at target distance from pivot, preserving direction
                    float distA = currentLen * (1.0f - wA); // a's distance from pivot
                    float distB = currentLen * wA;          // b's distance from pivot
                    float scale = targetLen / currentLen;
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

                    float dx = b->x - a->x;
                    float dy = b->y - a->y;
                    float currentLen = std::sqrt(dx * dx + dy * dy);
                    float targetLen = c.value;

                    if (currentLen < 1e-6f) break;
                    if (std::fabs(currentLen - targetLen) < 1e-6f) break;

                    int ra = countRefs(c.entityA);
                    int rb = countRefs(c.entityB);
                    float totalR = (float)(ra + rb);
                    if (totalR < 1.0f) totalR = 1.0f;
                    float wA = (float)ra / totalR;
                    float pivotX = a->x * wA + b->x * (1.0f - wA);
                    float pivotY = a->y * wA + b->y * (1.0f - wA);
                    float ux = dx / currentLen, uy = dy / currentLen;
                    float distA = currentLen * (1.0f - wA);
                    float distB = currentLen * wA;
                    float scale = targetLen / currentLen;
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
                                float targetR = c.value;
                                // Scale start and end points radially from center
                                auto scalePoint = [&](PointEntity* pt) {
                                    float dx = pt->x - cp->x, dy = pt->y - cp->y;
                                    float curR = std::sqrt(dx*dx + dy*dy);
                                    if (curR > 1e-7f) {
                                        float scale = targetR / curR;
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
                    float targetR = c.value * 0.5f;
                    if (std::fabs(circle->radius - targetR) > 1e-6f) {
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

                    constexpr float kPi = 3.14159265358979f;
                    constexpr float kTwoPi = 2.0f * kPi;
                    float eps = 1e-3f;
                    auto pEq = [eps](PointEntity* p, PointEntity* q) {
                        return std::fabs(p->x - q->x) < eps && std::fabs(p->y - q->y) < eps;
                    };

                    // Find vertex (shared endpoint or line intersection)
                    float vx, vy;
                    bool sharedVertex = false;
                    if (pEq(a1, a2) || pEq(a1, b2))      { vx = a1->x; vy = a1->y; sharedVertex = true; }
                    else if (pEq(b1, a2) || pEq(b1, b2)) { vx = b1->x; vy = b1->y; sharedVertex = true; }
                    else {
                        // Compute intersection of infinite lines
                        float ldx1 = b1->x-a1->x, ldy1 = b1->y-a1->y;
                        float ldx2 = b2->x-a2->x, ldy2 = b2->y-a2->y;
                        float denom = ldx1*ldy2 - ldy1*ldx2;
                        if (std::fabs(denom) < 1e-6f) break; // parallel
                        float t = ((a2->x-a1->x)*ldy2 - (a2->y-a1->y)*ldx2) / denom;
                        vx = a1->x + t*ldx1; vy = a1->y + t*ldy1;
                    }

                    // Direction from vertex toward farther endpoint of each line
                    auto dist2 = [](float ax, float ay, float bx, float by) {
                        float dx = ax-bx, dy = ay-by; return dx*dx + dy*dy;
                    };
                    PointEntity* far1 = (dist2(vx,vy,b1->x,b1->y) >= dist2(vx,vy,a1->x,a1->y)) ? b1 : a1;
                    PointEntity* far2 = (dist2(vx,vy,b2->x,b2->y) >= dist2(vx,vy,a2->x,a2->y)) ? b2 : a2;

                    float dx1 = far1->x - vx, dy1 = far1->y - vy;
                    float dx2 = far2->x - vx, dy2 = far2->y - vy;
                    float len1 = std::sqrt(dx1*dx1 + dy1*dy1);
                    float len2 = std::sqrt(dx2*dx2 + dy2*dy2);
                    if (len1 < 1e-6f || len2 < 1e-6f) break;

                    float dot = dx1*dx2 + dy1*dy2;
                    float cross = dx1*dy2 - dy1*dx2;
                    float ccwRad = std::atan2(cross, dot);
                    if (ccwRad < 0) ccwRad += kTwoPi;
                    float cwRad = kTwoPi - ccwRad;
                    // Use the stored sector (CW or CCW) from when the constraint was created
                    float currentAngle, targetAngle;
                    bool cwSector = c.angleCW;
                    if (!cwSector) {
                        currentAngle = ccwRad;
                        targetAngle = c.value * kPi / 180.0f;
                    } else {
                        currentAngle = cwRad;
                        targetAngle = c.value * kPi / 180.0f;
                    }

                    float diff = targetAngle - currentAngle;
                    if (diff > kPi) diff -= kTwoPi;
                    if (diff < -kPi) diff += kTwoPi;
                    if (std::fabs(diff) < 1e-6f) break;

                    // CW sector: rotation direction is inverted
                    float totalRot = cwSector ? -diff : diff;

                    // Weight: more-constrained line rotates less
                    int r1 = countLineRefs(line1);
                    int r2 = countLineRefs(line2);
                    float totalR = (float)(r1 + r2);
                    if (totalR < 1.0f) totalR = 1.0f;
                    float w1 = (float)r1 / totalR;
                    float rot2 = totalRot * w1;           // line2's rotation
                    float rot1 = -totalRot * (1.0f - w1); // line1's rotation (opposite)

                    auto rotatePt = [](PointEntity* pt, float ox, float oy, float cosR, float sinR) {
                        float rx = pt->x-ox, ry = pt->y-oy;
                        pt->x = ox + rx*cosR - ry*sinR;
                        pt->y = oy + rx*sinR + ry*cosR;
                    };

                    if (sharedVertex) {
                        // Rotate far endpoints around shared vertex
                        PointEntity* far1_ = (dist2(vx,vy,b1->x,b1->y) >= dist2(vx,vy,a1->x,a1->y)) ? b1 : a1;
                        float cos1 = std::cos(rot1), sin1 = std::sin(rot1);
                        float cos2 = std::cos(rot2), sin2 = std::sin(rot2);
                        rotatePt(far1_, vx, vy, cos1, sin1);
                        rotatePt(far2, vx, vy, cos2, sin2);
                    } else {
                        // Rotate each line around the intersection point
                        float cos1 = std::cos(rot1), sin1 = std::sin(rot1);
                        float cos2 = std::cos(rot2), sin2 = std::sin(rot2);
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

                    float ldx = lb->x - la->x, ldy = lb->y - la->y;
                    float lineLen2 = ldx*ldx + ldy*ldy;
                    if (lineLen2 < 1e-12f) break;
                    float lineLen = std::sqrt(lineLen2);

                    // Signed perpendicular distance from point to line
                    float cross = (pt->x - la->x)*ldy - (pt->y - la->y)*ldx;
                    float dist = cross / lineLen;

                    // Target signed distance: initial side * value
                    // Negative value flips the side
                    float sideSign = c.negativeSide ? -1.0f : 1.0f;
                    float targetSigned = sideSign * c.value;

                    if (std::fabs(dist - targetSigned) < 1e-6f) break;

                    float nx = -ldy / lineLen;
                    float ny = ldx / lineLen;
                    float error = targetSigned - dist;

                    // Weight: more constrained entity moves less
                    int rPt = countRefs(c.entityA);
                    int rLine = countLineRefs(line);
                    float totalR = (float)(rPt + rLine);
                    if (totalR < 1.0f) totalR = 1.0f;
                    // lineFrac = how much point moves (high when line is constrained)
                    // ptFrac = how much line moves (high when point is constrained)
                    float lineFrac = (float)rLine / totalR;
                    float ptFrac = (float)rPt / totalR;

                    // Point target: project onto line, then offset to target distance
                    // pt = proj - normal * targetSigned (derived from sign convention)
                    float t = ((pt->x - la->x)*ldx + (pt->y - la->y)*ldy) / lineLen2;
                    float projX = la->x + t*ldx;
                    float projY = la->y + t*ldy;
                    float ptTargetX = projX - nx * targetSigned;
                    float ptTargetY = projY - ny * targetSigned;
                    float ptMoveX = ptTargetX - pt->x;
                    float ptMoveY = ptTargetY - pt->y;

                    // Line shift: moving line by (nx*s) changes dist by +s
                    // We need dist to change by +error, so s = error
                    float lineShift = error;

                    // Apply weighted
                    pt->x += ptMoveX * lineFrac;
                    pt->y += ptMoveY * lineFrac;
                    float ls = lineShift * ptFrac;
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

                    float ldx = lb->x - la->x, ldy = lb->y - la->y;
                    float len2 = ldx*ldx + ldy*ldy;
                    if (len2 < 1e-12f) break;
                    float lineLen = std::sqrt(len2);

                    // Signed perpendicular distance from point to line
                    float cross = (pt->x - la->x)*ldy - (pt->y - la->y)*ldx;
                    float dist = cross / lineLen;
                    if (std::fabs(dist) < 1e-6f) break;

                    // Normal direction (perpendicular to line)
                    float nx = -ldy / lineLen;
                    float ny = ldx / lineLen;

                    // Weight: point vs line
                    int rPt = countRefs(c.entityA);
                    int rLine = countLineRefs(line);
                    float totalR = (float)(rPt + rLine);
                    if (totalR < 1.0f) totalR = 1.0f;
                    float lineFrac = (float)rLine / totalR; // how much point moves
                    float ptFrac = (float)rPt / totalR;     // how much line moves

                    // Move point toward line
                    pt->x += nx * dist * lineFrac;
                    pt->y += ny * dist * lineFrac;
                    // Shift line toward point
                    float ls = -dist * ptFrac;
                    la->x += nx * ls;
                    la->y += ny * ls;
                    lb->x += nx * ls;
                    lb->y += ny * ls;
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

                    float dx1 = b1->x-a1->x, dy1 = b1->y-a1->y;
                    float dx2 = b2->x-a2->x, dy2 = b2->y-a2->y;
                    float len1 = std::sqrt(dx1*dx1 + dy1*dy1);
                    float len2 = std::sqrt(dx2*dx2 + dy2*dy2);
                    if (len1 < 1e-6f || len2 < 1e-6f) break;

                    if (std::fabs(len1 - len2) < 1e-6f) break;

                    // Weighted target: more-constrained line changes less
                    int r1 = countLineRefs(l1);
                    int r2 = countLineRefs(l2);
                    float totalR = (float)(r1 + r2);
                    if (totalR < 1.0f) totalR = 1.0f;
                    float w1 = (float)r1 / totalR; // line1's weight — high = changes less
                    // Target length biased toward more-constrained line
                    float target = len1 * w1 + len2 * (1.0f - w1);

                    // Scale each line from weighted pivot
                    auto scaleLine = [&](PointEntity* a, PointEntity* b, float dx, float dy, float curLen,
                                         EntityID startPt, EntityID endPt) {
                        int ra = countRefs(startPt);
                        int rb = countRefs(endPt);
                        float tr = (float)(ra + rb);
                        if (tr < 1.0f) tr = 1.0f;
                        float wa = (float)ra / tr;
                        float pivX = a->x * wa + b->x * (1.0f - wa);
                        float pivY = a->y * wa + b->y * (1.0f - wa);
                        float ux = dx / curLen, uy = dy / curLen;
                        float dA = curLen * (1.0f - wa);
                        float dB = curLen * wa;
                        float s = target / curLen;
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

                    float dx1 = b1->x-a1->x, dy1 = b1->y-a1->y;
                    float dx2 = b2->x-a2->x, dy2 = b2->y-a2->y;
                    float len1 = std::sqrt(dx1*dx1 + dy1*dy1);
                    float len2 = std::sqrt(dx2*dx2 + dy2*dy2);
                    if (len1 < 1e-6f || len2 < 1e-6f) break;

                    float dot = (dx1*dx2 + dy1*dy2) / (len1*len2);
                    if (std::fabs(dot) < 1e-6f) break; // already perpendicular

                    // Total rotation needed to make lines perpendicular
                    float cross = dx1*dy2 - dy1*dx2;
                    float curAngle = std::atan2(cross, dx1*dx2 + dy1*dy2);
                    constexpr float kPi = 3.14159265358979f;
                    float targetAngle = (curAngle >= 0) ? kPi * 0.5f : -kPi * 0.5f;
                    float totalRot = targetAngle - curAngle;
                    while (totalRot > kPi) totalRot -= 2*kPi;
                    while (totalRot < -kPi) totalRot += 2*kPi;
                    if (std::fabs(totalRot) < 1e-6f) break;

                    // Weight: more-constrained line rotates less
                    int r1 = countLineRefs(l1);
                    int r2 = countLineRefs(l2);
                    float totalR = (float)(r1 + r2);
                    if (totalR < 1.0f) totalR = 1.0f;
                    float w1 = (float)r1 / totalR; // line1's weight
                    // line2 rotates by totalRot * w1, line1 rotates by -totalRot * w2
                    float rot2 = totalRot * w1;
                    float rot1 = -totalRot * (1.0f - w1);

                    // Check for shared vertex
                    float eps = 1e-3f;
                    auto pEq = [eps](PointEntity* p, PointEntity* q) {
                        return std::fabs(p->x-q->x)<eps && std::fabs(p->y-q->y)<eps;
                    };
                    auto dist2 = [](float ax,float ay,float bx,float by){ float ddx=ax-bx,ddy=ay-by; return ddx*ddx+ddy*ddy; };
                    float cx = 0, cy = 0;
                    bool shared = false;
                    if (pEq(a1,a2)||pEq(a1,b2))      { cx=a1->x; cy=a1->y; shared=true; }
                    else if (pEq(b1,a2)||pEq(b1,b2)) { cx=b1->x; cy=b1->y; shared=true; }

                    auto rotatePt = [](PointEntity* pt, float ox, float oy, float cosR, float sinR) {
                        float rx = pt->x-ox, ry = pt->y-oy;
                        pt->x = ox + rx*cosR - ry*sinR;
                        pt->y = oy + rx*sinR + ry*cosR;
                    };

                    if (shared) {
                        // Rotate far endpoints around shared vertex
                        PointEntity* far1 = (dist2(cx,cy,b1->x,b1->y)>=dist2(cx,cy,a1->x,a1->y)) ? b1 : a1;
                        PointEntity* far2 = (dist2(cx,cy,b2->x,b2->y)>=dist2(cx,cy,a2->x,a2->y)) ? b2 : a2;
                        float cos1 = std::cos(rot1), sin1 = std::sin(rot1);
                        float cos2 = std::cos(rot2), sin2 = std::sin(rot2);
                        rotatePt(far1, cx, cy, cos1, sin1);
                        rotatePt(far2, cx, cy, cos2, sin2);
                    } else {
                        // Rotate each line around its midpoint
                        float cos1 = std::cos(rot1), sin1 = std::sin(rot1);
                        float cos2 = std::cos(rot2), sin2 = std::sin(rot2);
                        float mx1 = (a1->x+b1->x)*0.5f, my1 = (a1->y+b1->y)*0.5f;
                        rotatePt(a1, mx1, my1, cos1, sin1);
                        rotatePt(b1, mx1, my1, cos1, sin1);
                        float mx2 = (a2->x+b2->x)*0.5f, my2 = (a2->y+b2->y)*0.5f;
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

                    float dx1 = b1->x-a1->x, dy1 = b1->y-a1->y;
                    float dx2 = b2->x-a2->x, dy2 = b2->y-a2->y;
                    float len1 = std::sqrt(dx1*dx1 + dy1*dy1);
                    float len2 = std::sqrt(dx2*dx2 + dy2*dy2);
                    if (len1 < 1e-6f || len2 < 1e-6f) break;

                    float cross = dx1*dy2 - dy1*dx2;
                    if (std::fabs(cross) < 1e-6f * len1 * len2) break; // already parallel

                    float dot = dx1*dx2 + dy1*dy2;
                    float totalAngle = std::atan2(cross, dot); // angle from parallel

                    // Weight: more-constrained line rotates less
                    int r1 = countLineRefs(l1);
                    int r2 = countLineRefs(l2);
                    float totalR = (float)(r1 + r2);
                    if (totalR < 1.0f) totalR = 1.0f;
                    float w1 = (float)r1 / totalR;
                    float rot2 = -totalAngle * w1;         // line2 rotates toward line1
                    float rot1 = totalAngle * (1.0f - w1); // line1 rotates toward line2

                    auto rotatePt = [](PointEntity* pt, float ox, float oy, float cosR, float sinR) {
                        float rx = pt->x-ox, ry = pt->y-oy;
                        pt->x = ox + rx*cosR - ry*sinR;
                        pt->y = oy + rx*sinR + ry*cosR;
                    };

                    float cos1 = std::cos(rot1), sin1 = std::sin(rot1);
                    float cos2 = std::cos(rot2), sin2 = std::sin(rot2);
                    float mx1 = (a1->x+b1->x)*0.5f, my1 = (a1->y+b1->y)*0.5f;
                    rotatePt(a1, mx1, my1, cos1, sin1);
                    rotatePt(b1, mx1, my1, cos1, sin1);
                    float mx2 = (a2->x+b2->x)*0.5f, my2 = (a2->y+b2->y)*0.5f;
                    rotatePt(a2, mx2, my2, cos2, sin2);
                    rotatePt(b2, mx2, my2, cos2, sin2);
                    changed = true;
                    break;
                }

                case ConstraintType::Tangent: {
                    // A=line, B=circle or arc
                    LineEntity* line = sketch.findLine(c.entityA);
                    if (!line) break;
                    PointEntity* la = sketch.findPoint(line->startPt);
                    PointEntity* lb = sketch.findPoint(line->endPt);
                    if (!la || !lb) break;

                    float cx, cy, radius;
                    CircleEntity* circle = sketch.findCircle(c.entityB);
                    ArcEntity* arc = sketch.findArc(c.entityB);
                    if (circle) {
                        PointEntity* cp = sketch.findPoint(circle->centerPt);
                        if (!cp) break;
                        cx = cp->x; cy = cp->y; radius = circle->radius;
                    } else if (arc) {
                        PointEntity* cp = sketch.findPoint(arc->centerPt);
                        PointEntity* sp = sketch.findPoint(arc->startPt);
                        if (!cp || !sp) break;
                        cx = cp->x; cy = cp->y;
                        float dx = sp->x-cp->x, dy = sp->y-cp->y;
                        radius = std::sqrt(dx*dx + dy*dy);
                    } else break;

                    // Determine which line endpoint is on/near the circle (contact point)
                    float distA = std::fabs(distance({la->x, la->y}, {cx, cy}) - radius);
                    float distB = std::fabs(distance({lb->x, lb->y}, {cx, cy}) - radius);
                    PointEntity* contact = (distA < distB) ? la : lb;  // endpoint on circle
                    PointEntity* far = (contact == la) ? lb : la;      // the other endpoint

                    // Step 1: Snap contact point onto the circle
                    float dcx = contact->x - cx, dcy = contact->y - cy;
                    float dcLen = std::sqrt(dcx*dcx + dcy*dcy);
                    if (dcLen < 1e-7f) break;
                    float newCx = cx + dcx / dcLen * radius;
                    float newCy = cy + dcy / dcLen * radius;
                    if (std::fabs(contact->x - newCx) > 1e-6f || std::fabs(contact->y - newCy) > 1e-6f)
                        changed = true;
                    contact->x = newCx;
                    contact->y = newCy;

                    // Step 2: Rotate the far endpoint around the contact point so the line
                    // is tangent to the circle (perpendicular to the radius at contact)
                    // Tangent direction at contact: perpendicular to radius vector
                    float rx = contact->x - cx, ry = contact->y - cy;
                    // Two tangent directions: (ry, -rx) and (-ry, rx)
                    // Pick the one closest to current line direction
                    float ldx = far->x - contact->x, ldy = far->y - contact->y;
                    float lineLen = std::sqrt(ldx*ldx + ldy*ldy);
                    if (lineLen < 1e-6f) break;

                    float dot1 = ldx * ry + ldy * (-rx);
                    float dot2 = ldx * (-ry) + ldy * rx;
                    float tx, ty;
                    if (dot1 >= dot2) { tx = ry; ty = -rx; }
                    else              { tx = -ry; ty = rx; }

                    // Place far endpoint along tangent direction at current distance
                    float tLen = std::sqrt(tx*tx + ty*ty);
                    if (tLen < 1e-7f) break;
                    float newFarX = contact->x + (tx / tLen) * lineLen;
                    float newFarY = contact->y + (ty / tLen) * lineLen;
                    if (std::fabs(far->x - newFarX) > 1e-6f || std::fabs(far->y - newFarY) > 1e-6f) {
                        far->x = newFarX;
                        far->y = newFarY;
                        changed = true;
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

                    float mx = (la->x + lb->x) * 0.5f;
                    float my = (la->y + lb->y) * 0.5f;
                    float errX = pt->x - mx, errY = pt->y - my;
                    if (std::fabs(errX) > 1e-6f || std::fabs(errY) > 1e-6f) {
                        // Weight: point vs line endpoints
                        int rPt = countRefs(c.entityA);
                        int rLine = countLineRefs(line);
                        float totalR = (float)(rPt + rLine);
                        if (totalR < 1.0f) totalR = 1.0f;
                        float lineFrac = (float)rLine / totalR; // how much point moves
                        float ptFrac = (float)rPt / totalR;     // how much line moves

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
                    float adx = ab->x-aa->x, ady = ab->y-aa->y;
                    float alen2 = adx*adx + ady*ady;
                    if (alen2 < 1e-12f) break;

                    // Reflection of p1: r = 2*proj(p1-aa, axis) + aa - (p1-aa) + aa
                    float px = p1->x-aa->x, py = p1->y-aa->y;
                    float t = (px*adx + py*ady) / alen2;
                    float reflX = 2*(aa->x + t*adx) - p1->x;
                    float reflY = 2*(aa->y + t*ady) - p1->y;

                    // Also reflect p2 across axis
                    float px2 = p2->x-aa->x, py2 = p2->y-aa->y;
                    float t2 = (px2*adx + py2*ady) / alen2;
                    float refl2X = 2*(aa->x + t2*adx) - p2->x;
                    float refl2Y = 2*(aa->y + t2*ady) - p2->y;

                    // Target: midpoint of both reflections (symmetric convergence)
                    float targ1X = (p1->x + refl2X) * 0.5f;
                    float targ1Y = (p1->y + refl2Y) * 0.5f;
                    float targ2X = (p2->x + reflX) * 0.5f;
                    float targ2Y = (p2->y + reflY) * 0.5f;

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

                    if (std::fabs(p1->x - p2->x) > 1e-6f || std::fabs(p1->y - p2->y) > 1e-6f) {
                        int r1 = countRefs(cp1ID);
                        int r2 = countRefs(cp2ID);
                        float totalR = (float)(r1 + r2);
                        if (totalR < 1.0f) totalR = 1.0f;
                        float w1 = (float)r1 / totalR;
                        float tX = p1->x * w1 + p2->x * (1.0f - w1);
                        float tY = p1->y * w1 + p2->y * (1.0f - w1);
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

            float dsx = sp->x - cp->x, dsy = sp->y - cp->y;
            float dex = ep->x - cp->x, dey = ep->y - cp->y;
            float rStart = std::sqrt(dsx*dsx + dsy*dsy);
            float rEnd   = std::sqrt(dex*dex + dey*dey);

            if (rStart < 1e-7f || rEnd < 1e-7f) continue;
            if (std::fabs(rStart - rEnd) < 1e-6f) continue;

            float targetR;
            if (draggedPoint == arc.startPt)
                targetR = rStart;
            else if (draggedPoint == arc.endPt)
                targetR = rEnd;
            else
                targetR = (rStart + rEnd) * 0.5f;

            sp->x = cp->x + dsx / rStart * targetR;
            sp->y = cp->y + dsy / rStart * targetR;
            ep->x = cp->x + dex / rEnd   * targetR;
            ep->y = cp->y + dey / rEnd   * targetR;
            changed = true;
        }

        result.iterations = iter + 1;
        if (!changed) {
            result.converged = true;
            break;
        }
    }

    // Check if all constraints are satisfied after solving
    for (const auto& c : sketch.constraints) {
        if (c.driven) continue;
        float err = 0;
        switch (c.type) {
            case ConstraintType::Distance: {
                LineEntity* line = sketch.findLine(c.entityA);
                if (!line) break;
                PointEntity* a = sketch.findPoint(line->startPt);
                PointEntity* b = sketch.findPoint(line->endPt);
                if (!a || !b) break;
                float dx = b->x - a->x, dy = b->y - a->y;
                err = std::fabs(std::sqrt(dx*dx + dy*dy) - c.value);
                break;
            }
            case ConstraintType::PointDistance: {
                PointEntity* a = sketch.findPoint(c.entityA);
                PointEntity* b = sketch.findPoint(c.entityB);
                if (!a || !b) break;
                float dx = b->x - a->x, dy = b->y - a->y;
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
                if (circle) err = std::fabs(circle->radius - c.value * 0.5f);
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
                float eps = 1e-3f;
                auto pEq = [eps](PointEntity* p, PointEntity* q) {
                    return std::fabs(p->x - q->x) < eps && std::fabs(p->y - q->y) < eps;
                };
                float vx, vy;
                if (pEq(a1, a2) || pEq(a1, b2))      { vx = a1->x; vy = a1->y; }
                else if (pEq(b1, a2) || pEq(b1, b2)) { vx = b1->x; vy = b1->y; }
                else {
                    float ldx1 = b1->x-a1->x, ldy1 = b1->y-a1->y;
                    float ldx2 = b2->x-a2->x, ldy2 = b2->y-a2->y;
                    float denom = ldx1*ldy2 - ldy1*ldx2;
                    if (std::fabs(denom) < 1e-6f) break;
                    float t = ((a2->x-a1->x)*ldy2 - (a2->y-a1->y)*ldx2) / denom;
                    vx = a1->x + t*ldx1; vy = a1->y + t*ldy1;
                }
                auto dist2v = [](float ax, float ay, float bx, float by) {
                    float dx = ax-bx, dy = ay-by; return dx*dx + dy*dy;
                };
                PointEntity* f1 = (dist2v(vx,vy,b1->x,b1->y)>=dist2v(vx,vy,a1->x,a1->y)) ? b1 : a1;
                PointEntity* f2 = (dist2v(vx,vy,b2->x,b2->y)>=dist2v(vx,vy,a2->x,a2->y)) ? b2 : a2;
                float dx1 = f1->x-vx, dy1 = f1->y-vy;
                float dx2 = f2->x-vx, dy2 = f2->y-vy;
                float dot = dx1*dx2 + dy1*dy2;
                float cross = dx1*dy2 - dy1*dx2;
                float ccwRad = std::atan2(cross, dot);
                if (ccwRad < 0) ccwRad += 2.0f * 3.14159265358979f;
                float ccwDeg = ccwRad * 180.0f / 3.14159265358979f;
                float cwDeg = 360.0f - ccwDeg;
                // Use the stored sector
                float currentDeg = c.angleCW ? cwDeg : ccwDeg;
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
                float ldx = lb->x-la->x, ldy = lb->y-la->y;
                float lineLen = std::sqrt(ldx*ldx+ldy*ldy);
                if (lineLen < 1e-6f) break;
                float cross = (pt->x-la->x)*ldy - (pt->y-la->y)*ldx;
                float dist = cross / lineLen;
                float sideSign = c.negativeSide ? -1.0f : 1.0f;
                float targetSigned = sideSign * c.value;
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
                float ldx = lb->x-la->x, ldy = lb->y-la->y;
                float lineLen = std::sqrt(ldx*ldx+ldy*ldy);
                if (lineLen < 1e-6f) break;
                float cross = (pt->x-la->x)*ldy - (pt->y-la->y)*ldx;
                err = std::fabs(cross) / lineLen;
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
                float dx1=b1->x-a1->x, dy1=b1->y-a1->y;
                float dx2=b2->x-a2->x, dy2=b2->y-a2->y;
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
                float dx1=b1->x-a1->x, dy1=b1->y-a1->y;
                float dx2=b2->x-a2->x, dy2=b2->y-a2->y;
                float len1=std::sqrt(dx1*dx1+dy1*dy1), len2=std::sqrt(dx2*dx2+dy2*dy2);
                if (len1>1e-6f && len2>1e-6f)
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
                float dx1=b1->x-a1->x, dy1=b1->y-a1->y;
                float dx2=b2->x-a2->x, dy2=b2->y-a2->y;
                float len1=std::sqrt(dx1*dx1+dy1*dy1), len2=std::sqrt(dx2*dx2+dy2*dy2);
                if (len1>1e-6f && len2>1e-6f)
                    err = std::fabs(dx1*dy2-dy1*dx2) / (len1*len2);
                break;
            }
            case ConstraintType::Tangent: {
                LineEntity* line = sketch.findLine(c.entityA);
                if (!line) break;
                PointEntity* la = sketch.findPoint(line->startPt);
                PointEntity* lb = sketch.findPoint(line->endPt);
                if (!la || !lb) break;
                float cx, cy, radius;
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
                    float dx=sp->x-cp->x, dy=sp->y-cp->y;
                    radius=std::sqrt(dx*dx+dy*dy);
                } else break;
                float ldx=lb->x-la->x, ldy=lb->y-la->y;
                float lineLen=std::sqrt(ldx*ldx+ldy*ldy);
                if (lineLen<1e-6f) break;
                float cross=(cx-la->x)*ldy-(cy-la->y)*ldx;
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
                float mx=(la->x+lb->x)*0.5f, my=(la->y+lb->y)*0.5f;
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
                float adx=ab->x-aa->x, ady=ab->y-aa->y;
                float alen2=adx*adx+ady*ady;
                if (alen2<1e-12f) break;
                float px=p1->x-aa->x, py=p1->y-aa->y;
                float t=(px*adx+py*ady)/alen2;
                float reflX=2*(aa->x+t*adx)-p1->x;
                float reflY=2*(aa->y+t*ady)-p1->y;
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
        result.totalError += err;
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
