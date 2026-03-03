#include "Solver.h"
#include <cmath>
#include <algorithm>

namespace shitcad {

SolveResult Solver::solve(Sketch& sketch, EntityID /*draggedPoint*/) {
    SolveResult result;
    result.ok = true;
    result.dof = 0;

    // Simple iterative constraint solver.
    // Handles: Horizontal, Vertical, Coincident, Distance.
    // Run multiple iterations to converge (constraints can chain).

    for (int iter = 0; iter < 20; iter++) {
        bool changed = false;

        for (const auto& c : sketch.constraints) {
            switch (c.type) {
                case ConstraintType::Horizontal: {
                    LineEntity* line = sketch.findLine(c.entityA);
                    if (!line) break;
                    PointEntity* a = sketch.findPoint(line->startPt);
                    PointEntity* b = sketch.findPoint(line->endPt);
                    if (!a || !b) break;

                    float avgY = (a->y + b->y) * 0.5f;
                    if (std::fabs(a->y - avgY) > 1e-6f || std::fabs(b->y - avgY) > 1e-6f) {
                        a->y = avgY;
                        b->y = avgY;
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

                    float avgX = (a->x + b->x) * 0.5f;
                    if (std::fabs(a->x - avgX) > 1e-6f || std::fabs(b->x - avgX) > 1e-6f) {
                        a->x = avgX;
                        b->x = avgX;
                        changed = true;
                    }
                    break;
                }

                case ConstraintType::Coincident: {
                    PointEntity* a = sketch.findPoint(c.entityA);
                    PointEntity* b = sketch.findPoint(c.entityB);
                    if (!a || !b) break;

                    float avgX = (a->x + b->x) * 0.5f;
                    float avgY = (a->y + b->y) * 0.5f;
                    if (std::fabs(a->x - avgX) > 1e-6f || std::fabs(a->y - avgY) > 1e-6f) {
                        a->x = avgX; a->y = avgY;
                        b->x = avgX; b->y = avgY;
                        changed = true;
                    }
                    break;
                }

                case ConstraintType::Distance: {
                    // Distance constraint on a line: set the line to the target length
                    // while keeping its midpoint and direction
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

                    // Scale from midpoint
                    float midX = (a->x + b->x) * 0.5f;
                    float midY = (a->y + b->y) * 0.5f;
                    float scale = targetLen / currentLen;
                    float halfDx = dx * 0.5f * scale;
                    float halfDy = dy * 0.5f * scale;

                    a->x = midX - halfDx;
                    a->y = midY - halfDy;
                    b->x = midX + halfDx;
                    b->y = midY + halfDy;
                    changed = true;
                    break;
                }

                default:
                    break;
            }
        }

        if (!changed) break;
    }

    return result;
}

} // namespace shitcad
