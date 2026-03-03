#include "Solver.h"
#include <cmath>

namespace shitcad {

SolveResult Solver::solve(Sketch& sketch, EntityID /*draggedPoint*/) {
    SolveResult result;
    result.ok = true;
    result.dof = 0;

    // Simple iterative constraint solver for basic constraints.
    // Handles: Horizontal, Vertical, Coincident.
    // More complex constraints (distance, angle, etc.) will use SolveSpace later.

    // Run a few iterations to converge (constraints can chain)
    for (int iter = 0; iter < 10; iter++) {
        bool changed = false;

        for (const auto& c : sketch.constraints) {
            switch (c.type) {
                case ConstraintType::Horizontal: {
                    // entityA is a line — make both endpoints share the same Y
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
                    // entityA is a line — make both endpoints share the same X
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
                    // entityA and entityB are both points — merge positions
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

                default:
                    break;
            }
        }

        if (!changed) break;
    }

    return result;
}

} // namespace shitcad
