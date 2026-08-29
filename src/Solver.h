#pragma once
#include "SketchData.h"
#include <vector>

namespace shitcad {

struct SolveResult {
    bool ok = false;
    int dof = 0;
    bool converged = false;    // true if solver reached convergence within iteration limit
    float totalError = 0.0f;   // sum of all constraint errors after solving
    int iterations = 0;        // number of iterations used
    // dof is clamped at 0, so it cannot express "more constraints than freedoms". These carry
    // that separately: excessDof is how far past zero the count went, and overConstrained is
    // set when the sketch has more constraint equations than remaining degrees of freedom.
    int excessDof = 0;
    bool overConstrained = false;
};

class Solver {
public:
    SolveResult solve(Sketch& sketch, EntityID draggedPoint = NullID);

private:
    // One relaxation pass. `pinnedPoint`, when set, is held completely immovable for the
    // duration of the pass.
    SolveResult solvePass(Sketch& sketch, EntityID pinnedPoint);
};

} // namespace shitcad
