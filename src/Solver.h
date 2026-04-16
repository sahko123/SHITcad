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
};

class Solver {
public:
    SolveResult solve(Sketch& sketch, EntityID draggedPoint = NullID);
};

} // namespace shitcad
