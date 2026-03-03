#pragma once
#include "SketchData.h"
#include <vector>

namespace shitcad {

struct SolveResult {
    bool ok = false;
    int dof = 0;
};

class Solver {
public:
    SolveResult solve(Sketch& sketch, EntityID draggedPoint = NullID);
};

} // namespace shitcad
