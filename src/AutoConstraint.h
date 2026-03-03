#pragma once
#include "SketchData.h"
#include <vector>

namespace shitcad {

struct PendingConstraint {
    ConstraintType type;
    EntityID entityA;
    EntityID entityB = NullID;
    float value = 0.0f;
};

// Analyze a newly-created line and return auto-constraints to apply.
std::vector<PendingConstraint> detectLineAutoConstraints(
    const Sketch& sketch, EntityID lineID);

} // namespace shitcad
