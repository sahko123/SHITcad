#pragma once
#include "FeatureHistory.h"
#include "SketchPlane.h"
#include "Scene3D.h"
#include <vector>
#include <set>

namespace shitcad {

// Clear scene and rebuild all 3D geometry by replaying the feature history.
void replayFeatures(FeatureHistory& history,
                    std::vector<SketchPlane>& planes,
                    Scene3D& scene);

// Match profile signatures against detected profiles.
// Returns set of matched profile indices.
std::set<int> matchProfiles(const std::vector<ProfileSignature>& sigs,
                            const std::vector<int>& fallback,
                            const std::vector<ClosedProfile>& detected,
                            const Sketch& sketch);

} // namespace shitcad
