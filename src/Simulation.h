#pragma once
#include "FeatureHistory.h"
#include <nlohmann/json_fwd.hpp>
#include <cstdint>
#include <string>
#include <vector>

namespace shitcad {

// ---- Simulation set-up ------------------------------------------------------
//
// What a spray simulation needs on top of the geometry: what each imported
// surface *is*, and where the nozzles are. Saved with the project, undoable,
// and exported as a cip-sim spec file (see cip-sim/spec/README.md).
//
// This is set-up data, not geometry, so it lives beside the feature history
// rather than in it: nothing here changes a body.

// What a surface is. Decides whether its area counts toward coverage
// ("scored") and, later, what boundary condition CFD gives it.
enum class SurfaceRole : uint8_t {
    Wall,          // vessel wall to be cleaned              - scored
    InternalWall,  // pipework, baffles, probes to be cleaned - scored
    Inlet,         // cap over the CIP supply entry           - blocks spray, not scored
    Drain,         // cap over the drain / outlet             - blocks spray, not scored
    Obstruction,   // blocks spray, not cleaning-relevant     - not scored
};
inline constexpr int kSurfaceRoleCount = 5;

const char* surfaceRoleName(SurfaceRole role);      // stable id used in files
const char* surfaceRoleLabel(SurfaceRole role);     // for the UI
bool surfaceRoleScored(SurfaceRole role);
bool surfaceRoleFromName(const std::string& name, SurfaceRole& out);

struct SimSurfaceRole {
    FeatureID meshFeature = NullFeatureID;
    SurfaceRole role = SurfaceRole::Wall;
};

// A spray nozzle. Position and axis are stored in the frame of the mesh it was
// placed on (mm, after that mesh's unit scaling, before its placement), so
// rotating or moving the vessel carries its nozzles with it. hostFeature 0
// means the values are already world coordinates.
struct SimNozzle {
    uint32_t id = 0;
    std::string name;
    FeatureID hostFeature = NullFeatureID;
    double position[3] = {0, 0, 0};
    double axis[3] = {0, -1, 0};
    float halfAngleDeg = 65.0f;  // 180 = full spray ball
    float mdotKgS = 0.30f;
    float pressureBar = 2.0f;
};

struct SimulationSetup {
    std::vector<SimSurfaceRole> roles; // only surfaces whose role was set; others are Wall
    std::vector<SimNozzle> nozzles;
    int rays = 400000;
    int bounces = 2;
    uint32_t nextNozzleID = 1;

    SurfaceRole roleFor(FeatureID meshFeature) const;
    void setRole(FeatureID meshFeature, SurfaceRole role);
    SimNozzle* findNozzle(uint32_t id);
    const SimNozzle* findNozzle(uint32_t id) const;
    uint32_t addNozzle(SimNozzle n); // assigns id and, if empty, a name
    void removeNozzle(uint32_t id);
    bool empty() const { return roles.empty() && nozzles.empty(); }
};

bool sameSimulation(const SimulationSetup& a, const SimulationSetup& b);

// World position (mm) and unit axis of a nozzle, following its host's placement.
// Returns false if the host feature no longer exists (values then unchanged).
bool nozzleWorld(const SimNozzle& n, const FeatureHistory& history, double pos[3], double axis[3]);

// Set a nozzle's position/axis from world values, keeping it attached to its host.
void setNozzleWorld(SimNozzle& n, const FeatureHistory& history, const double pos[3], const double axis[3]);

// Build a cip-sim spec (JSON text) for Tier 1. Surfaces are the active
// (not suppressed, not rolled back, error-free) MeshImport features. Fails with
// a message a user can act on when the set-up cannot produce a valid spec.
bool buildTier1Spec(const SimulationSetup& sim, const FeatureHistory& history,
                    std::string& outJson, std::vector<std::string>& warnings, std::string& error);

void simulationToJson(const SimulationSetup& sim, nlohmann::json& out);
bool simulationFromJson(const nlohmann::json& in, SimulationSetup& out, std::string& error);

} // namespace shitcad
