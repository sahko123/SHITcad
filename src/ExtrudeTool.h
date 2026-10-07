#pragma once
#include "ProfileDetector.h"
#include "SketchPlane.h"
#include "Scene3D.h"
#include <vector>
#include <set>
#include <cstdio>
#include <chrono>

namespace shitcad {

enum class ExtrudeOperation : uint8_t {
    NewBody,
    Cut,
};

enum class BooleanOperation : uint8_t {
    Union,
    Subtract,
};

enum class ExtrudeDirection : uint8_t {
    OneSide,
    OtherSide,
    BothSides,
    Symmetric,
};

enum class ExtrudePhase : uint8_t {
    SelectingProfiles,
};

// Pre-computed tessellation + ear-clipping for profile rendering/hit testing
struct ProfileRenderCache {
    std::vector<Point2D> tessPoints;     // tessellated outer boundary (for hit testing + boundary rendering)
    std::vector<Point2D> triMeshPoints;  // merged polygon (outer + holes via bridge edges, for triangulation)
    std::vector<int> triIndices;         // ear-clipped triangle indices into triMeshPoints
    std::vector<std::vector<Point2D>> holeTessPoints; // tessellated hole boundaries
};

// Build render cache for all profiles (call once when entering extrude mode)
void buildProfileRenderCaches(const Sketch& sketch,
                              const std::vector<ClosedProfile>& profiles,
                              std::vector<ProfileRenderCache>& out);

// State every 3D feature tool (Extrude, Revolve, Loft) carries.
struct FeatureToolBase {
    ExtrudeOperation operation = ExtrudeOperation::NewBody;
    Body3D previewBody;
    bool previewDirty = true;
    uint32_t editingFeatureID = 0; // 0 (NullFeatureID) = creating new, otherwise editing existing
};

// Extrude and Revolve both work from profiles detected in one sketch plane.
struct ProfileToolBase : FeatureToolBase {
    std::vector<ClosedProfile> allProfiles;
    std::set<int> selectedProfileIndices;
    std::vector<ProfileRenderCache> renderCache; // cached tessellation per profile
    std::chrono::steady_clock::time_point lastPreviewTime; // throttle preview rebuilds
    int sketchPlaneIndex = -1; // which sketch plane the profiles come from

    bool hasSelectedProfiles() const { return !selectedProfileIndices.empty(); }
};

struct ExtrudeToolState : ProfileToolBase {
    float height = 10.0f;
    float offset = 0.0f;
    ExtrudeDirection direction = ExtrudeDirection::OneSide;

    ExtrudePhase phase = ExtrudePhase::SelectingProfiles;
    bool isDragging = false;
    float dragStartMouseX = 0.0f;
    float dragStartMouseY = 0.0f;
    float dragStartHeight = 0.0f;

    // 3D drag handle state
    float handleBaseWorld[3] = {0,0,0}; // centroid of selected profiles in world space
    bool handleVisible = false;

    char heightBuf[32] = "10.0";
    char offsetBuf[32] = "0.0";

    void reset() { *this = {}; }
};

enum class RevolvePhase : uint8_t {
    SelectingProfiles,
    SelectingAxis,
    Adjusting,
};

struct RevolveToolState : ProfileToolBase {
    EntityID axisLineID = NullID;
    float angleDeg = 360.0f;

    RevolvePhase phase = RevolvePhase::SelectingProfiles;

    char angleBuf[32] = "360.0";

    void reset() { *this = {}; }
};

// Loft tool: select one profile per sketch plane, then loft between them
struct LoftToolSection {
    int sketchPlaneIndex = -1;
    int profileIndex = -1;
    std::vector<ClosedProfile> detectedProfiles; // profiles available on that plane
    std::vector<ProfileRenderCache> renderCache;
};

struct LoftToolState : FeatureToolBase {
    std::vector<LoftToolSection> sections;
    bool solid = true;

    int activeSection = -1; // which section is being edited (-1 = adding new)

    void reset() { *this = {}; }
    bool canCommit() const { return sections.size() >= 2; }
};

// Boolean tool: pick two bodies to union or subtract
struct BooleanToolState {
    BooleanOperation operation = BooleanOperation::Union;
    int targetBodyIndex = -1;   // first picked body
    int toolBodyIndex = -1;     // second picked body
    Body3D previewBody;
    bool previewValid = false;
    bool previewDirty = false;
    uint32_t editingFeatureID = 0;

    void reset() { *this = {}; }
    bool hasTarget() const { return targetBodyIndex >= 0; }
    bool hasTool() const { return toolBodyIndex >= 0; }
    bool canCommit() const { return targetBodyIndex >= 0 && toolBodyIndex >= 0; }
};

// Hit test a 2D local position against detected profiles.
// Returns profile index or -1 if no hit.
// Uses cached tessellation if available (renderCache non-empty), falls back to computing.
int hitTestProfile(const Sketch& sketch, const std::vector<ClosedProfile>& profiles,
                   Point2D localPos,
                   const std::vector<ProfileRenderCache>& renderCache = {});

} // namespace shitcad
