#pragma once
#include "Overlay2D.h" // Color32
#include <cstdint>

namespace shitcad {

// Profile detection backend selection
enum class ProfileDetectorBackend : uint8_t {
    Custom = 0,  // Half-edge tracer (original)
    OCCT = 1,    // BOPAlgo_BuilderFace (exact geometry)
};

struct Theme {
    // 3D viewport
    float bgColor[3];
    float gridMinor[3];
    float gridMajor[3];

    // Sketch
    float sketchLine[3];
    float sketchUnderconstrained[3];
    float sketchSelected[3];
    float sketchGridMinor;
    float sketchGridMajor;
    float toolPreview[4];
    float snapColor[4];
    float sketchProjected[3];

    // Dimensions
    Color32 dimLineColor;
    Color32 dimTextColor;
    Color32 dimBgColor;
    Color32 dimDrivenLineColor;
    Color32 dimDrivenTextColor;

    // Bodies
    float bodyColor[3];
    float edgeColor[3];

    static Theme dark();
    static Theme light();
};

// Global active theme pointer — set by Preferences::applyTheme()
const Theme& activeTheme();
Theme& activeThemeMut();

// Global profile detector backend setting
ProfileDetectorBackend activeProfileBackend();
void setActiveProfileBackend(ProfileDetectorBackend backend);

struct Preferences {
    bool lightMode = true;
    bool showWireframe = false;

    // User-adjustable colors and thicknesses
    float edgeColor[3] = {0.30f, 0.30f, 0.30f};
    float edgeThickness = 1.0f;
    float sketchLineColor[3] = {0.1f, 0.1f, 0.1f};
    float sketchLineThickness = 1.5f;
    float tangentSnapPx = 6.0f; // tangent snap distance in pixels

    // Profile detection backend
    ProfileDetectorBackend profileBackend = ProfileDetectorBackend::Custom;

    // Dimension label colors (RGBA 0-1)
    float dimLineCol[4] = {0.0f, 0.55f, 0.63f, 0.86f};
    float dimTextCol[4] = {0.08f, 0.08f, 0.08f, 0.94f};
    float dimBgCol[4] = {0.94f, 0.94f, 0.96f, 0.78f};

    // Constraint icon colors (RGBA 0-1)
    float conTextCol[4] = {0.39f, 0.24f, 0.63f, 0.78f};
    float conBgCol[4] = {1.0f, 1.0f, 1.0f, 0.63f};

    void applyTheme();
};

bool operator==(const Preferences& a, const Preferences& b);
inline bool operator!=(const Preferences& a, const Preferences& b) { return !(a == b); }

} // namespace shitcad
