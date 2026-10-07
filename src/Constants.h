#pragma once
#include <cmath>

namespace shitcad {

// ─── Math ────────────────────────────────────────────────────────────
// Float constants for rendering/UI code, double ones (…D) for solver and sketch geometry.
constexpr float kPi       = 3.14159265358979f;
constexpr float kTwoPi    = 2.0f * kPi;
constexpr float kDegToRad = kPi / 180.0f;
constexpr float kRadToDeg = 180.0f / kPi;
constexpr double kPiD       = 3.14159265358979323846;
constexpr double kTwoPiD    = 2.0 * kPiD;
constexpr double kDegToRadD = kPiD / 180.0;
constexpr double kRadToDegD = 180.0 / kPiD;

// Angle wrapped into [0, 2pi).
inline double wrap2Pi(double a) {
    a = std::fmod(a, kTwoPiD);
    return a < 0.0 ? a + kTwoPiD : a;
}

// Counter-clockwise sweep from `start` to `end`, in (0, 2pi]: equal angles are a full turn.
inline double ccwSweep(double start, double end) {
    const double s = end - start;
    return s <= 0.0 ? s + kTwoPiD : s;
}

// ─── Constraint solver ──────────────────────────────────────────────
constexpr int   kSolverMaxIterations     = 40;
constexpr float kSolverConvergenceTol    = 1e-6f;   // per-constraint convergence threshold
constexpr int   kProjectedPointRefCount  = 10000;    // ref count for immovable projected points
// Adaptive under-relaxation: the solver takes full Gauss-Seidel steps while the residual is
// shrinking and halves the step whenever a pass makes things worse, so that mutually
// contradictory constraints settle on a compromise instead of ping-ponging until the
// iteration cap. This is the floor for that step factor.
constexpr double kSolverMinRelax         = 0.15;

// ─── Sketch geometry ────────────────────────────────────────────────
// Arcs are stored CCW with sweep in (0, 2pi]. Sweeps are clamped to this floor rather than
// being allowed to wrap past zero and re-enter as a near-full arc.
constexpr double kMinArcSweep            = 1e-4;    // radians

// ─── Profile detection & tessellation ───────────────────────────────
constexpr int   kCircleTessSteps         = 72;       // subdivisions for full circle tessellation
constexpr float kArcTessDegreesPerStep   = 5.0f;     // degrees per subdivision step for arcs
constexpr int   kEllipseSampleCount      = 64;       // sample count for ellipse rendering/hit-test
constexpr int   kSplineSampleCount       = 64;       // sample count for spline rendering/hit-test

// ─── Preview throttle ───────────────────────────────────────────────
constexpr int   kPreviewThrottleMs       = 50;       // minimum ms between preview rebuilds

// ─── Geometry tolerances ────────────────────────────────────────────
constexpr float kMinLineLength           = 0.001f;   // minimum line/radius to create

// ─── Rendering helpers ──────────────────────────────────────────────
// Explicit double→float narrowing for rendering/UI call sites
inline float f(double x) { return static_cast<float>(x); }

} // namespace shitcad
