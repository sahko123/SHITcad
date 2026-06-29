#pragma once

namespace shitcad {

// ─── Math ────────────────────────────────────────────────────────────
constexpr float kPi    = 3.14159265358979f;
constexpr float kTwoPi = 2.0f * kPi;
constexpr float kDegToRad = kPi / 180.0f;
constexpr float kRadToDeg = 180.0f / kPi;

// ─── Constraint solver ──────────────────────────────────────────────
constexpr int   kSolverMaxIterations     = 40;
constexpr float kSolverConvergenceTol    = 1e-6f;   // per-constraint convergence threshold
constexpr float kSolverDistanceTol       = 1e-4f;   // general distance comparison epsilon
constexpr int   kProjectedPointRefCount  = 10000;    // ref count for immovable projected points

// ─── Profile detection & tessellation ───────────────────────────────
constexpr int   kCircleTessSteps         = 72;       // subdivisions for full circle tessellation
constexpr float kArcTessDegreesPerStep   = 5.0f;     // degrees per subdivision step for arcs
constexpr int   kEllipseSampleCount      = 64;       // sample count for ellipse rendering/hit-test
constexpr int   kSplineSampleCount       = 64;       // sample count for spline rendering/hit-test

// ─── Preview throttle ───────────────────────────────────────────────
constexpr int   kPreviewThrottleMs       = 50;       // minimum ms between preview rebuilds

// ─── Geometry tolerances ────────────────────────────────────────────
constexpr float kDegenerateLen           = 1e-6f;    // length below which geometry is degenerate
constexpr float kMinLineLength           = 0.001f;   // minimum line/radius to create

// ─── Rendering helpers ──────────────────────────────────────────────
// Explicit double→float narrowing for rendering/UI call sites
inline float f(double x) { return static_cast<float>(x); }

} // namespace shitcad
