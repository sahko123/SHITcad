#pragma once
#include <array>
#include <map>
#include <string>
#include <vector>

namespace shitcad {

// ---- Result mesh (cip-sim "cipsim-trimesh" format, see cip-sim/cipsim/viewer.py) ----

struct ResultField {
    std::string name;
    bool categorical = false;
    std::string unit;
    std::vector<std::string> labels;
    // Categorical display colours by category index. An entry that failed to
    // parse keeps its slot with a negative red channel, so later categories are
    // not shifted onto the wrong meaning.
    std::vector<std::array<float, 3>> colours;
    float minValue = 0, maxValue = 0, p05 = 0, p95 = 0;
    int noData = 0;                           // values that are not a measurement
    std::vector<float> values;                // one per triangle
};

struct ResultMesh {
    size_t triangles = 0;
    std::vector<float> positionsMm;  // triangles * 9, converted from the file's units
    std::vector<ResultField> fields;
    std::vector<std::string> surfaceNames;
    std::vector<bool> surfaceScored;  // parallel to surfaceNames
    float boundsMin[3] = {0, 0, 0};
    float boundsMax[3] = {0, 0, 0};

    int fieldIndex(const std::string& name) const;
};

bool loadResultMesh(const std::string& jsonPath, ResultMesh& out, std::string& error);

// One colour (RGB 0-1) per triangle for a field. Categorical fields use their
// own colours (or a fixed palette); continuous fields map 0..p95 onto a ramp.
//
// Three cases are deliberately NOT the ramp, because each means "this is not a
// measurement" and painting them as one misleads:
//   * no data (NaN)                       -> grey
//   * zero                                -> grey
//   * a flux field on a face that was hit by no forward ray -> amber:
//     under-sampled, not dry. (Needs `direct_rays` and `reach` in the file.)
// Triangles belonging to unscored surfaces (caps) are muted, since the
// percentages quoted beside the legend count scored wall only.
void colourByField(const ResultMesh& mesh, int fieldIndex, std::vector<float>& rgbPerTriangle);

// Continuous ramp, t in [0, 1]. Exposed for the legend.
void rampColour(float t, float rgb[3]);
// Colour of category k: the field's own colour if it has one, else a palette.
void categoryColour(const ResultField& field, int k, float rgb[3]);

// GLSL for drawing a result mesh (position, normal, colour; optional cut plane).
extern const char* kResultVertSrc;
extern const char* kResultFragSrc;
extern const float kNoValueColour[3];
extern const float kUnsampledColour[3];

// ---- Run summary (the CLI's "result" event) --------------------------------------

struct CoverageRow {
    std::string name;
    bool scored = true;
    double areaM2 = 0, directPct = 0, splashPct = 0, dryPct = 0;
    double fluxMean = 0, fluxP05 = 0;
};

struct RunSummary {
    CoverageRow overall;                 // scored surfaces combined
    std::vector<CoverageRow> surfaces;
    bool fluxTrustworthy = false;
    double seconds = 0;
    std::string viewerJson, vtp;
};

// Parse a JSON-lines event. Returns the "event" value ("" if not JSON).
std::string eventKind(const std::string& line);
// Human-readable text for progress/warning/error events.
std::string eventMessage(const std::string& line);
bool parseResultEvent(const std::string& line, RunSummary& out, std::string& error);

} // namespace shitcad
