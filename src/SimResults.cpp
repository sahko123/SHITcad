#include "SimResults.h"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>

using json = nlohmann::json;
namespace fs = std::filesystem;

namespace shitcad {

const float kNoValueColour[3] = {0.35f, 0.35f, 0.38f};

const char* kResultVertSrc = R"(
#version 330 core
layout(location = 0) in vec3 aPos;
layout(location = 1) in vec3 aNormal;
layout(location = 2) in vec3 aColor;
uniform mat4 uView;
uniform mat4 uProj;
out vec3 vWorldPos;
out vec3 vNormal;
out vec3 vColor;
void main() {
    vWorldPos = aPos;
    vNormal = aNormal;
    vColor = aColor;
    gl_Position = uProj * uView * vec4(aPos, 1.0);
}
)";

// Mesh-shader lighting, softened so the colour map stays readable, plus an
// optional cut plane: fragments on the positive side are discarded.
const char* kResultFragSrc = R"(
#version 330 core
in vec3 vWorldPos;
in vec3 vNormal;
in vec3 vColor;
uniform vec3 uEyePos;
uniform vec3 uLightDir;
uniform float uClipOn;
uniform vec3 uClipNormal;
uniform float uClipOffset;
out vec4 FragColor;
void main() {
    if (uClipOn > 0.5 && dot(vWorldPos, uClipNormal) > uClipOffset) discard;
    vec3 N = normalize(vNormal);
    vec3 V = normalize(uEyePos - vWorldPos);
    if (dot(N, V) < 0.0) N = -N;   // two-sided: the inside of a cut vessel is lit too
    float diff = max(dot(N, normalize(uLightDir)), 0.0) * 0.35;
    float fill = max(dot(N, V), 0.0) * 0.35;
    vec3 color = vColor * (0.40 + diff + fill);
    FragColor = vec4(color, 1.0);
}
)";

int ResultMesh::fieldIndex(const std::string& name) const {
    for (int i = 0; i < (int)fields.size(); i++) if (fields[i].name == name) return i;
    return -1;
}

static bool hexColour(const std::string& s, std::array<float, 3>& out) {
    unsigned r, g, b;
    if (s.size() != 7 || s[0] != '#' || std::sscanf(s.c_str() + 1, "%2x%2x%2x", &r, &g, &b) != 3) return false;
    out = {r / 255.0f, g / 255.0f, b / 255.0f};
    return true;
}

static double unitToMm(const std::string& u) {
    if (u == "m") return 1000.0;
    if (u == "mm") return 1.0;
    if (u == "cm") return 10.0;
    return 0.0;
}

bool loadResultMesh(const std::string& jsonPath, ResultMesh& out, std::string& error) {
    try {
        std::ifstream jf(fs::path(jsonPath), std::ios::binary);
        if (!jf) { error = "Cannot open " + jsonPath; return false; }
        json h = json::parse(jf);
        if (h.value("format", "") != "cipsim-trimesh") {
            error = "Not a cip-sim result file: " + jsonPath;
            return false;
        }
        if (h.value("version", 0) != 1) {
            error = "Result file version " + std::to_string(h.value("version", 0)) + " is not supported";
            return false;
        }
        const double scale = unitToMm(h.value("units", ""));
        if (scale == 0.0) { error = "Unknown result units '" + h.value("units", "") + "'"; return false; }

        const size_t n = h.at("triangles").get<size_t>();
        const fs::path binPath = fs::path(jsonPath).parent_path() / h.at("bin").get<std::string>();
        const uintmax_t bytes = h.at("bytes").get<uintmax_t>();
        std::error_code ec;
        if (fs::file_size(binPath, ec) < bytes || ec) {
            error = "Result data file is missing or truncated: " + binPath.string();
            return false;
        }
        std::ifstream bf(binPath, std::ios::binary);
        std::vector<float> raw((size_t)(bytes / 4));
        bf.read((char*)raw.data(), (std::streamsize)(raw.size() * 4)); // little-endian float32, as x64 is

        ResultMesh m;
        m.triangles = n;
        const auto& pos = h.at("positions");
        const size_t pOff = pos.at("offset").get<size_t>() / 4, pCount = pos.at("count").get<size_t>();
        if (pCount != n * 9 || pOff + pCount > raw.size()) { error = "Result positions do not match triangle count"; return false; }
        m.positionsMm.resize(pCount);
        for (int k = 0; k < 3; k++) { m.boundsMin[k] = 1e30f; m.boundsMax[k] = -1e30f; }
        for (size_t i = 0; i < pCount; i++) {
            const float v = (float)(raw[pOff + i] * scale);
            m.positionsMm[i] = v;
            int k = (int)(i % 3);
            m.boundsMin[k] = std::min(m.boundsMin[k], v);
            m.boundsMax[k] = std::max(m.boundsMax[k], v);
        }

        for (const auto& f : h.at("fields")) {
            ResultField rf;
            rf.name = f.at("name").get<std::string>();
            rf.categorical = f.value("kind", "") == "categorical";
            rf.unit = f.value("unit", "");
            const size_t off = f.at("offset").get<size_t>() / 4, count = f.at("count").get<size_t>();
            if (count != n || off + count > raw.size()) { error = "Field '" + rf.name + "' has the wrong size"; return false; }
            rf.values.assign(raw.begin() + off, raw.begin() + off + count);
            for (const auto& l : f.value("labels", json::array())) rf.labels.push_back(l.get<std::string>());
            for (const auto& c : f.value("colours", json::array())) {
                std::array<float, 3> rgb;
                if (hexColour(c.get<std::string>(), rgb)) rf.colours.push_back(rgb);
            }
            rf.minValue = f.value("min", 0.0f);
            rf.maxValue = f.value("max", 0.0f);
            rf.p05 = f.value("p05", 0.0f);
            rf.p95 = f.value("p95", 0.0f);
            m.fields.push_back(std::move(rf));
        }
        for (const auto& s : h.value("surfaces", json::array())) m.surfaceNames.push_back(s.value("name", ""));
        out = std::move(m);
        return true;
    } catch (const std::exception& e) {
        error = std::string("Invalid result file: ") + e.what();
        return false;
    }
}

void rampColour(float t, float rgb[3]) {
    // Viridis, 5 stops: readable in greyscale and for red-green colour blindness.
    static const float stops[5][3] = {
        {0.267f, 0.005f, 0.329f}, {0.231f, 0.322f, 0.545f}, {0.129f, 0.569f, 0.549f},
        {0.369f, 0.788f, 0.384f}, {0.993f, 0.906f, 0.144f}};
    t = std::clamp(t, 0.0f, 1.0f) * 4.0f;
    int i = std::min((int)t, 3);
    float f = t - (float)i;
    for (int k = 0; k < 3; k++) rgb[k] = stops[i][k] + (stops[i + 1][k] - stops[i][k]) * f;
}

void categoryColour(const ResultField& f, int k, float rgb[3]) {
    static const float palette[8][3] = {
        {0.40f, 0.65f, 0.90f}, {0.95f, 0.60f, 0.25f}, {0.45f, 0.80f, 0.45f}, {0.85f, 0.40f, 0.55f},
        {0.65f, 0.55f, 0.85f}, {0.85f, 0.80f, 0.35f}, {0.40f, 0.80f, 0.80f}, {0.70f, 0.70f, 0.70f}};
    if (k >= 0 && k < (int)f.colours.size()) {
        for (int j = 0; j < 3; j++) rgb[j] = f.colours[k][j];
        return;
    }
    k = ((k % 8) + 8) % 8;
    for (int j = 0; j < 3; j++) rgb[j] = palette[k][j];
}

void colourByField(const ResultMesh& mesh, int fieldIndex, std::vector<float>& rgb) {
    rgb.assign(mesh.triangles * 3, 0.7f);
    if (fieldIndex < 0 || fieldIndex >= (int)mesh.fields.size()) return;
    const ResultField& f = mesh.fields[fieldIndex];

    const float hi = f.p95 > 0 ? f.p95 : (f.maxValue > 0 ? f.maxValue : 1.0f);
    for (size_t t = 0; t < mesh.triangles; t++) {
        const float v = f.values[t];
        float c[3];
        if (f.categorical) {
            categoryColour(f, (int)std::lround(v), c);
        } else if (!(v > 0.0f)) {
            for (int j = 0; j < 3; j++) c[j] = kNoValueColour[j];
        } else {
            rampColour(v / hi, c);
        }
        for (int j = 0; j < 3; j++) rgb[t * 3 + j] = c[j];
    }
}

// ---- events ---------------------------------------------------------------------

std::string eventKind(const std::string& line) {
    try {
        json j = json::parse(line);
        return j.value("event", "");
    } catch (...) {
        return "";
    }
}

std::string eventMessage(const std::string& line) {
    try {
        json j = json::parse(line);
        return j.value("message", j.dump());
    } catch (...) {
        return line;
    }
}

static CoverageRow row(const std::string& name, const json& s) {
    CoverageRow r;
    r.name = name;
    r.scored = s.value("scored", true);
    r.areaM2 = s.value("area_m2", 0.0);
    r.directPct = s.value("direct_pct", 0.0);
    r.splashPct = s.value("splash_pct", 0.0);
    r.dryPct = s.value("dry_pct", 0.0);
    r.fluxMean = s.value("flux_mean", 0.0);
    r.fluxP05 = s.value("flux_p05", 0.0);
    return r;
}

bool parseResultEvent(const std::string& line, RunSummary& out, std::string& error) {
    try {
        json j = json::parse(line);
        if (j.value("event", "") != "result") { error = "not a result event"; return false; }
        RunSummary s;
        s.overall = row("All walls", j.at("stats"));
        for (const auto& [name, st] : j.at("per_surface").items()) s.surfaces.push_back(row(name, st));
        s.fluxTrustworthy = j.value("flux_trustworthy", false);
        s.seconds = j.value("seconds", 0.0);
        s.viewerJson = j.at("files").at("viewer").get<std::string>();
        s.vtp = j.at("files").value("vtp", "");
        out = std::move(s);
        return true;
    } catch (const std::exception& e) {
        error = std::string("Could not read the result: ") + e.what();
        return false;
    }
}

} // namespace shitcad
