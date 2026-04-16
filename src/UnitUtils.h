#pragma once
#include <cstring>
#include <cstdio>
#include <cstdlib>
#include <cmath>
#include <string>

namespace shitcad {

// ---- Unit definitions ----

struct UnitInfo {
    const char* name;        // canonical display name
    const char* aliases[8];  // input aliases (lowercase, null-terminated)
    float toMm;              // multiply by this to convert to mm
};

inline const UnitInfo kUnits[] = {
    // Metric
    {"um",   {"um", "\xCE\xBCm", "micron", "microns", "micrometer", "micrometers", "micrometre", nullptr}, 0.001f},
    {"mm",   {"mm", "millimeter", "millimeters", "millimetre", "millimetres", nullptr, nullptr, nullptr}, 1.0f},
    {"cm",   {"cm", "centimeter", "centimeters", "centimetre", "centimetres", nullptr, nullptr, nullptr}, 10.0f},
    {"dm",   {"dm", "decimeter", "decimeters", "decimetre", "decimetres", nullptr, nullptr, nullptr}, 100.0f},
    {"m",    {"m", "meter", "meters", "metre", "metres", nullptr, nullptr, nullptr}, 1000.0f},
    {"km",   {"km", "kilometer", "kilometers", "kilometre", "kilometres", nullptr, nullptr, nullptr}, 1000000.0f},

    // Imperial / US customary
    {"thou", {"thou", "mil", "mils", nullptr, nullptr, nullptr, nullptr, nullptr}, 0.0254f},
    {"in",   {"in", "inch", "inches", "\"", nullptr, nullptr, nullptr, nullptr}, 25.4f},
    {"ft",   {"ft", "foot", "feet", "'", nullptr, nullptr, nullptr, nullptr}, 304.8f},
    {"yd",   {"yd", "yard", "yards", nullptr, nullptr, nullptr, nullptr, nullptr}, 914.4f},
    {"mi",   {"mi", "mile", "miles", nullptr, nullptr, nullptr, nullptr, nullptr}, 1609344.0f},
};
inline constexpr int kUnitCount = sizeof(kUnits) / sizeof(kUnits[0]);

// Parse a string like "10mm", "25.4 in", "1 foot", "10" (no unit = mm)
// Returns value in mm, and sets outUnit to the canonical unit name (empty if primary/mm)
inline float parseUnitInput(const char* input, std::string& outUnit, float& outInputValue) {
    const char* p = input;
    while (*p == ' ') p++;

    char* numEnd = nullptr;
    float numVal = std::strtof(p, &numEnd);
    if (numEnd == p) { outUnit.clear(); outInputValue = 0; return 0; }

    const char* unitStart = numEnd;
    while (*unitStart == ' ') unitStart++;

    if (*unitStart == '\0') {
        outUnit.clear();
        outInputValue = numVal;
        return numVal; // already in mm
    }

    // Lowercase the unit string for matching
    char unitBuf[32] = {};
    int ui = 0;
    for (const char* c = unitStart; *c && ui < 30; c++, ui++) {
        unitBuf[ui] = (*c >= 'A' && *c <= 'Z') ? (*c + 32) : *c;
    }
    while (ui > 0 && unitBuf[ui - 1] == ' ') unitBuf[--ui] = '\0';

    for (int i = 0; i < kUnitCount; i++) {
        for (int a = 0; kUnits[i].aliases[a]; a++) {
            if (strcmp(unitBuf, kUnits[i].aliases[a]) == 0) {
                outUnit = kUnits[i].name;
                outInputValue = numVal;
                return numVal * kUnits[i].toMm;
            }
        }
    }

    outUnit.clear();
    outInputValue = numVal;
    return numVal;
}

// Format dimension display: "25.4mm (1in)" or just "10mm" if input was mm
inline void formatDimensionText(char* buf, int bufSize, float valueMm,
                                const std::string& inputUnit, float inputValue) {
    if (inputUnit.empty() || inputUnit == "mm") {
        snprintf(buf, bufSize, "%.4gmm", valueMm);
    } else {
        snprintf(buf, bufSize, "%.4gmm (%.4g%s)", valueMm, inputValue, inputUnit.c_str());
    }
}

// Parse angle input: "45", "45deg", "45 degrees", "1.5rad" -> returns degrees
inline float parseAngleInput(const char* input) {
    const char* p = input;
    while (*p == ' ') p++;
    char* numEnd = nullptr;
    float numVal = std::strtof(p, &numEnd);
    if (numEnd == p) return -1;

    const char* unitStart = numEnd;
    while (*unitStart == ' ') unitStart++;

    if (*unitStart == '\0' || strcmp(unitStart, "deg") == 0 || strcmp(unitStart, "\xC2\xB0") == 0
        || strcmp(unitStart, "degree") == 0 || strcmp(unitStart, "degrees") == 0) {
        return numVal;
    }
    if (strcmp(unitStart, "rad") == 0 || strcmp(unitStart, "radian") == 0 || strcmp(unitStart, "radians") == 0) {
        return numVal * 180.0f / 3.14159265358979f;
    }
    return numVal;
}

// Format angle display: "45.0 degrees"
inline void formatAngleText(char* buf, int bufSize, float degrees) {
    snprintf(buf, bufSize, "%.4g\xC2\xB0", degrees);
}

// Project a 3D world point to 2D screen coordinates
inline bool worldToScreen(const float world[3], const float view[16], const float proj[16],
                          float vpW, float vpH, float& sx, float& sy) {
    float vx = view[0]*world[0] + view[4]*world[1] + view[8]*world[2]  + view[12];
    float vy = view[1]*world[0] + view[5]*world[1] + view[9]*world[2]  + view[13];
    float vz = view[2]*world[0] + view[6]*world[1] + view[10]*world[2] + view[14];
    float vw = view[3]*world[0] + view[7]*world[1] + view[11]*world[2] + view[15];
    float cx = proj[0]*vx + proj[4]*vy + proj[8]*vz  + proj[12]*vw;
    float cy = proj[1]*vx + proj[5]*vy + proj[9]*vz  + proj[13]*vw;
    float cw = proj[3]*vx + proj[7]*vy + proj[11]*vz + proj[15]*vw;
    if (std::fabs(cw) < 1e-6f) return false;
    if (cw < 0) return false;
    float ndcX = cx / cw;
    float ndcY = cy / cw;
    sx = (ndcX * 0.5f + 0.5f) * vpW;
    sy = (1.0f - (ndcY * 0.5f + 0.5f)) * vpH;
    return true;
}

} // namespace shitcad
