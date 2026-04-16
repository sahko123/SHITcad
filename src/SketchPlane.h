#pragma once
#include "SketchData.h"
#include <string>
#include <cmath>
#include <cstring>

namespace shitcad {

using PlaneID = uint32_t;
constexpr PlaneID NullPlaneID = 0;

struct SketchPlane {
    PlaneID planeID = NullPlaneID; // stable ID (survives array reordering)
    float origin[3] = {0, 0, 0};
    float normal[3] = {0, 0, 1};
    float uAxis[3]  = {1, 0, 0};
    float vAxis[3]  = {0, 1, 0};

    Sketch sketch;
    std::string name;
    float color[4] = {0.3f, 0.3f, 0.8f, 0.12f};
    int sourceBodyIndex = -1;
    bool visible = true;       // reference plane quad visibility
    bool sketchVisible = true; // sketch geometry visibility (independent)
    bool isReferencePlane = false; // true for built-in + user-created offset planes

    void localToWorld(float lx, float ly, float& wx, float& wy, float& wz) const {
        wx = origin[0] + uAxis[0] * lx + vAxis[0] * ly;
        wy = origin[1] + uAxis[1] * lx + vAxis[1] * ly;
        wz = origin[2] + uAxis[2] * lx + vAxis[2] * ly;
    }

    void worldToLocal(float wx, float wy, float wz, float& lx, float& ly) const {
        float dx = wx - origin[0];
        float dy = wy - origin[1];
        float dz = wz - origin[2];
        lx = dx * uAxis[0] + dy * uAxis[1] + dz * uAxis[2];
        ly = dx * vAxis[0] + dy * vAxis[1] + dz * vAxis[2];
    }

    bool rayIntersect(const float rayOrig[3], const float rayDir[3],
                      float& hitLocalX, float& hitLocalY, float& t) const {
        float denom = rayDir[0]*normal[0] + rayDir[1]*normal[1] + rayDir[2]*normal[2];
        if (std::fabs(denom) < 1e-7f) return false;

        float dx = origin[0] - rayOrig[0];
        float dy = origin[1] - rayOrig[1];
        float dz = origin[2] - rayOrig[2];
        t = (dx*normal[0] + dy*normal[1] + dz*normal[2]) / denom;
        if (t < 0.0f) return false;

        float hitX = rayOrig[0] + rayDir[0] * t;
        float hitY = rayOrig[1] + rayDir[1] * t;
        float hitZ = rayOrig[2] + rayDir[2] * t;
        worldToLocal(hitX, hitY, hitZ, hitLocalX, hitLocalY);
        return true;
    }
};

// Math utilities
void mat4Multiply(float* out, const float* a, const float* b);
bool mat4Inverse(const float* m, float* out);
void screenToRay(float sx, float sy, float vpX, float vpY, float vpW, float vpH,
                 const float view[16], const float proj[16],
                 float rayOrigin[3], float rayDir[3]);
float computeApparentScale(const SketchPlane& plane, const float view[16],
                           const float proj[16], float vpW, float vpH);

} // namespace shitcad
