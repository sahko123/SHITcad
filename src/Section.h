#pragma once
#include "ShaderProgram.h"

namespace shitcad {

// A section (cut) plane through the whole scene: imported meshes, solid bodies
// and simulation results all honour it, so the inside of a closed vessel can be
// seen without hiding anything.
//
// Fragments on the positive side of the plane are discarded by the shaders;
// `cap` then fills the opening so the cut reads as solid rather than hollow.
struct SectionPlane {
    bool enabled = false;
    int axis = 1;            // 0 X, 1 Y (up in this app), 2 Z
    float position = 0.0f;   // mm along that axis
    bool flip = false;       // keep the other half instead
    bool cap = true;

    void normal(float out[3]) const {
        out[0] = out[1] = out[2] = 0.0f;
        out[axis] = flip ? -1.0f : 1.0f;
    }
    float offset() const { return flip ? -position : position; }
    // True when a point is cut away (matches what the shaders discard).
    bool cuts(const float p[3]) const {
        if (!enabled) return false;
        float n[3];
        normal(n);
        return p[0] * n[0] + p[1] * n[1] + p[2] * n[2] > offset();
    }
};

// Set the clip uniforms on a shader that declares them. Passing nullptr (or a
// disabled plane) turns clipping off - always call it, because uniforms persist
// on the program between draws and a stale one would cut the wrong thing.
inline void applyClip(const ShaderProgram& shader, const SectionPlane* plane) {
    if (plane && plane->enabled) {
        float n[3];
        plane->normal(n);
        shader.setFloat("uClipOn", 1.0f);
        shader.setVec3("uClipNormal", n[0], n[1], n[2]);
        shader.setFloat("uClipOffset", plane->offset());
    } else {
        shader.setFloat("uClipOn", 0.0f);
    }
}

} // namespace shitcad
