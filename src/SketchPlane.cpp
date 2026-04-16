#include "SketchPlane.h"
#include <cmath>
#include <cstring>

namespace shitcad {

void mat4Multiply(float* out, const float* a, const float* b) {
    float tmp[16];
    for (int r = 0; r < 4; r++)
        for (int c = 0; c < 4; c++) {
            tmp[c * 4 + r] = 0;
            for (int k = 0; k < 4; k++)
                tmp[c * 4 + r] += a[k * 4 + r] * b[c * 4 + k];
        }
    memcpy(out, tmp, sizeof(tmp));
}

bool mat4Inverse(const float* m, float* inv) {
    float t[16];
    t[0]  =  m[5]*m[10]*m[15] - m[5]*m[11]*m[14] - m[9]*m[6]*m[15] + m[9]*m[7]*m[14] + m[13]*m[6]*m[11] - m[13]*m[7]*m[10];
    t[4]  = -m[4]*m[10]*m[15] + m[4]*m[11]*m[14] + m[8]*m[6]*m[15] - m[8]*m[7]*m[14] - m[12]*m[6]*m[11] + m[12]*m[7]*m[10];
    t[8]  =  m[4]*m[9]*m[15]  - m[4]*m[11]*m[13] - m[8]*m[5]*m[15] + m[8]*m[7]*m[13] + m[12]*m[5]*m[11] - m[12]*m[7]*m[9];
    t[12] = -m[4]*m[9]*m[14]  + m[4]*m[10]*m[13] + m[8]*m[5]*m[14] - m[8]*m[6]*m[13] - m[12]*m[5]*m[10] + m[12]*m[6]*m[9];
    t[1]  = -m[1]*m[10]*m[15] + m[1]*m[11]*m[14] + m[9]*m[2]*m[15] - m[9]*m[3]*m[14] - m[13]*m[2]*m[11] + m[13]*m[3]*m[10];
    t[5]  =  m[0]*m[10]*m[15] - m[0]*m[11]*m[14] - m[8]*m[2]*m[15] + m[8]*m[3]*m[14] + m[12]*m[2]*m[11] - m[12]*m[3]*m[10];
    t[9]  = -m[0]*m[9]*m[15]  + m[0]*m[11]*m[13] + m[8]*m[1]*m[15] - m[8]*m[3]*m[13] - m[12]*m[1]*m[11] + m[12]*m[3]*m[9];
    t[13] =  m[0]*m[9]*m[14]  - m[0]*m[10]*m[13] - m[8]*m[1]*m[14] + m[8]*m[2]*m[13] + m[12]*m[1]*m[10] - m[12]*m[2]*m[9];
    t[2]  =  m[1]*m[6]*m[15]  - m[1]*m[7]*m[14]  - m[5]*m[2]*m[15] + m[5]*m[3]*m[14] + m[13]*m[2]*m[7]  - m[13]*m[3]*m[6];
    t[6]  = -m[0]*m[6]*m[15]  + m[0]*m[7]*m[14]  + m[4]*m[2]*m[15] - m[4]*m[3]*m[14] - m[12]*m[2]*m[7]  + m[12]*m[3]*m[6];
    t[10] =  m[0]*m[5]*m[15]  - m[0]*m[7]*m[13]  - m[4]*m[1]*m[15] + m[4]*m[3]*m[13] + m[12]*m[1]*m[7]  - m[12]*m[3]*m[5];
    t[14] = -m[0]*m[5]*m[14]  + m[0]*m[6]*m[13]  + m[4]*m[1]*m[14] - m[4]*m[2]*m[13] - m[12]*m[1]*m[6]  + m[12]*m[2]*m[5];
    t[3]  = -m[1]*m[6]*m[11]  + m[1]*m[7]*m[10]  + m[5]*m[2]*m[11] - m[5]*m[3]*m[10] - m[9]*m[2]*m[7]   + m[9]*m[3]*m[6];
    t[7]  =  m[0]*m[6]*m[11]  - m[0]*m[7]*m[10]  - m[4]*m[2]*m[11] + m[4]*m[3]*m[10] + m[8]*m[2]*m[7]   - m[8]*m[3]*m[6];
    t[11] = -m[0]*m[5]*m[11]  + m[0]*m[7]*m[9]   + m[4]*m[1]*m[11] - m[4]*m[3]*m[9]  - m[8]*m[1]*m[7]   + m[8]*m[3]*m[5];
    t[15] =  m[0]*m[5]*m[10]  - m[0]*m[6]*m[9]   - m[4]*m[1]*m[10] + m[4]*m[2]*m[9]  + m[8]*m[1]*m[6]   - m[8]*m[2]*m[5];

    float det = m[0]*t[0] + m[1]*t[4] + m[2]*t[8] + m[3]*t[12];
    if (std::fabs(det) < 1e-12f) return false;

    float invDet = 1.0f / det;
    for (int i = 0; i < 16; i++) inv[i] = t[i] * invDet;
    return true;
}

void screenToRay(float sx, float sy, float vpX, float vpY, float vpW, float vpH,
                 const float view[16], const float proj[16],
                 float rayOrigin[3], float rayDir[3]) {
    float ndcX = 2.0f * (sx - vpX) / vpW - 1.0f;
    float ndcY = 1.0f - 2.0f * (sy - vpY) / vpH;

    float pvMat[16], invPV[16];
    mat4Multiply(pvMat, proj, view);
    mat4Inverse(pvMat, invPV);

    // Unproject near point (z = -1)
    auto unproj = [&](float nz, float out[3]) {
        float v[4] = {ndcX, ndcY, nz, 1.0f};
        float r[4];
        for (int i = 0; i < 4; i++)
            r[i] = invPV[i]*v[0] + invPV[4+i]*v[1] + invPV[8+i]*v[2] + invPV[12+i]*v[3];
        if (std::fabs(r[3]) > 1e-12f) {
            out[0] = r[0]/r[3]; out[1] = r[1]/r[3]; out[2] = r[2]/r[3];
        }
    };

    float nearPt[3], farPt[3];
    unproj(-1.0f, nearPt);
    unproj(1.0f, farPt);

    rayOrigin[0] = nearPt[0];
    rayOrigin[1] = nearPt[1];
    rayOrigin[2] = nearPt[2];

    float dx = farPt[0] - nearPt[0];
    float dy = farPt[1] - nearPt[1];
    float dz = farPt[2] - nearPt[2];
    float len = std::sqrt(dx*dx + dy*dy + dz*dz);
    if (len > 1e-12f) { dx /= len; dy /= len; dz /= len; }
    rayDir[0] = dx; rayDir[1] = dy; rayDir[2] = dz;
}

float computeApparentScale(const SketchPlane& plane, const float view[16],
                           const float proj[16], float vpW, float vpH) {
    float pvMat[16];
    mat4Multiply(pvMat, proj, view);

    auto project = [&](float wx, float wy, float wz, float& sx, float& sy) {
        float v[4] = {wx, wy, wz, 1.0f};
        float r[4];
        for (int i = 0; i < 4; i++)
            r[i] = pvMat[i]*v[0] + pvMat[4+i]*v[1] + pvMat[8+i]*v[2] + pvMat[12+i]*v[3];
        if (std::fabs(r[3]) > 1e-12f) {
            sx = (r[0]/r[3] * 0.5f + 0.5f) * vpW;
            sy = (0.5f - r[1]/r[3] * 0.5f) * vpH;
        }
    };

    float wx0, wy0, wz0, wx1, wy1, wz1;
    plane.localToWorld(0, 0, wx0, wy0, wz0);
    plane.localToWorld(1, 0, wx1, wy1, wz1);

    float sx0, sy0, sx1, sy1;
    project(wx0, wy0, wz0, sx0, sy0);
    project(wx1, wy1, wz1, sx1, sy1);

    float dx = sx1 - sx0, dy = sy1 - sy0;
    return std::sqrt(dx*dx + dy*dy);
}

} // namespace shitcad
