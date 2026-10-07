#include "Viewport3D.h"
#include "Constants.h"
#include "Preferences.h"
#include "Section.h"
#include <cstring>
#include <vector>
#include <algorithm>

namespace shitcad {

// ---- Math helpers ----


static void mat4Identity(float* m) {
    memset(m, 0, 16 * sizeof(float));
    m[0] = m[5] = m[10] = m[15] = 1.0f;
}

void makePerspective(float* out, float fovDeg, float aspect, float nearP, float farP) {
    memset(out, 0, 16 * sizeof(float));
    float f = 1.0f / std::tan(fovDeg * kDegToRad * 0.5f);
    out[0] = f / aspect;
    out[5] = f;
    out[10] = (farP + nearP) / (nearP - farP);
    out[11] = -1.0f;
    out[14] = (2.0f * farP * nearP) / (nearP - farP);
}

void makeOrthographic(float* out, float halfH, float aspect, float nearP, float farP) {
    memset(out, 0, 16 * sizeof(float));
    float halfW = halfH * aspect;
    out[0]  = 1.0f / halfW;
    out[5]  = 1.0f / halfH;
    out[10] = 2.0f / (nearP - farP);
    out[14] = (farP + nearP) / (nearP - farP);
    out[15] = 1.0f;
}

// ---- OrbitCamera ----

void OrbitCamera::setOrthographic(bool on) {
    autoOrtho = false;
    if (orthographic == on) return;
    // Visible half-height at the target: `distance` in ortho,
    // distance * tan(fov / 2) in perspective.
    const float k = std::tan(45.0f * kDegToRad * 0.5f);
    distance = on ? distance * k : distance / k;
    orthographic = on;
}

void OrbitCamera::orbit(float dx, float dy) {
    if (autoOrtho && (dx != 0.0f || dy != 0.0f)) setOrthographic(false);
    yaw -= dx * 0.3f;
    pitch += dy * 0.3f;
}

void OrbitCamera::pan(float dx, float dy, float viewportW, float viewportH) {
    (void)viewportW;
    float scale;
    if (orthographic) {
        // In ortho, visible height = distance (used as ortho half-height)
        scale = 2.0f * distance / viewportH;
    } else {
        float fovRad = 45.0f * kDegToRad;
        scale = 2.0f * distance * std::tan(fovRad * 0.5f) / viewportH;
    }

    float yawR = yaw * kDegToRad;
    float pitchR = pitch * kDegToRad;

    // Camera right vector
    float rightX = std::cos(yawR);
    float rightZ = -std::sin(yawR);

    // Camera up vector
    float upX = -std::sin(pitchR) * std::sin(yawR);
    float upY = std::cos(pitchR);
    float upZ = -std::sin(pitchR) * std::cos(yawR);

    // drag right → scene moves right → target moves in -right direction
    // drag up   → scene moves up   → target moves in +up direction
    targetX += (-dx * rightX + dy * upX) * scale;
    targetY += dy * upY * scale;
    targetZ += (-dx * rightZ + dy * upZ) * scale;
}

void OrbitCamera::zoom(float delta) {
    distance *= std::pow(1.1f, -delta);
    distance = std::clamp(distance, 0.5f, 1'000'000.0f);
}

void OrbitCamera::getEyePosition(float* out) const {
    float yawR = yaw * kDegToRad;
    float pitchR = pitch * kDegToRad;
    out[0] = targetX + distance * std::cos(pitchR) * std::sin(yawR);
    out[1] = targetY + distance * std::sin(pitchR);
    out[2] = targetZ + distance * std::cos(pitchR) * std::cos(yawR);
}

void OrbitCamera::getViewMatrix(float* out) const {
    float eye[3];
    getEyePosition(eye);

    // Forward direction (eye → target)
    float fx = targetX - eye[0];
    float fy = targetY - eye[1];
    float fz = targetZ - eye[2];
    float flen = std::sqrt(fx*fx + fy*fy + fz*fz);
    fx /= flen; fy /= flen; fz /= flen;

    // Right vector derived from yaw (always horizontal, no flip)
    float yawR = yaw * kDegToRad;
    float sx = std::cos(yawR);
    float sy = 0.0f;
    float sz = -std::sin(yawR);

    // up = right x forward
    float ux = sy * fz - sz * fy;
    float uy = sz * fx - sx * fz;
    float uz = sx * fy - sy * fx;

    mat4Identity(out);
    out[0] = sx;  out[4] = sy;  out[8]  = sz;
    out[1] = ux;  out[5] = uy;  out[9]  = uz;
    out[2] = -fx; out[6] = -fy; out[10] = -fz;

    out[12] = -(sx * eye[0] + sy * eye[1] + sz * eye[2]);
    out[13] = -(ux * eye[0] + uy * eye[1] + uz * eye[2]);
    out[14] = (fx * eye[0] + fy * eye[1] + fz * eye[2]);
}

int OrbitCamera::viewAxis() const {
    const float yawR = yaw * kDegToRad, pitchR = pitch * kDegToRad;
    const float dir[3] = {std::cos(pitchR) * std::sin(yawR), std::sin(pitchR),
                          std::cos(pitchR) * std::cos(yawR)};
    const float kAligned = std::cos(1.0f * kDegToRad);
    for (int a = 0; a < 3; a++)
        if (std::fabs(dir[a]) >= kAligned) return a;
    return -1;
}

void OrbitCamera::getProjection(float* out, float aspect) const {
    // Scale near/far with distance so the depth ratio stays ~200000:1
    // at any zoom level — safe for a 24-bit depth buffer without z-fighting.
    float nearP = std::max(0.1f, distance * 0.001f);
    float farP  = distance * 200.0f + 5000.0f;
    if (orthographic) {
        float depthRange = farP;
        makeOrthographic(out, distance, aspect, -depthRange, depthRange);
    } else {
        makePerspective(out, 45.0f, aspect, nearP, farP);
    }
}

// ---- Shaders ----

static const char* kMeshVertSrc = R"(
#version 330 core
layout(location = 0) in vec3 aPos;
layout(location = 1) in vec3 aNormal;

uniform mat4 uView;
uniform mat4 uProj;

out vec3 vWorldPos;
out vec3 vNormal;

void main() {
    vWorldPos = aPos;
    vNormal = aNormal;
    gl_Position = uProj * uView * vec4(aPos, 1.0);
}
)";

static const char* kMeshFragSrc = R"(
#version 330 core
in vec3 vWorldPos;
in vec3 vNormal;

uniform vec3 uEyePos;
uniform vec3 uLightDir;
uniform vec3 uColor;
uniform float uAlpha;
uniform float uClipOn;
uniform vec3 uClipNormal;
uniform float uClipOffset;

out vec4 FragColor;

void main() {
    if (uClipOn > 0.5 && dot(vWorldPos, uClipNormal) > uClipOffset) discard;
    vec3 N = normalize(vNormal);
    vec3 V = normalize(uEyePos - vWorldPos);

    // Key light (upper-right-forward)
    vec3 L1 = normalize(uLightDir);
    vec3 H1 = normalize(L1 + V);
    float diff1 = max(dot(N, L1), 0.0) * 0.55;
    float spec1 = pow(max(dot(N, H1), 0.0), 64.0) * 0.25;

    // Fill light (lower-left-back, softer, no specular)
    vec3 L2 = normalize(vec3(-0.5, 0.3, -0.4));
    float diff2 = max(dot(N, L2), 0.0) * 0.25;

    // Two-sided lighting for back faces
    float diffBack = max(dot(-N, L1), 0.0) * 0.25;

    // Hemisphere ambient: warm above, cool below
    float hemi = dot(N, vec3(0.0, 1.0, 0.0)) * 0.5 + 0.5;
    vec3 ambient = mix(vec3(0.10, 0.10, 0.13), vec3(0.20, 0.20, 0.22), hemi);

    // Fresnel rim for silhouette definition
    float fresnel = pow(1.0 - max(dot(N, V), 0.0), 3.0) * 0.12;

    vec3 color = uColor * (ambient + diff1 + diff2 + diffBack) + vec3(spec1 + fresnel);
    FragColor = vec4(color, uAlpha);
}
)";

static const char* kGridVertSrc = R"(
#version 330 core
layout(location = 0) in vec3 aPos;
layout(location = 1) in vec4 aColor;

uniform mat4 uView;
uniform mat4 uProj;

out vec4 vColor;
out vec3 vWorldPos;

void main() {
    vColor = aColor;
    vWorldPos = aPos;
    gl_Position = uProj * uView * vec4(aPos, 1.0);
}
)";

static const char* kGridFragSrc = R"(
#version 330 core
in vec4 vColor;
in vec3 vWorldPos;
uniform float uClipOn;
uniform vec3 uClipNormal;
uniform float uClipOffset;
out vec4 FragColor;

void main() {
    if (uClipOn > 0.5 && dot(vWorldPos, uClipNormal) > uClipOffset) discard;
    FragColor = vColor;
}
)";

// One triangle covering the screen; vNdc is the position in normalized
// device coordinates.
static const char* kBackgroundVertSrc = R"(
#version 330 core
out vec2 vNdc;

void main() {
    vec2 p = vec2((gl_VertexID << 1) & 2, gl_VertexID & 2) * 2.0 - 1.0;
    vNdc = p;
    gl_Position = vec4(p, 0.0, 1.0);
}
)";

static const char* kBackgroundFragSrc = R"(
#version 330 core
in vec2 vNdc;

uniform vec3 uRight;
uniform vec3 uUp;
uniform vec3 uForward;
uniform float uScaleX;   // tan(fov / 2) * aspect
uniform float uScaleY;   // tan(fov / 2)
uniform vec3 uZenith;
uniform vec3 uSkyHorizon;
uniform vec3 uGroundHorizon;
uniform vec3 uNadir;

out vec4 FragColor;

void main() {
    vec3 d = normalize(uForward + vNdc.x * uScaleX * uRight + vNdc.y * uScaleY * uUp);
    float y = d.y;   // the world is Y-up
    // One continuous gradient with no line at the horizon: the pale horizon
    // colour at y = 0 eases up into the zenith, and down into the ground
    // over a wide band before darkening towards the nadir. smoothstep eases
    // in and out of every stop, so no band has a visible edge.
    vec3 c;
    if (y >= 0.0) {
        c = mix(uSkyHorizon, uZenith, smoothstep(0.0, 0.85, y));
    } else {
        c = mix(uSkyHorizon, uGroundHorizon, smoothstep(0.0, 0.35, -y));
        c = mix(c, uNadir, smoothstep(0.2, 1.0, -y));
    }
    // Dither away the 8-bit banding of a slow gradient.
    float n = fract(sin(dot(gl_FragCoord.xy, vec2(12.9898, 78.233))) * 43758.5453);
    c += (n - 0.5) / 255.0;
    FragColor = vec4(c, 1.0);
}
)";

// ---- Viewport3D ----

bool Viewport3D::init() {
    if (!meshShader_.compile(kMeshVertSrc, kMeshFragSrc)) return false;
    if (!gridShader_.compile(kGridVertSrc, kGridFragSrc)) return false;
    if (!backgroundShader_.compile(kBackgroundVertSrc, kBackgroundFragSrc)) return false;
    glGenVertexArrays(1, &backgroundVAO_);
    buildGrid();
    return true;
}

void Viewport3D::drawBackground(const float* view, float aspect) {
    // The view matrix's rows are the camera's right, up and backward axes.
    // Spread over a 90-degree field rather than the camera's 45, so the
    // horizon stays on screen for most orbit angles (level views still put it
    // through the centre). Nothing drawn sits at infinity to disagree.
    const float t = std::tan(90.0f * kDegToRad * 0.5f);
    backgroundShader_.use();
    backgroundShader_.setVec3("uRight", view[0], view[4], view[8]);
    backgroundShader_.setVec3("uUp", view[1], view[5], view[9]);
    backgroundShader_.setVec3("uForward", -view[2], -view[6], -view[10]);
    backgroundShader_.setFloat("uScaleX", t * aspect);
    backgroundShader_.setFloat("uScaleY", t);
    const auto& th = activeTheme();
    backgroundShader_.setVec3("uZenith", th.skyZenith[0], th.skyZenith[1], th.skyZenith[2]);
    backgroundShader_.setVec3("uSkyHorizon", th.skyHorizon[0], th.skyHorizon[1], th.skyHorizon[2]);
    backgroundShader_.setVec3("uGroundHorizon", th.groundHorizon[0], th.groundHorizon[1], th.groundHorizon[2]);
    backgroundShader_.setVec3("uNadir", th.groundNadir[0], th.groundNadir[1], th.groundNadir[2]);

    const GLboolean depthTest = glIsEnabled(GL_DEPTH_TEST);
    const GLboolean cull = glIsEnabled(GL_CULL_FACE);
    GLboolean depthMask = GL_TRUE;
    glGetBooleanv(GL_DEPTH_WRITEMASK, &depthMask);
    glDisable(GL_DEPTH_TEST);
    glDisable(GL_CULL_FACE);
    glDepthMask(GL_FALSE);
    glBindVertexArray(backgroundVAO_);
    glDrawArrays(GL_TRIANGLES, 0, 3);
    glBindVertexArray(0);
    glDepthMask(depthMask);
    if (depthTest) glEnable(GL_DEPTH_TEST);
    if (cull) glEnable(GL_CULL_FACE);
}

void Viewport3D::shutdown() {
    if (backgroundVAO_) { glDeleteVertexArrays(1, &backgroundVAO_); backgroundVAO_ = 0; }
    if (gridVAO_) { glDeleteVertexArrays(1, &gridVAO_); gridVAO_ = 0; }
    if (gridVBO_) { glDeleteBuffers(1, &gridVBO_); gridVBO_ = 0; }
}

void Viewport3D::buildGrid() {
    // Empty dynamic buffer: drawGrid rebuilds the lines every frame, since
    // their spacing and extent follow the zoom.
    glGenVertexArrays(1, &gridVAO_);
    glGenBuffers(1, &gridVBO_);

    glBindVertexArray(gridVAO_);
    glBindBuffer(GL_ARRAY_BUFFER, gridVBO_);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 7 * sizeof(float), (void*)0);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, 7 * sizeof(float), (void*)(3 * sizeof(float)));
    glBindVertexArray(0);
}

void Viewport3D::drawGrid(const float* view, const float* proj, float viewportW, float viewportH) {
    const int normal = camera_.viewAxis();
    if (normal < 0 || viewportW < 1.0f || viewportH < 1.0f) return;

    // World size of a pixel at the orbit target.
    const float mmPerPx = camera_.orthographic
        ? 2.0f * camera_.distance / viewportH
        : 2.0f * camera_.distance * std::tan(45.0f * kDegToRad * 0.5f) / viewportH;

    const float axisColor[3][3] = {{0.7f, 0.2f, 0.2f}, {0.2f, 0.7f, 0.2f}, {0.2f, 0.2f, 0.7f}};
    const auto& theme = activeTheme();
    // The plane's two axes, in the order X, Y, Z.
    const int u = normal == 0 ? 1 : 0;
    const int v = normal == 2 ? 1 : 2;
    const float target[3] = {camera_.targetX, camera_.targetY, camera_.targetZ};

    // Cover the viewport around the target (generously, for perspective).
    const float half = 0.5f * std::sqrt(viewportW * viewportW + viewportH * viewportH) * mmPerPx
                       * (camera_.orthographic ? 1.1f : 3.0f);
    const float cu = target[u], cv = target[v];

    // Line colour: the theme's minor->major direction pushed further, so
    // the lines stay visible against the sky/ground.
    const float lineC = std::clamp(theme.gridMinor[0] + (theme.gridMajor[0] - theme.gridMinor[0]) * 1.8f, 0.0f, 1.0f);

    std::vector<float> verts;
    auto push = [&](float pu, float pv, const float* col, float a) {
        float p[3] = {};
        p[u] = pu; p[v] = pv;
        verts.insert(verts.end(), {p[0], p[1], p[2], col[0], col[1], col[2], a});
    };
    auto line = [&](float u0, float v0, float u1, float v1, const float* col, float a) {
        push(u0, v0, col, a); push(u1, v1, col, a);
    };

    // One level per power of ten. A level fades in as its lines spread from
    // kMinPx to kFullPx apart, so zooming crossfades between levels; the
    // lines every tenth step belong to the next coarser level.
    constexpr float kMinPx = 6.0f, kFullPx = 60.0f;
    constexpr int kLevels = 4;
    const float firstStep = std::pow(10.0f, std::ceil(std::log10(std::max(mmPerPx * kMinPx, 1e-6f))));
    for (int lvl = 0; lvl < kLevels; lvl++) {
        const float step = firstStep * std::pow(10.0f, (float)lvl);
        const float t = std::clamp((step / mmPerPx - kMinPx) / (kFullPx - kMinPx), 0.0f, 1.0f);
        const float alpha = 0.85f * t * t * (3.0f - 2.0f * t);
        if (alpha <= 0.0f) continue;
        const bool coarserDrawn = lvl + 1 < kLevels;
        const float col[3] = {lineC, lineC, lineC};
        auto family = [&](float centre, float across, bool alongU) {
            const long lo = (long)std::floor((centre - half) / step);
            const long hi = (long)std::ceil((centre + half) / step);
            for (long i = lo; i <= hi; i++) {
                if (i == 0) continue; // the axis is drawn below
                if (coarserDrawn && i % 10 == 0) continue;
                const float pos = (float)i * step;
                if (alongU) line(pos, across - half, pos, across + half, col, alpha);
                else        line(across - half, pos, across + half, pos, col, alpha);
            }
        };
        family(cu, cv, true);
        family(cv, cu, false);
    }
    // The plane's axes in their colours: X red, Y green, Z blue
    line(cu - half, 0, cu + half, 0, axisColor[u], 1.0f);
    line(0, cv - half, 0, cv + half, axisColor[v], 1.0f);

    gridShader_.use();
    gridShader_.setMat4("uView", view);
    gridShader_.setMat4("uProj", proj);
    applyClip(gridShader_, nullptr); // the ground grid is never sectioned

    GLboolean depthMask = GL_TRUE;
    glGetBooleanv(GL_DEPTH_WRITEMASK, &depthMask);
    const GLboolean blendWas = glIsEnabled(GL_BLEND);
    glDepthMask(GL_FALSE);
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glBindVertexArray(gridVAO_);
    glBindBuffer(GL_ARRAY_BUFFER, gridVBO_);
    glBufferData(GL_ARRAY_BUFFER, verts.size() * sizeof(float), verts.data(), GL_STREAM_DRAW);
    glDrawArrays(GL_LINES, 0, (GLsizei)(verts.size() / 7));
    glBindVertexArray(0);
    if (!blendWas) glDisable(GL_BLEND);
    glDepthMask(depthMask);
}

void Viewport3D::handleInput(const InputFrame& in, float canvasX, float canvasY, float canvasW, float canvasH) {
    // Don't handle input when a panel wants the mouse (e.g., scrolling in a panel)

    // Check if mouse is within the viewport area
    bool hovered = in.mouseX >= canvasX && in.mouseX < canvasX + canvasW &&
                   in.mouseY >= canvasY && in.mouseY < canvasY + canvasH;
    if (!hovered) return;

    // Middle mouse: orbit (or pan with shift)
    if (in.dragging(MouseButton::Middle, 0.0f)) {
        if (in.shift) {
            camera_.pan(in.mouseDX, in.mouseDY, canvasW, canvasH);
        } else {
            camera_.orbit(in.mouseDX, in.mouseDY);
        }
    }

    // Scroll: zoom
    if (in.wheel != 0.0f) {
        camera_.zoom(in.wheel);
    }

}

void Viewport3D::render(float x, float y, float w, float h) {
    glViewport((int)x, (int)y, (int)w, (int)h);

    glEnable(GL_DEPTH_TEST);
    const auto& bg = activeTheme().bgColor;
    glClearColor(bg[0], bg[1], bg[2], 1.0f);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    float view[16], proj[16];
    camera_.getViewMatrix(view);
    float aspect = (h > 0) ? w / h : 1.0f;
    camera_.getProjection(proj, aspect);

    drawBackground(view, aspect);
    drawGrid(view, proj, w, h);

    glDisable(GL_DEPTH_TEST);
}

void Viewport3D::drawGroundGrid(const float* view, const float* proj, float viewportW, float viewportH) {
    drawGrid(view, proj, viewportW, viewportH);
}

} // namespace shitcad
