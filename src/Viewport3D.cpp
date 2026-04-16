#include "Viewport3D.h"
#include "Preferences.h"
#include <imgui.h>
#include <cstring>
#include <vector>
#include <algorithm>

namespace shitcad {

// ---- Math helpers ----

static constexpr float kPi = 3.14159265358979323846f;
static float toRad(float deg) { return deg * kPi / 180.0f; }

static void mat4Identity(float* m) {
    memset(m, 0, 16 * sizeof(float));
    m[0] = m[5] = m[10] = m[15] = 1.0f;
}

void makePerspective(float* out, float fovDeg, float aspect, float nearP, float farP) {
    memset(out, 0, 16 * sizeof(float));
    float f = 1.0f / std::tan(toRad(fovDeg) * 0.5f);
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

void OrbitCamera::orbit(float dx, float dy) {
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
        float fovRad = toRad(45.0f);
        scale = 2.0f * distance * std::tan(fovRad * 0.5f) / viewportH;
    }

    float yawR = toRad(yaw);
    float pitchR = toRad(pitch);

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
    distance = std::clamp(distance, 1.0f, 10000.0f);
}

void OrbitCamera::getEyePosition(float* out) const {
    float yawR = toRad(yaw);
    float pitchR = toRad(pitch);
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
    float yawR = toRad(yaw);
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

void OrbitCamera::getProjection(float* out, float aspect) const {
    if (orthographic) {
        // Use a large symmetric depth range so nothing clips
        float depthRange = 5000.0f;
        makeOrthographic(out, distance, aspect, -depthRange, depthRange);
    } else {
        makePerspective(out, 45.0f, aspect, 0.1f, 5000.0f);
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

out vec4 FragColor;

void main() {
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
layout(location = 1) in vec3 aColor;

uniform mat4 uView;
uniform mat4 uProj;

out vec3 vColor;

void main() {
    vColor = aColor;
    gl_Position = uProj * uView * vec4(aPos, 1.0);
}
)";

static const char* kGridFragSrc = R"(
#version 330 core
in vec3 vColor;
out vec4 FragColor;

void main() {
    FragColor = vec4(vColor, 1.0);
}
)";

// ---- Viewport3D ----

bool Viewport3D::init() {
    if (!meshShader_.compile(kMeshVertSrc, kMeshFragSrc)) return false;
    if (!gridShader_.compile(kGridVertSrc, kGridFragSrc)) return false;
    buildGrid();
    return true;
}

void Viewport3D::shutdown() {
    if (gridVAO_) { glDeleteVertexArrays(1, &gridVAO_); gridVAO_ = 0; }
    if (gridVBO_) { glDeleteBuffers(1, &gridVBO_); gridVBO_ = 0; }
}

void Viewport3D::buildGrid() {
    struct GridVert { float x, y, z, r, g, b; };
    std::vector<GridVert> verts;

    float extent = 100.0f;
    float step = 1.0f;

    // Minor grid lines
    for (float i = -extent; i <= extent; i += step) {
        const auto& theme = activeTheme();
        if (std::fabs(i) < 0.001f) continue; // skip axes
        bool major = (std::fabs(std::fmod(i, 10.0f)) < 0.001f);
        float c = major ? theme.gridMajor[0] : theme.gridMinor[0];
        verts.push_back({i, 0, -extent, c, c, c});
        verts.push_back({i, 0,  extent, c, c, c});
        verts.push_back({-extent, 0, i, c, c, c});
        verts.push_back({ extent, 0, i, c, c, c});
    }

    // X axis (red)
    verts.push_back({-extent, 0, 0, 0.7f, 0.2f, 0.2f});
    verts.push_back({ extent, 0, 0, 0.7f, 0.2f, 0.2f});
    // Z axis (blue)
    verts.push_back({0, 0, -extent, 0.2f, 0.2f, 0.7f});
    verts.push_back({0, 0,  extent, 0.2f, 0.2f, 0.7f});

    gridVertCount_ = (int)verts.size();

    glGenVertexArrays(1, &gridVAO_);
    glGenBuffers(1, &gridVBO_);

    glBindVertexArray(gridVAO_);
    glBindBuffer(GL_ARRAY_BUFFER, gridVBO_);
    glBufferData(GL_ARRAY_BUFFER, verts.size() * sizeof(GridVert), verts.data(), GL_STATIC_DRAW);

    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(GridVert), (void*)0);
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(GridVert), (void*)(3 * sizeof(float)));

    glBindVertexArray(0);
}

void Viewport3D::drawGrid(const float* view, const float* proj) {
    gridShader_.use();
    gridShader_.setMat4("uView", view);
    gridShader_.setMat4("uProj", proj);

    glBindVertexArray(gridVAO_);
    glDrawArrays(GL_LINES, 0, gridVertCount_);
    glBindVertexArray(0);
}

void Viewport3D::handleInput(float canvasX, float canvasY, float canvasW, float canvasH) {
    ImGuiIO& io = ImGui::GetIO();
    ImVec2 mouse = io.MousePos;

    // Don't handle input when ImGui wants the mouse (e.g., scrolling in a panel)
    if (io.WantCaptureMouse) return;

    // Check if mouse is within the viewport area
    bool hovered = mouse.x >= canvasX && mouse.x < canvasX + canvasW &&
                   mouse.y >= canvasY && mouse.y < canvasY + canvasH;
    if (!hovered) return;

    // Middle mouse: orbit (or pan with shift)
    if (ImGui::IsMouseDragging(ImGuiMouseButton_Middle, 0.0f)) {
        if (io.KeyShift) {
            camera_.pan(io.MouseDelta.x, io.MouseDelta.y, canvasW, canvasH);
        } else {
            camera_.orbit(io.MouseDelta.x, io.MouseDelta.y);
        }
    }

    // Scroll: zoom
    if (io.MouseWheel != 0.0f) {
        camera_.zoom(io.MouseWheel);
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

    drawGrid(view, proj);

    glDisable(GL_DEPTH_TEST);
}

void Viewport3D::drawGroundGrid(const float* view, const float* proj) {
    drawGrid(view, proj);
}

} // namespace shitcad
