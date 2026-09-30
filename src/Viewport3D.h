#pragma once
#include "ShaderProgram.h"
#include "ViewportInput.h"
#include <glad/gl.h>
#include <cmath>

namespace shitcad {

struct OrbitCamera {
    float distance = 50.0f;
    float yaw = 45.0f;     // degrees
    float pitch = 30.0f;   // degrees
    float panX = 0.0f;
    float panY = 0.0f;
    float targetX = 0.0f;
    float targetY = 0.0f;
    float targetZ = 0.0f;
    bool orthographic = false;
    // Orthographic only because an axis view switched it (Blender's Auto
    // Perspective): orbiting away returns to perspective.
    bool autoOrtho = false;

    // Switch projection, keeping the size of things at the target on screen.
    void setOrthographic(bool on);
    void orbit(float dx, float dy);
    void pan(float dx, float dy, float viewportW, float viewportH);
    void zoom(float delta);

    void getViewMatrix(float* out) const;
    void getEyePosition(float* out) const;
    void getProjection(float* out, float aspect) const;
    // The world axis the view looks straight along (0 X, 1 Y, 2 Z, within
    // a degree), or -1 for an oblique view.
    int viewAxis() const;
};

void makePerspective(float* out, float fovDeg, float aspect, float near, float far);
void makeOrthographic(float* out, float halfH, float aspect, float near, float far);

class Viewport3D {
public:
    bool init();
    void shutdown();

    void render(float x, float y, float w, float h);
    // Sky above the horizon, ground below, from the camera's orientation
    // (the same in ortho and perspective). Fills the viewport; writes no
    // depth or stencil. Draw it first.
    void drawBackground(const float* view, float aspect);
    // The grid in the plane facing the camera, drawn only while the view
    // looks straight along an axis (viewAxis()). A backdrop: it writes no
    // depth, so everything drawn after it is in front.
    void drawGroundGrid(const float* view, const float* proj, float viewportW, float viewportH);

    OrbitCamera& camera() { return camera_; }
    const OrbitCamera& camera() const { return camera_; }
    ShaderProgram& meshShader() { return meshShader_; }
    ShaderProgram& gridShader() { return gridShader_; }

    void handleInput(const InputFrame& in, float canvasX, float canvasY, float canvasW, float canvasH);
    void rebuildGrid() { if (gridVAO_) { glDeleteVertexArrays(1, &gridVAO_); gridVAO_ = 0; } if (gridVBO_) { glDeleteBuffers(1, &gridVBO_); gridVBO_ = 0; } buildGrid(); }

private:
    OrbitCamera camera_;
    ShaderProgram meshShader_;
    ShaderProgram gridShader_;
    ShaderProgram backgroundShader_;
    GLuint backgroundVAO_ = 0;   // empty: the triangle comes from gl_VertexID

    // Dynamic line buffer, refilled each frame by drawGrid.
    GLuint gridVAO_ = 0;
    GLuint gridVBO_ = 0;

    void buildGrid();
    void drawGrid(const float* view, const float* proj, float viewportW, float viewportH);
};

} // namespace shitcad
