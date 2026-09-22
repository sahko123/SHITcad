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

    void orbit(float dx, float dy);
    void pan(float dx, float dy, float viewportW, float viewportH);
    void zoom(float delta);

    void getViewMatrix(float* out) const;
    void getEyePosition(float* out) const;
    void getProjection(float* out, float aspect) const;
};

void makePerspective(float* out, float fovDeg, float aspect, float near, float far);
void makeOrthographic(float* out, float halfH, float aspect, float near, float far);

class Viewport3D {
public:
    bool init();
    void shutdown();

    void render(float x, float y, float w, float h);
    void drawGroundGrid(const float* view, const float* proj);

    OrbitCamera& camera() { return camera_; }
    ShaderProgram& meshShader() { return meshShader_; }
    ShaderProgram& gridShader() { return gridShader_; }

    void handleInput(const InputFrame& in, float canvasX, float canvasY, float canvasW, float canvasH);
    void rebuildGrid() { if (gridVAO_) { glDeleteVertexArrays(1, &gridVAO_); gridVAO_ = 0; } if (gridVBO_) { glDeleteBuffers(1, &gridVBO_); gridVBO_ = 0; } buildGrid(); }

private:
    OrbitCamera camera_;
    ShaderProgram meshShader_;
    ShaderProgram gridShader_;

    GLuint gridVAO_ = 0;
    GLuint gridVBO_ = 0;
    int gridVertCount_ = 0;

    void buildGrid();
    void drawGrid(const float* view, const float* proj);
};

} // namespace shitcad
