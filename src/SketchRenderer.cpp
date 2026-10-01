#include "SketchRenderer.h"
#include "ProfileDetector.h"
#include "ExtrudeTool.h"
#include "Preferences.h"
#include <vector>
#include <algorithm>
#include <cmath>

namespace shitcad {

static const char* kLineVertSrc = R"(
#version 330 core
layout(location = 0) in vec3 aPos;
layout(location = 1) in vec4 aColor;
uniform mat4 uView;
uniform mat4 uProj;
out vec4 vColor;
void main() {
    vColor = aColor;
    gl_Position = uProj * uView * vec4(aPos, 1.0);
}
)";

static const char* kLineFragSrc = R"(
#version 330 core
in vec4 vColor;
out vec4 FragColor;
void main() {
    FragColor = vColor;
}
)";

static const char* kPlaneVertSrc = R"(
#version 330 core
layout(location = 0) in vec3 aPos;
uniform mat4 uView;
uniform mat4 uProj;
void main() {
    gl_Position = uProj * uView * vec4(aPos, 1.0);
}
)";

static const char* kPlaneFragSrc = R"(
#version 330 core
uniform vec4 uColor;
out vec4 FragColor;
void main() {
    FragColor = uColor;
}
)";

bool SketchRenderer::init() {
    if (!lineShader_.compile(kLineVertSrc, kLineFragSrc)) return false;
    if (!planeShader_.compile(kPlaneVertSrc, kPlaneFragSrc)) return false;

    glGenVertexArrays(1, &dynamicVAO_);
    glGenBuffers(1, &dynamicVBO_);
    return true;
}

void SketchRenderer::shutdown() {
    if (dynamicVAO_) { glDeleteVertexArrays(1, &dynamicVAO_); dynamicVAO_ = 0; }
    if (dynamicVBO_) { glDeleteBuffers(1, &dynamicVBO_); dynamicVBO_ = 0; }
}

void SketchRenderer::drawDynamic(GLenum mode, const void* data, int vertexCount, bool colored) {
    const GLsizei stride = (GLsizei)(colored ? sizeof(ColorVertex) : 3 * sizeof(float));
    glBindVertexArray(dynamicVAO_);
    glBindBuffer(GL_ARRAY_BUFFER, dynamicVBO_);
    glBufferData(GL_ARRAY_BUFFER, (GLsizeiptr)vertexCount * stride, data, GL_STREAM_DRAW);

    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, stride, (void*)0);
    if (colored) {
        glEnableVertexAttribArray(1);
        glVertexAttribPointer(1, 4, GL_FLOAT, GL_FALSE, stride, (void*)(3 * sizeof(float)));
    } else {
        glDisableVertexAttribArray(1);
    }

    glDrawArrays(mode, 0, vertexCount);
    glBindVertexArray(0);
}

void SketchRenderer::drawLines(const float* view, const float* proj,
                                const ColorVertex* verts, int count) {
    if (count < 2) return;
    lineShader_.use();
    lineShader_.setMat4("uView", view);
    lineShader_.setMat4("uProj", proj);

    drawDynamic(GL_LINES, verts, count, true);
}

void SketchRenderer::drawPoints(const float* view, const float* proj,
                                 const ColorVertex* verts, int count, float size) {
    if (count < 1) return;
    lineShader_.use();
    lineShader_.setMat4("uView", view);
    lineShader_.setMat4("uProj", proj);

    glPointSize(size);
    drawDynamic(GL_POINTS, verts, count, true);
}

void SketchRenderer::drawQuad(const float* view, const float* proj,
                               const float corners[12], const float color[4]) {
    planeShader_.use();
    planeShader_.setMat4("uView", view);
    planeShader_.setMat4("uProj", proj);
    GLint loc = glGetUniformLocation(planeShader_.id(), "uColor");
    glUniform4f(loc, color[0], color[1], color[2], color[3]);

    // Two triangles: 0-1-2, 0-2-3
    float verts[18] = {
        corners[0], corners[1], corners[2],
        corners[3], corners[4], corners[5],
        corners[6], corners[7], corners[8],
        corners[0], corners[1], corners[2],
        corners[6], corners[7], corners[8],
        corners[9], corners[10], corners[11],
    };

    drawDynamic(GL_TRIANGLES, verts, 6, false);
}

void SketchRenderer::renderReferencePlanes(const SketchPlane* planes, int count,
                                            int activeIndex,
                                            const float* view, const float* proj) {
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glDisable(GL_CULL_FACE);
    glDepthMask(GL_FALSE);

    float extent = 10.0f;

    for (int i = 0; i < count; i++) {
        const auto& p = planes[i];
        if (!p.isReferencePlane) continue;
        if (!p.visible && i != activeIndex) continue;
        float corners[12];
        float wx, wy, wz;

        p.localToWorld(-extent, -extent, wx, wy, wz);
        corners[0] = wx; corners[1] = wy; corners[2] = wz;
        p.localToWorld( extent, -extent, wx, wy, wz);
        corners[3] = wx; corners[4] = wy; corners[5] = wz;
        p.localToWorld( extent,  extent, wx, wy, wz);
        corners[6] = wx; corners[7] = wy; corners[8] = wz;
        p.localToWorld(-extent,  extent, wx, wy, wz);
        corners[9] = wx; corners[10] = wy; corners[11] = wz;

        float color[4] = {p.color[0], p.color[1], p.color[2], p.color[3]};
        if (i == activeIndex) color[3] = 0.25f;

        drawQuad(view, proj, corners, color);

        // Draw border lines
        if (i == activeIndex) {
            ColorVertex border[8];
            float bc[4] = {p.color[0], p.color[1], p.color[2], 0.8f};
            for (int j = 0; j < 4; j++) {
                int next = (j + 1) % 4;
                border[j*2]   = {corners[j*3], corners[j*3+1], corners[j*3+2], bc[0], bc[1], bc[2], bc[3]};
                border[j*2+1] = {corners[next*3], corners[next*3+1], corners[next*3+2], bc[0], bc[1], bc[2], bc[3]};
            }
            glDisable(GL_BLEND);
            drawLines(view, proj, border, 8);
            glEnable(GL_BLEND);
        }
    }

    glDepthMask(GL_TRUE);
    glEnable(GL_CULL_FACE);
    glDisable(GL_BLEND);
}

void SketchRenderer::renderPlanePreview(const SketchPlane& plane, const float* view,
                                        const float* proj, const float color[4], float extent) {
    float corners[12];
    float wx, wy, wz;
    plane.localToWorld(-extent, -extent, wx, wy, wz);
    corners[0] = wx; corners[1] = wy; corners[2] = wz;
    plane.localToWorld( extent, -extent, wx, wy, wz);
    corners[3] = wx; corners[4] = wy; corners[5] = wz;
    plane.localToWorld( extent,  extent, wx, wy, wz);
    corners[6] = wx; corners[7] = wy; corners[8] = wz;
    plane.localToWorld(-extent,  extent, wx, wy, wz);
    corners[9] = wx; corners[10] = wy; corners[11] = wz;

    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glDisable(GL_CULL_FACE);
    glDepthMask(GL_FALSE);

    drawQuad(view, proj, corners, color);

    // Border
    ColorVertex border[8];
    float bc[4] = {color[0], color[1], color[2], 0.9f};
    for (int j = 0; j < 4; j++) {
        int next = (j + 1) % 4;
        border[j*2]   = {corners[j*3], corners[j*3+1], corners[j*3+2], bc[0], bc[1], bc[2], bc[3]};
        border[j*2+1] = {corners[next*3], corners[next*3+1], corners[next*3+2], bc[0], bc[1], bc[2], bc[3]};
    }
    drawLines(view, proj, border, 8);

    glDepthMask(GL_TRUE);
    glEnable(GL_CULL_FACE);
    glDisable(GL_BLEND);
}

void SketchRenderer::renderSketch(const SketchPlane& plane, const float* view,
                                   const float* proj, bool isActive,
                                   const SelectionState& sel, int dof) {
    float alpha = isActive ? 1.0f : 0.4f;
    std::vector<ColorVertex> lineVerts;
    std::vector<ColorVertex> pointVerts;

    // Small offset along normal to prevent z-fighting with body faces
    float nOff = 0.02f;
    float nx = plane.normal[0] * nOff;
    float ny = plane.normal[1] * nOff;
    float nz = plane.normal[2] * nOff;

    // Draw lines
    for (const auto& line : plane.sketch.lines) {
        const PointEntity* a = plane.sketch.findPoint(line.startPt);
        const PointEntity* b = plane.sketch.findPoint(line.endPt);
        if (!a || !b) continue;

        float wax, way, waz, wbx, wby, wbz;
        plane.localToWorld(f(a->x), f(a->y), wax, way, waz);
        plane.localToWorld(f(b->x), f(b->y), wbx, wby, wbz);
        wax += nx; way += ny; waz += nz;
        wbx += nx; wby += ny; wbz += nz;

        bool selected = isActive && sel.isSelected(HitType::Line, line.id);
        const auto& tc = line.projected ? activeTheme().sketchProjected
                       : selected ? activeTheme().sketchSelected
                       : (dof > 0) ? activeTheme().sketchUnderconstrained
                       : activeTheme().sketchLine;
        float r = tc[0], g = tc[1], bl = tc[2];

        lineVerts.push_back({wax, way, waz, r, g, bl, alpha});
        lineVerts.push_back({wbx, wby, wbz, r, g, bl, alpha});
    }

    // Draw circles (tessellated)
    for (const auto& circle : plane.sketch.circles) {
        const PointEntity* center = plane.sketch.findPoint(circle.centerPt);
        if (!center) continue;

        bool selected = isActive && sel.isSelected(HitType::Circle, circle.id);
        const auto& tc = circle.projected ? activeTheme().sketchProjected
                       : selected ? activeTheme().sketchSelected
                       : (dof > 0) ? activeTheme().sketchUnderconstrained
                       : activeTheme().sketchLine;
        float r = tc[0], g = tc[1], bl = tc[2];

        int segments = 64;
        for (int i = 0; i < segments; i++) {
            float a0 = kTwoPi * i / segments;
            float a1 = kTwoPi * (i + 1) / segments;
            double lx0 = center->x + circle.radius * std::cos(a0);
            double ly0 = center->y + circle.radius * std::sin(a0);
            double lx1 = center->x + circle.radius * std::cos(a1);
            double ly1 = center->y + circle.radius * std::sin(a1);

            float wx0, wy0, wz0, wx1, wy1, wz1;
            plane.localToWorld(f(lx0), f(ly0), wx0, wy0, wz0);
            plane.localToWorld(f(lx1), f(ly1), wx1, wy1, wz1);
            wx0 += nx; wy0 += ny; wz0 += nz;
            wx1 += nx; wy1 += ny; wz1 += nz;

            lineVerts.push_back({wx0, wy0, wz0, r, g, bl, alpha});
            lineVerts.push_back({wx1, wy1, wz1, r, g, bl, alpha});
        }
    }

    // Draw arcs (tessellated)
    for (const auto& arc : plane.sketch.arcs) {
        const PointEntity* center = plane.sketch.findPoint(arc.centerPt);
        const PointEntity* startP = plane.sketch.findPoint(arc.startPt);
        if (!center || !startP) continue;

        double radius = distance({center->x, center->y}, {startP->x, startP->y});
        bool selected = isActive && sel.isSelected(HitType::Arc, arc.id);
        const auto& tc = arc.projected ? activeTheme().sketchProjected
                       : selected ? activeTheme().sketchSelected
                       : (dof > 0) ? activeTheme().sketchUnderconstrained
                       : activeTheme().sketchLine;
        float r = tc[0], g = tc[1], bl = tc[2];

        float sweep = arc.endAngle - arc.startAngle;
        if (sweep <= 0) sweep += kTwoPi;
        int segments = std::max(8, (int)(std::fabs(sweep) / (kTwoPi) * 64));
        for (int i = 0; i < segments; i++) {
            float a0 = arc.startAngle + sweep * i / segments;
            float a1 = arc.startAngle + sweep * (i + 1) / segments;
            double lx0 = center->x + radius * std::cos(a0);
            double ly0 = center->y + radius * std::sin(a0);
            double lx1 = center->x + radius * std::cos(a1);
            double ly1 = center->y + radius * std::sin(a1);

            float wx0, wy0, wz0, wx1, wy1, wz1;
            plane.localToWorld(f(lx0), f(ly0), wx0, wy0, wz0);
            plane.localToWorld(f(lx1), f(ly1), wx1, wy1, wz1);
            wx0 += nx; wy0 += ny; wz0 += nz;
            wx1 += nx; wy1 += ny; wz1 += nz;

            lineVerts.push_back({wx0, wy0, wz0, r, g, bl, alpha});
            lineVerts.push_back({wx1, wy1, wz1, r, g, bl, alpha});
        }
    }

    // Draw ellipses (tessellated)
    for (const auto& ellipse : plane.sketch.ellipses) {
        const PointEntity* center = plane.sketch.findPoint(ellipse.centerPt);
        if (!center) continue;

        bool selected = isActive && sel.isSelected(HitType::Ellipse, ellipse.id);
        const auto& tc = ellipse.projected ? activeTheme().sketchProjected
                       : selected ? activeTheme().sketchSelected
                       : (dof > 0) ? activeTheme().sketchUnderconstrained
                       : activeTheme().sketchLine;
        float r = tc[0], g = tc[1], bl = tc[2];

        int segments = 64;
        double cosR = std::cos(ellipse.rotation), sinR = std::sin(ellipse.rotation);
        for (int i = 0; i < segments; i++) {
            float a0 = kTwoPi * i / segments;
            float a1 = kTwoPi * (i + 1) / segments;
            double ex0 = ellipse.semiMajor * std::cos(a0), ey0 = ellipse.semiMinor * std::sin(a0);
            double ex1 = ellipse.semiMajor * std::cos(a1), ey1 = ellipse.semiMinor * std::sin(a1);
            double lx0 = center->x + ex0 * cosR - ey0 * sinR;
            double ly0 = center->y + ex0 * sinR + ey0 * cosR;
            double lx1 = center->x + ex1 * cosR - ey1 * sinR;
            double ly1 = center->y + ex1 * sinR + ey1 * cosR;

            float wx0, wy0, wz0, wx1, wy1, wz1;
            plane.localToWorld(f(lx0), f(ly0), wx0, wy0, wz0);
            plane.localToWorld(f(lx1), f(ly1), wx1, wy1, wz1);
            wx0 += nx; wy0 += ny; wz0 += nz;
            wx1 += nx; wy1 += ny; wz1 += nz;

            lineVerts.push_back({wx0, wy0, wz0, r, g, bl, alpha});
            lineVerts.push_back({wx1, wy1, wz1, r, g, bl, alpha});
        }
    }

    // Draw ellipse arcs (tessellated)
    for (const auto& ea : plane.sketch.ellipseArcs) {
        const PointEntity* center = plane.sketch.findPoint(ea.centerPt);
        if (!center) continue;

        bool selected = isActive && sel.isSelected(HitType::EllipseArc, ea.id);
        const auto& tc = ea.projected ? activeTheme().sketchProjected
                       : selected ? activeTheme().sketchSelected
                       : (dof > 0) ? activeTheme().sketchUnderconstrained
                       : activeTheme().sketchLine;
        float r = tc[0], g = tc[1], bl = tc[2];

        float sweep = ea.endAngle - ea.startAngle;
        if (sweep <= 0) sweep += kTwoPi;
        int segments = std::max(8, (int)(std::fabs(sweep) / (kTwoPi) * 64));
        double cosR = std::cos(ea.rotation), sinR = std::sin(ea.rotation);
        for (int i = 0; i < segments; i++) {
            float a0 = ea.startAngle + sweep * i / segments;
            float a1 = ea.startAngle + sweep * (i + 1) / segments;
            double ex0 = ea.semiMajor * std::cos(a0), ey0 = ea.semiMinor * std::sin(a0);
            double ex1 = ea.semiMajor * std::cos(a1), ey1 = ea.semiMinor * std::sin(a1);
            double lx0 = center->x + ex0 * cosR - ey0 * sinR;
            double ly0 = center->y + ex0 * sinR + ey0 * cosR;
            double lx1 = center->x + ex1 * cosR - ey1 * sinR;
            double ly1 = center->y + ex1 * sinR + ey1 * cosR;

            float wx0, wy0, wz0, wx1, wy1, wz1;
            plane.localToWorld(f(lx0), f(ly0), wx0, wy0, wz0);
            plane.localToWorld(f(lx1), f(ly1), wx1, wy1, wz1);
            wx0 += nx; wy0 += ny; wz0 += nz;
            wx1 += nx; wy1 += ny; wz1 += nz;

            lineVerts.push_back({wx0, wy0, wz0, r, g, bl, alpha});
            lineVerts.push_back({wx1, wy1, wz1, r, g, bl, alpha});
        }
    }

    // Draw splines (evaluated B-spline curve)
    for (const auto& sp : plane.sketch.splines) {
        if (sp.controlPtIDs.size() < 2) continue;

        bool selected = isActive && sel.isSelected(HitType::Spline, sp.id);
        const auto& tc = sp.projected ? activeTheme().sketchProjected
                       : selected ? activeTheme().sketchSelected
                       : (dof > 0) ? activeTheme().sketchUnderconstrained
                       : activeTheme().sketchLine;
        float r = tc[0], g = tc[1], bl = tc[2];

        auto pts = sampleSpline(sp, plane.sketch, kSplineSampleCount);
        for (int i = 0; i + 1 < (int)pts.size(); i++) {
            float wx0, wy0, wz0, wx1, wy1, wz1;
            plane.localToWorld(f(pts[i].x), f(pts[i].y), wx0, wy0, wz0);
            plane.localToWorld(f(pts[i+1].x), f(pts[i+1].y), wx1, wy1, wz1);
            wx0 += nx; wy0 += ny; wz0 += nz;
            wx1 += nx; wy1 += ny; wz1 += nz;

            lineVerts.push_back({wx0, wy0, wz0, r, g, bl, alpha});
            lineVerts.push_back({wx1, wy1, wz1, r, g, bl, alpha});
        }
    }

    // Draw points (skip projected points — they're internal polyline vertices)
    for (const auto& pt : plane.sketch.points) {
        if (pt.projected) continue;
        float wx, wy, wz;
        plane.localToWorld(f(pt.x), f(pt.y), wx, wy, wz);
        wx += nx; wy += ny; wz += nz;

        bool selected = isActive && sel.isSelected(HitType::Point, pt.id);
        const auto& tc = selected ? activeTheme().sketchSelected
                       : (dof > 0) ? activeTheme().sketchUnderconstrained
                       : activeTheme().sketchLine;
        float r = tc[0], g = tc[1], bl = tc[2];

        pointVerts.push_back({wx, wy, wz, r, g, bl, alpha});
    }

    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    if (!lineVerts.empty())
        drawLines(view, proj, lineVerts.data(), (int)lineVerts.size());
    if (!pointVerts.empty())
        drawPoints(view, proj, pointVerts.data(), (int)pointVerts.size(), 5.0f);

    glDisable(GL_BLEND);
}

void SketchRenderer::renderGrid(const SketchPlane& plane, const float* view,
                                 const float* proj, float gridStep,
                                 float startU, float endU, float startV, float endV) {
    std::vector<ColorVertex> verts;

    // Normal offset to prevent z-fighting on body faces
    float nOff = 0.01f;
    float onx = plane.normal[0] * nOff;
    float ony = plane.normal[1] * nOff;
    float onz = plane.normal[2] * nOff;

    // Lines parallel to the V axis (varying U position)
    for (float u = std::ceil(startU / gridStep) * gridStep; u <= endU; u += gridStep) {
        if (std::fabs(u) < gridStep * 0.01f) continue; // drawn as axis below
        bool isMajor = std::fabs(std::fmod(u, gridStep * 10.0f)) < gridStep * 0.1f;
        float c = isMajor ? activeTheme().sketchGridMajor : activeTheme().sketchGridMinor;
        float wx0, wy0, wz0, wx1, wy1, wz1;
        plane.localToWorld(u, startV, wx0, wy0, wz0);
        plane.localToWorld(u, endV,   wx1, wy1, wz1);
        verts.push_back({wx0+onx, wy0+ony, wz0+onz, c, c, c, 0.5f});
        verts.push_back({wx1+onx, wy1+ony, wz1+onz, c, c, c, 0.5f});
    }

    // Lines parallel to the U axis (varying V position)
    for (float v = std::ceil(startV / gridStep) * gridStep; v <= endV; v += gridStep) {
        if (std::fabs(v) < gridStep * 0.01f) continue; // drawn as axis below
        bool isMajor = std::fabs(std::fmod(v, gridStep * 10.0f)) < gridStep * 0.1f;
        float c = isMajor ? activeTheme().sketchGridMajor : activeTheme().sketchGridMinor;
        float wx0, wy0, wz0, wx1, wy1, wz1;
        plane.localToWorld(startU, v, wx0, wy0, wz0);
        plane.localToWorld(endU,   v, wx1, wy1, wz1);
        verts.push_back({wx0+onx, wy0+ony, wz0+onz, c, c, c, 0.5f});
        verts.push_back({wx1+onx, wy1+ony, wz1+onz, c, c, c, 0.5f});
    }

    // Local axes on sketch plane (always span the full visible range)
    float wx0, wy0, wz0, wx1, wy1, wz1;
    plane.localToWorld(startU, 0, wx0, wy0, wz0);
    plane.localToWorld(endU,   0, wx1, wy1, wz1);
    verts.push_back({wx0+onx, wy0+ony, wz0+onz, 0.7f, 0.2f, 0.2f, 0.8f});
    verts.push_back({wx1+onx, wy1+ony, wz1+onz, 0.7f, 0.2f, 0.2f, 0.8f});

    plane.localToWorld(0, startV, wx0, wy0, wz0);
    plane.localToWorld(0, endV,   wx1, wy1, wz1);
    verts.push_back({wx0+onx, wy0+ony, wz0+onz, 0.2f, 0.7f, 0.2f, 0.8f});
    verts.push_back({wx1+onx, wy1+ony, wz1+onz, 0.2f, 0.7f, 0.2f, 0.8f});

    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    if (!verts.empty())
        drawLines(view, proj, verts.data(), (int)verts.size());
    glDisable(GL_BLEND);
}

void SketchRenderer::renderToolPreview(const SketchPlane& plane, const float* view,
                                        const float* proj, const ToolState& tool,
                                        const ArcToolState& arcTool,
                                        Point2D cursorLocal) {
    bool hasState = tool.hasFirstPoint || arcTool.clickCount > 0;
    if (!hasState) return;

    const auto& tp = activeTheme().toolPreview;
    float r = tp[0], g = tp[1], b = tp[2], a = tp[3];

    // Normal offset to prevent z-fighting
    float nOff = 0.03f;
    float onx = plane.normal[0] * nOff;
    float ony = plane.normal[1] * nOff;
    float onz = plane.normal[2] * nOff;

    std::vector<ColorVertex> verts;

    auto addSeg = [&](double lx0, double ly0, double lx1, double ly1) {
        float wx0, wy0, wz0, wx1, wy1, wz1;
        plane.localToWorld(f(lx0), f(ly0), wx0, wy0, wz0);
        plane.localToWorld(f(lx1), f(ly1), wx1, wy1, wz1);
        verts.push_back({wx0+onx, wy0+ony, wz0+onz, r, g, b, a});
        verts.push_back({wx1+onx, wy1+ony, wz1+onz, r, g, b, a});
    };

    switch (tool.type) {
        case ToolType::Line:
            addSeg(tool.firstPoint.x, tool.firstPoint.y, cursorLocal.x, cursorLocal.y);
            break;
        case ToolType::Circle: {
            int segments = 64;
            double radius = distance(tool.firstPoint, cursorLocal);
            for (int i = 0; i < segments; i++) {
                float a0 = kTwoPi * i / segments;
                float a1 = kTwoPi * (i + 1) / segments;
                addSeg(tool.firstPoint.x + radius * std::cos(a0),
                       tool.firstPoint.y + radius * std::sin(a0),
                       tool.firstPoint.x + radius * std::cos(a1),
                       tool.firstPoint.y + radius * std::sin(a1));
            }
            break;
        }
        case ToolType::Rectangle: {
            Point2D p1 = tool.firstPoint;
            Point2D p2 = cursorLocal;
            float corners[4][2] = {{(float)p1.x,(float)p1.y},{(float)p2.x,(float)p1.y},{(float)p2.x,(float)p2.y},{(float)p1.x,(float)p2.y}};
            for (int i = 0; i < 4; i++) {
                int next = (i+1) % 4;
                addSeg(corners[i][0], corners[i][1], corners[next][0], corners[next][1]);
            }
            break;
        }
        case ToolType::CenterRect: {
            double cx = tool.firstPoint.x, cy = tool.firstPoint.y;
            double dx = cursorLocal.x - cx, dy = cursorLocal.y - cy;
            double corners[4][2] = {{cx-dx,cy-dy},{cx+dx,cy-dy},{cx+dx,cy+dy},{cx-dx,cy+dy}};
            for (int i = 0; i < 4; i++) {
                int next = (i+1) % 4;
                addSeg(corners[i][0], corners[i][1], corners[next][0], corners[next][1]);
            }
            break;
        }
        case ToolType::Arc3Point: {
            if (arcTool.clickCount == 1) {
                // Line from start to cursor
                addSeg(arcTool.point1.x, arcTool.point1.y, cursorLocal.x, cursorLocal.y);
            } else if (arcTool.clickCount == 2) {
                // Compute circumcircle from 3 points and draw arc
                Point2D p1 = arcTool.point1, p2 = arcTool.point2, p3 = cursorLocal;
                double ax = p1.x, ay = p1.y;
                double bx = p2.x, by = p2.y;
                double cx = p3.x, cy = p3.y;
                double D = 2.0 * (ax*(by-cy) + bx*(cy-ay) + cx*(ay-by));
                if (std::fabs(D) < 1e-6) {
                    // Collinear — draw line
                    addSeg(p1.x, p1.y, p3.x, p3.y);
                } else {
                    double a2 = ax*ax+ay*ay, b2 = bx*bx+by*by, c2 = cx*cx+cy*cy;
                    double ux = (a2*(by-cy)+b2*(cy-ay)+c2*(ay-by))/D;
                    double uy = (a2*(cx-bx)+b2*(ax-cx)+c2*(bx-ax))/D;
                    double rad = std::sqrt((p1.x-ux)*(p1.x-ux)+(p1.y-uy)*(p1.y-uy));
                    double sa = std::atan2(p1.y-uy, p1.x-ux);
                    double ea = std::atan2(p3.y-uy, p3.x-ux);
                    double ta = std::atan2(p2.y-uy, p2.x-ux);

                    // Determine sweep direction so arc passes through p2
                    double nsa = wrap2Pi(sa), nea = wrap2Pi(ea), nta = wrap2Pi(ta);
                    double ccw = ccwSweep(nsa, nea);
                    double toThrough = nta - nsa;
                    if (toThrough < 0) toThrough += kTwoPiD;
                    double sweep = (toThrough < ccw) ? ccw : -(kTwoPiD - ccw);

                    int segs = std::max(8, (int)(std::fabs(sweep) / (2.0*kPi) * 64));
                    for (int i = 0; i < segs; i++) {
                        double t0 = sa + sweep * i / segs;
                        double t1 = sa + sweep * (i+1) / segs;
                        addSeg(ux+rad*std::cos(t0), uy+rad*std::sin(t0),
                               ux+rad*std::cos(t1), uy+rad*std::sin(t1));
                    }
                }
            }
            break;
        }
        case ToolType::ArcCenter: {
            if (arcTool.clickCount == 1) {
                // Radius line from center to cursor + full circle preview
                double rad = distance(arcTool.point1, cursorLocal);
                addSeg(arcTool.point1.x, arcTool.point1.y, cursorLocal.x, cursorLocal.y);
                int segments = 64;
                for (int i = 0; i < segments; i++) {
                    float a0 = kTwoPi * i / segments;
                    float a1 = kTwoPi * (i + 1) / segments;
                    addSeg(arcTool.point1.x + rad * std::cos(a0),
                           arcTool.point1.y + rad * std::sin(a0),
                           arcTool.point1.x + rad * std::cos(a1),
                           arcTool.point1.y + rad * std::sin(a1));
                }
            } else if (arcTool.clickCount == 2) {
                // Arc from start point sweeping CCW to cursor angle
                double rad = distance(arcTool.point1, arcTool.point2);
                double sa = std::atan2(arcTool.point2.y - arcTool.point1.y,
                                       arcTool.point2.x - arcTool.point1.x);
                double ea = std::atan2(cursorLocal.y - arcTool.point1.y,
                                       cursorLocal.x - arcTool.point1.x);
                double sweep = ea - sa;
                if (sweep <= 0) sweep += 2.0 * kPi;
                int segs = std::max(8, (int)(std::fabs(sweep) / (2.0*kPi) * 64));
                for (int i = 0; i < segs; i++) {
                    double t0 = sa + sweep * i / segs;
                    double t1 = sa + sweep * (i+1) / segs;
                    addSeg(arcTool.point1.x + rad*std::cos(t0),
                           arcTool.point1.y + rad*std::sin(t0),
                           arcTool.point1.x + rad*std::cos(t1),
                           arcTool.point1.y + rad*std::sin(t1));
                }
                // Radius lines to start and end
                addSeg(arcTool.point1.x, arcTool.point1.y, arcTool.point2.x, arcTool.point2.y);
                double ex = arcTool.point1.x + rad * std::cos(ea);
                double ey = arcTool.point1.y + rad * std::sin(ea);
                addSeg(arcTool.point1.x, arcTool.point1.y, ex, ey);
            }
            break;
        }
        default:
            break;
    }

    if (!verts.empty()) {
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        drawLines(view, proj, verts.data(), (int)verts.size());
        glDisable(GL_BLEND);
    }
}

void SketchRenderer::renderProfileHighlights(const SketchPlane& plane, const float* view,
                                              const float* proj, const Sketch& sketch,
                                              const std::vector<ClosedProfile>& profiles,
                                              const std::set<int>& selectedIndices,
                                              int hoveredIndex,
                                              const std::vector<ProfileRenderCache>& renderCache) {
    float nOff = 0.025f;
    float nx = plane.normal[0] * nOff;
    float ny = plane.normal[1] * nOff;
    float nz = plane.normal[2] * nOff;

    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glDepthMask(GL_FALSE);
    glDisable(GL_CULL_FACE);

    for (int i = 0; i < (int)profiles.size(); i++) {
        bool isSelected = selectedIndices.count(i) > 0;
        bool isHovered = (i == hoveredIndex);
        if (!isSelected && !isHovered) continue;

        float fillColor[4];
        if (isSelected) {
            fillColor[0] = 0.2f; fillColor[1] = 0.4f; fillColor[2] = 0.9f; fillColor[3] = 0.2f;
        } else {
            fillColor[0] = 0.5f; fillColor[1] = 0.5f; fillColor[2] = 0.5f; fillColor[3] = 0.1f;
        }

        const auto& profile = profiles[i];

        if (profile.isCircle()) {
            // Filled disc as triangle fan
            const CircleEntity* circle = sketch.findCircle(profile.circleID);
            if (!circle) continue;
            Point2D center = sketch.getPointPos(circle->centerPt);

            int segments = 48;
            // Build triangles for the disc
            planeShader_.use();
            planeShader_.setMat4("uView", view);
            planeShader_.setMat4("uProj", proj);
            GLint loc = glGetUniformLocation(planeShader_.id(), "uColor");
            glUniform4f(loc, fillColor[0], fillColor[1], fillColor[2], fillColor[3]);

            std::vector<float> verts;
            for (int s = 0; s < segments; s++) {
                float a0 = kTwoPi * s / segments;
                float a1 = kTwoPi * (s + 1) / segments;

                float wcx, wcy, wcz;
                plane.localToWorld(f(center.x), f(center.y), wcx, wcy, wcz);
                verts.push_back(wcx + nx); verts.push_back(wcy + ny); verts.push_back(wcz + nz);

                double lx0 = center.x + circle->radius * std::cos(a0);
                double ly0 = center.y + circle->radius * std::sin(a0);
                float wx0, wy0, wz0;
                plane.localToWorld(f(lx0), f(ly0), wx0, wy0, wz0);
                verts.push_back(wx0 + nx); verts.push_back(wy0 + ny); verts.push_back(wz0 + nz);

                double lx1 = center.x + circle->radius * std::cos(a1);
                double ly1 = center.y + circle->radius * std::sin(a1);
                float wx1, wy1, wz1;
                plane.localToWorld(f(lx1), f(ly1), wx1, wy1, wz1);
                verts.push_back(wx1 + nx); verts.push_back(wy1 + ny); verts.push_back(wz1 + nz);
            }

            drawDynamic(GL_TRIANGLES, verts.data(), (int)(verts.size() / 3), false);

            // Boundary ring
            float edgeColor[4];
            if (isSelected) {
                edgeColor[0] = 1.0f; edgeColor[1] = 0.65f; edgeColor[2] = 0.0f; edgeColor[3] = 0.9f;
            } else {
                edgeColor[0] = 0.4f; edgeColor[1] = 0.7f; edgeColor[2] = 1.0f; edgeColor[3] = 0.6f;
            }
            std::vector<ColorVertex> edgeVerts;
            for (int s = 0; s < segments; s++) {
                float a0 = kTwoPi * s / segments;
                float a1 = kTwoPi * (s + 1) / segments;
                float wx0, wy0, wz0, wx1, wy1, wz1;
                plane.localToWorld(f(center.x + circle->radius * std::cos(a0)),
                                   f(center.y + circle->radius * std::sin(a0)), wx0, wy0, wz0);
                plane.localToWorld(f(center.x + circle->radius * std::cos(a1)),
                                   f(center.y + circle->radius * std::sin(a1)), wx1, wy1, wz1);
                edgeVerts.push_back({wx0+nx, wy0+ny, wz0+nz, edgeColor[0], edgeColor[1], edgeColor[2], edgeColor[3]});
                edgeVerts.push_back({wx1+nx, wy1+ny, wz1+nz, edgeColor[0], edgeColor[1], edgeColor[2], edgeColor[3]});
            }
            drawLines(view, proj, edgeVerts.data(), (int)edgeVerts.size());

        } else {
            // Polygon: use cached tessellation + ear-clipping if available
            bool hasCache = (i < (int)renderCache.size() && !renderCache[i].tessPoints.empty());

            // triMeshPoints is the merged polygon (outer + holes) used for fill triangulation
            const std::vector<Point2D>& pts = hasCache && !renderCache[i].triMeshPoints.empty()
                ? renderCache[i].triMeshPoints
                : (hasCache ? renderCache[i].tessPoints : [&]() -> const std::vector<Point2D>& {
                    static thread_local std::vector<Point2D> fallback;
                    fallback = tessellateProfile(sketch, profile);
                    return fallback;
                }());
            int n = (int)pts.size();
            if (n < 3) continue;

            // Use cached triangle indices or compute on demand
            const std::vector<int>& triIdx = hasCache ? renderCache[i].triIndices : [&]() -> const std::vector<int>& {
                static thread_local std::vector<int> fallbackTri;
                fallbackTri.clear();
                // Inline ear-clipping fallback (no holes)
                std::vector<int> indices(n);
                for (int j = 0; j < n; j++) indices[j] = j;
                double signedArea = 0;
                for (int j = 0; j < n; j++) {
                    int jn = (j + 1) % n;
                    signedArea += pts[j].x * pts[jn].y - pts[jn].x * pts[j].y;
                }
                if (signedArea < 0) std::reverse(indices.begin(), indices.end());
                auto cross2D = [](Point2D o, Point2D a, Point2D b) -> double {
                    return (a.x - o.x) * (b.y - o.y) - (a.y - o.y) * (b.x - o.x);
                };
                auto pointInTriangle = [&](Point2D p, Point2D a, Point2D b, Point2D c) -> bool {
                    float d1 = cross2D(p, a, b), d2 = cross2D(p, b, c), d3 = cross2D(p, c, a);
                    return !((d1 < 0 || d2 < 0 || d3 < 0) && (d1 > 0 || d2 > 0 || d3 > 0));
                };
                while ((int)indices.size() > 2) {
                    int sz = (int)indices.size();
                    bool earFound = false;
                    for (int j = 0; j < sz; j++) {
                        int prev = (j - 1 + sz) % sz, next = (j + 1) % sz;
                        Point2D pA = pts[indices[prev]], pB = pts[indices[j]], pC = pts[indices[next]];
                        if (cross2D(pA, pB, pC) <= 1e-7f) continue;
                        bool isEar = true;
                        for (int k = 0; k < sz; k++) {
                            if (k == prev || k == j || k == next) continue;
                            if (pointInTriangle(pts[indices[k]], pA, pB, pC)) { isEar = false; break; }
                        }
                        if (!isEar) continue;
                        fallbackTri.push_back(indices[prev]);
                        fallbackTri.push_back(indices[j]);
                        fallbackTri.push_back(indices[next]);
                        indices.erase(indices.begin() + j);
                        earFound = true;
                        break;
                    }
                    if (!earFound) break;
                }
                return fallbackTri;
            }();

            planeShader_.use();
            planeShader_.setMat4("uView", view);
            planeShader_.setMat4("uProj", proj);
            GLint loc = glGetUniformLocation(planeShader_.id(), "uColor");
            glUniform4f(loc, fillColor[0], fillColor[1], fillColor[2], fillColor[3]);

            std::vector<float> verts;
            for (int ti = 0; ti + 2 < (int)triIdx.size(); ti += 3) {
                for (int k = 0; k < 3; k++) {
                    int idx = triIdx[ti + k];
                    if (idx < 0 || idx >= n) continue;
                    float wx, wy, wz;
                    plane.localToWorld(f(pts[idx].x), f(pts[idx].y), wx, wy, wz);
                    verts.push_back(wx + nx); verts.push_back(wy + ny); verts.push_back(wz + nz);
                }
            }

            drawDynamic(GL_TRIANGLES, verts.data(), (int)(verts.size() / 3), false);

            // Boundary edges (outer + holes)
            float edgeColor[4];
            if (isSelected) {
                edgeColor[0] = 1.0f; edgeColor[1] = 0.65f; edgeColor[2] = 0.0f; edgeColor[3] = 0.9f;
            } else {
                edgeColor[0] = 0.4f; edgeColor[1] = 0.7f; edgeColor[2] = 1.0f; edgeColor[3] = 0.6f;
            }
            std::vector<ColorVertex> edgeVerts;

            // Draw outer boundary from the original (non-merged) tessellation
            auto drawBoundary = [&](const std::vector<Point2D>& bnd) {
                int bn = (int)bnd.size();
                for (int j = 0; j < bn; j++) {
                    int jn = (j + 1) % bn;
                    float wax, way, waz, wbx, wby, wbz;
                    plane.localToWorld(f(bnd[j].x), f(bnd[j].y), wax, way, waz);
                    plane.localToWorld(f(bnd[jn].x), f(bnd[jn].y), wbx, wby, wbz);
                    edgeVerts.push_back({wax+nx, way+ny, waz+nz, edgeColor[0], edgeColor[1], edgeColor[2], edgeColor[3]});
                    edgeVerts.push_back({wbx+nx, wby+ny, wbz+nz, edgeColor[0], edgeColor[1], edgeColor[2], edgeColor[3]});
                }
            };

            // Outer boundary: use original tessellation (pre-merge) if holes exist
            bool hasHoleCache = (hasCache && !renderCache[i].holeTessPoints.empty());
            if (hasHoleCache) {
                // renderCache tessPoints was merged with holes — use profile's own tessellation for outer boundary
                auto outerPts = tessellateProfile(sketch, profile);
                drawBoundary(outerPts);
                // Hole boundaries
                for (const auto& holePts : renderCache[i].holeTessPoints) {
                    drawBoundary(holePts);
                }
            } else {
                drawBoundary(pts);
            }

            drawLines(view, proj, edgeVerts.data(), (int)edgeVerts.size());
        }
    }

    glDepthMask(GL_TRUE);
    glEnable(GL_CULL_FACE);
    glDisable(GL_BLEND);
}

void SketchRenderer::renderSelectionOverlay(const SketchPlane& plane, const float* view,
                                             const float* proj, const SelectionState& sel,
                                             Point2D cursorLocal) {
    // Normal offset
    float nOff = 0.04f;
    float nx = plane.normal[0] * nOff;
    float ny = plane.normal[1] * nOff;
    float nz = plane.normal[2] * nOff;

    float cyan[4] = {0.0f, 0.9f, 0.9f, 0.8f};

    if (sel.dragMode == SelectionDragMode::BoxSelect) {
        // Draw rectangle border
        Point2D a = sel.dragAnchor;
        Point2D b = cursorLocal;
        float corners[4][2] = {{(float)a.x,(float)a.y},{(float)b.x,(float)a.y},{(float)b.x,(float)b.y},{(float)a.x,(float)b.y}};

        ColorVertex border[8];
        for (int i = 0; i < 4; i++) {
            int next = (i + 1) % 4;
            float wa[3], wb[3];
            plane.localToWorld(corners[i][0], corners[i][1], wa[0], wa[1], wa[2]);
            plane.localToWorld(corners[next][0], corners[next][1], wb[0], wb[1], wb[2]);
            border[i*2]   = {wa[0]+nx, wa[1]+ny, wa[2]+nz, cyan[0], cyan[1], cyan[2], cyan[3]};
            border[i*2+1] = {wb[0]+nx, wb[1]+ny, wb[2]+nz, cyan[0], cyan[1], cyan[2], cyan[3]};
        }

        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        drawLines(view, proj, border, 8);

        // Translucent fill
        float qCorners[12];
        for (int i = 0; i < 4; i++) {
            float wx, wy, wz;
            plane.localToWorld(corners[i][0], corners[i][1], wx, wy, wz);
            qCorners[i*3]   = wx + nx;
            qCorners[i*3+1] = wy + ny;
            qCorners[i*3+2] = wz + nz;
        }
        float fillColor[4] = {0.0f, 0.9f, 0.9f, 0.15f};
        glDepthMask(GL_FALSE);
        drawQuad(view, proj, qCorners, fillColor);
        glDepthMask(GL_TRUE);
        glDisable(GL_BLEND);

    } else if (sel.dragMode == SelectionDragMode::LassoSelect) {
        if (sel.lassoPoints.size() < 2) return;

        // Draw polygon edges
        std::vector<ColorVertex> verts;
        for (int i = 0; i < (int)sel.lassoPoints.size() - 1; i++) {
            float wa[3], wb[3];
            plane.localToWorld(f(sel.lassoPoints[i].x), f(sel.lassoPoints[i].y), wa[0], wa[1], wa[2]);
            plane.localToWorld(f(sel.lassoPoints[i+1].x), f(sel.lassoPoints[i+1].y), wb[0], wb[1], wb[2]);
            verts.push_back({wa[0]+nx, wa[1]+ny, wa[2]+nz, cyan[0], cyan[1], cyan[2], cyan[3]});
            verts.push_back({wb[0]+nx, wb[1]+ny, wb[2]+nz, cyan[0], cyan[1], cyan[2], cyan[3]});
        }
        // Closing segment back to cursor
        {
            float wa[3], wb[3];
            plane.localToWorld(f(sel.lassoPoints.back().x), f(sel.lassoPoints.back().y), wa[0], wa[1], wa[2]);
            plane.localToWorld(f(cursorLocal.x), f(cursorLocal.y), wb[0], wb[1], wb[2]);
            verts.push_back({wa[0]+nx, wa[1]+ny, wa[2]+nz, cyan[0], cyan[1], cyan[2], cyan[3]});
            verts.push_back({wb[0]+nx, wb[1]+ny, wb[2]+nz, cyan[0], cyan[1], cyan[2], cyan[3]});
        }
        // Close polygon: cursor back to first point
        {
            float wa[3], wb[3];
            plane.localToWorld(f(cursorLocal.x), f(cursorLocal.y), wa[0], wa[1], wa[2]);
            plane.localToWorld(f(sel.lassoPoints[0].x), f(sel.lassoPoints[0].y), wb[0], wb[1], wb[2]);
            verts.push_back({wa[0]+nx, wa[1]+ny, wa[2]+nz, cyan[0], cyan[1], cyan[2], cyan[3]});
            verts.push_back({wb[0]+nx, wb[1]+ny, wb[2]+nz, cyan[0], cyan[1], cyan[2], cyan[3]});
        }

        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        if (!verts.empty())
            drawLines(view, proj, verts.data(), (int)verts.size());
        glDisable(GL_BLEND);
    }
}

void SketchRenderer::renderSnapIndicator(const SketchPlane& plane, const float* view,
                                          const float* proj, const SnapResult& snap) {
    if (snap.type == SnapType::None) return;

    // Normal offset to prevent z-fighting
    float nOff = 0.03f;
    float onx = plane.normal[0] * nOff;
    float ony = plane.normal[1] * nOff;
    float onz = plane.normal[2] * nOff;

    const auto& sc = activeTheme().snapColor;
    float r = sc[0], g = sc[1], b = sc[2], a = sc[3];
    float sz = 0.15f; // world-space size of indicator

    std::vector<ColorVertex> verts;

    float wx0, wy0, wz0, wx1, wy1, wz1;

    if (snap.type == SnapType::Quadrant) {
        // Diamond indicator (rotated square) for quadrant snaps
        auto addSeg = [&](float ax, float ay, float bx, float by) {
            plane.localToWorld(ax, ay, wx0, wy0, wz0);
            plane.localToWorld(bx, by, wx1, wy1, wz1);
            verts.push_back({wx0+onx, wy0+ony, wz0+onz, r, g, b, a});
            verts.push_back({wx1+onx, wy1+ony, wz1+onz, r, g, b, a});
        };
        float px = (float)snap.position.x, py = (float)snap.position.y;
        addSeg(px,      py + sz, px + sz, py     ); // N → E
        addSeg(px + sz, py,      px,      py - sz ); // E → S
        addSeg(px,      py - sz, px - sz, py     ); // S → W
        addSeg(px - sz, py,      px,      py + sz ); // W → N
    } else {
        // Cross indicator for all other snap types
        plane.localToWorld(f(snap.position.x) - sz, f(snap.position.y), wx0, wy0, wz0);
        plane.localToWorld(f(snap.position.x) + sz, f(snap.position.y), wx1, wy1, wz1);
        verts.push_back({wx0+onx, wy0+ony, wz0+onz, r, g, b, a});
        verts.push_back({wx1+onx, wy1+ony, wz1+onz, r, g, b, a});

        plane.localToWorld(f(snap.position.x), f(snap.position.y) - sz, wx0, wy0, wz0);
        plane.localToWorld(f(snap.position.x), f(snap.position.y) + sz, wx1, wy1, wz1);
        verts.push_back({wx0+onx, wy0+ony, wz0+onz, r, g, b, a});
        verts.push_back({wx1+onx, wy1+ony, wz1+onz, r, g, b, a});
    }

    drawLines(view, proj, verts.data(), (int)verts.size());
}

} // namespace shitcad
