#pragma once
#include "SketchPlane.h"
#include "ShaderProgram.h"
#include "Tools.h"
#include "Snap.h"
#include "Selection.h"
#include "ProfileDetector.h"
#include <glad/gl.h>
#include <set>

namespace shitcad {

struct ProfileRenderCache;

class SketchRenderer {
public:
    bool init();
    void shutdown();

    void renderSketch(const SketchPlane& plane, const float* view, const float* proj,
                      bool isActive, const SelectionState& sel, int dof = 0);

    void renderGrid(const SketchPlane& plane, const float* view, const float* proj,
                    float gridStep,
                    float startU, float endU, float startV, float endV);

    void renderToolPreview(const SketchPlane& plane, const float* view, const float* proj,
                           const ToolState& tool, const ArcToolState& arcTool,
                           Point2D cursorLocal);

    void renderSnapIndicator(const SketchPlane& plane, const float* view, const float* proj,
                             const SnapResult& snap);

    void renderSelectionOverlay(const SketchPlane& plane, const float* view, const float* proj,
                                const SelectionState& sel, Point2D cursorLocal);

    void renderProfileHighlights(const SketchPlane& plane, const float* view, const float* proj,
                                  const Sketch& sketch,
                                  const std::vector<ClosedProfile>& profiles,
                                  const std::set<int>& selectedIndices,
                                  int hoveredIndex,
                                  const std::vector<ProfileRenderCache>& renderCache = {});

    void renderReferencePlanes(const SketchPlane* planes, int count, int activeIndex,
                               const float* view, const float* proj);

    void renderPlanePreview(const SketchPlane& plane, const float* view, const float* proj,
                            const float color[4], float extent = 10.0f);

private:
    ShaderProgram lineShader_;
    ShaderProgram planeShader_;
    GLuint dynamicVAO_ = 0;
    GLuint dynamicVBO_ = 0;

    // Upload vertices to the shared dynamic buffer and draw them: position + RGBA colour per
    // vertex (ColorVertex) or, if not `colored`, position only.
    void drawDynamic(GLenum mode, const void* data, int vertexCount, bool colored);

    struct ColorVertex { float x, y, z, r, g, b, a; };

    void drawLines(const float* view, const float* proj,
                   const ColorVertex* verts, int count);
    void drawPoints(const float* view, const float* proj,
                    const ColorVertex* verts, int count, float size);
    void drawQuad(const float* view, const float* proj,
                  const float corners[12], const float color[4]);
};

} // namespace shitcad
