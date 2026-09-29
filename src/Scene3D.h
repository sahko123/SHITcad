#pragma once
#include "Section.h"
#include "ShaderProgram.h"
#include <glad/gl.h>
#include <TopoDS_Shape.hxx>
#include <cstdint>
#include <string>
#include <vector>

namespace shitcad {

struct MeshVertex {
    float px, py, pz;
    float nx, ny, nz;
};

struct EdgeVertex {
    float px, py, pz;
};

// A body's geometry lives on the CPU (vertices, edges); its GL buffers are a
// cache that Scene3D fills lazily when it renders (gpuDirty). Building,
// replacing and removing bodies therefore never touches OpenGL, so replay,
// undo and commit work without a current GL context.
struct Body3D {
    TopoDS_Shape shape; // null for mesh-only bodies (imported STL) - check before OCCT ops
    std::vector<MeshVertex> vertices;
    std::vector<EdgeVertex> edges;  // wireframe, extracted when the body is built
    GLuint vao = 0;
    GLuint vbo = 0;
    int vertexCount = 0;            // vertices.size(), set when the body is built
    bool gpuDirty = true;           // GL buffers do not match vertices/edges yet
    float colorR = 0.6f, colorG = 0.65f, colorB = 0.7f;
    bool visible = true;
    // Identity: the feature that created the body and which of its bodies this
    // is (solid i of an import, split j of a cut). Stable across replays, which
    // body indices are not: a Boolean finds its bodies by it. A body a later
    // feature joins or cuts keeps its identity. 0 = not set (a preview).
    uint32_t sourceFeature = 0;
    int sourceIndex = 0;
    bool closed = true;        // watertight? OCCT solids are; an imported STL may not be
    std::string name;          // part name from an imported file, else empty
    bool hasFileColor = false; // colour from an imported file, restored by resetBodyColors()
    float fileColor[3] = {0, 0, 0};

    bool isMeshOnly() const { return shape.IsNull(); }

    // Wireframe edges
    GLuint edgeVAO = 0;
    GLuint edgeVBO = 0;
    int edgeVertexCount = 0;

    // RAII: destructor frees OpenGL resources
    ~Body3D() {
        if (vao) glDeleteVertexArrays(1, &vao);
        if (vbo) glDeleteBuffers(1, &vbo);
        if (edgeVAO) glDeleteVertexArrays(1, &edgeVAO);
        if (edgeVBO) glDeleteBuffers(1, &edgeVBO);
    }

    // Move constructor: transfer ownership, zero source
    Body3D(Body3D&& other) noexcept
        : shape(std::move(other.shape)), vertices(std::move(other.vertices)),
          edges(std::move(other.edges)),
          vao(other.vao), vbo(other.vbo), vertexCount(other.vertexCount), gpuDirty(other.gpuDirty),
          colorR(other.colorR), colorG(other.colorG), colorB(other.colorB),
          visible(other.visible), sourceFeature(other.sourceFeature), sourceIndex(other.sourceIndex),
          closed(other.closed),
          name(std::move(other.name)), hasFileColor(other.hasFileColor),
          fileColor{other.fileColor[0], other.fileColor[1], other.fileColor[2]},
          edgeVAO(other.edgeVAO), edgeVBO(other.edgeVBO), edgeVertexCount(other.edgeVertexCount) {
        other.vao = 0; other.vbo = 0;
        other.edgeVAO = 0; other.edgeVBO = 0;
        other.vertexCount = 0; other.edgeVertexCount = 0;
    }

    // Move assignment: clean up old, transfer, zero source
    Body3D& operator=(Body3D&& other) noexcept {
        if (this != &other) {
            // Free our current GL resources
            if (vao) glDeleteVertexArrays(1, &vao);
            if (vbo) glDeleteBuffers(1, &vbo);
            if (edgeVAO) glDeleteVertexArrays(1, &edgeVAO);
            if (edgeVBO) glDeleteBuffers(1, &edgeVBO);
            // Transfer
            shape = std::move(other.shape);
            vertices = std::move(other.vertices);
            edges = std::move(other.edges);
            vao = other.vao; vbo = other.vbo; vertexCount = other.vertexCount;
            gpuDirty = other.gpuDirty;
            colorR = other.colorR; colorG = other.colorG; colorB = other.colorB;
            visible = other.visible;
            sourceFeature = other.sourceFeature;
            sourceIndex = other.sourceIndex;
            closed = other.closed;
            name = std::move(other.name);
            hasFileColor = other.hasFileColor;
            for (int i = 0; i < 3; i++) fileColor[i] = other.fileColor[i];
            edgeVAO = other.edgeVAO; edgeVBO = other.edgeVBO; edgeVertexCount = other.edgeVertexCount;
            // Zero source
            other.vao = 0; other.vbo = 0;
            other.edgeVAO = 0; other.edgeVBO = 0;
            other.vertexCount = 0; other.edgeVertexCount = 0;
        }
        return *this;
    }

    // No copy (would double-free GL handles)
    Body3D(const Body3D&) = delete;
    Body3D& operator=(const Body3D&) = delete;

    // Default constructor
    Body3D() = default;
};

class Scene3D {
public:
    // `deflection` is the tessellation tolerance in mm; <= 0 picks one from
    // the shape's size (tessellationDeflection in CadImport.h). `premeshed`:
    // the shape was tessellated already (an import), so use it as it is.
    void addBody(const TopoDS_Shape& shape, double deflection = 0.0, bool premeshed = false);
    void addMeshBody(Body3D&& body);
    void replaceBody(int index, const TopoDS_Shape& newShape);
    void removeBody(int index);
    void removeLastBody();
    void clear();
    // Back to the theme's body colour (or the file's), after a tool tinted bodies.
    void resetBodyColors();

    // `hideFeatures`: mesh bodies whose sourceFeature is listed are skipped,
    // used while simulation results are drawn on those same triangles. Only the
    // surfaces the results actually cover are hidden - a mesh imported after the
    // run stays visible.
    void render(ShaderProgram& shader, const float* view, const float* proj,
                const float* eyePos, const std::vector<uint32_t>* hideFeatures = nullptr,
                const SectionPlane* section = nullptr);

    bool empty() const { return bodies_.empty(); }
    size_t bodyCount() const { return bodies_.size(); }
    const Body3D& getBody(int index) const { return bodies_[index]; }
    // Index of the body with this identity, or -1.
    int findBody(uint32_t sourceFeature, int sourceIndex) const;
    Body3D& getBodyMut(int index) { return bodies_[index]; }

    static void triangulateShape(const TopoDS_Shape& shape, std::vector<MeshVertex>& out,
                                 double deflection = 0.0, bool premeshed = false);
    static void extractEdges(const TopoDS_Shape& shape, std::vector<EdgeVertex>& out,
                             double deflection = 0.0);
    // Immediate upload, for tool preview bodies that live outside the scene.
    static void uploadMesh(Body3D& body);

    // Frees buffers of removed/replaced bodies and uploads dirty ones. Needs a
    // current GL context; render() and renderEdges() call it themselves.
    void syncGpu();

    void renderEdges(ShaderProgram& shader, const float* view, const float* proj,
                     const float* edgeColor, const SectionPlane* section = nullptr);

private:
    static void uploadEdges(Body3D& body);
    // Hand a body's GL buffers to the pending lists so its destructor does not
    // call GL; they are freed by the next syncGpu().
    void releaseGpu(Body3D& body);

    std::vector<Body3D> bodies_;
    std::vector<GLuint> pendingVAOs_, pendingVBOs_;
};

} // namespace shitcad
