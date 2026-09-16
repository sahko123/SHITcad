#pragma once
#include "Section.h"
#include "ShaderProgram.h"
#include <glad/gl.h>
#include <TopoDS_Shape.hxx>
#include <cstdint>
#include <vector>

namespace shitcad {

struct MeshVertex {
    float px, py, pz;
    float nx, ny, nz;
};

struct EdgeVertex {
    float px, py, pz;
};

struct Body3D {
    TopoDS_Shape shape; // null for mesh-only bodies (imported STL) - check before OCCT ops
    std::vector<MeshVertex> vertices;
    GLuint vao = 0;
    GLuint vbo = 0;
    int vertexCount = 0;
    float colorR = 0.6f, colorG = 0.65f, colorB = 0.7f;
    bool visible = true;
    uint32_t sourceFeature = 0; // FeatureID of the MeshImport that made a mesh-only body, else 0

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
          vao(other.vao), vbo(other.vbo), vertexCount(other.vertexCount),
          colorR(other.colorR), colorG(other.colorG), colorB(other.colorB),
          visible(other.visible), sourceFeature(other.sourceFeature),
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
            vao = other.vao; vbo = other.vbo; vertexCount = other.vertexCount;
            colorR = other.colorR; colorG = other.colorG; colorB = other.colorB;
            visible = other.visible;
            sourceFeature = other.sourceFeature;
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
    void addBody(const TopoDS_Shape& shape);
    void addMeshBody(Body3D&& body);
    void replaceBody(int index, const TopoDS_Shape& newShape);
    void removeBody(int index);
    void removeLastBody();
    void clear();

    // skipMeshOnly: leave out imported meshes, e.g. while simulation results
    // (drawn on the same triangles) are shown in their place.
    void render(ShaderProgram& shader, const float* view, const float* proj,
                const float* eyePos, bool skipMeshOnly = false,
                const SectionPlane* section = nullptr);

    bool empty() const { return bodies_.empty(); }
    size_t bodyCount() const { return bodies_.size(); }
    const Body3D& getBody(int index) const { return bodies_[index]; }
    Body3D& getBodyMut(int index) { return bodies_[index]; }

    static void triangulateShape(const TopoDS_Shape& shape, std::vector<MeshVertex>& out);
    static void extractEdges(const TopoDS_Shape& shape, std::vector<EdgeVertex>& out);
    static void uploadMesh(Body3D& body);
    static void uploadEdges(Body3D& body);

    void renderEdges(ShaderProgram& shader, const float* view, const float* proj,
                     const float* edgeColor, const SectionPlane* section = nullptr);

private:
    std::vector<Body3D> bodies_;
};

} // namespace shitcad
