#pragma once
#include "ShaderProgram.h"
#include <glad/gl.h>
#include <TopoDS_Shape.hxx>
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
    TopoDS_Shape shape;
    std::vector<MeshVertex> vertices;
    GLuint vao = 0;
    GLuint vbo = 0;
    int vertexCount = 0;
    float colorR = 0.6f, colorG = 0.65f, colorB = 0.7f;
    bool visible = true;

    // Wireframe edges
    GLuint edgeVAO = 0;
    GLuint edgeVBO = 0;
    int edgeVertexCount = 0;
};

class Scene3D {
public:
    void addBody(const TopoDS_Shape& shape);
    void addMeshBody(Body3D&& body);
    void replaceBody(int index, const TopoDS_Shape& newShape);
    void removeBody(int index);
    void removeLastBody();
    void clear();

    void render(ShaderProgram& shader, const float* view, const float* proj,
                const float* eyePos);

    bool empty() const { return bodies_.empty(); }
    size_t bodyCount() const { return bodies_.size(); }
    const Body3D& getBody(int index) const { return bodies_[index]; }
    Body3D& getBodyMut(int index) { return bodies_[index]; }

    static void triangulateShape(const TopoDS_Shape& shape, std::vector<MeshVertex>& out);
    static void extractEdges(const TopoDS_Shape& shape, std::vector<EdgeVertex>& out);
    static void uploadMesh(Body3D& body);
    static void uploadEdges(Body3D& body);

    void renderEdges(ShaderProgram& shader, const float* view, const float* proj,
                     const float* edgeColor);

private:
    std::vector<Body3D> bodies_;
};

} // namespace shitcad
