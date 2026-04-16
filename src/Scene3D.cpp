#include "Scene3D.h"
#include "Preferences.h"

#include <BRepMesh_IncrementalMesh.hxx>
#include <TopExp_Explorer.hxx>
#include <TopoDS.hxx>
#include <TopoDS_Face.hxx>
#include <BRep_Tool.hxx>
#include <Poly_Triangulation.hxx>
#include <TopLoc_Location.hxx>
#include <BRepGProp_Face.hxx>
#include <gp_Pnt.hxx>
#include <gp_Vec.hxx>
#include <BRepAdaptor_Curve.hxx>
#include <GCPnts_TangentialDeflection.hxx>

namespace shitcad {

void Scene3D::triangulateShape(const TopoDS_Shape& shape, std::vector<MeshVertex>& out) {
    // Finer tessellation: linear deflection 0.01, angular deflection 0.1 rad (~6°)
    BRepMesh_IncrementalMesh mesher(shape, 0.01, false, 0.1);
    mesher.Perform();

    for (TopExp_Explorer exp(shape, TopAbs_FACE); exp.More(); exp.Next()) {
        const TopoDS_Face& face = TopoDS::Face(exp.Current());
        TopLoc_Location loc;
        Handle(Poly_Triangulation) tri = BRep_Tool::Triangulation(face, loc);
        if (tri.IsNull()) continue;

        bool reversed = (face.Orientation() == TopAbs_REVERSED);

        // Compute smooth normals from the surface at each UV node
        BRepGProp_Face prop(face);
        std::vector<gp_Vec> nodeNormals(tri->NbNodes() + 1); // 1-indexed
        bool hasUV = tri->HasUVNodes();

        if (hasUV) {
            for (int ni = 1; ni <= tri->NbNodes(); ni++) {
                gp_Pnt sp;
                gp_Vec sn;
                gp_Pnt2d uv = tri->UVNode(ni);
                prop.Normal(uv.X(), uv.Y(), sp, sn);
                if (reversed) sn.Reverse();
                nodeNormals[ni] = sn;
            }
        }

        for (int i = 1; i <= tri->NbTriangles(); i++) {
            int n1, n2, n3;
            tri->Triangle(i).Get(n1, n2, n3);

            if (reversed) std::swap(n2, n3);

            gp_Pnt p1 = tri->Node(n1).Transformed(loc.Transformation());
            gp_Pnt p2 = tri->Node(n2).Transformed(loc.Transformation());
            gp_Pnt p3 = tri->Node(n3).Transformed(loc.Transformation());

            float nx1, ny1, nz1, nx2, ny2, nz2, nx3, ny3, nz3;

            if (hasUV) {
                // Use smooth per-vertex normals from surface
                nx1 = (float)nodeNormals[n1].X(); ny1 = (float)nodeNormals[n1].Y(); nz1 = (float)nodeNormals[n1].Z();
                nx2 = (float)nodeNormals[n2].X(); ny2 = (float)nodeNormals[n2].Y(); nz2 = (float)nodeNormals[n2].Z();
                nx3 = (float)nodeNormals[n3].X(); ny3 = (float)nodeNormals[n3].Y(); nz3 = (float)nodeNormals[n3].Z();
            } else {
                // Fallback: flat face normal from cross product
                float e1x = (float)(p2.X() - p1.X()), e1y = (float)(p2.Y() - p1.Y()), e1z = (float)(p2.Z() - p1.Z());
                float e2x = (float)(p3.X() - p1.X()), e2y = (float)(p3.Y() - p1.Y()), e2z = (float)(p3.Z() - p1.Z());
                float fnx = e1y*e2z - e1z*e2y, fny = e1z*e2x - e1x*e2z, fnz = e1x*e2y - e1y*e2x;
                float nlen = std::sqrt(fnx*fnx + fny*fny + fnz*fnz);
                if (nlen > 1e-8f) { fnx /= nlen; fny /= nlen; fnz /= nlen; }
                nx1 = nx2 = nx3 = fnx;
                ny1 = ny2 = ny3 = fny;
                nz1 = nz2 = nz3 = fnz;
            }

            out.push_back({(float)p1.X(), (float)p1.Y(), (float)p1.Z(), nx1, ny1, nz1});
            out.push_back({(float)p2.X(), (float)p2.Y(), (float)p2.Z(), nx2, ny2, nz2});
            out.push_back({(float)p3.X(), (float)p3.Y(), (float)p3.Z(), nx3, ny3, nz3});
        }
    }
}

void Scene3D::extractEdges(const TopoDS_Shape& shape, std::vector<EdgeVertex>& out) {
    for (TopExp_Explorer exp(shape, TopAbs_EDGE); exp.More(); exp.Next()) {
        const TopoDS_Edge& edge = TopoDS::Edge(exp.Current());
        BRepAdaptor_Curve curve(edge);
        GCPnts_TangentialDeflection discretizer(curve, 0.01, 0.2);
        int nbPts = discretizer.NbPoints();
        for (int i = 1; i < nbPts; i++) {
            gp_Pnt p1 = discretizer.Value(i);
            gp_Pnt p2 = discretizer.Value(i + 1);
            out.push_back({(float)p1.X(), (float)p1.Y(), (float)p1.Z()});
            out.push_back({(float)p2.X(), (float)p2.Y(), (float)p2.Z()});
        }
    }
}

void Scene3D::uploadEdges(Body3D& body) {
    if (body.edgeVAO) { glDeleteVertexArrays(1, &body.edgeVAO); body.edgeVAO = 0; }
    if (body.edgeVBO) { glDeleteBuffers(1, &body.edgeVBO); body.edgeVBO = 0; }
    body.edgeVertexCount = 0;

    std::vector<EdgeVertex> edgeVerts;
    if (!body.shape.IsNull())
        extractEdges(body.shape, edgeVerts);
    if (edgeVerts.empty()) return;

    body.edgeVertexCount = (int)edgeVerts.size();
    glGenVertexArrays(1, &body.edgeVAO);
    glGenBuffers(1, &body.edgeVBO);
    glBindVertexArray(body.edgeVAO);
    glBindBuffer(GL_ARRAY_BUFFER, body.edgeVBO);
    glBufferData(GL_ARRAY_BUFFER, edgeVerts.size() * sizeof(EdgeVertex),
                 edgeVerts.data(), GL_STATIC_DRAW);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(EdgeVertex), (void*)0);
    glBindVertexArray(0);
}

void Scene3D::uploadMesh(Body3D& body) {
    if (body.vao) { glDeleteVertexArrays(1, &body.vao); body.vao = 0; }
    if (body.vbo) { glDeleteBuffers(1, &body.vbo); body.vbo = 0; }

    if (body.vertices.empty()) return;

    body.vertexCount = (int)body.vertices.size();

    glGenVertexArrays(1, &body.vao);
    glGenBuffers(1, &body.vbo);

    glBindVertexArray(body.vao);
    glBindBuffer(GL_ARRAY_BUFFER, body.vbo);
    glBufferData(GL_ARRAY_BUFFER,
                 body.vertices.size() * sizeof(MeshVertex),
                 body.vertices.data(), GL_STATIC_DRAW);

    // Position
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, sizeof(MeshVertex), (void*)0);
    // Normal
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, sizeof(MeshVertex),
                          (void*)(3 * sizeof(float)));

    glBindVertexArray(0);
}

void Scene3D::addBody(const TopoDS_Shape& shape) {
    Body3D body;
    body.shape = shape;
    const auto& bc = activeTheme().bodyColor;
    body.colorR = bc[0]; body.colorG = bc[1]; body.colorB = bc[2];
    triangulateShape(shape, body.vertices);
    uploadMesh(body);
    uploadEdges(body);
    bodies_.push_back(std::move(body));
}

void Scene3D::addMeshBody(Body3D&& body) {
    const auto& bc = activeTheme().bodyColor;
    body.colorR = bc[0]; body.colorG = bc[1]; body.colorB = bc[2];
    bodies_.push_back(std::move(body));
}

void Scene3D::replaceBody(int index, const TopoDS_Shape& newShape) {
    if (index < 0 || index >= (int)bodies_.size()) return;
    auto& b = bodies_[index];
    if (b.vao) { glDeleteVertexArrays(1, &b.vao); b.vao = 0; }
    if (b.vbo) { glDeleteBuffers(1, &b.vbo); b.vbo = 0; }
    if (b.edgeVAO) { glDeleteVertexArrays(1, &b.edgeVAO); b.edgeVAO = 0; }
    if (b.edgeVBO) { glDeleteBuffers(1, &b.edgeVBO); b.edgeVBO = 0; }
    b.shape = newShape;
    b.vertices.clear();
    triangulateShape(newShape, b.vertices);
    uploadMesh(b);
    uploadEdges(b);
}

void Scene3D::removeBody(int index) {
    if (index < 0 || index >= (int)bodies_.size()) return;
    auto& b = bodies_[index];
    if (b.vao) glDeleteVertexArrays(1, &b.vao);
    if (b.vbo) glDeleteBuffers(1, &b.vbo);
    if (b.edgeVAO) glDeleteVertexArrays(1, &b.edgeVAO);
    if (b.edgeVBO) glDeleteBuffers(1, &b.edgeVBO);
    bodies_.erase(bodies_.begin() + index);
}

void Scene3D::removeLastBody() {
    if (bodies_.empty()) return;
    auto& b = bodies_.back();
    if (b.vao) glDeleteVertexArrays(1, &b.vao);
    if (b.vbo) glDeleteBuffers(1, &b.vbo);
    if (b.edgeVAO) glDeleteVertexArrays(1, &b.edgeVAO);
    if (b.edgeVBO) glDeleteBuffers(1, &b.edgeVBO);
    bodies_.pop_back();
}

void Scene3D::clear() {
    for (auto& b : bodies_) {
        if (b.vao) glDeleteVertexArrays(1, &b.vao);
        if (b.vbo) glDeleteBuffers(1, &b.vbo);
        if (b.edgeVAO) glDeleteVertexArrays(1, &b.edgeVAO);
        if (b.edgeVBO) glDeleteBuffers(1, &b.edgeVBO);
    }
    bodies_.clear();
}

void Scene3D::render(ShaderProgram& shader, const float* view, const float* proj,
                     const float* eyePos) {
    shader.use();
    shader.setMat4("uView", view);
    shader.setMat4("uProj", proj);
    shader.setVec3("uEyePos", eyePos[0], eyePos[1], eyePos[2]);
    shader.setVec3("uLightDir", 0.3f, 0.8f, 0.5f);
    shader.setFloat("uAlpha", 1.0f);

    for (const auto& body : bodies_) {
        if (!body.visible) continue;
        if (body.vao == 0 || body.vertexCount == 0) continue;
        shader.setVec3("uColor", body.colorR, body.colorG, body.colorB);
        glBindVertexArray(body.vao);
        glDrawArrays(GL_TRIANGLES, 0, body.vertexCount);
    }

    glBindVertexArray(0);
}

void Scene3D::renderEdges(ShaderProgram& shader, const float* view, const float* proj,
                          const float* edgeColor) {
    shader.use();
    shader.setMat4("uView", view);
    shader.setMat4("uProj", proj);

    // Use the grid shader which takes position (attr 0) + color (attr 1)
    // We set per-vertex color via a default vertex attrib
    glVertexAttrib3f(1, edgeColor[0], edgeColor[1], edgeColor[2]);

    for (const auto& body : bodies_) {
        if (!body.visible) continue;
        if (body.edgeVAO == 0 || body.edgeVertexCount == 0) continue;
        glBindVertexArray(body.edgeVAO);
        glDrawArrays(GL_LINES, 0, body.edgeVertexCount);
    }

    glBindVertexArray(0);
}

} // namespace shitcad
