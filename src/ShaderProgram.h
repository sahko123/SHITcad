#pragma once
#include <glad/gl.h>
#include <string>

namespace shitcad {

class ShaderProgram {
public:
    ShaderProgram() = default;
    ~ShaderProgram();

    ShaderProgram(const ShaderProgram&) = delete;
    ShaderProgram& operator=(const ShaderProgram&) = delete;
    ShaderProgram(ShaderProgram&& other) noexcept;
    ShaderProgram& operator=(ShaderProgram&& other) noexcept;

    bool compile(const char* vertexSrc, const char* fragmentSrc);
    void use() const;
    GLuint id() const { return program_; }

    void setMat4(const char* name, const float* mat) const;
    void setVec3(const char* name, float x, float y, float z) const;
    void setFloat(const char* name, float val) const;

private:
    GLuint program_ = 0;
    static GLuint compileShader(GLenum type, const char* src);
};

} // namespace shitcad
