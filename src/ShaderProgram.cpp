#include "ShaderProgram.h"
#include <cstdio>

namespace shitcad {

ShaderProgram::~ShaderProgram() {
    if (program_) glDeleteProgram(program_);
}

ShaderProgram::ShaderProgram(ShaderProgram&& other) noexcept : program_(other.program_) {
    other.program_ = 0;
}

ShaderProgram& ShaderProgram::operator=(ShaderProgram&& other) noexcept {
    if (this != &other) {
        if (program_) glDeleteProgram(program_);
        program_ = other.program_;
        other.program_ = 0;
    }
    return *this;
}

GLuint ShaderProgram::compileShader(GLenum type, const char* src) {
    GLuint shader = glCreateShader(type);
    glShaderSource(shader, 1, &src, nullptr);
    glCompileShader(shader);

    GLint ok = 0;
    glGetShaderiv(shader, GL_COMPILE_STATUS, &ok);
    if (!ok) {
        char log[512];
        glGetShaderInfoLog(shader, sizeof(log), nullptr, log);
        fprintf(stderr, "Shader compile error: %s\n", log);
        glDeleteShader(shader);
        return 0;
    }
    return shader;
}

bool ShaderProgram::compile(const char* vertexSrc, const char* fragmentSrc) {
    GLuint vs = compileShader(GL_VERTEX_SHADER, vertexSrc);
    if (!vs) return false;

    GLuint fs = compileShader(GL_FRAGMENT_SHADER, fragmentSrc);
    if (!fs) { glDeleteShader(vs); return false; }

    program_ = glCreateProgram();
    glAttachShader(program_, vs);
    glAttachShader(program_, fs);
    glLinkProgram(program_);

    glDeleteShader(vs);
    glDeleteShader(fs);

    GLint ok = 0;
    glGetProgramiv(program_, GL_LINK_STATUS, &ok);
    if (!ok) {
        char log[512];
        glGetProgramInfoLog(program_, sizeof(log), nullptr, log);
        fprintf(stderr, "Shader link error: %s\n", log);
        glDeleteProgram(program_);
        program_ = 0;
        return false;
    }
    return true;
}

void ShaderProgram::use() const {
    glUseProgram(program_);
}

void ShaderProgram::setMat4(const char* name, const float* mat) const {
    glUniformMatrix4fv(glGetUniformLocation(program_, name), 1, GL_FALSE, mat);
}

void ShaderProgram::setVec3(const char* name, float x, float y, float z) const {
    glUniform3f(glGetUniformLocation(program_, name), x, y, z);
}

void ShaderProgram::setFloat(const char* name, float val) const {
    glUniform1f(glGetUniformLocation(program_, name), val);
}

} // namespace shitcad
