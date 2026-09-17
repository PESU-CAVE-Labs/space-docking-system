#pragma once

#include <glad/glad.h>
#include <vector>
#include "math/Vec3.hpp"

struct Vertex {
    Vec3 Position;
    Vec3 Normal;
};

class Mesh {
public:
    Mesh(const std::vector<Vertex>& vertices, const std::vector<unsigned int>& indices, GLenum drawMode = GL_TRIANGLES);
    ~Mesh();
    
    // Disable copy
    Mesh(const Mesh&) = delete;
    Mesh& operator=(const Mesh&) = delete;

    // Enable move
    Mesh(Mesh&& other) noexcept;
    Mesh& operator=(Mesh&& other) noexcept;

    void Draw() const;

private:
    void SetupMesh();

    std::vector<Vertex> m_vertices;
    std::vector<unsigned int> m_indices;
    GLuint m_vao, m_vbo, m_ebo;
    GLenum m_drawMode;
};
