#include "Mesh.hpp"

Mesh::Mesh(const std::vector<Vertex>& vertices, const std::vector<unsigned int>& indices, GLenum drawMode)
    : m_vertices(vertices), m_indices(indices), m_vao(0), m_vbo(0), m_ebo(0), m_drawMode(drawMode) {
    SetupMesh();
}

Mesh::~Mesh() {
    if (m_vao != 0) glDeleteVertexArrays(1, &m_vao);
    if (m_vbo != 0) glDeleteBuffers(1, &m_vbo);
    if (m_ebo != 0) glDeleteBuffers(1, &m_ebo);
}

Mesh::Mesh(Mesh&& other) noexcept
    : m_vertices(std::move(other.m_vertices)),
      m_indices(std::move(other.m_indices)),
      m_vao(other.m_vao),
      m_vbo(other.m_vbo),
      m_ebo(other.m_ebo),
      m_drawMode(other.m_drawMode) {
    other.m_vao = 0;
    other.m_vbo = 0;
    other.m_ebo = 0;
}

Mesh& Mesh::operator=(Mesh&& other) noexcept {
    if (this != &other) {
        if (m_vao != 0) glDeleteVertexArrays(1, &m_vao);
        if (m_vbo != 0) glDeleteBuffers(1, &m_vbo);
        if (m_ebo != 0) glDeleteBuffers(1, &m_ebo);

        m_vertices = std::move(other.m_vertices);
        m_indices = std::move(other.m_indices);
        m_vao = other.m_vao;
        m_vbo = other.m_vbo;
        m_ebo = other.m_ebo;
        m_drawMode = other.m_drawMode;

        other.m_vao = 0;
        other.m_vbo = 0;
        other.m_ebo = 0;
    }
    return *this;
}

void Mesh::SetupMesh() {
    glGenVertexArrays(1, &m_vao);
    glGenBuffers(1, &m_vbo);
    glGenBuffers(1, &m_ebo);
  
    glBindVertexArray(m_vao);
    
    glBindBuffer(GL_ARRAY_BUFFER, m_vbo);
    glBufferData(GL_ARRAY_BUFFER, m_vertices.size() * sizeof(Vertex), &m_vertices[0], GL_STATIC_DRAW);  

    if (!m_indices.empty()) {
        glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, m_ebo);
        glBufferData(GL_ELEMENT_ARRAY_BUFFER, m_indices.size() * sizeof(unsigned int), &m_indices[0], GL_STATIC_DRAW);
    }

    // Vertex positions
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(0, 3, GL_DOUBLE, GL_FALSE, sizeof(Vertex), (void*)0);
    
    // Vertex normals
    glEnableVertexAttribArray(1);
    glVertexAttribPointer(1, 3, GL_DOUBLE, GL_FALSE, sizeof(Vertex), (void*)offsetof(Vertex, Normal));

    glBindVertexArray(0);
}

void Mesh::Draw() const {
    glBindVertexArray(m_vao);
    if (!m_indices.empty()) {
        glDrawElements(m_drawMode, m_indices.size(), GL_UNSIGNED_INT, 0);
    } else {
        glDrawArrays(m_drawMode, 0, m_vertices.size());
    }
    glBindVertexArray(0);
}
