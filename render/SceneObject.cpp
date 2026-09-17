#include "SceneObject.hpp"
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>

SceneObject::SceneObject(std::shared_ptr<Mesh> mesh, std::shared_ptr<Shader> shader)
    : m_mesh(std::move(mesh)), m_shader(std::move(shader)) {
}

void SceneObject::Draw(const glm::mat4& viewProj) const {
    if (!m_mesh || !m_shader) return;

    // Build model matrix
    glm::mat4 model = glm::mat4(1.0f);
    model = glm::translate(model, glm::vec3(m_position.x, m_position.y, m_position.z));
    
    // GLM constructor is glm::quat(w, x, y, z); our Quaternion stores (q1, q2, q3, q4) where q4 is scalar w
    glm::quat q(static_cast<float>(m_rotation.q4),
                 static_cast<float>(m_rotation.q1),
                 static_cast<float>(m_rotation.q2),
                 static_cast<float>(m_rotation.q3));
    model = model * glm::mat4_cast(q);
    
    model = glm::scale(model, glm::vec3(m_scale.x, m_scale.y, m_scale.z));

    glm::mat4 mvp = viewProj * model;

    m_shader->Use();
    m_shader->SetMat4("uMVP", mvp);
    m_shader->SetMat4("uModel", model);
    m_shader->SetVec3("uColor", m_color);

    m_mesh->Draw();
}
