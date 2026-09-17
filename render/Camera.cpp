#include "Camera.hpp"

Camera::Camera(const Vec3& pos, const Vec3& target, const Vec3& up, float fov, float aspect)
    : m_position(pos), m_target(target), m_up(up), m_fov(fov), m_aspect(aspect) {
}

glm::mat4 Camera::GetViewProj() const {
    glm::mat4 view = glm::lookAt(
        glm::vec3(m_position.x, m_position.y, m_position.z),
        glm::vec3(m_target.x, m_target.y, m_target.z),
        glm::vec3(m_up.x, m_up.y, m_up.z)
    );
    
    glm::mat4 proj = glm::perspective(glm::radians(m_fov), m_aspect, m_near, m_far);
    
    return proj * view;
}
