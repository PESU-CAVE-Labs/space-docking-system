#pragma once

#include "math/Vec3.hpp"
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

class Camera {
public:
    Camera(const Vec3& pos, const Vec3& target, const Vec3& up, float fov, float aspect);

    void SetPosition(const Vec3& pos) { m_position = pos; }
    void SetTarget(const Vec3& tgt) { m_target = tgt; }
    void SetAspect(float aspect) { m_aspect = aspect; }

    glm::mat4 GetViewProj() const;
    Vec3 GetPosition() const { return m_position; }

private:
    Vec3 m_position;
    Vec3 m_target;
    Vec3 m_up;
    float m_fov;
    float m_aspect;
    float m_near = 0.1f;
    float m_far = 100000000.0f; // Big enough for earth
};
