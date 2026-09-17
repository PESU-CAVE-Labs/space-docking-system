#pragma once

#include "Mesh.hpp"
#include "math/Vec3.hpp"
#include "math/Mat4.hpp"
#include "math/Quaternion.hpp"
#include "Shader.hpp"
#include <memory>

class SceneObject {
public:
    SceneObject(std::shared_ptr<Mesh> mesh, std::shared_ptr<Shader> shader);

    void SetPosition(const Vec3& pos) { m_position = pos; }
    void SetRotation(const Quaternion& q) { m_rotation = q; }
    void SetScale(const Vec3& scale) { m_scale = scale; }
    void SetColor(const Vec3& color) { m_color = color; }

    void Draw(const glm::mat4& viewProj) const;

private:
    std::shared_ptr<Mesh> m_mesh;
    std::shared_ptr<Shader> m_shader;
    
    Vec3 m_position{0,0,0};
    Quaternion m_rotation; // Identity
    Vec3 m_scale{1,1,1};
    Vec3 m_color{1,1,1};
};
