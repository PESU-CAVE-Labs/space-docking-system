#pragma once

#include <glad/glad.h>
#include <string>
#include "math/Mat4.hpp"
#include "math/Vec3.hpp"
#include <glm/glm.hpp>
#include <glm/gtc/type_ptr.hpp>

class Shader {
public:
    Shader(const std::string& vertPath, const std::string& fragPath);
    ~Shader();

    void Use() const;
    void SetMat4(const std::string& name, const glm::mat4& mat) const;
    void SetVec3(const std::string& name, const glm::vec3& vec) const;
    void SetVec3(const std::string& name, const Vec3& vec) const;
    
private:
    GLuint m_program;
};
