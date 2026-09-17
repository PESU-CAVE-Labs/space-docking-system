#pragma once

#include "GLContext.hpp"
#include "Camera.hpp"
#include "SceneObject.hpp"
#include "Shader.hpp"
#include "MeshFactory.hpp"
#include <vector>
#include <memory>

class Renderer {
public:
    Renderer(int width, int height, const std::string& title);
    
    GLContext& GetContext() { return m_context; }
    
    void AddObject(std::shared_ptr<SceneObject> obj) { m_objects.push_back(obj); }
    void ClearObjects() { m_objects.clear(); }

    void Render(const Camera& camera);

private:
    GLContext m_context;
    std::vector<std::shared_ptr<SceneObject>> m_objects;
};
