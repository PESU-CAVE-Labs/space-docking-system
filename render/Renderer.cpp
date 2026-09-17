#include "Renderer.hpp"

Renderer::Renderer(int width, int height, const std::string& title) 
    : m_context(width, height, title) {
    glClearColor(0.05f, 0.05f, 0.1f, 1.0f); // Dark space-like background
}

void Renderer::Render(const Camera& camera) {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    
    glm::mat4 vp = camera.GetViewProj();
    
    for (const auto& obj : m_objects) {
        obj->Draw(vp);
    }
}
