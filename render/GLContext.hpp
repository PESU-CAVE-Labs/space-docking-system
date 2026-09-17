#pragma once

#include <glad/glad.h>
#include <GLFW/glfw3.h>
#include <string>

class GLContext {
public:
    GLContext(int width, int height, const std::string& title);
    ~GLContext();

    void PollEvents();
    void SwapBuffers();
    bool ShouldClose() const;
    GLFWwindow* Handle() const { return m_window; }

    // Returns current framebuffer pixel dimensions (correct for HiDPI / fullscreen)
    void GetFramebufferSize(int& width, int& height) const {
        glfwGetFramebufferSize(m_window, &width, &height);
    }

private:
    GLFWwindow* m_window = nullptr;
};
