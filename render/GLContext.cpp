#include "GLContext.hpp"
#include <iostream>
#include <stdexcept>

GLContext::GLContext(int width, int height, const std::string& title) {
    if (!glfwInit()) {
        throw std::runtime_error("Failed to initialize GLFW");
    }

    glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
    glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
    glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
#ifdef __APPLE__
    glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
#endif

    m_window = glfwCreateWindow(width, height, title.c_str(), NULL, NULL);
    if (m_window == NULL) {
        glfwTerminate();
        throw std::runtime_error("Failed to create GLFW window");
    }
    
    glfwMakeContextCurrent(m_window);
    
    if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
        throw std::runtime_error("Failed to initialize GLAD");
    }
    
    glEnable(GL_DEPTH_TEST);
}

GLContext::~GLContext() {
    glfwDestroyWindow(m_window);
    glfwTerminate();
}

void GLContext::PollEvents() {
    glfwPollEvents();
}

void GLContext::SwapBuffers() {
    glfwSwapBuffers(m_window);
}

bool GLContext::ShouldClose() const {
    return glfwWindowShouldClose(m_window);
}
