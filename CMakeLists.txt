cmake_minimum_required(VERSION 3.20)
project(docking_sim VERSION 1.0 LANGUAGES C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
add_compile_definitions(_USE_MATH_DEFINES)

# Dependencies
include(FetchContent)

# GLFW
FetchContent_Declare(
    glfw
    GIT_REPOSITORY https://github.com/glfw/glfw.git
    GIT_TAG        3.3.8
)
set(GLFW_BUILD_DOCS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_TESTS OFF CACHE BOOL "" FORCE)
set(GLFW_BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(glfw)

# GLM
FetchContent_Declare(
    glm
    GIT_REPOSITORY https://github.com/g-truc/glm.git
    GIT_TAG        master
)
FetchContent_MakeAvailable(glm)

# ImGui
FetchContent_Declare(
    imgui
    GIT_REPOSITORY https://github.com/ocornut/imgui.git
    GIT_TAG        docking
)
FetchContent_MakeAvailable(imgui)

# ImPlot
FetchContent_Declare(
    implot
    GIT_REPOSITORY https://github.com/epezent/implot.git
    GIT_TAG        master
)
FetchContent_MakeAvailable(implot)

# GoogleTest
FetchContent_Declare(
  googletest
  GIT_REPOSITORY https://github.com/google/googletest.git
  GIT_TAG        v1.14.0
)
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

# GLAD library (static)
add_library(glad third_party/glad/src/glad.c)
target_include_directories(glad PUBLIC third_party/glad/include)

# ImGui and ImPlot library
add_library(imgui_lib STATIC
    ${imgui_SOURCE_DIR}/imgui.cpp
    ${imgui_SOURCE_DIR}/imgui_demo.cpp
    ${imgui_SOURCE_DIR}/imgui_draw.cpp
    ${imgui_SOURCE_DIR}/imgui_tables.cpp
    ${imgui_SOURCE_DIR}/imgui_widgets.cpp
    ${imgui_SOURCE_DIR}/backends/imgui_impl_glfw.cpp
    ${imgui_SOURCE_DIR}/backends/imgui_impl_opengl3.cpp
    ${implot_SOURCE_DIR}/implot.cpp
    ${implot_SOURCE_DIR}/implot_items.cpp
    ${implot_SOURCE_DIR}/implot_demo.cpp
)
target_include_directories(imgui_lib PUBLIC
    ${imgui_SOURCE_DIR}
    ${imgui_SOURCE_DIR}/backends
    ${implot_SOURCE_DIR}
)
target_link_libraries(imgui_lib PUBLIC glfw glad)

# Includes for project
include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/engine
    ${CMAKE_CURRENT_SOURCE_DIR}/engine/math
    ${CMAKE_CURRENT_SOURCE_DIR}/render
    ${CMAKE_CURRENT_SOURCE_DIR}/ui
    ${CMAKE_CURRENT_SOURCE_DIR}/io
    ${CMAKE_CURRENT_SOURCE_DIR}/app
)

# Application executable
add_executable(docking_sim
    app/main.cpp
    render/GLContext.cpp
    render/Shader.cpp
    render/Mesh.cpp
    render/MeshFactory.cpp
    render/SceneObject.cpp
    render/Camera.cpp
    render/Renderer.cpp
    engine/math/Quaternion.cpp
    engine/math/Euler.cpp
    engine/frames/FrameTransformer.cpp
    engine/dynamics/Gravity.cpp
    engine/dynamics/RigidBody.cpp
    engine/dynamics/Integrator.cpp
    engine/DynamicsEngine.cpp
    engine/dynamics/disturbances/AtmosphericDrag.cpp
    engine/dynamics/disturbances/GravityGradient.cpp
    engine/dynamics/disturbances/J2Perturbation.cpp
    engine/contact/ContactSpringModel.cpp
    engine/logging/DataLogger.cpp
    ${GLAD_SOURCE}
)

target_link_libraries(docking_sim PRIVATE glad glfw glm imgui_lib)

if (APPLE)
    target_link_libraries(docking_sim PRIVATE "-framework OpenGL" "-framework Cocoa" "-framework IOKit" "-framework CoreVideo")
    target_compile_definitions(docking_sim PRIVATE GL_SILENCE_DEPRECATION)
elseif (WIN32)
    target_link_libraries(docking_sim PRIVATE opengl32)
endif()

# Tests
enable_testing()
add_executable(docking_sim_tests
    tests/test_math.cpp
    tests/test_frames.cpp
    tests/test_dynamics.cpp
    engine/math/Quaternion.cpp
    engine/math/Euler.cpp
    engine/frames/FrameTransformer.cpp
    engine/dynamics/Gravity.cpp
    engine/dynamics/RigidBody.cpp
    engine/dynamics/Integrator.cpp
    engine/DynamicsEngine.cpp
    engine/dynamics/disturbances/AtmosphericDrag.cpp
    engine/dynamics/disturbances/GravityGradient.cpp
    engine/dynamics/disturbances/J2Perturbation.cpp
    engine/contact/ContactSpringModel.cpp
    tests/test_contact.cpp
)
# Make sure gtest and gtest_main are linked
target_link_libraries(docking_sim_tests PRIVATE GTest::gtest_main glm)
add_test(NAME docking_sim_tests COMMAND docking_sim_tests)
