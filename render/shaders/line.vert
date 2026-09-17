#version 330 core
layout (location = 0) in vec3 aPos;
layout (location = 1) in vec3 aColor; // For the axis triad we mapped color to normals buffer, but we can also use a uniform.

out vec3 VertexColor;

uniform mat4 uMVP;
uniform vec3 uColor; // If we use uniform color

void main()
{
    VertexColor = aColor; // using normal as color
    gl_Position = uMVP * vec4(aPos, 1.0);
}
