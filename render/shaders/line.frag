#version 330 core
out vec4 FragColor;

in vec3 VertexColor;
uniform vec3 uColor;

void main()
{
    // If uColor is not zero, use it, else use VertexColor
    vec3 c = (length(uColor) > 0.0) ? uColor : VertexColor;
    FragColor = vec4(c, 1.0);
}
