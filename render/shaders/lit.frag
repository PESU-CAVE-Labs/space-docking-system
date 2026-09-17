#version 330 core
out vec4 FragColor;

in vec3 FragPos;
in vec3 Normal;

uniform vec3 uColor;
// Simple directional light simulating the sun
const vec3 sunDir = normalize(vec3(1.0, 0.5, 0.2));
const vec3 lightColor = vec3(1.0, 1.0, 1.0);
const float ambientStrength = 0.2;

void main()
{
    // ambient
    vec3 ambient = ambientStrength * lightColor;
  	
    // diffuse 
    vec3 norm = normalize(Normal);
    float diff = max(dot(norm, sunDir), 0.0);
    vec3 diffuse = diff * lightColor;
            
    vec3 result = (ambient + diffuse) * uColor;
    FragColor = vec4(result, 1.0);
}
