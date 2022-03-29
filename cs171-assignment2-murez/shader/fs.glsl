#version 330 core

out vec4 FragColor;

in vec3 Normal;
in vec3 FragPos;

uniform vec3 lightPos;
uniform vec3 lightColor;
uniform vec3 objectColor;
uniform vec3 viewPos;
vec3 calculatePointLight(vec3 light_pos, vec3 light_color, vec3 frag_pos, vec3 normal, vec3 view_pos);
void main()
{
    vec3 result = calculatePointLight(lightPos, lightColor, FragPos, Normal, viewPos);
    FragColor = vec4(result, 1.0);
}

vec3 calculatePointLight(vec3 light_pos, vec3 light_color, vec3 frag_pos, vec3 normal, vec3 view_pos){
    // Ambient Lighting
    float ambientStrength = 0.1;
    vec3 ambient = ambientStrength * light_color;

    // Diffuse Lighting
    vec3 norm = normalize(normal);
    vec3 lightDir = normalize(light_pos - frag_pos);
    float diff = max(dot(norm, lightDir), 0.0);
    vec3 diffuse = diff * light_color;

    // Specular Lighting
    float specularStrength = 0.5;
    vec3 viewDir = normalize(view_pos - frag_pos);
    vec3 reflectDir = reflect(-lightDir, norm);
    float spec = pow(max(dot(viewDir, reflectDir), 0.0), 32);
    vec3 specular = specularStrength * spec * light_color;

    return (ambient + diffuse + specular) * objectColor;
}