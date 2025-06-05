#version 330 core

in vec3 fragPos;
in vec3 fragNormal;

out vec4 FragColor;

uniform vec3 lightPos;
uniform vec3 viewPos;
uniform vec3 lightColor;
uniform vec3 objectColor;  // You can set this to white in the C++ code

void main()
{
    vec3 normal = normalize(fragNormal);

    // Ambient component
    float ambientStrength = 0.3;
    vec3 ambient = ambientStrength * lightColor;

    // Diffuse component
    vec3 lightDir = normalize(lightPos - fragPos);
    float diff = max(dot(normal, lightDir), 0.0);
    vec3 diffuse = diff * lightColor;

    // Specular component (soft specular)
    float specularStrength = 0.3;
    vec3 viewDir = normalize(viewPos - fragPos);
    vec3 reflectDir = reflect(-lightDir, normal);
    float spec = pow(max(dot(viewDir, reflectDir), 0.0), 16);
    vec3 specular = specularStrength * spec * lightColor;

    // Combine all components
    vec3 result = (ambient + diffuse + specular) * objectColor;
    
    // Apply a slight blur to soften the look
    result = mix(result, vec3(1.0), 0.2); // blending with white for soft look

    FragColor = vec4(result, 1.0);
}
