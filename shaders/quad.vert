#version 440

layout(location = 0) in vec2 a_position;
layout(location = 1) in vec2 a_texCoord;
layout(location = 0) out vec2 v_texCoord;

layout(std140, binding = 0) uniform Uniforms {
    mat4 qt_Matrix;
} ubuf;

void main() {
    gl_Position = ubuf.qt_Matrix * vec4(a_position, 0.0, 1.0);
    v_texCoord = a_texCoord;
}
