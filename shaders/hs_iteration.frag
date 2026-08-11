#version 440
// Optical-flow stitching pass 3: one Horn-Schunck Jacobi relaxation step.
// Ping-pongs between two RGBA16F flow textures; the neighbourhood average
// (u-bar, v-bar) wraps in u (phi is periodic) and clamps in v (theta is not),
// via the texture wrap modes.

layout(location = 0) in vec2 v_texCoord;
layout(location = 0) out vec2 outFlow;    // (du, dv) -> R,G channels

layout(binding = 0) uniform sampler2D u_gradients;   // RGBA16F: Ix, Iy, It, _
layout(binding = 1) uniform sampler2D u_flowIn;      // RGBA16F: current flow
uniform float u_alpha;

void main() {
    vec4 g = texture(u_gradients, v_texCoord);
    float Ix = g.x;
    float Iy = g.y;
    float It = g.z;

    vec2 fL = textureOffset(u_flowIn, v_texCoord, ivec2(-1, 0)).rg;
    vec2 fR = textureOffset(u_flowIn, v_texCoord, ivec2( 1, 0)).rg;
    vec2 fD = textureOffset(u_flowIn, v_texCoord, ivec2(0, -1)).rg;
    vec2 fU = textureOffset(u_flowIn, v_texCoord, ivec2(0,  1)).rg;
    vec2 avg = 0.25 * (fL + fR + fD + fU);

    float denom = u_alpha * u_alpha + Ix * Ix + Iy * Iy;
    float d = (Ix * avg.x + Iy * avg.y + It) / denom;

    outFlow = vec2(avg.x - Ix * d, avg.y - Iy * d);
}
