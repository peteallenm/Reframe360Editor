#version 440
// Optical-flow stitching pass 2: compute the Horn-Schunck spatio-temporal
// gradients over the band. Ix/Iy are central differences of the average image
// (front+rear)/2, It = front - rear. The u axis is periodic (phi wraps), the
// v axis clamps (theta does not wrap) — handled by the texture wrap modes.

layout(location = 0) in vec2 v_texCoord;
layout(location = 0) out vec4 outGrad;   // (Ix, Iy, It, _) -> RGBA16F

layout(binding = 0) uniform sampler2D u_bandFront;
layout(binding = 1) uniform sampler2D u_bandRear;

float avgAt(ivec2 off) {
    // Scale the luma from [0,1] to [0,255] so the Horn-Schunck smoothness
    // weight (alpha=10) matches the standard formulation. With [0,1] luma the
    // gradients are O(0.001-0.01), alpha^2=100 dominates the denominator, and
    // the flow stays essentially zero.
    return 255.0 * 0.5 * (textureOffset(u_bandFront, v_texCoord, off).r
                       + textureOffset(u_bandRear,  v_texCoord, off).r);
}

void main() {
    float avgC = avgAt(ivec2(0, 0));
    float avgL = avgAt(ivec2(-1, 0));
    float avgR = avgAt(ivec2( 1, 0));
    float avgD = avgAt(ivec2(0, -1));
    float avgU = avgAt(ivec2(0,  1));

    float Ix = 0.5 * (avgR - avgL);
    float Iy = 0.5 * (avgU - avgD);
    // It = rear - front so the resulting flow is the front->rear displacement
    // (project.frag adds the flow when reprojecting the rear sample).
    float It = 255.0 * (texture(u_bandRear,  v_texCoord).r
                      - texture(u_bandFront, v_texCoord).r);

    outGrad = vec4(Ix, Iy, It, 0.0);
}
