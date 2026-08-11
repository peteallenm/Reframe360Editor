#version 440
// Optical-flow stitching pass 1: sample the stacked-YUV luma at every texel of
// the camera-native blend band (phi in [0,2pi) on the u axis, theta in
// [bandTheta0, bandTheta1] on the v axis) for BOTH fisheye lenses, via MRT.
//
// The band parameterization (see OpticalFlow.md):
//     phi   = u * 2*PI            (u in [0,1), wraps at 1.0)
//     theta = mix(theta0, theta1, v)   (v in [0,1], does NOT wrap)
//     ray   = (sin(theta)*cos(phi), sin(theta)*sin(phi), -cos(theta))
//
// The fisheye project math below is a byte-for-byte duplicate of project.frag,
// and the stacked-frame remap (front = top half, rear = bottom half) matches
// project.frag exactly: front at (uv.x, uv.y*0.5), rear at (uv.x, uv.y*0.5+0.5).

layout(location = 0) in vec2 v_texCoord;
layout(location = 0) out float outFront;   // band_front (R8)
layout(location = 1) out float outRear;    // band_rear  (R8)

layout(binding = 0) uniform sampler2D u_texY;

// ---- Calibration (identical semantics to project.frag's Uniforms block) ----
uniform vec2  u_frontCenter;
uniform float u_frontRadius;
uniform float u_frontK1;
uniform float u_frontK2;
uniform int   u_frontHFlip;
uniform vec2  u_rearCenter;
uniform float u_rearRadius;
uniform float u_rearK1;
uniform float u_rearK2;
uniform int   u_rearHFlip;
uniform float u_frontRotation;
uniform float u_rearRotation;
uniform float u_bandTheta0;
uniform float u_bandTheta1;

const float PI = 3.14159265359;

vec2 rotateVec2(vec2 v, float ang) {
    float c = cos(ang);
    float s = sin(ang);
    return vec2(v.x * c - v.y * s, v.x * s + v.y * c);
}

void main() {
    // phi in [-pi, pi) so it matches project.frag's atan2(y,x) and its
    // u_band = mod(phi + PI, 2*PI) / (2*PI) band-coordinate mapping.
    float phi = v_texCoord.x * 2.0 * PI - PI;
    float theta = mix(u_bandTheta0, u_bandTheta1, v_texCoord.y);
    vec3 ray = vec3(sin(theta) * cos(phi), sin(theta) * sin(phi), -cos(theta));

    // ---- Front lens (identical math to project.frag) ----
    float r_front = theta / (PI * 0.5);
    float r_front_dist = r_front * (1.0 + u_frontK1 * r_front * r_front + u_frontK2 * r_front * r_front * r_front * r_front);
    vec2 frontOff = rotateVec2(vec2(cos(phi), sin(phi)), radians(u_frontRotation)) * r_front_dist * u_frontRadius;
    if (mod(float(u_frontHFlip), 2.0) != 0.0)
        frontOff.x = -frontOff.x;
    vec2 uv_front = u_frontCenter + frontOff;

    // ---- Rear lens (identical math to project.frag) ----
    float theta_rear = PI - theta;
    float r_rear = theta_rear / (PI * 0.5);
    float r_rear_dist = r_rear * (1.0 + u_rearK1 * r_rear * r_rear + u_rearK2 * r_rear * r_rear * r_rear * r_rear);
    vec2 rearOff = rotateVec2(vec2(cos(phi), sin(phi)), radians(u_rearRotation)) * r_rear_dist * u_rearRadius;
    if (mod(float(u_rearHFlip), 2.0) != 0.0)
        rearOff.x = -rearOff.x;
    vec2 uv_rear = u_rearCenter + rearOff;

    // Stacked-frame YUV remap: front occupies the top half, rear the bottom.
    vec2 uv_front_tex = vec2(uv_front.x, uv_front.y * 0.5);
    vec2 uv_rear_tex  = vec2(uv_rear.x,  uv_rear.y  * 0.5 + 0.5);

    outFront = texture(u_texY, uv_front_tex).r;
    outRear  = texture(u_texY, uv_rear_tex).r;
}
