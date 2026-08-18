#version 440
// Seam-placement pre-pass 1c: one column of the DP accumulation.
// DP recurrence (processing columns left to right, u direction):
//   M[u, v] = C[u, v] + min(M[u-1, v-1], M[u-1, v], M[u-1, v+1])
// v clamps at boundaries (theta is not periodic).
// Draw W times (once per column) with scissor set to column u_col.

layout(location = 0) in vec2 v_texCoord;
layout(location = 0) out float outCost;

layout(binding = 0) uniform sampler2D u_cost;     // RGBA16F: cost volume
layout(binding = 1) uniform sampler2D u_dpIn;     // R16F: previous cumulative cost

uniform int u_col;       // current column being processed
uniform int u_width;     // W
uniform int u_height;    // H

void main() {
    ivec2 coord = ivec2(gl_FragCoord.xy);
    int v = coord.y;
    int H = u_height;

    float cost = texelFetch(u_cost, coord, 0).r;

    if (u_col == 0) {
        outCost = cost;
    } else {
        int prevU = u_col - 1;
        float pL = texelFetch(u_dpIn, ivec2(prevU, clamp(v - 1, 0, H - 1)), 0).r;
        float pC = texelFetch(u_dpIn, ivec2(prevU, v), 0).r;
        float pR = texelFetch(u_dpIn, ivec2(prevU, clamp(v + 1, 0, H - 1)), 0).r;
        outCost = cost + min(pL + 0.1, min(pC, pR + 0.1));
    }
}
