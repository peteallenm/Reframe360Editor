// Reframe360 Editor -- 360 video stabiliser and stitcher for dual-fisheye footage.
// Copyright (C) 2026 Peter Allen
// SPDX-License-Identifier: GPL-3.0-or-later
//
// This program is free software under the GNU General Public License, version
// 3 or (at your option) any later version; see LICENSE for the full text.
// It is distributed in the hope that it will be useful, but WITHOUT ANY
// WARRANTY, without even the implied warranty of MERCHANTABILITY or FITNESS
// FOR A PARTICULAR PURPOSE.

#version 440
// Seam-placement pre-pass 1b: per-texel cost C(phi,theta) for the DP seam
// carving. Combines front/rear disagreement, Harris corner strength, and
// block-match SAD to reward seam placement where features agree.
//
// Output RGBA16F: cost in .r, Harris front in .g, Harris rear in .b

layout(location = 0) in vec2 v_texCoord;
layout(location = 0) out vec4 outCost;

layout(binding = 0) uniform sampler2D u_bandFront;  // R8
layout(binding = 1) uniform sampler2D u_bandRear;   // R8

uniform float u_harrisK;            // 0.04
uniform float u_blockMatchWeight;   // 1.0
uniform int u_blockMatchRadius;     // 4
uniform float u_equatorBiasWeight;  // 0.5

// Harris response at a given texel coordinate (using 3x3 Sobel)
float harrisAt(sampler2D tex, ivec2 coord) {
    float tl = texelFetchOffset(tex, coord, 0, ivec2(-1, 1)).r;
    float t  = texelFetchOffset(tex, coord, 0, ivec2( 0, 1)).r;
    float tr = texelFetchOffset(tex, coord, 0, ivec2( 1, 1)).r;
    float l  = texelFetchOffset(tex, coord, 0, ivec2(-1, 0)).r;
    float r  = texelFetchOffset(tex, coord, 0, ivec2( 1, 0)).r;
    float bl = texelFetchOffset(tex, coord, 0, ivec2(-1,-1)).r;
    float b  = texelFetchOffset(tex, coord, 0, ivec2( 0,-1)).r;
    float br = texelFetchOffset(tex, coord, 0, ivec2( 1,-1)).r;

    // Sobel gradients
    float Ix = (-tl + tr - 2.0*l + 2.0*r - bl + br) * 0.125;
    float Iy = (tl + 2.0*t + tr - bl - 2.0*b - br) * 0.125;

    float Ixx = Ix * Ix;
    float Iyy = Iy * Iy;
    float Ixy = Ix * Iy;

    float det = Ixx * Iyy - Ixy * Ixy;
    float trace = Ixx + Iyy;
    return det - u_harrisK * trace * trace;
}

void main() {
    ivec2 coord = ivec2(gl_FragCoord.xy);
    ivec2 texSize = textureSize(u_bandFront, 0);

    float front = texelFetch(u_bandFront, coord, 0).r;
    float rear  = texelFetch(u_bandRear,  coord, 0).r;

    // Penalize regions where either lens is outside its fisheye (black).
    // This prevents the seam from drifting to band edges where one lens
    // has no coverage.
    float validFront = step(0.06, front);
    float validRear  = step(0.06, rear);
    float validityPenalty = (1.0 - validFront * validRear) * 1000.0;

    // Harris response of each lens
    float hFront = harrisAt(u_bandFront, coord);
    float hRear  = harrisAt(u_bandRear,  coord);
    float F = max(0.5 * (hFront + hRear), 0.0);

    // Block-match SAD: shift rear by +/-radius in theta (v direction),
    // take min SAD over the range
    float minSAD = 1e30;
    const int uRadius = 4;  // smaller horizontal search radius
    for (int du = -uRadius; du <= uRadius; ++du) {
        for (int dv = -u_blockMatchRadius; dv <= u_blockMatchRadius; ++dv) {
            ivec2 shifted = coord + ivec2(du, dv);
            shifted.x = clamp(shifted.x, 0, texSize.x - 1);
            shifted.y = clamp(shifted.y, 0, texSize.y - 1);
            float rearShifted = texelFetch(u_bandRear, shifted, 0).r;
            minSAD = min(minSAD, abs(front - rearShifted));
        }
    }

    // Bias toward the equator (v=0.5) so the seam stays in the region
    // where both lenses have good coverage.
    float equatorBias = abs(v_texCoord.y - 0.5) * 2.0 * u_equatorBiasWeight;

    // Combined cost: lower block-match SAD + lower Harris response =
    // lower cost, so the seam is pushed toward smooth, featureless regions
    // where both lenses agree (corners/edges would make the cut visible).
    float cost = u_blockMatchWeight * minSAD + 0.1 * F + validityPenalty + equatorBias;
    cost = max(cost, 0.0);

    outCost = vec4(cost, hFront, hRear, 0.0);
}
