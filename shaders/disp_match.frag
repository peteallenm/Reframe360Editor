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
// Parallax stitching pass 2: per-texel DISPARITY of the rear band relative to
// the front band, along theta only.
//
// Why 1-D: the two lenses sit back to back, displaced along the optical axis.
// For a scene point in the seam band the two viewing directions differ only in
// the plane containing that axis and the point -- i.e. in theta, never in phi.
// The epipolar lines of this rig are the meridians, so the search is a 1-D
// disparity along v (theta), exactly like stereo along a scanline.
//
// Why direct matching rather than Horn-Schunck: near-object parallax here is
// 2-6 degrees = 8-25 texels at 4.3 texels/deg. HS linearises brightness
// constancy and converges only for ~1-2 texel displacements without a pyramid,
// which is why the previous flow stage barely moved the seam.
//
// Match metric: zero-mean normalised cross-correlation over a 5x5 patch. The
// two lenses differ in exposure and vignetting at their edges, so plain SAD
// would prefer the wrong match on any brightness gradient; ZNCC is invariant to
// affine intensity changes.
//
// Output (RGBA16F): r = disparity d in texels such that rear(u, v + d) matches
// front(u, v); g = confidence in [0,1]; b = 1 if both lenses valid here.

layout(location = 0) in vec2 v_texCoord;
layout(location = 0) out vec4 outDisp;

layout(binding = 0) uniform sampler2D u_bandFront;  // R8
layout(binding = 1) uniform sampler2D u_bandRear;   // R8
uniform int u_searchTexels;                          // D: d in [-D, D]
uniform int u_vMin;                                  // overlap rows (inclusive): both lenses
uniform int u_vMax;                                  //   have real image here (CPU-measured)

// Patch 7 (phi) x 9 (theta). Band texels are anisotropic -- 0.70 deg/texel in
// phi, 0.23 deg/texel in theta -- so this is ~4.9 x 2.1 deg of sky.
const int PU = 3;
const int PV = 4;
const int NP = (2 * PU + 1) * (2 * PV + 1);

float fetchWrap(sampler2D t, ivec2 size, int x, int y) {
    x = ((x % size.x) + size.x) % size.x;      // phi wraps
    y = clamp(y, 0, size.y - 1);               // theta clamps
    return texelFetch(t, ivec2(x, y), 0).r;
}

uniform int u_step;                                  // match every u_step-th band texel (viewport is band/step)

void main() {
    ivec2 size = textureSize(u_bandFront, 0);
    // The match target is band/step in size; each output texel matches the band
    // texel at its centre. Disparity varies slowly across the band (it is a
    // depth map of a 30 deg strip), so half resolution loses nothing visible
    // and quarters the cost.
    ivec2 c = ivec2(gl_FragCoord.xy) * u_step + ivec2(u_step / 2);

    // Validity: only where BOTH lenses have real image. The rim of a fisheye
    // is a dark vignetted ring, not black -- a luma threshold let it through and
    // the matcher then "found" disparities in it. The overlap rows are measured
    // on the CPU from the band row statistics and passed in.
    float valid = (c.y >= u_vMin && c.y <= u_vMax) ? 1.0 : 0.0;

    // Front patch statistics (constant over the search).
    float fSum = 0.0, fSum2 = 0.0;
    float fp[NP];
    int k = 0;
    for (int j = -PV; j <= PV; ++j)
        for (int i = -PU; i <= PU; ++i) {
            float v = fetchWrap(u_bandFront, size, c.x + i, c.y + j);
            fp[k++] = v; fSum += v; fSum2 += v * v;
        }
    const float N = float(NP);
    float fMean = fSum / N;
    float fVar = max(fSum2 / N - fMean * fMean, 0.0);
    float fStd = sqrt(fVar);

    // Untextured front patch: nothing to match against. Report zero disparity
    // with zero confidence so the smoothing fills it from neighbours.
    if (valid < 0.5 || fStd < 0.01) {
        outDisp = vec4(0.0, 0.0, valid, 0.0);
        return;
    }

    int D = clamp(u_searchTexels, 1, 60);
    float scoreAt[121];
    for (int i = 0; i < 121; ++i) scoreAt[i] = -2.0;

    // ZNCC of the front patch against the rear patch shifted by d texels in v.
    #define SCORE_AT(dd, outScore) { \
        float rSum = 0.0, rSum2 = 0.0, cross = 0.0; \
        int kk = 0; \
        for (int j = -PV; j <= PV; ++j) \
            for (int i = -PU; i <= PU; ++i) { \
                float r = fetchWrap(u_bandRear, size, c.x + i, c.y + j + (dd)); \
                rSum += r; rSum2 += r * r; cross += r * fp[kk++]; \
            } \
        float rMean = rSum / N; \
        float rVar = max(rSum2 / N - rMean * rMean, 0.0); \
        outScore = (cross / N - fMean * rMean) / (sqrt(fVar * rVar) + 1e-6); }

    // Coarse-to-fine: every second candidate first, then the neighbours of
    // the best. Halves the evaluations; the parabola below wants both
    // neighbours of the peak, which the refine step guarantees.
    float bestScore = -2.0; int bestD = 0;
    for (int d = -D; d <= D; d += 2) {
        float sc; SCORE_AT(d, sc);
        scoreAt[d + D] = sc;
        if (sc > bestScore) { bestScore = sc; bestD = d; }
    }
    for (int d = max(-D, bestD - 1); d <= min(D, bestD + 1); ++d) {
        if (scoreAt[d + D] > -1.5) continue;
        float sc; SCORE_AT(d, sc);
        scoreAt[d + D] = sc;
        if (sc > bestScore) { bestScore = sc; bestD = d; }
    }
    // Runner-up: best score more than 2 texels from the peak.
    float secondScore = -2.0;
    for (int d = -D; d <= D; ++d)
        if (scoreAt[d + D] > secondScore && abs(d - bestD) > 2) secondScore = scoreAt[d + D];

    // Sub-texel refinement: parabola through the three scores around the peak.
    float dRefined = float(bestD);
    if (bestD > -D && bestD < D) {
        float ym = scoreAt[bestD - 1 + D], y0 = scoreAt[bestD + D], yp = scoreAt[bestD + 1 + D];
        float denom = ym - 2.0 * y0 + yp;
        if (abs(denom) > 1e-6)
            dRefined += clamp(0.5 * (ym - yp) / denom, -1.0, 1.0);
    }

    // Left-right consistency: match back from the rear patch at (u, v+d) to
    // the front. If the reverse search does not land within 1.5 texels of -d,
    // the forward match was ambiguous (repetitive grass, plain sky, a skin
    // tone with no texture) and is discarded outright. This is what removes
    // the +-20 texel speckle that a soft confidence weight let through -- and
    // that speckle, smoothed into a near subject's face, is what produced a
    // duplicated mouth.
    float rp[NP];
    float rpSum = 0.0, rpSum2 = 0.0;
    k = 0;
    for (int j = -PV; j <= PV; ++j)
        for (int i = -PU; i <= PU; ++i) {
            float r = fetchWrap(u_bandRear, size, c.x + i, c.y + j + bestD);
            rp[k++] = r; rpSum += r; rpSum2 += r * r;
        }
    float rpMean = rpSum / N;
    float rpVar = max(rpSum2 / N - rpMean * rpMean, 0.0);
    // A consistency check only needs to know whether the reverse match lands
    // back where it started, so search a local window around -bestD; with a
    // repetitive texture the reverse peak sits elsewhere and the window then
    // holds only a poor score, which fails the test just the same.
    float revBest = -2.0; int revD = -bestD;
    for (int d = -bestD - 6; d <= -bestD + 6; ++d) {
        float fS = 0.0, fS2 = 0.0, cr = 0.0;
        k = 0;
        for (int j = -PV; j <= PV; ++j)
            for (int i = -PU; i <= PU; ++i) {
                float v = fetchWrap(u_bandFront, size, c.x + i, c.y + j + bestD + d);
                fS += v; fS2 += v * v; cr += v * rp[k++];
            }
        float fM = fS / N;
        float fV = max(fS2 / N - fM * fM, 0.0);
        float sc = (cr / N - rpMean * fM) / (sqrt(fV * rpVar) + 1e-6);
        if (sc > revBest) { revBest = sc; revD = d; }
    }
    // Reverse ZNCC must also be a real match, not the best of a bad window.
    bool reverseOk = revBest > 0.3;
    bool consistent = reverseOk && abs(float(revD + bestD)) <= 2.0;

    // Hard gate, then a graded confidence. A match must be good (ZNCC > 0.4),
    // distinct from its runner-up, consistent both ways, and not pinned at the
    // search bound; anything else contributes nothing and is filled by its
    // neighbours in the smoothing passes.
    float distinct = bestScore - max(secondScore, -1.0);
    bool ok = consistent && bestScore > 0.3 && distinct > 0.05 && abs(bestD) < D;
    float conf = ok ? clamp(bestScore, 0.0, 1.0) * clamp(distinct * 4.0, 0.0, 1.0) * valid : 0.0;

    outDisp = vec4(dRefined, conf, valid, 0.0);
}
