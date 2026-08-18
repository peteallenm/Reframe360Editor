#version 440

layout(location = 0) in vec2 v_texCoord;
layout(location = 0) out vec4 fragColor;

layout(binding = 1) uniform sampler2D u_texY;
layout(binding = 2) uniform sampler2D u_texU;
layout(binding = 3) uniform sampler2D u_texV;
layout(binding = 4) uniform sampler2D u_flow;  // optical-flow field (RG, encoded)
layout(binding = 5) uniform sampler2D u_seam;  // 1D seam curve (W×1, R8)

// NOTE: GpuRenderer::flattenUniformBlock turns every non-empty line in the
// block below into "uniform <line>;", so standalone comment lines are NOT
// allowed inside it (they would become "uniform // ..." and fail to compile).
// Keep comments on the same line as a member, after its semicolon.
// Also NOTE: std140 arrays of scalars are strided to 16 bytes, so the padding
// below MUST be scalar members (not float _padG[3]) to keep offsets 292..335
// locked to the C++ ViewerUniforms struct (352 bytes total).
layout(std140, binding = 0) uniform Uniforms {
    mat4 qt_Matrix;
    float qt_Opacity;
    float u_yaw;
    float u_pitch;
    float u_roll;
    float u_fov;
    int u_activeLens;
    float u_viewAspect;
    float u_padA;
    int u_fullRange;
    int u_projection;
    vec2 u_frontCenter;
    float u_frontRadius;
    float u_frontK1;
    float u_frontK2;
    int u_hflipFlags;
    vec2 u_rearCenter;
    float u_rearRadius;
    float u_rearK1;
    float u_rearK2;
    float u_blendStart;
    float u_frontRotation;
    float u_rearRotation;
    mat4 u_imuMatrix;
    float u_brightness;
    float u_contrast;
    float u_saturation;
    float u_pop;
    float u_brightLows;
    float u_brightLowMids;
    float u_brightHighMids;
    float u_brightHighs;
    float u_redLows;
    float u_redMids;
    float u_redHighs;
    float u_greenLows;
    float u_greenMids;
    float u_greenHighs;
    float u_blueLows;
    float u_blueMids;
    float u_blueHighs;
    float _padG0;          // offset 292
    float _padG1;          // offset 296
    float _padG2;          // offset 300
    int u_flowStitch;      // offset 304 (1 = apply flow warp in the blend band)
    int u_flowIterations;  // offset 308 (for FlowRenderer, not the shader)
    float u_flowStrength;  // offset 312 (scales the displacement; 0 disables)
    float u_bandTheta0;    // offset 316 (band lower bound, radians)
    float u_bandTheta1;    // offset 320 (band upper bound, radians)
    float u_flowEncode;    // offset 324 (decode scale of the packed flow texture)
    float _padH0;          // offset 328 -> struct rounds to 336
    float _padH1;          // offset 332
    int u_seamStitch;      // offset 336 (1 = use seam texture for blend placement)
    float u_seamStrength;  // offset 340 (0 = equator, 1 = full seam)
    float _padI[2];        // offset 344 -> struct rounds to 352
};

const float PI = 3.14159265359;

vec2 rotateVec2(vec2 v, float ang) {
    float c = cos(ang);
    float s = sin(ang);
    return vec2(v.x * c - v.y * s, v.x * s + v.y * c);
}

vec3 yuvToRgb(float y, float u, float v) {
    float r, g, b;
    if (u_fullRange == 1) {
        r = y + 1.402 * (v - 0.5);
        g = y - 0.344136 * (u - 0.5) - 0.714136 * (v - 0.5);
        b = y + 1.772 * (u - 0.5);
    } else {
        y = (y - 16.0/255.0) * 255.0/219.0;
        u = u - 0.5;
        v = v - 0.5;
        r = y + 1.402 * v;
        g = y - 0.344136 * u - 0.714136 * v;
        b = y + 1.772 * u;
    }
    return clamp(vec3(r, g, b), 0.0, 1.0);
}

mat3 eulerRotation(float yaw, float pitch, float roll) {
    float cy = cos(radians(yaw)), sy = sin(radians(yaw));
    float cp = cos(radians(pitch)), sp = sin(radians(pitch));
    float cr = cos(radians(roll)), sr = sin(radians(roll));

    mat3 rotY = mat3(cy, 0.0, -sy, 0.0, 1.0, 0.0, sy, 0.0, cy);
    mat3 rotX = mat3(1.0, 0.0, 0.0, 0.0, cp, sp, 0.0, -sp, cp);
    mat3 rotZ = mat3(cr, sr, 0.0, -sr, cr, 0.0, 0.0, 0.0, 1.0);

    return rotY * rotX * rotZ;
}

void main() {
    vec3 ray;
    if (u_projection == 1) {
        // Equirectangular: maps the full 360x180 sphere onto the viewport.
        float lon = (v_texCoord.x - 0.5) * 2.0 * PI;
        float lat = (0.5 - v_texCoord.y) * PI;
        ray = vec3(cos(lat) * sin(lon), sin(lat), -cos(lat) * cos(lon));
    } else if (u_projection == 2) {
        // Stereographic projection: conformal, so wide FOVs don't stretch the
        // edges the way a rectilinear tan() projection does (which also blows
        // up as the FOV approaches 180 deg). It matches a rectilinear view for
        // small angles and maps the whole sphere smoothly. fovScale is chosen
        // so the vertical FOV still equals u_fov: 2*atan(fovScale/2) = fov/2.
        vec2 ndc = v_texCoord * 2.0 - 1.0;
        float fovScale = 2.0 * tan(radians(u_fov * 0.25));
        float aspect = u_viewAspect;
        vec2 p = vec2(ndc.x * fovScale * aspect, -ndc.y * fovScale);
        float rho = length(p);
        vec2 n = (rho > 1e-6) ? (p / rho) : vec2(0.0);
        float ang = 2.0 * atan(rho * 0.5);
        ray = vec3(n * sin(ang), -cos(ang));
    } else if (u_projection == 3) {
        // SportsView (GoPro SuperView style): the perspective view of the full
        // 4:3 sensor field of view, stretched vertically to fill the 16:9
        // frame. The centre of the frame and the horizon stay natural while
        // the top and bottom show more of the scene, stretched, for an
        // immersive ultra-wide feel. Vertical FOV is ~4/3 x u_fov.
        vec2 ndc = v_texCoord * 2.0 - 1.0;
        float fovScale = tan(radians(u_fov * 0.5));
        float aspect = u_viewAspect;
        float sensorAspect = 4.0 / 3.0;
        float verticalStretch = (16.0 / 9.0) / sensorAspect; // 4/3
        ray = normalize(vec3(ndc.x * fovScale * aspect, -ndc.y * fovScale * verticalStretch, -1.0));
    } else {
        // Perspective (rectilinear): straight lines stay straight, at the
        // cost of edge stretching as the FOV widens (the tan() mapping blows
        // up approaching 180 deg). Vertical FOV still equals u_fov.
        vec2 ndc = v_texCoord * 2.0 - 1.0;
        float fovScale = tan(radians(u_fov * 0.5));
        float aspect = u_viewAspect;
        ray = normalize(vec3(ndc.x * fovScale * aspect, -ndc.y * fovScale, -1.0));
    }

    // Apply the user's manual look rotation first, then the IMU
    // counter-rotation. The sampled video direction is imuMatrix * euler * ray
    // (euler = user look, imuMatrix = Q_imu^-1), so the stabilized world
    // direction Q_imu * sampled = euler * ray is fully described by the euler
    // angles; doing it the other way round makes the view drift as the camera
    // rotates whenever yaw/pitch/roll are non-zero.
    ray = eulerRotation(u_yaw, u_pitch, u_roll) * ray;

    vec4 imuRay = u_imuMatrix * vec4(ray, 0.0);
    ray = normalize(imuRay.xyz);

    float theta = acos(clamp(-ray.z, -1.0, 1.0));
    float phi = atan(ray.y, ray.x);

    float r_front = theta / (PI * 0.5);
    float r_front_dist = r_front * (1.0 + u_frontK1 * r_front * r_front + u_frontK2 * r_front * r_front * r_front * r_front);
    vec2 frontOff = rotateVec2(vec2(cos(phi), sin(phi)), radians(u_frontRotation)) * r_front_dist * u_frontRadius;
    // mod() instead of '&' so the shader also compiles under GLSL ES 1.00
    // (GLES2), where bitwise operators need GL_EXT_gpu_shader4.
    if (mod(float(u_hflipFlags), 2.0) != 0.0)
        frontOff.x = -frontOff.x;
    vec2 uv_front = u_frontCenter + frontOff;

    float theta_rear = PI - theta;
    float r_rear = theta_rear / (PI * 0.5);
    float r_rear_dist = r_rear * (1.0 + u_rearK1 * r_rear * r_rear + u_rearK2 * r_rear * r_rear * r_rear * r_rear);
    vec2 rearOff = rotateVec2(vec2(cos(phi), sin(phi)), radians(u_rearRotation)) * r_rear_dist * u_rearRadius;
    if (mod(float(u_hflipFlags), 4.0) >= 2.0)
        rearOff.x = -rearOff.x;
    vec2 uv_rear = u_rearCenter + rearOff;

    vec3 rgb;

    if (u_activeLens == 0) {
        vec2 uv_tex = vec2(uv_front.x, uv_front.y * 0.5);
        float y = texture(u_texY, uv_tex).r;
        float u = texture(u_texU, uv_tex).r;
        float v = texture(u_texV, uv_tex).r;
        rgb = yuvToRgb(y, u, v);
    } else if (u_activeLens == 1) {
        vec2 uv_tex = vec2(uv_rear.x, uv_rear.y * 0.5 + 0.5);
        float y = texture(u_texY, uv_tex).r;
        float u = texture(u_texU, uv_tex).r;
        float v = texture(u_texV, uv_tex).r;
        rgb = yuvToRgb(y, u, v);
    } else {
        float blendStart = u_blendStart;
        float blendEnd = 1.0;
        if (u_seamStitch != 0) {
            float u_band = mod(phi + PI, 2.0 * PI) / (2.0 * PI);
            float seamV = texture(u_seam, vec2(u_band, 0.5)).r;
            float seamTheta = mix(u_bandTheta0, u_bandTheta1, seamV);
            float seamRFront = seamTheta / (PI * 0.5);
            // Shift the blend center toward the seam.
            float origWidth = 1.0 - u_blendStart;  // 0.1 (9°)
            float origCenter = (u_blendStart + 1.0) * 0.5;  // 0.95
            float center = mix(origCenter, seamRFront, u_seamStrength);
            // Clamp center to maintain minimum width within bounds
            float minHalfWidth = 0.025;  // 4.5° minimum
            center = clamp(center, u_blendStart + minHalfWidth, 1.0 - minHalfWidth);
            // Compute width: full width if possible, narrower if near edges
            float halfWidth = origWidth * 0.5;  // 0.05
            float maxHalfWidth = min(center - u_blendStart, 1.0 - center);
            halfWidth = min(halfWidth, maxHalfWidth);
            blendStart = center - halfWidth;
            blendEnd = center + halfWidth;
        }
        float blend = smoothstep(blendStart, blendEnd, r_front);

        vec2 uv_f = vec2(uv_front.x, uv_front.y * 0.5);
        float yf = texture(u_texY, uv_f).r;
        float uf = texture(u_texU, uv_f).r;
        float vf = texture(u_texV, uv_f).r;
        vec3 rgb_f = yuvToRgb(yf, uf, vf);

        vec2 uv_r = vec2(uv_rear.x, uv_rear.y * 0.5 + 0.5);
        float yr = texture(u_texY, uv_r).r;
        float ur = texture(u_texU, uv_r).r;
        float vr = texture(u_texV, uv_r).r;
        vec3 rgb_r = yuvToRgb(yr, ur, vr);

        // ---- Optical-flow parallax stitching (see OpticalFlow.md) ----
        // Only the transition band (where both lenses contribute) is warped:
        // w = 4*blend*(1-blend) is a tent peaking at blend=0.5 (the seam
        // centre) and 0 at pure front (blend=0) / pure rear (blend=1), so the
        // regions that render from a single lens stay geometrically
        // unmodified. The flow field lives in the camera-native band
        // (phi in [0,2pi), theta in [bandTheta0, bandTheta1]) and stores the
        // rear-vs-front displacement in normalized band units, so we warp the
        // rear sample to align with the front before blending.
        if (u_flowStitch != 0) {
            float w = 4.0 * blend * (1.0 - blend);
            if (w > 0.0) {
                float u_band = mod(phi + PI, 2.0 * PI) / (2.0 * PI);
                float v_band = clamp((theta - u_bandTheta0) / (u_bandTheta1 - u_bandTheta0), 0.0, 1.0);
                vec2 enc = texture(u_flow, vec2(u_band, v_band)).rg;
                vec2 nduv = (enc - 0.5) / u_flowEncode * u_flowStrength;

                float phi2 = mod(u_band + nduv.x, 1.0) * 2.0 * PI - PI;
                float theta2 = mix(u_bandTheta0, u_bandTheta1,
                                   clamp(v_band + nduv.y, 0.0, 1.0));
                vec3 ray2 = vec3(sin(theta2) * cos(phi2),
                                 sin(theta2) * sin(phi2), -cos(theta2));

                // Project the warped direction into the REAR fisheye (same
                // math as the geometric rear sample above).
                float theta_rear2 = PI - theta2;
                float r_rear2 = theta_rear2 / (PI * 0.5);
                float r_dist2 = r_rear2 * (1.0 + u_rearK1 * r_rear2 * r_rear2 + u_rearK2 * r_rear2 * r_rear2 * r_rear2 * r_rear2);
                vec2 off2 = rotateVec2(vec2(cos(phi2), sin(phi2)), radians(u_rearRotation)) * r_dist2 * u_rearRadius;
                if (mod(float(u_hflipFlags), 4.0) >= 2.0)
                    off2.x = -off2.x;
                vec2 uv_r2 = u_rearCenter + off2;
                vec2 uv_tex2 = vec2(uv_r2.x, uv_r2.y * 0.5 + 0.5);
                vec3 rgb_warped = yuvToRgb(texture(u_texY, uv_tex2).r,
                                           texture(u_texU, uv_tex2).r,
                                           texture(u_texV, uv_tex2).r);
                rgb_r = mix(rgb_r, rgb_warped, w);
            }
        }

        rgb = mix(rgb_f, rgb_r, blend);
    }

    // ---- Colour grading ----
    // 3-way corrector first (weights from the ungraded luma). Each slider
    // shifts its channel in the shadows (wL), midtones (wM) or highlights
    // (wH); the weights are luma-based so the same values match the C++
    // exporter's per-pixel math exactly.
    float luma = dot(rgb, vec3(0.2126, 0.7152, 0.0722));
    float wL = (1.0 - luma) * (1.0 - luma);
    float wH = luma * luma;
    float wM = 1.0 - wL - wH;
    // The brightness mid band is split into low-mids and high-mids: the two
    // shares always sum to the old wM and cross over exactly at luma 0.5
    // (below 0.25 all mid weight goes to low mids, above 0.75 all to high
    // mids), so setting both equal reproduces the old single mids slider.
    float tMid = clamp((luma - 0.5) * 4.0, -1.0, 1.0);
    float wLM = wM * 0.5 * (1.0 - tMid);
    float wHM = wM * 0.5 * (1.0 + tMid);

    rgb += vec3(u_brightLows * wL + u_brightLowMids * wLM + u_brightHighMids * wHM + u_brightHighs * wH);
    rgb.r += u_redLows * wL + u_redMids * wM + u_redHighs * wH;
    rgb.g += u_greenLows * wL + u_greenMids * wM + u_greenHighs * wH;
    rgb.b += u_blueLows * wL + u_blueMids * wM + u_blueHighs * wH;

    // Pop: midtone contrast ("clarity") — pushes midtones toward black/white
    // for extra depth while leaving pure black/white untouched.
    luma = dot(rgb, vec3(0.2126, 0.7152, 0.0722));
    float midW = 1.0 - abs(luma * 2.0 - 1.0);
    rgb += u_pop * 0.2 * midW * (rgb - 0.5);

    // Saturation: mix each channel toward luma (1 = neutral, 0 = greyscale).
    luma = dot(rgb, vec3(0.2126, 0.7152, 0.0722));
    rgb = mix(vec3(luma), rgb, u_saturation);

    // Contrast about 0.5 (1 = neutral) then brightness offset (0 = neutral).
    rgb = (rgb - 0.5) * u_contrast + 0.5;
    rgb += u_brightness;

    fragColor = vec4(clamp(rgb, 0.0, 1.0), qt_Opacity);
}