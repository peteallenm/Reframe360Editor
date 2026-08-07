#version 440

layout(location = 0) in vec2 v_texCoord;
layout(location = 0) out vec4 fragColor;

layout(binding = 1) uniform sampler2D u_texY;
layout(binding = 2) uniform sampler2D u_texU;
layout(binding = 3) uniform sampler2D u_texV;

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
    if ((u_hflipFlags & 1) != 0)
        frontOff.x = -frontOff.x;
    vec2 uv_front = u_frontCenter + frontOff;

    float theta_rear = PI - theta;
    float r_rear = theta_rear / (PI * 0.5);
    float r_rear_dist = r_rear * (1.0 + u_rearK1 * r_rear * r_rear + u_rearK2 * r_rear * r_rear * r_rear * r_rear);
    vec2 rearOff = rotateVec2(vec2(cos(phi), sin(phi)), radians(u_rearRotation)) * r_rear_dist * u_rearRadius;
    if ((u_hflipFlags & 2) != 0)
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
        float blend = smoothstep(u_blendStart, 1.0, r_front);

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

        rgb = mix(rgb_f, rgb_r, blend);
    }

    fragColor = vec4(rgb, qt_Opacity);
}