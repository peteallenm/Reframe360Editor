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
