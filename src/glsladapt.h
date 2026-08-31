// Reframe360 Editor -- 360 video stabiliser and stitcher for dual-fisheye footage.
// Copyright (C) 2026 Peter Allen
// SPDX-License-Identifier: GPL-3.0-or-later
//
// This program is free software under the GNU General Public License, version
// 3 or (at your option) any later version; see LICENSE for the full text.
// It is distributed in the hope that it will be useful, but WITHOUT ANY
// WARRANTY, without even the implied warranty of MERCHANTABILITY or FITNESS
// FOR A PARTICULAR PURPOSE.

#ifndef GLSLADAPT_H
#define GLSLADAPT_H

#include <QByteArray>

// ---------------------------------------------------------------------------
// GLSL source adaptation for the two standalone GL renderers (GpuRenderer and
// FlowRenderer).
//
// The shipped shaders are written for Qt's RHI (qsb): #version 440, with
// binding-qualified samplers and a std140 uniform block. The standalone
// renderers compile that same source directly -- so preview and export can
// never drift apart -- which means it has to be lowered to whatever the
// runtime context actually is:
//
//   desktop GL  -> #version 330 core   (GL 3.3, universal since ~2012)
//   OpenGL ES   -> #version 300 es     (GLES 3.0, every Android device we care
//                                       about; Mali-G77 on the Edge 40 is 3.2)
//
// In both cases layout(binding = N) on samplers is a GLSL 4.20 / ES 3.1
// feature and must go: sampler units are assigned by name after linking.
// ES additionally requires explicit default precision qualifiers, which have
// no meaning on desktop and must NOT be emitted there.
//
// Keeping this in one place matters: when the tone-curve LUT was added at
// binding 6 and the adapter only knew about bindings 1-5, the export shader
// silently failed to compile and the exporter fell back to the CPU rasterizer
// with an inverted pitch. Anything that changes the shaders' declaration
// syntax has to be handled here, for both targets, or that class of bug comes
// straight back.
// ---------------------------------------------------------------------------

enum class GlslTarget {
    DesktopCore330,   // #version 330 core
    EmbeddedEs300,    // #version 300 es
};

enum class GlslStage {
    Vertex,
    Fragment,
};

// Lower the #version directive, inject ES precision defaults, and strip the
// layout() qualifiers the target cannot parse.
//
// The stage matters because layout(location = N) means three different things
// and only one of them survives:
//   * vertex inputs   -- stripped; locations are pinned with
//                        QOpenGLShaderProgram::bindAttributeLocation instead
//   * varyings (vertex out / fragment in) -- stripped; this form needs GLSL
//                        410 / ES 3.1, and matching by name is equivalent
//   * fragment outputs -- KEPT; core in both 330 and 300 es, and the only way
//                        to address MRT attachments on ES (there is no
//                        glBindFragDataLocation outside desktop GL), which
//                        band_extract.frag relies on for its two outputs
QByteArray adaptGlsl(const QByteArray &src, GlslTarget target, GlslStage stage);

// Replace the std140 "uniform Uniforms { ... }" block with loose global
// uniforms of the same names, so values can be uploaded by name through
// QOpenGLShaderProgram (which cannot write into a UBO). Applied after
// adaptGlsl(). Shaders with no such block pass through unchanged.
QByteArray flattenUniformBlock(const QByteArray &src);

#endif // GLSLADAPT_H
