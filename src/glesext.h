#ifndef GLESEXT_H
#define GLESEXT_H

// ---------------------------------------------------------------------------
// GLES 3.0 constants on an Android build.
//
// Qt for Android ships configured to the OpenGL ES 2.0 baseline
// (QT_FEATURE_opengles3 == -1 in qtgui-config.h), so <QOpenGLFunctions> pulls
// in <GLES2/gl2.h> and every GLES 3.0 enum this project uses -- GL_R8, GL_RED,
// GL_RGBA8, GL_RG, GL_RG16F, GL_RGBA16F, GL_HALF_FLOAT, GL_UNPACK_ROW_LENGTH,
// GL_COLOR_ATTACHMENT1 -- is undeclared.
//
// Only the CONSTANTS are missing. The GLES 3.0 entry points themselves are
// resolved at runtime by QOpenGLExtraFunctions, which is what GpuRenderer and
// FlowRenderer already use, and both refuse to run on a context below ES 3.0.
// So pulling in the NDK's own GLES 3 header is enough, and is preferable to
// hand-defining the values: new enums come for free and cannot be mistyped.
//
// Including this alongside Qt's <GLES2/gl2.h> is safe -- GLES3/gl3.h is a
// superset and the overlapping macros and typedefs are token-identical.
//
// Include AFTER the Qt OpenGL headers in any file that names a GLES 3 enum.
// ---------------------------------------------------------------------------

#include <QtGlobal>

#ifdef Q_OS_ANDROID
#  include <GLES3/gl3.h>
#endif

#endif // GLESEXT_H
