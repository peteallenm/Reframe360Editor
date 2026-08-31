/*
 * glescheck.c -- compile GLSL files under a REAL OpenGL ES 3.x context.
 *
 * Qt on this machine is built against desktop GL and cannot create a GLES
 * context, so this tool talks to EGL + libGLESv2 directly (no Qt).
 *
 * BUILD (one line):
 *   gcc -O2 -Wall -o tools/glescheck tools/glescheck.c -lEGL -lGLESv2
 *
 * RUN (no env vars needed on this machine -- the NVIDIA EGL driver gives a
 * surfaceless OpenGL ES 3.2 context straight away):
 *   ./tools/glescheck frag shaders/foo.frag
 *     GL_VERSION  : OpenGL ES 3.2 NVIDIA 535.309.01  / GLSL ES 3.20
 *
 * To cross-check against the Mesa GLSL ES compiler instead (llvmpipe --
 * a second opinion, and closer to a non-NVIDIA Android driver):
 *   __EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/50_mesa.json \
 *   LIBGL_ALWAYS_SOFTWARE=1 MESA_LOADER_DRIVER_OVERRIDE=llvmpipe \
 *   ./tools/glescheck frag shaders/foo.frag
 *     GL_VERSION  : OpenGL ES 3.2 Mesa 25.2.8  / llvmpipe
 *   Only the *compiler* matters here, so software rendering is fine.
 *
 * NOTE: both drivers here expose ES 3.2, so an ES 3.1+ construct will compile
 * even though an ES 3.0-only Android device would reject it. Keep the
 * `#version 300 es` line honest in the shader itself.
 *
 * USAGE:
 *   glescheck <vert|frag> <file.glsl> [more files...]
 *   glescheck --link <vert.glsl> <frag.glsl>
 *
 * Prints "OK <file>" or "FAIL <file>" + the driver info log; exits non-zero
 * if anything failed. --link also compiles both stages, links a program and
 * reports the link log (where sampler / uniform-block mismatches surface).
 */

#include <EGL/egl.h>
#include <EGL/eglext.h>
#include <GLES3/gl3.h>

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static EGLDisplay g_dpy = EGL_NO_DISPLAY;
static EGLContext g_ctx = EGL_NO_CONTEXT;
static EGLSurface g_surf = EGL_NO_SURFACE;

static const char *egl_err_str(EGLint e)
{
    switch (e) {
    case EGL_SUCCESS:             return "EGL_SUCCESS";
    case EGL_NOT_INITIALIZED:     return "EGL_NOT_INITIALIZED";
    case EGL_BAD_ACCESS:          return "EGL_BAD_ACCESS";
    case EGL_BAD_ALLOC:           return "EGL_BAD_ALLOC";
    case EGL_BAD_ATTRIBUTE:       return "EGL_BAD_ATTRIBUTE";
    case EGL_BAD_CONFIG:          return "EGL_BAD_CONFIG";
    case EGL_BAD_CONTEXT:         return "EGL_BAD_CONTEXT";
    case EGL_BAD_DISPLAY:         return "EGL_BAD_DISPLAY";
    case EGL_BAD_MATCH:           return "EGL_BAD_MATCH";
    case EGL_BAD_NATIVE_WINDOW:   return "EGL_BAD_NATIVE_WINDOW";
    case EGL_BAD_PARAMETER:       return "EGL_BAD_PARAMETER";
    case EGL_BAD_SURFACE:         return "EGL_BAD_SURFACE";
    default:                      return "EGL_<unknown>";
    }
}

/* Pick an ES3-renderable config. want_pbuffer selects EGL_PBUFFER_BIT. */
static int choose_config(EGLDisplay dpy, int want_pbuffer, EGLConfig *out)
{
    EGLint attribs[] = {
        EGL_SURFACE_TYPE,    want_pbuffer ? EGL_PBUFFER_BIT : EGL_PBUFFER_BIT,
        EGL_RENDERABLE_TYPE, EGL_OPENGL_ES3_BIT,
        EGL_RED_SIZE,        8,
        EGL_GREEN_SIZE,      8,
        EGL_BLUE_SIZE,       8,
        EGL_ALPHA_SIZE,      8,
        EGL_DEPTH_SIZE,      0,
        EGL_NONE
    };
    EGLint n = 0;
    if (!eglChooseConfig(dpy, attribs, out, 1, &n) || n < 1)
        return 0;
    return 1;
}

static int make_context(EGLDisplay dpy, int with_pbuffer)
{
    EGLConfig cfg;
    if (!choose_config(dpy, with_pbuffer, &cfg)) {
        fprintf(stderr, "glescheck: eglChooseConfig failed (%s)\n",
                egl_err_str(eglGetError()));
        return 0;
    }

    static const EGLint ctx_attribs[] = {
        EGL_CONTEXT_CLIENT_VERSION, 3,
        EGL_NONE
    };
    EGLContext ctx = eglCreateContext(dpy, cfg, EGL_NO_CONTEXT, ctx_attribs);
    if (ctx == EGL_NO_CONTEXT) {
        fprintf(stderr, "glescheck: eglCreateContext failed (%s)\n",
                egl_err_str(eglGetError()));
        return 0;
    }

    EGLSurface surf = EGL_NO_SURFACE;
    if (with_pbuffer) {
        static const EGLint pb[] = { EGL_WIDTH, 16, EGL_HEIGHT, 16, EGL_NONE };
        surf = eglCreatePbufferSurface(dpy, cfg, pb);
        if (surf == EGL_NO_SURFACE) {
            fprintf(stderr, "glescheck: eglCreatePbufferSurface failed (%s)\n",
                    egl_err_str(eglGetError()));
            eglDestroyContext(dpy, ctx);
            return 0;
        }
    }

    if (!eglMakeCurrent(dpy, surf, surf, ctx)) {
        fprintf(stderr, "glescheck: eglMakeCurrent failed (%s)\n",
                egl_err_str(eglGetError()));
        if (surf != EGL_NO_SURFACE) eglDestroySurface(dpy, surf);
        eglDestroyContext(dpy, ctx);
        return 0;
    }

    g_dpy = dpy;
    g_ctx = ctx;
    g_surf = surf;
    return 1;
}

static int init_gles(void)
{
    EGLint major = 0, minor = 0;

    if (!eglBindAPI(EGL_OPENGL_ES_API)) {
        fprintf(stderr, "glescheck: eglBindAPI(EGL_OPENGL_ES_API) failed (%s)\n",
                egl_err_str(eglGetError()));
        return 0;
    }

    /* 1. Surfaceless platform (Mesa / EGL 1.5). */
    PFNEGLGETPLATFORMDISPLAYEXTPROC getPlatformDisplay =
        (PFNEGLGETPLATFORMDISPLAYEXTPROC)
        eglGetProcAddress("eglGetPlatformDisplayEXT");
    if (getPlatformDisplay) {
        EGLDisplay dpy = getPlatformDisplay(EGL_PLATFORM_SURFACELESS_MESA,
                                           EGL_DEFAULT_DISPLAY, NULL);
        if (dpy != EGL_NO_DISPLAY && eglInitialize(dpy, &major, &minor)) {
            eglBindAPI(EGL_OPENGL_ES_API);
            if (make_context(dpy, /*with_pbuffer=*/0)) {
                fprintf(stderr, "glescheck: EGL %d.%d (surfaceless)\n",
                        major, minor);
                return 1;
            }
            /* Surfaceless config w/o surface refused: retry with a pbuffer. */
            if (make_context(dpy, /*with_pbuffer=*/1)) {
                fprintf(stderr,
                        "glescheck: EGL %d.%d (surfaceless + pbuffer)\n",
                        major, minor);
                return 1;
            }
            eglTerminate(dpy);
        }
    }

    /* 2. Default display + pbuffer. */
    {
        EGLDisplay dpy = eglGetDisplay(EGL_DEFAULT_DISPLAY);
        if (dpy != EGL_NO_DISPLAY && eglInitialize(dpy, &major, &minor)) {
            eglBindAPI(EGL_OPENGL_ES_API);
            if (make_context(dpy, /*with_pbuffer=*/1)) {
                fprintf(stderr, "glescheck: EGL %d.%d (default display, pbuffer)\n",
                        major, minor);
                return 1;
            }
            if (make_context(dpy, /*with_pbuffer=*/0)) {
                fprintf(stderr, "glescheck: EGL %d.%d (default display, surfaceless)\n",
                        major, minor);
                return 1;
            }
            eglTerminate(dpy);
        }
    }

    fprintf(stderr,
            "glescheck: could not create an OpenGL ES 3 context.\n"
            "  Try forcing Mesa/llvmpipe:\n"
            "    __EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/50_mesa.json \\\n"
            "    LIBGL_ALWAYS_SOFTWARE=1 MESA_LOADER_DRIVER_OVERRIDE=llvmpipe glescheck ...\n");
    return 0;
}

static void shutdown_gles(void)
{
    if (g_dpy == EGL_NO_DISPLAY)
        return;
    eglMakeCurrent(g_dpy, EGL_NO_SURFACE, EGL_NO_SURFACE, EGL_NO_CONTEXT);
    if (g_surf != EGL_NO_SURFACE) eglDestroySurface(g_dpy, g_surf);
    if (g_ctx != EGL_NO_CONTEXT) eglDestroyContext(g_dpy, g_ctx);
    eglTerminate(g_dpy);
}

static char *read_file(const char *path, long *len_out)
{
    FILE *f = fopen(path, "rb");
    if (!f) {
        fprintf(stderr, "glescheck: cannot open %s\n", path);
        return NULL;
    }
    if (fseek(f, 0, SEEK_END) != 0) { fclose(f); return NULL; }
    long len = ftell(f);
    if (len < 0) { fclose(f); return NULL; }
    rewind(f);
    char *buf = (char *)malloc((size_t)len + 1);
    if (!buf) { fclose(f); return NULL; }
    size_t got = fread(buf, 1, (size_t)len, f);
    fclose(f);
    buf[got] = '\0';
    if (len_out) *len_out = (long)got;
    return buf;
}

static void print_log(const char *label, const char *path, GLuint obj, int is_prog)
{
    GLint loglen = 0;
    if (is_prog) glGetProgramiv(obj, GL_INFO_LOG_LENGTH, &loglen);
    else         glGetShaderiv(obj, GL_INFO_LOG_LENGTH, &loglen);
    if (loglen <= 1)
        return;
    char *log = (char *)malloc((size_t)loglen + 1);
    if (!log) return;
    GLsizei written = 0;
    if (is_prog) glGetProgramInfoLog(obj, loglen, &written, log);
    else         glGetShaderInfoLog(obj, loglen, &written, log);
    log[written] = '\0';
    fprintf(stderr, "--- %s log: %s ---\n%s", label, path, log);
    if (written > 0 && log[written - 1] != '\n') fputc('\n', stderr);
    free(log);
}

/* Compile one file. Returns the shader object, or 0 on failure. */
static GLuint compile_file(const char *path, GLenum stage, int *ok_out)
{
    *ok_out = 0;
    long len = 0;
    char *src = read_file(path, &len);
    if (!src) {
        printf("FAIL %s\n", path);
        return 0;
    }

    GLuint sh = glCreateShader(stage);
    if (!sh) {
        fprintf(stderr, "glescheck: glCreateShader failed (GL error 0x%x)\n",
                glGetError());
        free(src);
        printf("FAIL %s\n", path);
        return 0;
    }

    const GLchar *srcs[1] = { src };
    const GLint lens[1] = { (GLint)len };
    glShaderSource(sh, 1, srcs, lens);
    glCompileShader(sh);
    free(src);

    GLint status = GL_FALSE;
    glGetShaderiv(sh, GL_COMPILE_STATUS, &status);

    if (status == GL_TRUE) {
        printf("OK %s\n", path);
        print_log("compile warnings", path, sh, 0);
        *ok_out = 1;
        return sh;
    }

    printf("FAIL %s\n", path);
    print_log("compile", path, sh, 0);
    glDeleteShader(sh);
    return 0;
}

static void usage(const char *argv0)
{
    fprintf(stderr,
        "usage: %s <vert|frag> <file.glsl> [more files...]\n"
        "       %s --link <vert.glsl> <frag.glsl>\n", argv0, argv0);
}

int main(int argc, char **argv)
{
    if (argc < 3) {
        usage(argv[0]);
        return 2;
    }

    /* Keep stdout in step with the info logs we write to stderr. */
    setvbuf(stdout, NULL, _IOLBF, 0);

    const char *mode = argv[1];
    int do_link = (strcmp(mode, "--link") == 0);
    GLenum stage = GL_FRAGMENT_SHADER;

    if (!do_link) {
        if (strcmp(mode, "vert") == 0)      stage = GL_VERTEX_SHADER;
        else if (strcmp(mode, "frag") == 0) stage = GL_FRAGMENT_SHADER;
        else { usage(argv[0]); return 2; }
    } else if (argc != 4) {
        usage(argv[0]);
        return 2;
    }

    if (!init_gles())
        return 3;

    printf("GL_VERSION                  : %s\n", (const char *)glGetString(GL_VERSION));
    printf("GL_SHADING_LANGUAGE_VERSION : %s\n", (const char *)glGetString(GL_SHADING_LANGUAGE_VERSION));
    printf("GL_RENDERER                 : %s\n", (const char *)glGetString(GL_RENDERER));
    printf("GL_VENDOR                   : %s\n", (const char *)glGetString(GL_VENDOR));
    fflush(stdout);

    int failures = 0;

    if (do_link) {
        int vok = 0, fok = 0;
        GLuint vs = compile_file(argv[2], GL_VERTEX_SHADER, &vok);
        GLuint fs = compile_file(argv[3], GL_FRAGMENT_SHADER, &fok);
        if (!vok || !fok) {
            failures++;
            printf("FAIL link %s + %s (compile errors above)\n", argv[2], argv[3]);
        } else {
            GLuint prog = glCreateProgram();
            glAttachShader(prog, vs);
            glAttachShader(prog, fs);
            glLinkProgram(prog);
            GLint linked = GL_FALSE;
            glGetProgramiv(prog, GL_LINK_STATUS, &linked);
            char label[1024];
            snprintf(label, sizeof label, "%s + %s", argv[2], argv[3]);
            if (linked) {
                printf("OK link %s\n", label);
                print_log("link warnings", label, prog, 1);
            } else {
                printf("FAIL link %s\n", label);
                print_log("link", label, prog, 1);
                failures++;
            }
            glDeleteProgram(prog);
        }
        if (vs) glDeleteShader(vs);
        if (fs) glDeleteShader(fs);
    } else {
        for (int i = 2; i < argc; i++) {
            int ok = 0;
            GLuint sh = compile_file(argv[i], stage, &ok);
            if (!ok) failures++;
            if (sh) glDeleteShader(sh);
        }
    }

    fflush(stdout);
    shutdown_gles();
    return failures ? 1 : 0;
}
