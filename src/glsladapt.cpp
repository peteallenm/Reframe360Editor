#include "glsladapt.h"

#include <QRegularExpression>
#include <QString>
#include <QStringList>

namespace {

// ES fragment shaders have no default precision for float/int/sampler types,
// so one must be declared or every shader fails to compile. highp is what the
// desktop path effectively uses, and every ES 3.0 implementation is required
// to support highp in the fragment stage.
const char *kEsPrecision =
    "precision highp float;\n"
    "precision highp int;\n"
    "precision highp sampler2D;\n";

} // namespace

QByteArray adaptGlsl(const QByteArray &src, GlslTarget target, GlslStage stage)
{
    const bool es = (target == GlslTarget::EmbeddedEs300);

    QString s = QString::fromUtf8(src);

    // 1. Version directive. Match whatever the source declares rather than a
    //    fixed list, so bumping shaders/*.frag to a newer #version does not
    //    quietly leave this behind.
    //    The whitespace classes here are deliberately [ \t] and not \s:
    //    \s matches newlines, so an optional trailing profile token could
    //    reach across the blank line into the next declaration and delete it
    //    (this ate project.frag's "in vec2 v_texCoord" until glescheck caught
    //    the resulting "undefined variable" errors).
    static const QRegularExpression versionLine(
        QStringLiteral("^[ \\t]*#version[ \\t]+\\d+([ \\t]+\\w+)?[ \\t]*"),
        QRegularExpression::MultilineOption);
    const QString header = es ? QStringLiteral("#version 300 es\n%1").arg(QString::fromLatin1(kEsPrecision))
                              : QStringLiteral("#version 330 core");
    s.replace(versionLine, header);

    // 2. layout(binding = N) on an opaque uniform is GLSL 4.20 / ES 3.1. Strip
    //    it on both targets; the caller assigns texture units by name after
    //    link. Matching the sampler TYPE generically (sampler2D, sampler2DArray,
    //    isampler*, usampler*, image*) rather than the one spelling in use
    //    today is deliberate: the previous adapter matched a fixed list, and a
    //    declaration it did not recognise made the whole shader fail to
    //    compile -- which silently dropped exports to the CPU rasterizer.
    static const QRegularExpression opaqueBinding(
        QStringLiteral("layout\\s*\\(\\s*binding\\s*=\\s*\\d+\\s*\\)\\s*uniform\\s+((?:[iu]?sampler|image)\\w*)"));
    s.replace(opaqueBinding, QStringLiteral("uniform \\1"));

    // 3. layout(location = N) on inputs -- vertex attributes and fragment
    //    varyings alike -- is stripped. Attributes get their locations from
    //    bindAttributeLocation(); varyings are matched by name.
    static const QRegularExpression inLocation(
        QStringLiteral("layout\\s*\\(\\s*location\\s*=\\s*\\d+\\s*\\)\\s*in\\b"));
    s.replace(inLocation, QStringLiteral("in"));

    // 4. layout(location = N) on outputs: a varying in the vertex stage (needs
    //    GLSL 410 / ES 3.1 -- strip), a draw-buffer index in the fragment
    //    stage (core in 330 and 300 es -- keep, MRT depends on it).
    if (stage == GlslStage::Vertex) {
        static const QRegularExpression outLocation(
            QStringLiteral("layout\\s*\\(\\s*location\\s*=\\s*\\d+\\s*\\)\\s*out\\b"));
        s.replace(outLocation, QStringLiteral("out"));
    }

    // 5. quad.vert addresses its uniform block members through the block
    //    instance name; flattenUniformBlock() turns them into globals.
    s.replace(QStringLiteral("ubuf.qt_Matrix"), QStringLiteral("qt_Matrix"));

    return s.toUtf8();
}

QByteArray flattenUniformBlock(const QByteArray &src)
{
    const QList<QByteArray> lines = src.split('\n');
    QList<QByteArray> out;
    out.reserve(lines.size());
    bool inBlock = false;
    for (const QByteArray &line : lines) {
        const QByteArray t = line.trimmed();
        if (t.startsWith("layout(std140") && t.contains("uniform Uniforms")) {
            inBlock = true;
            continue;
        }
        if (inBlock) {
            if (t == "};" || t == "} ubuf;") {
                inBlock = false;
                continue;
            }
            if (!t.isEmpty())
                out.append(QByteArray("uniform ") + t);
            continue;
        }
        out.append(line);
    }
    return out.join('\n');
}
