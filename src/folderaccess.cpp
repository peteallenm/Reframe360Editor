#include "folderaccess.h"

#include <QDir>
#include <QFileInfo>
#include <QUrl>

#ifdef Q_OS_ANDROID
#include <QJniEnvironment>
#include <QJniObject>
#include <QCoreApplication>
#include <QtCore/private/qandroidextras_p.h>
#endif

namespace {

// Android constants, spelled out rather than looked up: these are frozen parts
// of the platform ABI and reading them back through JNI adds calls for nothing.
constexpr int kFlagGrantRead        = 0x00000001;  // Intent.FLAG_GRANT_READ_URI_PERMISSION
constexpr int kFlagGrantWrite       = 0x00000002;  // Intent.FLAG_GRANT_WRITE_URI_PERMISSION
constexpr int kFlagGrantPersistable = 0x00000040;  // Intent.FLAG_GRANT_PERSISTABLE_URI_PERMISSION
constexpr int kResultOk             = -1;          // Activity.RESULT_OK
constexpr int kRequestPickFolder    = 0x360F;      // ours; any value the app does not reuse

bool looksLikeVideo(const QString &name)
{
    const QString n = name.toLower();
    return n.endsWith(QStringLiteral(".mp4")) || n.endsWith(QStringLiteral(".mov"))
        || n.endsWith(QStringLiteral(".mkv"));
}

bool looksLikeProxy(const QString &name)
{
    const QString n = name.toLower();
    return looksLikeVideo(n) && QFileInfo(n).completeBaseName().endsWith(QStringLiteral("_thm"));
}

} // namespace

FolderAccess::FolderAccess(QObject *parent)
    : QObject(parent)
{
}

bool FolderAccess::isAvailable()
{
#ifdef Q_OS_ANDROID
    return true;
#else
    return false;
#endif
}

QString FolderAccess::folderName() const
{
    if (m_tree.isEmpty())
        return QString();
#ifdef Q_OS_ANDROID
    // A tree URI ends in the document id, e.g. ".../tree/primary%3AMovies%2F360".
    // Show the last path segment of that id -- the folder the user chose.
    const QString decoded = QUrl::fromPercentEncoding(m_tree.toUtf8());
    const int slash = decoded.lastIndexOf(QLatin1Char('/'));
    const int colon = decoded.lastIndexOf(QLatin1Char(':'));
    const int cut = qMax(slash, colon);
    return cut >= 0 ? decoded.mid(cut + 1) : decoded;
#else
    return QDir(m_tree).dirName();
#endif
}

QStringList FolderAccess::clips() const
{
    QStringList out;
    for (auto it = m_entries.constBegin(); it != m_entries.constEnd(); ++it) {
        if (!looksLikeVideo(it.key()) || looksLikeProxy(it.key()))
            continue;
        // The app's own exports (default name <clip>_export.mp4) land in this
        // same folder; listing them as source clips is clutter -- they are
        // flat renders, not dual-fisheye sources. Openable via the file
        // picker if ever needed.
        if (it.key().contains(QLatin1String("_export."), Qt::CaseInsensitive))
            continue;
        out.append(it.key());
    }
    out.sort(Qt::CaseInsensitive);
    return out;
}

QString FolderAccess::uriFor(const QString &displayName) const
{
    return m_entries.value(displayName);
}

QString FolderAccess::imuFor(const QString &clipName) const
{
    return uriFor(clipName + QStringLiteral(".imu"));
}

QString FolderAccess::proxyFor(const QString &clipName) const
{
    const QFileInfo fi(clipName);
    return uriFor(fi.completeBaseName() + QStringLiteral("_thm.") + fi.suffix());
}

QString FolderAccess::keyframesFor(const QString &clipName) const
{
    return uriFor(clipName + QStringLiteral(".keyframes.json"));
}

void FolderAccess::restore(const QString &treeUri)
{
    if (treeUri.isEmpty())
        return;
    m_tree = treeUri;
    refresh();
    emit folderChanged();
}

void FolderAccess::forgetFolder()
{
    m_tree.clear();
    m_entries.clear();
    emit folderChanged();
}

void FolderAccess::rescan()
{
    if (m_tree.isEmpty())
        return;
    refresh();
    emit folderChanged();
}

#ifdef Q_OS_ANDROID

// --- Android: Storage Access Framework -------------------------------------

static QJniObject androidContext()
{
    return QJniObject(QNativeInterface::QAndroidApplication::context());
}

static QJniObject contentResolver()
{
    return androidContext().callObjectMethod("getContentResolver",
                                             "()Landroid/content/ContentResolver;");
}

static QJniObject uriFromString(const QString &s)
{
    return QJniObject::callStaticObjectMethod(
        "android/net/Uri", "parse", "(Ljava/lang/String;)Landroid/net/Uri;",
        QJniObject::fromString(s).object<jstring>());
}

// The tree's own document URI -- the parent that children are created under.
static QJniObject treeDocumentUri(const QString &treeUriString)
{
    const QJniObject tree = uriFromString(treeUriString);
    const QJniObject docId = QJniObject::callStaticObjectMethod(
        "android/provider/DocumentsContract", "getTreeDocumentId",
        "(Landroid/net/Uri;)Ljava/lang/String;", tree.object());
    return QJniObject::callStaticObjectMethod(
        "android/provider/DocumentsContract", "buildDocumentUriUsingTree",
        "(Landroid/net/Uri;Ljava/lang/String;)Landroid/net/Uri;",
        tree.object(), docId.object<jstring>());
}

void FolderAccess::pickFolder()
{
    QJniObject action = QJniObject::fromString(QStringLiteral("android.intent.action.OPEN_DOCUMENT_TREE"));
    QJniObject intent("android/content/Intent", "(Ljava/lang/String;)V", action.object<jstring>());
    if (!intent.isValid()) {
        emit pickFailed(tr("Could not create the folder-picker intent"));
        return;
    }
    // Ask for persistable read+write so the grant survives a restart and the
    // keyframe sidecar can be written back.
    intent.callObjectMethod("addFlags", "(I)Landroid/content/Intent;",
                            jint(kFlagGrantRead | kFlagGrantWrite | kFlagGrantPersistable));

    QtAndroidPrivate::startActivity(
        intent, kRequestPickFolder,
        [this](int /*requestCode*/, int resultCode, const QJniObject &data) {
            if (resultCode != kResultOk || !data.isValid())
                return;                              // user cancelled
            const QJniObject uri = data.callObjectMethod("getData", "()Landroid/net/Uri;");
            if (!uri.isValid()) {
                emit pickFailed(tr("The picker returned no folder"));
                return;
            }
            // Without this the grant dies with the activity.
            contentResolver().callMethod<void>(
                "takePersistableUriPermission", "(Landroid/net/Uri;I)V",
                uri.object(), jint(kFlagGrantRead | kFlagGrantWrite));

            m_tree = uri.callObjectMethod("toString", "()Ljava/lang/String;").toString();
            refresh();
            emit folderChanged();
        });
}

void FolderAccess::refresh()
{
    m_entries.clear();
    if (m_tree.isEmpty())
        return;

    const QJniObject tree = uriFromString(m_tree);
    const QJniObject docId = QJniObject::callStaticObjectMethod(
        "android/provider/DocumentsContract", "getTreeDocumentId",
        "(Landroid/net/Uri;)Ljava/lang/String;", tree.object());
    const QJniObject childrenUri = QJniObject::callStaticObjectMethod(
        "android/provider/DocumentsContract", "buildChildDocumentsUriUsingTree",
        "(Landroid/net/Uri;Ljava/lang/String;)Landroid/net/Uri;",
        tree.object(), docId.object<jstring>());
    if (!childrenUri.isValid())
        return;

    QJniEnvironment env;
    jclass stringClass = env->FindClass("java/lang/String");
    jobjectArray projection = env->NewObjectArray(2, stringClass, nullptr);
    const QJniObject colId = QJniObject::fromString(QStringLiteral("document_id"));
    const QJniObject colName = QJniObject::fromString(QStringLiteral("_display_name"));
    env->SetObjectArrayElement(projection, 0, colId.object());
    env->SetObjectArrayElement(projection, 1, colName.object());

    const QJniObject cursor = contentResolver().callObjectMethod(
        "query",
        "(Landroid/net/Uri;[Ljava/lang/String;Ljava/lang/String;[Ljava/lang/String;Ljava/lang/String;)"
        "Landroid/database/Cursor;",
        childrenUri.object(), projection, nullptr, nullptr, nullptr);
    env->DeleteLocalRef(projection);
    if (!cursor.isValid())
        return;

    while (cursor.callMethod<jboolean>("moveToNext")) {
        const QString id = cursor.callObjectMethod("getString", "(I)Ljava/lang/String;", 0).toString();
        const QString name = cursor.callObjectMethod("getString", "(I)Ljava/lang/String;", 1).toString();
        if (id.isEmpty() || name.isEmpty())
            continue;
        const QJniObject docUri = QJniObject::callStaticObjectMethod(
            "android/provider/DocumentsContract", "buildDocumentUriUsingTree",
            "(Landroid/net/Uri;Ljava/lang/String;)Landroid/net/Uri;",
            tree.object(), QJniObject::fromString(id).object<jstring>());
        if (docUri.isValid())
            m_entries.insert(name, docUri.callObjectMethod("toString", "()Ljava/lang/String;").toString());
    }
    cursor.callMethod<void>("close");
}

QString FolderAccess::writableUriFor(const QString &displayName, const QString &mimeType)
{
    if (m_tree.isEmpty())
        return QString();
    // Already there? Then it is writable through the tree grant.
    const QString existing = m_entries.value(displayName);
    if (!existing.isEmpty())
        return existing;

    const QJniObject parent = treeDocumentUri(m_tree);
    const QJniObject created = QJniObject::callStaticObjectMethod(
        "android/provider/DocumentsContract", "createDocument",
        "(Landroid/content/ContentResolver;Landroid/net/Uri;Ljava/lang/String;Ljava/lang/String;)"
        "Landroid/net/Uri;",
        contentResolver().object(), parent.object(),
        QJniObject::fromString(mimeType).object<jstring>(),
        QJniObject::fromString(displayName).object<jstring>());
    if (!created.isValid())
        return QString();

    const QString uri = created.callObjectMethod("toString", "()Ljava/lang/String;").toString();
    // createDocument may uniquify the name (foo (1).json); re-list so the map
    // reflects what is actually there.
    refresh();
    return uri;
}

#else  // --- everywhere else: a plain directory ------------------------------

void FolderAccess::pickFolder()
{
    // Desktop has a working file dialog and derives sidecars from the path;
    // there is nothing for a folder grant to add.
}

void FolderAccess::refresh()
{
    m_entries.clear();
    if (m_tree.isEmpty())
        return;
    const QDir dir(m_tree);
    const auto files = dir.entryInfoList(QDir::Files | QDir::Readable, QDir::Name);
    for (const QFileInfo &fi : files)
        m_entries.insert(fi.fileName(), fi.absoluteFilePath());
}

QString FolderAccess::writableUriFor(const QString &displayName, const QString &)
{
    if (m_tree.isEmpty())
        return QString();
    return QDir(m_tree).filePath(displayName);
}

#endif
