#ifndef FOLDERACCESS_H
#define FOLDERACCESS_H

#include <QObject>
#include <QMap>
#include <QString>
#include <QStringList>

// ---------------------------------------------------------------------------
// Access to a whole FOLDER of clips.
//
// Android's single-file picker returns an opaque content:// URI: you cannot
// append ".imu" to it, and the grant covers only that one file, so sidecars are
// unreachable and nothing can be written back. The Storage Access Framework's
// answer is to grant a TREE: the user picks the folder once, the grant is
// persistable across reboots, and every child can then be addressed by name.
//
// That is what this class wraps. On Android it drives
// ACTION_OPEN_DOCUMENT_TREE through JNI and keeps a name -> document-URI map of
// the folder's contents. Everywhere else it is a thin shim over a plain
// directory, so the same App code works on both.
// ---------------------------------------------------------------------------
class FolderAccess : public QObject
{
    Q_OBJECT
    Q_PROPERTY(bool available READ isAvailable CONSTANT)
    Q_PROPERTY(QString folderName READ folderName NOTIFY folderChanged)
    Q_PROPERTY(bool hasFolder READ hasFolder NOTIFY folderChanged)
    Q_PROPERTY(QStringList clips READ clips NOTIFY folderChanged)

public:
    explicit FolderAccess(QObject *parent = nullptr);

    // True where picking a folder is meaningful and implemented (Android).
    // Elsewhere the ordinary file dialog is the right tool and this stays off.
    static bool isAvailable();

    bool hasFolder() const { return !m_tree.isEmpty(); }
    QString folderName() const;

    // Display names of the video files in the folder, proxies ("*_thm.*")
    // excluded -- they are sidecars of another entry, not clips in their own
    // right. Sorted.
    QStringList clips() const;

    // Ask the user for a folder. Asynchronous: folderChanged() is emitted when
    // a folder has been granted and its contents listed.
    Q_INVOKABLE void pickFolder();

    // Forget the current grant.
    Q_INVOKABLE void forgetFolder();

    // The document URI of a file in the folder, or an empty string when the
    // folder holds no such name.
    Q_INVOKABLE QString uriFor(const QString &displayName) const;

    // Sidecars of a clip, by the project's naming rules:
    //   <clip>.imu, <base>_thm.<ext>, <clip>.keyframes.json
    // Each is empty when the folder does not contain it.
    Q_INVOKABLE QString imuFor(const QString &clipName) const;
    Q_INVOKABLE QString proxyFor(const QString &clipName) const;
    Q_INVOKABLE QString keyframesFor(const QString &clipName) const;

    // A writable URI for a file in the folder, creating it if absent. Used for
    // the keyframe sidecar, which a single-file grant could never write.
    // Returns an empty string if the file cannot be created.
    QString writableUriFor(const QString &displayName, const QString &mimeType);

    // Restore a previously granted folder (called at startup).
    void restore(const QString &treeUri);
    QString treeUri() const { return m_tree; }

signals:
    void folderChanged();
    void pickFailed(const QString &message);

private:
    void refresh();          // re-list the folder's contents

    QString m_tree;                       // tree URI, or a local path off-Android
    QMap<QString, QString> m_entries;     // display name -> document URI
};

#endif // FOLDERACCESS_H
