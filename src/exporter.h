#ifndef EXPORTER_H
#define EXPORTER_H

#include <QObject>
#include <QString>

class Exporter : public QObject
{
    Q_OBJECT
public:
    explicit Exporter(QObject *parent = nullptr);

    Q_INVOKABLE void exportFrame(const QString &path, int width, int height);
    Q_INVOKABLE void exportVideo(const QString &path, int width, int height,
                                  double fps, double startTime, double endTime);

signals:
    void exportProgress(double progress);
    void exportFinished(const QString &path);
    void exportError(const QString &message);
};

#endif // EXPORTER_H
