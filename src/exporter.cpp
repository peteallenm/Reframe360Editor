#include "exporter.h"

Exporter::Exporter(QObject *parent)
    : QObject(parent)
{
}

void Exporter::exportFrame(const QString &path, int width, int height)
{
    Q_UNUSED(path);
    Q_UNUSED(width);
    Q_UNUSED(height);
}

void Exporter::exportVideo(const QString &path, int width, int height,
                            double fps, double startTime, double endTime)
{
    Q_UNUSED(path);
    Q_UNUSED(width);
    Q_UNUSED(height);
    Q_UNUSED(fps);
    Q_UNUSED(startTime);
    Q_UNUSED(endTime);
}
