#ifndef EXPORTER_H
#define EXPORTER_H

#include <QObject>
#include <QString>
#include <QQuaternion>
#include <functional>

#include "flowrenderer.h"

class QThread;

// Everything needed to render one output frame, mirroring the uniforms the
// ViewerMaterial sends to project.frag. Filled in by App (which knows the
// keyframe interpolation, IMU orientation, calibration, etc.) and evaluated
// once per exported frame, from the export worker thread.
struct ExportFrameState {
    double yaw = 0.0;
    double pitch = 0.0;
    double roll = 0.0;
    double fov = 90.0;
    int activeLens = 2;   // 0 front, 1 rear, 2 auto stitch
    int projection = 0;   // 0 perspective, 1 equirect, 2 stereographic, 3 sportsview
    bool fullRange = true;

    // Front lens calibration
    float frontCenterX = 0.5f;
    float frontCenterY = 0.5f;
    float frontRadius = 0.5f;
    float frontK1 = 0.0f;
    float frontK2 = 0.0f;
    float frontRotation = 0.0f;
    bool frontHFlip = false;

    // Rear lens calibration
    float rearCenterX = 0.5f;
    float rearCenterY = 0.5f;
    float rearRadius = 0.5f;
    float rearK1 = 0.0f;
    float rearK2 = 0.0f;
    float rearRotation = 180.0f;
    bool rearHFlip = false;

    float blendStart = 0.9f;

    // Optical-flow stitching (per-export settings, not keyframed). When
    // flowStitch is set the exporter estimates the Horn-Schunck parallax field
    // with FlowRenderer and uploads it to GpuRenderer before rendering.
    bool flowStitch = false;
    float flowStrength = 1.0f;
    int flowIterations = kDefaultFlowIterations;
    float flowAlpha = kDefaultFlowAlpha;
    float bandTheta0 = kDefaultBandTheta0;
    float bandTheta1 = kDefaultBandTheta1;

    // Colour grading (mirrors the ColorGrade properties and project.frag
    // uniforms, so exported frames match the viewer).
    float brightness = 0.0f;   // additive offset (-1..1)
    float contrast = 1.0f;     // scale about 0.5 (0..2)
    float saturation = 1.0f;   // mix with luma (0..2)
    float pop = 0.0f;          // midtone contrast / clarity (-1..1)
    float brightLows = 0.0f, brightLowMids = 0.0f, brightHighMids = 0.0f, brightHighs = 0.0f;
    float redLows = 0.0f, redMids = 0.0f, redHighs = 0.0f;
    float greenLows = 0.0f, greenMids = 0.0f, greenHighs = 0.0f;
    float blueLows = 0.0f, blueMids = 0.0f, blueHighs = 0.0f;

    // Same quaternion App::imuOrientation() reports; the exporter applies its
    // conjugate to the view ray, exactly like LensViewer does.
    QQuaternion imuOrientation{1.0f, 0.0f, 0.0f, 0.0f};
};

class Exporter : public QObject
{
    Q_OBJECT
public:
    explicit Exporter(QObject *parent = nullptr);
    ~Exporter();

    // Returns per-time render state. Called from the export worker thread, so
    // it must only touch snapshot/copy data (never live GUI objects).
    using StateProvider = std::function<ExportFrameState(double time)>;

    // Render <videoPath> over [startTime, endTime] at fps to <outPath> (MP4).
    // Runs on a worker thread; progress/finished/error are signalled back.
    // If useGpu is true the frames are rendered with the offscreen GL
    // pipeline (same shader as the live viewer); if no usable GL context can
    // be created it automatically falls back to the CPU rasterizer.
    void exportVideo(const QString &videoPath, const QString &outPath,
                     int width, int height, double fps,
                     double startTime, double endTime,
                     const StateProvider &state, bool useGpu = true);

    // Render a single frame at <time> and save it as an image (PNG/JPG by
    // extension) to <outPath>.
    void exportFrame(const QString &videoPath, const QString &outPath,
                     int width, int height, double time,
                     const ExportFrameState &state, bool useGpu = true);

    bool isRunning() const { return m_thread != nullptr; }

signals:
    void exportProgress(double progress);   // 0..1
    void exportFinished(const QString &path);
    void exportError(const QString &message);

private:
    // Refuse to start a new export while one is running; cleans up a finished
    // thread whose queued cleanup hasn't been delivered yet.
    bool beginExport();

    void runVideo(const QString &videoPath, const QString &outPath,
                  int width, int height, double fps,
                  double startTime, double endTime, StateProvider state,
                  bool useGpu);
    void runFrame(const QString &videoPath, const QString &outPath,
                  int width, int height, double time, ExportFrameState state,
                  bool useGpu);

    QThread *m_thread = nullptr;
};

#endif // EXPORTER_H
