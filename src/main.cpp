// Reframe360 Editor -- 360 video stabiliser and stitcher for dual-fisheye footage.
// Copyright (C) 2026 Peter Allen
// SPDX-License-Identifier: GPL-3.0-or-later
//
// This program is free software under the GNU General Public License, version
// 3 or (at your option) any later version; see LICENSE for the full text.
// It is distributed in the hope that it will be useful, but WITHOUT ANY
// WARRANTY, without even the implied warranty of MERCHANTABILITY or FITNESS
// FOR A PARTICULAR PURPOSE.

#include <QGuiApplication>
#include <QSettings>
#include <QQmlApplicationEngine>
#include <QQuickStyle>
#include <QQmlContext>
#include <QUrl>
#include <QCommandLineParser>
#include <QTimer>
#include <QEventLoop>
#include <QQuickWindow>
#include <QFile>
#include "app.h"
#include "videodecoder.h"
#include "flowrenderer.h"

#ifdef Q_OS_ANDROID
#include <QJniEnvironment>
extern "C" {
#include <libavcodec/jni.h>
}
#endif

int main(int argc, char *argv[])
{
    // Register the types queued to the optical-flow worker thread.
    qRegisterMetaType<DecodedFrame>();
    qRegisterMetaType<FlowCalibration>();
    qRegisterMetaType<FlowSettings>();

    // Qt's default platform theme provides no native file-dialog helper, so
    // FileDialog silently falls back to a non-native dialog that ignores
    // fileMode: SaveFile (it always shows "Open"). The GTK3 platform theme
    // supplies a real save-as dialog helper, so prefer it when available
    // (Qt falls back gracefully if the theme plugin is missing).
    //
    // Desktop Linux only: Android has its own theme and file picker (and the
    // storage model there is the Storage Access Framework, not paths), so
    // forcing gtk3 would just log a missing-plugin warning.
#if defined(Q_OS_LINUX) && !defined(Q_OS_ANDROID)
    if (qEnvironmentVariableIsEmpty("QT_QPA_PLATFORMTHEME"))
        qputenv("QT_QPA_PLATFORMTHEME", "gtk3");
#endif

    // We also deliberately do NOT set QT_QUICK_CONTROLS_NATIVE_DIALOGS=0:
    // the non-native Qt Quick dialog implementation ignores SaveFile mode.
    QGuiApplication app(argc, argv);

#ifdef Q_OS_ANDROID
    // FFmpeg's MediaCodec decoders/encoders reach Java through JNI and need
    // the VM registered once up front; without this every *_mediacodec open
    // fails and playback silently stays on software decode.
    av_jni_set_java_vm(QJniEnvironment::javaVM(), nullptr);
#endif
    // Display name is set separately below. applicationName/organizationName
    // are the SETTINGS IDENTITY -- QSettings derives ~/.config/<org>/<app>.conf
    // from them, and tracking_tests hardcodes the same pair. Renaming these
    // would orphan every stored setting and the camera-wide gyro calibration,
    // so they stay as they are.
    app.setApplicationName("render360");
    app.setOrganizationName("render360");
    app.setApplicationDisplayName("Reframe360 Editor");
    app.setApplicationVersion("1.0.0");

    QCommandLineParser parser;
    parser.setApplicationDescription(
        "Reframe360 Editor -- stabilise, reframe and stitch dual-fisheye 360 footage");
    parser.addHelpOption();
    // Not addVersionOption(): its output is built from applicationName, which
    // is the QSettings identity ("render360") and not what the product is
    // called. Same spelling of the option, handled below.
    QCommandLineOption versionOption(QStringList() << "v" << "version",
                                     "Displays version information.");
    parser.addOption(versionOption);
    
    QCommandLineOption videoOption(QStringList() << "i" << "input",
                                   "Load video file", "file");
    parser.addOption(videoOption);
    
    QCommandLineOption headlessOption("headless", "Run without GUI (for testing)");
    parser.addOption(headlessOption);
    
    QCommandLineOption grabOption("grab", "Grab the window after a short delay and save to FILE", "file");
    parser.addOption(grabOption);

    QCommandLineOption tabOption("tab", "Start on the given side-panel tab (0 View, 1 Stabilise, 2 Stitch, 3 Colour)", "index");
    parser.addOption(tabOption);
    
    QCommandLineOption projectionOption("projection", "Rendering projection: 0 perspective, 1 equirectangular, 2 stereographic, 3 sportsview", "mode");
    parser.addOption(projectionOption);
    
    QCommandLineOption lensOption("lens", "Lens mode: 0 front, 1 rear, 2 auto stitch", "mode");
    parser.addOption(lensOption);

    QCommandLineOption imuOption("imu", "Enable IMU stabilization");
    parser.addOption(imuOption);

    // The camera-wide gyro calibration is applied silently to every clip that
    // has no calibration of its own, so a bad one is invisible and permanent.
    // This is the scriptable escape hatch (the GUI equivalent is
    // App::clearCameraGyroDefaults()).
    QCommandLineOption clearCameraCalOption(
        "clear-camera-calibration",
        "Delete the camera-wide gyro calibration defaults and exit");
    parser.addOption(clearCameraCalOption);

    // Headless video export (implies --headless): render the loaded video to
    // an MP4, then exit. The trim range defaults to the whole clip.
    QCommandLineOption exportVideoOption("export-video", "Export the loaded video to FILE as MP4 (headless)", "file");
    parser.addOption(exportVideoOption);
    QCommandLineOption exportWOption("export-w", "Export width (default 640)", "px");
    parser.addOption(exportWOption);
    QCommandLineOption exportHOption("export-h", "Export height (default 360)", "px");
    parser.addOption(exportHOption);
    QCommandLineOption exportFpsOption("export-fps", "Export frame rate (default 30)", "fps");
    parser.addOption(exportFpsOption);
    QCommandLineOption exportStartOption("export-start", "Export start time in seconds (default 0)", "sec");
    parser.addOption(exportStartOption);
    QCommandLineOption exportEndOption("export-end", "Export end time in seconds (default: clip duration)", "sec");
    parser.addOption(exportEndOption);
    QCommandLineOption exportBackendOption("export-backend", "Render backend: gpu, cpu or auto (default auto)", "backend");
    QCommandLineOption exportSphericalOption("export-spherical", "Tag the export as a 360 equirectangular video (requires --projection 1)");
    QCommandLineOption exportNoAudioOption("export-no-audio", "Leave the clip's audio out of the export (it is copied in by default)");
    parser.addOption(exportBackendOption);
    parser.addOption(exportSphericalOption);
    parser.addOption(exportNoAudioOption);
    QCommandLineOption exportCodecOption("export-codec", "Encoder: libx264, libx265 or hevc_nvenc (default libx264)", "codec");
    parser.addOption(exportCodecOption);
    QCommandLineOption exportCrfOption("export-crf", "CRF quality for CPU codecs, 0..51 (default 19)", "crf");
    parser.addOption(exportCrfOption);
    QCommandLineOption exportBitrateOption("export-bitrate", "Target bitrate in Mbps for hevc_nvenc (default 12)", "mbps");
    parser.addOption(exportBitrateOption);
    QCommandLineOption exportVidstabOption("export-vidstab", "Enable FFmpeg vidstab post-processing stabilization");
    parser.addOption(exportVidstabOption);
    QCommandLineOption exportVidstabHybridOption("export-vidstab-hybrid", "Hybrid stabilization (EXPERIMENTAL): vidstab-detected residuals folded into the native render (single-pass, no re-encode)");
    parser.addOption(exportVidstabHybridOption);
    
    parser.process(app);

    if (parser.isSet(versionOption)) {
        printf("%s %s\n", qPrintable(QGuiApplication::applicationDisplayName()),
                           qPrintable(QCoreApplication::applicationVersion()));
        return 0;
    }

    // Handle before any window or App is constructed: this is a maintenance
    // action, not a viewing session.
    if (parser.isSet(clearCameraCalOption)) {
        QSettings s;
        const bool had = s.contains(QStringLiteral("camera/gyroMatrix"))
                      || s.contains(QStringLiteral("camera/gyroBias"))
                      || s.contains(QStringLiteral("camera/gyroScaleX"));
        s.remove(QStringLiteral("camera/gyroMatrix"));
        s.remove(QStringLiteral("camera/gyroBias"));
        s.remove(QStringLiteral("camera/gyroScaleX"));
        s.remove(QStringLiteral("camera/gyroScaleY"));
        s.remove(QStringLiteral("camera/gyroScaleZ"));
        s.sync();
        qInfo().noquote() << (had ? "Camera-wide gyro calibration defaults cleared."
                                  : "No camera-wide gyro calibration defaults were set.");
        return 0;
    }

    QQuickStyle::setStyle("Material");

    App appController;

    // View options apply to headless exports as well as the GUI, so set them
    // before the headless branch starts its export (they used to be applied
    // only on the GUI path below, which made --lens/--projection/--imu silent
    // no-ops for --export-video).
    if (parser.isSet(projectionOption))
        appController.setProjection(parser.value(projectionOption).toInt());
    if (parser.isSet(lensOption))
        appController.setActiveLens(parser.value(lensOption).toInt());
    if (parser.isSet(imuOption))
        appController.setImuStabilize(true);

    if (parser.isSet(headlessOption)) {
        if (parser.isSet(videoOption)) {
            QString videoPath = parser.value(videoOption);
            
            if (parser.isSet(exportVideoOption)) {
                // Export the clip headlessly, then exit 0 on success / 1 on
                // failure once the worker reports completion.
                const QString outPath = parser.value(exportVideoOption);
                const int w = parser.value(exportWOption).toInt() ? parser.value(exportWOption).toInt() : 640;
                const int h = parser.value(exportHOption).toInt() ? parser.value(exportHOption).toInt() : 360;
                const double fps = parser.value(exportFpsOption).toDouble() > 0.0
                        ? parser.value(exportFpsOption).toDouble() : 30.0;
                const double start = parser.value(exportStartOption).toDouble();
                const bool hasEnd = parser.isSet(exportEndOption);
                const double end = hasEnd ? parser.value(exportEndOption).toDouble() : -1.0;
                const bool gpu = parser.value(exportBackendOption) != QLatin1String("cpu");
                const QString codec = parser.value(exportCodecOption).isEmpty()
                        ? QStringLiteral("libx264") : parser.value(exportCodecOption);
                const int crf = (parser.value(exportCrfOption).toInt() > 0)
                        ? parser.value(exportCrfOption).toInt() : 19;
                const int bitrate = (parser.value(exportBitrateOption).toInt() > 0)
                        ? parser.value(exportBitrateOption).toInt() : 12;
                const bool vidstab = parser.isSet(exportVidstabOption);
                const bool vidstabInformed = parser.isSet(exportVidstabHybridOption);
                const bool enableImu = parser.isSet(imuOption);
                const int projIdx = parser.isSet(projectionOption) ? parser.value(projectionOption).toInt() : -1;

                // Kick off the decoder, wait for it to report the clip
                // duration (needed for the default end time), then start the
                // export. The completion poll is only started afterwards, so
                // it can never fire before the export begins.
                QTimer::singleShot(0, [&appController, videoPath, enableImu, projIdx]() {
                    appController.setVideoPath(videoPath);
                    if (enableImu)
                        appController.setImuStabilize(true);
                    if (projIdx >= 0)
                        appController.setProjection(projIdx);
                });
                QTimer *wait = new QTimer(&app);
                wait->setInterval(100);
                const bool exportSpherical = parser.isSet(exportSphericalOption);
                const bool exportAudio = !parser.isSet(exportNoAudioOption);
                QObject::connect(wait, &QTimer::timeout, &app, [&app, &appController, wait, outPath, w, h, fps, start, end, gpu, codec, crf, bitrate, vidstab, vidstabInformed, exportSpherical, exportAudio]() {
                    if (appController.duration() <= 0.0)
                        return;   // keep waiting for the decoder
                    wait->stop();
                    const double startTime = start;
                    const double endTime = (end >= 0.0) ? end : appController.duration();
                    appController.exportVideo(outPath, w, h, fps, startTime, endTime, codec, crf, bitrate, vidstab, vidstabInformed, gpu, exportSpherical,
                                              exportAudio);

                    // Poll until the export finishes, then exit with a code
                    // that reflects success/failure.
                    QTimer *poll = new QTimer(&app);
                    QObject::connect(poll, &QTimer::timeout, &app, [&appController, outPath]() {
                        if (appController.exportRunning())
                            return;
                        const QString status = appController.exportStatus();
                        const bool ok = status.startsWith(QLatin1String("Export complete"))
                                     || QFile::exists(outPath);
                        qDebug().noquote() << "Export status:" << status;
                        QCoreApplication::exit(ok ? 0 : 1);
                    });
                    poll->start(250);
                });
                wait->start();
            } else {
                QTimer::singleShot(0, [&appController, videoPath]() {
                    appController.setVideoPath(videoPath);
                    
                    QTimer::singleShot(3000, [&appController, videoPath]() {
                        qDebug() << "Video loaded successfully:" << videoPath;
                        qDebug() << "Duration:" << appController.duration() << "seconds";
                        if (appController.videoDecoder()) {
                            qDebug() << "Video size:" << appController.videoDecoder()->videoWidth() 
                                     << "x" << appController.videoDecoder()->videoHeight();
                            qDebug() << "Full range:" << appController.videoDecoder()->isFullRange();
                        }
                        QCoreApplication::quit();
                    });
                });
            }
        }
        return app.exec();
    }

    QQmlApplicationEngine engine;
    engine.rootContext()->setContextProperty("app", &appController);

    const QUrl url(QStringLiteral("qrc:/Render360/qml/main.qml"));
    QObject::connect(
        &engine,
        &QQmlApplicationEngine::objectCreated,
        &app,
        [url](QObject *obj, const QUrl &objUrl) {
            if (!obj && url == objUrl)
                QCoreApplication::exit(-1);
        },
        Qt::QueuedConnection);
    engine.load(url);

    if (parser.isSet(videoOption)) {
        QString videoPath = parser.value(videoOption);
        appController.setVideoPath(videoPath);
        appController.setIsPlaying(true);
    }

    if (parser.isSet(projectionOption))
        appController.setProjection(parser.value(projectionOption).toInt());
    if (parser.isSet(lensOption))
        appController.setActiveLens(parser.value(lensOption).toInt());

    if (parser.isSet(imuOption))
        appController.setImuStabilize(true);

    if (parser.isSet(tabOption)) {
        QTimer::singleShot(0, [&engine, tabIdx = parser.value(tabOption).toInt()]() {
            QObject *root = engine.rootObjects().value(0);
            if (root) {
                if (QObject *tabs = root->findChild<QObject*>(QStringLiteral("panelTabs")))
                    tabs->setProperty("currentIndex", qBound(0, tabIdx, 3));
            }
        });
    }

    if (parser.isSet(grabOption)) {
        QString grabPath = parser.value(grabOption);
        QTimer::singleShot(3000, [&engine, &appController, grabPath]() {
            QObject *root = engine.rootObjects().value(0);
            if (root) {
                QQuickWindow *win = qobject_cast<QQuickWindow*>(root);
                if (win) {
                    QImage img = win->grabWindow();
                    img.save(grabPath);
                    qDebug() << "Grabbed window" << img.size() << img.isNull() << "->" << grabPath;
                    if (appController.videoDecoder() && appController.videoDecoder()->hasFrame()) {
                        qDebug() << "Grabbed frame ts" << appController.videoDecoder()->currentFrame().timestamp;
                    }
                }
            }
            QCoreApplication::quit();
        });
    }

    return app.exec();
}
