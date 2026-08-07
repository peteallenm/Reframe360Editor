#include <QGuiApplication>
#include <QQmlApplicationEngine>
#include <QQuickStyle>
#include <QQmlContext>
#include <QUrl>
#include <QCommandLineParser>
#include <QTimer>
#include <QEventLoop>
#include <QQuickWindow>
#include "app.h"

int main(int argc, char *argv[])
{
    // Qt's default platform theme provides no native file-dialog helper, so
    // FileDialog silently falls back to a non-native dialog that ignores
    // fileMode: SaveFile (it always shows "Open"). The GTK3 platform theme
    // supplies a real save-as dialog helper, so prefer it when available
    // (Qt falls back gracefully if the theme plugin is missing).
    if (qEnvironmentVariableIsEmpty("QT_QPA_PLATFORMTHEME"))
        qputenv("QT_QPA_PLATFORMTHEME", "gtk3");

    // We also deliberately do NOT set QT_QUICK_CONTROLS_NATIVE_DIALOGS=0:
    // the non-native Qt Quick dialog implementation ignores SaveFile mode.
    QGuiApplication app(argc, argv);
    app.setApplicationName("render360");
    app.setOrganizationName("render360");
    app.setApplicationVersion("1.0.0");

    QCommandLineParser parser;
    parser.setApplicationDescription("360° video viewer with fisheye dewarping");
    parser.addHelpOption();
    parser.addVersionOption();
    
    QCommandLineOption videoOption(QStringList() << "i" << "input",
                                   "Load video file", "file");
    parser.addOption(videoOption);
    
    QCommandLineOption headlessOption("headless", "Run without GUI (for testing)");
    parser.addOption(headlessOption);
    
    QCommandLineOption grabOption("grab", "Grab the window after a short delay and save to FILE", "file");
    parser.addOption(grabOption);
    
    QCommandLineOption projectionOption("projection", "Rendering projection: 0 perspective, 1 equirectangular, 2 stereographic, 3 sportsview", "mode");
    parser.addOption(projectionOption);
    
    QCommandLineOption lensOption("lens", "Lens mode: 0 front, 1 rear, 2 auto stitch", "mode");
    parser.addOption(lensOption);

    QCommandLineOption imuOption("imu", "Enable IMU stabilization");
    parser.addOption(imuOption);
    
    parser.process(app);

    QQuickStyle::setStyle("Material");

    App appController;

    if (parser.isSet(headlessOption)) {
        if (parser.isSet(videoOption)) {
            QString videoPath = parser.value(videoOption);
            
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
