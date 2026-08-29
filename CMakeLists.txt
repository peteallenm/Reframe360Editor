cmake_minimum_required(VERSION 3.19)
project(render360 VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

find_package(Qt6 REQUIRED COMPONENTS Quick QuickControls2 ShaderTools OpenGL)
find_package(PkgConfig REQUIRED)
pkg_check_modules(FFMPEG REQUIRED libavformat libavcodec libavutil libswscale)
# videoio is used only by the tracking_tests diagnostics (--seam-check),
# which grab arbitrary frames directly rather than via the FFmpeg decode path.
find_package(OpenCV REQUIRED COMPONENTS core imgproc features2d calib3d videoio)

qt_add_executable(render360
    src/main.cpp
    src/app.cpp
    src/videodecoder.cpp
    src/imuparser.cpp
    src/gyroscopeintegrator.cpp
    src/lensviewer.cpp
    src/viewermaterial.cpp
    src/calibration.cpp
    src/keyframe.cpp
    src/exporter.cpp
    src/colorgrade.cpp
    src/gpurenderer.cpp
    src/vidstab.cpp
    src/flowrenderer.cpp
    src/visualrotation.cpp
    src/syncsolver.cpp
    src/visualfusion.cpp
    src/gyrocalibration.cpp
)

# Independent tracking/diagnostics test harness. Builds the IMU/visual/gyro
# pipeline modules only (no GUI) and verifies each stage.
#   cmake --build build --target tracking_tests
#   build/tracking_tests --quick <video.mp4>
qt_add_executable(tracking_tests
    src/tracking_tests.cpp
    src/imuparser.cpp
    src/gyroscopeintegrator.cpp
    src/calibration.cpp
    src/visualrotation.cpp
    src/syncsolver.cpp
    src/gyrocalibration.cpp
    src/visualfusion.cpp
    src/keyframe.cpp
)

qt_add_qml_module(render360
    URI Render360
    VERSION 1.0
    QML_FILES
        qml/main.qml
        qml/ViewerPane.qml
        qml/ControlPanel.qml
        qml/Timeline.qml
        qml/CalibrationDialog.qml
        qml/ExportDialog.qml
    RESOURCES
        resources/presets/yi360_default.json
)

qt_add_shaders(render360 "shaders"
    PREFIX "/"
    BATCHABLE
    FILES
        shaders/quad.vert
        shaders/project.frag
    GLSL "100 es,120,330,440,450,460"
)

# Embed the raw GLSL too: the GPU exporter compiles project.frag/quad.vert
# directly (the same files the viewer's qsb is baked from), so preview and
# export can never drift apart. The optical-flow shaders are also embedded
# here — they are compiled at runtime by FlowRenderer's core-4.4 context (they
# use MRT / layout(binding=...) / RG32F, which cannot be baked for GLSL 100es
# or 120), so they live only in this raw resource block.
qt_add_resources(render360 "shaders_raw"
    PREFIX "/shaders_raw"
    FILES
        shaders/quad.vert
        shaders/project.frag
        shaders/band_extract.frag
        shaders/disp_match.frag
        shaders/seam_cost.frag
)

target_include_directories(render360 PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${FFMPEG_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(render360 PRIVATE
    Qt6::Quick
    Qt6::QuickControls2
    Qt6::OpenGL
    ${FFMPEG_LIBRARIES}
    ${OpenCV_LIBS}
    OpenGL::GL
)

target_include_directories(tracking_tests PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${FFMPEG_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
)

target_link_libraries(tracking_tests PRIVATE
    Qt6::Core
    Qt6::Gui
    ${FFMPEG_LIBRARIES}
    ${OpenCV_LIBS}
)

target_compile_definitions(tracking_tests PRIVATE
    QT_DEPRECATED_WARNINGS
    QT_DISABLE_DEPRECATED_BEFORE=0x060000
)

target_compile_definitions(render360 PRIVATE
    QT_DEPRECATED_WARNINGS
    QT_DISABLE_DEPRECATED_BEFORE=0x060000
)
