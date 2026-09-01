# Reframe360 Editor -- 360 video stabiliser and stitcher for dual-fisheye footage.
# Copyright (C) 2026 Peter Allen
# SPDX-License-Identifier: GPL-3.0-or-later
#
# This program is free software under the GNU General Public License, version
# 3 or (at your option) any later version; see LICENSE for the full text.
# It is distributed in the hope that it will be useful, but WITHOUT ANY
# WARRANTY, without even the implied warranty of MERCHANTABILITY or FITNESS
# FOR A PARTICULAR PURPOSE.

cmake_minimum_required(VERSION 3.19)
project(render360 VERSION 1.0.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)
set(CMAKE_AUTOUIC ON)

find_package(Qt6 REQUIRED COMPONENTS Quick QuickControls2 ShaderTools OpenGL)
if(ANDROID)
    # QtAndroidPrivate::startActivity() -- the activity-result callback the
    # folder picker needs -- is private Core API.
    find_package(Qt6 REQUIRED COMPONENTS CorePrivate)
endif()

# --- FFmpeg -----------------------------------------------------------------
# Desktop Linux has pkg-config. An Android NDK build has neither pkg-config nor
# system libraries, so point RENDER360_FFMPEG_DIR at a prebuilt arm64-v8a tree
# (include/ + lib/); see android/README.md for how to build one.
if(ANDROID)
    set(RENDER360_FFMPEG_DIR "" CACHE PATH
        "Prebuilt FFmpeg for this Android ABI (contains include/ and lib/)")
    if(NOT RENDER360_FFMPEG_DIR)
        message(FATAL_ERROR
            "Android build needs -DRENDER360_FFMPEG_DIR=<prebuilt ffmpeg for ${ANDROID_ABI}>. "
            "See android/README.md.")
    endif()

    # A multi-ABI APK configures once per ABI, so the path may name either one
    # ABI's tree directly or a parent holding one subdirectory per ABI. Prefer
    # the ABI-specific subdirectory when it exists.
    set(_ff "${RENDER360_FFMPEG_DIR}")
    if(EXISTS "${RENDER360_FFMPEG_DIR}/${ANDROID_ABI}/lib/libavcodec.a")
        set(_ff "${RENDER360_FFMPEG_DIR}/${ANDROID_ABI}")
    endif()
    if(NOT EXISTS "${_ff}/lib/libavcodec.a")
        message(FATAL_ERROR "No FFmpeg for ${ANDROID_ABI} under ${RENDER360_FFMPEG_DIR}")
    endif()
    message(STATUS "FFmpeg for ${ANDROID_ABI}: ${_ff}")
    # Deliberately NOT written back into RENDER360_FFMPEG_DIR: that variable is
    # forwarded verbatim to the per-ABI sub-configures of a multi-ABI build
    # (QT_ANDROID_MULTI_ABI_FORWARD_VARS), so overwriting it here sent every ABI
    # to whichever tree the first one resolved.
    set(FFMPEG_INCLUDE_DIRS "${_ff}/include")
    # Link order matters with static FFmpeg: dependents before dependencies.
    set(FFMPEG_LIBRARIES
        "${_ff}/lib/libavformat.a"
        "${_ff}/lib/libavcodec.a"
        # Software H.264 encoder (5.7K exports exceed MediaCodec's ~4K cap).
        # After avcodec, which depends on it.
        "${_ff}/lib/libx264.a"
        "${_ff}/lib/libswscale.a"
        "${_ff}/lib/libavutil.a"
        # MediaCodec decode/encode wrappers call into the Android media NDK.
        mediandk android log z m)
else()
    find_package(PkgConfig REQUIRED)
    pkg_check_modules(FFMPEG REQUIRED libavformat libavcodec libavutil libswscale)
endif()

# --- OpenCV -----------------------------------------------------------------
# videoio is used only by the tracking_tests diagnostics (--seam-check), which
# grab arbitrary frames directly rather than via the FFmpeg decode path. The
# app itself never touches it, so the Android build (which has no
# tracking_tests) does not need that module.
if(ANDROID)
    find_package(OpenCV REQUIRED COMPONENTS core imgproc features2d calib3d)
else()
    find_package(OpenCV REQUIRED COMPONENTS core imgproc features2d calib3d videoio)
endif()

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
    src/glsladapt.cpp
    src/avinput.cpp
    src/vidstab.cpp
    src/flowrenderer.cpp
    src/visualrotation.cpp
    src/syncsolver.cpp
    src/visualfusion.cpp
    src/gyrocalibration.cpp
    src/folderaccess.cpp
    src/track.cpp
    src/objecttracker.cpp
)

# Independent tracking/diagnostics test harness. Builds the IMU/visual/gyro
# pipeline modules only (no GUI) and verifies each stage.
#   cmake --build build --target tracking_tests
#   build/tracking_tests --quick <video.mp4>
if(NOT ANDROID)
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
    src/track.cpp
    src/objecttracker.cpp
    src/avinput.cpp
)
endif()

qt_add_qml_module(render360
    URI Render360
    VERSION 1.0
    QML_FILES
        qml/main.qml
        qml/ViewerPane.qml
        qml/ControlPanel.qml
        qml/CurveEditor.qml
        qml/Timeline.qml
        qml/CalibrationDialog.qml
        qml/ExportDialog.qml
        qml/AboutDialog.qml
    RESOURCES
        resources/presets/yi360_default.json
)

qt_add_shaders(render360 "shaders"
    PREFIX "/"
    BATCHABLE
    FILES
        shaders/quad.vert
        shaders/project.frag
    GLSL "100 es,300 es,120,330,440,450,460"
)

# Embed the raw GLSL too: the GPU exporter compiles project.frag/quad.vert
# directly (the same files the viewer's qsb is baked from), so preview and
# export can never drift apart. The optical-flow shaders are also embedded
# here — they are compiled at runtime by FlowRenderer's core-4.4 context (they
# use MRT / layout(binding=...) / RG32F, which cannot be baked for GLSL 100es
# or 120), so they live only in this raw resource block.
# Factory calibration presets. Seeded into a fresh install (no
# calibration_presets.json in the config dir yet) so a new machine -- notably a
# phone, which has no way to import the desktop's file -- starts with the
# measured YI 360 profiles rather than the nominal 0.5/0.5/0.5 placeholder.
qt_add_resources(render360 "presets"
    PREFIX "/presets"
    FILES
        resources/presets/calibration_presets.json
)

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
)

if(NOT ANDROID)
    target_link_libraries(render360 PRIVATE OpenGL::GL)
else()
    # QtAndroidPrivate::startActivity() (the activity-result callback the
    # folder picker needs) lives in Qt's private Core API.
    target_link_libraries(render360 PRIVATE Qt6::CorePrivate)
endif()

# --- Android packaging ------------------------------------------------------
if(ANDROID)
    set_property(TARGET render360 PROPERTY QT_ANDROID_PACKAGE_SOURCE_DIR
        "${CMAKE_CURRENT_SOURCE_DIR}/android")
    # compileSdk 36 is forced by Qt 6.11's own androidx.core 1.17 dependency,
    # which refuses to be consumed by anything older. min 28 is Qt 6.11's floor
    # (androiddeployqt rejects less) and covers Android 9 upward -- the Edge 40
    # ships 13.
    set_property(TARGET render360 PROPERTY QT_ANDROID_TARGET_SDK_VERSION 36)
    set_property(TARGET render360 PROPERTY QT_ANDROID_MIN_SDK_VERSION 28)
    set_property(TARGET render360 PROPERTY QT_ANDROID_VERSION_NAME "1.0.0")
    # Google Play refuses an upload whose versionCode it has already seen, so
    # this has to move for every store upload (a sideloaded APK only needs it
    # to not go backwards). Override with -DRENDER360_VERSION_CODE=N.
    set(RENDER360_VERSION_CODE "2" CACHE STRING "Android versionCode")
    set_property(TARGET render360 PROPERTY QT_ANDROID_VERSION_CODE ${RENDER360_VERSION_CODE})

    # To point a release APK at a clip for testing, configure with
    #   -DQT_ANDROID_APPLICATION_ARGUMENTS="-i /sdcard/.../clip.MP4 --imu"
    # (a plain Qt variable, read by Qt6AndroidMacros when it writes the
    # deployment settings). Qt refuses the `applicationArguments` intent extra
    # in a release build -- "Not in debug mode! It is not allowed to use extra
    # arguments" -- and the app has no Android file-open path yet, so this is
    # currently the only way in.
endif()

if(NOT ANDROID)
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
endif()

target_compile_definitions(render360 PRIVATE
    QT_DEPRECATED_WARNINGS
    QT_DISABLE_DEPRECATED_BEFORE=0x060000
)

# --- Linux/desktop naming and installation ---------------------------------
# The CMake target stays "render360" (the QML module URI, the Android library
# name androiddeployqt writes into the manifest, and every build command in the
# docs depend on it). Only the produced BINARY is renamed, and only off Android.
if(NOT ANDROID)
    set_target_properties(render360 PROPERTIES OUTPUT_NAME "reframe360-editor")

    include(GNUInstallDirs)
    install(TARGETS render360 RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR})
    install(FILES resources/reframe360-editor.desktop
            DESTINATION ${CMAKE_INSTALL_DATADIR}/applications)
    install(FILES resources/icon-512.png
            DESTINATION ${CMAKE_INSTALL_DATADIR}/icons/hicolor/512x512/apps
            RENAME reframe360-editor.png)
endif()
