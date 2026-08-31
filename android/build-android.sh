#!/usr/bin/env bash
# Reframe360 Editor -- 360 video stabiliser and stitcher for dual-fisheye footage.
# Copyright (C) 2026 Peter Allen
# SPDX-License-Identifier: GPL-3.0-or-later
#
# This program is free software under the GNU General Public License, version
# 3 or (at your option) any later version; see LICENSE for the full text.
# It is distributed in the hope that it will be useful, but WITHOUT ANY
# WARRANTY, without even the implied warranty of MERCHANTABILITY or FITNESS
# FOR A PARTICULAR PURPOSE.

# Build Reframe360 Editor for Android arm64-v8a.
#
# Verified end-to-end on 2026-08-30: produces a 72 MB APK containing
# librender360_arm64-v8a.so (10.5 MB, static FFmpeg + OpenCV), the Qt 6.11.2
# runtime and the QML bundle. See android/README.md for how the prerequisites
# were installed, and for the gotchas each of these settings works around.
set -euo pipefail

QT_VER=6.11.2
QT_ANDROID=$HOME/Qt/$QT_VER/android_arm64_v8a
QT_HOST=$HOME/Qt/$QT_VER/gcc_64
DEPS=$HOME/Build/360Render/android-deps
BUILD=${1:-$(cd "$(dirname "$0")/.." && pwd)/build-android}

# JDK 21, not 17: the java-17-openjdk-amd64 package on this machine is a JRE
# with no javac, and Gradle fails with "does not provide the required
# capabilities: [JAVA_COMPILER]".
export JAVA_HOME=/usr/lib/jvm/java-21-openjdk-amd64
export PATH=$JAVA_HOME/bin:$PATH
export ANDROID_SDK_ROOT=$HOME/Android/Sdk
# NDK r27c: the version Qt 6.11.2 itself was built against (its
# qt.toolchain.cmake records /opt/android/r27c).
export ANDROID_NDK_ROOT=$ANDROID_SDK_ROOT/ndk/27.2.12479018

for p in "$QT_ANDROID" "$QT_HOST" "$ANDROID_NDK_ROOT" "$JAVA_HOME" \
         "$DEPS/ffmpeg-arm64/lib/libavcodec.a" \
         "$DEPS/OpenCV-android-sdk/sdk/native/jni"; do
    [ -e "$p" ] || { echo "missing prerequisite: $p" >&2; exit 1; }
done

"$QT_ANDROID/bin/qt-cmake" -S "$(dirname "$0")/.." -B "$BUILD" -G Ninja \
    -DCMAKE_BUILD_TYPE=Release \
    -DQT_HOST_PATH="$QT_HOST" \
    -DANDROID_ABI=arm64-v8a \
    -DANDROID_PLATFORM=android-28 \
    -DRENDER360_FFMPEG_DIR="$DEPS/ffmpeg-arm64" \
    -DOpenCV_DIR="$DEPS/OpenCV-android-sdk/sdk/native/jni"

"$HOME/Qt/Tools/CMake/bin/cmake" --build "$BUILD" -j"$(nproc)"

APK=$(find "$BUILD" -name '*.apk' | head -1)
echo
echo "APK: $APK  ($(du -h "$APK" | cut -f1))"
echo "install with:  \$ANDROID_SDK_ROOT/platform-tools/adb install -r $APK"
