#!/usr/bin/env bash
# Build and SIGN the release APKs:
#   reframe360-arm64.apk      arm64-v8a only (phones; the Edge 40)
#   reframe360-universal.apk  arm64-v8a + x86_64 (adds Chromebooks/emulators)
#
# Signing uses ~/.android/render360-release.keystore (alias render360). Keep
# that file safe: Android only accepts UPDATES signed by the same key, so a
# lost keystore means users must uninstall (losing their folder grant and
# settings) to move to a new build.
set -euo pipefail

QT_VER=6.11.2
QT_HOST=$HOME/Qt/$QT_VER/gcc_64
DEPS=$HOME/Build/360Render/android-deps
SRC=$(cd "$(dirname "$0")/.." && pwd)
OUT=$HOME/Build/360Render

export JAVA_HOME=/usr/lib/jvm/java-21-openjdk-amd64
export PATH=$JAVA_HOME/bin:$PATH
export ANDROID_SDK_ROOT=$HOME/Android/Sdk
export ANDROID_NDK_ROOT=$ANDROID_SDK_ROOT/ndk/27.2.12479018
APKSIGNER=$ANDROID_SDK_ROOT/build-tools/36.0.0/apksigner
KS=$HOME/.android/render360-release.keystore

sign() { # sign <unsigned.apk> <out.apk>
    "$APKSIGNER" sign --ks "$KS" --ks-pass pass:render360 \
        --ks-key-alias render360 --out "$2" "$1"
    "$APKSIGNER" verify "$2"
    echo "signed: $2 ($(du -h "$2" | cut -f1))"
}

# ---- arm64-only ------------------------------------------------------------
"$HOME/Qt/$QT_VER/android_arm64_v8a/bin/qt-cmake" -S "$SRC" -B "$SRC/build-android" -G Ninja \
    -DCMAKE_BUILD_TYPE=Release \
    -DQT_HOST_PATH="$QT_HOST" \
    -DANDROID_ABI=arm64-v8a \
    -DANDROID_PLATFORM=android-28 \
    -DRENDER360_FFMPEG_DIR="$DEPS/ffmpeg-arm64" \
    -DOpenCV_DIR="$DEPS/OpenCV-android-sdk/sdk/native/jni"
"$HOME/Qt/Tools/CMake/bin/cmake" --build "$SRC/build-android" -j"$(nproc)"
sign "$(find "$SRC/build-android/android-build/build/outputs/apk" -name '*.apk' | head -1)" \
     "$OUT/reframe360-arm64.apk"

# ---- universal (arm64-v8a + x86_64) ----------------------------------------
# RENDER360_FFMPEG_DIR points at the parent tree holding one subdirectory per
# ABI ($DEPS/ffmpeg/{arm64-v8a,x86_64}); CMakeLists resolves the right one per
# sub-configure. MULTI_ABI_FORWARD_VARS forwards it verbatim -- CMakeLists
# must never write the resolved path back into it.
"$HOME/Qt/$QT_VER/android_arm64_v8a/bin/qt-cmake" -S "$SRC" -B "$SRC/build-android-universal" -G Ninja \
    -DCMAKE_BUILD_TYPE=Release \
    -DQT_HOST_PATH="$QT_HOST" \
    -DANDROID_PLATFORM=android-28 \
    -DQT_ANDROID_ABIS="arm64-v8a;x86_64" \
    -DQT_ANDROID_MULTI_ABI_FORWARD_VARS="RENDER360_FFMPEG_DIR;OpenCV_DIR" \
    -DRENDER360_FFMPEG_DIR="$DEPS/ffmpeg" \
    -DOpenCV_DIR="$DEPS/OpenCV-android-sdk/sdk/native/jni"
"$HOME/Qt/Tools/CMake/bin/cmake" --build "$SRC/build-android-universal" -j"$(nproc)"
sign "$(find "$SRC/build-android-universal/android-build/build/outputs/apk" -name '*.apk' | head -1)" \
     "$OUT/reframe360-universal.apk"
