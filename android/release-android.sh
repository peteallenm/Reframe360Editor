#!/usr/bin/env bash
# Build (and, if a keystore is available, SIGN) the release APKs:
#   reframe360-arm64.apk      arm64-v8a only (phones; the Edge 40)
#   reframe360-universal.apk  arm64-v8a + x86_64 (adds Chromebooks/emulators)
#
#   android/release-android.sh            # both APK variants (default)
#   android/release-android.sh arm64      # just the phone APK
#   android/release-android.sh universal
#   android/release-android.sh bundle     # .aab for Google Play (both ABIs)
#
# Every path is an environment override with this machine's layout as the
# default, so the same script drives the GitHub runner (.github/workflows/
# build.yml). Signing is skipped -- and the APK left with an "-unsigned"
# suffix -- when no keystore is present, which is what a fork or a PR build
# gets.
#
# Signing normally uses ~/.android/render360-release.keystore (alias
# render360). Keep that file safe: Android only accepts UPDATES signed by the
# same key, so a lost keystore means users must uninstall (losing their folder
# grant and settings) to move to a new build.
set -euo pipefail

VARIANT=${1:-both}

QT_VER=${QT_VER:-6.11.2}
QT_ROOT=${QT_ROOT:-$HOME/Qt}
QT_HOST=${QT_HOST:-$QT_ROOT/$QT_VER/gcc_64}
QT_ANDROID=${QT_ANDROID:-$QT_ROOT/$QT_VER/android_arm64_v8a}
DEPS=${DEPS:-$HOME/Build/360Render/android-deps}
SRC=$(cd "$(dirname "$0")/.." && pwd)
OUT=${OUT:-$HOME/Build/360Render}
ANDROID_PLATFORM=${ANDROID_PLATFORM:-android-28}
# Overridable so a test build against a different dependency tree does not
# invalidate the working build directory.
# Play refuses an upload whose versionCode it has seen; override per upload.
VERSION_CODE=${VERSION_CODE:-2}
BUILD_ARM64=${BUILD_ARM64:-$SRC/build-android}
BUILD_UNIVERSAL=${BUILD_UNIVERSAL:-$SRC/build-android-universal}

export JAVA_HOME=${JAVA_HOME:-/usr/lib/jvm/java-21-openjdk-amd64}
export PATH=$JAVA_HOME/bin:$PATH
export ANDROID_SDK_ROOT=${ANDROID_SDK_ROOT:-$HOME/Android/Sdk}
export ANDROID_HOME=$ANDROID_SDK_ROOT
export ANDROID_NDK_ROOT=${ANDROID_NDK_ROOT:-$ANDROID_SDK_ROOT/ndk/27.2.12479018}

# Qt's own CMake if it is there (the installer ships one), else whatever is on
# PATH -- aqtinstall, which CI uses, does not install Tools/CMake.
if [ -z "${CMAKE:-}" ]; then
    CMAKE=$QT_ROOT/Tools/CMake/bin/cmake
    [ -x "$CMAKE" ] || CMAKE=$(command -v cmake)
fi

KEYSTORE=${KEYSTORE:-$HOME/.android/render360-release.keystore}
KEYSTORE_PASS=${KEYSTORE_PASS:-render360}
KEY_ALIAS=${KEY_ALIAS:-render360}
APKSIGNER=${APKSIGNER:-$(ls -d "$ANDROID_SDK_ROOT"/build-tools/*/apksigner 2>/dev/null | sort -V | tail -1 || true)}

mkdir -p "$OUT"

for p in "$QT_HOST" "$ANDROID_NDK_ROOT" "$JAVA_HOME" \
         "$DEPS/OpenCV-android-sdk/sdk/native/jni"; do
    [ -e "$p" ] || { echo "missing prerequisite: $p" >&2; exit 1; }
done

publish() { # publish <unsigned.apk> <name-without-extension>
    local apk=$1 name=$2
    if [ -f "$KEYSTORE" ] && [ -n "$APKSIGNER" ]; then
        "$APKSIGNER" sign --ks "$KEYSTORE" --ks-pass "pass:$KEYSTORE_PASS" \
            --ks-key-alias "$KEY_ALIAS" --out "$OUT/$name.apk" "$apk"
        "$APKSIGNER" verify "$OUT/$name.apk"
        echo "signed: $OUT/$name.apk ($(du -h "$OUT/$name.apk" | cut -f1))"
    else
        cp "$apk" "$OUT/$name-unsigned.apk"
        echo "NO KEYSTORE at $KEYSTORE -- left unsigned: $OUT/$name-unsigned.apk"
    fi
}

build_arm64() {
    [ -e "$DEPS/ffmpeg-arm64/lib/libavcodec.a" ] || {
        echo "missing $DEPS/ffmpeg-arm64 (run android/build-x264-ffmpeg.sh)" >&2; exit 1; }
    "$QT_ANDROID/bin/qt-cmake" -S "$SRC" -B "$BUILD_ARM64" -G Ninja \
        -DCMAKE_BUILD_TYPE=Release \
        -DQT_HOST_PATH="$QT_HOST" \
        -DANDROID_ABI=arm64-v8a \
        -DANDROID_PLATFORM="$ANDROID_PLATFORM" \
        -DRENDER360_FFMPEG_DIR="$DEPS/ffmpeg-arm64" \
        -DOpenCV_DIR="$DEPS/OpenCV-android-sdk/sdk/native/jni" \
        -DRENDER360_VERSION_CODE="$VERSION_CODE"
    "$CMAKE" --build "$BUILD_ARM64" -j"$(nproc)"
    publish "$(find "$BUILD_ARM64/android-build/build/outputs/apk" -name '*.apk' | head -1)" \
            reframe360-arm64
}

# RENDER360_FFMPEG_DIR points at the parent tree holding one subdirectory per
# ABI ($DEPS/ffmpeg/{arm64-v8a,x86_64}); CMakeLists resolves the right one per
# sub-configure. MULTI_ABI_FORWARD_VARS forwards it verbatim -- CMakeLists
# must never write the resolved path back into it.
configure_universal() {
    [ -e "$DEPS/ffmpeg/x86_64/lib/libavcodec.a" ] || {
        echo "missing $DEPS/ffmpeg/x86_64 (run android/build-x264-ffmpeg.sh)" >&2; exit 1; }
    "$QT_ANDROID/bin/qt-cmake" -S "$SRC" -B "$BUILD_UNIVERSAL" -G Ninja \
        -DCMAKE_BUILD_TYPE=Release \
        -DQT_HOST_PATH="$QT_HOST" \
        -DANDROID_PLATFORM="$ANDROID_PLATFORM" \
        -DQT_ANDROID_ABIS="arm64-v8a;x86_64" \
        -DQT_ANDROID_MULTI_ABI_FORWARD_VARS="RENDER360_FFMPEG_DIR;OpenCV_DIR" \
        -DRENDER360_FFMPEG_DIR="$DEPS/ffmpeg" \
        -DOpenCV_DIR="$DEPS/OpenCV-android-sdk/sdk/native/jni" \
        -DRENDER360_VERSION_CODE="$VERSION_CODE"
}

build_universal() {
    configure_universal
    "$CMAKE" --build "$BUILD_UNIVERSAL" -j"$(nproc)"
    publish "$(find "$BUILD_UNIVERSAL/android-build/build/outputs/apk" -name '*.apk' | head -1)" \
            reframe360-universal
}

# Google Play takes an app bundle, not an APK, and splits it per device -- so
# the bundle carries both ABIs while each phone still downloads one. It shares
# the universal build tree (same objects, a different Gradle task).
#
# Bundles are signed with jarsigner: apksigner only handles APKs. This key is
# the *upload* key; whether Play then serves APKs signed with this same key or
# one Google generates is decided once, when the app is created in the Console.
build_bundle() {
    configure_universal
    "$CMAKE" --build "$BUILD_UNIVERSAL" --target aab -j"$(nproc)"
    local aab
    aab=$(find "$BUILD_UNIVERSAL/android-build/build/outputs/bundle" -name '*.aab' | head -1)
    [ -n "$aab" ] || { echo "no .aab produced" >&2; exit 1; }
    if [ ! -f "$KEYSTORE" ]; then
        cp "$aab" "$OUT/reframe360-release-unsigned.aab"
        echo "NO KEYSTORE at $KEYSTORE -- left unsigned: $OUT/reframe360-release-unsigned.aab"
        return
    fi
    "$JAVA_HOME/bin/jarsigner" -keystore "$KEYSTORE" \
        -storepass "$KEYSTORE_PASS" \
        -signedjar "$OUT/reframe360-release.aab" "$aab" "$KEY_ALIAS"
    "$JAVA_HOME/bin/jarsigner" -verify "$OUT/reframe360-release.aab" > /dev/null
    echo "signed: $OUT/reframe360-release.aab ($(du -h "$OUT/reframe360-release.aab" | cut -f1))"
    echo "versionCode $VERSION_CODE -- Play rejects an upload that does not increase it"
}

case "$VARIANT" in
    arm64)     build_arm64 ;;
    universal) build_universal ;;
    bundle)    build_bundle ;;
    both)      build_arm64; build_universal ;;
    *) echo "usage: $0 [arm64|universal|bundle|both]" >&2; exit 2 ;;
esac
