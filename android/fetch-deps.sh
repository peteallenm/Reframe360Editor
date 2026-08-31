#!/usr/bin/env bash
# Fetch the third-party sources the Android build needs into $DEPS.
#
# Everything here is downloaded, never vendored into the repo: FFmpeg and x264
# are cross-built by build-x264-ffmpeg.sh, OpenCV comes as a prebuilt Android
# SDK. Safe to re-run -- each piece is skipped if it is already present.
#
#   DEPS=~/Build/360Render/android-deps android/fetch-deps.sh
#   android/fetch-deps.sh --prune     # drop sources/other ABIs after building
#
# --prune is for CI, where only the built artefacts are worth caching. Do NOT
# run it on a dev machine: it deletes the FFmpeg source trees, so the next
# build-x264-ffmpeg.sh has to download them again.
set -euo pipefail

DEPS=${DEPS:-$HOME/Build/360Render/android-deps}
FFMPEG_VERSION=${FFMPEG_VERSION:-6.1.1}
OPENCV_VERSION=${OPENCV_VERSION:-4.10.0}
# Pinned so a cached dependency tree and a fresh one are the same thing.
X264_COMMIT=${X264_COMMIT:-0480cb05fa188d37ae87e8f4fd8f1aea3711f7ee}

mkdir -p "$DEPS"
cd "$DEPS"

if [ "${1:-}" = "--prune" ]; then
    # Keep: ffmpeg-arm64, ffmpeg-x86_64 (headers + static libs), the symlink
    # tree, and OpenCV's native jni/static libs for the two ABIs we ship.
    rm -rf "ffmpeg-$FFMPEG_VERSION" ffmpeg-x86_64-src x264-src \
           "ffmpeg-$FFMPEG_VERSION.tar.xz" opencv-android-sdk.zip
    rm -rf OpenCV-android-sdk/samples OpenCV-android-sdk/sdk/java \
           OpenCV-android-sdk/sdk/etc
    for abi in armeabi-v7a x86; do
        rm -rf "OpenCV-android-sdk/sdk/native/libs/$abi" \
               "OpenCV-android-sdk/sdk/native/staticlibs/$abi" \
               "OpenCV-android-sdk/sdk/native/3rdparty/libs/$abi"
    done
    du -sh "$DEPS"
    exit 0
fi

fetch() { # fetch <url> <output>
    echo "downloading $1"
    curl -fL --retry 3 --retry-delay 5 -o "$2.part" "$1"
    mv "$2.part" "$2"
}

# --- FFmpeg -----------------------------------------------------------------
# Two source trees: FFmpeg configures in-tree, and the two ABIs are configured
# with different toolchains, so they cannot share one.
if [ ! -f "ffmpeg-$FFMPEG_VERSION.tar.xz" ]; then
    fetch "https://ffmpeg.org/releases/ffmpeg-$FFMPEG_VERSION.tar.xz" \
          "ffmpeg-$FFMPEG_VERSION.tar.xz"
fi
[ -d "ffmpeg-$FFMPEG_VERSION" ] || tar xf "ffmpeg-$FFMPEG_VERSION.tar.xz"
if [ ! -d ffmpeg-x86_64-src ]; then
    tmp=$(mktemp -d "$DEPS/.ffx.XXXXXX")
    tar xf "ffmpeg-$FFMPEG_VERSION.tar.xz" -C "$tmp"
    mv "$tmp/ffmpeg-$FFMPEG_VERSION" ffmpeg-x86_64-src
    rmdir "$tmp"
fi

# --- x264 -------------------------------------------------------------------
if [ ! -d x264-src ]; then
    git clone https://code.videolan.org/videolan/x264.git x264-src \
        || git clone https://github.com/mirror/x264.git x264-src
fi
# A full clone already has the pin; only reach out if it does not.
git -C x264-src checkout --detach "$X264_COMMIT" 2>/dev/null || {
    git -C x264-src fetch origin
    git -C x264-src checkout --detach "$X264_COMMIT"
}

# --- OpenCV Android SDK (prebuilt) -----------------------------------------
if [ ! -d OpenCV-android-sdk ]; then
    if [ ! -f opencv-android-sdk.zip ]; then
        fetch "https://github.com/opencv/opencv/releases/download/$OPENCV_VERSION/opencv-$OPENCV_VERSION-android-sdk.zip" \
              opencv-android-sdk.zip
    fi
    unzip -q opencv-android-sdk.zip
fi
[ -d OpenCV-android-sdk/sdk/native/jni ] || {
    echo "OpenCV SDK unpacked but sdk/native/jni is missing" >&2; exit 1; }

echo "deps ready in $DEPS"
