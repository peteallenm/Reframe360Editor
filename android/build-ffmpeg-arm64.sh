#!/usr/bin/env bash
# Cross-build FFmpeg 6.1.1 for Android arm64-v8a with MediaCodec decode AND
# encode. Verified 2026-08-30. Source: https://ffmpeg.org/releases/ffmpeg-6.1.1.tar.xz
# extracted to ~/Build/360Render/android-deps/ffmpeg-6.1.1.
# Software h264 decode is kept deliberately: the camera's 2880x5760 stream
# exceeds every hardware decoder (MediaCodec caps at 4096x2304).
set -e
cd ~/Build/360Render/android-deps/ffmpeg-6.1.1
NDK=$HOME/Android/Sdk/ndk/27.2.12479018
TOOLCHAIN=$NDK/toolchains/llvm/prebuilt/linux-x86_64
API=28
PREFIX=$HOME/Build/360Render/android-deps/ffmpeg-arm64
make distclean >/dev/null 2>&1 || true
./configure   --prefix=$PREFIX   --target-os=android --arch=aarch64 --cpu=armv8-a --enable-cross-compile   --cc=$TOOLCHAIN/bin/aarch64-linux-android$API-clang   --cxx=$TOOLCHAIN/bin/aarch64-linux-android$API-clang++   --ar=$TOOLCHAIN/bin/llvm-ar --nm=$TOOLCHAIN/bin/llvm-nm   --ranlib=$TOOLCHAIN/bin/llvm-ranlib --strip=$TOOLCHAIN/bin/llvm-strip   --sysroot=$TOOLCHAIN/sysroot   --enable-static --disable-shared --enable-pic   --disable-programs --disable-doc --disable-avdevice --disable-avfilter --disable-postproc   --disable-network --disable-symver   --enable-jni --enable-mediacodec   --disable-everything   --enable-demuxer=mov,matroska,h264,hevc   --enable-muxer=mp4,mov   --enable-parser=h264,hevc,mpeg4video   --enable-decoder=h264,hevc,mpeg4,h264_mediacodec,hevc_mediacodec   --enable-encoder=h264_mediacodec,hevc_mediacodec,mpeg4   --enable-bsf=h264_mp4toannexb,hevc_mp4toannexb,extract_extradata   --enable-protocol=file,pipe,fd   --enable-swscale --enable-avformat --enable-avcodec
