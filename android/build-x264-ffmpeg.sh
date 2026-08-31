#!/usr/bin/env bash
# Cross-build x264 (static) for both Android ABIs and rebuild FFmpeg 6.1.1
# with --enable-gpl --enable-libx264 so the phone has a SOFTWARE H.264
# encoder. Needed for 5.7K (5760x2880) exports: MediaCodec encoders cap at
# ~4096x2304, and until this the Android build had no other H.264 encoder.
set -e
# Paths are overridable so the same script runs on a dev machine and on a
# GitHub runner (see .github/workflows/build.yml). Defaults are this machine.
NDK=${ANDROID_NDK_ROOT:-$HOME/Android/Sdk/ndk/27.2.12479018}
TC=$NDK/toolchains/llvm/prebuilt/linux-x86_64
API=${ANDROID_API:-28}
DEPS=${DEPS:-$HOME/Build/360Render/android-deps}
X264=$DEPS/x264-src
FFMPEG_VERSION=${FFMPEG_VERSION:-6.1.1}

for p in "$TC/bin/aarch64-linux-android$API-clang" "$X264/configure" \
         "$DEPS/ffmpeg-$FFMPEG_VERSION/configure" "$DEPS/ffmpeg-x86_64-src/configure"; do
    [ -e "$p" ] || { echo "missing prerequisite: $p (run android/fetch-deps.sh)" >&2; exit 1; }
done

build_x264() { # <triple> <prefix> <extra configure args...>
    local triple=$1 prefix=$2; shift 2
    cd "$X264"
    make clean >/dev/null 2>&1 || true
    CC="$TC/bin/${triple}${API}-clang" ./configure \
        --prefix="$prefix" --host="$triple" \
        --cross-prefix="$TC/bin/llvm-" \
        --enable-static --enable-pic --disable-cli "$@"
    make -j"$(nproc)" >/dev/null
    make install >/dev/null
    echo "x264 for $triple -> $prefix"
}

build_x264 aarch64-linux-android "$DEPS/ffmpeg-arm64"
build_x264 x86_64-linux-android  "$DEPS/ffmpeg-x86_64" --disable-asm

ffmpeg_common="--enable-static --disable-shared --enable-pic
 --disable-programs --disable-doc --disable-avdevice --disable-avfilter --disable-postproc
 --disable-network --disable-symver --enable-jni --enable-mediacodec
 --disable-everything
 --enable-demuxer=mov,matroska,h264,hevc
 --enable-muxer=mp4,mov
 --enable-parser=h264,hevc,mpeg4video
 --enable-decoder=h264,hevc,mpeg4,h264_mediacodec,hevc_mediacodec
 --enable-encoder=h264_mediacodec,hevc_mediacodec,mpeg4,libx264
 --enable-bsf=h264_mp4toannexb,hevc_mp4toannexb,extract_extradata,h264_metadata,hevc_metadata
 --enable-protocol=file,pipe,fd
 --enable-swscale --enable-avformat --enable-avcodec
 --enable-gpl --enable-libx264
 --extra-libs=-lm"   # bionic keeps exp2/log/log10 in libm; x264 needs them,
                     # and without it configure reports the misleading
                     # "x264 not found using pkg-config"

# ---- arm64 ----
cd $DEPS/ffmpeg-$FFMPEG_VERSION
PREFIX=$DEPS/ffmpeg-arm64
# PKG_CONFIG_LIBDIR *replaces* pkg-config's search path, so the only x264 it
# can see is the one just cross-built. Without it a machine that happens to
# have a host libx264-dev installed satisfies the probe with the DESKTOP
# package (and a machine that does not -- every CI runner -- fails with the
# misleading "x264 not found using pkg-config").
export PKG_CONFIG_LIBDIR="$PREFIX/lib/pkgconfig"
unset PKG_CONFIG_PATH
make distclean >/dev/null 2>&1 || true
./configure --prefix=$PREFIX --target-os=android --arch=aarch64 --cpu=armv8-a \
  --enable-cross-compile \
  --cc=$TC/bin/aarch64-linux-android$API-clang --cxx=$TC/bin/aarch64-linux-android$API-clang++ \
  --ar=$TC/bin/llvm-ar --nm=$TC/bin/llvm-nm --ranlib=$TC/bin/llvm-ranlib --strip=$TC/bin/llvm-strip \
  --sysroot=$TC/sysroot \
  --extra-cflags="-I$PREFIX/include" --extra-ldflags="-L$PREFIX/lib" \
  $ffmpeg_common
make -j"$(nproc)" >/dev/null && make install >/dev/null
echo "ffmpeg arm64 done"

# ---- x86_64 ----
cd $DEPS/ffmpeg-x86_64-src
PREFIX=$DEPS/ffmpeg-x86_64
export PKG_CONFIG_LIBDIR="$PREFIX/lib/pkgconfig"
make distclean >/dev/null 2>&1 || true
./configure --prefix=$PREFIX --target-os=android --arch=x86_64 \
  --enable-cross-compile \
  --cc=$TC/bin/x86_64-linux-android$API-clang --cxx=$TC/bin/x86_64-linux-android$API-clang++ \
  --ar=$TC/bin/llvm-ar --nm=$TC/bin/llvm-nm --ranlib=$TC/bin/llvm-ranlib --strip=$TC/bin/llvm-strip \
  --sysroot=$TC/sysroot \
  --disable-x86asm --disable-inline-asm \
  --extra-cflags="-I$PREFIX/include" --extra-ldflags="-L$PREFIX/lib" \
  $ffmpeg_common
make -j"$(nproc)" >/dev/null && make install >/dev/null
echo "ffmpeg x86_64 done"

# The multi-ABI (universal) build is given one parent directory and resolves
# $DIR/$ANDROID_ABI itself, so lay that view out here. Relative links, so the
# whole tree stays movable (a CI cache restores it under a different $HOME).
mkdir -p "$DEPS/ffmpeg"
ln -sfn ../ffmpeg-arm64  "$DEPS/ffmpeg/arm64-v8a"
ln -sfn ../ffmpeg-x86_64 "$DEPS/ffmpeg/x86_64"

echo ALL-DONE
