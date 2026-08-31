# Reframe360 Editor

Takes a dual lens 360 camera video, allows user to pan/tilt/yaw, and uses
OpenGL shaders to render a 2d output video quickly and efficiently.

Runs on desktop Linux and on Android (see `android/README.md`).

## Building on Linux

```bash
sudo apt install cmake ninja-build pkg-config \
    qt6-base-dev qt6-base-dev-tools qt6-declarative-dev \
    qt6-declarative-dev-tools qt6-shadertools-dev libgl-dev libglx-dev \
    libavformat-dev libavcodec-dev libavutil-dev libswscale-dev libopencv-dev

cmake -S . -B build -G Ninja -DCMAKE_BUILD_TYPE=Release
cmake --build build -j"$(nproc)"
./build/reframe360-editor
```

`tracking_tests` (the non-GUI IMU/visual pipeline harness) is built by the same
command; see `TESTING.md`.

## Building for Android

Prerequisites are Qt for Android, the Android SDK/NDK and a JDK — installed by
hand once, as described in `android/README.md`. Then:

```bash
android/fetch-deps.sh          # FFmpeg + x264 sources, OpenCV Android SDK
android/build-x264-ffmpeg.sh   # cross-build both ABIs (~20 min, once)
android/release-android.sh     # arm64 + universal APKs, signed if a keystore
```

Every path in those scripts is an environment override with this machine's
layout as the default (`DEPS`, `QT_ROOT`, `OUT`, `KEYSTORE`, …), which is what
lets the CI reuse them unchanged.

## Continuous integration

`.github/workflows/build.yml` builds both targets on GitHub runners for every
push and pull request:

| job | what it does |
|---|---|
| `linux` | apt Qt 6.4 build + headless smoke test, uploads an install tarball |
| `android-deps` | cross-builds FFmpeg/x264 for arm64-v8a and x86_64, caches the tree |
| `android` | installs Qt for Android via aqtinstall, builds (and signs) the APK |
| `release` | on a `v*` tag, publishes the artifacts as a GitHub release |

The Android dependency tree is cached on a key made of the FFmpeg/OpenCV/x264/
NDK versions and the hashes of the two build scripts, so it is rebuilt only
when one of those actually changes. Signing details are in
`android/README.md`.
