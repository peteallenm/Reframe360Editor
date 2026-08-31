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

## Licence

Reframe360 Editor is free software under the **GNU General Public License,
version 3 or later** — the full text is in [LICENSE](LICENSE), and every
source file carries an `SPDX-License-Identifier: GPL-3.0-or-later` header.

GPLv3 is not a preference here, it is the only licence this dependency set
permits:

| component | licence | what it forces |
|---|---|---|
| x264 (Android, for the 5.7K software encode) | GPL-2.0-or-later | the combined work must be GPL |
| FFmpeg, built `--enable-gpl` | GPL-2.0-or-later | same |
| Qt 6, open-source build | LGPL-3.0 | combines with GPLv3, **not** with GPLv2-only |
| OpenCV 4 | Apache-2.0 | combines with GPLv3, **not** with GPLv2 |

x264 and FFmpeg rule out anything permissive; Qt's LGPLv3 and OpenCV's
Apache-2.0 rule out GPLv2. "Version 3 or later" is what makes the combination
lawful.

### What shipping a binary obliges

* **Offer the source.** Anyone given a build must be able to obtain the
  corresponding source. This repository being public satisfies that; a release
  or store listing should link to it, and the in-app About box does.
* **Keep Qt dynamically linked.** Qt is LGPL and the APK ships it as separate
  `.so` files, which is what lets a user relink against their own Qt. Statically
  linking Qt would break that; FFmpeg, x264 and OpenCV are static and that is
  fine, because they are GPL/permissive rather than LGPL.
* **Say so in the app.** The About box (toolbar → About) states the licence,
  disclaims warranty, points at the source and credits Qt, FFmpeg, x264 and
  OpenCV — the notice the GPL asks an interactive program to display, and the
  attribution Qt's LGPL asks for.

The desktop build links the distribution's own FFmpeg, which may be a non-GPL
configuration; that changes nothing about this program's licence.

Copyright is currently attributed to Peter Allen. If this is work owned by
Overview, change the copyright line in the headers and in the About box.
