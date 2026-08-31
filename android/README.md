# Render360 on Android — build setup


---

## VERIFIED BUILD (2026-08-30)

The APK builds. Run `android/build-android.sh` (prerequisites below are already
installed on this machine); it produces a **72 MB APK**, arm64-v8a only, with
`librender360_arm64-v8a.so` at 10.5 MB carrying static FFmpeg and OpenCV.
Confirmed present in the binary: 3125 OpenCV symbols incl. ORB and BFMatcher,
FFmpeg's `ff_h264_mediacodec_encoder`/`decoder`, and `libmediandk/libandroid/
libEGL/libGLESv2` in DT_NEEDED. `aapt dump badging`: package
`uk.co.overview.render360`, minSdk 28, targetSdk 36, compileSdk 36.

**Exact toolchain that works** (several of these differ from the guesses in the
sections below, which were written before anything was installed):

| piece | value | why this one |
|---|---|---|
| Qt | **6.11.2**, `~/Qt/6.11.2/{android_arm64_v8a,gcc_64}` | host and target must match exactly |
| NDK | **27.2.12479018 (r27c)** | Qt 6.11.2's own `qt.toolchain.cmake` records `/opt/android/r27c` — NOT r26b |
| JDK | **21** (`/usr/lib/jvm/java-21-openjdk-amd64`) | `java-17-openjdk-amd64` here is a **JRE with no javac**; Gradle fails with "does not provide the required capabilities: [JAVA_COMPILER]" |
| SDK platform | **android-36** + build-tools 36.0.0 | Qt 6.11 pulls `androidx.core:1.17.0`, which refuses to be consumed below compileSdk 36. android-34 alone fails |
| targetSdk / minSdk | 36 / 28 | 28 is Qt 6.11's floor |
| FFmpeg | 6.1.1, static, `--enable-mediacodec --enable-jni` | see `build-ffmpeg-arm64.sh`; 6.1.1 has MediaCodec **encoders** as well as decoders |
| OpenCV | 4.10.0 Android SDK, `sdk/native/jni` | core/imgproc/features2d/calib3d; no `videoio` needed |

**Source changes the port required** (all landed, desktop build unaffected):

* `src/glesext.h` — Qt for Android is built to the GLES 2.0 baseline
  (`QT_FEATURE_opengles3 == -1`), so `<QOpenGLFunctions>` pulls `GLES2/gl2.h`
  and every GLES 3 *enum* (`GL_R8`, `GL_RGBA16F`, `GL_HALF_FLOAT`,
  `GL_UNPACK_ROW_LENGTH`, `GL_COLOR_ATTACHMENT1`, ...) is undeclared. Only the
  constants are missing — the entry points come from `QOpenGLExtraFunctions` at
  runtime — so this header just pulls in the NDK's `GLES3/gl3.h` on Android.

**AndroidManifest.xml gotchas**, each of which broke the build once:

1. **No `<uses-sdk>` element.** androiddeployqt injects it from the CMake
   properties; leaving a `%%INSERT_MIN_SDK%%` placeholder there makes it parse
   the token as a number and fail with "minSdkVersion must be >= 28".
2. **Qt 6.11 wraps placeholders as `-- %%INSERT_X%% --`**, not `%%INSERT_X%%`.
   Compare `$QT_ANDROID/src/android/templates/AndroidManifest.xml`.
3. **XML comments may not contain `--`.** So a commented-out block must not
   carry a wrapped placeholder — which is exactly what (2) creates.
4. **Never set `android:extractNativeLibs`.** AGP 8 fails `:packageRelease`
   outright: "Avoid setting android:extractNativeLibs=true explicitly".

**Not yet verified:** nothing has run on a device — no phone was attached when
this was built. Install with
`$ANDROID_SDK_ROOT/platform-tools/adb install -r <apk>` and expect the storage
layer, MediaCodec selection and touch UI still to be missing (see Known gaps).

**Note:** Qt injects `INTERNET`, `ACCESS_NETWORK_STATE` and
`WRITE_EXTERNAL_STORAGE` permissions of its own into the merged manifest. The
last is unwanted for a SAF-based app and should be stripped with
`tools:node="remove"` before any release.

---

Target device: **Motorola Edge 40** (MediaTek Dimensity 8020, Mali-G77 MC9,
8 GB RAM, Android 13/14, OpenGL ES 3.2, Vulkan 1.1, USB-C 2.0 with OTG host).
Target ABI: **arm64-v8a only** — nothing else is worth building.

Everything below was written against the actual state of this machine
(Ubuntu 24.04.4, x86_64) on 2026-08-29. Read step 0 first: most of the
toolchain is *not* here yet.

---

## 0. What this machine has right now

Verified, not assumed:

| Thing | State |
|---|---|
| OS | Ubuntu 24.04.4 LTS, x86_64 |
| `adb` | **present**, `/usr/bin/adb` → `/usr/lib/android-sdk/platform-tools/adb`, v1.0.41 (34.0.4-debian) |
| `gradle` | **present but useless**, `/usr/bin/gradle` is **Gradle 4.4.1** (2017-era) |
| Android SDK | `/usr/lib/android-sdk` exists but contains **only `platform-tools/`** — no `platforms/`, no `build-tools/`, no `cmdline-tools/`, no `ndk/` |
| `sdkmanager` | **not installed**, not on `PATH` |
| NDK | **not installed** anywhere (`/usr/lib/android-ndk`, `/opt/android*`, `~/Android` all absent) |
| Java | OpenJDK **21.0.11** is the default and is a full JDK (`openjdk-21-jdk`). The `java-17-openjdk-amd64` tree is **JRE-only** — no `javac` — so it cannot be used for the Gradle step |
| `JAVA_HOME`, `ANDROID_HOME`, `ANDROID_SDK_ROOT`, `ANDROID_NDK_ROOT` | all **unset** |
| Qt | distro Qt **6.4.2** (`/usr`), **desktop x86_64 only** — no Android kit, no `~/Qt`, no `qt-cmake` |
| OpenCV | 4.6.0 desktop (`/usr/include/opencv4`) — x86_64, not usable for Android |
| FFmpeg | 6.1.1 desktop shared libs — x86_64, not usable for Android |
| CMake / Ninja | 3.28.3 / 1.11.1 — both fine as-is |
| `adb devices` | nothing attached |

Two consequences worth internalising:

* **Ignore the distro `gradle`.** Android Gradle Plugin 8.x needs Gradle 8.x.
  You never invoke it by hand anyway: `androiddeployqt` generates a Gradle
  *wrapper* in the build tree and that wrapper downloads the correct Gradle.
  Do not put `/usr/bin/gradle` in front of it.
* **Do not install SDK packages into `/usr/lib/android-sdk`.** It is root-owned
  and half-populated by the Debian packaging. Build a clean, user-owned SDK at
  `~/Android/Sdk` instead, and let it have its own `platform-tools`.

Pick a scratch root once and reuse it throughout:

```bash
export SDK=$HOME/Android/Sdk          # new, user-owned Android SDK
export DEPS=$HOME/Build/360Render/android-deps   # FFmpeg + OpenCV prebuilts
mkdir -p "$SDK" "$DEPS"
```

---

## 1. Qt for Android — must be installed by hand

**This step cannot be scripted.** The Qt Online Installer (and `qt-unified-*`)
requires an **interactive login to a Qt account** and accepts the open-source
licence through a GUI/TUI flow. There is no unattended path that does not
involve stuffing credentials into `--email`/`--pw` on a command line, so do it
by hand, once.

The distro Qt 6.4.2 is desktop-only and cannot be used for Android. It also
cannot act as the host Qt for a newer Android Qt: **`androiddeployqt` requires
the host Qt and the Android Qt to be the exact same version.**

1. Download the installer from <https://www.qt.io/download-qt-installer-oss>
   (`qt-online-installer-linux-x64-*.run`), `chmod +x`, run it.
2. Sign in / create a Qt account, choose the **open-source** licence.
3. Choose **Custom installation**, and under a Qt **6.7 or newer** release tick:
   * **Android** (this is the `android_arm64_v8a` + `android_armv7` + x86 kits)
   * **Desktop gcc 64-bit** — *the same version number*. This is the host Qt
     that provides `qmlimportscanner`, `rcc`, `moc` and `androiddeployqt`.
   * **Qt Shader Tools** (the build calls `qt_add_shaders()` — without it CMake
     fails at `find_package(Qt6 ... ShaderTools)`).
   * **Qt Quick Controls 2**, **Qt Quick** and **Qt Declarative** are pulled in
     by the base Qt component; verify they are ticked.
   * Under *Developer and Designer Tools*: **Qt Creator** is optional, but the
     **OpenSSL** and **Android Openssl** entries are not needed by this app.
4. Install to the default `~/Qt`.

> **You choose the version.** The rest of this document writes `6.11.2` as a
> concrete example. Substitute whatever you installed — but keep the two paths
> in lockstep:
>
> ```bash
> export QT_ANDROID=$HOME/Qt/6.11.2/android_arm64_v8a
> export QT_HOST=$HOME/Qt/6.11.2/gcc_64
> ls "$QT_ANDROID/bin/qt-cmake" "$QT_HOST/bin/androiddeployqt"   # both must exist
> ```

Qt 6.7 is the minimum here because this manifest uses the modern
`org.qtproject.qt.android.bindings.*` class names and the `%%INSERT_*%%`
placeholder set that CMake's `QT_ANDROID_*` target properties feed.

---

## 2. Android command-line tools, platform, build-tools and NDK

### 2.1 Command-line tools

Download the Linux **commandline-tools** zip from
<https://developer.android.com/studio#command-line-tools-only>. The URL always
has the form `https://dl.google.com/android/repository/commandlinetools-linux-<buildnumber>_latest.zip`.
A known-good one:

```bash
cd /tmp
wget https://dl.google.com/android/repository/commandlinetools-linux-11076708_latest.zip
```

> Check the download page for the current build number and use that instead if
> it is newer — the `11076708` build works, but Google rotates it.

It **must** be unzipped into `cmdline-tools/latest/` or `sdkmanager` refuses to
run (it locates the SDK root by walking up exactly two directories):

```bash
mkdir -p "$SDK/cmdline-tools"
unzip -q /tmp/commandlinetools-linux-11076708_latest.zip -d "$SDK/cmdline-tools"
mv "$SDK/cmdline-tools/cmdline-tools" "$SDK/cmdline-tools/latest"
export PATH="$SDK/cmdline-tools/latest/bin:$PATH"
```

### 2.2 Pin the JDK

`sdkmanager`, the Gradle wrapper and AGP 8.x are all validated against
**JDK 21.** The guess that AGP would need JDK 17 was wrong on two counts: the
JDK 17 package installed here is a JRE with no `javac` (Gradle fails with
"Toolchain installation ... does not provide the required capabilities:
[JAVA_COMPILER]"), and the AGP that Qt 6.11.2 generates is happy on 21. Pin it
explicitly so a future default change does not silently switch compilers:

```bash
export JAVA_HOME=/usr/lib/jvm/java-21-openjdk-amd64
export PATH="$JAVA_HOME/bin:$PATH"
java -version   # must now say 21.x
```

Do this in the same shell as every `sdkmanager`, `cmake` and `androiddeployqt`
invocation below. (Newer AGP does tolerate 21; pinning 17 removes the variable.)

### 2.3 Install the packages

```bash
export ANDROID_SDK_ROOT="$SDK"
export ANDROID_HOME="$SDK"          # some Qt/Gradle paths still read this

sdkmanager --sdk_root="$SDK" --licenses          # accept all, interactive y/n
sdkmanager --sdk_root="$SDK" \
    "platform-tools" \
    "platforms;android-34" \
    "build-tools;34.0.0" \
    "ndk;27.2.12479018"

export ANDROID_NDK_ROOT="$SDK/ndk/27.2.12479018"
```

* `platforms;android-34` — matches `QT_ANDROID_TARGET_SDK_VERSION 34`.
* `build-tools;34.0.0` — aapt2/zipalign/apksigner for that platform.
* `ndk;27.2.12479018` — NDK **r27c**, the version Qt 6.11.2 is built and
  tested against. A mismatched NDK is the single most common cause of
  `androiddeployqt` link failures; do not "upgrade" it casually.
* `platform-tools` — a user-owned `adb` that matches this SDK. The Debian
  `adb` at `/usr/bin/adb` also works; just be consistent about which one is
  first on `PATH`.

---

## 3. FFmpeg for arm64-v8a

`CMakeLists.txt` refuses to configure an Android build without
`-DRENDER360_FFMPEG_DIR`, and expects that directory to contain `include/` and
`lib/` with **static** `libavformat.a`, `libavcodec.a`, `libswscale.a`,
`libavutil.a` (it links `mediandk android log z` alongside them).

Build **FFmpeg 6.1.1** — the same series as the desktop build this code is
already compiled against, so no API drift:

```bash
cd "$DEPS"
git clone --depth 1 --branch n6.1.1 https://git.ffmpeg.org/ffmpeg.git ffmpeg-src
cd ffmpeg-src
```

Set up the NDK toolchain. The NDK's unified toolchain provides per-API-level
clang wrappers, so the API level goes in the compiler name, not in a flag:

```bash
export NDK="$SDK/ndk/27.2.12479018"
export TOOLCHAIN="$NDK/toolchains/llvm/prebuilt/linux-x86_64"
export API=28                                   # matches QT_ANDROID_MIN_SDK_VERSION
export PREFIX="$DEPS/ffmpeg/arm64-v8a"
```

Full configure invocation:

```bash
./configure \
  --prefix="$PREFIX" \
  --target-os=android \
  --arch=aarch64 \
  --cpu=armv8-a \
  --enable-cross-compile \
  --sysroot="$TOOLCHAIN/sysroot" \
  --cross-prefix="$TOOLCHAIN/bin/llvm-" \
  --cc="$TOOLCHAIN/bin/aarch64-linux-android$API-clang" \
  --cxx="$TOOLCHAIN/bin/aarch64-linux-android$API-clang++" \
  --ar="$TOOLCHAIN/bin/llvm-ar" \
  --nm="$TOOLCHAIN/bin/llvm-nm" \
  --ranlib="$TOOLCHAIN/bin/llvm-ranlib" \
  --strip="$TOOLCHAIN/bin/llvm-strip" \
  --extra-cflags="-O3 -fPIC -DANDROID" \
  --extra-ldflags="-fPIC" \
  --enable-pic \
  --enable-static \
  --disable-shared \
  --disable-programs \
  --disable-doc \
  --disable-avdevice \
  --disable-symver \
  --enable-mediacodec \
  --enable-jni \
  --enable-decoder=h264_mediacodec,hevc_mediacodec \
  --enable-encoder=h264_mediacodec,hevc_mediacodec \
  --enable-hwaccel=h264_mediacodec,hevc_mediacodec

make -j"$(nproc)"
make install
ls "$PREFIX/lib"/libav*.a "$PREFIX/lib/libswscale.a"
```

Why each of the load-bearing flags:

* `--enable-mediacodec --enable-jni` — MediaCodec is reached through JNI, so
  `--enable-jni` is not optional; without it the `*_mediacodec` codecs are
  silently unavailable. This is also why the manifest keeps the app in one
  process with a JavaVM alive.
* `--enable-decoder=h264_mediacodec,hevc_mediacodec` and the matching
  `--enable-encoder=` — additive. **We deliberately do NOT pass
  `--disable-decoders` / `--disable-everything`**, so the *software* h264 and
  hevc decoders stay in the build. That is not laziness: the YI 360 records
  **2880x5760**, which exceeds every hardware decoder we have measured (the
  desktop VAAPI path rejects it outright with "Hardware does not support image
  size 2880x5760", and mobile decoders cap far lower). Full-resolution decode
  on Android has to be software; MediaCodec is only useful for the `_thm.MP4`
  720x1440 proxy and for the *encode* side of export.
* `--enable-static --disable-shared` — CMake links `.a` files by absolute path.
  Static also avoids shipping five extra `.so`s that androiddeployqt would have
  to be told about.
* `--disable-programs --disable-doc` — we need libraries, not the `ffmpeg` CLI
  (which cannot be shipped or exec'd on Android anyway; see "Known gaps").
* `--disable-avdevice` — CMake links only avformat/avcodec/swscale/avutil.
* `--cross-prefix` + explicit `--ar/--nm/--ranlib/--strip` — the NDK ships
  `llvm-ar` etc. rather than `aarch64-linux-android-ar`, so the plain
  cross-prefix alone is not enough on r26.

`$PREFIX` (`$DEPS/ffmpeg/arm64-v8a`) is what you pass as
`-DRENDER360_FFMPEG_DIR`.

**No libx264/libx265.** Building those for Android is a separate cross-compile
and they are GPL. Consequence: see "Known gaps" — the exporter's default
`libx264` will not be found.

---

## 4. OpenCV for Android

Do **not** cross-compile OpenCV; the official Android SDK zip ships prebuilt
static libs for all four ABIs plus a ready CMake package.

```bash
cd "$DEPS"
# From https://github.com/opencv/opencv/releases — pick a 4.x release and take
# the opencv-<version>-android-sdk.zip asset. 4.10.0 is a good default.
wget https://github.com/opencv/opencv/releases/download/4.10.0/opencv-4.10.0-android-sdk.zip
unzip -q opencv-4.10.0-android-sdk.zip      # -> OpenCV-android-sdk/
```

The CMake package directory (the one holding `OpenCVConfig.cmake`) is:

```bash
export OPENCV_DIR="$DEPS/OpenCV-android-sdk/sdk/native/jni"
ls "$OPENCV_DIR/OpenCVConfig.cmake"
```

Pass that as `-DOpenCV_DIR=`. Stay on OpenCV **4.x**: the build asks for
`core imgproc features2d calib3d`, and `calib3d` was reorganised in OpenCV 5.
`videoio` is correctly *not* requested on Android — it is only used by the
desktop `tracking_tests` diagnostics target, which `CMakeLists.txt` excludes
from Android builds.

---

## 5. Configure and build Render360

From the repo root (`/home/pallen/Build/360Render/Render360`):

```bash
export QT_ANDROID=$HOME/Qt/6.11.2/android_arm64_v8a     # your installed version
export QT_HOST=$HOME/Qt/6.11.2/gcc_64
export JAVA_HOME=/usr/lib/jvm/java-21-openjdk-amd64
export ANDROID_SDK_ROOT=$HOME/Android/Sdk
export ANDROID_NDK_ROOT=$ANDROID_SDK_ROOT/ndk/27.2.12479018

"$QT_ANDROID/bin/qt-cmake" -G Ninja \
  -S . -B build-android \
  -DCMAKE_BUILD_TYPE=Release \
  -DANDROID_ABI=arm64-v8a \
  -DANDROID_PLATFORM=android-28 \
  -DANDROID_SDK_ROOT="$ANDROID_SDK_ROOT" \
  -DANDROID_NDK_ROOT="$ANDROID_NDK_ROOT" \
  -DQT_HOST_PATH="$QT_HOST" \
  -DRENDER360_FFMPEG_DIR="$DEPS/ffmpeg/arm64-v8a" \
  -DOpenCV_DIR="$DEPS/OpenCV-android-sdk/sdk/native/jni"
```

`qt-cmake` is only a thin wrapper that sets the toolchain file. If you prefer
plain `cmake`, this is the equivalent — Qt's toolchain file chains to the NDK's
`android.toolchain.cmake` itself, so pass Qt's, not the NDK's:

```bash
cmake -G Ninja -S . -B build-android \
  -DCMAKE_TOOLCHAIN_FILE="$QT_ANDROID/lib/cmake/Qt6/qt.toolchain.cmake" \
  ...same -D flags as above...
```

Build the native library, then package:

```bash
cmake --build build-android --parallel                 # librender360_arm64-v8a.so
cmake --build build-android --target apk               # runs androiddeployqt + Gradle
```

`--target apk` is what invokes `androiddeployqt`; the equivalent by hand is:

```bash
"$QT_HOST/bin/androiddeployqt" \
  --input  build-android/android-render360-deployment-settings.json \
  --output build-android/android-build \
  --android-platform android-34 \
  --jdk "$JAVA_HOME" \
  --gradle
```

The APK lands under `build-android/android-build/build/outputs/apk/`; confirm
the exact name with:

```bash
find build-android -name '*.apk'
```

(An unsigned release build needs `--target aab` / your own signing config;
the default `apk` target produces a debug-signed APK, which is all you need
for sideloading onto the Edge 40.)

**The first `--target apk` will download Gradle and the AGP dependencies** —
give it network access and a few minutes.

---

## 6. Install and run on the Motorola Edge 40

Enable Developer options → USB debugging on the phone, plug it in, accept the
RSA prompt.

```bash
adb devices                       # the Edge 40 must show as "device", not "unauthorized"
adb install -r "$(find build-android -name '*.apk' | head -1)"
adb shell am start -n uk.co.overview.render360/org.qtproject.qt.android.bindings.QtActivity
```

Watch the log. Qt's messages come out under the tag `Qt`; native crashes under
`DEBUG` / `libc`:

```bash
adb logcat -c                     # clear first, then launch
adb logcat Qt:V libRender360:V DEBUG:V AndroidRuntime:E *:S
```

If that filter is too narrow while something is going wrong early, widen it:

```bash
adb logcat | grep -iE 'qt|render360|libav|opencv|mediacodec'
```

### Checking which RHI backend Qt Quick picked

Qt Quick on Android defaults to the **OpenGL** RHI backend. That matters here:
`GpuRenderer` and `FlowRenderer` create their own GL contexts and compile raw
GLSL (`#version 300 es` on Android, via `glsladapt`), so **the app must stay on
the OpenGL backend** — forcing Vulkan would leave those renderers without a GL
context to share. Vulkan is worth trying only as an experiment.

Set env vars for a Qt Android app through the activity's `extraenvvars` extra,
which QtActivity base64-decodes and splits on tabs:

```bash
# QSG_INFO=1 (tab) QSG_RHI_BACKEND=opengl
ENVB64=$(printf 'QSG_INFO=1\tQSG_RHI_BACKEND=opengl' | base64 -w0)
adb shell am start -n uk.co.overview.render360/org.qtproject.qt.android.bindings.QtActivity \
    -e extraenvvars "$ENVB64"
```

Swap `opengl` for `vulkan` to test the other path. With `QSG_INFO=1` the log
prints the chosen backend and the device caps — grep for it:

```bash
adb logcat -d | grep -iE 'QRhi|Using .* backend|GL_VERSION|Vulkan'
```

You are looking for a line naming the backend (`OpenGL` / `Vulkan`) and, for
OpenGL, a `GL_VERSION` reporting **OpenGL ES 3.2** on the Mali-G77 MC9.

---

## 7. Known gaps — the APK will build before it is useful

Getting a green build is step one. As of this commit the following are **not
ported**, and each will bite at runtime:

1. **SAF storage layer — not written.** The app still opens plain filesystem
   paths (`QString` → `avformat_open_input`) and the manifest deliberately asks
   for no storage permission. Nothing yet issues
   `ACTION_OPEN_DOCUMENT_TREE`, takes the persistable grant, enumerates the
   clip plus its `.imu` / `_thm.MP4` / `.keyframes.json` siblings, or turns a
   `content://` URI into something FFmpeg can read (that needs a JNI
   `ParcelFileDescriptor` → custom `AVIOContext`, since libavformat cannot open
   a content URI). **Until this exists the app can open nothing.**
2. **MediaCodec decode/encode selection — not wired.** `VideoDecoder` calls
   `avcodec_find_decoder(codecpar->codec_id)`, which always returns the
   software decoder even when `h264_mediacodec` is compiled in. Nothing routes
   the low-resolution `_thm.MP4` proxy to hardware, and nothing chooses a
   hardware *encoder* for export.
3. **Export encoder will fall back to MPEG-4.** `Exporter` defaults to
   `libx264` via `avcodec_find_encoder_by_name()`, and the Android FFmpeg built
   above has no libx264 — the existing fallback path then silently picks
   `AV_CODEC_ID_MPEG4`. Android needs `h264_mediacodec` / `hevc_mediacodec`
   selected explicitly (and the bitrate-vs-CRF branch reworked, since
   MediaCodec encoders are bitrate-only).
4. **Foreground-service export — not implemented.** The permissions and the
   commented `<service android:foregroundServiceType="dataSync">` block are in
   the manifest, but the `ExportService` Java class does not exist. Export
   currently runs on the app's own threads and will be killed when
   backgrounded.
5. **Touch UI — not adapted.** The QML (`ControlPanel`, `CurveEditor`,
   `Timeline`) is laid out for a desktop window with mouse hover, drag handles
   and a fixed-width side panel. It renders on a phone but is not usable
   one-handed, and there is no `FileDialog` replacement for the SAF picker.
6. **Desktop-only features that will never exist on Android:**
   * the **external `ffmpeg` CLI / vidstab passes** — `Exporter` shells out via
     `QProcess` to `ffmpeg` for `vidstabdetect`/`vidstabtransform`. Android
     apps cannot exec a bundled binary from app storage on modern API levels;
     the `--export-vidstab` and hybrid paths must be compiled out or reimplemented
     in-process.
   * **`hevc_nvenc`** — NVIDIA-only. The `hevc_nvenc` branch in `Exporter` and
     the `--export-codec hevc_nvenc` CLI option are dead code on Android.
7. **The optical-flow / stitching shaders need a GLES audit.** `FlowRenderer`
   deliberately targets `#version 300 es` on Android, but the desktop path uses
   MRT + `layout(binding=...)` + RG32F. Colour-buffer float rendering and the
   binding-layout syntax need verifying on the Mali-G77 before the seam
   matcher can be trusted on-device.

---

## Continuous integration (GitHub Actions)

`.github/workflows/build.yml` reproduces this whole setup on a hosted
`ubuntu-24.04` runner. It calls the *same three scripts* a developer calls —
`fetch-deps.sh`, `build-x264-ffmpeg.sh`, `release-android.sh` — which is why
they take their paths from the environment (`DEPS`, `QT_ROOT`, `QT_VER`,
`OUT`, `ANDROID_SDK_ROOT`, `ANDROID_NDK_ROOT`, `KEYSTORE`, `KEYSTORE_PASS`,
`KEY_ALIAS`, `CMAKE`) and fall back to this machine's layout. Change the build
in one place and both paths follow.

What the runner has to install, and how long it costs:

| piece | how | cold | cached |
|---|---|---|---|
| JDK 21 | `actions/setup-java` | ~10 s | — |
| Android SDK 36, build-tools 36.0.0, NDK 27.2.12479018 | `sdkmanager` (the image already has cmdline-tools) | ~2 min | — |
| Qt 6.11.2 host + `android_arm64_v8a` + `android_x86_64` | `aqtinstall` (unattended; the Qt online installer's account login is not scriptable) | ~5 min | ~1 min |
| FFmpeg + x264, both ABIs, and the OpenCV Android SDK | the two scripts above, in a separate `android-deps` job | ~20 min | ~1 min |
| Gradle | androiddeployqt's own wrapper | ~2 min | ~20 s |

The dependency cache key is
`android-deps-ff<ffmpeg>-ocv<opencv>-x264<commit>-ndk<ver>-<hash of both
scripts>`, so editing a configure flag rebuilds it and nothing else does. The
job probes the cache with `lookup-only` and skips even the download on a hit;
only the APK job restores the tree.

`fetch-deps.sh --prune` (CI only — it deletes source trees) drops the FFmpeg
sources, the OpenCV samples/Java SDK and the two ABIs we do not ship, which is
what keeps the cached tree inside GitHub's 10 GB repo cache budget alongside
Qt.

### Signing

Without secrets the workflow still builds; `release-android.sh` just writes
`reframe360-arm64-unsigned.apk` and says so. To have CI produce installable
APKs, add three repository secrets (Settings → Secrets and variables →
Actions):

| secret | value |
|---|---|
| `ANDROID_KEYSTORE_BASE64` | `base64 -w0 ~/.android/render360-release.keystore` |
| `ANDROID_KEYSTORE_PASSWORD` | the store password (`render360`) |
| `ANDROID_KEY_ALIAS` | the key alias (`render360`) |

Understand what that means before doing it: the release key then lives in
GitHub, and anyone who can push a workflow change to this repository can sign
an APK as you. On a private repo with only you as a collaborator that is a
reasonable trade; if that ever stops being true, drop the secrets and sign the
CI artifact locally with `apksigner` instead. The keystore itself must still be
backed up off this machine — losing it means every user has to uninstall (and
loses their SAF folder grant and settings) to move to a rebuilt key.

The workflow writes the decoded keystore to `$RUNNER_TEMP`, never into the
workspace, and deletes it in an `always()` step so a failed build cannot leave
it behind for an artifact upload to pick up.

### Triggers

* push to `main`, and any pull request → Linux build + **arm64 APK**
* manual run (Actions → build → Run workflow) → optional **universal APK** too
* push a `v*` tag → both APKs, plus a GitHub release with everything attached

Two things the CI deliberately does **not** do: it does not bump
`QT_ANDROID_VERSION_CODE` (that stays a conscious edit in `CMakeLists.txt`
before a release), and it does not run the on-device tests — nothing in the
workflow has a phone, a GPU or a sample clip, so a green run means "it
compiles, links and starts", not "it renders correctly".

Note that the Android binaries are **GPL** (x264 is linked in for the 5.7K
software encode), so a published release carries that licence.
