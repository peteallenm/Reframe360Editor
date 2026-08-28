# Tracking Pipeline Tests

Build and run:

```
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build --target tracking_tests -j$(nproc)
build/tracking_tests [--quick] <video1.MP4> [video2.MP4 ...]
```

`--quick` skips the slow visual-rotation / calibration tests (which run ORB feature
matching over full clips — minutes on long footage) and only runs the fast
IMU/gyro-integration diagnostics (~0.05 s per clip).

The harness builds the IMU/visual/gyro pipeline modules standalone (no GUI) and
drives them directly, so each stage can be verified independently of the app.

Exit code is 0 if no test fails.

## Tests

| # | Name | Verifies | Debug value |
|---|------|----------|-------------|
| 1 | IMU parser | Timestamps monotonic, dt sane, counter increments plausible, duration > 0 | Corruption / counter-wrap / dropped-packet handling |
| 2a | Integration continuity | No >3 deg jump between adjacent 400 Hz samples | Single-pass integration stability |
| 2b | 30 fps unsmoothed continuity | No >20 deg jumps in the `q_actual` path (the flicker metric) | **The exact flicker diagnostic** |
| 3 | Bearing back-projection | (implicit via test 4 axis check) | Lens calibration sanity |
| 4 | Visual rotation axis on Just* clips | Dominant axis of accumulated visual rotation ≈ clip's intended axis | Bearing/lens-frame correctness |
| 5 | Gyro calibration matrix | Reports the solved 3x3 matrix + residual on Just* clips | Scale/recovery vs hardcoded 32.18/33.51/33.64 |
| 6 | Sync determinism | Two `solve()` runs give offset within 10 ms | Cross-correlation stability |
| 7 | Fusion continuity | Fused chain jumps < 5 deg, fused vs gyro within 20 deg | Visual-fusion drift correction sanity |

## How to debug specific symptoms

### "Auto-stabilize toggling appears to do nothing"
Run the interpreter probe on the clip. Two things to check:

1. **Is JITTER present?** Compare `30fps unsmoothed path continuous` (Test 2b).
   On a smooth controlled clip (e.g. JustPitch, JustRoll — a single steady pan)
   there is almost no high-frequency shake, so HIGH-PASS stabilization
   legitimately produces a tiny correction (max ~0.5-0.7°). The view keeps
   following the 360° motion by design — that is NOT "nothing happens".

2. **Sign inversion.** This was a real bug: `imuOrientation` was returned as
   `qActual^{-1} * qVirtual` but the shader needs `qVirtual^{-1} * qActual`
   (because LensViewer builds its matrix from the conjugate). The inverted
   ordering *applied* the shake instead of cancelling it, so the toggle looked
   like a no-op. Both `App::imuOrientationAt` and `ExportSnapshot::stateAt`
   return `qVirtual.conjugated() * qActual` now.

**Dual-mode stabilization:** the smoothing slider now has two regions:
- Smoothing **≤ 0.9** — HIGH-PASS ("remove shake, keep motion"): q_virtual is
  the Gaussian-smoothed virtual path; the correction cancels shake while pans
  still move. A full 360° rotation stays visible (correct).
- Smoothing **> 0.9** — LOW-PASS ("hold world steady"): q_virtual is pinned to
  the first orientation, so the correction cancels ALL rotation up to ~180°.
  Even a 360° clip appears locked to its first frame.

To SEE stabilization working on JustPitch: slide smoothing past 0.9 — the
360° pitch motion will freeze to the first frame. At default 0.5 it follows the
pan while removing any handheld jitter.

### "Flickers to a different view for a frame, every couple of seconds"
This is **Test 2b**. Run:
```
build/tracking_tests ./YIVR_0830_360.MP4
```
Look at the `30fps unsmoothed path continuous` line. A healthy clip should have
~0 jumps > 20 deg. Many jumps (tens to hundreds) means the **stored orientation
chain itself is discontinuous** — the two-pass forward/backward integration
blend snaps between two divergent passes in regions with little camera stillness.
This is NOT a fusion / visual-rotation issue; it happens with stabilisation on
but before any visual pipeline, because `q_actual` is taken unsmoothed from the
integrated chain.

Known-good repro: `JustRoll.MP4` used to pass while `YIVR_*` handheld clips failed
badly (49/27/134/681 jumps). That was the two-pass forward/backward blend snapping
between two anchors: on continuously-moving handheld footage the accelerometer-
derived seed attitudes and the gyro-integrated trajectory disagree by up to ~170°
(measured: 426° gyro rotation vs 44° gravity change between anchors), so the two
passes sit in different absolute attitudes and any blend flips between them.

**FIXED (single-pass integration):** the integrator now uses one forward Mahony
pass anchored at the earliest still sample, integrating both directions from it
(before and after the seed). A single continuous pass cannot flicker. All clips
now pass 10/10. Any slow drift this sacrifices is recovered by the visual-fusion
correction on the virtual path (q_virtual).

### "Auto sync fails / hangs"
Run `--quick` first — if Tests 1-2 pass, the IMU/integration core is healthy and
the failure is in visual-rotation or downstream. Then run the full suite on the
trouble clip. Watch for:
- Test 4 `visual pairs` too few / axis violated → feature matching or bearing
  back-projection is poor on that footage.
- Test 5 residual very high → visual rotation quality too low for calibration.

### "Roll drifts over time"
- Test 2b should be clean (no integration jumps). If it is, drift is a
  low-frequency bias in the gyro, not a jump — check the two-pass integral term
  (Test 2a) and confirm the gate weights are letting the bias accumulate during
  still moments (not currently separately tested; add a gate-weight dump).
- If Test 5 reports an off-diagonal matrix element, cross-axis misalignment is
  being fitted — verify against the camera's true axes.

### "Sync offset looks wrong"
- Test 6 determines whether sync is deterministic. If it fails (offset varies
  > 10 ms run-to-run) the cross-correlation peak is weak — poor visual rotation
  quality (Test 4) or insufficient motion.
- Test 1 verifies the IMU timestamps that the sync model rests on.

## Golden-clip expectations (for a healthy pipeline)

Axes are in the camera frame after the IMU->camera rotation (measured from the
IMU data): JustRoll rotates about **Z**, JustPitch about **X**, JustYaw about
**Y**. The rotational-axis check uses an angle-weighted per-pair projection
(|axis_i · expected|), sign-agnostic, because these clips perform full 360°
rotations whose accumulated rotation vector wraps to ~0 (the +180° half cancels
the -180° half) — that is expected physical behaviour, not a tracking failure.
A dominance > 0.60 means the motion is predominantly on the expected axis.

Visual rotation now uses motion-adaptive frame density (wide hops on still
sections, narrowed to consecutive frames during fast motion) and an adaptive
outlier threshold (generous start, tightened toward the residual floor), so
fast 360° calibration clips stay trackable at default settings.

- `JustRoll.MP4` (Z-axis): accumulated visual rotation axis dot(Z) > 0.7.
- `JustPitch.MP4` (X-axis): axis dot(X) > 0.7.
- `JustYaw.MP4` (Y-axis): axis dot(Y) > 0.7, >= 20 visual pairs.
- `YIVR_*` handheld: few/no > 20 deg jumps in Test 2b (all clips now pass).