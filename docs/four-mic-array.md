# Four-microphone array (experimental)

Two additional firmware targets use all four Satellite1 HAT microphones. The
production target `satellite1_firmware_fixed_delay` is unchanged.

| Target | Microphones | AEC | Front end before IC |
| --- | --- | --- | --- |
| `satellite1_firmware_fixed_delay` (production) | E, W | 2 mic x 2 ref | none: IC on (E, W) |
| `satellite1_firmware_fixed_delay_4mic` | E, W, N, S | 4 mic x 1 ref | two IC paths, (E, W) and (N, S); best output VNR wins |
| `satellite1_firmware_fixed_delay_4mic_bf` | E, W, N, S | 4 mic x 1 ref | 8 fixed delay-and-sum beams; IC gets look and opposite beam |

NS, AGC and the two-lane I2S output to the ESP32 are identical in all three, so
no ESPHome change is needed to run them.

## Geometry

From `Satellite1-Hardware` `hat/rev4.1hat3D.step` and `hat/rev6.1hat3D.step`:
MK1..MK4 (CUI CMM-4030DT-261280-TR) on a 32.08 mm radius circle, 90 degrees
apart. Adjacent spacing 45.37 mm, opposite 64.16 mm. Maximum inter-microphone
delay 187 us (2.99 samples at 16 kHz). See `audio_pipelines/array/array_config.h`
for the data-line and SEL wiring and what can and cannot be inferred from it
(the handedness of the E/W/N/S labels is not verifiable from source).

## Why these choices

* **fwk_voice v0.9.0**: `AEC_MAX_Y_CHANNELS` / `AEC_MAX_X_CHANNELS` became
  compile-time options. v0.8.0 hard-coded 2, which is why
  `experiment/satellite1-4y1x-aec-residual-average` forked lib_aec.
* **4 mic x 1 ref AEC**: `aec_state_t` is 216 KB for 2x2, 376 KB for 4x2 and
  211 KB for 4x1 (10 main / 5 shadow phases). 4x2 cannot fit tile 1. The
  TAS2780 plays one mono signal, by default the (L+R)/2 downmix set by
  ESPHome, so a (L+R)/2 reference models the internal speaker exactly.
* **No naive summing**: averaging the four AEC outputs without alignment (as
  the experiment branch did) has a null at 3.78 kHz for a talker at 45 degrees
  and at 5.35 kHz at 0 degrees, and is -8 to -10 dB at 3 kHz.
* **IC stays two-channel**: lib_ic is fixed at one target and one reference
  channel ("NOT USER MODIFIABLE"), so the front end reduces 4 to 2.

## Known limitations

* With the TAS2780 in Left-only or Right-only mode and stereo content, or a
  stereo line-out speaker, the mono reference cannot model the echo (the host
  test measures ~0 dB ERLE for uncorrelated L/R in that case). Set
  `appconfARRAY_REF_MIX` to match, or keep the 2-mic build for those setups.
* The beam build feeds beams, not raw microphones, to IC, which XMOS tuned
  for a ~71 mm raw pair. IC re-adapts after a beam change.
* Beam selection follows the strongest pre-emphasised energy during speech; a
  louder talker (e.g. TV dialogue) can win. Wake-word-assisted locking needs
  an ESP32 to XMOS message and is future work.
* Nothing here has been measured on hardware yet: XU316 timing and memory fit
  must be confirmed with the diagnostics below and the build `-report`.

## Building

```bash
cmake -B build --toolchain xmos_cmake_toolchain/xs3a.cmake -DUSE_DEV_TRACKING=ON
cd build
make create_flash_img_satellite1_firmware_fixed_delay_4mic
make create_flash_img_satellite1_firmware_fixed_delay_4mic_bf
```

Add `-DSAT1_ARRAY_DEV_TAP=ON` for a development build that can route raw, AEC
or beam signals onto the output lanes. Never ship a dev-tap build.

CI-style build in the XMOS builder container:
`tools/ci/build_firmware.sh tools/ci/firmwares_array_dev.txt`.

## Diagnostics

Device-control resource 233 (read command 0x80) returns
`array_diag_status_t` (`fixed_delay_array/array_diag.h`): per-microphone raw and
post-AEC level, peak, clipping, AEC ERLE, selected pair/beam and scores, stage
timing (AEC, front end + IC, NS, AGC) with deadline misses, mic input lateness
and an estimate of dropped mic frames, and heap low-water marks for both tiles.
The four-mic builds register one extra device-control servicer for it.

The Satellite1-ESPHome package `config/common/xmos_array_debug.yaml` polls it
and exposes Home Assistant diagnostic entities.

## Host tests

```bash
cmake -S tests/test_array_host -B build_host && cmake --build build_host
ctest --test-dir build_host --output-on-failure
```

Builds lib_xcore_math's reference C implementation and lib_aec from source, and
checks: the 4x1 AEC state bounds and convergence, bit-exactness of the firmware
AEC driver against `aec_process_frame()` with the 4-channel schedule,
fractional-delay accuracy, beam patterns against the analytical model, beam
selection, the selector, and that the new `app_conf.h` constants reproduce the
2-mic production values. The pipeline sources are also type-checked against
the real fwk_voice headers with small stubs for the XTC/RTOS headers.
