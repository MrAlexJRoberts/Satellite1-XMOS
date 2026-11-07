// Copyright 2026 Future Proof Homes
//
// Satellite1 microphone-array configuration shared by the firmware and the
// host unit tests. Pure C, no RTOS or xcore dependencies.

#ifndef ARRAY_CONFIG_H_
#define ARRAY_CONFIG_H_

#include <stdint.h>

/* ---------------------------------------------------------------------------
 * Framing. Must match appconfAUDIO_PIPELINE_FRAME_ADVANCE and the 16 kHz
 * pipeline rate; the pipeline checks this with static asserts.
 * ------------------------------------------------------------------------- */
#define ARRAY_FRAME_ADVANCE         (240)
#define ARRAY_SAMPLE_RATE_HZ        (16000)
#define ARRAY_MIC_COUNT             (4)

/* ---------------------------------------------------------------------------
 * Geometry.
 *
 * Source: FutureProofHomes/Satellite1-Hardware, hat/rev4.1hat3D.step and
 * hat/rev6.1hat3D.step (KiCad STEP export). The four CUI CMM-4030DT-261280-TR
 * bottom-port PDM microphones MK1..MK4 sit on a circle around their common
 * centroid (identical in both revisions), PCB top view, millimetres:
 *
 *     MK1 (+0.10, +32.08)   MIC_DATA_2, SEL = GND
 *     MK2 (+32.08, -0.10)   MIC_DATA_2, SEL = VDD
 *     MK3 (-0.10, -32.08)   MIC_DATA_1, SEL = GND
 *     MK4 (-32.08, +0.10)   MIC_DATA_1, SEL = VDD
 *
 * rev6.1 schematic: MIC_DATA_1 -> X1D16 (PORT_4D bit 0), MIC_DATA_2 -> X1D17
 * (PORT_4D bit 1), MIC_CLK -> X1D22 (PORT_1G). Radius 32.08 mm, so adjacent
 * microphones are 45.37 mm apart and opposite microphones 64.16 mm apart. The
 * 0.10 mm tangential offsets (0.18 degrees) are ignored here.
 *
 * Firmware channel order (bsp_config/SATELLITE1/SATELLITE1.cmake, 4-mic board):
 * PDM inputs {4, 5, 0, 1} = {East, West, North, South}. The compass names come
 * from the existing firmware comment. The PCB data line/SEL wiring proves that
 * East/West and North/South are each a diametrically opposite pair, but it does
 * not fix which SEL level lib_mic_array assigns to which DDR index. So the
 * angular handedness (a mirror image) and which way "North" faces in the
 * enclosure cannot be verified from source. Beam performance does not depend
 * on either; only the beam index -> physical direction label does. The test
 * plan includes a one-off on-device check.
 *
 * Angles: mathematical convention, counter-clockwise from East, in units of
 * 45 degrees (ARRAY_ANGLE_STEPS steps per turn) so that the symmetry used by
 * the beamformer is exact.
 * ------------------------------------------------------------------------- */
#define ARRAY_RADIUS_UM             (32080)     /* micrometres */
#define ARRAY_SPEED_OF_SOUND_MM_S   (343000)    /* 343 m/s */
#define ARRAY_ANGLE_STEPS           (8)         /* 45 degree steps */

#define ARRAY_CH_EAST               (0)
#define ARRAY_CH_WEST               (1)
#define ARRAY_CH_NORTH              (2)
#define ARRAY_CH_SOUTH              (3)

/* Microphone angular positions, in 45-degree steps, indexed by channel. */
#define ARRAY_MIC_ANGLE_STEPS_INIT  { 0, 4, 2, 6 }

/* The two diametrically opposite pairs (pair A is the production E/W pair). */
#define ARRAY_PAIR_COUNT            (2)
#define ARRAY_PAIR_A_TARGET         ARRAY_CH_EAST
#define ARRAY_PAIR_A_REFERENCE      ARRAY_CH_WEST
#define ARRAY_PAIR_B_TARGET         ARRAY_CH_NORTH
#define ARRAY_PAIR_B_REFERENCE      ARRAY_CH_SOUTH

/* ---------------------------------------------------------------------------
 * Fractional-delay filters for the delay-and-sum beamformer.
 *
 * Far-field delay of microphone m relative to the array centre for a source at
 * azimuth theta: tau_m = (r / c) * cos(theta - phi_m). With r = 32.08 mm,
 * c = 343 m/s, fs = 16 kHz: r*fs/c = 1.4965 samples, so tau spans +/-1.50
 * samples (+/-93.5 us) and opposite microphones differ by up to 2.99 samples
 * (187 us). Integer-sample steering would leave up to 0.5 sample (31 us) of
 * error, i.e. 45 degrees of phase error at 4 kHz, so fractional delays are used.
 *
 * Kaiser-windowed sinc, 11 taps, beta 4.5, integer bulk delay of 5 samples (the
 * filter centre). Chosen by sweeping length and beta over the full +/-1.5
 * sample range; the fixed-point filters measure a worst gain error of 0.21 dB
 * and worst phase error of 0.82 degrees from 250 Hz to 6 kHz
 * (tests/test_array_host). The zero-offset case is an exact pure delay.
 * Group delay: 5 samples (0.31 ms).
 * ------------------------------------------------------------------------- */
#define ARRAY_FD_TAPS               (11)
#define ARRAY_FD_BULK_DELAY         ((ARRAY_FD_TAPS - 1) / 2)
#define ARRAY_FD_KAISER_BETA        (4.5f)
#define ARRAY_FD_COEF_FRAC_BITS     (30)        /* Q2.30 coefficients */

/* ---------------------------------------------------------------------------
 * Beam bank.
 * ------------------------------------------------------------------------- */
#define ARRAY_BEAM_COUNT            (ARRAY_ANGLE_STEPS)   /* 8 beams, 45 deg apart */

/* ---------------------------------------------------------------------------
 * Selection. Times are expressed in 15 ms frames.
 *
 * Beam scores are pre-emphasised (first difference) frame energies in dB. With
 * this geometry a talker on a beam axis gives its neighbours about -4.7 dB of
 * pre-emphasised, speech-weighted energy, but only about -0.7 dB full-band,
 * because a 64 mm aperture is nearly omnidirectional below 1 kHz. Hence the
 * pre-emphasis, and a 1.5 dB switching margin.
 *
 * Pair scores are the two IC paths' output VNR predictions (0..1).
 * ------------------------------------------------------------------------- */
#define ARRAY_SELECT_SMOOTH_ALPHA        (0.95f)   /* ~0.3 s smoothing */
#define ARRAY_BEAM_SWITCH_MARGIN_DB      (1.5f)
#define ARRAY_PAIR_SWITCH_MARGIN_VNR     (0.10f)
#define ARRAY_PAIR_SPEECH_VNR_THRESHOLD  (0.50f)
#define ARRAY_SELECT_CONFIRM_FRAMES      (10)      /* 150 ms */
#define ARRAY_SELECT_MIN_DWELL_FRAMES    (34)      /* ~0.5 s */

/* ---------------------------------------------------------------------------
 * Front-end modes (appconfARRAY_FRONTEND_MODE), reference mixes
 * (appconfARRAY_REF_MIX) and development audio taps.
 * ------------------------------------------------------------------------- */
#define ARRAY_FRONTEND_PAIR_SELECT  (1)   /* two IC paths (E/W, N/S), best output VNR wins */
#define ARRAY_FRONTEND_BEAM         (2)   /* 8 fixed DAS beams, IC fed look/anti-look beams */

/* Mono AEC reference formed from the stereo speaker path. Must match what the
 * TAS2780 plays: ESPHome "Speaker channel output" Mono (Left + Right) is the
 * TDM_CFG2 RX_SCFG stereo downmix (L+R)/2 (the default); Left/Right Channel
 * Only play one channel. */
#define ARRAY_REF_MIX_DOWNMIX       (0)
#define ARRAY_REF_MIX_LEFT          (1)
#define ARRAY_REF_MIX_RIGHT         (2)

#define ARRAY_TAP_OFF               (0)   /* normal output */
#define ARRAY_TAP_RAW               (1)   /* lanes = raw mics a, b (after fixed gain, before AEC) */
#define ARRAY_TAP_AEC               (2)   /* lanes = AEC outputs a, b */
#define ARRAY_TAP_FRONTEND          (3)   /* lane 0 = front-end output (before NS), lane 1 = final output */
#define ARRAY_TAP_BEAMS             (4)   /* lanes = beams a, b (beam builds) */

/* ---------------------------------------------------------------------------
 * Statistics.
 * ------------------------------------------------------------------------- */
/* A sample counts as clipped at or above 0.999 of full scale (-0.009 dBFS). */
#define ARRAY_CLIP_THRESHOLD        (INT32_MAX - (INT32_MAX / 1000))

#endif /* ARRAY_CONFIG_H_ */
