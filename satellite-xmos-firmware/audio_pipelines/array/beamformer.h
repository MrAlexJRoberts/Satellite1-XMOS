// Copyright 2026 Future Proof Homes
//
// Fixed delay-and-sum beam bank for the Satellite1 four-microphone circular
// array (see array_config.h for the geometry and its provenance).
//
// Beam k looks towards azimuth k * 45 degrees (k = 0..7) in the firmware's
// channel coordinate frame. For a far-field source at azimuth theta, microphone
// m (at angle phi_m, radius r) receives the wavefront tau_m = (r/c) cos(theta -
// phi_m) earlier than the array centre. Beam k delays microphone m by
// D + (r/c) cos(theta_k - phi_m) and averages, so a source on the look axis
// adds coherently with unity gain and no phase distortion.
//
// Because the microphones are 90 degrees apart and the beams 45 degrees apart,
// theta_k - phi_m is always a multiple of 45 degrees, so each microphone only
// needs five distinct delays (cos = 1, 0.707, 0, -0.707, -1), the middle one an
// exact integer delay. All eight beams are built from those 4 x 5 delayed
// signals: 16 fractional-delay FIRs per frame instead of 32.

#ifndef BEAMFORMER_H_
#define BEAMFORMER_H_

#include <stdint.h>
#include "array_config.h"
#include "fractional_delay.h"

/* Distinct |theta_k - phi_m| values: 0, 45, 90, 135, 180 degrees. */
#define ARRAY_DELAY_CLASSES   (ARRAY_ANGLE_STEPS / 2 + 1)

typedef struct {
    array_fd_filter_t fd[ARRAY_DELAY_CLASSES];
    float   offset_samples[ARRAY_DELAY_CLASSES];        /* steering offset per class */
    uint8_t is_pure_delay[ARRAY_DELAY_CLASSES];         /* offset 0: no FIR needed */
    uint8_t class_of[ARRAY_BEAM_COUNT][ARRAY_MIC_COUNT];
    int32_t hist[ARRAY_MIC_COUNT][ARRAY_FD_HISTORY];    /* per-microphone input history */
    int32_t prev_last[ARRAY_BEAM_COUNT];                /* for pre-emphasis continuity */
} array_beamformer_t;

/* Design the filters and clear the history. Uses float maths; call once at
 * start-up. Returns 0 on success. */
int array_beamformer_init(array_beamformer_t *bf);

/* Process one frame.
 *   mics:  ARRAY_MIC_COUNT channels of ARRAY_FRAME_ADVANCE Q1.31 samples, in
 *          firmware channel order (E, W, N, S). Typically the AEC outputs.
 *   beams: ARRAY_BEAM_COUNT output channels. Unity gain on the look axis; each
 *          output is the mean of four delayed inputs so it cannot exceed full
 *          scale by more than the interpolation overshoot, which saturates.
 *   beam_energy_db: optional (may be NULL). Pre-emphasised (first difference)
 *          frame energy of each beam in dB, used for beam selection: the
 *          pre-emphasis removes the low-frequency band where a 64 mm array has
 *          almost no directivity.
 * Latency: ARRAY_FD_BULK_DELAY samples (0.31 ms). */
void array_beamformer_process(array_beamformer_t *bf,
                              const int32_t (*mics)[ARRAY_FRAME_ADVANCE],
                              int32_t (*beams)[ARRAY_FRAME_ADVANCE],
                              float *beam_energy_db);

/* The beam looking the opposite way (used as the IC noise reference). */
static inline unsigned array_beam_opposite(unsigned beam)
{
    return (beam + ARRAY_BEAM_COUNT / 2) % ARRAY_BEAM_COUNT;
}

#endif /* BEAMFORMER_H_ */
