// Copyright 2026 Future Proof Homes
//
// Per-channel level statistics for the microphone array diagnostics.

#ifndef ARRAY_STATS_H_
#define ARRAY_STATS_H_

#include <stdint.h>
#include "array_config.h"

#define ARRAY_STATS_MAX_CHANNELS   (ARRAY_BEAM_COUNT)

typedef struct {
    unsigned num_channels;
    uint32_t frames;                                   /* frames observed */
    float    power_ema[ARRAY_STATS_MAX_CHANNELS];      /* smoothed mean square, full scale = 1.0 */
    float    last_rms_dbfs[ARRAY_STATS_MAX_CHANNELS];  /* RMS of the most recent frame */
    int32_t  peak_abs[ARRAY_STATS_MAX_CHANNELS];       /* largest |x| since the last peak reset */
    uint32_t clip_count[ARRAY_STATS_MAX_CHANNELS];     /* samples >= ARRAY_CLIP_THRESHOLD, cumulative */
} array_stats_t;

void array_stats_init(array_stats_t *s, unsigned num_channels);

/* Update with one frame per channel. frames[ch] holds ARRAY_FRAME_ADVANCE
 * Q1.31 samples. Cost: one pass over the data per channel, integer only, plus
 * one log10f per channel. */
void array_stats_update(array_stats_t *s,
                        const int32_t (*frames)[ARRAY_FRAME_ADVANCE]);

/* Smoothed RMS of a channel in dBFS (sine at full scale = -3.01 dBFS). */
float array_stats_rms_dbfs(const array_stats_t *s, unsigned ch);

/* Clear the peak hold (called after the peaks have been reported). */
void array_stats_reset_peaks(array_stats_t *s);

/* Mean square of one frame relative to full scale, without overflow. Exposed
 * for the beam energy computation and for tests. */
float array_frame_mean_square(const int32_t *x, unsigned n);

/* 10*log10 with a floor, so silence gives a finite number. */
float array_power_to_db(float power);

#endif /* ARRAY_STATS_H_ */
