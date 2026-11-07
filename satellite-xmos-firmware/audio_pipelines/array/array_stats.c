// Copyright 2026 Future Proof Homes

#include <math.h>
#include <string.h>
#include "array_stats.h"

/* Samples are scaled down by 2^4 before squaring: (2^31 >> 4)^2 = 2^54 and a
 * 240-sample sum stays below 2^62, so the int64 accumulator cannot overflow. */
#define STATS_PRE_SHIFT      (4)
#define STATS_SQUARE_BITS    (2 * (31 - STATS_PRE_SHIFT))     /* 54 */

/* Power smoothing: ~0.3 s time constant at 15 ms frames. */
#define STATS_POWER_ALPHA    (0.95f)

/* -140 dBFS floor: below the 32-bit noise floor, keeps logs finite. */
#define STATS_POWER_FLOOR    (1e-14f)

float array_power_to_db(float power)
{
    if (power < STATS_POWER_FLOOR) {
        power = STATS_POWER_FLOOR;
    }
    return 10.0f * log10f(power);
}

float array_frame_mean_square(const int32_t *x, unsigned n)
{
    int64_t acc = 0;
    for (unsigned i = 0; i < n; i++) {
        const int32_t v = x[i] >> STATS_PRE_SHIFT;
        acc += (int64_t)v * v;
    }
    /* Normalise so that a full-scale DC signal is 1.0. */
    return ldexpf((float)acc, -STATS_SQUARE_BITS) / (float)n;
}

void array_stats_init(array_stats_t *s, unsigned num_channels)
{
    memset(s, 0, sizeof(*s));
    if (num_channels > ARRAY_STATS_MAX_CHANNELS) {
        num_channels = ARRAY_STATS_MAX_CHANNELS;
    }
    s->num_channels = num_channels;
    for (unsigned ch = 0; ch < ARRAY_STATS_MAX_CHANNELS; ch++) {
        s->last_rms_dbfs[ch] = array_power_to_db(0.0f);
    }
}

void array_stats_update(array_stats_t *s,
                        const int32_t (*frames)[ARRAY_FRAME_ADVANCE])
{
    for (unsigned ch = 0; ch < s->num_channels; ch++) {
        const int32_t *x = frames[ch];
        int32_t peak = s->peak_abs[ch];
        uint32_t clips = 0;

        for (unsigned i = 0; i < ARRAY_FRAME_ADVANCE; i++) {
            /* |INT32_MIN| does not fit in int32_t; saturate it to INT32_MAX. */
            const int32_t a = (x[i] == INT32_MIN) ? INT32_MAX : (x[i] < 0 ? -x[i] : x[i]);
            if (a > peak) {
                peak = a;
            }
            if (a >= ARRAY_CLIP_THRESHOLD) {
                clips++;
            }
        }

        const float ms = array_frame_mean_square(x, ARRAY_FRAME_ADVANCE);
        s->power_ema[ch] = (s->frames == 0)
                ? ms
                : STATS_POWER_ALPHA * s->power_ema[ch] + (1.0f - STATS_POWER_ALPHA) * ms;
        s->last_rms_dbfs[ch] = array_power_to_db(ms);
        s->peak_abs[ch] = peak;
        s->clip_count[ch] += clips;
    }
    s->frames++;
}

float array_stats_rms_dbfs(const array_stats_t *s, unsigned ch)
{
    return array_power_to_db(s->power_ema[ch]);
}

void array_stats_reset_peaks(array_stats_t *s)
{
    memset(s->peak_abs, 0, sizeof(s->peak_abs));
}
