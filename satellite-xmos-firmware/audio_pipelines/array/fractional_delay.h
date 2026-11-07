// Copyright 2026 Future Proof Homes
//
// Fixed-point fractional-delay FIR filters (Kaiser-windowed sinc).

#ifndef FRACTIONAL_DELAY_H_
#define FRACTIONAL_DELAY_H_

#include <stdint.h>
#include "array_config.h"

/* Length of the history a caller must prepend to each input block. */
#define ARRAY_FD_HISTORY   (ARRAY_FD_TAPS - 1)

typedef struct {
    /* h[k] in Q2.30; y[n] = sum_k h[k] * x[n - k]. Coefficients sum to exactly
     * 1.0 (2^30), so the DC gain is exactly unity. */
    int32_t coef[ARRAY_FD_TAPS];
} array_fd_filter_t;

/* Design a filter whose delay is ARRAY_FD_BULK_DELAY + offset samples.
 * |offset| must not exceed ARRAY_FD_BULK_DELAY - 1 (accuracy is specified for
 * |offset| <= 1.5, which covers this array). Offset 0 yields an exact pure
 * delay. Uses float maths; call at initialisation only. Returns 0 on success,
 * -1 if the offset is out of range. */
int array_fd_design(array_fd_filter_t *f, float offset_samples);

/* Filter n output samples.
 *   x_ext: ARRAY_FD_HISTORY samples of history followed by the n new input
 *          samples (x_ext[ARRAY_FD_HISTORY + i] is input sample i).
 *   y:     n output samples (may not alias x_ext).
 * Accumulates in 64 bits, rounds, and saturates to int32. */
void array_fd_apply(const array_fd_filter_t *f,
                    const int32_t *x_ext,
                    int32_t *y,
                    unsigned n);

/* Saturate a 64-bit value to int32 range. */
static inline int32_t array_sat32(int64_t v)
{
    if (v > INT32_MAX) {
        return INT32_MAX;
    }
    if (v < INT32_MIN) {
        return INT32_MIN;
    }
    return (int32_t)v;
}

#endif /* FRACTIONAL_DELAY_H_ */
