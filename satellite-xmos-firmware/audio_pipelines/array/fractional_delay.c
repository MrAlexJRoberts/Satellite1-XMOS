// Copyright 2026 Future Proof Homes

#include <math.h>
#include <stdlib.h>
#include "fractional_delay.h"

#define FD_PI   (3.14159265358979323846f)
#define FD_ONE  ((int32_t)1 << ARRAY_FD_COEF_FRAC_BITS)

/* Zeroth-order modified Bessel function of the first kind, by power series.
 * The terms fall off factorially, so 30 terms are plenty for beta <= 10. */
static float bessel_i0(float x)
{
    const float half_x_sq = 0.25f * x * x;
    float term = 1.0f;
    float sum = 1.0f;
    for (int k = 1; k < 30; k++) {
        term *= half_x_sq / ((float)k * (float)k);
        sum += term;
        if (term < 1e-9f * sum) {
            break;
        }
    }
    return sum;
}

/* sin(pi t) / (pi t), with sin(pi t) evaluated on the fractional part of t
 * so that integer t gives exactly 0 (float pi is not exact, and sinf(pi * k)
 * would otherwise leave ~1e-8 residues on what should be a pure delay). */
static float sinc(float t)
{
    if (fabsf(t) < 1e-6f) {
        return 1.0f;
    }
    const float n = roundf(t);
    const float r = t - n;
    float s = sinf(FD_PI * r);
    if (fmodf(fabsf(n), 2.0f) == 1.0f) {
        s = -s;
    }
    return s / (FD_PI * t);
}

int array_fd_design(array_fd_filter_t *f, float offset_samples)
{
    if (fabsf(offset_samples) > (float)(ARRAY_FD_BULK_DELAY - 1)) {
        return -1;
    }

    const float delay = (float)ARRAY_FD_BULK_DELAY + offset_samples;
    const float half_width = 0.5f * (float)ARRAY_FD_TAPS;
    const float i0_beta = bessel_i0(ARRAY_FD_KAISER_BETA);
    float h[ARRAY_FD_TAPS];
    float sum = 0.0f;

    for (int k = 0; k < ARRAY_FD_TAPS; k++) {
        const float t = (float)k - delay;
        const float u = t / half_width;
        float w = 0.0f;
        if (fabsf(u) < 1.0f) {
            w = bessel_i0(ARRAY_FD_KAISER_BETA * sqrtf(1.0f - u * u)) / i0_beta;
        }
        h[k] = sinc(t) * w;
        sum += h[k];
    }

    /* Normalise to unity DC gain, quantise, then put the rounding residue on
     * the largest tap so the fixed-point coefficients sum to exactly 1.0. */
    int64_t qsum = 0;
    int largest = 0;
    for (int k = 0; k < ARRAY_FD_TAPS; k++) {
        const float v = h[k] / sum;
        f->coef[k] = (int32_t)lrintf(ldexpf(v, ARRAY_FD_COEF_FRAC_BITS));
        qsum += f->coef[k];
        if (abs(f->coef[k]) > abs(f->coef[largest])) {
            largest = k;
        }
    }
    f->coef[largest] += (int32_t)(FD_ONE - qsum);
    return 0;
}

void array_fd_apply(const array_fd_filter_t *f,
                    const int32_t *x_ext,
                    int32_t *y,
                    unsigned n)
{
    const int64_t round = (int64_t)1 << (ARRAY_FD_COEF_FRAC_BITS - 1);
    for (unsigned i = 0; i < n; i++) {
        /* x_ext[ARRAY_FD_HISTORY + i - k] is x[i - k]. */
        const int32_t *xp = &x_ext[ARRAY_FD_HISTORY + i];
        int64_t acc = 0;
        for (int k = 0; k < ARRAY_FD_TAPS; k++) {
            acc += (int64_t)f->coef[k] * xp[-k];
        }
        y[i] = array_sat32((acc + round) >> ARRAY_FD_COEF_FRAC_BITS);
    }
}
