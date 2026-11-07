// Copyright 2026 Future Proof Homes

#include <string.h>
#include "array_config.h"
#include "array_reference.h"

void array_reference_mix(const int32_t *left,
                         const int32_t *right,
                         int32_t *out,
                         unsigned n,
                         int mode)
{
    switch (mode) {
    case ARRAY_REF_MIX_LEFT:
        if (out != left) {
            memmove(out, left, n * sizeof(int32_t));
        }
        break;
    case ARRAY_REF_MIX_RIGHT:
        if (out != right) {
            memmove(out, right, n * sizeof(int32_t));
        }
        break;
    case ARRAY_REF_MIX_DOWNMIX:
    default:
        for (unsigned i = 0; i < n; i++) {
            /* floor((L + R) / 2): within half an LSB of the exact mean. */
            out[i] = (int32_t)(((int64_t)left[i] + (int64_t)right[i]) >> 1);
        }
        break;
    }
}
