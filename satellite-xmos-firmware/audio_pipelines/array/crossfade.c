// Copyright 2026 Future Proof Homes

#include "crossfade.h"

void array_crossfade(const int32_t *from, const int32_t *to, int32_t *out, unsigned n)
{
    for (unsigned i = 0; i < n; i++) {
        const int64_t a = from[i];
        const int64_t b = to[i];
        /* The result lies between a and b, so it always fits in int32. */
        out[i] = (int32_t)(a + ((b - a) * (int64_t)(i + 1)) / (int64_t)n);
    }
}
