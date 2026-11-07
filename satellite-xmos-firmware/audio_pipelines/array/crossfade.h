// Copyright 2026 Future Proof Homes
//
// One-frame linear crossfade used when the array front end switches source
// (pair or beam), so a switch never produces a waveform step.

#ifndef CROSSFADE_H_
#define CROSSFADE_H_

#include <stdint.h>

/* out[i] = from[i] + (to[i] - from[i]) * (i + 1) / n, computed in 64 bits.
 * out may alias from or to. The last sample equals to[n - 1], so the next
 * frame continues seamlessly from the new source. */
void array_crossfade(const int32_t *from, const int32_t *to, int32_t *out, unsigned n);

#endif /* CROSSFADE_H_ */
