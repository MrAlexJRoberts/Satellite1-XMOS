// Copyright 2026 Future Proof Homes
//
// Mono AEC reference for the four-microphone pipeline.
//
// Why mono: a 4Y x 2X AEC state needs 376 KB (fwk_voice v0.9.0 aec_state_t,
// 10 main + 5 shadow phases), which does not fit on tile 1. 4Y x 1X needs
// 211 KB, slightly less than today's 2Y x 2X (216 KB).
//
// Why that is correct for the internal speaker: the Satellite1 TAS2780 is a
// mono amplifier. ESPHome configures its TDM receiver (tas2780.cpp, TDM_CFG2
// RX_SCFG) as the (L+R)/2 stereo downmix by default, or as Left-only /
// Right-only. The echo is a linear function of exactly that mono signal, so a
// matching mono reference loses nothing. A mismatched mix (e.g. the amplifier
// in Left-only mode with a downmix reference) cannot be cancelled for the part
// of the content where L and R differ; the host AEC test quantifies this.

#ifndef ARRAY_REFERENCE_H_
#define ARRAY_REFERENCE_H_

#include <stdint.h>

/* Form the mono reference. mode is one of ARRAY_REF_MIX_*. The downmix is
 * computed in 64 bits and cannot overflow. out may alias left or right. */
void array_reference_mix(const int32_t *left,
                         const int32_t *right,
                         int32_t *out,
                         unsigned n,
                         int mode);

#endif /* ARRAY_REFERENCE_H_ */
