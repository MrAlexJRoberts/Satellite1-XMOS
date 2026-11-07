// Copyright 2026 Future Proof Homes
//
// Hysteretic N-way selector used for both microphone-pair selection and beam
// selection. Scores are smoothed only on frames the caller marks as valid (e.g.
// speech present), and a switch needs a margin, sustained for a confirmation
// period, after a minimum dwell time. This keeps the output from hopping
// between candidates on noise or between words.

#ifndef ARRAY_SELECT_H_
#define ARRAY_SELECT_H_

#include <stdint.h>
#include "array_config.h"

#define ARRAY_SELECT_MAX_CANDIDATES  (ARRAY_BEAM_COUNT)

typedef struct {
    unsigned num_candidates;
    unsigned selected;
    unsigned challenger;
    unsigned confirm_count;
    unsigned frames_since_switch;
    uint32_t switch_count;
    int      scores_valid;
    float    score[ARRAY_SELECT_MAX_CANDIDATES];   /* smoothed scores */

    /* Tuning */
    float    alpha;            /* smoothing coefficient per valid frame */
    float    margin;           /* score units */
    unsigned confirm_frames;
    unsigned min_dwell_frames;
} array_select_t;

void array_select_init(array_select_t *s,
                       unsigned num_candidates,
                       unsigned initial,
                       float alpha,
                       float margin,
                       unsigned confirm_frames,
                       unsigned min_dwell_frames);

/* Feed one frame. raw_scores has num_candidates entries (higher is better).
 * When valid is 0 the scores are ignored and the selection is held (the dwell
 * counter still advances). Returns 1 if the selection changed on this frame. */
int array_select_update(array_select_t *s, const float *raw_scores, int valid);

/* Force a selection (e.g. a compile-time or host-requested override). */
void array_select_force(array_select_t *s, unsigned selected);

#endif /* ARRAY_SELECT_H_ */
