// Copyright 2026 Future Proof Homes

#include <string.h>
#include "array_select.h"

void array_select_init(array_select_t *s,
                       unsigned num_candidates,
                       unsigned initial,
                       float alpha,
                       float margin,
                       unsigned confirm_frames,
                       unsigned min_dwell_frames)
{
    memset(s, 0, sizeof(*s));
    if (num_candidates > ARRAY_SELECT_MAX_CANDIDATES) {
        num_candidates = ARRAY_SELECT_MAX_CANDIDATES;
    }
    s->num_candidates = num_candidates;
    s->selected = (initial < num_candidates) ? initial : 0;
    s->challenger = s->selected;
    s->alpha = alpha;
    s->margin = margin;
    s->confirm_frames = confirm_frames;
    s->min_dwell_frames = min_dwell_frames;
    /* Allow an early first decision once scores have settled. */
    s->frames_since_switch = min_dwell_frames;
}

void array_select_force(array_select_t *s, unsigned selected)
{
    if (selected < s->num_candidates && selected != s->selected) {
        s->selected = selected;
        s->challenger = selected;
        s->confirm_count = 0;
        s->frames_since_switch = 0;
        s->switch_count++;
    }
}

int array_select_update(array_select_t *s, const float *raw_scores, int valid)
{
    if (s->frames_since_switch < UINT32_MAX) {
        s->frames_since_switch++;
    }
    if (!valid) {
        return 0;
    }

    if (!s->scores_valid) {
        for (unsigned i = 0; i < s->num_candidates; i++) {
            s->score[i] = raw_scores[i];
        }
        s->scores_valid = 1;
    } else {
        for (unsigned i = 0; i < s->num_candidates; i++) {
            s->score[i] = s->alpha * s->score[i] + (1.0f - s->alpha) * raw_scores[i];
        }
    }

    unsigned best = s->selected;
    for (unsigned i = 0; i < s->num_candidates; i++) {
        if (s->score[i] > s->score[best]) {
            best = i;
        }
    }

    if (best == s->selected || s->score[best] < s->score[s->selected] + s->margin) {
        s->confirm_count = 0;
        s->challenger = s->selected;
        return 0;
    }

    /* A clear winner. Require it to be the same candidate for the whole
     * confirmation period, and respect the minimum dwell time. */
    if (best != s->challenger) {
        s->challenger = best;
        s->confirm_count = 0;
    }
    s->confirm_count++;

    if (s->confirm_count >= s->confirm_frames &&
        s->frames_since_switch >= s->min_dwell_frames) {
        s->selected = best;
        s->confirm_count = 0;
        s->frames_since_switch = 0;
        s->switch_count++;
        return 1;
    }
    return 0;
}
