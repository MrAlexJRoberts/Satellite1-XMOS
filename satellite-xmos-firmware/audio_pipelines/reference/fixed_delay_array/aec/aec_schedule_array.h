// Copyright 2026 Future Proof Homes
//
// AEC task distribution for the four-microphone pipeline.
//
// fwk_voice v0.9.0 only ships default schedules when
// max(AEC_MAX_Y_CHANNELS, AEC_MAX_X_CHANNELS) == 2 (aec_schedule_defaults.c),
// and aec_init() asserts that a schedule is supplied. The firmware drives the
// AEC with aec_process_frame_1thread(), which does not consult the schedule,
// but this table is a complete, valid single-thread schedule for up to four
// channels so that the library's aec_process_frame() is also correct with it.

#ifndef AEC_SCHEDULE_ARRAY_H_
#define AEC_SCHEDULE_ARRAY_H_

#include "aec_schedule.h"

/* Single hardware thread, every (task, channel) pair run in sequence.
 * aec_process_frame() skips channels >= the configured channel count, so the
 * same table is valid for 4Y x 1X and 4Y x 2X. */
extern const aec_task_distribution_t aec_tdist_chans4_threads1;

#endif /* AEC_SCHEDULE_ARRAY_H_ */
