// Copyright 2026 Future Proof Homes

#include "aec_schedule_array.h"

#if (AEC_LIB_MAX_CHANNELS == 4)

/* Same channel-major ordering as fwk_voice's aec_tdist_chans2_threads1. */
const aec_task_distribution_t aec_tdist_chans4_threads1 = {
    .thread_count = 1,
    .passes_for_3_tasks_and_channels = 12,
    .par_3_tasks_and_channels = {
        {{0, 0, 1}, {1, 0, 1}, {2, 0, 1},
         {0, 1, 1}, {1, 1, 1}, {2, 1, 1},
         {0, 2, 1}, {1, 2, 1}, {2, 2, 1},
         {0, 3, 1}, {1, 3, 1}, {2, 3, 1}},
    },
    .passes_for_2_tasks_and_channels = 8,
    .par_2_tasks_and_channels = {
        {{0, 0, 1}, {1, 0, 1},
         {0, 1, 1}, {1, 1, 1},
         {0, 2, 1}, {1, 2, 1},
         {0, 3, 1}, {1, 3, 1}},
    },
    .passes_for_1_tasks_and_channels = 4,
    .par_1_tasks_and_channels = {
        {{0, 0, 1}, {0, 1, 1}, {0, 2, 1}, {0, 3, 1}},
    },
    .passes_for_3_tasks = 3,
    .par_3_tasks = {
        {{0, 1}, {1, 1}, {2, 1}},
    },
    .passes_for_2_tasks = 2,
    .par_2_tasks = {
        {{0, 1}, {1, 1}},
    },
};

#else
/* Built only into four-channel AEC configurations. */
typedef int aec_schedule_array_unused_t;
#endif
