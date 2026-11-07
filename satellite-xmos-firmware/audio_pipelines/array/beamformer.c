// Copyright 2026 Future Proof Homes

#include <math.h>
#include <string.h>
#include "beamformer.h"
#include "array_stats.h"

#define BF_PI   (3.14159265358979323846f)

/* Each beam is the mean of ARRAY_MIC_COUNT delayed inputs. Scaling every term
 * by 1/4 before summing keeps the int32 accumulator in range; the 2 LSBs lost
 * are at about -180 dBFS. */
#define BF_MIC_SHIFT   (2)
_Static_assert((1 << BF_MIC_SHIFT) == ARRAY_MIC_COUNT, "beam averaging shift assumes 4 microphones");

/* Pre-emphasis energy: the first difference of int32 samples needs 33 bits;
 * scaling by 2^-5 before squaring keeps a 240-sample sum below 2^62. */
#define BF_DIFF_SHIFT  (5)

int array_beamformer_init(array_beamformer_t *bf)
{
    memset(bf, 0, sizeof(*bf));

    /* r * fs / c in samples: 32.08 mm * 16 kHz / 343 m/s = 1.4964 */
    const float a = ((float)ARRAY_RADIUS_UM * 1e-3f) * (float)ARRAY_SAMPLE_RATE_HZ
                    / (float)ARRAY_SPEED_OF_SOUND_MM_S;

    for (unsigned c = 0; c < ARRAY_DELAY_CLASSES; c++) {
        float offset = a * cosf(2.0f * BF_PI * (float)c / (float)ARRAY_ANGLE_STEPS);
        /* cos(90 degrees) is not exactly zero in float; make that class an
         * exact integer delay. */
        if (fabsf(offset) < 1e-4f) {
            offset = 0.0f;
        }
        bf->offset_samples[c] = offset;
        bf->is_pure_delay[c] = (offset == 0.0f);
        if (array_fd_design(&bf->fd[c], offset) != 0) {
            return -1;
        }
    }

    const int8_t mic_steps[ARRAY_MIC_COUNT] = ARRAY_MIC_ANGLE_STEPS_INIT;
    for (unsigned k = 0; k < ARRAY_BEAM_COUNT; k++) {
        for (unsigned m = 0; m < ARRAY_MIC_COUNT; m++) {
            int d = ((int)k - (int)mic_steps[m]) % ARRAY_ANGLE_STEPS;
            if (d < 0) {
                d += ARRAY_ANGLE_STEPS;
            }
            /* cos(d * 45deg) == cos((8 - d) * 45deg) */
            const int cls = (d <= ARRAY_ANGLE_STEPS / 2) ? d : (ARRAY_ANGLE_STEPS - d);
            bf->class_of[k][m] = (uint8_t)cls;
        }
    }
    return 0;
}

void array_beamformer_process(array_beamformer_t *bf,
                              const int32_t (*mics)[ARRAY_FRAME_ADVANCE],
                              int32_t (*beams)[ARRAY_FRAME_ADVANCE],
                              float *beam_energy_db)
{
    int32_t x_ext[ARRAY_FD_HISTORY + ARRAY_FRAME_ADVANCE];
    int32_t delayed[ARRAY_FRAME_ADVANCE];

    memset(beams, 0, ARRAY_BEAM_COUNT * sizeof(beams[0]));

    for (unsigned m = 0; m < ARRAY_MIC_COUNT; m++) {
        memcpy(&x_ext[0], bf->hist[m], sizeof(bf->hist[m]));
        memcpy(&x_ext[ARRAY_FD_HISTORY], mics[m], sizeof(mics[m]));

        for (unsigned c = 0; c < ARRAY_DELAY_CLASSES; c++) {
            /* Skip delay classes no beam uses for this microphone. */
            int used = 0;
            for (unsigned k = 0; k < ARRAY_BEAM_COUNT; k++) {
                used |= (bf->class_of[k][m] == c);
            }
            if (!used) {
                continue;
            }

            if (bf->is_pure_delay[c]) {
                memcpy(delayed, &x_ext[ARRAY_FD_HISTORY - ARRAY_FD_BULK_DELAY],
                       sizeof(delayed));
            } else {
                array_fd_apply(&bf->fd[c], x_ext, delayed, ARRAY_FRAME_ADVANCE);
            }

            for (unsigned k = 0; k < ARRAY_BEAM_COUNT; k++) {
                if (bf->class_of[k][m] != c) {
                    continue;
                }
                int32_t *b = beams[k];
                for (unsigned i = 0; i < ARRAY_FRAME_ADVANCE; i++) {
                    b[i] += delayed[i] >> BF_MIC_SHIFT;
                }
            }
        }

        memcpy(bf->hist[m], &x_ext[ARRAY_FRAME_ADVANCE], sizeof(bf->hist[m]));
    }

    for (unsigned k = 0; k < ARRAY_BEAM_COUNT; k++) {
        const int32_t *b = beams[k];
        if (beam_energy_db != NULL) {
            int64_t acc = 0;
            int32_t prev = bf->prev_last[k];
            for (unsigned i = 0; i < ARRAY_FRAME_ADVANCE; i++) {
                const int32_t d = (int32_t)(((int64_t)b[i] - prev) >> BF_DIFF_SHIFT);
                acc += (int64_t)d * d;
                prev = b[i];
            }
            /* Relative to int32 full scale: (2^31 >> 5)^2 = 2^52. */
            const float e = ldexpf((float)acc, -2 * (31 - BF_DIFF_SHIFT)) / (float)ARRAY_FRAME_ADVANCE;
            beam_energy_db[k] = array_power_to_db(e);
        }
        bf->prev_last[k] = b[ARRAY_FRAME_ADVANCE - 1];
    }
}
