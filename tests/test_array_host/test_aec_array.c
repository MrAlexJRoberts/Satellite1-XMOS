// Copyright 2026 Future Proof Homes
//
// Host test of the fwk_voice v0.9.0 AEC in the channel configuration selected
// at compile time (AEC_MAX_Y_CHANNELS x AEC_MAX_X_CHANNELS), driven by the
// firmware's own aec_process_frame_1thread().
//
// Scenario: the Satellite1 TAS2780 plays (L + R) / 2 (its default "Mono
// (Left + Right)" downmix, see Satellite1-ESPHome tas2780.cpp). Each
// microphone receives that signal through its own synthetic room impulse
// response plus low-level uncorrelated noise.
//   2Y x 2X (production): AEC gets mics 0,1 and the stereo reference L, R.
//   4Y x 1X (four-mic):   AEC gets mics 0..3 and the mono reference (L+R)/2.
//
// Checks:
//   1. Guard words around aec_state_t are intact after every frame (the state
//      layout really holds the configured channel count).
//   2. Echo return loss enhancement (ERLE) after convergence on every channel.
//   3. (four-channel build) the library's aec_process_frame() with the custom
//      4-channel schedule produces bit-identical output to the firmware driver.
//   4. (four-channel build) the documented limitation: TAS2780 in Left-only
//      mode with uncorrelated L/R content, while the reference is (L+R)/2.

#include <assert.h>
#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include "aec.h"

#if (AEC_LIB_MAX_CHANNELS == 4)
#include "aec_schedule_array.h"
#define TEST_TDIST (&aec_tdist_chans4_threads1)
#else
#define TEST_TDIST (&aec_tdist_chans2_threads1)
#endif

void aec_process_frame_1thread(aec_filter_state_t *main_state,
                               aec_filter_state_t *shadow_state,
                               int32_t (*output_main)[AEC_FRAME_ADVANCE],
                               int32_t (*output_shadow)[AEC_FRAME_ADVANCE],
                               const int32_t (*y_data)[AEC_FRAME_ADVANCE],
                               const int32_t (*x_data)[AEC_FRAME_ADVANCE]);

#define NUM_MICS          4
#define FRAMES            1600   /* 24 s */
#define MEASURE_FRAMES    400    /* last 6 s */
#define RIR_LEN           1200   /* 75 ms, inside the 150 ms main filter */
#define MAIN_PHASES       10
#define SHADOW_PHASES     5
#define CANARY            UINT64_C(0x6e37f491a582c0bd)

typedef struct {
    uint64_t before;
    aec_state_t state;
    uint64_t after;
} guarded_aec_t;

/* --- deterministic signal generation ------------------------------------ */

static uint32_t rng_state = 0x12345678u;
static double rand_uniform(void)
{
    rng_state ^= rng_state << 13;
    rng_state ^= rng_state >> 17;
    rng_state ^= rng_state << 5;
    return (rng_state / 4294967296.0) * 2.0 - 1.0;
}

static double rir[NUM_MICS][RIR_LEN];

static void make_rirs(void)
{
    for (int m = 0; m < NUM_MICS; m++) {
        const int direct = 20 + 3 * m;            /* ~1.3 ms + array spread */
        for (int n = 0; n < RIR_LEN; n++) {
            double v = 0.0;
            if (n == direct) {
                v = 0.5;
            } else if (n > direct) {
                /* exponential decay, ~60 dB over 1200 samples */
                v = 0.15 * rand_uniform() * exp(-6.9 * (n - direct) / (double)RIR_LEN);
            }
            rir[m][n] = v;
        }
    }
}

/* Speech-like loudspeaker content: low-passed noise with a slow syllabic
 * envelope. 'decorrelated' gives L and R independent content (worst case for
 * a mono reference); otherwise R = L (mono content). */
static void make_speaker(double *L, double *R, int n, int decorrelated)
{
    double lpL = 0, lpR = 0;
    for (int i = 0; i < n; i++) {
        const double env = 0.55 + 0.45 * sin(2.0 * M_PI * 3.0 * i / 16000.0);
        lpL = 0.7 * lpL + 0.3 * rand_uniform();
        lpR = 0.7 * lpR + 0.3 * rand_uniform();
        L[i] = 0.25 * env * lpL;
        R[i] = decorrelated ? 0.25 * env * lpR : L[i];
    }
}

static int32_t to_q31(double v)
{
    if (v > 0.999999) v = 0.999999;
    if (v < -0.999999) v = -0.999999;
    return (int32_t)lrint(v * 2147483648.0);
}

/* --- run one scenario ---------------------------------------------------- */

typedef struct {
    double erle_db[NUM_MICS];
    double us_per_frame;
    int canary_ok;
} result_t;

typedef enum { DRIVE_DOWNMIX, DRIVE_LEFT_ONLY } drive_t;

static int32_t DWORD_ALIGNED y_frame[AEC_MAX_Y_CHANNELS][AEC_FRAME_ADVANCE];
static int32_t DWORD_ALIGNED x_frame[AEC_MAX_X_CHANNELS][AEC_FRAME_ADVANCE];
static int32_t DWORD_ALIGNED out_main[AEC_MAX_Y_CHANNELS][AEC_FRAME_ADVANCE];
static int32_t DWORD_ALIGNED out_lib[AEC_MAX_Y_CHANNELS][AEC_FRAME_ADVANCE];

static result_t run(drive_t drive, int decorrelated, guarded_aec_t *g, guarded_aec_t *g_lib,
                    uint64_t *lib_mismatches)
{
    const int total = FRAMES * AEC_FRAME_ADVANCE;
    double *L = calloc(total, sizeof(double));
    double *R = calloc(total, sizeof(double));
    double *drive_sig = calloc(total, sizeof(double));
    assert(L && R && drive_sig);
    rng_state = 0x12345678u;
    make_rirs();
    make_speaker(L, R, total, decorrelated);
    for (int i = 0; i < total; i++) {
        drive_sig[i] = (drive == DRIVE_DOWNMIX) ? 0.5 * (L[i] + R[i]) : L[i];
    }

    memset(g, 0, sizeof(*g));
    g->before = g->after = CANARY;
    aec_init(&g->state, AEC_MAX_Y_CHANNELS, AEC_MAX_X_CHANNELS,
             MAIN_PHASES, SHADOW_PHASES, TEST_TDIST);
    if (g_lib) {
        memset(g_lib, 0, sizeof(*g_lib));
        g_lib->before = g_lib->after = CANARY;
        aec_init(&g_lib->state, AEC_MAX_Y_CHANNELS, AEC_MAX_X_CHANNELS,
                 MAIN_PHASES, SHADOW_PHASES, TEST_TDIST);
    }

    result_t res = {0};
    res.canary_ok = 1;
    double in_e[NUM_MICS] = {0}, out_e[NUM_MICS] = {0};
    double elapsed = 0.0;

    for (int f = 0; f < FRAMES; f++) {
        for (int i = 0; i < AEC_FRAME_ADVANCE; i++) {
            const int n = f * AEC_FRAME_ADVANCE + i;
            for (int m = 0; m < AEC_MAX_Y_CHANNELS; m++) {
                double acc = 0.0;
                for (int k = 0; k < RIR_LEN && k <= n; k++) {
                    acc += rir[m][k] * drive_sig[n - k];
                }
                acc += 1e-4 * rand_uniform();     /* -80 dBFS near-end noise */
                y_frame[m][i] = to_q31(acc);
            }
#if (AEC_MAX_X_CHANNELS == 2)
            x_frame[0][i] = to_q31(L[n]);
            x_frame[1][i] = to_q31(R[n]);
#else
            x_frame[0][i] = to_q31(0.5 * (L[n] + R[n]));
#endif
        }

        int32_t y_copy[AEC_MAX_Y_CHANNELS][AEC_FRAME_ADVANCE];
        int32_t x_copy[AEC_MAX_X_CHANNELS][AEC_FRAME_ADVANCE];
        memcpy(y_copy, y_frame, sizeof(y_copy));
        memcpy(x_copy, x_frame, sizeof(x_copy));

        struct timespec t0, t1;
        clock_gettime(CLOCK_MONOTONIC, &t0);
        aec_process_frame_1thread(&g->state.main_state, &g->state.shadow_state,
                                  out_main, NULL,
                                  (const int32_t (*)[AEC_FRAME_ADVANCE])y_frame,
                                  (const int32_t (*)[AEC_FRAME_ADVANCE])x_frame);
        clock_gettime(CLOCK_MONOTONIC, &t1);
        elapsed += (t1.tv_sec - t0.tv_sec) * 1e6 + (t1.tv_nsec - t0.tv_nsec) / 1e3;

        if (g_lib) {
            aec_process_frame(&g_lib->state, out_lib, NULL, y_copy, x_copy);
            if (memcmp(out_lib, out_main, sizeof(out_main)) != 0) {
                (*lib_mismatches)++;
            }
            if (g_lib->before != CANARY || g_lib->after != CANARY) {
                res.canary_ok = 0;
            }
        }
        if (g->before != CANARY || g->after != CANARY) {
            res.canary_ok = 0;
        }

        if (f >= FRAMES - MEASURE_FRAMES) {
            /* AEC output is delayed relative to its input by the overlap of the
             * 512-sample processing frame, so compare energies over the window
             * rather than sample by sample. */
            for (int m = 0; m < AEC_MAX_Y_CHANNELS; m++) {
                for (int i = 0; i < AEC_FRAME_ADVANCE; i++) {
                    in_e[m] += (double)y_copy[m][i] * y_copy[m][i];
                    out_e[m] += (double)out_main[m][i] * out_main[m][i];
                }
            }
        }
    }

    for (int m = 0; m < AEC_MAX_Y_CHANNELS; m++) {
        res.erle_db[m] = 10.0 * log10(in_e[m] / (out_e[m] + 1.0));
    }
    res.us_per_frame = elapsed / FRAMES;
    free(L); free(R); free(drive_sig);
    return res;
}

static void print_result(const char *name, const result_t *r)
{
    printf("  %-44s ERLE:", name);
    for (int m = 0; m < AEC_MAX_Y_CHANNELS; m++) {
        printf(" ch%d %5.1f dB", m, r->erle_db[m]);
    }
    printf("   host %.0f us/frame\n", r->us_per_frame);
}

int main(void)
{
    printf("AEC %dY x %dX: sizeof(aec_state_t) = %zu bytes (host ABI)\n",
           AEC_MAX_Y_CHANNELS, AEC_MAX_X_CHANNELS, sizeof(aec_state_t));

    guarded_aec_t *g = malloc(sizeof(*g));
    guarded_aec_t *g_lib = NULL;
    uint64_t lib_mismatches = 0;
#if (AEC_LIB_MAX_CHANNELS == 4)
    g_lib = malloc(sizeof(*g_lib));
#endif
    assert(g);
    int failures = 0;

    /* Mono speaker content, default TAS2780 downmix: the case both
     * configurations must handle. */
    result_t r = run(DRIVE_DOWNMIX, 0, g, g_lib, &lib_mismatches);
    print_result("TAS downmix, mono content", &r);
    for (int m = 0; m < AEC_MAX_Y_CHANNELS; m++) {
        if (!(r.erle_db[m] > 20.0)) {
            printf("  FAIL: ch%d ERLE %.1f dB <= 20 dB\n", m, r.erle_db[m]);
            failures++;
        }
    }
    failures += !r.canary_ok;

    /* Stereo content through the default downmix. A (L+R)/2 reference is an
     * exact model of what the TAS2780 plays, so this must also converge. */
    r = run(DRIVE_DOWNMIX, 1, g, NULL, &lib_mismatches);
    print_result("TAS downmix, decorrelated stereo content", &r);
    for (int m = 0; m < AEC_MAX_Y_CHANNELS; m++) {
        if (!(r.erle_db[m] > 20.0)) {
            printf("  FAIL: ch%d ERLE %.1f dB <= 20 dB\n", m, r.erle_db[m]);
            failures++;
        }
    }
    failures += !r.canary_ok;

#if (AEC_MAX_X_CHANNELS == 1)
    /* Left-only amplifier mode with mono content (TTS, most announcements):
     * L == R, so the (L+R)/2 reference is still exact. */
    r = run(DRIVE_LEFT_ONLY, 0, g, NULL, &lib_mismatches);
    print_result("TAS left-only, mono content", &r);
    for (int m = 0; m < AEC_MAX_Y_CHANNELS; m++) {
        if (!(r.erle_db[m] > 20.0)) {
            printf("  FAIL: ch%d ERLE %.1f dB <= 20 dB\n", m, r.erle_db[m]);
            failures++;
        }
    }
    failures += !r.canary_ok;

    /* Known limitation, reported not asserted: speaker in Left-only mode
     * with uncorrelated stereo content while the reference is (L+R)/2. */
    r = run(DRIVE_LEFT_ONLY, 1, g, NULL, &lib_mismatches);
    print_result("LIMITATION: TAS left-only, stereo content", &r);
    failures += !r.canary_ok;
#endif

    if (g_lib) {
        printf("  library aec_process_frame() vs firmware driver: %llu of %d frames differ\n",
               (unsigned long long)lib_mismatches, FRAMES);
        if (lib_mismatches != 0) {
            failures++;
        }
    }

    printf("  guard words: %s\n", failures ? "see above" : "intact");
    free(g);
    free(g_lib);
    printf("%s\n", failures ? "FAILED" : "PASSED");
    return failures ? 1 : 0;
}
