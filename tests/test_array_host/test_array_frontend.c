// Copyright 2026 Future Proof Homes
//
// Host unit tests for the Satellite1 array front end (audio_pipelines/array).
// Plane waves are synthesised analytically (exact fractional delays from the
// hardware geometry), so the tests check the DSP against the physics, not
// against itself.

#include <math.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "array_config.h"
#include "array_reference.h"
#include "array_select.h"
#include "array_stats.h"
#include "beamformer.h"
#include "crossfade.h"
#include "fractional_delay.h"

#define FS          ((double)ARRAY_SAMPLE_RATE_HZ)
#define C_SOUND     ((double)ARRAY_SPEED_OF_SOUND_MM_S * 1e-3)
#define RADIUS_M    ((double)ARRAY_RADIUS_UM * 1e-6)
#define FRAMES      40
#define N           (FRAMES * ARRAY_FRAME_ADVANCE)

static int failures;
#define CHECK(cond, ...) do { if (!(cond)) { failures++; printf("  FAIL %s:%d: ", __FILE__, __LINE__); printf(__VA_ARGS__); printf("\n"); } } while (0)

static const int mic_steps[ARRAY_MIC_COUNT] = ARRAY_MIC_ANGLE_STEPS_INIT;

static double mic_angle_rad(int m) { return 2.0 * M_PI * mic_steps[m] / ARRAY_ANGLE_STEPS; }

/* Time (in samples) by which microphone m leads the array centre for a
 * far-field source at azimuth theta. */
static double lead_samples(int m, double theta)
{
    return RADIUS_M * cos(theta - mic_angle_rad(m)) / C_SOUND * FS;
}

static int32_t q31(double v)
{
    if (v > 0.999999999) v = 0.999999999;
    if (v < -1.0) v = -1.0;
    return (int32_t)llround(v * 2147483648.0);
}

/* Multi-tone source: a sum of tones, so any delay can be applied exactly. */
typedef struct { int n; double f[64], a[64], ph[64]; } tones_t;

static double tones_at(const tones_t *t, double time_samples)
{
    double v = 0;
    for (int i = 0; i < t->n; i++) {
        v += t->a[i] * sin(2.0 * M_PI * t->f[i] / FS * time_samples + t->ph[i]);
    }
    return v;
}

static void make_mics(const tones_t *t, double theta, int32_t (*mics)[N])
{
    for (int m = 0; m < ARRAY_MIC_COUNT; m++) {
        const double lead = lead_samples(m, theta);
        for (int n = 0; n < N; n++) {
            mics[m][n] = q31(tones_at(t, n + lead));
        }
    }
}

static double rms(const int32_t *x, int n)
{
    double acc = 0;
    for (int i = 0; i < n; i++) acc += (double)x[i] * x[i];
    return sqrt(acc / n) / 2147483648.0;
}

/* Analytical DAS gain of a beam looking at theta_look for a tone at f from
 * theta_src (what the fixed-point beamformer should reproduce). */
static double das_model_gain(double f, double theta_look, double theta_src)
{
    double re = 0, im = 0;
    for (int m = 0; m < ARRAY_MIC_COUNT; m++) {
        const double d = (lead_samples(m, theta_src) - lead_samples(m, theta_look)) / FS;
        re += cos(2 * M_PI * f * d);
        im += sin(2 * M_PI * f * d);
    }
    return sqrt(re * re + im * im) / ARRAY_MIC_COUNT;
}

/* ------------------------------------------------------------------------ */

static void test_fractional_delay(void)
{
    printf("fractional delay filters\n");
    const double offsets[] = { -1.4965, -1.0582, -0.5, 0.0, 0.25, 1.0582, 1.4965 };
    const double freqs[] = { 250, 1000, 3000, 5000, 6000 };
    double worst_mag = 0, worst_ph = 0;

    for (unsigned o = 0; o < sizeof(offsets) / sizeof(offsets[0]); o++) {
        array_fd_filter_t f;
        CHECK(array_fd_design(&f, (float)offsets[o]) == 0, "design failed for %f", offsets[o]);
        int64_t sum = 0;
        for (int k = 0; k < ARRAY_FD_TAPS; k++) sum += f.coef[k];
        CHECK(sum == ((int64_t)1 << ARRAY_FD_COEF_FRAC_BITS), "DC gain not exactly 1 at offset %f", offsets[o]);

        if (offsets[o] == 0.0) {
            for (int k = 0; k < ARRAY_FD_TAPS; k++) {
                const int32_t want = (k == ARRAY_FD_BULK_DELAY) ? (1 << ARRAY_FD_COEF_FRAC_BITS) : 0;
                CHECK(f.coef[k] == want, "offset 0 is not a pure delay (tap %d = %d)", k, (int)f.coef[k]);
            }
        }

        /* Measure gain and phase by correlating the output with the ideally
         * delayed tone. */
        for (unsigned fi = 0; fi < sizeof(freqs) / sizeof(freqs[0]); fi++) {
            static int32_t x_ext[ARRAY_FD_HISTORY + N];
            static int32_t y[N];
            const double w = 2 * M_PI * freqs[fi] / FS;
            const double delay = ARRAY_FD_BULK_DELAY + offsets[o];
            for (int n = 0; n < ARRAY_FD_HISTORY + N; n++) {
                x_ext[n] = q31(0.5 * sin(w * (n - ARRAY_FD_HISTORY)));
            }
            array_fd_apply(&f, x_ext, y, N);
            double c = 0, s = 0;
            for (int n = 200; n < N; n++) {
                const double ph = w * (n - delay);
                c += (y[n] / 2147483648.0) * sin(ph);
                s += (y[n] / 2147483648.0) * cos(ph);
            }
            const double amp = 2.0 * sqrt(c * c + s * s) / (N - 200) / 0.5;
            const double ph_err = fabs(atan2(s, c)) * 180.0 / M_PI;
            const double mag_db = fabs(20 * log10(amp));
            if (mag_db > worst_mag) worst_mag = mag_db;
            if (ph_err > worst_ph) worst_ph = ph_err;
        }
    }
    printf("  worst |gain error| %.3f dB, worst phase error %.3f deg (250 Hz - 6 kHz, |offset| <= 1.4965)\n",
           worst_mag, worst_ph);
    CHECK(worst_mag < 0.30, "gain error %.3f dB exceeds design spec", worst_mag);
    CHECK(worst_ph < 1.0, "phase error %.3f deg exceeds design spec", worst_ph);
    CHECK(array_fd_design(&(array_fd_filter_t){0}, 4.5f) != 0, "out-of-range offset accepted");
}

static void test_beam_patterns(void)
{
    printf("delay-and-sum beam bank vs analytical model\n");
    static int32_t mics[ARRAY_MIC_COUNT][N];
    static int32_t beams_all[ARRAY_BEAM_COUNT][N];
    const double freqs[] = { 500, 1000, 2000, 3000, 4000, 6000 };
    double worst_look = 0, worst_model = 0;

    for (unsigned fi = 0; fi < sizeof(freqs) / sizeof(freqs[0]); fi++) {
        for (int src = 0; src < 16; src++) {      /* sources every 22.5 degrees */
            const double theta = 2 * M_PI * src / 16.0;
            tones_t t = { .n = 1, .f = { freqs[fi] }, .a = { 0.4 }, .ph = { 0.3 } };
            make_mics(&t, theta, mics);

            array_beamformer_t bf;
            CHECK(array_beamformer_init(&bf) == 0, "beamformer init");
            for (int fr = 0; fr < FRAMES; fr++) {
                int32_t in[ARRAY_MIC_COUNT][ARRAY_FRAME_ADVANCE];
                int32_t out[ARRAY_BEAM_COUNT][ARRAY_FRAME_ADVANCE];
                for (int m = 0; m < ARRAY_MIC_COUNT; m++)
                    memcpy(in[m], &mics[m][fr * ARRAY_FRAME_ADVANCE], sizeof(in[m]));
                array_beamformer_process(&bf, (const int32_t (*)[ARRAY_FRAME_ADVANCE])in, out, NULL);
                for (int k = 0; k < ARRAY_BEAM_COUNT; k++)
                    memcpy(&beams_all[k][fr * ARRAY_FRAME_ADVANCE], out[k], sizeof(out[k]));
            }
            const double in_rms = rms(&mics[0][ARRAY_FRAME_ADVANCE], N - ARRAY_FRAME_ADVANCE);
            for (int k = 0; k < ARRAY_BEAM_COUNT; k++) {
                const double look = 2 * M_PI * k / ARRAY_BEAM_COUNT;
                const double g = rms(&beams_all[k][ARRAY_FRAME_ADVANCE], N - ARRAY_FRAME_ADVANCE) / in_rms;
                const double model = das_model_gain(freqs[fi], look, theta);
                const double g_db = 20 * log10(g + 1e-9), model_db = 20 * log10(model + 1e-9);
                if (src == 2 * k) {        /* source on this beam's axis */
                    if (fabs(g_db) > worst_look) worst_look = fabs(g_db);
                }
                /* Compare where the model is above -25 dB (deep nulls are
                 * dominated by the 0.3 dB filter ripple and quantisation). */
                if (model_db > -25.0 && fabs(g_db - model_db) > worst_model) {
                    worst_model = fabs(g_db - model_db);
                }
            }
        }
    }
    printf("  on-axis gain error worst %.3f dB; deviation from DAS model worst %.3f dB (above -25 dB)\n",
           worst_look, worst_model);
    CHECK(worst_look < 0.35, "look-direction gain error %.3f dB", worst_look);
    CHECK(worst_model < 0.6, "beam pattern deviates from model by %.3f dB", worst_model);
}

/* Speech-like multi-tone (tilt -6 dB/octave above 500 Hz) for selection. */
static void make_speechlike(tones_t *t, unsigned seed)
{
    srand(seed);
    t->n = 48;
    for (int i = 0; i < t->n; i++) {
        t->f[i] = 150.0 + i * 145.0 + (rand() % 40);
        t->a[i] = 0.05 / sqrt(1.0 + pow(t->f[i] / 500.0, 2.0));
        t->ph[i] = 2 * M_PI * (rand() / (double)RAND_MAX);
    }
}

static void test_beam_selection_energy(void)
{
    printf("beam selection energy (pre-emphasised)\n");
    static int32_t mics[ARRAY_MIC_COUNT][N];
    int correct = 0, adjacent_ok = 0, tested_between = 0;
    double min_margin = 1e9;

    for (int src = 0; src < 16; src++) {
        const double theta = 2 * M_PI * src / 16.0;
        tones_t t;
        make_speechlike(&t, 1000u + (unsigned)src);
        make_mics(&t, theta, mics);
        array_beamformer_t bf;
        array_beamformer_init(&bf);
        double e_acc[ARRAY_BEAM_COUNT] = {0};
        for (int fr = 0; fr < FRAMES; fr++) {
            int32_t in[ARRAY_MIC_COUNT][ARRAY_FRAME_ADVANCE];
            int32_t out[ARRAY_BEAM_COUNT][ARRAY_FRAME_ADVANCE];
            float e[ARRAY_BEAM_COUNT];
            for (int m = 0; m < ARRAY_MIC_COUNT; m++)
                memcpy(in[m], &mics[m][fr * ARRAY_FRAME_ADVANCE], sizeof(in[m]));
            array_beamformer_process(&bf, (const int32_t (*)[ARRAY_FRAME_ADVANCE])in, out, e);
            if (fr > 0) for (int k = 0; k < ARRAY_BEAM_COUNT; k++) e_acc[k] += e[k];
        }
        int best = 0;
        for (int k = 1; k < ARRAY_BEAM_COUNT; k++) if (e_acc[k] > e_acc[best]) best = k;
        if (src % 2 == 0) {
            const int want = src / 2;
            correct += (best == want);
            double second = -1e9;
            for (int k = 0; k < ARRAY_BEAM_COUNT; k++) if (k != want && e_acc[k] > second) second = e_acc[k];
            const double margin = (e_acc[want] - second) / (FRAMES - 1);
            if (margin < min_margin) min_margin = margin;
        } else {
            tested_between++;
            const int lo = src / 2, hi = (src / 2 + 1) % ARRAY_BEAM_COUNT;
            adjacent_ok += (best == lo || best == hi);
        }
    }
    printf("  on-axis sources: %d/8 select the right beam, min margin over next beam %.2f dB\n",
           correct, min_margin);
    printf("  between-beam sources: %d/%d select one of the two neighbouring beams\n",
           adjacent_ok, tested_between);
    CHECK(correct == 8, "beam selection wrong for on-axis sources");
    CHECK(adjacent_ok == tested_between, "between-beam sources selected a non-neighbour beam");
    CHECK(min_margin > ARRAY_BEAM_SWITCH_MARGIN_DB, "on-axis margin %.2f dB below switching margin", min_margin);
}

static void test_naive_average_vs_das(void)
{
    printf("naive 4-mic average (experiment branch) vs steered DAS\n");
    static int32_t mics[ARRAY_MIC_COUNT][N];
    const struct { double theta_deg, f; } cases[] = {
        { 0, 3000 }, { 0, 4000 }, { 0, 5346 }, { 45, 3000 }, { 45, 3780 },
    };
    for (unsigned c = 0; c < sizeof(cases) / sizeof(cases[0]); c++) {
        const double theta = cases[c].theta_deg * M_PI / 180.0;
        tones_t t = { .n = 1, .f = { cases[c].f }, .a = { 0.4 }, .ph = { 0.1 } };
        make_mics(&t, theta, mics);
        static int32_t avg[N];
        for (int n = 0; n < N; n++) {
            int64_t s = 0;
            for (int m = 0; m < ARRAY_MIC_COUNT; m++) s += mics[m][n];
            avg[n] = (int32_t)(s / ARRAY_MIC_COUNT);
        }
        array_beamformer_t bf;
        array_beamformer_init(&bf);
        static int32_t look[N];
        const int k = (int)lround(cases[c].theta_deg / 45.0) % ARRAY_BEAM_COUNT;
        for (int fr = 0; fr < FRAMES; fr++) {
            int32_t in[ARRAY_MIC_COUNT][ARRAY_FRAME_ADVANCE];
            int32_t out[ARRAY_BEAM_COUNT][ARRAY_FRAME_ADVANCE];
            for (int m = 0; m < ARRAY_MIC_COUNT; m++)
                memcpy(in[m], &mics[m][fr * ARRAY_FRAME_ADVANCE], sizeof(in[m]));
            array_beamformer_process(&bf, (const int32_t (*)[ARRAY_FRAME_ADVANCE])in, out, NULL);
            memcpy(&look[fr * ARRAY_FRAME_ADVANCE], out[k], sizeof(out[k]));
        }
        const double ref = rms(mics[0], N);
        const double g_avg = 20 * log10(rms(avg, N) / ref + 1e-12);
        const double g_das = 20 * log10(rms(&look[ARRAY_FRAME_ADVANCE], N - ARRAY_FRAME_ADVANCE) / ref);
        printf("  talker at %3.0f deg, %4.0f Hz: naive average %7.1f dB, DAS beam %d %5.2f dB\n",
               cases[c].theta_deg, cases[c].f, g_avg, k, g_das);
        CHECK(fabs(g_das) < 0.35, "DAS look gain %.2f dB", g_das);
    }
}

static void test_selector(void)
{
    printf("hysteretic selector\n");
    array_select_t s;
    array_select_init(&s, 3, 0, 0.0f /* no smoothing: raw scores */, 1.0f, 3, 5);
    float sc[3] = { 0.0f, 0.5f, 0.0f };

    for (int i = 0; i < 10; i++) array_select_update(&s, sc, 1);
    CHECK(s.selected == 0 && s.switch_count == 0, "switched on a sub-margin challenger");

    sc[1] = 2.0f;
    array_select_update(&s, sc, 1);
    array_select_update(&s, sc, 1);
    CHECK(s.selected == 0, "switched before the confirmation period");
    array_select_update(&s, sc, 1);
    CHECK(s.selected == 1 && s.switch_count == 1, "did not switch after confirmation");

    /* Immediately prefer candidate 2: dwell time must hold the selection. */
    sc[1] = 0.0f; sc[2] = 5.0f;
    for (int i = 0; i < 3; i++) array_select_update(&s, sc, 1);
    CHECK(s.selected == 1, "minimum dwell not respected");
    for (int i = 0; i < 3; i++) array_select_update(&s, sc, 1);
    CHECK(s.selected == 2, "did not switch after the dwell time");

    /* Invalid frames hold the selection whatever the scores. */
    sc[0] = 100.0f;
    for (int i = 0; i < 20; i++) array_select_update(&s, sc, 0);
    CHECK(s.selected == 2, "selection changed on invalid frames");

    /* A challenger that drops out resets the confirmation count. */
    array_select_t t;
    array_select_init(&t, 2, 0, 0.0f, 1.0f, 3, 0);
    float a[2] = { 0, 2 }, b[2] = { 0, 0 };
    array_select_update(&t, a, 1);
    array_select_update(&t, a, 1);
    array_select_update(&t, b, 1);
    array_select_update(&t, a, 1);
    array_select_update(&t, a, 1);
    CHECK(t.selected == 0, "confirmation not reset by an interruption");
    array_select_update(&t, a, 1);
    CHECK(t.selected == 1, "did not switch after an uninterrupted confirmation");
}

static void test_crossfade_reference_stats(void)
{
    printf("crossfade, reference mix, statistics\n");
    int32_t from[ARRAY_FRAME_ADVANCE], to[ARRAY_FRAME_ADVANCE], out[ARRAY_FRAME_ADVANCE];
    for (int i = 0; i < ARRAY_FRAME_ADVANCE; i++) { from[i] = INT32_MIN; to[i] = INT32_MAX; }
    array_crossfade(from, to, out, ARRAY_FRAME_ADVANCE);
    int monotonic = 1;
    for (int i = 1; i < ARRAY_FRAME_ADVANCE; i++) monotonic &= (out[i] >= out[i - 1]);
    CHECK(monotonic, "crossfade not monotonic between extremes");
    CHECK(out[ARRAY_FRAME_ADVANCE - 1] == INT32_MAX, "crossfade does not end on the new source");
    CHECK(out[0] < INT32_MIN / 2, "crossfade starts away from the old source");

    int32_t l[4] = { INT32_MAX, INT32_MIN, 1000, -7 }, r[4] = { INT32_MAX, INT32_MIN, -1000, 3 }, m[4];
    array_reference_mix(l, r, m, 4, ARRAY_REF_MIX_DOWNMIX);
    CHECK(m[0] == INT32_MAX && m[1] == INT32_MIN && m[2] == 0 && m[3] == -2, "downmix wrong (%d %d %d %d)",
          (int)m[0], (int)m[1], (int)m[2], (int)m[3]);
    array_reference_mix(l, r, m, 4, ARRAY_REF_MIX_LEFT);
    CHECK(memcmp(m, l, sizeof(m)) == 0, "left mix wrong");
    array_reference_mix(l, r, m, 4, ARRAY_REF_MIX_RIGHT);
    CHECK(memcmp(m, r, sizeof(m)) == 0, "right mix wrong");

    array_stats_t st;
    array_stats_init(&st, 2);
    int32_t frames[2][ARRAY_FRAME_ADVANCE];
    for (int fr = 0; fr < 100; fr++) {
        for (int i = 0; i < ARRAY_FRAME_ADVANCE; i++) {
            frames[0][i] = q31(0.999999 * sin(2 * M_PI * 1000.0 * (fr * ARRAY_FRAME_ADVANCE + i) / FS));
            frames[1][i] = (i == 5) ? INT32_MIN : 0;
        }
        array_stats_update(&st, (const int32_t (*)[ARRAY_FRAME_ADVANCE])frames);
    }
    const float r0 = array_stats_rms_dbfs(&st, 0);
    printf("  full-scale 1 kHz sine: %.3f dBFS (expect -3.010); clips counted %u\n", r0, st.clip_count[0]);
    CHECK(fabsf(r0 + 3.0103f) < 0.05f, "sine RMS %.3f dBFS", r0);
    CHECK(st.clip_count[1] == 100 && st.peak_abs[1] == INT32_MAX, "INT32_MIN clip/peak handling");
    array_stats_reset_peaks(&st);
    CHECK(st.peak_abs[0] == 0, "peak reset");
}

static void test_geometry_numbers(void)
{
    printf("geometry (from Satellite1-Hardware STEP models)\n");
    const double adj = RADIUS_M * sqrt(2.0), opp = 2 * RADIUS_M;
    printf("  radius %.2f mm, adjacent %.2f mm, opposite %.2f mm\n", RADIUS_M * 1e3, adj * 1e3, opp * 1e3);
    printf("  max inter-mic delay %.1f us = %.3f samples @16 kHz; r/c = %.4f samples\n",
           opp / C_SOUND * 1e6, opp / C_SOUND * FS, RADIUS_M / C_SOUND * FS);
    printf("  spatial aliasing above c/(2d): adjacent %.0f Hz, opposite %.0f Hz\n",
           C_SOUND / (2 * adj), C_SOUND / (2 * opp));
    CHECK(fabs(opp / C_SOUND * FS - 2.993) < 0.01, "opposite-pair delay");
}

int main(void)
{
    test_geometry_numbers();
    test_fractional_delay();
    test_beam_patterns();
    test_beam_selection_energy();
    test_naive_average_vs_das();
    test_selector();
    test_crossfade_reference_stats();
    printf("%s (%d failures)\n", failures ? "FAILED" : "PASSED", failures);
    return failures ? 1 : 0;
}
