// Copyright 2022-2024 XMOS LIMITED.
// Copyright 2026 Future Proof Homes
// This Software is subject to the terms of the XMOS Public Licence: Version 1.
//
// Four-microphone fixed_delay pipeline, tile 0: array front end, IC, VNR, NS,
// AGC.
//
// fwk_voice's interference canceller (IC) is a fixed two-channel block: one
// target (y) and one noise reference (x), "NOT USER MODIFIABLE" (ic_defines.h),
// tuned for a ~71 mm microphone pair. The front end therefore reduces the four
// AEC outputs to an IC input pair without ever summing microphones that are
// not time-aligned:
//
//   ARRAY_FRONTEND_PAIR_SELECT (four-mic baseline)
//     Two complete IC + VNR paths run every frame, one per diametrically
//     opposite pair: A = (E, W), the production pair, and B = (N, S). Both
//     are 64.2 mm pairs, the geometry IC already runs on today. The output is
//     the path whose smoothed output VNR is clearly higher during speech, with
//     hysteresis and a one-frame crossfade on a change. Both paths stay
//     converged, so switching causes no adaptation transient.
//
//   ARRAY_FRONTEND_BEAM (four-mic beamformer)
//     Eight fixed delay-and-sum beams (beamformer.h). The beam with the most
//     pre-emphasised energy during speech becomes the IC target and the
//     opposite beam the IC noise reference.
//
// NS, AGC and the two-lane output to the ESP32 are unchanged.

/* STD headers */
#include <string.h>
#include <stdint.h>
#include <xassert.h>
#include <xcore/hwtimer.h>

/* FreeRTOS headers */
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "queue.h"
#include "stream_buffer.h"

/* Library headers */
#include "generic_pipeline.h"

/* App headers */
#include "app_conf.h"
#include "platform/platform_conf.h"
#include "audio_pipeline.h"
#include "audio_pipeline_dsp.h"
#include "array_diag.h"
#include "array_select.h"
#include "array_stats.h"
#include "beamformer.h"
#include "crossfade.h"

#if appconfAUDIO_PIPELINE_FRAME_ADVANCE != 240
#error This pipeline is only configured for 240 frame advance
#endif

#if appconfAUDIO_PIPELINE_SKIP_IC_AND_VNR
#error "The array front end feeds IC; appconfAUDIO_PIPELINE_SKIP_IC_AND_VNR is not supported"
#endif

#if (appconfARRAY_FRONTEND_MODE != ARRAY_FRONTEND_PAIR_SELECT) && \
    (appconfARRAY_FRONTEND_MODE != ARRAY_FRONTEND_BEAM)
#error "Unknown appconfARRAY_FRONTEND_MODE"
#endif

#define VNR_AGC_THRESHOLD (0.5)

#define AP_REF_CLOCK_HZ   (100000000)
#define AP_FRAME_TICKS    (appconfAUDIO_PIPELINE_FRAME_ADVANCE * (AP_REF_CLOCK_HZ / appconfAUDIO_PIPELINE_SAMPLE_RATE))

/* Priority of the diagnostics servicer: same as the other servicers. */
#define AP_DIAG_SERVICER_PRIORITY  (appconfDEVICE_CONTROL_SPI_PRIORITY)

#if ON_TILE(0)

#if appconfARRAY_FRONTEND_MODE == ARRAY_FRONTEND_PAIR_SELECT
#define AP_IC_PATHS (ARRAY_PAIR_COUNT)
#else
#define AP_IC_PATHS (1)
#endif

static ic_stage_ctx_t DWORD_ALIGNED ic_stage_state[AP_IC_PATHS] = {};
static vnr_pred_stage_ctx_t DWORD_ALIGNED vnr_pred_stage_state[AP_IC_PATHS] = {};
static ns_stage_ctx_t DWORD_ALIGNED ns_stage_state = {};
static agc_stage_ctx_t DWORD_ALIGNED agc_stage_state = {};

static array_select_t selector;
#if appconfARRAY_FRONTEND_MODE == ARRAY_FRONTEND_BEAM
static array_beamformer_t beamformer;
static int32_t DWORD_ALIGNED beams[ARRAY_BEAM_COUNT][appconfAUDIO_PIPELINE_FRAME_ADVANCE];
static int prev_frame_speech;
#endif

/* Diagnostics state. Owned by the front-end stage except where noted. */
static array_stats_t raw_stats;
static array_stats_t aec_stats;
static float frontend_power;
static volatile float output_power;          /* written by the AGC stage */
static volatile uint32_t ns_ticks_max;       /* written by the NS stage */
static volatile uint32_t agc_ticks_max;      /* written by the AGC stage */
static volatile uint32_t late_stage_misses;  /* NS/AGC deadline misses */
static struct {
    uint32_t frames;
    uint32_t fe_ticks_sum, fe_ticks_max;
    uint32_t aec_ticks_sum, aec_ticks_max;
    uint32_t window_frames;
    uint32_t deadline_misses;
} timing;

#if appconfARRAY_DEV_TAP
static uint8_t tap_mode = appconfARRAY_DEV_TAP_DEFAULT_MODE;
static uint8_t tap_a = appconfARRAY_DEV_TAP_DEFAULT_A;
static uint8_t tap_b = appconfARRAY_DEV_TAP_DEFAULT_B;
#endif

static void *audio_pipeline_input_i(void *input_app_data)
{
    frame_data_t *frame_data;

    frame_data = pvPortMalloc(sizeof(frame_data_t));
    memset(frame_data, 0x00, sizeof(frame_data_t));

    size_t bytes_received = 0;
    bytes_received = rtos_intertile_rx_len(
            intertile_ctx,
            appconfAUDIOPIPELINE_PORT,
            portMAX_DELAY);

    xassert(bytes_received == sizeof(frame_data_t));

    rtos_intertile_rx_data(
            intertile_ctx,
            frame_data,
            bytes_received);

    return frame_data;
}

static int audio_pipeline_output_i(frame_data_t *frame_data,
                                   void *output_app_data)
{
    return audio_pipeline_output(output_app_data,
                               (int32_t **)frame_data->samples,
                               appconfAUDIO_PIPELINE_PROC_CHANNELS +
                               appconfAUDIO_PIPELINE_REF_CHANNELS +
                               appconfAUDIO_PIPELINE_MIC_CHANNELS,
                               appconfAUDIO_PIPELINE_FRAME_ADVANCE);
}

/* One IC + VNR path. y is the target and is delayed in place by ic_filter(). */
static void run_ic_path(unsigned path, int32_t *y, int32_t *x, int32_t *out)
{
    ic_state_t *ic = &ic_stage_state[path].state;
    ic_filter(ic, y, x, out);
    voice_ic_calc_vnr_pred(ic, &vnr_pred_stage_state[path].vnr_pred_state);
    ic_adapt(ic);
}

static int16_t db_to_cdb(float db)
{
    float c = 100.0f * db;
    if (c > (float)INT16_MAX) c = (float)INT16_MAX;
    if (c < (float)INT16_MIN) c = (float)INT16_MIN;
    return (int16_t)c;
}

static int16_t peak_to_cdb(int32_t peak)
{
    const float p = (float)peak / 2147483648.0f;
    return db_to_cdb(array_power_to_db(p * p));
}

static void publish_diagnostics(const frame_data_t *frame_data, float selected_vnr)
{
    array_diag_status_t s;
    memset(&s, 0, sizeof(s));

    s.version = ARRAY_DIAG_VERSION;
    s.frontend_mode = appconfARRAY_FRONTEND_MODE;
    s.selected = (uint8_t)selector.selected;
    s.ref_mix = appconfARRAY_REF_MIX;
    s.frames = timing.frames;
    s.switch_count = selector.switch_count;

    for (unsigned ch = 0; ch < ARRAY_MIC_COUNT; ch++) {
        s.raw_rms_cdb[ch] = db_to_cdb(array_stats_rms_dbfs(&raw_stats, ch));
        s.raw_peak_cdb[ch] = peak_to_cdb(raw_stats.peak_abs[ch]);
        s.raw_clip_count[ch] = raw_stats.clip_count[ch];
        s.aec_rms_cdb[ch] = db_to_cdb(array_stats_rms_dbfs(&aec_stats, ch));
        s.aec_erle_cdb[ch] = frame_data->t1.aec_erle_cdb[ch];
    }
    s.frontend_rms_cdb = db_to_cdb(array_power_to_db(frontend_power));
    s.output_rms_cdb = db_to_cdb(array_power_to_db(output_power));

    for (unsigned i = 0; i < selector.num_candidates; i++) {
#if appconfARRAY_FRONTEND_MODE == ARRAY_FRONTEND_PAIR_SELECT
        s.score_cdb[i] = (int16_t)(selector.score[i] * 10000.0f);
#else
        s.score_cdb[i] = db_to_cdb(selector.score[i]);
#endif
    }
    float v = selected_vnr * 10000.0f;
    s.vnr_x10000 = (uint16_t)(v < 0.0f ? 0.0f : (v > 10000.0f ? 10000.0f : v));
    s.ref_active = (uint8_t)(frame_data->ref_active_flag != 0);
#if appconfARRAY_DEV_TAP
    s.tap_mode = tap_mode;
    s.tap_a = tap_a;
    s.tap_b = tap_b;
#else
    s.tap_mode = 0xFF;
#endif

    const uint32_t n = timing.window_frames ? timing.window_frames : 1;
    s.t1_aec_ticks_avg = timing.aec_ticks_sum / n;
    s.t1_aec_ticks_max = timing.aec_ticks_max;
    s.t0_frontend_ticks_avg = timing.fe_ticks_sum / n;
    s.t0_frontend_ticks_max = timing.fe_ticks_max;
    s.t0_ns_ticks_max = ns_ticks_max;
    s.t0_agc_ticks_max = agc_ticks_max;
    s.deadline_misses = timing.deadline_misses + late_stage_misses;
    s.t1_input_late_ticks_max = frame_data->t1.input_late_ticks;
    s.t1_dropped_frames_est = frame_data->t1.dropped_frames_est;
    s.t0_heap_min_free = (uint32_t)xPortGetMinimumEverFreeHeapSize();
    s.t1_heap_min_free = frame_data->t1.heap_min_free;

    array_diag_publish(&s);

    timing.window_frames = 0;
    timing.fe_ticks_sum = 0;
    timing.aec_ticks_sum = 0;
}

#if appconfARRAY_DEV_TAP
static const int32_t *tap_source(const frame_data_t *frame_data, unsigned ch, int aec)
{
    ch %= ARRAY_MIC_COUNT;
    return aec ? frame_data->samples[ch] : frame_data->mic_samples_passthrough[ch];
}
#endif

static void stage_frontend_and_ic(frame_data_t *frame_data)
{
    const uint32_t t_start = get_reference_time();
    int32_t DWORD_ALIGNED ic_output[appconfAUDIO_PIPELINE_FRAME_ADVANCE];
    float_s32_t selected_vnr;

    if (array_diag_take_peak_reset_request()) {
        array_stats_reset_peaks(&raw_stats);
    }
    array_stats_update(&raw_stats, (const int32_t (*)[ARRAY_FRAME_ADVANCE])frame_data->mic_samples_passthrough);
    array_stats_update(&aec_stats, (const int32_t (*)[ARRAY_FRAME_ADVANCE])frame_data->samples);

#if appconfARRAY_DEV_TAP
    {
        uint8_t m, a, b;
        if (array_diag_take_tap_request(&m, &a, &b)) {
            tap_mode = m;
            tap_a = a;
            tap_b = b;
        }
        frame_data->tap_mode = tap_mode;
        /* Capture before the IC overwrites the AEC outputs in place. */
        if (tap_mode == ARRAY_TAP_RAW || tap_mode == ARRAY_TAP_AEC) {
            const int aec = (tap_mode == ARRAY_TAP_AEC);
            memcpy(frame_data->tap[0], tap_source(frame_data, tap_a, aec), sizeof(frame_data->tap[0]));
            memcpy(frame_data->tap[1], tap_source(frame_data, tap_b, aec), sizeof(frame_data->tap[1]));
        }
    }
#endif

#if appconfARRAY_FRONTEND_MODE == ARRAY_FRONTEND_PAIR_SELECT
    {
        static const unsigned pair_target[ARRAY_PAIR_COUNT] = { ARRAY_PAIR_A_TARGET, ARRAY_PAIR_B_TARGET };
        static const unsigned pair_ref[ARRAY_PAIR_COUNT] = { ARRAY_PAIR_A_REFERENCE, ARRAY_PAIR_B_REFERENCE };
        int32_t DWORD_ALIGNED pair_out[ARRAY_PAIR_COUNT][appconfAUDIO_PIPELINE_FRAME_ADVANCE];
        float score[ARRAY_PAIR_COUNT];
        float best = 0.0f;

        /* Both paths every frame, sequentially: the VNR inference engine in
         * lib_vnr is a single global instance and must not be re-entered. */
        for (unsigned p = 0; p < ARRAY_PAIR_COUNT; p++) {
            run_ic_path(p, frame_data->samples[pair_target[p]], frame_data->samples[pair_ref[p]], pair_out[p]);
            score[p] = float_s32_to_float(vnr_pred_stage_state[p].vnr_pred_state.output_vnr_pred);
            if (score[p] > best) {
                best = score[p];
            }
        }

        const unsigned previous = selector.selected;
#if appconfARRAY_PAIR_FORCE >= 0
        array_select_force(&selector, appconfARRAY_PAIR_FORCE);
#else
        /* Only compare paths while at least one of them hears speech. */
        (void) array_select_update(&selector, score, best > ARRAY_PAIR_SPEECH_VNR_THRESHOLD);
#endif
        const unsigned sel = selector.selected;
        if (sel != previous) {
            array_crossfade(pair_out[previous], pair_out[sel], ic_output, appconfAUDIO_PIPELINE_FRAME_ADVANCE);
        } else {
            memcpy(ic_output, pair_out[sel], sizeof(ic_output));
        }
        selected_vnr = vnr_pred_stage_state[sel].vnr_pred_state.output_vnr_pred;
    }
#else /* ARRAY_FRONTEND_BEAM */
    {
        float energy_db[ARRAY_BEAM_COUNT];
        int32_t DWORD_ALIGNED y[appconfAUDIO_PIPELINE_FRAME_ADVANCE];
        int32_t DWORD_ALIGNED x[appconfAUDIO_PIPELINE_FRAME_ADVANCE];

        array_beamformer_process(&beamformer,
                                 (const int32_t (*)[ARRAY_FRAME_ADVANCE])frame_data->samples,
                                 beams, energy_db);

        /* Steer only on frames the previous frame's VNR marked as speech, so
         * the beam follows the talker rather than the loudest noise. */
        const unsigned previous = selector.selected;
        (void) array_select_update(&selector, energy_db, prev_frame_speech);
        const unsigned sel = selector.selected;

        if (sel != previous) {
            array_crossfade(beams[previous], beams[sel], y, appconfAUDIO_PIPELINE_FRAME_ADVANCE);
            array_crossfade(beams[array_beam_opposite(previous)], beams[array_beam_opposite(sel)],
                            x, appconfAUDIO_PIPELINE_FRAME_ADVANCE);
        } else {
            memcpy(y, beams[sel], sizeof(y));
            memcpy(x, beams[array_beam_opposite(sel)], sizeof(x));
        }

#if appconfARRAY_DEV_TAP
        if (frame_data->tap_mode == ARRAY_TAP_BEAMS) {
            memcpy(frame_data->tap[0], beams[tap_a % ARRAY_BEAM_COUNT], sizeof(frame_data->tap[0]));
            memcpy(frame_data->tap[1], beams[tap_b % ARRAY_BEAM_COUNT], sizeof(frame_data->tap[1]));
        }
#endif
        run_ic_path(0, y, x, ic_output);
        selected_vnr = vnr_pred_stage_state[0].vnr_pred_state.output_vnr_pred;
    }
#endif

    float_s32_t agc_vnr_threshold = f32_to_float_s32(VNR_AGC_THRESHOLD);
    frame_data->vnr_pred_flag = float_s32_gt(selected_vnr, agc_vnr_threshold);
#if appconfARRAY_FRONTEND_MODE == ARRAY_FRONTEND_BEAM
    prev_frame_speech = frame_data->vnr_pred_flag;
#endif

    memcpy(frame_data->samples[0], ic_output, appconfAUDIO_PIPELINE_FRAME_ADVANCE * sizeof(int32_t));
#if appconfAUDIO_PIPELINE_STORE_IC_AUDIO
    memcpy(frame_data->aec_reference_audio_samples[0], ic_output, appconfAUDIO_PIPELINE_FRAME_ADVANCE * sizeof(int32_t));   // Store the interference cancelled audio in the first reference channel
#endif
#if appconfARRAY_DEV_TAP
    if (frame_data->tap_mode == ARRAY_TAP_FRONTEND) {
        memcpy(frame_data->tap[0], ic_output, sizeof(frame_data->tap[0]));
    }
#endif

    {
        const float ms = array_frame_mean_square(ic_output, appconfAUDIO_PIPELINE_FRAME_ADVANCE);
        frontend_power = (timing.frames == 0) ? ms : 0.95f * frontend_power + 0.05f * ms;
    }

    /* Timing: this stage, and tile 1's AEC stage carried in the frame. */
    const uint32_t fe_ticks = get_reference_time() - t_start;
    timing.frames++;
    timing.window_frames++;
    timing.fe_ticks_sum += fe_ticks;
    timing.aec_ticks_sum += frame_data->t1.aec_ticks;
    if (fe_ticks > timing.fe_ticks_max) timing.fe_ticks_max = fe_ticks;
    if (frame_data->t1.aec_ticks > timing.aec_ticks_max) timing.aec_ticks_max = frame_data->t1.aec_ticks;
    if (fe_ticks > AP_FRAME_TICKS) timing.deadline_misses++;
    if (frame_data->t1.aec_ticks > AP_FRAME_TICKS) timing.deadline_misses++;

    if ((timing.frames % ARRAY_DIAG_PUBLISH_FRAMES) == 0) {
        publish_diagnostics(frame_data, float_s32_to_float(selected_vnr));
    }
}

static void stage_ns(frame_data_t *frame_data)
{
    const uint32_t t_start = get_reference_time();
#if appconfAUDIO_PIPELINE_SKIP_NS
#else
    int32_t DWORD_ALIGNED ns_output[appconfAUDIO_PIPELINE_FRAME_ADVANCE];
    configASSERT(NS_FRAME_ADVANCE == appconfAUDIO_PIPELINE_FRAME_ADVANCE);
    ns_process_frame(
                &ns_stage_state.state,
                ns_output,
                frame_data->samples[0]);
    memcpy(frame_data->samples[0], ns_output, appconfAUDIO_PIPELINE_FRAME_ADVANCE * sizeof(int32_t));
#if appconfAUDIO_PIPELINE_STORE_NS_AUDIO
    memcpy(frame_data->aec_reference_audio_samples[1], ns_output, appconfAUDIO_PIPELINE_FRAME_ADVANCE * sizeof(int32_t));   // Store NS audio in the second reference channel
#endif
#endif
    const uint32_t ticks = get_reference_time() - t_start;
    if (ticks > ns_ticks_max) ns_ticks_max = ticks;
    if (ticks > AP_FRAME_TICKS) late_stage_misses++;
}

static void stage_agc(frame_data_t *frame_data)
{
    const uint32_t t_start = get_reference_time();
#if appconfAUDIO_PIPELINE_SKIP_AGC
#else
    int32_t DWORD_ALIGNED agc_output[appconfAUDIO_PIPELINE_FRAME_ADVANCE];
    configASSERT(AGC_FRAME_ADVANCE == appconfAUDIO_PIPELINE_FRAME_ADVANCE);

    agc_stage_state.md.vnr_flag = f32_to_float_s32(frame_data->vnr_pred_flag);
    agc_stage_state.md.aec_ref_power = frame_data->max_ref_energy;
    agc_stage_state.md.aec_corr_factor = frame_data->aec_corr_factor;
    agc_stage_state.md.ref_active_flag = frame_data->ref_active_flag;

    agc_process_frame(
            &agc_stage_state.state,
            agc_output,
            frame_data->samples[0],
            &agc_stage_state.md);
    memcpy(frame_data->samples, agc_output, appconfAUDIO_PIPELINE_FRAME_ADVANCE * sizeof(int32_t));
#endif

    {
        const float ms = array_frame_mean_square(frame_data->samples[0], appconfAUDIO_PIPELINE_FRAME_ADVANCE);
        output_power = 0.95f * output_power + 0.05f * ms;
    }

#if appconfARRAY_DEV_TAP
    /* Replace the two output lanes with the tapped signals. Lane 0 goes to the
     * ESP32 voice-assistant (STT) stream, lane 1 to the wake-word engine. */
    switch (frame_data->tap_mode) {
    case ARRAY_TAP_RAW:
    case ARRAY_TAP_AEC:
    case ARRAY_TAP_BEAMS:
        memcpy(frame_data->samples[appconfAUDIO_PIPELINE_OUT_IDX_AGC], frame_data->tap[0], sizeof(frame_data->tap[0]));
        memcpy(frame_data->aec_reference_audio_samples[1], frame_data->tap[1], sizeof(frame_data->tap[1]));
        break;
    case ARRAY_TAP_FRONTEND:
        /* lane 0 = front end before NS/AGC, lane 1 = final output */
        memcpy(frame_data->aec_reference_audio_samples[1], frame_data->samples[0], sizeof(frame_data->tap[1]));
        memcpy(frame_data->samples[appconfAUDIO_PIPELINE_OUT_IDX_AGC], frame_data->tap[0], sizeof(frame_data->tap[0]));
        break;
    default:
        break;
    }
#endif

    const uint32_t ticks = get_reference_time() - t_start;
    if (ticks > agc_ticks_max) agc_ticks_max = ticks;
    if (ticks > AP_FRAME_TICKS) late_stage_misses++;
}

static void initialize_pipeline_stages(void)
{
    for (unsigned p = 0; p < AP_IC_PATHS; p++) {
        int32_t ic_status = voice_ic_init(&ic_stage_state[p].state, &vnr_pred_stage_state[p].vnr_pred_state);
        configASSERT(ic_status == 0);
        (void) ic_status;
    }

#if appconfARRAY_FRONTEND_MODE == ARRAY_FRONTEND_PAIR_SELECT
    array_select_init(&selector, ARRAY_PAIR_COUNT, 0,
                      ARRAY_SELECT_SMOOTH_ALPHA, ARRAY_PAIR_SWITCH_MARGIN_VNR,
                      ARRAY_SELECT_CONFIRM_FRAMES, ARRAY_SELECT_MIN_DWELL_FRAMES);
#else
    configASSERT(array_beamformer_init(&beamformer) == 0);
    array_select_init(&selector, ARRAY_BEAM_COUNT, 0,
                      ARRAY_SELECT_SMOOTH_ALPHA, ARRAY_BEAM_SWITCH_MARGIN_DB,
                      ARRAY_SELECT_CONFIRM_FRAMES, ARRAY_SELECT_MIN_DWELL_FRAMES);
#endif

    array_stats_init(&raw_stats, ARRAY_MIC_COUNT);
    array_stats_init(&aec_stats, ARRAY_MIC_COUNT);

    ns_init(&ns_stage_state.state);

    agc_init(&agc_stage_state.state, &AGC_PROFILE_ASR);
    agc_stage_state.md.aec_ref_power = AGC_META_DATA_NO_AEC;
    agc_stage_state.md.aec_corr_factor = AGC_META_DATA_NO_AEC;
}

void audio_pipeline_init(
    void *input_app_data,
    void *output_app_data)
{
    const int stage_count = 3;
    const pipeline_stage_t stages[] = {
        (pipeline_stage_t)stage_frontend_and_ic,
        (pipeline_stage_t)stage_ns,
        (pipeline_stage_t)stage_agc,
    };

    const configSTACK_DEPTH_TYPE stage_stack_sizes[] = {
        configMINIMAL_STACK_SIZE + RTOS_THREAD_STACK_SIZE(stage_frontend_and_ic) + RTOS_THREAD_STACK_SIZE(audio_pipeline_input_i),
        configMINIMAL_STACK_SIZE + RTOS_THREAD_STACK_SIZE(stage_ns),
        configMINIMAL_STACK_SIZE + RTOS_THREAD_STACK_SIZE(stage_agc) + RTOS_THREAD_STACK_SIZE(audio_pipeline_output_i),
    };

    initialize_pipeline_stages();

#if appconfDEVICE_CTRL_SPI
    _Static_assert(appconfARRAY_DIAG_SERVICER_COUNT == 1,
                   "the pipeline target must declare its diagnostics servicer");
    array_diag_servicer_start(AP_DIAG_SERVICER_PRIORITY);
#endif

    generic_pipeline_init((pipeline_input_t)audio_pipeline_input_i,
                        (pipeline_output_t)audio_pipeline_output_i,
                        input_app_data,
                        output_app_data,
                        stages,
                        (const size_t*) stage_stack_sizes,
                        appconfAUDIO_PIPELINE_TASK_PRIORITY,
                        stage_count);
}

#endif /* ON_TILE(0)*/
