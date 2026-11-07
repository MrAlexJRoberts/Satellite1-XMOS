// Copyright 2022-2024 XMOS LIMITED.
// Copyright 2026 Future Proof Homes
// This Software is subject to the terms of the XMOS Public Licence: Version 1.
//
// Four-microphone fixed_delay pipeline, tile 1: capture and 4Y x 1X AEC.

/* STD headers */
#include <math.h>
#include <string.h>
#include <stdint.h>
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
#include "audio_pipeline.h"
#include "audio_pipeline_dsp.h"
#include "aec_schedule_array.h"
#include "array_reference.h"

#if appconfAUDIO_PIPELINE_FRAME_ADVANCE != 240
#error This pipeline is only configured for 240 frame advance
#endif

/* xcore reference clock: 100 MHz. One frame is 240 samples at 16 kHz = 15 ms. */
#define AP_REF_CLOCK_HZ        (100000000)
#define AP_FRAME_TICKS         (appconfAUDIO_PIPELINE_FRAME_ADVANCE * (AP_REF_CLOCK_HZ / appconfAUDIO_PIPELINE_SAMPLE_RATE))

/* Window over which the minimum input lateness is taken (~1 s). */
#define AP_LATENESS_WINDOW     (64)

/* Activity threshold for the reference, as in the 2-mic pipeline. */
#define AP_REF_ACTIVE_THRESHOLD (0.001)

#if ON_TILE(1)
static aec_ctx_t DWORD_ALIGNED aec_state = {};

/*
 * Mic overrun estimate without touching driver internals.
 *
 * The PDM mic clock and the 100 MHz reference clock come from the same
 * crystal, so frame n becomes available at t_first + n * 15 ms exactly. When
 * the pipeline keeps up, the input stage returns within the 2-frame mic
 * buffer of that time. Each frame dropped by the mic-array ISR on overrun
 * shifts every later return by one frame period for good. So the minimum
 * lateness over a window, rounded to whole frames, counts dropped frames, and
 * transient lateness that the buffer absorbed does not.
 */
static struct {
    int started;
    uint32_t expected;
    int32_t window_min_late;
    unsigned window_count;
    uint32_t dropped_est;
    uint32_t max_late;
} input_clock;

static void track_input_timing(frame_data_t *frame_data)
{
    const uint32_t now = get_reference_time();

    if (!input_clock.started) {
        input_clock.started = 1;
        input_clock.expected = now;
        input_clock.window_min_late = INT32_MAX;
    }

    /* Signed difference of wrapping 32-bit tick counts: valid for +/-21 s. */
    int32_t late = (int32_t)(now - input_clock.expected);
    if (late < 0) {
        late = 0;
    }
    if ((uint32_t)late > input_clock.max_late) {
        input_clock.max_late = (uint32_t)late;
    }
    if (late < input_clock.window_min_late) {
        input_clock.window_min_late = late;
    }
    if (++input_clock.window_count >= AP_LATENESS_WINDOW) {
        const uint32_t est = ((uint32_t)input_clock.window_min_late + AP_FRAME_TICKS / 2) / AP_FRAME_TICKS;
        if (est > input_clock.dropped_est) {
            input_clock.dropped_est = est;
        }
        input_clock.window_count = 0;
        input_clock.window_min_late = INT32_MAX;
    }
    input_clock.expected += AP_FRAME_TICKS;

    frame_data->t1.input_late_ticks = input_clock.max_late;
    frame_data->t1.dropped_frames_est = input_clock.dropped_est;
}

static void *audio_pipeline_input_i(void *input_app_data)
{
    frame_data_t *frame_data;
    frame_data = pvPortMalloc(sizeof(frame_data_t));
    memset(frame_data, 0x00, sizeof(frame_data_t));

    /* Reference channels followed by microphone channels, written in place
     * starting at aec_reference_audio_samples (layout checked in the header). */
    audio_pipeline_input(input_app_data,
                       (int32_t **)frame_data->aec_reference_audio_samples,
                       appconfAUDIO_PIPELINE_REF_CHANNELS + appconfAUDIO_PIPELINE_MIC_CHANNELS,
                       appconfAUDIO_PIPELINE_FRAME_ADVANCE);

    track_input_timing(frame_data);

    frame_data->vnr_pred_flag = 0;

    memcpy(frame_data->samples, frame_data->mic_samples_passthrough, sizeof(frame_data->samples));

    return frame_data;
}

static int audio_pipeline_output_i(frame_data_t *frame_data,
                                   void *output_app_data)
{
    rtos_intertile_tx(intertile_ctx,
                      appconfAUDIOPIPELINE_PORT,
                      frame_data,
                      sizeof(frame_data_t));

    return AUDIO_PIPELINE_FREE_FRAME;
}

/* Kept as a separate (empty) stage so tile 1 has the same two-stage pipeline
 * shape, and therefore the same latency, as the 2-mic fixed_delay pipeline. */
static void stage_delay(frame_data_t *frame_data)
{
    (void) frame_data;
}

static int16_t ratio_to_cdb(float_s32_t num, float_s32_t den)
{
    const float n = float_s32_to_float(num);
    const float d = float_s32_to_float(den);
    if (!(n > 0.0f) || !(d > 0.0f)) {
        return 0;
    }
    float cdb = 1000.0f * log10f(n / d);
    if (cdb > (float)INT16_MAX) {
        cdb = (float)INT16_MAX;
    }
    if (cdb < (float)INT16_MIN) {
        cdb = (float)INT16_MIN;
    }
    return (int16_t)cdb;
}

static void stage_aec(frame_data_t *frame_data)
{
    const uint32_t t_start = get_reference_time();

#if appconfAUDIO_PIPELINE_SKIP_AEC
#else
    int32_t DWORD_ALIGNED stage1_output[AEC_MAX_Y_CHANNELS][appconfAUDIO_PIPELINE_FRAME_ADVANCE];
    int32_t DWORD_ALIGNED mono_ref[AEC_MAX_X_CHANNELS][appconfAUDIO_PIPELINE_FRAME_ADVANCE];

    /* The TAS2780 plays one mono signal; build the reference that matches it
     * (see array_reference.h). The stereo reference stays in the frame for the
     * diagnostics tap and for tile 0, which reuses the slots for output audio. */
    array_reference_mix(frame_data->aec_reference_audio_samples[0],
                        frame_data->aec_reference_audio_samples[1],
                        mono_ref[0],
                        appconfAUDIO_PIPELINE_FRAME_ADVANCE,
                        appconfARRAY_REF_MIX);

    aec_process_frame_1thread(
            &aec_state.state.main_state,
            &aec_state.state.shadow_state,
            stage1_output,
            NULL,
            (const int32_t (*)[AEC_FRAME_ADVANCE])frame_data->samples,
            (const int32_t (*)[AEC_FRAME_ADVANCE])mono_ref);

    frame_data->max_ref_energy = aec_calc_max_input_energy(
                                    (const int32_t (*)[AEC_FRAME_ADVANCE])mono_ref,
                                    aec_state.state.main_state.shared_state->num_x_channels);
    /* AGC loss control uses the correlation of one microphone with the echo
     * estimate; channel 0 is the production East microphone. */
    frame_data->aec_corr_factor = aec_calc_corr_factor(&aec_state.state.main_state, 0);
    frame_data->ref_active_flag = aec_detect_input_activity(
            (const int32_t (*)[AEC_FRAME_ADVANCE])mono_ref,
            f64_to_float_s32(AP_REF_ACTIVE_THRESHOLD), AEC_MAX_X_CHANNELS);
    memcpy(frame_data->samples, stage1_output, AEC_MAX_Y_CHANNELS * appconfAUDIO_PIPELINE_FRAME_ADVANCE * sizeof(int32_t));

    /* Running ERLE estimate per microphone from the AEC's own input and
     * residual energy averages. Only meaningful while the reference is active. */
    for (unsigned ch = 0; ch < AEC_MAX_Y_CHANNELS; ch++) {
        frame_data->t1.aec_erle_cdb[ch] = ratio_to_cdb(
                aec_state.state.shared_state.y_ema_energy[ch],
                aec_state.state.main_state.error_ema_energy[ch]);
    }
#endif

    frame_data->t1.aec_ticks = get_reference_time() - t_start;
    frame_data->t1.heap_min_free = (uint32_t)xPortGetMinimumEverFreeHeapSize();
}

static void initialize_pipeline_stages(void)
{
    /* fwk_voice only provides default task schedules for two channels; the
     * single-thread frame driver ignores the schedule, but aec_init() requires
     * a valid one. */
    aec_init(&aec_state.state,
             AEC_MAX_Y_CHANNELS,
             AEC_MAX_X_CHANNELS,
             AEC_MAIN_FILTER_PHASES,
             AEC_SHADOW_FILTER_PHASES,
             &aec_tdist_chans4_threads1);
}

void audio_pipeline_init(
    void *input_app_data,
    void *output_app_data)
{
    const int stage_count = 2;
    const pipeline_stage_t stages[] = {
        (pipeline_stage_t)stage_delay,
        (pipeline_stage_t)stage_aec,
    };

    const configSTACK_DEPTH_TYPE stage_stack_sizes[] = {
        configMINIMAL_STACK_SIZE + RTOS_THREAD_STACK_SIZE(stage_delay) + RTOS_THREAD_STACK_SIZE(audio_pipeline_input_i),
        configMINIMAL_STACK_SIZE + RTOS_THREAD_STACK_SIZE(stage_aec) + RTOS_THREAD_STACK_SIZE(audio_pipeline_output_i),
    };

    initialize_pipeline_stages();

    generic_pipeline_init((pipeline_input_t)audio_pipeline_input_i,
                        (pipeline_output_t)audio_pipeline_output_i,
                        input_app_data,
                        output_app_data,
                        stages,
                        (const size_t*) stage_stack_sizes,
                        appconfAUDIO_PIPELINE_TASK_PRIORITY,
                        stage_count);

}
#endif /* ON_TILE(1) */
