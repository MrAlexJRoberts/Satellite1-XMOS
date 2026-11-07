// Copyright 2022-2023 XMOS LIMITED.
// Copyright 2026 Future Proof Homes
// This Software is subject to the terms of the XMOS Public Licence: Version 1.
//
// Four-microphone variant of the fixed_delay reference pipeline.
//
//   tile 1: 4 PDM mics -> fixed 10x gain -> 4Y x 1X AEC (mono reference)
//   tile 0: array front end (pair-select IC or beamformer + IC) -> NS -> AGC
//           -> the unchanged 2-lane I2S output to the ESP32
//
// The production 2-mic pipeline (reference/fixed_delay) is not modified.

#ifndef AUDIO_PIPELINE_DSP_H_
#define AUDIO_PIPELINE_DSP_H_

#include <stddef.h>
#include <stdint.h>
#include "FreeRTOS.h"
#include "stream_buffer.h"
#include "app_conf.h"

#include "array_config.h"

#ifndef appconfARRAY_FRONTEND_MODE
#error "appconfARRAY_FRONTEND_MODE must be set by the pipeline target"
#endif

#ifndef appconfARRAY_REF_MIX
#define appconfARRAY_REF_MIX ARRAY_REF_MIX_DOWNMIX
#endif

/* Development audio tap: compiled in only when requested (see CMake option
 * SAT1_ARRAY_DEV_TAP). Production-candidate builds have no tap code. */
#ifndef appconfARRAY_DEV_TAP
#define appconfARRAY_DEV_TAP 0
#endif
#ifndef appconfARRAY_DEV_TAP_DEFAULT_MODE
#define appconfARRAY_DEV_TAP_DEFAULT_MODE ARRAY_TAP_AEC
#endif
#ifndef appconfARRAY_DEV_TAP_DEFAULT_A
#define appconfARRAY_DEV_TAP_DEFAULT_A ARRAY_CH_EAST
#endif
#ifndef appconfARRAY_DEV_TAP_DEFAULT_B
#define appconfARRAY_DEV_TAP_DEFAULT_B ARRAY_CH_NORTH
#endif

/* Pair selection: -1 = automatic, 0 = force pair A (E/W, the production
 * pair), 1 = force pair B (N/S). Forcing is for A/B debugging only. */
#ifndef appconfARRAY_PAIR_FORCE
#define appconfARRAY_PAIR_FORCE (-1)
#endif

/* Pipeline config */
#define AP_MAX_Y_CHANNELS (ARRAY_MIC_COUNT)
#define AP_MAX_X_CHANNELS (1)
#define AP_FRAME_ADVANCE  (240)

#include "aec.h"
#include "agc.h"
#include "ic.h"
#include "ns.h"
#include "voice_compat.h"

/* The AEC state layout is fixed at compile time by fwk_voice; the pipeline
 * target sets AEC_MAX_Y_CHANNELS / AEC_MAX_X_CHANNELS to match. */
_Static_assert(AEC_MAX_Y_CHANNELS == AP_MAX_Y_CHANNELS, "AEC microphone ABI mismatch");
_Static_assert(AEC_MAX_X_CHANNELS == AP_MAX_X_CHANNELS, "AEC reference ABI mismatch");
_Static_assert(AEC_MAIN_FILTER_PHASES == 10 && AEC_SHADOW_FILTER_PHASES == 5, "AEC phase ABI mismatch");
_Static_assert(appconfAUDIO_PIPELINE_MIC_CHANNELS == ARRAY_MIC_COUNT,
               "four-mic pipeline needs the four-mic board (fph::ffva::satellite1_4mic)");
_Static_assert(appconfAUDIO_PIPELINE_PROC_CHANNELS == ARRAY_MIC_COUNT,
               "four-mic pipeline carries all four AEC outputs to tile 0");
_Static_assert(appconfAUDIO_PIPELINE_FRAME_ADVANCE == ARRAY_FRAME_ADVANCE, "frame size mismatch");
_Static_assert(appconfAUDIO_PIPELINE_SAMPLE_RATE == ARRAY_SAMPLE_RATE_HZ, "sample rate mismatch");
_Static_assert(AEC_FRAME_ADVANCE == ARRAY_FRAME_ADVANCE, "AEC frame size mismatch");
_Static_assert(IC_FRAME_ADVANCE == ARRAY_FRAME_ADVANCE, "IC frame size mismatch");

/* Static mic/reference delay is not implemented for the array pipeline. */
#if appconfINPUT_SAMPLES_MIC_DELAY_MS != 0
#error "appconfINPUT_SAMPLES_MIC_DELAY_MS is not supported by the four-mic pipeline"
#endif

/* Per-frame information produced on tile 1 for the diagnostics on tile 0. */
typedef struct {
    int16_t  aec_erle_cdb[ARRAY_MIC_COUNT];  /* AEC input / residual energy, centi-dB */
    uint32_t aec_ticks;                      /* AEC stage time, 100 MHz ticks */
    uint32_t input_late_ticks;               /* mic input lateness vs the 15 ms frame clock */
    uint32_t dropped_frames_est;             /* mic frames lost to overruns (estimate) */
    uint32_t heap_min_free;                  /* tile 1 FreeRTOS heap low-water mark */
} array_t1_meta_t;

/* Note: the order of the first three arrays is the channel order of
 * audio_pipeline_input() (reference then microphones, starting at
 * aec_reference_audio_samples) and audio_pipeline_output() (everything,
 * starting at samples). */
typedef struct {
    int32_t samples[appconfAUDIO_PIPELINE_PROC_CHANNELS][appconfAUDIO_PIPELINE_FRAME_ADVANCE];
    int32_t aec_reference_audio_samples[appconfAUDIO_PIPELINE_REF_CHANNELS][appconfAUDIO_PIPELINE_FRAME_ADVANCE];
    int32_t mic_samples_passthrough[appconfAUDIO_PIPELINE_MIC_CHANNELS][appconfAUDIO_PIPELINE_FRAME_ADVANCE];

    /* Below is additional context needed by other stages on a per frame basis */
    int32_t vnr_pred_flag;
    float_s32_t max_ref_energy;
    float_s32_t aec_corr_factor;
    int32_t ref_active_flag;

    array_t1_meta_t t1;
#if appconfARRAY_DEV_TAP
    /* Signals captured by the front end for the development tap; substituted
     * for the two output lanes at the end of the pipeline. */
    int32_t tap[2][appconfAUDIO_PIPELINE_FRAME_ADVANCE];
    int32_t tap_mode;
#endif
} frame_data_t;

_Static_assert(offsetof(frame_data_t, aec_reference_audio_samples) ==
               offsetof(frame_data_t, samples) + sizeof(((frame_data_t *)0)->samples),
               "frame layout must be contiguous for audio_pipeline_output()");
_Static_assert(offsetof(frame_data_t, mic_samples_passthrough) ==
               offsetof(frame_data_t, aec_reference_audio_samples) +
               sizeof(((frame_data_t *)0)->aec_reference_audio_samples),
               "frame layout must be contiguous for audio_pipeline_input()");

typedef struct aec_ctx {
    aec_state_t DWORD_ALIGNED state;
} aec_ctx_t;

typedef struct ic_stage_ctx {
    ic_state_t DWORD_ALIGNED state;
} ic_stage_ctx_t;

typedef struct vnr_pred_stage_ctx {
    voice_output_vnr_state_t vnr_pred_state;
} vnr_pred_stage_ctx_t;

typedef struct ns_stage_ctx {
    ns_state_t DWORD_ALIGNED state;
} ns_stage_ctx_t;

typedef struct agc_stage_ctx {
    agc_meta_data_t md;
    agc_state_t DWORD_ALIGNED state;
} agc_stage_ctx_t;

void aec_process_frame_1thread(
        aec_filter_state_t *main_state,
        aec_filter_state_t *shadow_state,
        int32_t (*output_main)[AEC_FRAME_ADVANCE],
        int32_t (*output_shadow)[AEC_FRAME_ADVANCE],
        const int32_t (*y_data)[AEC_FRAME_ADVANCE],
        const int32_t (*x_data)[AEC_FRAME_ADVANCE]);

#endif /* AUDIO_PIPELINE_DSP_H_ */
