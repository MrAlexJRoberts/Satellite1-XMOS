// Copyright 2026 Future Proof Homes
//
// Microphone-array diagnostics, read over the existing SPI device-control link
// (the only observability path on a Satellite1: there is no xTAG, so
// rtos_printf output goes nowhere).
//
// Resource ARRAY_DIAG_RESID (233). IDs 230-232 are used by the audio pipeline
// control servicers on the four_mics_sandbox / SQ66 branches and 231 by the
// speaker EQ branch, so 233 avoids a clash if those land.
//
//   cmd 0x80 ARRAY_DIAG_CMD_GET_STATUS (read, sizeof(array_diag_status_t)):
//            the latest snapshot. Reading clears the peak holds.
//   cmd 0x01 ARRAY_DIAG_CMD_SET_TAP (write, 3 bytes: mode, ch_a, ch_b):
//            dev-tap builds only; CONTROL_BAD_COMMAND otherwise.
//
// The snapshot is published by the tile 0 pipeline every
// ARRAY_DIAG_PUBLISH_FRAMES frames; reading it never blocks the audio path.

#ifndef ARRAY_DIAG_H_
#define ARRAY_DIAG_H_

#include <stdint.h>
#include "array_config.h"

#define ARRAY_DIAG_RESID              (233)
#define ARRAY_DIAG_VERSION            (1)
#define ARRAY_DIAG_PUBLISH_FRAMES     (16)     /* 240 ms */

enum array_diag_cmds {
    ARRAY_DIAG_CMD_GET_STATUS = 0,
    ARRAY_DIAG_CMD_SET_TAP = 1,
    NUM_ARRAY_DIAG_CMDS
};

/* Wire format: little-endian, packed. Levels in centi-dBFS (-3010 = -30.10
 * dBFS); ratios in centi-dB; times in 100 MHz reference-clock ticks
 * (1 frame = 15 ms = 1,500,000 ticks). Bump ARRAY_DIAG_VERSION on change. */
typedef struct __attribute__((packed)) {
    uint8_t  version;
    uint8_t  frontend_mode;          /* ARRAY_FRONTEND_* */
    uint8_t  selected;               /* pair (0 = E/W, 1 = N/S) or beam (0..7, 45 deg steps) */
    uint8_t  ref_mix;                /* ARRAY_REF_MIX_* */
    uint32_t frames;                 /* tile 0 frames processed */
    uint32_t switch_count;           /* pair/beam changes since boot */

    int16_t  raw_rms_cdb[ARRAY_MIC_COUNT];   /* after the fixed 10x gain, before AEC */
    int16_t  raw_peak_cdb[ARRAY_MIC_COUNT];  /* since the previous read */
    uint32_t raw_clip_count[ARRAY_MIC_COUNT];/* cumulative samples at >= -0.009 dBFS */
    int16_t  aec_rms_cdb[ARRAY_MIC_COUNT];   /* AEC outputs */
    int16_t  aec_erle_cdb[ARRAY_MIC_COUNT];  /* AEC input/residual energy, valid when ref_active */
    int16_t  frontend_rms_cdb;               /* front-end output (IC output, before NS) */
    int16_t  output_rms_cdb;                 /* final AGC output (lane 0 to the ESP32) */
    int16_t  score_cdb[ARRAY_BEAM_COUNT];    /* beams: smoothed pre-emphasised energy;
                                                pairs: smoothed output VNR x 10000 */
    uint16_t vnr_x10000;                     /* VNR of the path in use */
    uint8_t  ref_active;
    uint8_t  tap_mode;                       /* ARRAY_TAP_*, 0xFF if not compiled in */
    uint8_t  tap_a;
    uint8_t  tap_b;

    uint32_t t1_aec_ticks_avg;
    uint32_t t1_aec_ticks_max;
    uint32_t t0_frontend_ticks_avg;          /* front end + IC + VNR stage */
    uint32_t t0_frontend_ticks_max;
    uint32_t t0_ns_ticks_max;
    uint32_t t0_agc_ticks_max;
    uint32_t deadline_misses;                /* stage executions longer than one frame */
    uint32_t t1_input_late_ticks_max;
    uint32_t t1_dropped_frames_est;
    uint32_t t0_heap_min_free;
    uint32_t t1_heap_min_free;
} array_diag_status_t;

_Static_assert(sizeof(array_diag_status_t) <= 250, "must fit one device-control read");

/* Pipeline side (tile 0). Publishing is lock-free (sequence counter); the
 * reader retries if it races with a publish. */
void array_diag_publish(const array_diag_status_t *status);

/* Servicer side. Returns 0 and fills *status, or -1 if no consistent snapshot
 * could be read (never published yet, or contention). */
int array_diag_read(array_diag_status_t *status);

/* Peak holds are cleared by the pipeline when this flag is set by a read. */
int array_diag_take_peak_reset_request(void);

/* Dev tap control (only meaningful when appconfARRAY_DEV_TAP). */
void array_diag_request_tap(uint8_t mode, uint8_t a, uint8_t b);
int array_diag_take_tap_request(uint8_t *mode, uint8_t *a, uint8_t *b);

/* Start the device-control servicer task on this tile (tile 0). The caller
 * must account for it in appconfARRAY_DIAG_SERVICER_COUNT. */
void array_diag_servicer_start(unsigned priority);

#endif /* ARRAY_DIAG_H_ */
