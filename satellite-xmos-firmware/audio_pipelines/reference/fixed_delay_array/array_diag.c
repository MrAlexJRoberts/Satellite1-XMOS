// Copyright 2026 Future Proof Homes
//
// Array diagnostics snapshot and its read-only device-control servicer.
// Structure follows led_ring/led_ring_servicer.c.

#include <string.h>
#include <platform.h>
#include <xassert.h>

#include "FreeRTOS.h"
#include "task.h"
#include "rtos_macros.h"

#include "platform/platform_conf.h"
#include "platform/driver_instances.h"
#include "servicer.h"
#include "array_diag.h"

#if ON_TILE(0)

/* ---------------------------------------------------------------------------
 * Snapshot exchange between the pipeline task and the servicer task.
 *
 * Both run on tile 0 but may be on different cores. The writer bumps the
 * sequence number to an odd value, copies, then bumps it to even; a reader
 * accepts a copy only if it saw the same even value before and after. Tile
 * SRAM has no cache, so a compiler barrier is sufficient ordering.
 * ------------------------------------------------------------------------- */
static volatile uint32_t diag_seq;
static array_diag_status_t diag_snapshot;
static volatile int diag_peak_reset_request;

static volatile int tap_request_pending;
static volatile uint8_t tap_request[3];

void array_diag_publish(const array_diag_status_t *status)
{
    diag_seq++;
    RTOS_MEMORY_BARRIER();
    memcpy((void *)&diag_snapshot, status, sizeof(diag_snapshot));
    RTOS_MEMORY_BARRIER();
    diag_seq++;
}

int array_diag_read(array_diag_status_t *status)
{
    for (int attempt = 0; attempt < 4; attempt++) {
        const uint32_t before = diag_seq;
        RTOS_MEMORY_BARRIER();
        if (before == 0 || (before & 1)) {
            if (before == 0) {
                return -1;          /* nothing published yet */
            }
            continue;               /* write in progress */
        }
        memcpy(status, (const void *)&diag_snapshot, sizeof(*status));
        RTOS_MEMORY_BARRIER();
        if (diag_seq == before) {
            return 0;
        }
    }
    return -1;
}

int array_diag_take_peak_reset_request(void)
{
    if (diag_peak_reset_request) {
        diag_peak_reset_request = 0;
        return 1;
    }
    return 0;
}

void array_diag_request_tap(uint8_t mode, uint8_t a, uint8_t b)
{
    tap_request[0] = mode;
    tap_request[1] = a;
    tap_request[2] = b;
    RTOS_MEMORY_BARRIER();
    tap_request_pending = 1;
}

int array_diag_take_tap_request(uint8_t *mode, uint8_t *a, uint8_t *b)
{
    if (!tap_request_pending) {
        return 0;
    }
    RTOS_MEMORY_BARRIER();
    *mode = tap_request[0];
    *a = tap_request[1];
    *b = tap_request[2];
    tap_request_pending = 0;
    return 1;
}

/* ---------------------------------------------------------------------------
 * Servicer
 * ------------------------------------------------------------------------- */
#define NUM_RESOURCES_ARRAY_DIAG   (1)

static control_cmd_info_t array_diag_cmd_map[] = {
    { ARRAY_DIAG_CMD_GET_STATUS, sizeof(array_diag_status_t), sizeof(uint8_t), CMD_READ_ONLY },
    { ARRAY_DIAG_CMD_SET_TAP,    3,                           sizeof(uint8_t), CMD_WRITE_ONLY },
};

static servicer_t array_diag_servicer_state;

DEVICE_CONTROL_CALLBACK_ATTR
static control_ret_t array_diag_read_cmd(control_resid_t resid, control_cmd_t cmd,
                                         uint8_t *payload, size_t payload_len, void *app_data)
{
    servicer_t *servicer = (servicer_t *)app_data;

    /* payload[0] is the status byte; the data follows. */
    control_resource_info_t *res_info = get_res_info(resid, servicer);
    xassert(res_info != NULL);
    control_cmd_info_t *cmd_info;
    control_ret_t ret = validate_cmd(&cmd_info, res_info, cmd, &payload[1], payload_len - 1);
    if (ret != CONTROL_SUCCESS) {
        payload[0] = ret;
        return ret;
    }

    switch (CONTROL_CMD_CLEAR_READ(cmd)) {
    case ARRAY_DIAG_CMD_GET_STATUS: {
        array_diag_status_t status;
        if (array_diag_read(&status) != 0) {
            ret = CONTROL_ERROR;
            break;
        }
        memcpy(&payload[1], &status, sizeof(status));
        diag_peak_reset_request = 1;
        break;
    }
    default:
        ret = CONTROL_BAD_COMMAND;
        break;
    }
    payload[0] = ret;
    return ret;
}

DEVICE_CONTROL_CALLBACK_ATTR
static control_ret_t array_diag_write_cmd(control_resid_t resid, control_cmd_t cmd,
                                          const uint8_t *payload, size_t payload_len, void *app_data)
{
    servicer_t *servicer = (servicer_t *)app_data;
    control_resource_info_t *res_info = get_res_info(resid, servicer);
    xassert(res_info != NULL);
    control_cmd_info_t *cmd_info;
    control_ret_t ret = validate_cmd(&cmd_info, res_info, cmd, payload, payload_len);
    if (ret != CONTROL_SUCCESS) {
        return ret;
    }

    switch (CONTROL_CMD_CLEAR_READ(cmd)) {
    case ARRAY_DIAG_CMD_SET_TAP:
#if appconfARRAY_DEV_TAP
        if (payload[0] > ARRAY_TAP_BEAMS) {
            return CONTROL_DATA_LENGTH_ERROR;
        }
        array_diag_request_tap(payload[0], payload[1], payload[2]);
        return CONTROL_SUCCESS;
#else
        return CONTROL_BAD_COMMAND;
#endif
    default:
        return CONTROL_BAD_COMMAND;
    }
}

static void array_diag_servicer(void *args)
{
    servicer_t *servicer = (servicer_t *)args;
    device_control_servicer_t servicer_ctx;
    device_control_t *device_control_ctx[1] = {device_control_spi_ctx};
    control_resid_t resources[NUM_RESOURCES_ARRAY_DIAG] = {ARRAY_DIAG_RESID};

    control_ret_t dc_ret = device_control_servicer_register(&servicer_ctx,
                                                            device_control_ctx,
                                                            1,
                                                            resources,
                                                            NUM_RESOURCES_ARRAY_DIAG);
    xassert(dc_ret == CONTROL_SUCCESS);
    (void) dc_ret;

    for (;;) {
        device_control_servicer_cmd_recv(&servicer_ctx,
                                         array_diag_read_cmd,
                                         array_diag_write_cmd,
                                         servicer,
                                         RTOS_OSAL_WAIT_FOREVER);
    }
}

void array_diag_servicer_start(unsigned priority)
{
    static control_resource_info_t res_info[NUM_RESOURCES_ARRAY_DIAG];
    servicer_t *servicer = &array_diag_servicer_state;

    memset(servicer, 0, sizeof(*servicer));
    servicer->id = ARRAY_DIAG_RESID;
    servicer->start_io = 0;
    servicer->num_resources = NUM_RESOURCES_ARRAY_DIAG;
    servicer->res_info = &res_info[0];
    servicer->res_info[0].resource = ARRAY_DIAG_RESID;
    servicer->res_info[0].command_map.num_commands = NUM_ARRAY_DIAG_CMDS;
    servicer->res_info[0].command_map.commands = array_diag_cmd_map;

    xTaskCreate((TaskFunction_t)array_diag_servicer,
                "array diag",
                RTOS_THREAD_STACK_SIZE(array_diag_servicer),
                servicer,
                priority,
                NULL);
}

#endif /* ON_TILE(0) */
