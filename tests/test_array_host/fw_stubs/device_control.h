/* Host compile-check stub (see README.md). Types and prototypes copied from
 * modules/fph/rtos_device_control/api/device_control.h; the shared protocol
 * definitions are the real header. */
#ifndef FW_STUB_DEVICE_CONTROL_H
#define FW_STUB_DEVICE_CONTROL_H
#include <stddef.h>
#include <stdint.h>
#undef DEVICE_CONTROL_CALLBACK_ATTR
#include "device_control_shared.h"
#undef DEVICE_CONTROL_CALLBACK_ATTR
#define DEVICE_CONTROL_CALLBACK_ATTR
#define RTOS_OSAL_WAIT_FOREVER 0xffffffffu
typedef struct device_control_struct device_control_t;
typedef struct { int opaque; } device_control_servicer_t;
typedef control_ret_t (*device_control_read_cmd_cb_t)(control_resid_t resid, control_cmd_t cmd, uint8_t *payload, size_t payload_len, void *app_data);
typedef control_ret_t (*device_control_write_cmd_cb_t)(control_resid_t resid, control_cmd_t cmd, const uint8_t *payload, size_t payload_len, void *app_data);
control_ret_t device_control_servicer_cmd_recv(device_control_servicer_t *ctx,
                                               device_control_read_cmd_cb_t read_cmd_cb,
                                               device_control_write_cmd_cb_t write_cmd_cb,
                                               void *app_data,
                                               unsigned timeout);
control_ret_t device_control_servicer_register(device_control_servicer_t *ctx,
                                               device_control_t *device_control_ctx[],
                                               size_t device_control_ctx_count,
                                               const control_resid_t resources[],
                                               size_t num_resources);
#endif
