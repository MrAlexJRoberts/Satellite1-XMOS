/* Host compile-check stub (see README.md): declarations used by the audio
 * pipelines, signatures copied from fwk_rtos rtos_intertile.h. */
#ifndef FW_STUB_DRIVER_INSTANCES_H
#define FW_STUB_DRIVER_INSTANCES_H
#include <stddef.h>
#include <stdint.h>
#include <platform.h>   /* real driver_instances.h includes it; provides ON_TILE() */
#include "device_control.h"
typedef struct rtos_intertile_struct rtos_intertile_t;
void rtos_intertile_tx(rtos_intertile_t *ctx, uint8_t port, void *msg, size_t len);
size_t rtos_intertile_rx_len(rtos_intertile_t *ctx, uint8_t port, unsigned timeout);
size_t rtos_intertile_rx_data(rtos_intertile_t *ctx, void *data, size_t len);
extern rtos_intertile_t *intertile_ctx;
extern device_control_t *device_control_spi_ctx;
#define FLASH_TILE_NO      0
#define SPEAKER_PIPELINE_TILE_NO 1
#endif
