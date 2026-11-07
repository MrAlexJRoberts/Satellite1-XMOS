/* Host compile-check stub (see README.md). */
#ifndef FW_STUB_FREERTOS_H
#define FW_STUB_FREERTOS_H
#include <stddef.h>
#include <stdint.h>
#include <assert.h>
#include "rtos_macros.h"
typedef long BaseType_t;
typedef unsigned long UBaseType_t;
#define configSTACK_DEPTH_TYPE   uint32_t
#define configMINIMAL_STACK_SIZE ((configSTACK_DEPTH_TYPE)256)
#define configMAX_PRIORITIES     32
#define configASSERT(x)          assert(x)
#define portMAX_DELAY            0xffffffffu
void *pvPortMalloc(size_t size);
void vPortFree(void *p);
size_t xPortGetMinimumEverFreeHeapSize(void);
size_t xPortGetFreeHeapSize(void);
#endif
