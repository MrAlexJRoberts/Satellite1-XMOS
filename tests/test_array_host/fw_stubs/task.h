/* Host compile-check stub (see README.md). */
#ifndef FW_STUB_TASK_H
#define FW_STUB_TASK_H
#include "FreeRTOS.h"
typedef void (*TaskFunction_t)(void *);
typedef void *TaskHandle_t;
BaseType_t xTaskCreate(TaskFunction_t pxTaskCode, const char *const pcName,
                       const configSTACK_DEPTH_TYPE usStackDepth, void *const pvParameters,
                       UBaseType_t uxPriority, TaskHandle_t *const pxCreatedTask);
#endif
