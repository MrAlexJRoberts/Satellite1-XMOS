/* Host compile-check stub (see README.md). RTOS_MEMORY_BARRIER is the real
 * definition; RTOS_THREAD_STACK_SIZE needs the xcore assembler, so it is a
 * constant here. */
#ifndef FW_STUB_RTOS_MACROS_H
#define FW_STUB_RTOS_MACROS_H
#include <stdint.h>
#define RTOS_MEMORY_BARRIER() asm volatile( "" ::: "memory" )
#define RTOS_THREAD_STACK_SIZE(thread_entry) ((uint32_t)1024)
#endif
