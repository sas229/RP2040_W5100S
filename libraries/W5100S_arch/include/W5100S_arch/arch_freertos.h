/*
 * Copyright (c) 2022 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _W5100S_ARCH_ARCH_FREERTOS_H
#define _W5100S_ARCH_ARCH_FREERTOS_H

// PICO_CONFIG: W5100S_TASK_STACK_SIZE, Stack size for the W5100S FreeRTOS task in 4-byte words, type=int, default=1024, group=W5100S_arch
#ifndef W5100S_TASK_STACK_SIZE
#define W5100S_TASK_STACK_SIZE 1024
#endif

// PICO_CONFIG: W5100S_TASK_PRIORITY, Priority for the W5100S FreeRTOS task, type=int default=4, group=W5100S_arch
#ifndef W5100S_TASK_PRIORITY
#define W5100S_TASK_PRIORITY (tskIDLE_PRIORITY + 4)
#endif

#endif
