/*
 * Copyright (c) 2022 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#if W5100S_ARCH_FREERTOS

#include "W5100S_arch.h"
#include "W5100S_driver.h"
#include "pico/async_context_freertos.h"

#if W5100S_LWIP
#include "pico/lwip_freertos.h"
#include <lwip/tcpip.h>
#endif

#if NO_SYS
#error example_W5100S_arch_freetos_sys requires NO_SYS=0
#endif

static async_context_freertos_t W5100S_async_context_freertos;

async_context_t *W5100S_arch_init_default_async_context(void) {
    async_context_freertos_config_t config = async_context_freertos_default_config();
#ifdef W5100S_TASK_PRIORITY
    config.task_priority = W5100S_TASK_PRIORITY;
#endif
#ifdef W5100S_TASK_STACK_SIZE
    config.task_stack_size = W5100S_TASK_STACK_SIZE;
#endif
    if (async_context_freertos_init(&W5100S_async_context_freertos, &config))
        return &W5100S_async_context_freertos.core;
    return NULL;
}

// todo - add a check to see if the cyw43 context exists and if so, use that.
int W5100S_arch_init(void) {
    async_context_t *context = W5100S_arch_async_context();
    if (!context) {
        context = W5100S_arch_init_default_async_context();
        if (!context) return PICO_ERROR_GENERIC;
        W5100S_arch_set_async_context(context);
    }
    bool ok = W5100S_driver_init(context);
#if W5100S_LWIP
    ok &= lwip_freertos_init(context);
#endif
    if (!ok) {
        W5100S_arch_deinit();
        return PICO_ERROR_GENERIC;
    } else {
        return 0;
    }
}

void W5100S_arch_deinit(void) {
    async_context_t *context = W5100S_arch_async_context();
    // There is a bit of a circular dependency here between lwIP and W5100S_driver. We
    // shut down W5100S_driver first as it has IRQs calling back into lwIP. Also lwIP itself
    // does not actually get shut down.
    W5100S_driver_deinit(context);
#if W5100S_LWIP
    lwip_freertos_deinit(context);
#endif
    // If it is our context, then we de-init it.
    if (context == &W5100S_async_context_freertos.core) {
        async_context_deinit(context);
        W5100S_arch_set_async_context(NULL);
    }
}

#endif