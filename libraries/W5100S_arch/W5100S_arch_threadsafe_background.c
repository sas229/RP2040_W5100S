/*
 * Copyright (c) 2022 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#if W5100S_ARCH_THREADSAFE_BACKGROUND

#include "W5100S_arch.h"
#include "W5100S_driver.h"
#include "pico/async_context_threadsafe_background.h"

#if W5100S_LWIP
#include "pico/lwip_nosys.h"
#endif

#if W5100S_LWIP && !NO_SYS
#error PICO_W5100S_ARCH_THREADSAFE_BACKGROUND requires lwIP NO_SYS=1
#endif
#if W5100S_LWIP && MEM_LIBC_MALLOC
// Would attempt to use malloc from IRQ context.
#error MEM_LIBC_MALLOC is incompatible with PICO_W5100S_ARCH_THREADSAFE_BACKGROUND
#endif

static async_context_threadsafe_background_t W5100S_async_context_threadsafe_background;

async_context_t *W5100S_arch_init_default_async_context(void) {
    async_context_threadsafe_background_config_t config = async_context_threadsafe_background_default_config();
    if (async_context_threadsafe_background_init(&W5100S_async_context_threadsafe_background, &config))
        return &W5100S_async_context_threadsafe_background.core;
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
    ok &= lwip_nosys_init(context);
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
    lwip_nosys_deinit(context);
#endif
    // if it is our context, then we de-init it.
    if (context == &W5100S_async_context_threadsafe_background.core) {
        async_context_deinit(context);
        W5100S_arch_set_async_context(NULL);
    }
}

#endif
