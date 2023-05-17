/*
 * Copyright (c) 2022 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdlib.h>
#include <stdio.h>
#include <string.h>

#include "pico/unique_id.h"
#include "W5100S_driver.h"
#include "W5100S_arch.h"

#if W5100S_ARCH_DEBUG_ENABLED
#define W5100S_ARCH_DEBUG(...) printf(__VA_ARGS__)
#else
#define W5100S_ARCH_DEBUG(...) ((void)0)
#endif

static async_context_t *async_context;

extern W5100S_t W5100S_state;

extern async_context_t W5100S_async_context;

extern async_at_time_worker_t sleep_timeout_worker;

void W5100S_arch_set_async_context(async_context_t *context) {
    async_context = context;
}

async_context_t *W5100S_arch_async_context(void) {
    return async_context;
}

void W5100S_arch_poll(void) {
    async_context_poll(async_context);
}

void W5100S_arch_wait_for_work_until(absolute_time_t until) {
    async_context_wait_for_work_until(async_context, until);
}

bool W5100S_arch_eth_connect(void) {
    W5100S_t *self = &W5100S_state;
    absolute_time_t expiry = make_timeout_time_ms(self->connect_timeout);
    while(!W5100S_cable_connected(self) && get_absolute_time() < expiry) {
        if (W5100S_cable_connected(self)) {
            // Bring up interface.
            self->link_up = W5100S_bring_link_up(self);
        }
        W5100S_process(self);
    }
    // If timeout reached, emit warning on log.
    if (!self->link_up) {
        PICOLOG_WARNING("Ethernet cable not connected.");
    }
    self->connected = true;
    return self->connected;
}