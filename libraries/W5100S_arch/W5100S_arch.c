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

void W5100S_arch_set_async_context(async_context_t *context) {
    async_context = context;
}

async_context_t *W5100S_arch_async_context(void) {
    return async_context;
}

void W5100S_arch_poll(void)
{
    async_context_poll(async_context);
}

void W5100S_arch_wait_for_work_until(absolute_time_t until) {
    async_context_wait_for_work_until(async_context, until);
}
