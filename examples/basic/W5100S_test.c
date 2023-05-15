#include <stdio.h>
#include "pico/stdlib.h"
#include "W5100S.h"
#include "picolog.h"

W5100S_t W5100S_state;

static void W5100S_do_work(async_context_t *context, async_at_time_worker_t *worker);
async_at_time_worker_t print_timeout = { .do_work = W5100S_do_work };

static void W5100S_do_work(async_context_t *context, async_at_time_worker_t *worker) {
    // printf("In async_context timeout...\n");
    W5100S_poll(&W5100S_state);
    async_context_add_at_time_worker_in_ms(context, worker, 1);
}

int main() {
    stdio_init_all();
    sleep_ms(3000);

    // Initialise logging.
    PICOLOG_INIT(PICOLOG_TRACE_LEVEL);

    W5100S_state.mac[0] = 0x00;
    W5100S_state.mac[1] = 0x08;
    W5100S_state.mac[2] = 0xDC;
    W5100S_state.mac[3] = 0x12;
    W5100S_state.mac[4] = 0x34;
    W5100S_state.mac[5] = 0x56;
    W5100S_state.spi_frequency = 36 * 1000 * 1000;

    W5100S_init(&W5100S_state);
    W5100S_connect(&W5100S_state);
    async_context_add_at_time_worker_in_ms(W5100S_get_async_context(), &print_timeout, 500);

    while (true) {
        printf("Hello, world!\n");
        sleep_ms(2000);
        W5100s_arch_poll();
    }
    return 0;
}