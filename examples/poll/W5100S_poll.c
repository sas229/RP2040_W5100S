#include <stdio.h>
#include "pico/stdlib.h"
#include "W5100S_arch.h"
#include "picolog.h"

W5100S_t W5100S_state = {
    .spi_frequency = 36*1000*1000,
    .dhcp_ip_allocated = false,
    .socket = 0,
    .p = NULL,
    .pack_len = 0,
};

int main() {
    stdio_init_all();
    sleep_ms(3000);

    // Initialise logging.
    PICOLOG_INIT(PICOLOG_TRACE_LEVEL);

    // Initialise W5100S.
    W5100S_arch_init();
    W5100S_arch_eth_connect();

    while (true) {
        W5100S_arch_poll();
        // printf("Hello loop...\n");
        sleep_ms(50);
    }
    return 0;
}