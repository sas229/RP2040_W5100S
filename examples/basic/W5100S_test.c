#include <stdio.h>
#include "pico/stdlib.h"
#include "W5100S.h"
#include "picolog.h"

W5100S_t W5100S_state;

static uint8_t MAC[6]; 

int main() {
    stdio_init_all();
    sleep_ms(3000);

    // Initialise logging.
    PICOLOG_INIT(PICOLOG_TRACE_LEVEL);

    W5100S_state.spi_frequency = 36 * 1000 * 1000;
    W5100S_state.dhcp_ip_allocated = false;
    W5100S_state.socket = 0;

    W5100S_init(&W5100S_state);
    W5100S_connect(&W5100S_state);

    while (true) {
        W5100s_arch_poll();
    }
    return 0;
}