#include <stdio.h>
#include "pico/stdlib.h"
#include "W5100S.h"
#include "picolog.h"

W5100S_t W5100S_state;

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

    while (true) {
        printf("Hello, world!\n");
        sleep_ms(2000);
        W5100s_arch_poll();
    }
    return 0;
}