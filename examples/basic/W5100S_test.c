#include <stdio.h>
#include "pico/stdlib.h"
#include "W5100S.h"
#include "picolog.h"

int main() {
    stdio_init_all();
    sleep_ms(3000);

    // Initialise logging.
    PICOLOG_INIT(PICOLOG_TRACE_LEVEL);

    W5100S_init(&W5100S_state);
    W5100S_connect(&W5100S_state);

    while (true) {
        W5100s_arch_poll();
    }
    return 0;
}