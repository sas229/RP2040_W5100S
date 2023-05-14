#include <stdio.h>
#include "pico/stdlib.h"
#include "W5100S.h"

W5100S_t *W5100S_state;

int main() {
    W5100S_init();
    setup_default_uart();
    printf("Hello, world!\n");
    return 0;
}