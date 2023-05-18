#ifndef _W5100S_LWIP_H
#define _W5100S_LWIP_H

#ifdef __cplusplus
extern "C" {
#endif

#include "W5100S_driver.h"
#include "W5100S_arch.h"

void W5100S_cb_tcpip_init(W5100S_t *self);

void W5100S_cb_tcpip_deinit(W5100S_t *self);

void W5100S_cb_process_ethernet(void *cb_data, size_t len, const uint8_t *buf);

void W5100S_cb_tcpip_set_link_up(W5100S_t *self);

void W5100S_cb_tcpip_set_link_down(W5100S_t *self);

int W5100S_tcpip_link_status(W5100S_t *self);

static const char* W5100S_tcpip_link_status_name(int status);

#ifdef __cplusplus
}
#endif

#endif