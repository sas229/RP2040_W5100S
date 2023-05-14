#ifndef W5100S_H
#define W5100S_H

#include "picolog.h"
#include "port_common.h"
#include "wizchip_conf.h"
#include "socket.h"
#include "w5x00_spi.h"
#include "w5x00_lwip.h"

#include "lwip/init.h"
#include "lwip/netif.h"
#include "lwip/timeouts.h"
#include "lwip/apps/lwiperf.h"
#include "lwip/etharp.h"
#include "lwip/dhcp.h"
#include "lwip/dns.h"
#include "lwip/tcpip.h"

typedef struct _W5100S_t  {
    uint8_t mac[6];
    struct netif *netif;
    uint8_t *pack;
    uint16_t pack_len;
    struct pbuf *p;
    int connect_timeout;
    bool cable_connected;
    bool link_up;
    bool connected;
    bool dhcp_ip_allocated;
} W5100S_t;

/** @brief Initialisation function. */
int W5100S_init(W5100S_t *state);

/** @brief Connect function. */
bool W5100S_connect(W5100S_t *state);

/** @brief Check if cable is connected. */
bool W5100S_cable_connected(W5100S_t *state);

/** @brief Process any data. */
void W5100S_process(W5100S_t *state);

/** @brief Poll for work. */
void W5100S_poll(W5100S_t *state);

/** @brief Bring the link up.*/
bool W5100S_bring_link_up(W5100S_t *state);

/** @brief Bring the link down.*/
void W5100S_bring_link_down(W5100S_t *state);

/** @brief Netif status callback. */
void W5100S_status_callback(struct netif *netif);

/** @brief Netif link callback. */
void W5100S_link_callback(struct netif *netif);

#endif