#include "W5100S_lwip.h"

void W5100S_cb_tcpip_init(W5100S_t *self) {
    // Add interface.
    #if NO_SYS
    netif_input_fn input_func = netif_input;
    #else
    netif_input_fn input_func = tcpip_input;
    #endif
    netif_add(self->netif, IP4_ADDR_ANY, IP4_ADDR_ANY, IP4_ADDR_ANY, NULL, W5100S_netif_init, input_func);
    self->netif->name[0] = 'e';
    self->netif->name[1] = '0';
    PICOLOG_DEBUG("W5100S network interface added.");

    // Assign callbacks for link and status.
    netif_set_link_callback(self->netif, W5100S_link_callback);
    netif_set_status_callback(self->netif, W5100S_status_callback);
    PICOLOG_DEBUG("W5100S link and status callbacks added.");
}

void W5100S_cb_tcpip_deinit(W5100S_t *self) {
    struct netif *n = self->netif;
    #if LWIP_IPV4
    dhcp_stop(n);
    #endif
    for (struct netif *netif = netif_list; netif != NULL; netif = netif->next) {
        if (netif == n) {
            netif_remove(netif);
            #if LWIP_IPV4
            ip_2_ip4(&netif->ip_addr)->addr = 0;
            #endif
            netif->flags = 0;
        }
    }
}

void W5100S_cb_process_ethernet(void *cb_data, size_t len, const uint8_t *buf) {
    W5100S_t *self = cb_data;
    struct netif *netif = self->netif;
    if (netif->flags & NETIF_FLAG_LINK_UP) {
        struct pbuf *p = pbuf_alloc(PBUF_RAW, len, PBUF_POOL);
        if (p != NULL) {
            pbuf_take(p, buf, len);
            if (netif->input(p, netif) != ERR_OK) {
                pbuf_free(p);
            }
        }
    }
}

void W5100S_cb_tcpip_set_link_up(W5100S_t *self) {
    netif_set_link_up(self->netif);
}

void W5100S_cb_tcpip_set_link_down(W5100S_t *self) {
    netif_set_link_down(self->netif);
}

int W5100S_tcpip_link_status(W5100S_t *self) {
    struct netif *netif = self->netif;
    if ((netif->flags & (NETIF_FLAG_UP | NETIF_FLAG_LINK_UP)) == (NETIF_FLAG_UP | NETIF_FLAG_LINK_UP)) {
        bool have_address = false;
        #if LWIP_IPV4
        have_address = (ip_2_ip4(&netif->ip_addr)->addr != 0);
        #else
        for (int i = 0; i < LWIP_IPV6_NUM_ADDRESSES; i++) {
            int state = netif_ip6_addr_state(netif, i);
            const ip6_addr_t *addr = netif_ip6_addr(netif, i);
            if (ip6_addr_ispreferred(state) && ip6_addr_isglobal(addr)) {
                have_address = true;
                break;
            }
        }
        #endif
        if (have_address) {
            self->status = W5100S_LINK_UP;
            return self->status;
        } else {
            self->status = W5100S_LINK_NOIP;
            return self->status;
        }
    } else {
        self->status = W5100S_arch_eth_link_status(self);
        return self->status;
    }
}

static const char* W5100S_tcpip_link_status_name(int status) {
    switch (status) {
    case W5100S_LINK_DISCONNECTED:
        return "Interface disconnected.";
    case W5100S_LINK_CONNECTED:
        return "Interface up, but cable disconnected.";
    case W5100S_LINK_NOIP:
        return "Cable connected, but no IP address.";
    case W5100S_LINK_UP:
        return "Cable connected with an IP address.";
    case W5100S_LINK_FAIL:
        return "Connection failed.";
    }
    return "Unknown!";
}
