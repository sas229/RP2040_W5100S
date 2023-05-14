#include "W5100S.h"

int W5100S_init(W5100S_t *state) {
    // Initialise WizNet chip.
    wizchip_spi_initialize();
    wizchip_cris_initialize();
    wizchip_reset();
    wizchip_initialize();
    wizchip_check();

    // Initialise lwip.
    // lwip_init();

    // Set ethernet chip MAC address.
    setSHAR(state->mac);
    if(ctlwizchip(CW_RESET_PHY, 0) < 0) {
        PICOLOG_ERROR("Reset PHY failed.");
        return -1;
    };

    // Add interface.
    netif_input_fn input_func = tcpip_input;
    netif_add(state->netif, IP4_ADDR_ANY, IP4_ADDR_ANY, IP4_ADDR_ANY, NULL, netif_initialize, input_func);
    state->netif->name[0] = 'e';
    state->netif->name[1] = '0';

    // Assign callbacks for link and status.
    netif_set_link_callback(state->netif, W5100S_link_callback);
    netif_set_status_callback(state->netif, W5100S_status_callback);

    // MACRAW socket open.
    if (socket(0, Sn_MR_MACRAW, 5001, 0x00) < 0) {
        PICOLOG_ERROR(" MACRAW socket open failed.");
        return false;
    }
    PICOLOG_DEBUG("W5100S chip initialised.");
}

bool W5100S_connect(W5100S_t *state) {
    absolute_time_t expiry = make_timeout_time_ms(state->connect_timeout);
    while(!W5100S_cable_connected(state) && get_absolute_time() < expiry) {
        if (W5100S_cable_connected(state)) {
            // Bring up interface.
            state->link_up = W5100S_bring_link_up(state);
        }
        W5100S_process(state);
    }
    // If timeout reached, emit warning on log.
    if (!state->link_up) {
        PICOLOG_WARNING("Ethernet cable not connected.");
    }
    state->connected = true;
    return state->connected;
}

bool W5100S_bring_link_up(W5100S_t *state) {
    // Bring up interface.
    netif_set_link_up(state->netif);
    netif_set_up(state->netif);

    // Set link status attribute. 
    state->link_up = true;
    
    // Start DHCP on interface.
    dhcp_start(state->netif);
    PICOLOG_DEBUG("Ethernet link up.");
    return true;
}

void W5100S_bring_link_down(W5100S_t *state) {
    // Stop DHCP on interface.
    dhcp_stop(state->netif);

    // Bring the interface down.
    netif_set_down(state->netif);
    netif_set_link_down(state->netif);

    // Set link status attribute.
    state->link_up = false;  
    PICOLOG_DEBUG("Ethernet link down.");
}

bool W5100S_cable_connected(W5100S_t *state) {
    uint8_t byte = WIZCHIP_READ(0x003C);
    int bit = 1;
    state->cable_connected = (bool)!(1 == ((byte >> bit) & 1));
    return state->cable_connected;
}

void W5100S_process(W5100S_t *state) {
    getsockopt(0, SO_RECVBUF, &state->pack_len);
    if (state->pack_len > 0) {
        state->pack_len = recv_lwip(0, (uint8_t *)state->pack, state->pack_len);
        if (state->pack_len) {
            state->p = pbuf_alloc(PBUF_RAW, state->pack_len, PBUF_POOL);
            pbuf_take(state->p, state->pack, state->pack_len);
        } else {
            PICOLOG_WARNING("No Ethernet packet received.");
        }
        if (state->pack_len && state->p != NULL) {
            LINK_STATS_INC(link.recv);
            if (state->netif->input(state->p, state->netif) != ERR_OK) {
                pbuf_free(state->p);
            }
        }
    }
}

void W5100S_poll(W5100S_t *state) {
    W5100S_cable_connected(state);
    // Check for DHCP IP address allocation.
    if (!state->dhcp_ip_allocated) {
        if (!ip4_addr_isany_val(ip_2_ip4(state->netif->ip_addr))) {
            PICOLOG_INFO("DHCP IP address allocated.");
            state->dhcp_ip_allocated = true;
        }
    }
    // If connected, either process, bring the link up or take it down.
    if (state->connected) {
        if (state->cable_connected && state->link_up) {
            W5100S_process(state);
        } else if (state->cable_connected && !state->link_up) {
            W5100S_bring_link_up(state);
        } else if (!state->cable_connected && state->link_up ) {
            W5100S_bring_link_down(state);
        } 
    }

    // // Cyclic lwip timers check.
    // sys_check_timeouts();
}

void W5100S_link_callback(struct netif *netif) {
    PICOLOG_INFO("Ethernet link %s.", netif_is_link_up(netif) ? "up" : "down");
}

void W5100S_status_callback(struct netif *netif) {
    if (!ip4_addr_isany_val(ip_2_ip4(netif->ip_addr))) {
        PICOLOG_INFO("Ethernet IP: %s", ip4addr_ntoa(netif_ip4_addr(netif)));
    } else {
        PICOLOG_DEBUG("Ethernet IP: %s", ip4addr_ntoa(netif_ip4_addr(netif)));
    }
}