#include "W5100S.h"

static void (*callback_ptr)(void);
static struct repeating_timer g_timer;

static async_context_poll_t W5100S_async_context_poll;
static async_context_t *W5100S_async_context;

static async_when_pending_worker_t W5100S_poll_worker = {
        .do_work = W5100S_do_poll
};

static async_at_time_worker_t sleep_timeout_worker = {
        .do_work = W5100S_sleep_timeout_reached
};

int W5100S_init(W5100S_t *self) {
    // Initialise WizNet chip.
    PICOLOG_INFO("Initialising W5100S chip.");
    W5100S_spi_init(self);
    W5100S_dma_init(self);
    W5100S_critical_section_init(self);
    W5100S_reset(self);
    W5100S_chip_init(self);
    W5100S_version_check(self);

    // Initialise lwip.
    // Do the following only if this is a Pico board, rather than Pico W. Needs some define logic or similar.
    // Currently only implemented for poll method. Threadsafe background and FreeRTOS methods to be added.
    async_context_poll_init_with_defaults(&W5100S_async_context_poll);
    W5100S_async_context = &W5100S_async_context_poll.core;
    bool ok = W5100S_driver_init(W5100S_async_context);  
    ok &= lwip_nosys_init(W5100S_async_context);
    PICOLOG_DEBUG("W5100S context initialised lwip.");

    // Set ethernet chip MAC address.
    setSHAR(self->mac);
    if(ctlwizchip(CW_RESET_PHY, 0) < 0) {
        PICOLOG_ERROR("Reset PHY failed.");
        return -1;
    };
    PICOLOG_DEBUG("W5100S MAC address set.");

    // Add interface.
    // netif_input_fn input_func = tcpip_input;
    // netif_add(self->netif, IP4_ADDR_ANY, IP4_ADDR_ANY, IP4_ADDR_ANY, NULL, netif_init, input_func);
    netif_add(self->netif, IP4_ADDR_ANY, IP4_ADDR_ANY, IP4_ADDR_ANY, NULL, W5100S_netif_init, netif_input);
    self->netif->name[0] = 'e';
    self->netif->name[1] = '0';
    PICOLOG_DEBUG("W5100S network interface added.");

    // Assign callbacks for link and status.
    netif_set_link_callback(self->netif, W5100S_link_callback);
    netif_set_status_callback(self->netif, W5100S_status_callback);
    PICOLOG_DEBUG("W5100S link and status callbacks added.");

    // MACRAW socket open.
    if (socket(0, Sn_MR_MACRAW, 5001, 0x00) < 0) {
        PICOLOG_ERROR(" MACRAW socket open failed.");
        return false;
    }
    PICOLOG_INFO("W5100S initialised.");
}

void W5100S_deinit(W5100S_t *self) {
    PICOLOG_INFO("W5100S deinitialised.");
}

bool W5100S_connect(W5100S_t *self) {
    absolute_time_t expiry = make_timeout_time_ms(self->connect_timeout);
    while(!W5100S_cable_connected(self) && get_absolute_time() < expiry) {
        if (W5100S_cable_connected(self)) {
            // Bring up interface.
            self->link_up = W5100S_bring_link_up(self);
        }
        W5100S_process(self);
    }
    // If timeout reached, emit warning on log.
    if (!self->link_up) {
        PICOLOG_WARNING("Ethernet cable not connected.");
    }
    self->connected = true;
    return self->connected;
}

bool W5100S_bring_link_up(W5100S_t *self) {
    // Bring up interface.
    netif_set_link_up(self->netif);
    netif_set_up(self->netif);

    // Set link status attribute. 
    self->link_up = true;
    
    // Start DHCP on interface.
    dhcp_start(self->netif);
    PICOLOG_DEBUG("Ethernet link up.");
    return true;
}

void W5100S_bring_link_down(W5100S_t *self) {
    // Stop DHCP on interface.
    dhcp_stop(self->netif);

    // Bring the interface down.
    netif_set_down(self->netif);
    netif_set_link_down(self->netif);

    // Set link status attribute.
    self->link_up = false;  
    PICOLOG_DEBUG("Ethernet link down.");
}

bool W5100S_cable_connected(W5100S_t *self) {
    uint8_t byte = WIZCHIP_READ(0x003C);
    int bit = 1;
    self->cable_connected = (bool)!(1 == ((byte >> bit) & 1));
    return self->cable_connected;
}

void W5100S_process(W5100S_t *self) {
    getsockopt(0, SO_RECVBUF, &self->pack_len);
    if (self->pack_len > 0) {
        self->pack_len = W5100S_recv_lwip(0, (uint8_t *)self->pack, self->pack_len);
        if (self->pack_len) {
            self->p = pbuf_alloc(PBUF_RAW, self->pack_len, PBUF_POOL);
            pbuf_take(self->p, self->pack, self->pack_len);
        } else {
            PICOLOG_WARNING("No Ethernet packet received.");
        }
        if (self->pack_len && self->p != NULL) {
            LINK_STATS_INC(link.recv);
            if (self->netif->input(self->p, self->netif) != ERR_OK) {
                pbuf_free(self->p);
            }
        }
    }
    PICOLOG_TRACE("In W5100S_process call.");
}

void W5100S_poll(W5100S_t *self) {
    W5100S_cable_connected(self);
    // Check for DHCP IP address allocation.
    if (!self->dhcp_ip_allocated) {
        if (!ip4_addr_isany_val(ip_2_ip4(self->netif->ip_addr))) {
            PICOLOG_INFO("DHCP IP address allocated.");
            self->dhcp_ip_allocated = true;
        }
    }
    // If connected, either process, bring the link up or take it down.
    if (self->connected) {
        if (self->cable_connected && self->link_up) {
            W5100S_process(self);
        } else if (self->cable_connected && !self->link_up) {
            W5100S_bring_link_up(self);
        } else if (!self->cable_connected && self->link_up ) {
            W5100S_bring_link_down(self);
        } 
    }

    // Cyclic lwip timers check.
    sys_check_timeouts();
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

void W5100S_reset(W5100S_t *self) {
    gpio_set_dir(PIN_RST, GPIO_OUT);
    gpio_put(PIN_RST, 0);
    sleep_ms(100);
    gpio_put(PIN_RST, 1);
    sleep_ms(100);
    PICOLOG_DEBUG("W5100S chip reset.");
}

static inline void W5100S_select() {
    gpio_put(PIN_CS, 0);
}

static inline void W5100S_deselect() {
    gpio_put(PIN_CS, 1);
}

static uint8_t W5100S_read(void) {
    uint8_t rx_data = 0;
    uint8_t tx_data = 0xFF;
    spi_read_blocking(SPI_PORT, tx_data, &rx_data, 1);
    return rx_data;
}

static void W5100S_write(uint8_t tx_data) {
    spi_write_blocking(SPI_PORT, &tx_data, 1);
}

static void W5100S_read_burst(uint8_t *pBuf, uint16_t len)
{
    W5100S_t *self = &W5100S_state;
    uint8_t dummy_data = 0xFF;

    channel_config_set_read_increment(&self->dma_channel_config_tx, false);
    channel_config_set_write_increment(&self->dma_channel_config_tx, false);
    dma_channel_configure(self->dma_tx, &self->dma_channel_config_tx,
                          &spi_get_hw(SPI_PORT)->dr, // Write address.
                          &dummy_data,               // Read address.
                          len,                       // Element count (each element is of size transfer_data_size).
                          false);                    // Don't start yet.

    channel_config_set_read_increment(&self->dma_channel_config_rx, false);
    channel_config_set_write_increment(&self->dma_channel_config_rx, true);
    dma_channel_configure(self->dma_rx, &self->dma_channel_config_rx,
                          pBuf,                      // Write address.
                          &spi_get_hw(SPI_PORT)->dr, // Read address.
                          len,                       // Element count (each element is of size transfer_data_size).
                          false);                    // Don't start yet.

    dma_start_channel_mask((1u << self->dma_tx) | (1u << self->dma_rx));
    dma_channel_wait_for_finish_blocking(self->dma_rx);
}

static void W5100S_write_burst(uint8_t *pBuf, uint16_t len)
{   
    W5100S_t *self = &W5100S_state;
    uint8_t dummy_data;

    channel_config_set_read_increment(&self->dma_channel_config_tx, true);
    channel_config_set_write_increment(&self->dma_channel_config_tx, false);
    dma_channel_configure(self->dma_tx, &self->dma_channel_config_tx,
                          &spi_get_hw(SPI_PORT)->dr, // Write address.
                          pBuf,                      // Read address.
                          len,                       // Element count (each element is of size transfer_data_size).
                          false);                    // Don't start yet.

    channel_config_set_read_increment(&self->dma_channel_config_rx, false);
    channel_config_set_write_increment(&self->dma_channel_config_rx, false);
    dma_channel_configure(self->dma_rx, &self->dma_channel_config_rx,
                          &dummy_data,               // Erite address.
                          &spi_get_hw(SPI_PORT)->dr, // Read address.
                          len,                       // Element count (each element is of size transfer_data_size).
                          false);                    // Don't start yet.

    dma_start_channel_mask((1u << self->dma_tx) | (1u << self->dma_rx));
    dma_channel_wait_for_finish_blocking(self->dma_rx);
}

static void W5100S_critical_section_lock()
{   
    W5100S_t *self = &W5100S_state;
    critical_section_enter_blocking(&self->critical_section);
}

static void W5100S_critical_section_unlock()
{   
    W5100S_t *self = &W5100S_state;
    critical_section_exit(&self->critical_section);
}

void W5100S_spi_init(W5100S_t *self)
{   
    // SPI interface.
    spi_init(SPI_PORT, self->spi_frequency);

    // SPI pin configuration.
    gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);
    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);

    // Make the SPI pins available to picotool.
    bi_decl(bi_3pins_with_func(PIN_MISO, PIN_MOSI, PIN_SCK, GPIO_FUNC_SPI));

    // Chip select is active-low, so we'll initialise it to a driven-high state.
    gpio_init(PIN_CS);
    gpio_set_dir(PIN_CS, GPIO_OUT);
    gpio_put(PIN_CS, 1);
    PICOLOG_DEBUG("W5100S SPI initialised at %i MHz.", self->spi_frequency/1000000);
}

void W5100S_dma_init(W5100S_t *self) {
    // Enable DMA support.
    self->dma_tx = dma_claim_unused_channel(true);
    self->dma_rx = dma_claim_unused_channel(true);

    // Transmit channel.
    self->dma_channel_config_tx = dma_channel_get_default_config(self->dma_tx);
    channel_config_set_transfer_data_size(&self->dma_channel_config_tx, DMA_SIZE_8);
    channel_config_set_dreq(&self->dma_channel_config_tx, DREQ_SPI0_TX);

    // Receive channel.
    self->dma_channel_config_rx = dma_channel_get_default_config(self->dma_rx);
    
    // We set the inbound DMA to transfer from the SPI receive FIFO
    // to a memory buffer paced by the SPI RX FIFO DREQ.
    channel_config_set_transfer_data_size(&self->dma_channel_config_rx, DMA_SIZE_8);
    channel_config_set_dreq(&self->dma_channel_config_rx, DREQ_SPI0_RX);

    // We configure the read address to remain unchanged for each element, but the write
    // address to increment (so data is written throughout the buffer).
    channel_config_set_read_increment(&self->dma_channel_config_rx, false);
    channel_config_set_write_increment(&self->dma_channel_config_rx, true);
    PICOLOG_DEBUG("W5100S DMA initialised.");
}

void W5100S_critical_section_init(W5100S_t *self)
{   
    // Initialise critical section lock.
    critical_section_init(&self->critical_section);
    reg_wizchip_cris_cbfunc(W5100S_critical_section_lock, W5100S_critical_section_unlock);
    PICOLOG_DEBUG("W5100S critical section initialised.");
}

void W5100S_chip_init(W5100S_t *self)
{
    // Deselect the FLASH : chip select high.
    W5100S_deselect(self);

    // CS function register.
    reg_wizchip_cs_cbfunc(W5100S_select, W5100S_deselect);

    // SPI function register.
    reg_wizchip_spi_cbfunc(W5100S_read, W5100S_write);
    reg_wizchip_spiburst_cbfunc(W5100S_read_burst, W5100S_write_burst);

    // W5100S chip initialisation.
    uint8_t temp;
    uint8_t memsize[2][4] = {{8, 0, 0, 0}, {8, 0, 0, 0}};
    if (ctlwizchip(CW_INIT_WIZCHIP, (void *)memsize) == -1) {
        PICOLOG_ERROR(" W5100S initialisation failed.");
        return;
    }
    PICOLOG_DEBUG("W5100S chip initialised.");
}

void W5100S_version_check(W5100S_t *self) {
    /* Read version register */
    if (getVER() != 0x51) {
        PICOLOG_ERROR("Version error: VERSION != 0x51; VERSION = 0x%02x.", getVER());
        while (1);
    }
}

int32_t W5100S_send_lwip(uint8_t sn, uint8_t *buf, uint16_t len)
{
    uint8_t tmp = 0;
    uint16_t freesize = 0;

    tmp = getSn_SR(sn);

    freesize = getSn_TxMAX(sn);
    if (len > freesize) {
        len = freesize; // Check size does not exceed MAX size.
    }

    wiz_send_data(sn, buf, len);
    setSn_CR(sn, Sn_CR_SEND);
    while (getSn_CR(sn));
    while (1) {
        uint8_t IRtemp = getSn_IR(sn);
        if (IRtemp & Sn_IR_SENDOK) {
            setSn_IR(sn, Sn_IR_SENDOK);
            break;
        } else if (IRtemp & Sn_IR_TIMEOUT) {
            setSn_IR(sn, Sn_IR_TIMEOUT);
            return -1;
        }
    }
    return (int32_t)len;
}

int32_t W5100S_recv_lwip(uint8_t sn, uint8_t *buf, uint16_t len)
{
    uint8_t head[2];
    uint16_t pack_len = 0;
    pack_len = getSn_RX_RSR(sn);
    if (pack_len > 0) {
        wiz_recv_data(sn, head, 2);
        setSn_CR(sn, Sn_CR_RECV);

        // Byte size of data packet (2 byte).
        pack_len = head[0];
        pack_len = (pack_len << 8) + head[1];
        pack_len -= 2;

        if (pack_len > len) {
            // Packet is bigger than buffer; drop the packet.
            wiz_recv_ignore(sn, pack_len);
            setSn_CR(sn, Sn_CR_RECV);
            return 0;
        }

        wiz_recv_data(sn, buf, pack_len); // Data copy.
        setSn_CR(sn, Sn_CR_RECV);
    }

    return (int32_t)pack_len;
}

err_t W5100S_netif_output(struct netif *netif, struct pbuf *p)
{
    W5100S_t *self = &W5100S_state;
    uint32_t send_len = 0;
    uint32_t tot_len = 0;
    memset(&self->tx_frame, 0x00, sizeof(self->tx_frame));
    for (struct pbuf *q = p; q != NULL; q = q->next) {
        memcpy(&self->tx_frame + tot_len, q->payload, q->len);
        tot_len += q->len;
        if (q->len == q->tot_len) {
            break;
        }
    }
    if (tot_len < 60) {
        // Pad.
        tot_len = 60;
    }
    uint32_t crc = W5100S_ethernet_frame_crc(&self->tx_frame, tot_len);
    send_len = W5100S_send_lwip(0, &self->tx_frame, tot_len);
    return ERR_OK;
}

err_t W5100S_netif_init(struct netif *netif)
{   
    W5100S_t *self = &W5100S_state;
    netif->linkoutput = W5100S_netif_output;
    netif->output = etharp_output;
    netif->mtu = ETHERNET_MTU;
    netif->flags = NETIF_FLAG_BROADCAST | NETIF_FLAG_ETHARP | NETIF_FLAG_ETHERNET | NETIF_FLAG_IGMP | NETIF_FLAG_MLD6;
    SMEMCPY(netif->hwaddr, self->mac, sizeof(netif->hwaddr));
    netif->hwaddr_len = sizeof(netif->hwaddr);
    return ERR_OK;
}

static uint32_t W5100S_ethernet_frame_crc(const uint8_t *data, int length) {
    uint32_t crc = 0xffffffff; /* Initial value. */
    uint32_t ethernet_polynomial_le = 0xedb88320U;
    while (--length >= 0) {
        uint8_t current_octet = *data++;
        for (int bit = 8; --bit >= 0; current_octet >>= 1) {
            if ((crc ^ current_octet) & 1) {
                crc >>= 1;
                crc ^= ethernet_polynomial_le;
            } else {
                crc >>= 1;
            }
        }
    }
    return ~crc;
}

void W5100S_gpio_interrupt_init(uint8_t socket, void (*callback)(void)) {
    uint16_t reg_val;
    int ret_val;
    reg_val = (SIK_CONNECTED | SIK_DISCONNECTED | SIK_RECEIVED | SIK_TIMEOUT); // Except SendOK.
    ret_val = ctlsocket(socket, CS_SET_INTMASK, (void *)&reg_val);
    reg_val = (1 << socket);
    ret_val = ctlwizchip(CW_SET_INTRMASK, (void *)&reg_val);
    callback_ptr = callback;
    gpio_set_irq_enabled_with_callback(PIN_INT, GPIO_IRQ_EDGE_FALL, true, &W5100S_gpio_interrupt_callback);
}

static void W5100S_gpio_interrupt_callback(uint gpio, uint32_t events) {
    if (callback_ptr != NULL) {
        callback_ptr();
    }
}

void W5100S_1ms_timer_init(void (*callback)(void)) {
    callback_ptr = callback;
    add_repeating_timer_us(-1000, W5100S_1ms_timer_callback, NULL, &g_timer);
}

bool W5100S_1ms_timer_callback(struct repeating_timer *t) {
    if (callback_ptr != NULL) {
        callback_ptr();
    }
}

void W5100S_delay_ms(uint32_t ms) {
    sleep_ms(ms);
}

bool W5100S_driver_init(async_context_t *context) {
    // async_context_execute_sync(context, W5100S_irq_init, NULL);
    async_context_add_when_pending_worker(context, &W5100S_poll_worker);
    PICOLOG_DEBUG("W5100S driver async_context setup.");
    return true;
}

void cyw43_driver_deinit(async_context_t *context) {
    assert(context == W5100S_async_context);
    async_context_remove_at_time_worker(context, &sleep_timeout_worker);
    async_context_remove_when_pending_worker(context, &W5100S_poll_worker);
    // The IRQ IS on the same core as the context, so must be de-initialized there.
    // async_context_execute_sync(context, W5100S_irq_deinit, NULL);
    W5100S_deinit(&W5100S_state);
    W5100S_async_context = NULL;
}

uint32_t W5100S_irq_init(__unused void *param) {
#ifndef NDEBUG
    assert(get_core_num() == async_context_core_num(W5100S_async_context));
#endif
    // gpio_add_raw_irq_handler_with_order_priority(CYW43_PIN_WL_HOST_WAKE, cyw43_gpio_irq_handler, CYW43_GPIO_IRQ_HANDLER_PRIORITY);
    // W5100S_set_irq_enabled(true);
    // irq_set_enabled(IO_IRQ_BANK0, true);
    return 0;
}

uint32_t W5100S_irq_deinit(__unused void *param) {
#ifndef NDEBUG
    assert(get_core_num() == async_context_core_num(W5100S_async_context));
#endif
    // gpio_remove_raw_irq_handler(CYW43_PIN_WL_HOST_WAKE, cyw43_gpio_irq_handler);
    // cyw43_set_irq_enabled(false);
    return 0;
}

static void W5100S_do_poll(async_context_t *context, __unused async_when_pending_worker_t *worker) {
#ifndef NDEBUG
    assert(get_core_num() == async_context_core_num(context));
#endif
    PICOLOG_DEBUG("In polling function via async_context.");
    W5100S_t *self = &W5100S_state;
    W5100S_poll(self);
}

static void W5100S_sleep_timeout_reached(async_context_t *context, __unused async_at_time_worker_t *worker) {
    assert(context == W5100S_async_context);
    assert(worker == &sleep_timeout_worker);
    async_context_set_work_pending(context, &W5100S_poll_worker);
}

void W5100s_arch_poll() {
    async_context_poll(W5100S_async_context);
}


