#include "W5100S.h"

static void (*callback_ptr)(void);
static struct repeating_timer g_timer;

int W5100S_init() {
    // Initialise WizNet chip.
    W5100S_spi_initialize();
    W5100S_cris_initialize();
    W5100S_reset();
    W5100S_initialize();
    W5100S_check();

    // // Initialise lwip.
    // // lwip_init();

    // // Set ethernet chip MAC address.
    // setSHAR(W5100S_state->mac);
    // if(ctlwizchip(CW_RESET_PHY, 0) < 0) {
    //     PICOLOG_ERROR("Reset PHY failed.");
    //     return -1;
    // };

    // // Add interface.
    // // netif_input_fn input_func = tcpip_input;
    // // netif_add(W5100S_state->netif, IP4_ADDR_ANY, IP4_ADDR_ANY, IP4_ADDR_ANY, NULL, netif_initialize, input_func);
    // netif_add(W5100S_state->netif, IP4_ADDR_ANY, IP4_ADDR_ANY, IP4_ADDR_ANY, NULL, W5100S_netif_initialize, NULL);
    // W5100S_state->netif->name[0] = 'e';
    // W5100S_state->netif->name[1] = '0';

    // // Assign callbacks for link and status.
    // netif_set_link_callback(W5100S_state->netif, W5100S_link_callback);
    // netif_set_status_callback(W5100S_state->netif, W5100S_status_callback);

    // // MACRAW socket open.
    // if (socket(0, Sn_MR_MACRAW, 5001, 0x00) < 0) {
    //     PICOLOG_ERROR(" MACRAW socket open failed.");
    //     return false;
    // }
    // PICOLOG_DEBUG("W5100S chip initialised.");
}

bool W5100S_connect() {
    absolute_time_t expiry = make_timeout_time_ms(W5100S_state->connect_timeout);
    while(!W5100S_cable_connected() && get_absolute_time() < expiry) {
        if (W5100S_cable_connected()) {
            // Bring up interface.
            W5100S_state->link_up = W5100S_bring_link_up();
        }
        W5100S_process();
    }
    // If timeout reached, emit warning on log.
    if (!W5100S_state->link_up) {
        PICOLOG_WARNING("Ethernet cable not connected.");
    }
    W5100S_state->connected = true;
    return W5100S_state->connected;
}

bool W5100S_bring_link_up() {
    // Bring up interface.
    netif_set_link_up(W5100S_state->netif);
    netif_set_up(W5100S_state->netif);

    // Set link status attribute. 
    W5100S_state->link_up = true;
    
    // Start DHCP on interface.
    dhcp_start(W5100S_state->netif);
    PICOLOG_DEBUG("Ethernet link up.");
    return true;
}

void W5100S_bring_link_down() {
    // Stop DHCP on interface.
    dhcp_stop(W5100S_state->netif);

    // Bring the interface down.
    netif_set_down(W5100S_state->netif);
    netif_set_link_down(W5100S_state->netif);

    // Set link status attribute.
    W5100S_state->link_up = false;  
    PICOLOG_DEBUG("Ethernet link down.");
}

bool W5100S_cable_connected() {
    uint8_t byte = WIZCHIP_READ(0x003C);
    int bit = 1;
    W5100S_state->cable_connected = (bool)!(1 == ((byte >> bit) & 1));
    return W5100S_state->cable_connected;
}

void W5100S_process() {
    getsockopt(0, SO_RECVBUF, &W5100S_state->pack_len);
    if (W5100S_state->pack_len > 0) {
        W5100S_state->pack_len = recv_lwip(0, (uint8_t *)W5100S_state->pack, W5100S_state->pack_len);
        if (W5100S_state->pack_len) {
            W5100S_state->p = pbuf_alloc(PBUF_RAW, W5100S_state->pack_len, PBUF_POOL);
            pbuf_take(W5100S_state->p, W5100S_state->pack, W5100S_state->pack_len);
        } else {
            PICOLOG_WARNING("No Ethernet packet received.");
        }
        if (W5100S_state->pack_len && W5100S_state->p != NULL) {
            LINK_STATS_INC(link.recv);
            if (W5100S_state->netif->input(W5100S_state->p, W5100S_state->netif) != ERR_OK) {
                pbuf_free(W5100S_state->p);
            }
        }
    }
}

void W5100S_poll() {
    W5100S_cable_connected();
    // Check for DHCP IP address allocation.
    if (!W5100S_state->dhcp_ip_allocated) {
        if (!ip4_addr_isany_val(ip_2_ip4(W5100S_state->netif->ip_addr))) {
            PICOLOG_INFO("DHCP IP address allocated.");
            W5100S_state->dhcp_ip_allocated = true;
        }
    }
    // If connected, either process, bring the link up or take it down.
    if (W5100S_state->connected) {
        if (W5100S_state->cable_connected && W5100S_state->link_up) {
            W5100S_process();
        } else if (W5100S_state->cable_connected && !W5100S_state->link_up) {
            W5100S_bring_link_up();
        } else if (!W5100S_state->cable_connected && W5100S_state->link_up ) {
            W5100S_bring_link_down();
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

void W5100S_reset() {
    gpio_set_dir(PIN_RST, GPIO_OUT);
    gpio_put(PIN_RST, 0);
    sleep_ms(100);
    gpio_put(PIN_RST, 1);
    sleep_ms(100);
    bi_decl(bi_1pin_with_name(PIN_RST, "W5x00 RESET"));
}

static inline void W5100S_select(void)
{
    gpio_put(PIN_CS, 0);
}

static inline void W5100S_deselect(void)
{
    gpio_put(PIN_CS, 1);
}

static uint8_t W5100S_read(void)
{
    uint8_t rx_data = 0;
    uint8_t tx_data = 0xFF;

    spi_read_blocking(SPI_PORT, tx_data, &rx_data, 1);

    return rx_data;
}

static void W5100S_write(uint8_t tx_data)
{
    spi_write_blocking(SPI_PORT, &tx_data, 1);
}

static void W5100S_read_burst(uint8_t *pBuf, uint16_t len)
{
    uint8_t dummy_data = 0xFF;

    channel_config_set_read_increment(&W5100S_state->dma_channel_config_tx, false);
    channel_config_set_write_increment(&W5100S_state->dma_channel_config_tx, false);
    dma_channel_configure(W5100S_state->dma_tx, &W5100S_state->dma_channel_config_tx,
                          &spi_get_hw(SPI_PORT)->dr, // write address
                          &dummy_data,               // read address
                          len,                       // element count (each element is of size transfer_data_size)
                          false);                    // don't start yet

    channel_config_set_read_increment(&W5100S_state->dma_channel_config_rx, false);
    channel_config_set_write_increment(&W5100S_state->dma_channel_config_rx, true);
    dma_channel_configure(W5100S_state->dma_rx, &W5100S_state->dma_channel_config_rx,
                          pBuf,                      // write address
                          &spi_get_hw(SPI_PORT)->dr, // read address
                          len,                       // element count (each element is of size transfer_data_size)
                          false);                    // don't start yet

    dma_start_channel_mask((1u << W5100S_state->dma_tx) | (1u << W5100S_state->dma_rx));
    dma_channel_wait_for_finish_blocking(W5100S_state->dma_rx);
}

static void W5100S_write_burst(uint8_t *pBuf, uint16_t len)
{
    uint8_t dummy_data;

    channel_config_set_read_increment(&W5100S_state->dma_channel_config_tx, true);
    channel_config_set_write_increment(&W5100S_state->dma_channel_config_tx, false);
    dma_channel_configure(W5100S_state->dma_tx, &W5100S_state->dma_channel_config_tx,
                          &spi_get_hw(SPI_PORT)->dr, // write address
                          pBuf,                      // read address
                          len,                       // element count (each element is of size transfer_data_size)
                          false);                    // don't start yet

    channel_config_set_read_increment(&W5100S_state->dma_channel_config_rx, false);
    channel_config_set_write_increment(&W5100S_state->dma_channel_config_rx, false);
    dma_channel_configure(W5100S_state->dma_rx, &W5100S_state->dma_channel_config_rx,
                          &dummy_data,               // write address
                          &spi_get_hw(SPI_PORT)->dr, // read address
                          len,                       // element count (each element is of size transfer_data_size)
                          false);                    // don't start yet

    dma_start_channel_mask((1u << W5100S_state->dma_tx) | (1u << W5100S_state->dma_rx));
    dma_channel_wait_for_finish_blocking(W5100S_state->dma_rx);
}

static void W5100S_critical_section_lock(void)
{
    critical_section_enter_blocking(&W5100S_state->critical_section);
}

static void W5100S_critical_section_unlock(void)
{
    critical_section_exit(&W5100S_state->critical_section);
}

void W5100S_spi_initialize(void)
{
    // this example will use SPI0 at 36MHz
    spi_init(SPI_PORT, 36000 * 1000);

    gpio_set_function(PIN_SCK, GPIO_FUNC_SPI);
    gpio_set_function(PIN_MOSI, GPIO_FUNC_SPI);
    gpio_set_function(PIN_MISO, GPIO_FUNC_SPI);

    // make the SPI pins available to picotool
    bi_decl(bi_3pins_with_func(PIN_MISO, PIN_MOSI, PIN_SCK, GPIO_FUNC_SPI));

    // chip select is active-low, so we'll initialise it to a driven-high state
    gpio_init(PIN_CS);
    gpio_set_dir(PIN_CS, GPIO_OUT);
    gpio_put(PIN_CS, 1);

    // make the SPI pins available to picotool
    bi_decl(bi_1pin_with_name(PIN_CS, "W5x00 CHIP SELECT"));

    // Enable DMA support.
    W5100S_state->dma_tx = dma_claim_unused_channel(true);
    W5100S_state->dma_rx = dma_claim_unused_channel(true);

    W5100S_state->dma_channel_config_tx = dma_channel_get_default_config(W5100S_state->dma_tx);
    channel_config_set_transfer_data_size(&W5100S_state->dma_channel_config_tx, DMA_SIZE_8);
    channel_config_set_dreq(&W5100S_state->dma_channel_config_tx, DREQ_SPI0_TX);

    // We set the inbound DMA to transfer from the SPI receive FIFO to a memory buffer paced by the SPI RX FIFO DREQ
    // We coinfigure the read address to remain unchanged for each element, but the write
    // address to increment (so data is written throughout the buffer)
    W5100S_state->dma_channel_config_rx = dma_channel_get_default_config(W5100S_state->dma_rx);
    channel_config_set_transfer_data_size(&W5100S_state->dma_channel_config_rx, DMA_SIZE_8);
    channel_config_set_dreq(&W5100S_state->dma_channel_config_rx, DREQ_SPI0_RX);
    channel_config_set_read_increment(&W5100S_state->dma_channel_config_rx, false);
    channel_config_set_write_increment(&W5100S_state->dma_channel_config_rx, true);
}

void W5100S_cris_initialize(void)
{
    critical_section_init(&W5100S_state->critical_section);
    reg_wizchip_cris_cbfunc(W5100S_critical_section_lock, W5100S_critical_section_unlock);
}

void W5100S_initialize(void)
{
    /* Deselect the FLASH : chip select high */
    W5100S_deselect();

    /* CS function register */
    reg_wizchip_cs_cbfunc(W5100S_select, W5100S_deselect);

    /* SPI function register */
    reg_wizchip_spi_cbfunc(W5100S_read, W5100S_write);
    reg_wizchip_spiburst_cbfunc(W5100S_read_burst, W5100S_write_burst);

    /* W5100S initialize */
    uint8_t temp;
    uint8_t memsize[2][4] = {{8, 0, 0, 0}, {8, 0, 0, 0}};

    if (ctlwizchip(CW_INIT_WIZCHIP, (void *)memsize) == -1)
    {
        printf(" W5x00 initialized fail\n");

        return;
    }

    /* Commented the following section out so that the Ethernet chip 
    will initialise even if the ethernet cable is unplugged. */

    // /* Check PHY link status */
    // do
    // {
    //     if (ctlwizchip(CW_GET_PHYLINK, (void *)&temp) == -1)
    //     {
    //         printf(" Unknown PHY link status\n");

    //         return;
    //     }
    //     printf("Status: %s", temp);
    // } 
    // while (temp == PHY_LINK_OFF);
}

void W5100S_check(void)
{
    /* Read version register */
    if (getVER() != 0x51)
    {
        printf(" ACCESS ERR : VERSION != 0x51, read value = 0x%02x\n", getVER());

        while (1)
            ;
    }
}

int32_t W5100S_send_lwip(uint8_t sn, uint8_t *buf, uint16_t len)
{
    uint8_t tmp = 0;
    uint16_t freesize = 0;

    tmp = getSn_SR(sn);

    freesize = getSn_TxMAX(sn);
    if (len > freesize)
        len = freesize; // check size not to exceed MAX size.

    wiz_send_data(sn, buf, len);
    setSn_CR(sn, Sn_CR_SEND);
    while (getSn_CR(sn))
        ;

    while (1)
    {
        uint8_t IRtemp = getSn_IR(sn);
        if (IRtemp & Sn_IR_SENDOK)
        {
            setSn_IR(sn, Sn_IR_SENDOK);
            // printf("Packet sent ok\n");
            break;
        }
        else if (IRtemp & Sn_IR_TIMEOUT)
        {
            setSn_IR(sn, Sn_IR_TIMEOUT);
            // printf("Socket is closed\n");
            //  There was a timeout
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

    if (pack_len > 0)
    {
        wiz_recv_data(sn, head, 2);
        setSn_CR(sn, Sn_CR_RECV);

        // byte size of data packet (2byte)
        pack_len = head[0];
        pack_len = (pack_len << 8) + head[1];
        pack_len -= 2;

        if (pack_len > len)
        {
            // Packet is bigger than buffer - drop the packet
            wiz_recv_ignore(sn, pack_len);
            setSn_CR(sn, Sn_CR_RECV);
            return 0;
        }

        wiz_recv_data(sn, buf, pack_len); // data copy
        setSn_CR(sn, Sn_CR_RECV);
    }

    return (int32_t)pack_len;
}

err_t W5100S_netif_output(struct netif *netif, struct pbuf *p)
{
    uint32_t send_len = 0;
    uint32_t tot_len = 0;

    memset(W5100S_state->tx_frame, 0x00, sizeof(W5100S_state->tx_frame));

    for (struct pbuf *q = p; q != NULL; q = q->next)
    {
        memcpy(W5100S_state->tx_frame + tot_len, q->payload, q->len);

        tot_len += q->len;

        if (q->len == q->tot_len)
        {
            break;
        }
    }

    if (tot_len < 60)
    {
        // pad
        tot_len = 60;
    }

    uint32_t crc = W5100S_ethernet_frame_crc(W5100S_state->tx_frame, tot_len);

    send_len = W5100S_send_lwip(0, W5100S_state->tx_frame, tot_len);

    return ERR_OK;
}

err_t W5100S_netif_initialize(struct netif *netif)
{
    netif->linkoutput = W5100S_netif_output;
    netif->output = etharp_output;
    netif->mtu = ETHERNET_MTU;
    netif->flags = NETIF_FLAG_BROADCAST | NETIF_FLAG_ETHARP | NETIF_FLAG_ETHERNET | NETIF_FLAG_IGMP | NETIF_FLAG_MLD6;
    SMEMCPY(netif->hwaddr, W5100S_state->mac, sizeof(netif->hwaddr));
    netif->hwaddr_len = sizeof(netif->hwaddr);
    return ERR_OK;
}

static uint32_t W5100S_ethernet_frame_crc(const uint8_t *data, int length)
{
    uint32_t crc = 0xffffffff; /* Initial value. */

    while (--length >= 0)
    {
        uint8_t current_octet = *data++;

        for (int bit = 8; --bit >= 0; current_octet >>= 1)
        {
            if ((crc ^ current_octet) & 1)
            {
                crc >>= 1;
                crc ^= W5100S_state->ethernet_polynomial_le;
            }
            else
                crc >>= 1;
        }
    }

    return ~crc;
}

void W5100S_gpio_interrupt_initialize(uint8_t socket, void (*callback)(void))
{
    uint16_t reg_val;
    int ret_val;

    reg_val = (SIK_CONNECTED | SIK_DISCONNECTED | SIK_RECEIVED | SIK_TIMEOUT); // Except SendOK.
    ret_val = ctlsocket(socket, CS_SET_INTMASK, (void *)&reg_val);

#if (_WIZCHIP_ == W5100S)
    reg_val = (1 << socket);
#elif (_WIZCHIP_ == W5500)
    reg_val = ((1 << socket) << 8);
#endif
    ret_val = ctlwizchip(CW_SET_INTRMASK, (void *)&reg_val);

    callback_ptr = callback;
    gpio_set_irq_enabled_with_callback(PIN_INT, GPIO_IRQ_EDGE_FALL, true, &W5100S_gpio_interrupt_callback);
}

static void W5100S_gpio_interrupt_callback(uint gpio, uint32_t events)
{
    if (callback_ptr != NULL)
    {
        callback_ptr();
    }
}

void W5100S_1ms_timer_initialize(void (*callback)(void))
{
    callback_ptr = callback;
    add_repeating_timer_us(-1000, W5100S_1ms_timer_callback, NULL, &g_timer);
}

bool W5100S_1ms_timer_callback(struct repeating_timer *t)
{
    if (callback_ptr != NULL)
    {
        callback_ptr();
    }
}

void W5100S_delay_ms(uint32_t ms)
{
    sleep_ms(ms);
}
