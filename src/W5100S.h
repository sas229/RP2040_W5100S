#ifdef __cplusplus
extern "C" {
#endif

#ifndef W5100S_H
#define W5100S_H

#include <string.h>

#include "picolog.h"

#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "pico/critical_section.h"
#include "hardware/spi.h"
#include "hardware/dma.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"

#include "wizchip_conf.h"
#include "socket.h"

#include "lwip/init.h"
#include "lwip/netif.h"
#include "lwip/timeouts.h"
#include "lwip/apps/lwiperf.h"
#include "lwip/etharp.h"
#include "lwip/dhcp.h"
#include "lwip/dns.h"
#include "lwip/tcpip.h"

// SPI setup.
#define SPI_PORT spi0
#define PIN_SCK 18
#define PIN_MOSI 19
#define PIN_MISO 16
#define PIN_CS 17
#define PIN_RST 20

// GPIO setup.
#define PIN_INT 21

// lwip setup.
#define ETHERNET_MTU 1500

/* Timeout */
#define RECV_TIMEOUT (1000 * 10) // 10 seconds

// static uint8_t tx_frame[1542];
// static const uint32_t ethernet_polynomial_le = 0xedb88320U;

typedef struct _W5100S_t  {
    uint8_t mac[6];
    struct netif *netif;
    uint8_t *pack;
    uint16_t pack_len;
    struct pbuf *p;
    uint8_t tx_frame;
    uint32_t ethernet_polynomial_le;
    
    int connect_timeout;
    bool cable_connected;
    bool link_up;
    bool connected;
    bool dhcp_ip_allocated;
    
    critical_section_t critical_section;
    
    uint dma_tx;
    uint dma_rx;
    dma_channel_config dma_channel_config_tx;
    dma_channel_config dma_channel_config_rx;
} W5100S_t;

extern W5100S_t *W5100S_state;

/** @brief Initialisation function. */
int W5100S_init();

/** @brief Connect function. */
bool W5100S_connect();

/** @brief Check if cable is connected. */
bool W5100S_cable_connected();

/** @brief Process any data. */
void W5100S_process();

/** @brief Poll for work. */
void W5100S_poll();

/** @brief Bring the link up.*/
bool W5100S_bring_link_up();

/** @brief Bring the link down.*/
void W5100S_bring_link_down();

/** @brief Netif status callback. */
void W5100S_status_callback(struct netif *netif);

/** @brief Netif link callback. */
void W5100S_link_callback(struct netif *netif);

/** @brief W5100S reset function. */
void W5100S_reset();

static inline void W5100S_select(void);

static inline void W5100S_deselect(void);

static uint8_t W5100S_read(void);

static void W5100S_write(uint8_t tx_data);

static void W5100S_read_burst(uint8_t *pBuf, uint16_t len);

static void W5100S_write_burst(uint8_t *pBuf, uint16_t len);

static void W5100S_critical_section_lock();

static void W5100S_critical_section_unlock();

void W5100S_spi_initialize(void);

void W5100S_cris_initialize();

void W5100S_initialize(void);

void W5100S_check(void);

int32_t W5100S_send_lwip(uint8_t sn, uint8_t *buf, uint16_t len);

int32_t W5100S_recv_lwip(uint8_t sn, uint8_t *buf, uint16_t len);

err_t W5100S_netif_output(struct netif *netif, struct pbuf *p);

err_t W5100S_netif_initialize(struct netif *netif);

static uint32_t W5100S_ethernet_frame_crc(const uint8_t *data, int length);

void W5100S_gpio_interrupt_initialize(uint8_t socket, void (*callback)(void));

static void W5100S_gpio_interrupt_callback(uint gpio, uint32_t events);

void W5100S_1ms_timer_initialize(void (*callback)(void));

bool W5100S_1ms_timer_callback(struct repeating_timer *t);

void W5100S_delay_ms(uint32_t ms);

#endif

#ifdef __cplusplus
}
#endif