/*
 * Copyright (c) 2022 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef _PICO_W5100S_DRIVER_H
#define _PICO_W5100S_DRIVER_H

#include <string.h>

#include "pico.h"
#include "pico/async_context.h"
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "pico/critical_section.h"
#include "pico/unique_id.h"
#include "hardware/spi.h"
#include "hardware/dma.h"
#include "hardware/clocks.h"
#include "hardware/gpio.h"

#include "picolog.h"

#include "wizchip_conf.h"
#include "socket.h"

#include "pico/lwip_nosys.h"

#include "lwip/init.h"
#include "lwip/netif.h"
#include "lwip/timeouts.h"
#include "lwip/apps/lwiperf.h"
#include "lwip/etharp.h"
#include "lwip/dhcp.h"
#include "lwip/dns.h"
#include "lwip/tcpip.h"

#include "pico/async_context_poll.h"
#include "pico/lwip_nosys.h"

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

/* GPIO IRQ handler priority. */
#ifndef W5100S_GPIO_IRQ_HANDLER_PRIORITY
#define W5100S_GPIO_IRQ_HANDLER_PRIORITY 0x40
#endif

/* Poll timeout. */
#ifndef W5100S_SLEEP_CHECK_MS
#define W5100S_SLEEP_CHECK_MS 50
#endif

#define W5100S_POST_POLL_HOOK W5100S_post_poll_hook();

#ifdef __cplusplus
extern "C" {
#endif

typedef struct _W5100S_t  {
    uint8_t mac[6];
    struct netif *netif;
    uint32_t ethernet_polynomial_le;
    int spi_frequency;
    uint8_t socket;
    uint8_t *pack;
    struct pbuf *p;
    uint16_t pack_len;

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

static void W5100S_set_irq_enabled(bool enabled);

static void W5100S_gpio_irq_handler(void);

uint32_t W5100S_irq_init(void *param);

uint32_t W5100S_irq_deinit(void *param);

void W5100S_post_poll_hook(void);

void W5100S_schedule_internal_poll_dispatch(__unused void (*func)(void));

static void W5100S_do_poll(async_context_t *context, async_when_pending_worker_t *worker);

static void W5100S_sleep_timeout_reached(async_context_t *context, async_at_time_worker_t *worker);

bool W5100S_init(W5100S_t *self);

void W5100S_deinit(W5100S_t *self);

uint32_t storage_read_blocks(__unused uint8_t *dest, __unused uint32_t block_num, __unused uint32_t num_blocks);

void W5100S_thread_enter(void);

void W5100S_thread_exit(void);

void cyw43_thread_lock_check(void);

bool W5100S_driver_init(async_context_t *context);

void W5100S_driver_deinit(async_context_t *context);

void W5100S_await_background_or_timeout_us(uint32_t timeout_us);

void W5100S_delay_ms(uint32_t ms);

void W5100S_delay_us(uint32_t us);



void W5100S_check_state(void);

bool W5100S_cable_connected(W5100S_t *self);

static inline uint64_t W5100S_mix(uint64_t h);

uint64_t W5100S_fast_hash_64(const void * buf, size_t len, uint64_t seed);

void W5100S_mac_init(W5100S_t *self);

/** @brief Bring the link up.*/
bool W5100S_bring_link_up(W5100S_t *self);

/** @brief Bring the link down.*/
void W5100S_bring_link_down(W5100S_t *self);

void W5100S_spi_init(W5100S_t *self);

void W5100S_dma_init(W5100S_t *self);

void W5100S_critical_section_init(W5100S_t *self);

void W5100S_chip_init(W5100S_t *self);

void W5100S_version_check(W5100S_t *self);

/** @brief W5100S reset function. */
void W5100S_reset(W5100S_t *self);

int32_t W5100S_send_lwip(uint8_t sn, uint8_t *buf, uint16_t len);

int32_t W5100S_recv_lwip(uint8_t sn, uint8_t *buf, uint16_t len);

err_t W5100S_netif_output(struct netif *netif, struct pbuf *p);

err_t W5100S_netif_init(struct netif *netif);

static uint32_t W5100S_ethernet_frame_crc(const uint8_t *data, int length);

/** @brief Netif status callback. */
void W5100S_status_callback(struct netif *netif);

/** @brief Netif link callback. */
void W5100S_link_callback(struct netif *netif);

static void W5100S_critical_section_lock();

static void W5100S_critical_section_unlock();

static inline void W5100S_select();

static inline void W5100S_deselect();

static uint8_t W5100S_read(void);

static void W5100S_write(uint8_t tx_data);

static void W5100S_read_burst(uint8_t *pBuf, uint16_t len);

static void W5100S_write_burst(uint8_t *pBuf, uint16_t len);

/** @brief Process any data. */
void W5100S_process(W5100S_t *self);

#ifdef __cplusplus
}
#endif

#endif