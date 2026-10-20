#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/*
 * WiFi + UDP for the wireless links (ESP32-C5 only; not built on host).
 *
 * One WiFi role per node: the IMU Controller is the access point of the
 * corner network, the Central the access point of the pit/dashboard network
 * (started by its pit link, lbr_link_wifi_start); corners and the dashboard
 * join as stations. The band is CONFIG_LBR_WIFI_5GHZ (5 GHz by default,
 * 2.4 GHz fallback, DAQ-WIRING.md section 8). Sends never block: a datagram
 * the stack cannot take is dropped and counted by the caller, who sees false.
 */

/* Start an access point (no-op if the pit link already started one). */
bool lbr_wifi_start_ap(const char *ssid, const char *password, int channel);

/* Join an access point; reconnects by itself when the link drops. */
bool lbr_wifi_start_sta(const char *ssid, const char *password);

/* The access point's address once joined (0 until then). */
uint32_t lbr_wifi_sta_gateway(void);

/* Apply CONFIG_LBR_WIFI_5GHZ. Only valid once esp_wifi_start has run
 * (ESP_ERR_WIFI_NOT_STARTED before); chips without 5 GHz: nothing to do. */
void lbr_wifi_apply_band(void);

typedef struct lbr_udp lbr_udp_t;

/* `self` is the socket the datagram came in on (valid even before
 * lbr_udp_open has returned to its caller). */
typedef void (*lbr_udp_rx_cb)(lbr_udp_t *self, const uint8_t *buf, size_t len, uint32_t from_ip,
                              uint16_t from_port, uint64_t rx_local_us, void *ctx);

#define LBR_UDP_BROADCAST 0xFFFFFFFFu

/* Bind `port` and call `cb` from a receive task for every datagram, with its
 * local receive time. NULL on failure. */
lbr_udp_t *lbr_udp_open(uint16_t port, lbr_udp_rx_cb cb, void *ctx);

/* `ip` in network byte order (or LBR_UDP_BROADCAST). Non-blocking. */
bool lbr_udp_send(lbr_udp_t *u, uint32_t ip, uint16_t port, const uint8_t *buf, size_t len);
