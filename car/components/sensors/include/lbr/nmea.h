#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "lbr/sensor.h"

/*
 * NMEA 0183 receiver for any GPS module with a serial NMEA output (u-blox,
 * MediaTek, Quectel, ...): part-independent, so the imu_gps node can carry
 * GPS before the module is chosen. Uses GGA (position, altitude, satellites,
 * HDOP, quality), RMC (position, speed, course, UTC date/time, status) and
 * GSA (2D/3D fix) from any talker (GP, GN, GL, GA, BD). Sentences with a bad
 * or missing checksum are counted and ignored.
 *
 * GPS UTC is a wall-clock anchor only; samples are timestamped by the node
 * runtime on the shared CAN time base like every other channel.
 *
 * Pure (host-tested). sensor_gps_nmea_esp.c owns the UART.
 */

#define LBR_NMEA_LINE_MAX 96u
#define LBR_NMEA_STALE_US 2000000u /* no good sentence for this long = NO_DATA */

typedef struct {
    /* Latest decoded state. *_us are local times of the sentence that set the
     * field (0 = never). */
    uint64_t pos_us, alt_us, vel_us, gga_us, gsa_us, rmc_us, utc_us;
    uint64_t last_good_us;
    int32_t  lat_e7, lon_e7;
    float    alt_m;
    float    speed_mps, course_deg;
    float    hdop;
    uint32_t utc_s;

    /* Counters (wrap). */
    uint32_t sentences;
    uint32_t checksum_errors;
    uint32_t overlong;

    uint8_t gga_quality; /* 0 none, 1 GPS, 2 DGPS, 6 dead reckoning, ... */
    uint8_t sats;
    uint8_t gsa_mode; /* 1 none, 2 2D, 3 3D */
    bool    hdop_valid;
    bool    course_valid;
    bool    rmc_active; /* RMC status A */

    /* Line assembly. */
    bool   in_sentence;
    size_t len;
    char   line[LBR_NMEA_LINE_MAX];
} lbr_nmea_t;

void lbr_nmea_init(lbr_nmea_t *p);

/* Feed received bytes; `now_us` is the local time they arrived. Returns the
 * number of sentences accepted. */
uint32_t lbr_nmea_feed(lbr_nmea_t *p, const uint8_t *data, size_t n, uint64_t now_us);

/* Fix type as published on GPS_FIX: 0 none, 1 dead reckoning, 2 2D, 3 3D. */
uint8_t lbr_nmea_fix_type(const lbr_nmea_t *p, uint64_t now_us);

/*
 * Fill one reading per signal. Understands the GPS_* catalog signals (any
 * order, any subset); anything else reads NOT_FITTED. Everything is NO_DATA
 * while the stream is stale; position, speed, and course need a 2D fix,
 * altitude a 3D fix, UTC an active RMC.
 */
void lbr_nmea_fill(const lbr_nmea_t *p, uint64_t now_us, const lbr_signal_id_t *signals, size_t n,
                   lbr_reading_t *out);

/* Days-from-civil based UTC to Unix seconds; 0 on a bad date. */
uint32_t lbr_nmea_unix_time(unsigned year, unsigned month, unsigned day, unsigned hour,
                            unsigned min, unsigned sec);

/* Hardware driver (sensor_gps_nmea_esp.c). uart_num, rx_gpio from Kconfig;
 * tx_gpio may be -1 (receive only). */
typedef struct {
    int      uart_num;
    int      rx_gpio;
    int      tx_gpio;
    uint32_t baud;
} lbr_gps_uart_cfg_t;

void lbr_sensor_gps_nmea(lbr_sensor_t *out, const char *name, const lbr_signal_id_t *signals,
                         size_t n_signals, const lbr_gps_uart_cfg_t *cfg);
