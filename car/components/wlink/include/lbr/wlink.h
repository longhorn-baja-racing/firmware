#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "lbr/can.h"
#include "lbr/frames.h"

/*
 * Wireless links (DAQ-WIRING.md section 8), pure codecs (host-tested).
 *
 *   corner node -> IMU Controller   IMU samples, clock requests   (LBR_LINK_CORNER)
 *   IMU Controller -> corner node   clock replies
 *   Central -> dashboard            display values, node status   (LBR_LINK_DASH)
 *   dashboard -> Central            dashboard battery
 *
 * Every datagram is one transport packet (lbr/packet.h: header, records,
 * CRC-32) over UDP, no TCP, so a lost datagram never stalls the next. The
 * packet seq is per sender, so the receiver counts lost datagrams.
 *
 * IMU_BATCH record (corner -> IMU), contiguous samples k0 .. k0 + n - 1:
 *
 *   0  4  k0, low 32 bits of the grid index of the first sample
 *   4  1  n
 *   5  1  battery: raw CORNER_xx_BATTERY field (lbr/frames.h encoding)
 *   6  1  flags: bit 0 clock synced, bit 1 IMU healthy and calibrated,
 *               bit 2 simulated data
 *   7  1  bytes per sample (s)
 *   8  .. n * s bytes: for each frame whose origin is this corner, in id
 *         order, its payload without the counter bytes (already in the
 *         frame-table encoding, so the IMU Controller forwards them as is)
 *
 * Each packet carries the previous batch again ahead of the new one, so one
 * lost datagram loses no samples; the IMU Controller forwards each sample
 * once and counts the ones that never arrived.
 *
 * Clock: the corner sends TIME_REQ (t1 u64, its local clock); the IMU
 * Controller answers TIME_RESP (t1 u64, t2 u64 shared time at reception,
 * t3 u64 shared time at reply, epoch u16, flags u8: bit 0 server synced).
 * The corner estimates the shared time at reception as t3 + rtt / 2 with
 * rtt = (t4 - t1) - (t3 - t2), and disciplines its servo with it.
 */

#define LBR_WLINK_CORNER_PORT 47801u /* IMU Controller listens; corners reply-to */
#define LBR_WLINK_DASH_PORT   47802u /* dashboard listens */
#define LBR_WLINK_STATUS_PORT 47803u /* Central listens for the dashboard */

#define LBR_WLINK_MAX_BATCH       10u      /* samples in one IMU_BATCH record */
#define LBR_WLINK_MAX_SAMPLE      16u      /* bytes per sample */
#define LBR_WLINK_LINK_TIMEOUT_US 500000u  /* corners send 20 datagrams/s, the Central 10/s */
#define LBR_WLINK_DASH_TIMEOUT_US 3000000u /* the dashboard reports once a second */
#define LBR_WLINK_MAX_RTT_US      20000u   /* clock replies slower than this are ignored */

#define LBR_WLINK_F_SYNCED    0x01u
#define LBR_WLINK_F_SENSOR_OK 0x02u
#define LBR_WLINK_F_SIM       0x04u

#define LBR_WLINK_BATCH_HDR 8u
#define LBR_WLINK_TIME_REQ  8u
#define LBR_WLINK_TIME_RESP 27u

typedef struct {
    uint32_t       k0;
    uint8_t        n;
    uint8_t        battery_raw;
    uint8_t        flags;
    uint8_t        sample_size;
    const uint8_t *samples; /* n * sample_size bytes */
} lbr_imu_batch_t;

/* Returns the record length, or 0 if it does not fit `cap`. */
size_t lbr_imu_batch_encode(const lbr_imu_batch_t *b, uint8_t *buf, size_t cap);
/* False if the record is malformed. b->samples points into `buf`. */
bool lbr_imu_batch_decode(const uint8_t *buf, size_t len, lbr_imu_batch_t *b);

/* The frames carrying samples taken by `origin` (a corner), in id order, and
 * the bytes one sample takes in an IMU_BATCH. Returns the frame count. */
size_t lbr_wlink_origin_frames(uint8_t origin, const lbr_frame_def_t **frames, size_t cap,
                               size_t *sample_size);

/* Copy frame `f`'s payload minus its counter bytes into `out` (returns the
 * byte count), and the reverse: rebuild the frame for grid index k. */
size_t lbr_wlink_strip_counter(const lbr_frame_def_t *d, const lbr_can_frame_t *f, uint8_t *out);
void   lbr_wlink_build_frame(const lbr_frame_def_t *d, const uint8_t *bytes, uint64_t k,
                             lbr_can_frame_t *out);

/* ------------------------------------------------------------------------ */
/* Clock over WiFi                                                           */

typedef struct {
    bool     pending;
    uint64_t t1;
    uint32_t accepted;
    uint32_t rejected; /* stale, unmatched, or too slow */
    uint32_t last_rtt_us;
    bool     server_synced;
} lbr_wclock_t;

/* Client: start a request at local time `local_now`; fills the TIME_REQ
 * payload. */
void lbr_wclock_request(lbr_wclock_t *c, uint64_t local_now, uint8_t out[LBR_WLINK_TIME_REQ]);

/* Client: a TIME_RESP arrived at local time `local_rx`. True if it is a
 * usable measurement: *shared_at_rx is the shared time at local_rx. */
bool lbr_wclock_on_response(lbr_wclock_t *c, const uint8_t *payload, size_t len, uint64_t local_rx,
                            uint64_t *shared_at_rx, uint16_t *epoch);

/* Server: answer a TIME_REQ. Returns the TIME_RESP length, or 0 if the
 * request is malformed. */
size_t lbr_wclock_reply(const uint8_t *req, size_t len, uint64_t t2_shared, uint64_t t3_shared,
                        uint16_t epoch, bool server_synced, uint8_t out[LBR_WLINK_TIME_RESP]);
