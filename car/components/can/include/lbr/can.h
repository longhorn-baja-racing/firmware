#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "lbr/frames.h"
#include "lbr/health.h"
#include "lbr/nodes.h"
#include "lbr/protocol.h"

/*
 * CAN id map and frame codec (pure; host-tested). See docs/can-ids.md and
 * DAQ-WIRING.md section 7.
 *
 * Every LBR frame is a classic CAN data frame with an 11-bit standard id;
 * lower ids win arbitration. Data frames (sensor values) are described by
 * the frame table (lbr/frames.h). The system frames are:
 *
 *   0x001         TIME_SYNC       Central, 10 Hz
 *   0x002         TIME_FOLLOW_UP  Central, after each TIME_SYNC
 *   0x010         COMMAND         Central, on event
 *   0x020 + n     EVENT / FAULT   node n (1..6), on event
 *   0x700 + n     HEARTBEAT       node n, 1 Hz: state, error flags
 *   0x780 + 16n + p  HEALTH page p (0..11) of node n, 1 Hz
 *
 * Each id has exactly one transmitter, as classic CAN requires, so the
 * sender of any frame follows from its id (lbr_can_sender). Extended-id and
 * remote frames are not LBR traffic; receivers count and ignore them.
 */

#define LBR_CAN_ID_TIME_SYNC      0x001u
#define LBR_CAN_ID_TIME_FOLLOW_UP 0x002u
#define LBR_CAN_ID_COMMAND        0x010u
#define LBR_CAN_ID_EVENT_BASE     0x020u /* + node id */
#define LBR_CAN_ID_HEARTBEAT_BASE 0x700u /* + node id */
#define LBR_CAN_ID_HEALTH_BASE    0x780u /* + 16 * node id + page */
#define LBR_CAN_ID_MAX            0x7FFu

/* Central TIME_SYNC + FOLLOW_UP pairs per second (fixed by the protocol). */
#define LBR_TIME_SYNC_HZ 10u

/* System-wide ceiling for the planned worst-case bus load, permille of the
 * bit rate (DAQ-WIRING.md: under ~50 %). Leaves headroom for
 * retransmissions after errors and for growth; enforced over the frame table
 * and every node plan by the host tests. */
#define LBR_CAN_SYSTEM_BUDGET_PERMILLE 500u

typedef struct {
    uint32_t id; /* 11-bit standard id */
    uint8_t  dlc;
    uint8_t  data[8];
} lbr_can_frame_t;

/* The node that transmits `id` (lbr_node_id_t), or 0 if `id` is not LBR. */
uint8_t lbr_can_sender(uint32_t id);

/* ------------------------------------------------------------------------ */
/* Data frames                                                               */

/*
 * Encode one value into its field (lbr/frames.h). `s` supplies valid,
 * reason, and value; an unavailable sample, or a value that does not fit the
 * encoding (or NaN), is written as the reserved raw value for its reason
 * (OUT_OF_RANGE for a misfit). Returns false if the value did not fit.
 */
bool lbr_can_field_encode(const lbr_field_def_t *fd, const lbr_sample_t *s, uint8_t *payload);

/* Decode one field into s->type, s->valid, and s->v (or the reason). */
void lbr_can_field_decode(const lbr_field_def_t *fd, const uint8_t *payload, lbr_sample_t *s);

/*
 * Build data frame `d` for grid sample `k`: values[i] is field i. Unused
 * payload bytes are zero. Returns the number of fields whose value did not
 * fit and went out as OUT_OF_RANGE.
 */
unsigned lbr_can_encode_data(const lbr_frame_def_t *d, const lbr_sample_t *values, uint64_t k,
                             lbr_can_frame_t *out);

/* Grid index of a received sample: the k nearest the receive time whose low
 * 16 bits equal `counter` (exact while delivery takes under 32768 periods). */
uint64_t lbr_can_grid_index(uint16_t counter, uint64_t rx_shared_us, uint32_t period_us);

/*
 * Decode a data frame into one sample per field (at most `cap`). Samples get
 * source = the frame's origin, seq = grid index k, t_mono_us = k * period,
 * and `synced` as given (see lbr_bus_decoder_t). Returns the number of
 * samples, 0 if `f` is not a data frame of the table or its dlc is wrong.
 */
size_t lbr_can_decode_data(const lbr_can_frame_t *f, uint64_t rx_shared_us, bool synced,
                           lbr_sample_t *out, size_t cap);

/* ------------------------------------------------------------------------ */
/* Events and faults                                                         */

/*
 * EVENT / FAULT frame of node n (id 0x020 + n), dlc 8:
 *
 *   0..1  event code (lbr_event_code_t)
 *   2..3  argument
 *   4     bit 0 fault, bit 1 synced, bits 2..7 seq (low 6 bits, per node)
 *   5..7  low 24 bits of the shared time (us) the event happened
 *
 * The receiver rebuilds the full time from its own clock
 * (lbr_can_unwrap_time), exact within +/- 8.38 s of delivery.
 */
bool lbr_can_encode_event(const lbr_sample_t *s, lbr_can_frame_t *out);
bool lbr_can_decode_event(const lbr_can_frame_t *f, uint64_t rx_shared_us, lbr_sample_t *out);

/* Rebuild a 64-bit time from its low 24 bits, nearest to `ref_us`. */
uint64_t lbr_can_unwrap_time(uint32_t t_lo24, uint64_t ref_us);

/* ------------------------------------------------------------------------ */
/* Time sync (Central -> all)                                                */

/*
 * TIME_SYNC (0x001, dlc 8): bytes 0..3 uint32 milliseconds since the Central
 * booted (coarse, for simple listeners), 4 seq, 5..6 epoch, 7 reserved (0).
 * Every receiver timestamps it on reception; TIME_FOLLOW_UP (0x002, dlc 8:
 * 0 seq, 1..7 uint56 us) then carries the exact shared time at which that
 * TIME_SYNC finished transmitting. `epoch` changes on every Central boot.
 */
void lbr_can_encode_time_sync(uint8_t sync_seq, uint16_t epoch, uint64_t now_us,
                              lbr_can_frame_t *out);
void lbr_can_encode_time_follow_up(uint8_t sync_seq, uint64_t tx_us, lbr_can_frame_t *out);
bool lbr_can_decode_time_sync(const lbr_can_frame_t *f, uint8_t *sync_seq, uint16_t *epoch);
bool lbr_can_decode_time_follow_up(const lbr_can_frame_t *f, uint8_t *sync_seq, uint64_t *tx_us);

/* ------------------------------------------------------------------------ */
/* Command (Central -> all)                                                  */

/* Command codes (byte 0 of 0x010). Append only. */
typedef enum {
    LBR_CMD_SESSION_START = 0x01, /* Central opened a log session; args 1..4: session number */
    LBR_CMD_SESSION_STOP  = 0x02, /* Central closed its log session */
    LBR_CMD_MARK          = 0x03, /* operator marker; args 1..2: marker number */
} lbr_command_t;

/* `args` is 7 bytes (zero padded). */
void lbr_can_encode_command(uint8_t code, const uint8_t args[7], lbr_can_frame_t *out);
bool lbr_can_decode_command(const lbr_can_frame_t *f, uint8_t *code, uint8_t args[7]);

/* ------------------------------------------------------------------------ */
/* Heartbeat and health                                                      */

/* Heartbeat error flags (byte 1 of 0x700 + n). */
#define LBR_HB_ERR_NOT_SYNCED 0x01u /* clock not disciplined to the Central: times are local */
#define LBR_HB_ERR_SENSOR     0x02u /* a sensor is faulted or uncalibrated */
#define LBR_HB_ERR_CAN        0x04u /* CAN errors: error passive, or a bus-off since last */
#define LBR_HB_ERR_CONFIG     0x08u /* configuration rejected */
#define LBR_HB_ERR_STORAGE    0x10u /* an expected log is not being written */
#define LBR_HB_ERR_SIM        0x20u /* simulated sensor data: not measurements */
#define LBR_HB_ERR_LOSS       0x40u /* samples, frames, or records were lost this period */
#define LBR_HB_ERR_LINK       0x80u /* a wireless link (corner, dashboard) is down */

/* HEARTBEAT (0x700 + n, dlc 2): 0 state (lbr_node_state_t), 1 error flags. */
bool lbr_can_encode_heartbeat(uint8_t node, uint8_t state, uint8_t err_flags, lbr_can_frame_t *out);
/* Returns the sending node, or 0 if `f` is not a heartbeat. */
uint8_t lbr_can_decode_heartbeat(const lbr_can_frame_t *f, uint8_t *state, uint8_t *err_flags);

/* Health page frames: one frame per page (lbr/health.h). */
bool lbr_can_encode_health_page(uint8_t node, const lbr_health_record_t *h, unsigned page,
                                lbr_can_frame_t *out);
/* Returns the page index decoded into `h` (and sets h->node_id), or -1. */
int lbr_can_decode_health_page(const lbr_can_frame_t *f, lbr_health_record_t *h);

/* ------------------------------------------------------------------------ */
/* Bus decoder                                                               */

/*
 * Turns any LBR frame into samples, tracking what a frame alone does not
 * say: whether its origin node's clock was synced. A CAN node is synced
 * while its latest heartbeat lacks LBR_HB_ERR_NOT_SYNCED; a corner node
 * while the IMU Controller is synced and the latest CORNER_STATUS sets the
 * corner's CORNER_SYNCED bit; the Central always (it is the time master).
 * Nothing heard yet means not synced, and a node's BOOT event, or a
 * NODE_LOST / NODE_FOUND event about it, means not synced until its next
 * heartbeat or CORNER_STATUS says otherwise.
 */
typedef struct {
    bool    hb_seen[LBR_NODE_CAN_MAX + 1];
    uint8_t hb_flags[LBR_NODE_CAN_MAX + 1];
    bool    corner_seen;
    uint8_t corner_synced; /* CORNER_SYNCED bits */
} lbr_bus_decoder_t;

void lbr_bus_decoder_init(lbr_bus_decoder_t *d);

/* True if samples taken by `origin` are on the shared time base. */
bool lbr_bus_decoder_synced(const lbr_bus_decoder_t *d, uint8_t origin);

/* Decode a frame: data frames give one sample per field, EVENT/FAULT one
 * sample; heartbeats and CORNER_STATUS also update the sync state. Returns
 * the number of samples written (at most `cap`). */
size_t lbr_bus_decode(lbr_bus_decoder_t *d, const lbr_can_frame_t *f, uint64_t rx_shared_us,
                      lbr_sample_t *out, size_t cap);

/* ------------------------------------------------------------------------ */
/* Raw frame log record                                                      */

/*
 * Raw frame log record (LBR_REC_CAN_FRAME), 22 bytes. The Central logs every
 * bus frame this way: the bytes are exactly what was on the bus, so the
 * authoritative log never depends on this firmware's decoder.
 *
 *   0  8  rx time, shared time base (us)
 *   8  4  CAN id
 *   12 1  flags: bit 0 extended id, bit 1 transmitted by the logging node
 *   13 1  dlc (0..8)
 *   14 8  data, zero padded past dlc
 */
#define LBR_CAN_FRAME_REC_SIZE  22u
#define LBR_CAN_REC_FLAG_EXT    0x01u
#define LBR_CAN_REC_FLAG_OWN_TX 0x02u

size_t lbr_can_frame_rec_encode(uint64_t t_us, uint8_t flags, const lbr_can_frame_t *f,
                                uint8_t *buf, size_t cap);
size_t lbr_can_frame_rec_decode(const uint8_t *buf, size_t len, uint64_t *t_us, uint8_t *flags,
                                lbr_can_frame_t *f);

/* Worst-case on-wire bits of a standard-id data frame (incl. bit stuffing
 * and interframe space): the unit of the CAN bandwidth budget. */
static inline uint32_t lbr_can_frame_bits_worst(uint8_t dlc)
{
    uint32_t stuffable = 34u + 8u * dlc; /* SOF .. CRC */
    return stuffable + (stuffable - 1u) / 4u + 13u;
}

/* Controller statistics (TWAI adapter on target, the virtual bus in the
 * simulator). Counters wrap. */
typedef struct {
    uint32_t tx_frames;
    uint32_t rx_frames;
    uint32_t tx_dropped; /* not queued (queue full, bus-off, not started) */
    uint32_t tx_failed;  /* queued but reported failed by the controller */
    uint32_t rx_dropped; /* RX queue full */
    uint32_t rx_foreign; /* extended-id or remote frames: not LBR, not queued */
    uint32_t bus_off_count;
    uint32_t bus_errors;
    uint8_t  tec;
    uint8_t  rec;
    bool     running; /* started and not bus-off */
} lbr_can_stats_t;
