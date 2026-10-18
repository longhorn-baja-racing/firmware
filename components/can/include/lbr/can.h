#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "lbr/health.h"
#include "lbr/protocol.h"

/*
 * CAN ID map and frame codec (pure; host-tested). See docs/can-ids.md.
 *
 * Every LBR frame is a classic CAN frame with a 29-bit extended ID:
 *
 *   bits 28..26  priority  0 (wins arbitration) .. 7
 *   bits 25..24  kind      0 = sample, 1 = system, 2..3 reserved
 *   bits 23..16  source    node id (lbr_node_id_t), unique per node
 *   bits 15..0   sub-id    signal id (kind 0) or system message id (kind 1)
 *
 * The source field makes every ID unique to one transmitter, which classic
 * CAN requires, and lets any listener attribute a frame without a payload
 * byte.
 */

#define LBR_CAN_KIND_SAMPLE 0u
#define LBR_CAN_KIND_SYSTEM 1u

/* System message ids (kind 1). Append only. */
#define LBR_SYS_TIME_SYNC      0x0001u /* gateway: sync_seq u8, epoch u16 */
#define LBR_SYS_TIME_FOLLOW_UP 0x0002u /* gateway: sync_seq u8, tx time u56 (us) */
#define LBR_SYS_HEALTH_BASE    0x0100u /* + page index 0..LBR_HEALTH_PAGE_COUNT-1 */

/* Gateway TIME_SYNC + FOLLOW_UP pairs per second (fixed by the protocol). */
#define LBR_TIME_SYNC_HZ 4u

/* System-wide ceiling for the planned worst-case bus load, permille of the
 * bit rate. Leaves headroom for retransmissions after errors and for growth;
 * enforced over all node plans by the host tests. */
#define LBR_CAN_SYSTEM_BUDGET_PERMILLE 600u

/* Priorities for system messages. Sample priorities come from the catalog. */
#define LBR_CAN_PRIO_TIME   1u
#define LBR_CAN_PRIO_HEALTH 6u

typedef struct {
    uint32_t id; /* 29-bit extended id */
    uint8_t  dlc;
    uint8_t  data[8];
} lbr_can_frame_t;

typedef struct {
    uint8_t  prio;
    uint8_t  kind;
    uint8_t  source;
    uint16_t sub;
} lbr_can_id_t;

static inline uint32_t lbr_can_id_make(uint8_t prio, uint8_t kind, uint8_t source, uint16_t sub)
{
    return ((uint32_t)(prio & 0x7u) << 26) | ((uint32_t)(kind & 0x3u) << 24) |
           ((uint32_t)source << 16) | sub;
}

static inline lbr_can_id_t lbr_can_id_parse(uint32_t id)
{
    lbr_can_id_t f = {
        .prio   = (uint8_t)((id >> 26) & 0x7u),
        .kind   = (uint8_t)((id >> 24) & 0x3u),
        .source = (uint8_t)(id >> 16),
        .sub    = (uint16_t)id,
    };
    return f;
}

/*
 * Sample frame payload (dlc 8):
 *
 *   bytes 0..3  value bits (lbr_sample_value_bits): the reading, or the
 *               lbr_unavail_reason_t when not valid
 *   bytes 4..6  low 24 bits of t_mono_us (shared time base)
 *   byte  7     bits 0..5 seq (low 6 bits), bit 6 synced, bit 7 valid
 *
 * The receiver rebuilds the full timestamp from its own shared-time clock at
 * reception (lbr_can_unwrap_time), valid while a frame is delivered within
 * +/- 8.38 s of being stamped. The value type is not carried: it is fixed by
 * the catalog for each signal id.
 */
#define LBR_CAN_SEQ_BITS 6u

/* False if the signal is not in the catalog or the sample is malformed. */
bool lbr_can_encode_sample(const lbr_sample_t *s, lbr_can_frame_t *out);

/* `rx_shared_us` is the receiver's shared-time clock when the frame arrived.
 * Returns false if the frame is not a well-formed sample frame. Unknown
 * signals decode with type LBR_VAL_RAW. */
bool lbr_can_decode_sample(const lbr_can_frame_t *f, uint64_t rx_shared_us, lbr_sample_t *out);

/* Rebuild a 64-bit time from its low 24 bits, nearest to `ref_us`. */
uint64_t lbr_can_unwrap_time(uint32_t t_lo24, uint64_t ref_us);

/* Time sync (gateway -> all). Two-step: SYNC is timestamped by every receiver
 * on reception; FOLLOW_UP then carries the gateway time at which SYNC finished
 * transmitting. `epoch` changes on every gateway boot. */
void lbr_can_encode_time_sync(uint8_t source, uint8_t sync_seq, uint16_t epoch,
                              lbr_can_frame_t *out);
void lbr_can_encode_time_follow_up(uint8_t source, uint8_t sync_seq, uint64_t tx_us,
                                   lbr_can_frame_t *out);
bool lbr_can_decode_time_sync(const lbr_can_frame_t *f, uint8_t *sync_seq, uint16_t *epoch);
bool lbr_can_decode_time_follow_up(const lbr_can_frame_t *f, uint8_t *sync_seq, uint64_t *tx_us);

/* Health page frames: one frame per page. */
bool lbr_can_encode_health_page(uint8_t source, const lbr_health_record_t *h, unsigned page,
                                lbr_can_frame_t *out);
/* Returns the page index decoded into `h`, or -1 if `f` is not a valid page. */
int lbr_can_decode_health_page(const lbr_can_frame_t *f, lbr_health_record_t *h);

/*
 * Raw frame log record (LBR_REC_CAN_FRAME), 22 bytes. The gateway logs every
 * bus frame this way: the bytes are exactly what was on the bus, so the
 * authoritative log never depends on this firmware's decoder.
 *
 *   0  8  rx time, shared time base (us)
 *   8  4  CAN id (29-bit)
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

/* Worst-case on-wire bits of an extended data frame (incl. bit stuffing and
 * interframe space): the unit of the CAN bandwidth budget. */
static inline uint32_t lbr_can_frame_bits_worst(uint8_t dlc)
{
    uint32_t stuffable = 54u + 8u * dlc; /* SOF .. CRC */
    return stuffable + (stuffable - 1u) / 4u + 13u;
}
