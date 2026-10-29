#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/*
 * Heartbeat / status record. Emitted on a fixed cadence even when measurements
 * are unchanged. All counters are uint32 that wrap modulo 2^32 (never reset or
 * saturate mid-session); consumers diff them with modular arithmetic.
 */

typedef enum {
    LBR_RESET_UNKNOWN = 0,
    LBR_RESET_POWER_ON,
    LBR_RESET_SOFTWARE,
    LBR_RESET_PANIC,
    LBR_RESET_WATCHDOG, /* interrupt, task, or other watchdog */
    LBR_RESET_BROWNOUT,
    LBR_RESET_DEEP_SLEEP,
    LBR_RESET_EXTERNAL, /* reset pin */
} lbr_reset_reason_t;

typedef enum {
    LBR_NODE_BOOT = 0,       /* initializing */
    LBR_NODE_RUN,            /* acquiring/publishing normally */
    LBR_NODE_DEGRADED,       /* running with a recoverable problem (sensor, SD, sync) */
    LBR_NODE_CONFIG_INVALID, /* configuration rejected; acquisition not started */
    LBR_NODE_FAULT,          /* cannot perform its function */
} lbr_node_state_t;

/* Health flags (lbr_health_record_t.flags). */
#define LBR_HF_TIME_SYNCED  0x0001u /* disciplined to the gateway time base */
#define LBR_HF_SD_PRESENT   0x0002u /* SD card mounted (gateway / local-log nodes) */
#define LBR_HF_SD_LOGGING   0x0004u /* a log session is open and being written */
#define LBR_HF_SIM_SENSORS  0x0008u /* simulated sensor data: NOT real measurements */
#define LBR_HF_CONFIG_VALID 0x0010u
#define LBR_HF_CAN_UP       0x0020u /* TWAI running and not bus-off */
#define LBR_HF_TIME_MASTER  0x0040u /* this node is the time-sync source (gateway) */

/* "No such age" sentinel for the *_age_ms fields (e.g. no SD on this node). */
#define LBR_AGE_NONE 0xFFFFFFFFu

typedef struct {
    uint8_t  node_id;
    uint8_t  state; /* lbr_node_state_t */
    uint16_t flags; /* LBR_HF_* */

    /* identity / versions */
    uint32_t fw_version;
    uint16_t protocol_version;
    uint16_t boot_count; /* low 16 bits of the persistent boot counter */
    uint32_t catalog_hash;
    uint32_t config_crc; /* CRC of the validated node configuration */

    uint32_t uptime_ms;
    uint8_t  reset_reason; /* lbr_reset_reason_t of THIS boot (cause of the previous exit) */

    /* rates, Hz (measured over the last heartbeat period) */
    uint16_t acquire_hz;
    uint16_t publish_hz;
    uint16_t log_hz;

    /* queue pressure / losses - never silently discard; count everything */
    uint32_t queue_high_water; /* worst queue fill seen, percent * 100 (0..10000) */
    uint32_t overflow_count;   /* records rejected by a full queue */
    uint32_t rejected_count;   /* malformed / undecodable records received */
    uint32_t write_error_count;
    uint32_t transport_error_count;

    /* CAN (TWAI) bus health */
    uint8_t  can_tec; /* transmit error counter (saturates at 255 for reporting) */
    uint8_t  can_rec; /* receive error counter */
    uint32_t can_bus_off_count;
    uint32_t can_tx_dropped; /* frames not queued for TX (queue full / bus-off) */
    uint32_t can_rx_dropped; /* frames lost on RX (queue full) */
    uint32_t can_bus_errors;

    /* sensors / storage */
    uint32_t sensor_status;     /* bit i set = sensor i healthy */
    uint32_t calibration_state; /* bit i set = sensor i calibrated */
    uint32_t last_sample_age_ms;
    uint32_t last_sd_write_age_ms;

    /* time base */
    uint32_t time_step_count; /* syncs applied as a step instead of a slew */
    int32_t  last_sync_error_us;
} lbr_health_record_t;

/*
 * Wire format. A health record is LBR_HEALTH_PAGE_COUNT fixed 8-byte pages.
 * On CAN each page is one frame (no multi-frame reassembly: any page can be
 * lost or reordered independently). In logs and transport packets the record
 * is [node_id u8] followed by all pages in order.
 *
 *   page  bytes 0..3                         bytes 4..7
 *   0     uptime_ms u32                      reset_reason u8, state u8, flags u16
 *   1     fw_version u32                     protocol_version u16, boot_count u16
 *   2     catalog_hash u32                   config_crc u32
 *   3     can_tec u8, can_rec u8, rsvd u16   can_bus_off_count u32
 *   4     can_tx_dropped u32                 can_rx_dropped u32
 *   5     overflow_count u32                 queue_high_water u32
 *   6     write_error_count u32              transport_error_count u32
 *   7     sensor_status u32                  calibration_state u32
 *   8     last_sample_age_ms u32             last_sd_write_age_ms u32
 *   9     acquire_hz u16, publish_hz u16     log_hz u16, rsvd u16
 *   10    time_step_count u32                last_sync_error_us i32
 *   11    rejected_count u32                 can_bus_errors u32
 *
 * Reserved fields are zero; a decoder rejects a page whose reserved field is
 * not zero.
 */
#define LBR_HEALTH_PAGE_COUNT 12u
#define LBR_HEALTH_PAGE_SIZE  8u
#define LBR_HEALTH_WIRE_SIZE  (1u + LBR_HEALTH_PAGE_COUNT * LBR_HEALTH_PAGE_SIZE)

/* Encode/decode one page. Decode only touches that page's fields. Return
 * false on a bad page index or a reserved field that is not zero. */
bool lbr_health_page_encode(const lbr_health_record_t *h, unsigned page, uint8_t out[8]);
bool lbr_health_page_decode(lbr_health_record_t *h, unsigned page, const uint8_t in[8]);

/* Whole record: returns LBR_HEALTH_WIRE_SIZE, or 0 on error. */
size_t lbr_health_encode(const lbr_health_record_t *h, uint8_t *buf, size_t cap);
size_t lbr_health_decode(const uint8_t *buf, size_t len, lbr_health_record_t *out);

/* ------------------------------------------------------------------------ */
/* Counter helpers                                                           */

/* Track a high-water mark expressed as percent * 100 of `cap`. */
static inline void lbr_high_water_update(uint32_t *hw, size_t used, size_t cap)
{
    if (cap == 0) {
        return;
    }
    uint32_t pct = (uint32_t)((used * 10000u) / cap);
    if (pct > *hw) {
        *hw = pct;
    }
}

/* Age in ms from `then_us` to `now_us`; LBR_AGE_NONE if `then_us` is 0 (never). */
static inline uint32_t lbr_age_ms(uint64_t now_us, uint64_t then_us)
{
    if (then_us == 0) {
        return LBR_AGE_NONE;
    }
    uint64_t ms = (now_us > then_us) ? (now_us - then_us) / 1000u : 0u;
    return ms >= LBR_AGE_NONE ? LBR_AGE_NONE - 1u : (uint32_t)ms;
}

/* ------------------------------------------------------------------------ */
/* Platform hooks (health_esp.c on target; tests provide their own)          */

/* Reset reason of the current boot, mapped to lbr_reset_reason_t. */
lbr_reset_reason_t lbr_health_reset_reason(void);

/* Persistent boot counter, incremented once per boot. */
uint32_t lbr_health_boot_count(void);
