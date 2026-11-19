#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "lbr/can.h"

/*
 * Firmware update over CAN (pure; host-tested). Operator guide:
 * docs/deploying.md. Wire format: docs/can-ids.md, "Firmware update".
 *
 * Only with the car parked. The laptop (dev/tools/lbr_deploy.py) drives each
 * step through the Central's USB link ("UPD ..." lines, lbr_upd_relay_line);
 * the Central puts the step on the bus and waits for the target node's
 * reply. A node writes the image into its spare OTA slot, checks it (CRC,
 * image hash, firmware made for this node), and switches to it only on
 * ACTIVATE. The new image must confirm itself by syncing to the Central's
 * clock, or the bootloader rolls it back (update_esp.c).
 *
 *   0x011      UPDATE_CTRL    Central, dlc 8: 0 op, 1 target node (0xFF all), 2..7 args
 *   0x012      UPDATE_DATA    Central, dlc 8: 0 low 8 bits of frame index k,
 *                             1..7 image bytes 7k .. 7k+6 (zero padded at the end)
 *   0x030 + n  UPDATE_STATUS  node n, dlc 8: 0 op answered, 1 result, 2 state,
 *                             3..7 per op (lbr_upd_status_t)
 *
 * CAN delivers one sender's frames in order and retransmits by itself, so a
 * data frame is lost only if the node's receive queue overflows or the node
 * restarts. A node that sees a frame out of sequence ignores data until the
 * next QUERY, whose reply says where to resume.
 */

#define LBR_UPD_VERSION         1u
#define LBR_UPD_BYTES_PER_FRAME 7u
#define LBR_UPD_TARGET_ALL      0xFFu
#define LBR_UPD_MAX_SIZE        0xFFFFFFu /* image size travels as 24 bits */

typedef enum {
    LBR_UPD_OP_BEGIN    = 1, /* args 0..2: image size u24. Starts a transfer (restarts one). */
    LBR_UPD_OP_QUERY    = 2, /* where the transfer stands; resumes data after a gap */
    LBR_UPD_OP_END      = 3, /* args 0..3: CRC-32 of the image. Check it and stage it. */
    LBR_UPD_OP_ACTIVATE = 4, /* boot the staged image (target or ALL) */
    LBR_UPD_OP_ABORT    = 5, /* drop a transfer or staged image (target or ALL) */
    LBR_UPD_OP_INFO     = 6, /* hash of the running image and whether it is confirmed */
} lbr_upd_op_t;

typedef enum {
    LBR_UPD_IDLE      = 0,
    LBR_UPD_RECEIVING = 1,
    LBR_UPD_STAGED    = 2, /* checked and waiting for ACTIVATE */
} lbr_upd_state_t;

typedef enum {
    LBR_UPD_OK           = 0,
    LBR_UPD_ERR_STATE    = 1, /* op not valid now (e.g. ACTIVATE with nothing staged) */
    LBR_UPD_ERR_SIZE     = 2, /* image empty or larger than the spare slot */
    LBR_UPD_ERR_LENGTH   = 3, /* END before every byte arrived */
    LBR_UPD_ERR_CRC      = 4, /* bytes received do not match the CRC: transfer dropped */
    LBR_UPD_ERR_IMAGE    = 5, /* not a valid image for this chip (header, hash) */
    LBR_UPD_ERR_WRONG_FW = 6, /* a valid image, but firmware for another node */
    LBR_UPD_ERR_FLASH    = 7, /* flash erase/write failed */
    LBR_UPD_ERR_NO_SLOT  = 8, /* no spare OTA slot: flash this node over USB once */
    LBR_UPD_ERR_OP       = 9, /* unknown op */
} lbr_upd_result_t;

const char *lbr_upd_result_name(uint8_t result);
const char *lbr_upd_state_name(uint8_t state);
const char *lbr_upd_op_name(uint8_t op);

/* Decoded UPDATE_STATUS. Bytes 3..7: for INFO, 3 = 1 if the running image
 * still awaits confirmation, 4..7 = first 4 bytes of its SHA-256 (big
 * endian, as printed); for every other op, 3..5 = next frame index k
 * expected (u24), 6 = LBR_UPD_VERSION, 7 = 0. */
typedef struct {
    uint8_t  node;
    uint8_t  op;
    uint8_t  result;
    uint8_t  state;
    uint32_t next_k;
    uint8_t  version;
    bool     pending;
    uint32_t hash;
} lbr_upd_status_t;

void lbr_upd_encode_ctrl(uint8_t op, uint8_t target, const uint8_t args[6], lbr_can_frame_t *out);
bool lbr_upd_decode_ctrl(const lbr_can_frame_t *f, uint8_t *op, uint8_t *target, uint8_t args[6]);
/* Frame k of an image: `n` (1..7) bytes, zero padded. */
void lbr_upd_encode_data(uint32_t k, const uint8_t *bytes, size_t n, lbr_can_frame_t *out);
void lbr_upd_encode_status(const lbr_upd_status_t *s, lbr_can_frame_t *out);
bool lbr_upd_decode_status(const lbr_can_frame_t *f, lbr_upd_status_t *out);

/* ------------------------------------------------------------------------ */
/* Node side                                                                 */

/* The node's flash, through the platform (update_esp.c on target). */
typedef struct {
    void *ctx;
    /* Open the spare slot for an image of `size` bytes. */
    lbr_upd_result_t (*begin)(void *ctx, uint32_t size);
    lbr_upd_result_t (*write)(void *ctx, const uint8_t *data, size_t len);
    /* Every byte written: close the slot, validate the image, check it is
     * firmware for this node. On failure the slot is left unused. */
    lbr_upd_result_t (*finish)(void *ctx);
    /* Forget an open or staged image (the running one is never touched). */
    void (*abort)(void *ctx);
    /* Boot the staged image. Does not return on target. */
    void (*activate)(void *ctx);
    /* Running image: first 4 bytes of its SHA-256; pending = not yet confirmed. */
    void (*info)(void *ctx, uint32_t *hash, bool *pending);
} lbr_upd_ota_t;

typedef struct {
    uint8_t              node_id;
    const lbr_upd_ota_t *ota; /* NULL: this platform cannot update (replies NO_SLOT) */
    bool (*send)(void *ctx, const lbr_can_frame_t *f);
    void *send_ctx;

    lbr_upd_state_t state;
    uint8_t         last_error; /* why a transfer stopped, until the next BEGIN */
    uint32_t        size;
    uint32_t        received; /* bytes written */
    uint32_t        next_k;   /* next data frame expected */
    uint32_t        crc;
    bool            resync; /* a frame was missed: ignore data until QUERY */

    uint32_t frames_ignored; /* data frames out of sequence or out of a transfer */
    uint32_t staged_count;
} lbr_upd_node_t;

void lbr_upd_node_init(lbr_upd_node_t *u, uint8_t node_id, const lbr_upd_ota_t *ota,
                       bool (*send)(void *ctx, const lbr_can_frame_t *f), void *send_ctx);

/* A received frame. Returns true if it was update traffic (consumed). Call
 * from one context only (the node's CAN receive task). */
bool lbr_upd_node_on_frame(lbr_upd_node_t *u, const lbr_can_frame_t *f);

/* ------------------------------------------------------------------------ */
/* Central side: USB "UPD" lines to bus traffic                              */

typedef struct {
    void *ctx;
    /* Queue a frame on the bus (and log it). false: not queued (TX full). */
    bool (*send)(void *ctx, const lbr_can_frame_t *f);
    /* Next UPDATE_STATUS received since the last flush, waiting up to
     * timeout_ms. false: none arrived. */
    bool (*wait_status)(void *ctx, uint32_t timeout_ms, lbr_upd_status_t *out);
    void (*flush_status)(void *ctx);
    void (*sleep_ms)(void *ctx, uint32_t ms);
    uint32_t (*now_ms)(void *ctx);
} lbr_upd_relay_hal_t;

/* Longest UPD line: a DATA line carries LBR_UPD_LINE_DATA_MAX image bytes
 * as base64. */
#define LBR_UPD_LINE_DATA_MAX 448u
#define LBR_UPD_LINE_MAX      720u

/*
 * Run one command (the text after "UPD ") and write the one-line reply:
 *
 *   <node> BEGIN <size>              -> UPD <node> BEGIN <RESULT> state=.. next=.. ver=..
 *   <node> DATA <k> <base64>         -> UPD <node> DATA <RESULT> state=.. next=.. ver=..
 *   <node> QUERY                     -> UPD <node> QUERY <RESULT> state=.. next=.. ver=..
 *   <node> END <crc32 hex>           -> UPD <node> END <RESULT> state=.. next=.. ver=..
 *   <node> INFO                      -> UPD <node> INFO OK state=.. hash=<8 hex> pending=0|1
 *   <node> ACTIVATE | ABORT          -> UPD <node> <OP> <RESULT> ...
 *   ALL ACTIVATE | ABORT             -> UPD ALL <OP> SENT
 *
 * <node> is a CAN node name (imu, corner_fl, ...) or number. No reply in
 * time gives "UPD <node> <OP> TIMEOUT"; a bad line "UPD ERR <reason>".
 * DATA sends frames k.. of the image, then a QUERY; `next` in the reply is
 * where the following DATA must start (a resend if bytes were missed).
 */
size_t lbr_upd_relay_line(const lbr_upd_relay_hal_t *hal, const char *line, char *reply,
                          size_t cap);

/* RFC 4648 base64 (no line breaks). Returns the decoded length, or -1. */
int lbr_upd_base64_decode(const char *in, size_t len, uint8_t *out, size_t cap);
