#include "lbr/update.h"

#include <string.h>

#include "lbr/bytes.h"
#include "lbr/crc32.h"
#include "lbr/nodes.h"

/* Pure firmware-update protocol, node side; see lbr/update.h. No platform headers here. */

const char *lbr_upd_result_name(uint8_t result)
{
    static const char *const names[] = {"OK",          "ERR_STATE", "ERR_SIZE",     "ERR_LENGTH",
                                        "ERR_CRC",     "ERR_IMAGE", "ERR_WRONG_FW", "ERR_FLASH",
                                        "ERR_NO_SLOT", "ERR_OP"};
    return result < sizeof names / sizeof names[0] ? names[result] : "ERR_UNKNOWN";
}

const char *lbr_upd_state_name(uint8_t state)
{
    static const char *const names[] = {"IDLE", "RECEIVING", "STAGED"};
    return state < sizeof names / sizeof names[0] ? names[state] : "UNKNOWN";
}

const char *lbr_upd_op_name(uint8_t op)
{
    static const char *const names[] = {"?", "BEGIN", "QUERY", "END", "ACTIVATE", "ABORT", "INFO"};
    return op < sizeof names / sizeof names[0] ? names[op] : "?";
}

/* ------------------------------------------------------------------------ */
/* Codec                                                                     */

void lbr_upd_encode_ctrl(uint8_t op, uint8_t target, const uint8_t args[6], lbr_can_frame_t *out)
{
    memset(out, 0, sizeof *out);
    out->id      = LBR_CAN_ID_UPDATE_CTRL;
    out->dlc     = 8;
    out->data[0] = op;
    out->data[1] = target;
    if (args != NULL) {
        memcpy(&out->data[2], args, 6);
    }
}

bool lbr_upd_decode_ctrl(const lbr_can_frame_t *f, uint8_t *op, uint8_t *target, uint8_t args[6])
{
    if (f->id != LBR_CAN_ID_UPDATE_CTRL || f->dlc != 8) {
        return false;
    }
    *op     = f->data[0];
    *target = f->data[1];
    memcpy(args, &f->data[2], 6);
    return true;
}

void lbr_upd_encode_data(uint32_t k, const uint8_t *bytes, size_t n, lbr_can_frame_t *out)
{
    memset(out, 0, sizeof *out);
    out->id      = LBR_CAN_ID_UPDATE_DATA;
    out->dlc     = 8;
    out->data[0] = (uint8_t)k;
    memcpy(&out->data[1], bytes, n < LBR_UPD_BYTES_PER_FRAME ? n : LBR_UPD_BYTES_PER_FRAME);
}

void lbr_upd_encode_status(const lbr_upd_status_t *s, lbr_can_frame_t *out)
{
    memset(out, 0, sizeof *out);
    out->id      = LBR_CAN_ID_UPDATE_STATUS + s->node;
    out->dlc     = 8;
    out->data[0] = s->op;
    out->data[1] = s->result;
    out->data[2] = s->state;
    if (s->op == LBR_UPD_OP_INFO) {
        out->data[3] = s->pending ? 1u : 0u;
        out->data[4] = (uint8_t)(s->hash >> 24);
        out->data[5] = (uint8_t)(s->hash >> 16);
        out->data[6] = (uint8_t)(s->hash >> 8);
        out->data[7] = (uint8_t)s->hash;
    } else {
        lbr_put_u24(&out->data[3], s->next_k);
        out->data[6] = s->version;
    }
}

bool lbr_upd_decode_status(const lbr_can_frame_t *f, lbr_upd_status_t *out)
{
    if (f->id <= LBR_CAN_ID_UPDATE_STATUS || f->id > LBR_CAN_ID_UPDATE_STATUS + LBR_NODE_CAN_MAX ||
        f->dlc != 8) {
        return false;
    }
    memset(out, 0, sizeof *out);
    out->node   = (uint8_t)(f->id - LBR_CAN_ID_UPDATE_STATUS);
    out->op     = f->data[0];
    out->result = f->data[1];
    out->state  = f->data[2];
    if (out->op == LBR_UPD_OP_INFO) {
        out->pending = f->data[3] != 0;
        out->hash    = (uint32_t)f->data[4] << 24 | (uint32_t)f->data[5] << 16 |
                       (uint32_t)f->data[6] << 8 | f->data[7];
    } else {
        out->next_k  = lbr_get_u24(&f->data[3]);
        out->version = f->data[6];
    }
    return true;
}

/* ------------------------------------------------------------------------ */
/* Node receiver                                                             */

void lbr_upd_node_init(lbr_upd_node_t *u, uint8_t node_id, const lbr_upd_ota_t *ota,
                       bool (*send)(void *ctx, const lbr_can_frame_t *f), void *send_ctx)
{
    memset(u, 0, sizeof *u);
    u->node_id  = node_id;
    u->ota      = ota;
    u->send     = send;
    u->send_ctx = send_ctx;
}

static void reply(lbr_upd_node_t *u, uint8_t op, uint8_t result)
{
    lbr_upd_status_t s = {
        .node    = u->node_id,
        .op      = op,
        .result  = result,
        .state   = (uint8_t)u->state,
        .next_k  = u->next_k,
        .version = LBR_UPD_VERSION,
    };
    if (op == LBR_UPD_OP_INFO && u->ota != NULL) {
        u->ota->info(u->ota->ctx, &s.hash, &s.pending);
    }
    lbr_can_frame_t f;
    lbr_upd_encode_status(&s, &f);
    if (u->send != NULL) {
        u->send(u->send_ctx, &f);
    }
}

/* Leave any transfer; `why` is reported until the next BEGIN. */
static void drop(lbr_upd_node_t *u, uint8_t why)
{
    if (u->state != LBR_UPD_IDLE) {
        u->ota->abort(u->ota->ctx);
    }
    u->state      = LBR_UPD_IDLE;
    u->last_error = why;
    u->resync     = false;
}

static void on_data(lbr_upd_node_t *u, const lbr_can_frame_t *f)
{
    if (u->state != LBR_UPD_RECEIVING || u->resync || f->dlc != 8) {
        u->frames_ignored++;
        return;
    }
    if (f->data[0] != (uint8_t)u->next_k) {
        /* A frame went missing: wait for the Central to ask, then resume. */
        u->resync = true;
        u->frames_ignored++;
        return;
    }
    uint32_t left = u->size - u->received;
    size_t   n    = left < LBR_UPD_BYTES_PER_FRAME ? left : LBR_UPD_BYTES_PER_FRAME;
    if (n == 0) {
        u->frames_ignored++; /* past the end */
        return;
    }
    lbr_upd_result_t r = u->ota->write(u->ota->ctx, &f->data[1], n);
    if (r != LBR_UPD_OK) {
        drop(u, (uint8_t)r);
        return;
    }
    u->crc = lbr_crc32_update(u->crc, &f->data[1], n);
    u->received += (uint32_t)n;
    u->next_k++;
}

static void on_ctrl(lbr_upd_node_t *u, uint8_t op, uint8_t target, const uint8_t args[6])
{
    bool all = target == LBR_UPD_TARGET_ALL;
    if (!all && target != u->node_id) {
        return;
    }
    if (all) {
        /* Broadcasts act only where there is something to act on, and only
         * those nodes answer. */
        if (u->ota == NULL) {
            return;
        }
        if (op == LBR_UPD_OP_ACTIVATE && u->state == LBR_UPD_STAGED) {
            reply(u, op, LBR_UPD_OK);
            u->state = LBR_UPD_IDLE;
            u->ota->activate(u->ota->ctx);
        } else if (op == LBR_UPD_OP_ABORT && u->state != LBR_UPD_IDLE) {
            drop(u, LBR_UPD_OK);
            reply(u, op, LBR_UPD_OK);
        }
        return;
    }
    if (u->ota == NULL) {
        reply(u, op, LBR_UPD_ERR_NO_SLOT);
        return;
    }

    switch (op) {
        case LBR_UPD_OP_BEGIN: {
            uint32_t size = lbr_get_u24(args);
            drop(u, LBR_UPD_OK); /* a new BEGIN replaces whatever was going on */
            u->next_k   = 0;
            u->received = 0;
            u->crc      = 0;
            if (size == 0) {
                reply(u, op, LBR_UPD_ERR_SIZE);
                return;
            }
            lbr_upd_result_t r = u->ota->begin(u->ota->ctx, size);
            if (r != LBR_UPD_OK) {
                u->last_error = (uint8_t)r;
                reply(u, op, (uint8_t)r);
                return;
            }
            u->size  = size;
            u->state = LBR_UPD_RECEIVING;
            reply(u, op, LBR_UPD_OK);
            return;
        }
        case LBR_UPD_OP_QUERY:
            u->resync = false;
            reply(u, op, u->state == LBR_UPD_IDLE ? u->last_error : LBR_UPD_OK);
            return;
        case LBR_UPD_OP_END: {
            if (u->state != LBR_UPD_RECEIVING) {
                reply(u, op, u->state == LBR_UPD_STAGED ? LBR_UPD_OK : LBR_UPD_ERR_STATE);
                return;
            }
            if (u->received != u->size) {
                reply(u, op, LBR_UPD_ERR_LENGTH);
                return;
            }
            if (u->crc != lbr_get_u32(args)) {
                drop(u, LBR_UPD_ERR_CRC);
                reply(u, op, LBR_UPD_ERR_CRC);
                return;
            }
            lbr_upd_result_t r = u->ota->finish(u->ota->ctx);
            if (r != LBR_UPD_OK) {
                u->state      = LBR_UPD_IDLE; /* finish already released the slot */
                u->last_error = (uint8_t)r;
                reply(u, op, (uint8_t)r);
                return;
            }
            u->state = LBR_UPD_STAGED;
            u->staged_count++;
            reply(u, op, LBR_UPD_OK);
            return;
        }
        case LBR_UPD_OP_ACTIVATE:
            if (u->state != LBR_UPD_STAGED) {
                reply(u, op, LBR_UPD_ERR_STATE);
                return;
            }
            reply(u, op, LBR_UPD_OK);
            u->state = LBR_UPD_IDLE;
            u->ota->activate(u->ota->ctx);
            return;
        case LBR_UPD_OP_ABORT:
            drop(u, LBR_UPD_OK);
            reply(u, op, LBR_UPD_OK);
            return;
        case LBR_UPD_OP_INFO:
            reply(u, op, LBR_UPD_OK);
            return;
        default:
            reply(u, op, LBR_UPD_ERR_OP);
            return;
    }
}

bool lbr_upd_node_on_frame(lbr_upd_node_t *u, const lbr_can_frame_t *f)
{
    if (f->id == LBR_CAN_ID_UPDATE_DATA) {
        on_data(u, f);
        return true;
    }
    uint8_t op, target, args[6];
    if (lbr_upd_decode_ctrl(f, &op, &target, args)) {
        on_ctrl(u, op, target, args);
        return true;
    }
    return f->id == LBR_CAN_ID_UPDATE_CTRL; /* malformed control frame: still ours */
}

/* ------------------------------------------------------------------------ */
/* Base64                                                                    */

static int b64_value(char c)
{
    if (c >= 'A' && c <= 'Z') {
        return c - 'A';
    }
    if (c >= 'a' && c <= 'z') {
        return c - 'a' + 26;
    }
    if (c >= '0' && c <= '9') {
        return c - '0' + 52;
    }
    if (c == '+') {
        return 62;
    }
    if (c == '/') {
        return 63;
    }
    return -1;
}

int lbr_upd_base64_decode(const char *in, size_t len, uint8_t *out, size_t cap)
{
    if (len % 4 != 0) {
        return -1;
    }
    size_t n = 0;
    for (size_t i = 0; i < len; i += 4) {
        int  v[4];
        int  pad  = 0;
        bool last = i + 4 == len;
        for (int j = 0; j < 4; j++) {
            char c = in[i + (size_t)j];
            if (c == '=' && last && j >= 2) {
                v[j] = 0;
                pad++;
            } else if (pad > 0 || (v[j] = b64_value(c)) < 0) {
                return -1; /* bad character, or data after padding */
            }
        }
        uint32_t w =
            (uint32_t)v[0] << 18 | (uint32_t)v[1] << 12 | (uint32_t)v[2] << 6 | (uint32_t)v[3];
        size_t m = 3u - (size_t)pad;
        if (n + m > cap) {
            return -1;
        }
        out[n++] = (uint8_t)(w >> 16);
        if (m > 1) {
            out[n++] = (uint8_t)(w >> 8);
        }
        if (m > 2) {
            out[n++] = (uint8_t)w;
        }
    }
    return (int)n;
}
