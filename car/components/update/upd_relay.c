#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "lbr/bytes.h"
#include "lbr/nodes.h"
#include "lbr/update.h"

/* Pure Central-side relay for firmware updates; see lbr/update.h. No platform headers here. */

/* How long a node may take to answer each step. END reads the whole image
 * back to check it; the others answer within a sector erase. */
#define TIMEOUT_MS_DEFAULT 2000u
#define TIMEOUT_MS_END     15000u

/* Data frames are queued in bursts so the Central's own traffic (time sync,
 * heartbeat) always finds room in the TX queue: 8 frames take about 2 ms
 * on a 500 kbit/s bus. */
#define BURST_FRAMES 8u
#define BURST_GAP_MS 2u
#define SEND_RETRIES 50u

typedef struct {
    const lbr_upd_relay_hal_t *hal;
    char                      *out;
    size_t                     cap;
    size_t                     len;
} relay_t;

static void say(relay_t *r, const char *fmt, ...) __attribute__((format(printf, 2, 3)));

static void say(relay_t *r, const char *fmt, ...)
{
    if (r->len >= r->cap) {
        return;
    }
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(r->out + r->len, r->cap - r->len, fmt, ap);
    va_end(ap);
    if (n > 0) {
        r->len += (size_t)n < r->cap - r->len ? (size_t)n : r->cap - r->len - 1;
    }
}

static bool send(relay_t *r, const lbr_can_frame_t *f)
{
    for (unsigned i = 0; i < SEND_RETRIES; i++) {
        if (r->hal->send(r->hal->ctx, f)) {
            return true;
        }
        r->hal->sleep_ms(r->hal->ctx, 1);
    }
    return false;
}

/* Wait for `node`'s answer to `op`; other status frames (late answers to
 * earlier steps, other nodes) are skipped. */
static bool await(relay_t *r, uint8_t node, uint8_t op, uint32_t timeout_ms, lbr_upd_status_t *st)
{
    uint32_t start = r->hal->now_ms(r->hal->ctx);
    for (;;) {
        uint32_t spent = r->hal->now_ms(r->hal->ctx) - start;
        if (spent >= timeout_ms) {
            return false;
        }
        if (!r->hal->wait_status(r->hal->ctx, timeout_ms - spent, st)) {
            return false;
        }
        if (st->node == node && st->op == op) {
            return true;
        }
    }
}

/* Next space-separated word of *p (NUL-terminated in place), or NULL. */
static char *word(char **p)
{
    while (**p == ' ') {
        (*p)++;
    }
    if (**p == '\0') {
        return NULL;
    }
    char *w = *p;
    while (**p != ' ' && **p != '\0') {
        (*p)++;
    }
    if (**p == ' ') {
        *(*p)++ = '\0';
    }
    return w;
}

/* A CAN sensor node (not the Central: it relays, and is flashed over USB). */
static bool parse_node(const char *s, uint8_t *node)
{
    if (strcmp(s, "ALL") == 0) {
        *node = LBR_UPD_TARGET_ALL;
        return true;
    }
    for (uint8_t id = 1; id <= LBR_NODE_CAN_MAX; id++) {
        if (id != LBR_NODE_CENTRAL && strcmp(s, lbr_node_name(id)) == 0) {
            *node = id;
            return true;
        }
    }
    char         *end;
    unsigned long v = strtoul(s, &end, 0);
    if (*s != '\0' && *end == '\0' && v <= LBR_NODE_CAN_MAX && lbr_node_is_can((uint8_t)v) &&
        v != LBR_NODE_CENTRAL) {
        *node = (uint8_t)v;
        return true;
    }
    return false;
}

static bool parse_u32(const char *s, int base, uint32_t *out)
{
    if (s == NULL || *s == '\0') {
        return false;
    }
    char         *end;
    unsigned long v = strtoul(s, &end, base);
    if (*end != '\0' || v > 0xFFFFFFFFul) {
        return false;
    }
    *out = (uint32_t)v;
    return true;
}

static void report(relay_t *r, const char *name, const char *opname, const lbr_upd_status_t *st)
{
    if (st->op == LBR_UPD_OP_INFO) {
        say(r, "UPD %s %s %s state=%s hash=%08lx pending=%u", name, opname,
            lbr_upd_result_name(st->result), lbr_upd_state_name(st->state), (unsigned long)st->hash,
            st->pending ? 1u : 0u);
    } else {
        say(r, "UPD %s %s %s state=%s next=%lu ver=%u", name, opname,
            lbr_upd_result_name(st->result), lbr_upd_state_name(st->state),
            (unsigned long)st->next_k, (unsigned)st->version);
    }
}

/* Send one control frame and report the target's answer. */
static void step(relay_t *r, uint8_t node, const char *name, uint8_t op, const uint8_t args[6],
                 const char *opname, uint32_t timeout_ms)
{
    lbr_can_frame_t  f;
    lbr_upd_status_t st;
    lbr_upd_encode_ctrl(op, node, args, &f);
    r->hal->flush_status(r->hal->ctx);
    if (!send(r, &f)) {
        say(r, "UPD %s %s ERR can_tx", name, opname);
    } else if (!await(r, node, op, timeout_ms, &st)) {
        say(r, "UPD %s %s TIMEOUT", name, opname);
    } else {
        report(r, name, opname, &st);
    }
}

static void do_data(relay_t *r, uint8_t node, const char *name, const char *k_text, const char *b64)
{
    uint32_t k;
    uint8_t  bytes[LBR_UPD_LINE_DATA_MAX];
    int      n = b64 != NULL ? lbr_upd_base64_decode(b64, strlen(b64), bytes, sizeof bytes) : -1;
    if (!parse_u32(k_text, 10, &k) || n <= 0) {
        say(r, "UPD ERR bad DATA line (DATA <k> <base64 of 1..%u bytes>)",
            (unsigned)LBR_UPD_LINE_DATA_MAX);
        return;
    }
    r->hal->flush_status(r->hal->ctx);
    size_t frames = ((size_t)n + LBR_UPD_BYTES_PER_FRAME - 1u) / LBR_UPD_BYTES_PER_FRAME;
    for (size_t i = 0; i < frames; i++) {
        size_t          off  = i * LBR_UPD_BYTES_PER_FRAME;
        size_t          left = (size_t)n - off;
        lbr_can_frame_t f;
        lbr_upd_encode_data(k + (uint32_t)i, bytes + off,
                            left < LBR_UPD_BYTES_PER_FRAME ? left : LBR_UPD_BYTES_PER_FRAME, &f);
        if (!send(r, &f)) {
            say(r, "UPD %s DATA ERR can_tx", name);
            return;
        }
        if ((i + 1u) % BURST_FRAMES == 0u) {
            r->hal->sleep_ms(r->hal->ctx, BURST_GAP_MS);
        }
    }
    /* The node handles frames in order, so its answer to this QUERY comes
     * after every data frame above. */
    lbr_can_frame_t  q;
    lbr_upd_status_t st;
    lbr_upd_encode_ctrl(LBR_UPD_OP_QUERY, node, NULL, &q);
    if (!send(r, &q)) {
        say(r, "UPD %s DATA ERR can_tx", name);
    } else if (!await(r, node, LBR_UPD_OP_QUERY, TIMEOUT_MS_DEFAULT, &st)) {
        say(r, "UPD %s DATA TIMEOUT", name);
    } else {
        report(r, name, "DATA", &st);
    }
}

size_t lbr_upd_relay_line(const lbr_upd_relay_hal_t *hal, const char *line, char *reply, size_t cap)
{
    relay_t r = {.hal = hal, .out = reply, .cap = cap, .len = 0};
    if (cap == 0) {
        return 0;
    }
    reply[0] = '\0';

    char   buf[LBR_UPD_LINE_MAX + 1];
    size_t n = strlen(line);
    if (n > LBR_UPD_LINE_MAX) {
        say(&r, "UPD ERR line too long");
        return r.len;
    }
    memcpy(buf, line, n + 1);
    char   *p   = buf;
    char   *who = word(&p);
    char   *op  = word(&p);
    char   *a1  = word(&p);
    char   *a2  = word(&p);
    uint8_t node;
    if (who == NULL || op == NULL || !parse_node(who, &node)) {
        say(&r, "UPD ERR usage: UPD <node|ALL> BEGIN|DATA|QUERY|END|INFO|ACTIVATE|ABORT ...");
        return r.len;
    }
    const char *name    = node == LBR_UPD_TARGET_ALL ? "ALL" : lbr_node_name(node);
    uint8_t     args[6] = {0};

    if (node == LBR_UPD_TARGET_ALL) {
        uint8_t         code = strcmp(op, "ACTIVATE") == 0 ? LBR_UPD_OP_ACTIVATE
                               : strcmp(op, "ABORT") == 0  ? LBR_UPD_OP_ABORT
                                                           : 0;
        lbr_can_frame_t f;
        if (code == 0) {
            say(&r, "UPD ERR ALL takes ACTIVATE or ABORT only");
        } else {
            lbr_upd_encode_ctrl(code, LBR_UPD_TARGET_ALL, NULL, &f);
            bool sent = send(&r, &f);
            say(&r, "UPD ALL %s %s", op, sent ? "SENT" : "ERR can_tx");
        }
        return r.len;
    }

    uint32_t v;
    if (strcmp(op, "BEGIN") == 0) {
        if (!parse_u32(a1, 10, &v) || v == 0 || v > LBR_UPD_MAX_SIZE) {
            say(&r, "UPD ERR BEGIN <size in bytes, 1..%lu>", (unsigned long)LBR_UPD_MAX_SIZE);
            return r.len;
        }
        lbr_put_u24(args, v);
        step(&r, node, name, LBR_UPD_OP_BEGIN, args, op, TIMEOUT_MS_DEFAULT);
    } else if (strcmp(op, "DATA") == 0) {
        do_data(&r, node, name, a1, a2);
    } else if (strcmp(op, "END") == 0) {
        if (!parse_u32(a1, 16, &v)) {
            say(&r, "UPD ERR END <crc32 in hex>");
            return r.len;
        }
        lbr_put_u32(args, v);
        step(&r, node, name, LBR_UPD_OP_END, args, op, TIMEOUT_MS_END);
    } else if (strcmp(op, "QUERY") == 0) {
        step(&r, node, name, LBR_UPD_OP_QUERY, args, op, TIMEOUT_MS_DEFAULT);
    } else if (strcmp(op, "INFO") == 0) {
        step(&r, node, name, LBR_UPD_OP_INFO, args, op, TIMEOUT_MS_DEFAULT);
    } else if (strcmp(op, "ACTIVATE") == 0) {
        step(&r, node, name, LBR_UPD_OP_ACTIVATE, args, op, TIMEOUT_MS_DEFAULT);
    } else if (strcmp(op, "ABORT") == 0) {
        step(&r, node, name, LBR_UPD_OP_ABORT, args, op, TIMEOUT_MS_DEFAULT);
    } else {
        say(&r, "UPD ERR unknown op %s", op);
    }
    return r.len;
}
