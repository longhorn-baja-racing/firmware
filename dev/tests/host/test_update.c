#include <stdlib.h>

#include "lbr/crc32.h"
#include "lbr/nodes.h"
#include "lbr/update.h"
#include "lbr_test.h"

/*
 * Firmware update over CAN: the Central's relay (USB lines in, bus frames
 * out) against node receivers on a fake bus with fake flash. Everything is
 * synchronous: a frame the relay sends is handled by every node before
 * send() returns, and their replies queue up for wait_status().
 */

#define MAX_NODES 3
#define FLASH_CAP (64u * 1024u)

typedef struct {
    uint8_t          mem[FLASH_CAP];
    uint32_t         size, len;
    bool             open, staged, activated;
    int              begin_calls, abort_calls;
    lbr_upd_result_t finish_result;
    uint32_t         running_hash;
    bool             pending;
} fake_flash_t;

typedef struct {
    lbr_upd_node_t   node[MAX_NODES];
    fake_flash_t     flash[MAX_NODES];
    lbr_upd_ota_t    ota[MAX_NODES];
    size_t           n_nodes;
    lbr_upd_status_t q[64];
    size_t           qn;
    uint32_t         now_ms;
    long             drop_data_k; /* data frame index to lose once (-1: none) */
    uint32_t         data_frames;
} bus_t;

static bus_t B;

/* ---- fake flash -------------------------------------------------------- */

static lbr_upd_result_t f_begin(void *ctx, uint32_t size)
{
    fake_flash_t *f = ctx;
    f->begin_calls++;
    if (size > FLASH_CAP) {
        return LBR_UPD_ERR_SIZE;
    }
    f->size   = size;
    f->len    = 0;
    f->open   = true;
    f->staged = false;
    return LBR_UPD_OK;
}

static lbr_upd_result_t f_write(void *ctx, const uint8_t *data, size_t len)
{
    fake_flash_t *f = ctx;
    if (!f->open || f->len + len > f->size) {
        return LBR_UPD_ERR_FLASH;
    }
    memcpy(f->mem + f->len, data, len);
    f->len += (uint32_t)len;
    return LBR_UPD_OK;
}

static lbr_upd_result_t f_finish(void *ctx)
{
    fake_flash_t *f = ctx;
    f->open         = false;
    if (f->finish_result != LBR_UPD_OK) {
        return f->finish_result;
    }
    f->staged = true;
    return LBR_UPD_OK;
}

static void f_abort(void *ctx)
{
    fake_flash_t *f = ctx;
    f->abort_calls++;
    f->open   = false;
    f->staged = false;
}

static void f_activate(void *ctx)
{
    fake_flash_t *f = ctx;
    if (f->staged) {
        f->activated = true;
    }
}

static void f_info(void *ctx, uint32_t *hash, bool *pending)
{
    fake_flash_t *f = ctx;
    *hash           = f->running_hash;
    *pending        = f->pending;
}

/* ---- fake bus ---------------------------------------------------------- */

static bool node_send(void *ctx, const lbr_can_frame_t *f)
{
    (void)ctx;
    lbr_upd_status_t st;
    CHECK(lbr_upd_decode_status(f, &st));
    CHECK_EQ_U(lbr_can_sender(f->id), st.node);
    if (B.qn < sizeof B.q / sizeof B.q[0]) {
        B.q[B.qn++] = st;
    }
    return true;
}

static bool bus_send(void *ctx, const lbr_can_frame_t *f)
{
    (void)ctx;
    CHECK_EQ_U(lbr_can_sender(f->id), LBR_NODE_CENTRAL);
    if (f->id == LBR_CAN_ID_UPDATE_DATA) {
        B.data_frames++;
        if (B.drop_data_k >= 0 && f->data[0] == (uint8_t)B.drop_data_k) {
            B.drop_data_k = -1; /* lost once (receive queue overflow) */
            return true;
        }
    }
    for (size_t i = 0; i < B.n_nodes; i++) {
        lbr_upd_node_on_frame(&B.node[i], f);
    }
    return true;
}

static bool bus_wait(void *ctx, uint32_t timeout_ms, lbr_upd_status_t *out)
{
    (void)ctx;
    if (B.qn == 0) {
        B.now_ms += timeout_ms;
        return false;
    }
    *out = B.q[0];
    memmove(B.q, B.q + 1, (B.qn - 1) * sizeof B.q[0]);
    B.qn--;
    return true;
}

static void bus_flush(void *ctx)
{
    (void)ctx;
    B.qn = 0;
}

static void bus_sleep(void *ctx, uint32_t ms)
{
    (void)ctx;
    B.now_ms += ms;
}

static uint32_t bus_now(void *ctx)
{
    (void)ctx;
    return B.now_ms;
}

static const lbr_upd_relay_hal_t k_relay = {
    .send         = bus_send,
    .wait_status  = bus_wait,
    .flush_status = bus_flush,
    .sleep_ms     = bus_sleep,
    .now_ms       = bus_now,
};

/* Bus with nodes `ids` (0 ends the list); with_ota false = old layout. */
static void setup(const uint8_t *ids, bool with_ota)
{
    memset(&B, 0, sizeof B);
    B.drop_data_k = -1;
    for (size_t i = 0; ids[i] != 0 && i < MAX_NODES; i++) {
        B.ota[i] = (lbr_upd_ota_t){
            .ctx      = &B.flash[i],
            .begin    = f_begin,
            .write    = f_write,
            .finish   = f_finish,
            .abort    = f_abort,
            .activate = f_activate,
            .info     = f_info,
        };
        lbr_upd_node_init(&B.node[i], ids[i], with_ota ? &B.ota[i] : NULL, node_send, NULL);
        B.n_nodes++;
    }
}

static const char *cmd(const char *line)
{
    static char reply[256];
    lbr_upd_relay_line(&k_relay, line, reply, sizeof reply);
    return reply;
}

static void b64(const uint8_t *in, size_t n, char *out)
{
    static const char t[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
    size_t            o   = 0;
    for (size_t i = 0; i < n; i += 3) {
        uint32_t w = (uint32_t)in[i] << 16;
        w |= i + 1 < n ? (uint32_t)in[i + 1] << 8 : 0u;
        w |= i + 2 < n ? (uint32_t)in[i + 2] : 0u;
        out[o++] = t[(w >> 18) & 63];
        out[o++] = t[(w >> 12) & 63];
        out[o++] = i + 1 < n ? t[(w >> 6) & 63] : '=';
        out[o++] = i + 2 < n ? t[w & 63] : '=';
    }
    out[o] = '\0';
}

/* What lbr_deploy.py does: DATA chunks from wherever the node says. */
static bool upload(const char *node, const uint8_t *img, uint32_t size, int *resends)
{
    char     enc[(LBR_UPD_LINE_DATA_MAX + 2) / 3 * 4 + 1];
    char     line[sizeof enc + 64];
    uint32_t k = 0;
    for (int guard = 0; guard < 1000; guard++) {
        uint32_t off = k * LBR_UPD_BYTES_PER_FRAME;
        if (off >= size) {
            return true;
        }
        uint32_t n = size - off < LBR_UPD_LINE_DATA_MAX ? size - off : LBR_UPD_LINE_DATA_MAX;
        b64(img + off, n, enc);
        snprintf(line, sizeof line, "%s DATA %lu %s", node, (unsigned long)k, enc);
        const char *r    = cmd(line);
        const char *next = strstr(r, "next=");
        if (strstr(r, " DATA OK state=RECEIVING ") == NULL || next == NULL) {
            return false;
        }
        uint32_t want = k + (n + LBR_UPD_BYTES_PER_FRAME - 1) / LBR_UPD_BYTES_PER_FRAME;
        k             = (uint32_t)strtoul(next + 5, NULL, 10);
        if (k != want && resends != NULL) {
            (*resends)++;
        }
    }
    return false;
}

static void make_image(uint8_t *img, uint32_t size, uint32_t seed)
{
    for (uint32_t i = 0; i < size; i++) {
        seed   = seed * 1103515245u + 12345u;
        img[i] = (uint8_t)(seed >> 16);
    }
}

/* ---- tests ------------------------------------------------------------- */

static void test_codec(void)
{
    lbr_can_frame_t f;
    uint8_t         args[6] = {1, 2, 3, 4, 5, 6}, got[6], op, target;
    lbr_upd_encode_ctrl(LBR_UPD_OP_END, 7, args, &f);
    CHECK_EQ_U(f.id, 0x011);
    CHECK(lbr_upd_decode_ctrl(&f, &op, &target, got));
    CHECK_EQ_U(op, LBR_UPD_OP_END);
    CHECK_EQ_U(target, 7);
    CHECK(memcmp(got, args, 6) == 0);

    uint8_t bytes[3] = {0xAA, 0xBB, 0xCC};
    lbr_upd_encode_data(0x1234, bytes, 3, &f);
    CHECK_EQ_U(f.id, 0x012);
    CHECK_EQ_U(f.dlc, 8);
    CHECK_EQ_U(f.data[0], 0x34);
    CHECK_EQ_U(f.data[3], 0xCC);
    CHECK_EQ_U(f.data[4], 0);

    lbr_upd_status_t s = {.node    = 10,
                          .op      = LBR_UPD_OP_QUERY,
                          .result  = LBR_UPD_ERR_CRC,
                          .state   = LBR_UPD_IDLE,
                          .next_k  = 0xABCDEF,
                          .version = 1};
    lbr_upd_status_t d;
    lbr_upd_encode_status(&s, &f);
    CHECK_EQ_U(f.id, 0x03A);
    CHECK(lbr_upd_decode_status(&f, &d));
    CHECK_EQ_U(d.node, 10);
    CHECK_EQ_U(d.next_k, 0xABCDEF);
    CHECK_EQ_U(d.result, LBR_UPD_ERR_CRC);
    s = (lbr_upd_status_t){.node = 2, .op = LBR_UPD_OP_INFO, .hash = 0xDEADBEEF, .pending = true};
    lbr_upd_encode_status(&s, &f);
    CHECK(lbr_upd_decode_status(&f, &d));
    CHECK_EQ_U(d.hash, 0xDEADBEEF);
    CHECK(d.pending);

    /* Sender of every id; 0x030 itself and 0x03B+ are nobody's. */
    CHECK_EQ_U(lbr_can_sender(LBR_CAN_ID_UPDATE_CTRL), LBR_NODE_CENTRAL);
    CHECK_EQ_U(lbr_can_sender(LBR_CAN_ID_UPDATE_DATA), LBR_NODE_CENTRAL);
    CHECK_EQ_U(lbr_can_sender(0x031), 1);
    CHECK_EQ_U(lbr_can_sender(0x03A), 10);
    CHECK_EQ_U(lbr_can_sender(0x030), 0);
    CHECK_EQ_U(lbr_can_sender(0x03B), 0);
    f.id = 0x030;
    CHECK(!lbr_upd_decode_status(&f, &d));
}

static void test_base64(void)
{
    uint8_t out[8];
    CHECK_EQ_I(lbr_upd_base64_decode("TWFu", 4, out, sizeof out), 3);
    CHECK(memcmp(out, "Man", 3) == 0);
    CHECK_EQ_I(lbr_upd_base64_decode("TWE=", 4, out, sizeof out), 2);
    CHECK_EQ_I(lbr_upd_base64_decode("TQ==", 4, out, sizeof out), 1);
    CHECK_EQ_I(lbr_upd_base64_decode("TQ=", 3, out, sizeof out), -1);
    CHECK_EQ_I(lbr_upd_base64_decode("T=Q=", 4, out, sizeof out), -1);
    CHECK_EQ_I(lbr_upd_base64_decode("TQ==TWFu", 8, out, sizeof out), -1);
    CHECK_EQ_I(lbr_upd_base64_decode("TW!u", 4, out, sizeof out), -1);
    CHECK_EQ_I(lbr_upd_base64_decode("TWFuTWFu", 8, out, 5), -1);
}

static void test_stage_and_activate(void)
{
    static const uint8_t ids[] = {LBR_NODE_IMU, LBR_NODE_POWERTRAIN, 0};
    static uint8_t       img[5000];
    char                 line[64];
    setup(ids, true);
    make_image(img, sizeof img, 1);

    CHECK(strstr(cmd("imu BEGIN 5000"), "UPD imu BEGIN OK state=RECEIVING next=0 ver=1") != NULL);
    CHECK(upload("imu", img, sizeof img, NULL));
    snprintf(line, sizeof line, "imu END %08lx", (unsigned long)lbr_crc32(img, sizeof img));
    CHECK(strstr(cmd(line), "UPD imu END OK state=STAGED") != NULL);
    CHECK_EQ_U(B.flash[0].len, sizeof img);
    CHECK(memcmp(B.flash[0].mem, img, sizeof img) == 0);
    CHECK(B.flash[0].staged);
    /* The other node heard everything and did nothing. */
    CHECK_EQ_I(B.flash[1].begin_calls, 0);
    CHECK_EQ_U(B.node[1].state, LBR_UPD_IDLE);
    /* A repeated END (lost reply) is harmless. */
    CHECK(strstr(cmd(line), "END OK state=STAGED") != NULL);

    /* ALL ACTIVATE boots staged nodes only; only they answer. */
    CHECK(strstr(cmd("ALL ACTIVATE"), "UPD ALL ACTIVATE SENT") != NULL);
    CHECK(B.flash[0].activated);
    CHECK(!B.flash[1].activated);
    CHECK_EQ_U(B.qn, 1);
    CHECK_EQ_U(B.q[0].node, LBR_NODE_IMU);

    B.flash[0].running_hash = 0x0123ABCD;
    B.flash[0].pending      = true;
    CHECK(strstr(cmd("imu INFO"), "UPD imu INFO OK state=IDLE hash=0123abcd pending=1") != NULL);
    CHECK(strstr(cmd("2 INFO"), "UPD powertrain INFO OK") != NULL);
}

static void test_lost_frame_is_resent(void)
{
    static const uint8_t ids[] = {LBR_NODE_STRESS, 0};
    static uint8_t       img[3001];
    char                 line[64];
    int                  resends = 0;
    setup(ids, true);
    make_image(img, sizeof img, 2);
    B.drop_data_k = 5;

    CHECK(strstr(cmd("stress BEGIN 3001"), "BEGIN OK") != NULL);
    CHECK(upload("stress", img, sizeof img, &resends));
    CHECK_EQ_I(resends, 1);
    CHECK(B.node[0].frames_ignored > 0);
    snprintf(line, sizeof line, "stress END %08lx", (unsigned long)lbr_crc32(img, sizeof img));
    CHECK(strstr(cmd(line), "END OK state=STAGED") != NULL);
    CHECK(memcmp(B.flash[0].mem, img, sizeof img) == 0);
}

static void test_bad_image_never_staged(void)
{
    static const uint8_t ids[] = {LBR_NODE_CORNER_FL, 0};
    static uint8_t       img[700];
    char                 line[64];
    setup(ids, true);
    make_image(img, sizeof img, 3);

    /* Corrupt CRC: dropped, and QUERY keeps saying why. */
    CHECK(strstr(cmd("corner_fl BEGIN 700"), "BEGIN OK") != NULL);
    CHECK(upload("corner_fl", img, sizeof img, NULL));
    snprintf(line, sizeof line, "corner_fl END %08lx",
             (unsigned long)(lbr_crc32(img, sizeof img) ^ 1u));
    CHECK(strstr(cmd(line), "END ERR_CRC state=IDLE") != NULL);
    CHECK(strstr(cmd("corner_fl QUERY"), "QUERY ERR_CRC state=IDLE") != NULL);
    CHECK(strstr(cmd("corner_fl ACTIVATE"), "ACTIVATE ERR_STATE") != NULL);
    CHECK(!B.flash[0].activated);

    /* Firmware for another node: refused at END. */
    B.flash[0].finish_result = LBR_UPD_ERR_WRONG_FW;
    CHECK(strstr(cmd("corner_fl BEGIN 700"), "BEGIN OK") != NULL);
    CHECK(upload("corner_fl", img, sizeof img, NULL));
    snprintf(line, sizeof line, "corner_fl END %08lx", (unsigned long)lbr_crc32(img, sizeof img));
    CHECK(strstr(cmd(line), "END ERR_WRONG_FW state=IDLE") != NULL);
    CHECK(!B.flash[0].staged);
    CHECK(strstr(cmd("ALL ACTIVATE"), "SENT") != NULL);
    CHECK(!B.flash[0].activated);

    /* END before every byte: not staged, transfer continues. */
    B.flash[0].finish_result = LBR_UPD_OK;
    CHECK(strstr(cmd("corner_fl BEGIN 700"), "BEGIN OK") != NULL);
    CHECK(strstr(cmd(line), "END ERR_LENGTH state=RECEIVING") != NULL);

    /* Too big for the slot, or empty. */
    CHECK(strstr(cmd("corner_fl BEGIN 70000"), "BEGIN ERR_SIZE state=IDLE") != NULL);
    CHECK(strstr(cmd("corner_fl BEGIN 0"), "UPD ERR BEGIN") != NULL);
}

static void test_abort_and_restart(void)
{
    static const uint8_t ids[] = {LBR_NODE_TEMPERATURE, 0};
    static uint8_t       img[200];
    setup(ids, true);
    make_image(img, sizeof img, 4);

    CHECK(strstr(cmd("temperature BEGIN 200"), "BEGIN OK") != NULL);
    CHECK(strstr(cmd("ALL ABORT"), "SENT") != NULL);
    CHECK_EQ_U(B.node[0].state, LBR_UPD_IDLE);
    CHECK_EQ_I(B.flash[0].abort_calls, 1);

    /* Node restarts mid-transfer (power cut): it forgot, and says so. */
    CHECK(strstr(cmd("temperature BEGIN 200"), "BEGIN OK") != NULL);
    lbr_upd_node_init(&B.node[0], LBR_NODE_TEMPERATURE, &B.ota[0], node_send, NULL);
    CHECK(!upload("temperature", img, sizeof img, NULL));
    CHECK(strstr(cmd("temperature QUERY"), "state=IDLE") != NULL);

    /* A second BEGIN restarts from zero. */
    CHECK(strstr(cmd("temperature BEGIN 200"), "BEGIN OK") != NULL);
    CHECK(upload("temperature", img, 100, NULL));
    CHECK(strstr(cmd("temperature BEGIN 200"), "BEGIN OK state=RECEIVING next=0") != NULL);
}

static void test_no_slot_and_silence(void)
{
    static const uint8_t ids[] = {LBR_NODE_DRIVER_INPUT, 0};
    setup(ids, false);
    CHECK(strstr(cmd("driver_input BEGIN 100"), "BEGIN ERR_NO_SLOT") != NULL);
    CHECK(strstr(cmd("driver_input INFO"), "INFO ERR_NO_SLOT") != NULL);
    /* Nobody home. */
    CHECK(strstr(cmd("corner_br QUERY"), "UPD corner_br QUERY TIMEOUT") != NULL);
}

static void test_bad_lines(void)
{
    static const uint8_t ids[] = {LBR_NODE_IMU, 0};
    setup(ids, true);
    CHECK(strstr(cmd("imu FROB"), "UPD ERR unknown op FROB") != NULL);
    CHECK(strstr(cmd("cockpit BEGIN 10"), "UPD ERR usage") != NULL); /* not on CAN */
    CHECK(strstr(cmd("central BEGIN 10"), "UPD ERR usage") != NULL); /* relays; USB only */
    CHECK(strstr(cmd("6 BEGIN 10"), "UPD ERR usage") != NULL);
    CHECK(strstr(cmd(""), "UPD ERR usage") != NULL);
    CHECK(strstr(cmd("imu DATA 0 !!!!"), "UPD ERR bad DATA") != NULL);
    CHECK(strstr(cmd("imu DATA x TWFu"), "UPD ERR bad DATA") != NULL);
    CHECK(strstr(cmd("imu END xyz"), "UPD ERR END") != NULL);
    CHECK(strstr(cmd("ALL BEGIN 1"), "UPD ERR ALL takes") != NULL);
    char big[LBR_UPD_LINE_MAX + 10];
    memset(big, 'A', sizeof big - 1);
    big[sizeof big - 1] = '\0';
    CHECK(strstr(cmd(big), "UPD ERR line too long") != NULL);
    CHECK_EQ_U(B.flash[0].begin_calls, 0);
}

int main(void)
{
    RUN(test_codec);
    RUN(test_base64);
    RUN(test_stage_and_activate);
    RUN(test_lost_frame_is_resent);
    RUN(test_bad_image_never_staged);
    RUN(test_abort_and_restart);
    RUN(test_no_slot_and_silence);
    RUN(test_bad_lines);
    TEST_MAIN_END();
}
