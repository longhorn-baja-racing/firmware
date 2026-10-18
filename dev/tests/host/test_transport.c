#include "lbr/cobs.h"
#include "lbr/lora_policy.h"
#include "lbr/packet.h"
#include "lbr/transport.h"
#include "lbr_test.h"

typedef struct {
    int     n;
    uint8_t types[64];
    size_t  lens[64];
} seen_t;

static void collect(uint8_t type, const uint8_t *payload, size_t len, void *ctx)
{
    (void)payload;
    seen_t *s = ctx;
    if (s->n < 64) {
        s->types[s->n] = type;
        s->lens[s->n]  = len;
    }
    s->n++;
}

static void test_packet_roundtrip_over_cobs(void)
{
    uint8_t           pkt[LBR_PKT_MAX_SIZE];
    lbr_pkt_builder_t b;
    lbr_pkt_begin(&b, pkt, sizeof pkt, LBR_LINK_USB, 1, 0xFFFFFFFFu, 123456789);
    CHECK_EQ_U(lbr_pkt_finish(&b), 0u); /* empty packets are not sent */
    uint8_t zeros[22] = {0};
    CHECK(lbr_pkt_add(&b, LBR_REC_CAN_FRAME, zeros, sizeof zeros));
    CHECK(lbr_pkt_add(&b, LBR_REC_TEXT, (const uint8_t *)"hi", 2));
    CHECK(lbr_pkt_add(&b, LBR_REC_TEXT, NULL, 0));
    size_t n = lbr_pkt_finish(&b);
    CHECK(n > 0);

    uint8_t enc[LBR_COBS_MAX_ENCODED(LBR_PKT_MAX_SIZE)];
    uint8_t dec[LBR_PKT_MAX_SIZE];
    size_t  e = lbr_cobs_encode(pkt, n, enc, sizeof enc);
    CHECK_EQ_U(lbr_cobs_decode(enc, e, dec, sizeof dec), n);

    lbr_pkt_header_t h;
    seen_t           seen = {0};
    CHECK(lbr_pkt_parse(dec, n, &h, collect, &seen));
    CHECK_EQ_U(h.link, LBR_LINK_USB);
    CHECK_EQ_U(h.seq, 0xFFFFFFFFu);
    CHECK_EQ_U(h.t_us, 123456789u);
    CHECK_EQ_I(seen.n, 3);
    CHECK_EQ_U(seen.types[0], LBR_REC_CAN_FRAME);
    CHECK_EQ_U(seen.lens[0], 22u);
    CHECK_EQ_U(seen.lens[2], 0u);
}

static void test_packet_rejects(void)
{
    uint8_t           pkt[64];
    lbr_pkt_builder_t b;
    lbr_pkt_begin(&b, pkt, sizeof pkt, LBR_LINK_LORA, 1, 7, 0);
    uint8_t big[64] = {0};
    CHECK(!lbr_pkt_add(&b, LBR_REC_TEXT, big, 60)); /* does not fit */
    CHECK(lbr_pkt_add(&b, LBR_REC_TEXT, big, 10));
    size_t n = lbr_pkt_finish(&b);

    lbr_pkt_header_t h;
    seen_t           seen = {0};
    for (size_t i = 0; i < n; i++) {
        pkt[i] ^= 0x01; /* any single-bit error is caught */
        CHECK(!lbr_pkt_parse(pkt, n, &h, collect, &seen));
        pkt[i] ^= 0x01;
    }
    CHECK_EQ_I(seen.n, 0);
    CHECK(!lbr_pkt_parse(pkt, n - 1, &h, collect, &seen));
    CHECK(lbr_pkt_parse(pkt, n, &h, collect, &seen));
}

static void test_packet_restamp(void)
{
    uint8_t           pkt[128];
    lbr_pkt_builder_t b;
    lbr_pkt_begin(&b, pkt, sizeof pkt, LBR_LINK_USB, 1, 0, 42);
    lbr_pkt_add(&b, LBR_REC_TEXT, (const uint8_t *)"x", 1);
    size_t n = lbr_pkt_finish(&b);
    /* A link stamps its own kind and per-link seq; the CRC stays valid. */
    lbr_pkt_restamp(pkt, n, LBR_LINK_WIFI, 0xABCDEF01u);
    lbr_pkt_header_t h;
    CHECK(lbr_pkt_parse(pkt, n, &h, NULL, NULL));
    CHECK_EQ_U(h.link, LBR_LINK_WIFI);
    CHECK_EQ_U(h.seq, 0xABCDEF01u);
    CHECK_EQ_U(h.t_us, 42u);
}

static lbr_sample_t sample(lbr_signal_id_t sig, uint64_t t)
{
    lbr_sample_t s = {.signal    = sig,
                      .source    = 5,
                      .type      = LBR_VAL_F32,
                      .valid     = true,
                      .t_mono_us = t,
                      .v.f32     = 1.0f};
    return s;
}

static void parse_lora(const uint8_t *buf, size_t n, seen_t *seen)
{
    lbr_pkt_header_t h;
    memset(seen, 0, sizeof *seen);
    CHECK(n <= LBR_PKT_LORA_MAX);
    CHECK(lbr_pkt_parse(buf, n, &h, collect, seen));
}

static void test_lora_priorities(void)
{
    lbr_lora_policy_t p;
    lbr_lora_policy_init(&p, lbr_lora_default_items, lbr_lora_default_item_count, 2000);
    uint8_t buf[LBR_PKT_LORA_MAX];
    seen_t  seen;

    CHECK_EQ_U(lbr_lora_policy_build(&p, 1000, 1, 0, buf, sizeof buf), 0u); /* nothing */

    for (int i = 0; i < 30; i++) {
        lbr_sample_t t = sample(LBR_SIG_TEMP_CVT, 1000 + (uint64_t)i);
        lbr_lora_policy_offer_sample(&p, &t);
    }
    lbr_sample_t f = sample(LBR_SIG_FAULT, 5);
    f.type         = LBR_VAL_STATUS;
    lbr_lora_policy_offer_sample(&p, &f);
    lbr_health_record_t h = {.node_id = 5, .state = 1};
    lbr_lora_policy_offer_health(&p, &h);

    size_t n = lbr_lora_policy_build(&p, 2000000, 1, 1, buf, sizeof buf);
    parse_lora(buf, n, &seen);
    /* Fault first, then health, then one (latest) CVT sample. */
    CHECK_EQ_I(seen.n, 3);
    CHECK_EQ_U(seen.types[0], LBR_REC_SAMPLE);
    CHECK_EQ_U(seen.types[1], LBR_REC_HEALTH_LITE);
    CHECK_EQ_U(seen.types[2], LBR_REC_SAMPLE);

    /* Nothing new: nothing sent. New CVT before its interval: held. */
    CHECK_EQ_U(lbr_lora_policy_build(&p, 2100000, 1, 2, buf, sizeof buf), 0u);
    lbr_sample_t t2 = sample(LBR_SIG_TEMP_CVT, 2100000);
    lbr_lora_policy_offer_sample(&p, &t2);
    CHECK_EQ_U(lbr_lora_policy_build(&p, 2500000, 1, 3, buf, sizeof buf), 0u);
    CHECK(lbr_lora_policy_build(&p, 3000001, 1, 4, buf, sizeof buf) > 0);

    /* Signals outside the subset never go out. */
    lbr_sample_t acc = sample(LBR_SIG_ACCEL_X, 1);
    lbr_lora_policy_offer_sample(&p, &acc);
    CHECK_EQ_U(lbr_lora_policy_build(&p, 9000000, 1, 5, buf, sizeof buf), 0u);
}

static void test_lora_event_queue_bounds(void)
{
    lbr_lora_policy_t p;
    lbr_lora_policy_init(&p, NULL, 0, 1000);
    lbr_sample_t e = sample(LBR_SIG_EVENT, 1);
    e.type         = LBR_VAL_STATUS;
    for (unsigned i = 0; i < LBR_LORA_EVT_QUEUE + 3; i++) {
        lbr_lora_policy_offer_sample(&p, &e);
    }
    CHECK_EQ_U(p.events_dropped, 3u);
    lbr_sample_t f = sample(LBR_SIG_FAULT, 2);
    f.type         = LBR_VAL_STATUS;
    lbr_lora_policy_offer_sample(&p, &f); /* evicts an event, still counted */
    CHECK_EQ_U(p.events_dropped, 4u);

    /* Drain across packets; the fault is still delivered. */
    uint8_t buf[LBR_PKT_LORA_MAX];
    int     faults = 0, total = 0;
    for (int k = 0; k < 4; k++) {
        size_t n = lbr_lora_policy_build(&p, 1, 1, (uint32_t)k, buf, sizeof buf);
        if (n == 0) {
            break;
        }
        seen_t seen;
        parse_lora(buf, n, &seen);
        total += seen.n;
        /* Byte 16+2 is the first record's signal id low byte: count faults. */
        for (size_t off = 16; off + 2 < n - 4; off += 2 + buf[off + 1]) {
            if (buf[off + 2] == (LBR_SIG_FAULT & 0xFF) && buf[off + 3] == (LBR_SIG_FAULT >> 8)) {
                faults++;
            }
        }
    }
    CHECK_EQ_I(total, (int)LBR_LORA_EVT_QUEUE);
    CHECK_EQ_I(faults, 1);
}

static void test_lora_round_robin_fills_packet(void)
{
    static const lbr_lora_item_t items[] = {
        {LBR_SIG_STRAIN_1, 0},   {LBR_SIG_STRAIN_2, 0},   {LBR_SIG_STRAIN_3, 0},
        {LBR_SIG_STRAIN_4, 0},   {LBR_SIG_STRAIN_5, 0},   {LBR_SIG_STRAIN_6, 0},
        {LBR_SIG_STRAIN_7, 0},   {LBR_SIG_STRAIN_8, 0},   {LBR_SIG_TEMP_AUX_1, 0},
        {LBR_SIG_TEMP_AUX_2, 0}, {LBR_SIG_TEMP_AUX_3, 0}, {LBR_SIG_TEMP_AUX_4, 0},
    };
    lbr_lora_policy_t p;
    lbr_lora_policy_init(&p, items, 12, 1000);
    uint8_t buf[LBR_PKT_LORA_MAX];
    int     sent[12] = {0};
    for (int round = 0; round < 6; round++) {
        for (size_t i = 0; i < 12; i++) {
            lbr_sample_t s = sample(items[i].signal, 1);
            lbr_lora_policy_offer_sample(&p, &s);
        }
        size_t n = lbr_lora_policy_build(&p, 1000 + (uint64_t)round, 1, 0, buf, sizeof buf);
        seen_t seen;
        parse_lora(buf, n, &seen);
        CHECK_EQ_I(seen.n, 10); /* 10 samples fit in one 255-byte packet */
        for (size_t i = 0; i < 12; i++) {
            sent[i] += p.slots[i].fresh ? 0 : 1;
        }
    }
    for (size_t i = 0; i < 12; i++) {
        CHECK(sent[i] >= 4); /* no signal starves */
    }
}

static void test_airtime(void)
{
    lbr_lora_modem_t m = {.sf = 7, .bw_hz = 125000, .cr = 1, .preamble = 8, .crc = true};
    CHECK_EQ_U(lbr_lora_airtime_us(&m, 10), 41216u);
    m.sf       = 12; /* low data rate optimization kicks in */
    uint32_t t = lbr_lora_airtime_us(&m, 255);
    CHECK(t > 9000000u && t < 9100000u);
    m.sf = 9;
    CHECK(lbr_lora_airtime_us(&m, 255) > lbr_lora_airtime_us(&m, 100));
    CHECK_EQ_U(lbr_lora_next_tx_us(1000, 100000, 100), 1000u + 900000u); /* 10 % duty */
    CHECK_EQ_U(lbr_lora_next_tx_us(1000, 100000, 1000), 1000u);
}

int main(void)
{
    RUN(test_packet_roundtrip_over_cobs);
    RUN(test_packet_rejects);
    RUN(test_packet_restamp);
    RUN(test_lora_priorities);
    RUN(test_lora_event_queue_bounds);
    RUN(test_lora_round_robin_fills_packet);
    RUN(test_airtime);
    TEST_MAIN_END();
}
