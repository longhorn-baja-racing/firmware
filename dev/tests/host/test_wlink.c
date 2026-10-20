/*
 * The wireless links (corner nodes -> IMU Controller, Central <-> cockpit
 * dashboard) through their pure cores and fake platforms.
 */
#include <math.h>

#include "lbr/bytes.h"
#include "lbr/corner_core.h"
#include "lbr/corner_rx.h"
#include "lbr/dash_core.h"
#include "lbr/dash_link.h"
#include "lbr/nodes.h"
#include "lbr/packet.h"
#include "lbr/transport.h"
#include "lbr/wlink.h"
#include "lbr_test.h"

static void test_batch_codec(void)
{
    uint8_t         samples[2 * 12];
    uint8_t         buf[64];
    lbr_imu_batch_t b = {.k0          = 0xFFFFFFFEu,
                         .n           = 2,
                         .battery_raw = 87,
                         .flags       = 3,
                         .sample_size = 12,
                         .samples     = samples};
    for (size_t i = 0; i < sizeof samples; i++) {
        samples[i] = (uint8_t)i;
    }
    size_t n = lbr_imu_batch_encode(&b, buf, sizeof buf);
    CHECK_EQ_U(n, LBR_WLINK_BATCH_HDR + 24u);
    lbr_imu_batch_t o;
    CHECK(lbr_imu_batch_decode(buf, n, &o));
    CHECK_EQ_U(o.k0, 0xFFFFFFFEu);
    CHECK_EQ_U(o.n, 2);
    CHECK_EQ_U(o.battery_raw, 87);
    CHECK(memcmp(o.samples, samples, 24) == 0);
    CHECK(!lbr_imu_batch_decode(buf, n - 1, &o)); /* length must match n */
    buf[4] = LBR_WLINK_MAX_BATCH + 1;
    CHECK(!lbr_imu_batch_decode(buf, n, &o));
    CHECK_EQ_U(lbr_imu_batch_encode(&b, buf, 10), 0u);
}

static void test_origin_frames(void)
{
    const lbr_frame_def_t *f[4];
    size_t                 size = 0;
    CHECK_EQ_U(lbr_wlink_origin_frames(LBR_NODE_CORNER_BR, f, 4, &size), 2u);
    CHECK_EQ_U(f[0]->can_id, 0x306);
    CHECK_EQ_U(f[1]->can_id, 0x307);
    CHECK_EQ_U(size, 12u); /* two frames of 8 minus their counters */
    /* The IMU Controller's own chassis frames are not forwarded ones. */
    CHECK_EQ_U(lbr_wlink_origin_frames(LBR_NODE_IMU, f, 4, &size), 0u);

    lbr_can_frame_t in = {.id = 0x306, .dlc = 8, .data = {1, 2, 3, 4, 5, 6, 0xAA, 0xBB}};
    uint8_t         bytes[8];
    CHECK_EQ_U(lbr_wlink_strip_counter(f[0], &in, bytes), 6u);
    lbr_can_frame_t out;
    lbr_wlink_build_frame(f[0], bytes, 0x10203, &out);
    CHECK_EQ_U(out.id, 0x306);
    CHECK(memcmp(out.data, in.data, 6) == 0);
    CHECK_EQ_U(lbr_get_u16(out.data + 6), 0x0203);
}

static void test_clock_exchange(void)
{
    lbr_wclock_t c = {0};
    uint8_t      req[LBR_WLINK_TIME_REQ], resp[LBR_WLINK_TIME_RESP];
    uint64_t     shared;
    uint16_t     epoch;
    bool         srv = false;

    /* Local 1000 us; 3 ms each way; the server holds it 200 us. Server clock
     * is 50 s ahead. */
    lbr_wclock_request(&c, 1000, req);
    uint64_t t2 = 50000000 + 1000 + 3000;
    CHECK_EQ_U(lbr_wclock_reply(req, sizeof req, t2, t2 + 200, 0x77, true, resp), sizeof resp);
    CHECK(lbr_wclock_on_response(&c, resp, sizeof resp, 1000 + 6200, &shared, &epoch, &srv));
    CHECK_EQ_U(shared, 50000000u + 7200u); /* t3 + rtt / 2, rtt = 6 ms */
    CHECK_EQ_U(epoch, 0x77);
    CHECK_EQ_U(c.last_rtt_us, 6000u);
    CHECK(srv);

    /* A reply to nothing outstanding, or a duplicate, is ignored. */
    CHECK(!lbr_wclock_on_response(&c, resp, sizeof resp, 9000, &shared, &epoch, &srv));
    /* A slow round trip is not used. */
    lbr_wclock_request(&c, 100000, req);
    lbr_wclock_reply(req, sizeof req, 0, 0, 1, false, resp);
    CHECK(!lbr_wclock_on_response(&c, resp, sizeof resp, 100000 + LBR_WLINK_MAX_RTT_US + 1, &shared,
                                  &epoch, &srv));
    CHECK_EQ_U(c.rejected, 2u);

    /* Within the 20 ms limit but far slower than the best path seen (6 ms):
     * likely uneven, so not used; one near the best is. */
    lbr_wclock_request(&c, 200000, req);
    lbr_wclock_reply(req, sizeof req, 0, 0, 1, true, resp);
    CHECK(!lbr_wclock_on_response(&c, resp, sizeof resp, 200000 + 12000, &shared, &epoch, &srv));
    lbr_wclock_request(&c, 300000, req);
    lbr_wclock_reply(req, sizeof req, 0, 0, 1, true, resp);
    CHECK(lbr_wclock_on_response(&c, resp, sizeof resp, 300000 + 6500, &shared, &epoch, &srv));
    CHECK_EQ_U(lbr_wclock_reply(req, 3, 0, 0, 0, false, resp), 0u);
}

/* ------------------------------------------------------------------------ */
/* Corner node -> IMU Controller                                             */

#define MAX_PKTS   64
#define MAX_FRAMES 512

typedef struct {
    uint64_t         mono;
    lbr_time_servo_t servo;
    uint8_t          pkt[MAX_PKTS][LBR_PKT_MAX_SIZE];
    size_t           len[MAX_PKTS];
    size_t           n_pkts;
} corner_fake_t;

static uint64_t cf_mono(void *ctx)
{
    return ((corner_fake_t *)ctx)->mono;
}
static uint64_t cf_now(void *ctx)
{
    corner_fake_t *f = ctx;
    return lbr_time_servo_now(&f->servo, f->mono);
}
static void cf_servo(void *ctx, lbr_time_servo_t *out)
{
    *out = ((corner_fake_t *)ctx)->servo;
}
static lbr_time_update_t cf_apply(void *ctx, uint64_t shared, uint64_t local, uint16_t epoch)
{
    return lbr_time_servo_update(&((corner_fake_t *)ctx)->servo, shared, local, epoch);
}
static bool cf_send(void *ctx, const uint8_t *pkt, size_t len)
{
    corner_fake_t *f = ctx;
    if (f->n_pkts >= MAX_PKTS) {
        return false;
    }
    memcpy(f->pkt[f->n_pkts], pkt, len);
    f->len[f->n_pkts++] = len;
    return true;
}

static int imu_read(lbr_sensor_t *self, uint64_t t_us, lbr_reading_t *out)
{
    for (size_t i = 0; i < self->n_signals; i++) {
        out[i] =
            (lbr_reading_t){.valid = true, .v.f32 = (float)i + (float)(t_us % 1000000u) / 1e6f};
    }
    return 0;
}
static int ok_init(lbr_sensor_t *self)
{
    (void)self;
    return 0;
}
static int batt_read(lbr_sensor_t *self, uint64_t t_us, lbr_reading_t *out)
{
    (void)self;
    (void)t_us;
    out[0] = (lbr_reading_t){.valid = true, .v.f32 = 64.0f};
    return 0;
}

static const lbr_signal_id_t k_imu_fl[] = {
    LBR_SIG_IMU_FL_ACCEL_X, LBR_SIG_IMU_FL_ACCEL_Y, LBR_SIG_IMU_FL_ACCEL_Z,
    LBR_SIG_IMU_FL_GYRO_X,  LBR_SIG_IMU_FL_GYRO_Y,  LBR_SIG_IMU_FL_GYRO_Z,
};
static const lbr_signal_id_t k_batt_fl[] = {LBR_SIG_CORNER_FL_BATTERY};

typedef struct {
    lbr_can_frame_t frames[MAX_FRAMES];
    size_t          n_frames;
    uint64_t        now;
    int             found, lost;
    uint8_t         reply[64];
    size_t          reply_len;
} imu_fake_t;

static uint64_t if_now(void *ctx)
{
    return ((imu_fake_t *)ctx)->now;
}
static bool if_synced(void *ctx)
{
    (void)ctx;
    return true;
}
static uint16_t if_epoch(void *ctx)
{
    (void)ctx;
    return 0x4242;
}
static bool if_send(void *ctx, const lbr_can_frame_t *f)
{
    imu_fake_t *m = ctx;
    if (m->n_frames >= MAX_FRAMES) {
        return false;
    }
    m->frames[m->n_frames++] = *f;
    return true;
}
static bool if_reply(void *ctx, uint8_t node, const uint8_t *pkt, size_t len)
{
    imu_fake_t *m = ctx;
    CHECK_EQ_U(node, LBR_NODE_CORNER_FL);
    memcpy(m->reply, pkt, len < sizeof m->reply ? len : sizeof m->reply);
    m->reply_len = len;
    return true;
}
static void if_event(void *ctx, lbr_event_code_t code, uint16_t arg, bool fault)
{
    imu_fake_t *m = ctx;
    (void)fault;
    CHECK_EQ_U(arg, LBR_NODE_CORNER_FL);
    if (code == LBR_EVT_NODE_FOUND) {
        m->found++;
    } else if (code == LBR_EVT_NODE_LOST) {
        m->lost++;
    }
}

typedef struct {
    corner_fake_t     cf;
    lbr_sensor_t      sensors[2];
    lbr_acq_t         acq[2];
    lbr_node_config_t cfg;
    lbr_corner_core_t corner;
    imu_fake_t        imu;
    lbr_corner_rx_t   rx;
} corner_rig_t;

static void corner_rig_init(corner_rig_t *r)
{
    memset(r, 0, sizeof *r);
    lbr_time_servo_init(&r->cf.servo);
    r->cf.mono    = 1000000;
    r->sensors[0] = (lbr_sensor_t){
        .name = "imu", .signals = k_imu_fl, .n_signals = 6, .init = ok_init, .read = imu_read};
    r->sensors[1] = (lbr_sensor_t){.name      = "battery",
                                   .signals   = k_batt_fl,
                                   .n_signals = 1,
                                   .init      = ok_init,
                                   .read      = batt_read};
    r->acq[0]     = (lbr_acq_t){&r->sensors[0], 100};
    r->acq[1]     = (lbr_acq_t){&r->sensors[1], 1};
    r->cfg        = (lbr_node_config_t){
        .node_id = LBR_NODE_CORNER_FL, .acq = r->acq, .n_acq = 2, .health_period_ms = 1000};
    lbr_corner_hal_t hal = {.ctx        = &r->cf,
                            .mono_us    = cf_mono,
                            .now_us     = cf_now,
                            .time_servo = cf_servo,
                            .time_apply = cf_apply,
                            .send       = cf_send};
    CHECK(lbr_corner_core_init(&r->corner, &r->cfg, &hal, 5));
    lbr_corner_core_slot_start(&r->corner, 0);
    lbr_corner_core_slot_start(&r->corner, 1);

    lbr_corner_rx_hal_t rh = {.ctx        = &r->imu,
                              .now_us     = if_now,
                              .synced     = if_synced,
                              .epoch      = if_epoch,
                              .send_frame = if_send,
                              .reply      = if_reply,
                              .event      = if_event};
    lbr_corner_rx_init(&r->rx, &rh);
}

/* Advance to the IMU slot's next grid point and sample. */
static void corner_tick(corner_rig_t *r)
{
    uint64_t due = lbr_corner_core_slot_next_due(&r->corner, 0);
    r->cf.mono += due - cf_now(&r->cf);
    lbr_corner_core_slot_tick(&r->corner, 0);
}

static void deliver(corner_rig_t *r, size_t i)
{
    r->imu.now = r->cf.mono;
    lbr_corner_rx_on_packet(&r->rx, r->cf.pkt[i], r->cf.len[i], r->imu.now);
}

static void test_corner_forwarding(void)
{
    static corner_rig_t r;
    corner_rig_init(&r);
    lbr_corner_core_slot_tick(&r.corner, 1); /* battery */
    for (int i = 0; i < 10; i++) {
        corner_tick(&r);
    }
    CHECK_EQ_U(r.cf.n_pkts, 2u); /* batches of 5 */
    deliver(&r, 0);
    deliver(&r, 1);
    CHECK_EQ_I(r.imu.found, 1);
    /* Each sample is forwarded once as two frames, whatever the redundancy. */
    CHECK_EQ_U(r.imu.n_frames, 20u);
    CHECK_EQ_U(r.rx.peer[0].forwarded, 10u);
    CHECK_EQ_U(r.rx.peer[0].duplicates, 5u);
    CHECK_EQ_U(r.imu.frames[0].id, 0x300);
    CHECK_EQ_U(r.imu.frames[1].id, 0x301);
    uint16_t k0 = lbr_get_u16(r.imu.frames[0].data + 6);
    for (size_t i = 0; i < 10; i++) {
        CHECK_EQ_U(lbr_get_u16(r.imu.frames[2 * i].data + 6), (uint16_t)(k0 + i));
    }
    /* The forwarded bytes decode to what the corner measured. */
    lbr_sample_t s[3];
    uint64_t     k = r.cf.mono / 10000u - 9u;
    CHECK_EQ_U(lbr_can_decode_data(&r.imu.frames[1], k * 10000u + 2000, true, s, 3), 3u);
    CHECK_EQ_U(s[0].source, LBR_NODE_CORNER_FL);
    CHECK_EQ_U(s[0].t_mono_us, k * 10000u);
    CHECK(s[2].valid &&
          fabsf(s[2].v.f32 - 5.0f - (float)((k * 10000u) % 1000000u) / 1e6f) < 0.002f);

    /* Status: link up, battery 64 %, not synced yet. */
    CHECK_EQ_U(lbr_corner_rx_link_mask(&r.rx), 0x01u);
    CHECK_EQ_U(lbr_corner_rx_synced_mask(&r.rx), 0x00u);
    lbr_sensor_t                 st;
    lbr_reading_t                rd[6];
    static const lbr_signal_id_t status[] = {
        LBR_SIG_CORNER_FL_BATTERY, LBR_SIG_CORNER_FR_BATTERY, LBR_SIG_CORNER_BL_BATTERY,
        LBR_SIG_CORNER_BR_BATTERY, LBR_SIG_CORNER_LINK,       LBR_SIG_CORNER_SYNCED,
    };
    lbr_sensor_corner_status(&st, "corner_status", status, 6, &r.rx);
    CHECK_EQ_I(st.read(&st, 0, rd), 0);
    CHECK(rd[0].valid && rd[0].v.f32 == 64.0f);
    CHECK(!rd[1].valid && rd[1].reason == LBR_UNAVAIL_NO_DATA); /* FR never heard */
    CHECK(rd[4].valid && rd[4].v.u32 == 1u);

    uint32_t loss  = 0;
    uint8_t  flags = lbr_corner_rx_tick(&r.rx, &loss);
    CHECK((flags & LBR_HB_ERR_LINK) != 0); /* three corners are down */
    CHECK_EQ_U(loss, 0u);
}

static void test_corner_loss_and_recovery(void)
{
    static corner_rig_t r;
    corner_rig_init(&r);
    for (int i = 0; i < 30; i++) {
        corner_tick(&r);
    }
    CHECK_EQ_U(r.cf.n_pkts, 6u);
    /* Datagram 1 lost: datagram 2 carries its batch again, nothing missing. */
    deliver(&r, 0);
    deliver(&r, 2);
    CHECK_EQ_U(r.rx.peer[0].forwarded, 15u);
    CHECK_EQ_U(r.rx.peer[0].packet_gaps, 0u); /* not yet: it could still arrive late */
    /* Datagrams 3 and 4 lost: batch 3 is gone for good. */
    deliver(&r, 5);
    CHECK_EQ_U(r.rx.peer[0].forwarded, 25u);
    uint32_t loss = 0;
    lbr_corner_rx_tick(&r.rx, &loss);
    CHECK_EQ_U(loss, 0u); /* still inside the window: it could arrive late */
    /* Counted once the samples (and the datagrams) leave their windows. */
    for (int i = 0; i < 170; i++) {
        corner_tick(&r);
    }
    for (size_t i = 6; i < r.cf.n_pkts; i++) {
        deliver(&r, i);
    }
    lbr_corner_rx_tick(&r.rx, &loss);
    CHECK_EQ_U(r.rx.peer[0].lost, 5u);
    CHECK_EQ_U(r.rx.peer[0].packet_gaps, 3u); /* datagrams 1, 3, 4, once out of the window */
    CHECK_EQ_U(loss, 5u);
    lbr_corner_rx_tick(&r.rx, &loss);
    CHECK_EQ_U(loss, 5u); /* reported once */

    /* Silence: the link drops (NODE_LOST), then comes back (NODE_FOUND). */
    r.imu.now += LBR_WLINK_LINK_TIMEOUT_US + 1;
    lbr_corner_rx_tick(&r.rx, &loss);
    CHECK_EQ_I(r.imu.lost, 1);
    CHECK_EQ_U(lbr_corner_rx_link_mask(&r.rx), 0u);
    for (int i = 0; i < 5; i++) {
        corner_tick(&r);
    }
    deliver(&r, r.cf.n_pkts - 1);
    CHECK_EQ_I(r.imu.found, 2);
}

/* A raw corner datagram: one IMU_BATCH of zeroed samples k0 .. k0 + n - 1. */
static size_t raw_batch(uint8_t *buf, size_t cap, uint32_t seq, uint32_t k0, uint8_t n, uint8_t tl)
{
    const lbr_frame_def_t *frames[4];
    size_t                 ss = 0;
    lbr_wlink_origin_frames(LBR_NODE_CORNER_FL, frames, 4, &ss);
    uint8_t         samples[LBR_WLINK_MAX_BATCH * LBR_WLINK_MAX_SAMPLE] = {0};
    lbr_imu_batch_t b = {.k0    = k0,
                         .n     = n,
                         .flags = (uint8_t)(LBR_WLINK_F_SENSOR_OK | (tl << LBR_WLINK_F_TL_SHIFT)),
                         .sample_size = (uint8_t)ss,
                         .samples     = samples};
    uint8_t         rec[LBR_WLINK_BATCH_HDR + sizeof samples];
    size_t          rn = lbr_imu_batch_encode(&b, rec, sizeof rec);
    CHECK(rn > 0);
    lbr_pkt_builder_t pb;
    lbr_pkt_begin(&pb, buf, cap, LBR_LINK_CORNER, LBR_NODE_CORNER_FL, seq, 0);
    lbr_pkt_add(&pb, LBR_REC_IMU_BATCH, rec, rn);
    return lbr_pkt_finish(&pb);
}

static void raw_deliver(corner_rig_t *r, uint32_t seq, uint32_t k0, uint8_t n, uint8_t tl,
                        uint64_t at)
{
    uint8_t pkt[256];
    size_t  len = raw_batch(pkt, sizeof pkt, seq, k0, n, tl);
    r->imu.now  = at;
    lbr_corner_rx_on_packet(&r->rx, pkt, len, at);
}

static void test_corner_timelines(void)
{
    static corner_rig_t r;
    corner_rig_init(&r);
    lbr_corner_peer_t *p = &r.rx.peer[0];

    raw_deliver(&r, 0, 1000, 5, 1, 1000000);
    raw_deliver(&r, 2, 1007, 3, 1, 1010000); /* 1005, 1006 still missing */
    CHECK_EQ_U(p->forwarded, 8u);
    CHECK_EQ_U(p->lost, 0u);

    /* The corner's clock stepped back: a new timeline, k far behind. Taken
     * and forwarded (not late), and the old window's holes counted. */
    raw_deliver(&r, 3, 200, 5, 2, 1020000);
    CHECK_EQ_U(p->forwarded, 13u);
    CHECK_EQ_U(p->late, 0u);
    CHECK_EQ_U(p->lost, 2u);

    /* The missing datagram, sent before the step, arrives reordered: a
     * straggler of the old timeline, not another timeline change. */
    raw_deliver(&r, 1, 1005, 2, 1, 1030000);
    CHECK_EQ_U(p->stale, 2u);
    CHECK_EQ_U(p->forwarded, 13u);
    raw_deliver(&r, 4, 205, 5, 2, 1040000);
    CHECK_EQ_U(p->forwarded, 18u);
    CHECK_EQ_U(p->lost, 2u);
    CHECK_EQ_U(p->packet_gaps, 0u);

    /* A reboot (the link drops while it boots and rejoins WiFi): the
     * timeline and the seq start again from 0. */
    r.imu.now = 1040000 + LBR_WLINK_LINK_TIMEOUT_US + 1;
    lbr_corner_rx_tick(&r.rx, NULL);
    raw_deliver(&r, 0, 3, 5, 0, 2000000);
    CHECK_EQ_U(p->forwarded, 23u);
    CHECK_EQ_U(p->lost, 2u);
    CHECK_EQ_U(p->late + p->stale, 2u);
}

static void test_seq_window(void)
{
    lbr_seq_window_t w    = {0};
    uint32_t         gaps = 0;
    CHECK_EQ_I(lbr_seq_window_take(&w, 10, &gaps), LBR_SEQ_NEW);
    CHECK_EQ_I(lbr_seq_window_take(&w, 13, &gaps), LBR_SEQ_NEW);
    CHECK_EQ_U(gaps, 0u); /* 11 and 12 may still come */
    CHECK_EQ_I(lbr_seq_window_take(&w, 11, &gaps), LBR_SEQ_LATE);
    CHECK_EQ_I(lbr_seq_window_take(&w, 11, &gaps), LBR_SEQ_DUPLICATE);
    CHECK_EQ_I(lbr_seq_window_take(&w, 13, &gaps), LBR_SEQ_DUPLICATE);
    CHECK_EQ_I(lbr_seq_window_take(&w, 13 + 32, &gaps), LBR_SEQ_NEW);
    CHECK_EQ_U(gaps, 1u); /* 12 left the window unseen; 14 .. 44 still may come */
    CHECK_EQ_I(lbr_seq_window_take(&w, 12, &gaps), LBR_SEQ_OLD);
    CHECK_EQ_I(lbr_seq_window_take(&w, 13 + 64, &gaps), LBR_SEQ_NEW);
    CHECK_EQ_U(gaps, 1u + 31u); /* now 14 .. 44 are gone too */
    /* Across the 32-bit wrap. */
    lbr_seq_window_t v = {0};
    gaps               = 0;
    CHECK_EQ_I(lbr_seq_window_take(&v, 0xFFFFFFFFu, &gaps), LBR_SEQ_NEW);
    CHECK_EQ_I(lbr_seq_window_take(&v, 0, &gaps), LBR_SEQ_NEW);
    CHECK_EQ_U(gaps, 0u);
}

static void test_corner_clock(void)
{
    static corner_rig_t r;
    corner_rig_init(&r);
    CHECK_EQ_U(lbr_corner_core_clock_tick(&r.corner), LBR_CORNER_SYNC_FAST_MS);
    CHECK_EQ_U(r.cf.n_pkts, 1u);
    r.imu.now = 777000000; /* the IMU Controller's shared clock */
    lbr_corner_rx_on_packet(&r.rx, r.cf.pkt[0], r.cf.len[0], r.imu.now);
    CHECK(r.imu.reply_len > 0);
    r.cf.mono += 2000; /* 2 ms round trip */
    lbr_corner_core_on_packet(&r.corner, r.imu.reply, r.imu.reply_len, r.cf.mono);
    CHECK(r.cf.servo.synced);
    CHECK_EQ_U(r.cf.servo.epoch, 0x4242);
    CHECK((lbr_corner_core_flags(&r.corner) & LBR_WLINK_F_SYNCED) != 0);
    CHECK_EQ_U(lbr_corner_core_clock_tick(&r.corner), LBR_CORNER_SYNC_PERIOD_MS);
    uint64_t now = cf_now(&r.cf);
    CHECK(now >= 777000000u && now < 777000000u + 3000u);

    /* Garbage and other nodes' datagrams are rejected and counted. */
    uint8_t junk[20] = {1, 2, 3};
    lbr_corner_core_on_packet(&r.corner, junk, sizeof junk, r.cf.mono);
    CHECK_EQ_U(r.corner.rejected, 1u);
    lbr_corner_rx_on_packet(&r.rx, junk, sizeof junk, 0);
    CHECK_EQ_U(r.rx.rejected, 1u);
}

/* ------------------------------------------------------------------------ */
/* Central <-> dashboard                                                     */

typedef struct {
    uint64_t mono;
    uint8_t  pkt[256];
    size_t   len;
} dash_fake_t;

static uint64_t df_mono(void *ctx)
{
    return ((dash_fake_t *)ctx)->mono;
}
static bool df_send(void *ctx, const uint8_t *pkt, size_t len)
{
    dash_fake_t *f = ctx;
    memcpy(f->pkt, pkt, len < sizeof f->pkt ? len : sizeof f->pkt);
    f->len = len;
    return true;
}

static const lbr_signal_id_t k_dash_batt[] = {LBR_SIG_DASH_BATTERY};

static int dash_batt_read(lbr_sensor_t *self, uint64_t t_us, lbr_reading_t *out)
{
    (void)self;
    (void)t_us;
    out[0] = (lbr_reading_t){.valid = true, .v.f32 = 91.0f};
    return 0;
}

static void test_dashboard_round_trip(void)
{
    /* Central side: values from the bus and one node's health. */
    lbr_value_table_t values;
    lbr_node_table_t  nodes;
    lbr_dash_link_t   link;
    lbr_value_table_init(&values, 2000000);
    for (size_t i = 0; i < lbr_dash_signal_count; i++) {
        lbr_value_table_watch(&values, lbr_dash_signals[i], 0);
    }
    lbr_node_table_init(&nodes, 3000000);
    lbr_dash_link_init(&link);
    lbr_sample_t rpm = {.signal = LBR_SIG_ENGINE_RPM,
                        .source = LBR_NODE_POWERTRAIN,
                        .type   = LBR_VAL_F32,
                        .valid  = true,
                        .v.f32  = 3600.0f};
    lbr_value_table_offer(&values, &rpm, 1000);
    lbr_sample_t fuel = {.signal = LBR_SIG_FUEL_LEVEL, .type = LBR_VAL_F32, .valid = true};
    lbr_value_table_offer(&values, &fuel, 1000);
    lbr_can_frame_t hb;
    lbr_can_encode_heartbeat(LBR_NODE_STRESS, LBR_NODE_DEGRADED, LBR_HB_ERR_SENSOR, &hb);
    lbr_node_table_on_frame(&nodes, &hb, 1000);

    uint8_t pkt[LBR_PKT_MAX_SIZE];
    size_t  n =
        lbr_dash_link_build(&link, 1500000, LBR_NODE_CENTRAL, &values, &nodes, pkt, sizeof pkt);
    CHECK(n > 0);

    /* Dashboard side. */
    dash_fake_t            df   = {.mono = 50};
    lbr_sensor_t           batt = {.name      = "battery",
                                   .signals   = k_dash_batt,
                                   .n_signals = 1,
                                   .init      = ok_init,
                                   .read      = dash_batt_read};
    lbr_acq_t              acq  = {&batt, 1};
    lbr_node_config_t      cfg  = {.node_id = LBR_NODE_DASHBOARD, .acq = &acq, .n_acq = 1};
    lbr_dash_hal_t         hal  = {.ctx = &df, .mono_us = df_mono, .send = df_send};
    static lbr_dash_core_t d;
    lbr_dash_core_init(&d, &cfg, &hal);
    lbr_dash_core_on_packet(&d, pkt, n, df.mono);
    CHECK_EQ_U(d.packets, 1u);
    CHECK(lbr_dash_core_link_up(&d, df.mono + 1000));
    CHECK(!lbr_dash_core_link_up(&d, df.mono + LBR_WLINK_LINK_TIMEOUT_US + 1));

    lbr_value_table_t shown;
    lbr_dash_node_t   shown_nodes[LBR_DASH_MAX_NODES];
    lbr_dash_core_snapshot(&d, &shown, shown_nodes);
    lbr_sample_t s;
    CHECK_EQ_I(lbr_value_table_get(&shown, LBR_SIG_ENGINE_RPM, df.mono, &s), LBR_VALUE_OK);
    CHECK(s.v.f32 == 3600.0f);
    /* Stale at the Central: shown as NO_DATA, never as the old number. */
    CHECK_EQ_I(lbr_value_table_get(&shown, LBR_SIG_FUEL_LEVEL, df.mono, &s), LBR_VALUE_OK);
    n = lbr_dash_link_build(&link, 1000 + 2000001, LBR_NODE_CENTRAL, &values, &nodes, pkt,
                            sizeof pkt);
    lbr_dash_core_on_packet(&d, pkt, n, df.mono + 10);
    lbr_dash_core_snapshot(&d, &shown, shown_nodes);
    CHECK_EQ_I(lbr_value_table_get(&shown, LBR_SIG_FUEL_LEVEL, df.mono + 10, &s),
               LBR_VALUE_INVALID);
    CHECK_EQ_U(s.v.u32, LBR_UNAVAIL_NO_DATA);
    CHECK_EQ_U(shown_nodes[0].node_id, LBR_NODE_STRESS);
    CHECK_EQ_U(shown_nodes[0].state, LBR_NODE_DEGRADED);

    /* The dashboard reports its battery; the Central takes it. */
    lbr_dash_core_slot_start(&d, 0);
    df.mono = 1000000;
    lbr_dash_core_slot_tick(&d, 0);
    CHECK(df.len > 0);
    lbr_dash_link_on_packet(&link, df.pkt, df.len, 5000000);
    CHECK(link.have_battery && link.battery.v.f32 == 91.0f);
    CHECK_EQ_I(lbr_dash_link_tick(&link, 5000001), 1);
    CHECK_EQ_I(lbr_dash_link_tick(&link, 5000002), 0);
    CHECK_EQ_I(lbr_dash_link_tick(&link, 5000000 + LBR_WLINK_LINK_TIMEOUT_US + 1), 0);
    CHECK_EQ_I(lbr_dash_link_tick(&link, 5000000 + LBR_WLINK_DASH_TIMEOUT_US + 1), -1);

    /* A datagram that is not from the Central is rejected. */
    lbr_dash_core_on_packet(&d, df.pkt, df.len, 0);
    CHECK_EQ_U(d.rejected, 1u);

    /* Reordered: an older datagram arriving after a newer one is ignored,
     * so the display never steps back to the older value. */
    uint8_t old_pkt[LBR_PKT_MAX_SIZE];
    rpm.v.f32 = 3000.0f;
    lbr_value_table_offer(&values, &rpm, 3000000);
    size_t old_n = lbr_dash_link_build(&link, 3000000, LBR_NODE_CENTRAL, &values, &nodes, old_pkt,
                                       sizeof old_pkt);
    rpm.v.f32    = 4000.0f;
    lbr_value_table_offer(&values, &rpm, 3100000);
    n = lbr_dash_link_build(&link, 3100000, LBR_NODE_CENTRAL, &values, &nodes, pkt, sizeof pkt);
    uint32_t packets = d.packets;
    lbr_dash_core_on_packet(&d, pkt, n, df.mono + 100);
    lbr_dash_core_on_packet(&d, old_pkt, old_n, df.mono + 200);
    CHECK_EQ_U(d.packets, packets + 1u);
    CHECK_EQ_U(d.packet_gaps, 0u);
    lbr_dash_core_snapshot(&d, &shown, shown_nodes);
    CHECK_EQ_I(lbr_value_table_get(&shown, LBR_SIG_ENGINE_RPM, df.mono + 200, &s), LBR_VALUE_OK);
    CHECK(s.v.f32 == 4000.0f);
}

int main(void)
{
    RUN(test_batch_codec);
    RUN(test_origin_frames);
    RUN(test_clock_exchange);
    RUN(test_corner_forwarding);
    RUN(test_corner_loss_and_recovery);
    RUN(test_corner_timelines);
    RUN(test_seq_window);
    RUN(test_corner_clock);
    RUN(test_dashboard_round_trip);
    TEST_MAIN_END();
}
