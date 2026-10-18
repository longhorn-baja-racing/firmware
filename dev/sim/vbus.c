/*
 * Virtual CAN bus: one classic-CAN segment shared by every simulated node.
 *
 * Models what the firmware relies on: bounded TX queues that drop (and
 * count) when full, bitwise arbitration by identifier, exact frame duration
 * including bit stuffing, the acceptance filter sensor nodes program, TX-done
 * timing for the gateway's time sync, error frames with TEC/REC counting,
 * bus-off with recovery, and unplugged cables.
 */
#include <string.h>

#include "lbr/nodes.h"
#include "sim.h"

#define BUS_OFF_RECOVERY_US 50000u /* 128 x 11 recessive bits plus the adapter's restart */
#define ERROR_FRAME_BITS    20u

/* ------------------------------------------------------------------------ */
/* Frame length                                                              */

static uint16_t crc15_bit(uint16_t crc, unsigned bit)
{
    unsigned top = ((crc >> 14) & 1u) ^ bit;
    crc          = (uint16_t)((crc << 1) & 0x7FFFu);
    return top ? (uint16_t)(crc ^ 0x4599u) : crc;
}

uint32_t vbus_frame_bits(const lbr_can_frame_t *f)
{
    uint8_t  bits[160];
    unsigned n   = 0;
    uint32_t id  = f->id & 0x1FFFFFFFu;
    uint8_t  dlc = f->dlc > 8 ? 8 : f->dlc;

    bits[n++] = 0; /* SOF */
    for (int i = 28; i >= 18; i--) {
        bits[n++] = (uint8_t)((id >> i) & 1u);
    }
    bits[n++] = 1; /* SRR */
    bits[n++] = 1; /* IDE */
    for (int i = 17; i >= 0; i--) {
        bits[n++] = (uint8_t)((id >> i) & 1u);
    }
    bits[n++] = 0; /* RTR */
    bits[n++] = 0; /* r1 */
    bits[n++] = 0; /* r0 */
    for (int i = 3; i >= 0; i--) {
        bits[n++] = (uint8_t)((dlc >> i) & 1u);
    }
    for (unsigned b = 0; b < dlc; b++) {
        for (int i = 7; i >= 0; i--) {
            bits[n++] = (uint8_t)((f->data[b] >> i) & 1u);
        }
    }
    uint16_t crc = 0;
    for (unsigned i = 0; i < n; i++) {
        crc = crc15_bit(crc, bits[i]);
    }
    for (int i = 14; i >= 0; i--) {
        bits[n++] = (uint8_t)((crc >> i) & 1u);
    }

    /* Stuffing from SOF to the end of the CRC: after five equal bits the
     * transmitter inserts the complement, which counts towards the next run. */
    uint32_t total = 0;
    unsigned run   = 0;
    uint8_t  last  = 2;
    for (unsigned i = 0; i < n; i++) {
        if (bits[i] == last) {
            run++;
        } else {
            last = bits[i];
            run  = 1;
        }
        total++;
        if (run == 5) {
            total++;
            last = (uint8_t)!last;
            run  = 1;
        }
    }
    return total + 13u; /* CRC delimiter, ACK, ACK delimiter, EOF, intermission */
}

/* ------------------------------------------------------------------------ */
/* Controllers                                                               */

void vbus_init(sim_t *s, uint32_t bitrate)
{
    memset(&s->bus, 0, sizeof s->bus);
    s->bus.bitrate = bitrate;
    s->bus.tx_node = -1;
}

static bool can_active(const sim_t *s, int n)
{
    const sim_can_t *c = &s->nodes[n].can;
    return s->nodes[n].powered && c->started && c->st.running;
}

static bool accepts(const sim_can_t *c, const lbr_can_frame_t *f)
{
    if (c->accept_all) {
        return true;
    }
    lbr_can_id_t id = lbr_can_id_parse(f->id);
    return id.kind == LBR_CAN_KIND_SYSTEM && id.source == LBR_NODE_GATEWAY;
}

bool vbus_send(sim_t *s, int node, const lbr_can_frame_t *f)
{
    sim_can_t *c = &s->nodes[node].can;
    if (!c->started || !c->st.running || c->tx_len == SIM_TXQ_LEN) {
        c->st.tx_dropped++;
        return false;
    }
    c->txq[(c->tx_head + c->tx_len) % SIM_TXQ_LEN] = *f;
    c->tx_len++;
    vbus_kick(s);
    return true;
}

void vbus_kick(sim_t *s)
{
    if (!s->bus.busy && !s->bus.arb_scheduled) {
        s->bus.arb_scheduled = true;
        sim_schedule(s, s->now, EV_BUS, -1, 0);
    }
}

void vbus_bus_off(sim_t *s, int node)
{
    sim_can_t *c = &s->nodes[node].can;
    if (!c->st.running) {
        return;
    }
    c->st.running = false;
    c->st.bus_off_count++;
    c->st.tx_dropped += (uint32_t)c->tx_len; /* the adapter flushes its queue */
    c->tx_len        = 0;
    c->bus_off_until = s->now + BUS_OFF_RECOVERY_US;
    sim_trace(s, node, "twai", "BUS-OFF (tec %u); recovery started", (unsigned)c->st.tec);
    sim_schedule(s, c->bus_off_until, EV_BUS_OFF_END, node, (int)s->nodes[node].gen);
}

void vbus_recover(sim_t *s, int node)
{
    sim_can_t *c = &s->nodes[node].can;
    if (c->st.running || !c->started) {
        return;
    }
    c->st.running    = true;
    c->st.tec        = 0;
    c->st.rec        = 0;
    c->bus_off_until = 0;
    sim_trace(s, node, "twai", "recovered from bus-off");
    vbus_kick(s);
}

static void roll_window(sim_t *s)
{
    sim_bus_t *b = &s->bus;
    while (s->now - b->window_start >= 1000000u) {
        b->busy_us_last   = b->busy_us_window;
        b->frames_last    = b->frames_window;
        b->busy_us_window = 0;
        b->frames_window  = 0;
        b->window_start += 1000000u;
    }
}

static void bump_tec(sim_t *s, int n, unsigned by)
{
    sim_can_t *c = &s->nodes[n].can;
    unsigned   t = (unsigned)c->st.tec + by;
    if (t > 255u) {
        c->st.tec = 255;
        vbus_bus_off(s, n);
        return;
    }
    c->st.tec = (uint8_t)t;
}

/* The frame on the wire finished (or was destroyed by an error). */
static void complete(sim_t *s)
{
    sim_bus_t *b  = &s->bus;
    int        tx = b->tx_node;
    b->busy       = false;
    b->tx_node    = -1;
    if (tx < 0) {
        return;
    }
    sim_can_t *c = &s->nodes[tx].can;
    if (!can_active(s, tx) || c->tx_len == 0) {
        return; /* went bus-off or lost power mid-frame */
    }
    lbr_can_frame_t f = c->txq[c->tx_head];

    sim_wire_t *w = &s->recent[s->recent_head++ % SIM_RECENT];
    w->t          = s->now;
    w->f          = f;
    w->error      = b->tx_error;

    if (b->tx_error) {
        b->errors_total++;
        c->st.bus_errors++;
        for (int n = 0; n < s->n_nodes; n++) {
            if (n != tx && can_active(s, n) && s->nodes[n].can.connected) {
                sim_can_t *r = &s->nodes[n].can;
                r->st.bus_errors++;
                r->st.rec = (uint8_t)(r->st.rec < 127 ? r->st.rec + 1 : 127);
            }
        }
        sim_trace(s, tx, "twai", "error frame on id 0x%08lx, retransmitting", (unsigned long)f.id);
        bump_tec(s, tx, 8); /* the frame stays queued and is retried */
        return;
    }

    c->tx_head = (c->tx_head + 1) % SIM_TXQ_LEN;
    c->tx_len--;
    c->st.tx_frames++;
    if (c->st.tec > 0) {
        c->st.tec--;
    }
    b->frames_total++;
    b->frames_window++;
    for (int n = 0; n < s->n_nodes; n++) {
        if (n == tx || !can_active(s, n) || !s->nodes[n].can.connected) {
            continue;
        }
        sim_can_t *r = &s->nodes[n].can;
        if (r->st.rec > 0) {
            r->st.rec--;
        }
        if (!accepts(r, &f)) {
            continue; /* hardware filter */
        }
        r->st.rx_frames++;
        sim_deliver(s, n, &f, s->now);
    }
    sim_tx_done(s, tx, &f, true, s->now);
}

/* Bus idle: the lowest identifier among the queue heads wins. */
static void arbitrate(sim_t *s)
{
    sim_bus_t *b    = &s->bus;
    int        win  = -1;
    uint32_t   best = 0;
    for (int n = 0; n < s->n_nodes; n++) {
        sim_can_t *c = &s->nodes[n].can;
        if (!can_active(s, n) || c->tx_len == 0) {
            continue;
        }
        if (!c->connected) {
            /* Nobody acknowledges: the controller keeps retrying and goes
             * error passive; its queue backs up and later sends drop. */
            if (c->st.tec < 128) {
                c->st.tec = 128;
                c->st.tx_failed++;
            }
            continue;
        }
        uint32_t id = c->txq[c->tx_head].id;
        if (win < 0 || id < best) {
            win  = n;
            best = id;
        }
    }
    if (win < 0) {
        return;
    }
    const lbr_can_frame_t *f    = &s->nodes[win].can.txq[s->nodes[win].can.tx_head];
    uint32_t               bits = vbus_frame_bits(f);
    b->tx_error = b->noise_permille > 0 && sim_rand(s) * 1000.0 < (double)b->noise_permille;
    if (b->tx_error) {
        bits = bits / 2u + ERROR_FRAME_BITS;
    }
    uint64_t dur  = ((uint64_t)bits * 1000000u + b->bitrate - 1u) / b->bitrate;
    b->busy       = true;
    b->tx_node    = win;
    b->busy_until = s->now + dur;
    b->busy_us_window += dur;
    sim_schedule(s, b->busy_until, EV_BUS, -1, 1);
}

void vbus_event(sim_t *s, int kind)
{
    roll_window(s);
    if (kind == 0) {
        /* Arbitration wake-up: a queue became non-empty while idle. */
        s->bus.arb_scheduled = false;
        if (!s->bus.busy) {
            arbitrate(s);
        }
        return;
    }
    /* End of the frame on the wire. */
    if (!s->bus.busy || s->now < s->bus.busy_until) {
        return;
    }
    complete(s);
    if (!s->bus.busy) {
        arbitrate(s);
    }
}
