#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "lbr/can.h"
#include "lbr/logic.h"
#include "lbr/node_table.h"

/*
 * Replay: re-run the car logic (lbr/logic.h) on a recorded log, AdvantageKit
 * style, to try a fix or add an output after the fact.
 *
 * Car logic sees the world through three things, and the Central's log holds
 * all of them in the order the logic saw them:
 *   - every CAN frame, byte for byte, with its receive time (CAN_FRAME);
 *     the Central logs a frame and feeds it to the logic under one lock, so
 *     log order is exactly the order the logic observed;
 *   - when each logic tick ran (TICK, kind LOGIC);
 *   - when the Central's heartbeat marked nodes lost or missing (TICK, kind
 *     HEARTBEAT), and when it started listening (TICK, kind START).
 * Replaying those records through the same decoder, node table, and logic
 * runner reproduces every output bit for bit; change a task and the replayed
 * outputs show what the change would have done on that run.
 */

/* TICK record (LBR_REC_TICK): t_us u64 (shared time), kind u8, can_up u8,
 * node_timeout_ms u16 (START only; 0 otherwise). */
#define LBR_TICK_REC_SIZE 12u

typedef enum : uint8_t {
    LBR_TICK_START     = 0, /* the Central started listening to the bus */
    LBR_TICK_LOGIC     = 1, /* a car logic tick ran */
    LBR_TICK_HEARTBEAT = 2, /* the Central's heartbeat checked for lost / missing nodes */
} lbr_tick_kind_t;

typedef struct {
    uint64_t        t_us;
    lbr_tick_kind_t kind;
    bool            can_up;          /* START */
    uint16_t        node_timeout_ms; /* START */
} lbr_tick_t;

size_t lbr_tick_encode(const lbr_tick_t *t, uint8_t *buf, size_t cap);
bool   lbr_tick_decode(const uint8_t *buf, size_t len, lbr_tick_t *out);

/* What the car logic sees, rebuilt from a log. */
typedef struct {
    lbr_bus_decoder_t decoder;
    lbr_node_table_t  nodes;
    lbr_logic_t       logic;
    uint64_t          listen_since_us;
    bool              can_up;
    bool              started; /* a START tick was seen */

    /* Counted, for the report. */
    uint32_t frames;
    uint32_t logic_ticks;
    uint32_t heartbeats;
    uint32_t bad_records; /* CAN_FRAME / TICK records that did not decode */
} lbr_replay_t;

/* Set up a replay of `tasks` (usually the car's, possibly changed) whose
 * outputs go to `sink`, exactly as on the Central. */
void lbr_replay_init(lbr_replay_t *r, const lbr_task *const *tasks, size_t n_tasks,
                     lbr_logic_sink_t sink);

/* Feed one log record, in log order. Records other than CAN_FRAME and TICK
 * are ignored (the caller copies or compares them). */
void lbr_replay_record(lbr_replay_t *r, uint8_t type, const uint8_t *payload, size_t len);

/* --- What the Central's core calls, so it and a replay do the same thing --- */

/* A frame the Central received (own_tx: one it sent) at shared time t_us:
 * decode it, update the node table, and feed the samples to `logic`. `out`
 * gets the samples (for the Central's other consumers); returns how many.
 * *found is set when the frame is a node's first (or first after it was lost
 * or missing), *back when that node had been lost or reported missing. */
size_t lbr_replay_observe_frame(lbr_bus_decoder_t *dec, lbr_node_table_t *nodes, lbr_logic_t *logic,
                                const lbr_can_frame_t *f, uint64_t t_us, bool own_tx,
                                lbr_sample_t *out, size_t cap, bool *found, bool *back);

/* The Central's heartbeat check at shared time now_us: nodes silent past the
 * timeout become lost (up to `cap` ids in lost[]), and, with CAN up, catalog
 * nodes never heard since listen_since_us become missing (missing[]). */
void lbr_replay_check_nodes(lbr_node_table_t *nodes, uint64_t now_us, bool can_up,
                            uint64_t listen_since_us, uint8_t *lost, size_t *n_lost,
                            uint8_t *missing, size_t *n_missing, size_t cap);
