#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "lbr/can.h"
#include "lbr/sensor.h"

/*
 * Shared sensor-node runtime. A node app only declares its configuration
 * (node id + which drivers run at which rates) and calls lbr_node_run();
 * acquisition, timestamping, CAN publishing, time sync, heartbeats, optional
 * local SD logging, and fault reporting are identical on every node.
 */

#define LBR_NODE_MAX_SENSORS     16u /* sensor_status / calibration bitmasks are 32-bit */
#define LBR_NODE_MAX_RATE_HZ     1000u
#define LBR_NODE_CONFIG_TEXT_MAX 4096u /* canonical config text, including driver params */

/* One driver scheduled at one rate. */
typedef struct {
    lbr_sensor_t *sensor;
    uint16_t      rate_hz; /* acquisition rate, 1..LBR_NODE_MAX_RATE_HZ, divides 1000000 */
} lbr_acq_t;

/* Called for every received CAN frame the runtime does not consume itself
 * (everything except time sync and commands). Runs in the CAN RX task. */
typedef void (*lbr_node_frame_cb)(const lbr_can_frame_t *f, uint64_t rx_shared_us, void *ctx);

/* Called once by lbr_node_run after the core started and before the
 * acquisition tasks run (e.g. the IMU Controller starts its corner link). */
typedef void (*lbr_node_started_cb)(void *ctx);

typedef struct {
    uint8_t          node_id; /* lbr_node_id_t: a CAN node other than the Central */
    const lbr_acq_t *acq;
    size_t           n_acq;

    uint16_t health_period_ms; /* heartbeat cadence */
    bool     local_log;        /* keep a local SD log (redundancy nodes only; none today) */

    /* Receive the whole bus instead of only Central time sync and commands,
     * and get frames through on_frame. */
    bool              listen_all;
    lbr_node_frame_cb on_frame;
    void             *on_frame_ctx;

    lbr_node_started_cb on_started;
    void               *on_started_ctx;
} lbr_node_config_t;

/* ------------------------------------------------------------------------ */
/* Node plan: the pure, host-checkable description of a node                */

/*
 * One acquisition slot: a group of channels read together by one driver at
 * one rate. Plans live in car/nodes/<name>/main/<name>_plan.c and are
 * compiled into the host tests, so every node's rates and CAN frames are
 * checked in CI before any hardware exists. The node app then binds a real
 * driver to each slot it has one for; unbound slots get the unconfigured
 * (or simulated) driver.
 *
 * Where a slot's samples go is the frame table (lbr/frames.h): every signal
 * of a CAN node's slot travels in a frame that this node sends at the slot's
 * rate, and every field of that frame comes from the same slot, so each
 * acquisition is published exactly once (lbr_plan_check).
 */
typedef struct {
    const char            *name; /* slot name, e.g. "imu" */
    const lbr_signal_id_t *signals;
    size_t                 n_signals;
    uint16_t               rate_hz;
} lbr_slot_t;

typedef struct {
    uint8_t           node_id;
    const lbr_slot_t *slots;
    size_t            n_slots;
    uint16_t          health_period_ms;
    bool              local_log;  /* this node keeps a local SD log */
    bool              listen_all; /* receives the whole bus */
} lbr_node_plan_t;

/*
 * Build a runnable configuration from a plan: slot i uses sensors[i], filled
 * by `unbound` (lbr_sensor_unconfigured, or lbr_sensor_sim on bench builds;
 * see lbr/node_app.h). `sensors` and `acq` must hold plan->n_slots entries
 * and outlive the node. Bind real drivers afterwards by overwriting
 * sensors[i].
 */
void lbr_node_build(const lbr_node_plan_t *plan, lbr_sensor_fill_t unbound, lbr_sensor_t *sensors,
                    lbr_acq_t *acq, lbr_node_config_t *cfg);

/*
 * Check a plan against the frame table. `can` is true for nodes that
 * transmit on CAN (their slot signals must travel in frames they send, at
 * the slot rate, all fields from that slot); false for wireless nodes (each
 * signal must be in the table with this node as origin, or be a field a
 * forwarder sends for it). Returns 0, or <0 with a reason in `err`.
 */
int lbr_plan_check(const lbr_node_plan_t *plan, bool can, char *err, size_t err_len);

/* ------------------------------------------------------------------------ */
/* Pure helpers (host-tested)                                                */

/* Worst-case CAN load of this node in bit/s: its data frames at their rates,
 * heartbeat, and health pages. */
uint32_t lbr_node_can_load_bps(const lbr_node_config_t *cfg);

/* Validate before starting a session: limits, catalog, and the frame-table
 * rules of lbr_plan_check. Returns 0 if valid; otherwise writes a one-line
 * reason to `err` and returns <0. */
int lbr_node_config_validate(const lbr_node_config_t *cfg, char *err, size_t err_len);

/* Canonical text form ("key=value" lines) persisted with every log and
 * hashed into config_crc. Returns length written (truncated if `cap` small). */
size_t lbr_node_config_text(const lbr_node_config_t *cfg, char *buf, size_t cap);

/* CRC-32 of lbr_node_config_text(); reported in every heartbeat. */
uint32_t lbr_node_config_crc(const lbr_node_config_t *cfg);

/*
 * The data frames node `node` sends for one slot's signals, in table order,
 * with chan[f][i] = the slot channel of field i of frames[f]. Returns the
 * number of frames (at most `cap`); a frame with a field outside the slot is
 * skipped (lbr_plan_check rejects that plan).
 */
size_t lbr_slot_frames(uint8_t node, const lbr_signal_id_t *signals, size_t n_signals,
                       const lbr_frame_def_t **frames, uint8_t (*chan)[LBR_FRAME_MAX_FIELDS],
                       size_t                  cap);

/* Turn one reading into a sample of grid index `k` (seq = k, time =
 * k * period_us). */
void lbr_reading_to_sample(uint8_t source, lbr_signal_id_t signal, const lbr_reading_t *r,
                           uint64_t k, uint32_t period_us, bool synced, lbr_sample_t *s);

/* ------------------------------------------------------------------------ */
/* Runtime (node_esp.c)                                                       */

/* Validate, start CAN + time sync + acquisition + heartbeat tasks. Does not
 * return. A configuration that fails validation still runs CAN and the
 * heartbeat (state CONFIG_INVALID) so the fault is visible on the bus. */
void lbr_node_run(const lbr_node_config_t *cfg);

/* Bind the part-independent input driver (lbr/sensor_io.h) to plan slot
 * `slot` when `spec` is non-empty; an empty spec leaves the slot as built.
 * Call between lbr_node_build() and lbr_node_run(). */
void lbr_node_bind_io(const lbr_node_plan_t *plan, size_t slot, lbr_sensor_t *sensors,
                      const char *spec);

/* Publish an event or fault (LBR_SIG_EVENT / LBR_SIG_FAULT) from app code. */
void lbr_node_emit_event(lbr_event_code_t code, uint16_t arg, bool fault);

/* The running node's core (for app code started from on_started). */
struct lbr_node_core;
struct lbr_node_core *lbr_node_running_core(void);
