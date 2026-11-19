#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "lbr/can.h"
#include "lbr/sensor.h"

/*
 * Shared sensor-node runtime. A node app only declares its configuration
 * (node id + which drivers run at which rates) with LBR_NODE_MAIN;
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

    /* Set when the app's setup went wrong (a slot bound twice or out of
     * range, ...). Validation then fails with this text, so the node runs in
     * CONFIG_INVALID and says why, instead of silently running without it. */
    const char *app_error;
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
/* How a slot is read, from the catalog (car/catalog.toml, slot "input"). The
 * board's role binds these by itself; LBR_INPUT_NONE slots are bound by the
 * app (lbr_bind) or stay NOT_FITTED. */
typedef enum : uint8_t {
    LBR_INPUT_NONE = 0,     /* a part driver bound in main.c, or not fitted */
    LBR_INPUT_IO,           /* on-chip ADC/pulse/switch inputs, spec in io_spec (menuconfig) */
    LBR_INPUT_GPS,          /* the NMEA GPS receiver (the Central's GPS UART settings) */
    LBR_INPUT_COCKPIT_LINK, /* the cockpit's battery and link state (the Central) */
} lbr_input_t;

typedef struct {
    const char            *name; /* slot name, e.g. "imu" */
    const lbr_signal_id_t *signals;
    size_t                 n_signals;
    uint16_t               rate_hz;
    lbr_input_t            input;
    const char            *io_spec; /* LBR_INPUT_IO: CONFIG_LBR_IO_<SLOT> ("" = not fitted) */
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
 * The node an app is setting up (see LBR_NODE_MAIN in lbr/node_app.h): its
 * plan, the configuration being built, and one sensor per slot.
 */
typedef struct {
    const lbr_node_plan_t *plan;
    lbr_node_config_t     *cfg;
    lbr_sensor_t          *sensors; /* plan->n_slots entries, slot order */
    uint32_t               bound;   /* bit i: the app bound slot i */
} lbr_node;

/*
 * The sensor of slot `slot`, for binding a part driver to it:
 *
 *     lbr_sensor_bmi270(lbr_slot(n, IMU_SLOT_CHASSIS), ...);
 *
 * The driver must produce exactly the slot's signals, in order (validation
 * checks). Binding a slot twice, or a slot the plan does not have, returns
 * NULL and marks the configuration invalid with the reason (cfg->app_error):
 * the node then reports CONFIG_INVALID on the bus rather than run without
 * it. Drivers given NULL must do nothing.
 */
lbr_sensor_t *lbr_slot(lbr_node *n, size_t slot);

/*
 * Run slot `slot` with part driver `drv` (lbr/driver.h) and its state `self`,
 * which must outlive the node (make it static):
 *
 *     static pressure_t brake_line = {.adc_pin = 34};
 *     lbr_bind(n, BRAKES_SLOT_LINE, &pressure_driver, &brake_line);
 *
 * A driver whose channel count differs from the slot's, a slot bound twice,
 * or one the plan lacks makes the configuration invalid with the reason.
 */
void lbr_bind(lbr_node *n, size_t slot, const struct lbr_driver *drv, void *self);

/*
 * Check a plan against the frame table. `can` is true for nodes that
 * transmit on CAN (their slot signals must travel in frames they send, at
 * the slot rate, all fields from that slot); false for the cockpit, which
 * is off the bus (each signal must travel in a frame of the table at the
 * slot rate, sent for it by the Central). Returns 0, or <0 with a reason in
 * `err`.
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
 * heartbeat (state CONFIG_INVALID) so the fault is visible on the bus. Apps
 * use LBR_NODE_MAIN (lbr/node_app.h), which calls this. */
[[noreturn]] void lbr_node_run(const lbr_node_config_t *cfg);

/* Publish an event or fault (LBR_SIG_EVENT / LBR_SIG_FAULT) from app code. */
void lbr_node_emit_event(lbr_event_code_t code, uint16_t arg, bool fault);
