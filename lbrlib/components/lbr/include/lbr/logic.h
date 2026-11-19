#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "lbr/node_table.h"
#include "lbr/protocol.h"
#include "lbr/value.h"

/*
 * Car logic: periodic tasks on the Central that turn the bus into answers.
 *
 * A task reads any catalog signal, with its status and age, and writes
 * named outputs that go into the authoritative log next to the raw data:
 *
 *     LBR_OUTPUT(cvt_ratio, "CVT/Ratio", F32, NONE);
 *
 *     static void cvt_periodic(lbr_cycle *c)
 *     {
 *         lbr_out(c, &cvt_ratio, lbr_div(lbr_get(c, ENGINE_RPM), lbr_get(c, CVT_RPM)));
 *     }
 *
 *     LBR_TASK(cvt, .rate_hz = 50, .periodic = cvt_periodic);
 *
 * and the task goes in the car's task list (car/logic/tasks.c). Values read
 * through lbr_get() are never silently wrong: a value from a node that
 * dropped off the bus reads NODE_LOST, one from a node never heard reads
 * NODE_MISSING, one that stopped updating reads STALE, and the reasons
 * sensor nodes report (NOT_FITTED, SENSOR_FAULT, ...) come through as they
 * are. Outputs keep that status, so the log says why a derived value is
 * missing.
 *
 * Tasks run one at a time, in list order, on the Central's logic tick
 * (LBR_LOGIC_TICK_HZ), each at its own rate on the shared clock. They must
 * be quick (no blocking, no I/O): everything a task needs comes in through
 * lbr_get() and everything it produces goes out through lbr_out(). That is
 * also what makes them replayable from a log. Keep state in static variables
 * of the task's file.
 */

#define LBR_LOGIC_TICK_HZ     100u /* the fastest a task can run */
#define LBR_LOGIC_MAX_TASKS   16u
#define LBR_LOGIC_MAX_OUTPUTS 64u
#define LBR_OUTPUT_KEY_MAX    48u /* key length, ASCII, "Group/Name" */

typedef struct lbr_cycle lbr_cycle;

typedef struct {
    const char *name;
    uint16_t    rate_hz;            /* 1..LBR_LOGIC_TICK_HZ, divides LBR_LOGIC_TICK_HZ */
    void (*init)(lbr_cycle *c);     /* optional: once, before the first periodic */
    void (*periodic)(lbr_cycle *c); /* every 1 / rate_hz seconds */
} lbr_task;

/* Define task `name_`: LBR_TASK(cvt, .rate_hz = 50, .periodic = cvt_periodic). */
#define LBR_TASK(name_, ...) const lbr_task name_ = {.name = #name_, __VA_ARGS__}

typedef struct {
    const char      *key;  /* "Group/Name": how it shows in the log and the dashboard */
    lbr_value_type_t type; /* LBR_VAL_F32 or LBR_VAL_BOOL */
    lbr_unit_t       unit;

    /* Kept by the runner: */
    uint16_t id;      /* assigned when first written; 0 until then */
    bool     written; /* written at least once */
    lbr_f32  last;    /* the last value written (bool outputs: 0 / 1) */
    uint64_t last_us; /* when */
} lbr_output;

/* Define output `var_`: LBR_OUTPUT(cvt_ratio, "CVT/Ratio", F32, NONE). TYPE is
 * F32 or BOOL; UNIT a catalog unit (NONE, RPM, DEG_C, PERCENT, ...). */
#define LBR_OUTPUT(var_, key_, TYPE, UNIT)                                                         \
    static lbr_output var_ = {.key = (key_), .type = LBR_VAL_##TYPE, .unit = LBR_UNIT_##UNIT}

/* ------------------------------------------------------------------------ */
/* Inside a task                                                             */

/* The latest value of catalog signal NAME (as in car/catalog.toml), with its
 * status and age: lbr_get(c, ENGINE_RPM). Integer, status, and bool signals
 * read as their numeric value. */
#define lbr_get(c, NAME)      lbr_signal((c), LBR_SIG_##NAME)
#define lbr_get_bool(c, NAME) lbr_signal_bool((c), LBR_SIG_##NAME)
#define lbr_get_i32(c, NAME)  lbr_signal_i32((c), LBR_SIG_##NAME)

lbr_f32  lbr_signal(lbr_cycle *c, lbr_signal_id_t sig);
lbr_bool lbr_signal_bool(lbr_cycle *c, lbr_signal_id_t sig);
lbr_i32  lbr_signal_i32(lbr_cycle *c, lbr_signal_id_t sig);

/* Write an output (logged with its status; an unavailable value is logged as
 * unavailable, with its reason). */
void lbr_out(lbr_cycle *c, lbr_output *o, lbr_f32 v);
void lbr_out_bool(lbr_cycle *c, lbr_output *o, lbr_bool v);

/* Shared time of this run, and the task's period in seconds. */
uint64_t lbr_now_us(const lbr_cycle *c);
float    lbr_dt_s(const lbr_cycle *c);

/* ------------------------------------------------------------------------ */
/* The runner (the Central's gateway core drives it; apps do not call this)  */

typedef struct {
    void *ctx;
    /* Append one record (LBR_REC_KEY / LBR_REC_VALUE) to the log. */
    void (*record)(void *ctx, uint8_t type, const uint8_t *payload, size_t len);
} lbr_logic_sink_t;

typedef struct {
    lbr_sample_t last;
    uint64_t     rx_us;
    bool         have;
} lbr_logic_value_t;

typedef struct {
    lbr_logic_value_t values[LBR_SIGNAL_COUNT];

    const lbr_task *const *tasks;
    size_t                 n_tasks;
    uint64_t               next_us[LBR_LOGIC_MAX_TASKS];
    bool                   started[LBR_LOGIC_MAX_TASKS];
    bool                   valid; /* the task list passed lbr_logic_check */

    lbr_output *outputs[LBR_LOGIC_MAX_OUTPUTS];
    size_t      n_outputs;

    lbr_logic_sink_t sink;

    /* Counted, never silent. */
    uint32_t runs;            /* task runs */
    uint32_t late;            /* task runs that started after their next one was due */
    uint32_t outputs_refused; /* writes to outputs beyond LBR_LOGIC_MAX_OUTPUTS, or bad outputs */
} lbr_logic_t;

/* Check a task list: rates, names, at most LBR_LOGIC_MAX_TASKS. 0, or <0 with
 * the reason in err. */
[[nodiscard]] int lbr_logic_check(const lbr_task *const *tasks, size_t n_tasks, char *err,
                                  size_t err_len);

/* Set up the runner. An invalid task list (lbr_logic_check) runs no task. */
void lbr_logic_init(lbr_logic_t *l, const lbr_task *const *tasks, size_t n_tasks,
                    lbr_logic_sink_t sink);

/* A decoded sample from the bus, received at shared time rx_us. */
void lbr_logic_offer(lbr_logic_t *l, const lbr_sample_t *s, uint64_t rx_us);

/* Run every task that is due at shared time now_us. `nodes` says which nodes
 * are present, lost, or missing. */
void lbr_logic_step(lbr_logic_t *l, uint64_t now_us, const lbr_node_table_t *nodes);

/* The output registered under `key`, or NULL (status displays, tests). */
const lbr_output *lbr_logic_find(const lbr_logic_t *l, const char *key);

/* Re-send every output's KEY record, so a log file opened later, or a live
 * viewer that joined late, can name the values (the Central does it 1 Hz). */
void lbr_logic_announce(lbr_logic_t *l);

/*
 * Log records (docs/protocol.md "Car logic outputs").
 *
 *   LBR_REC_KEY (0x20):   key_id u16, value type u8, unit u8, key (ASCII, no NUL)
 *   LBR_REC_VALUE (0x21): t_us u64 (shared time), key_id u16, status u8
 *                         (lbr_status; 0 = OK), value bits u32 (f32 IEEE-754, or
 *                         bool 0|1; 0 when status is not OK)
 */
#define LBR_VALUE_REC_SIZE 15u
