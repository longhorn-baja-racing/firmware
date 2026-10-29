#pragma once

/*
 * LBR car simulator (development only; never built into firmware).
 *
 * Runs the real firmware logic of every node (lbr_node_core_t on the wired
 * controllers, lbr_gw_core_t on the Central, lbr_corner_core_t on the corner
 * nodes, lbr_corner_rx_t on the IMU Controller, lbr_dash_core_t on the
 * dashboard, the acquisition engine, the time servo, the logger, the packet
 * and LoRa code, the input conversions, the NMEA parser) on a virtual car: a
 * bit-timed CAN bus with arbitration and error counters, 5 GHz WiFi links
 * with latency and loss, per-node drifting clocks, a vehicle model behind
 * the sensors, and an SD card that writes a real .LBR file. Every call the
 * firmware makes into its hardware layer is a "command" that can be traced.
 *
 * Time is simulated (microseconds since the simulator started), so runs are
 * deterministic for a given seed and can go faster or slower than real time.
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

#include "lbr/corner_core.h"
#include "lbr/corner_rx.h"
#include "lbr/dash_core.h"
#include "lbr/gw_core.h"
#include "lbr/log_keeper.h"
#include "lbr/logging.h"
#include "lbr/nmea.h"
#include "lbr/node.h"
#include "lbr/node_core.h"
#include "lbr/sensor_io.h"
#include "lbr/value_table.h"

#define SIM_MAX_NODES   12 /* six CAN controllers, four corners, the dashboard */
#define SIM_MAX_SLOTS   8
#define SIM_TXQ_LEN     128 /* the largest CONFIG_LBR_CAN_TX_QUEUE_LEN (IMU Controller) */
#define SIM_MAX_SIGNALS 0x0800u
#define SIM_RECENT      16

/* Periodic jobs after the acquisition slots (sim_node_t.next_local index). */
#define SIM_JOB_HEALTH (SIM_MAX_SLOTS + 0)
#define SIM_JOB_PUMP   (SIM_MAX_SLOTS + 1) /* SD log pump */
#define SIM_JOB_AUX    (SIM_MAX_SLOTS + 2) /* corner clock request / IMU corner service */
#define SIM_JOB_SYNC   (SIM_MAX_SLOTS + 3) /* Central: time sync */
#define SIM_JOB_STREAM (SIM_MAX_SLOTS + 4) /* Central: USB/WiFi batch */
#define SIM_JOB_LORA   (SIM_MAX_SLOTS + 5) /* Central: LoRa */
#define SIM_JOB_DASH   (SIM_MAX_SLOTS + 6) /* Central: dashboard datagram */

typedef struct sim sim_t;

/* ------------------------------------------------------------------------ */
/* Virtual CAN controller (one per node) and bus                             */

typedef struct {
    bool            connected; /* cable attached */
    bool            started;
    bool            accept_all;
    uint64_t        bus_off_until; /* sim us; 0 = not bus-off */
    lbr_can_stats_t st;
    lbr_can_frame_t txq[SIM_TXQ_LEN];
    size_t          tx_head, tx_len;
    size_t          tx_cap; /* this node's CONFIG_LBR_CAN_TX_QUEUE_LEN */
} sim_can_t;

typedef struct {
    uint32_t bitrate;
    bool     busy;
    bool     arb_scheduled;
    uint64_t busy_until;
    int      tx_node; /* node whose frame is on the wire */
    bool     tx_error;
    uint32_t noise_permille;

    /* Load accounting over a sliding one-second window. */
    uint64_t window_start;
    uint64_t busy_us_window, busy_us_last;
    uint32_t frames_window, frames_last;
    uint64_t frames_total, errors_total;
} sim_bus_t;

/* Frames recently on the wire (for the layout animation). */
typedef struct {
    uint64_t        t;
    lbr_can_frame_t f;
    bool            error;
} sim_wire_t;

/* ------------------------------------------------------------------------ */
/* Sensors                                                                   */

typedef enum {
    SIM_SLOT_DIRECT = 0, /* digital part: values straight from the vehicle model */
    SIM_SLOT_IO,         /* on-chip inputs: model -> pin voltage/level -> lbr_io conversion */
    SIM_SLOT_NMEA,       /* GPS: model -> NMEA sentences -> lbr_nmea parser */
    SIM_SLOT_FIRMWARE,   /* the firmware's own driver (corner status, dashboard link) */
} sim_slot_kind_t;

typedef struct {
    sim_t          *sim;
    int             node;
    int             slot;
    sim_slot_kind_t kind;
    const char     *spec; /* SIM_SLOT_IO */
    lbr_io_chan_t   chans[LBR_SENSOR_MAX_SIGNALS];
    lbr_io_din_t    din[LBR_SENSOR_MAX_SIGNALS];
    /* Pulse inputs: an edge counter fed from the model, read through the
     * firmware's lbr_io_pulse_update as sensor_io_esp.c does. */
    lbr_io_pulse_t pulse[LBR_SENSOR_MAX_SIGNALS];
    double         pulse_phase[LBR_SENSOR_MAX_SIGNALS];
    uint32_t       pulse_count[LBR_SENSOR_MAX_SIGNALS];
    uint64_t       pulse_edge_us[LBR_SENSOR_MAX_SIGNALS];
    uint64_t       pulse_read_us[LBR_SENSOR_MAX_SIGNALS];
    lbr_nmea_t     nmea;
    uint64_t       nmea_last_us;
    bool           unplugged; /* part not responding */
    uint32_t       reads;
} sim_slot_t;

/* ------------------------------------------------------------------------ */
/* WiFi (one 5 GHz UDP link per wireless node, to its IMU Controller or the  */
/* Central)                                                                  */

typedef struct {
    bool     up;            /* in range, radio working */
    uint32_t loss_permille; /* random datagram loss */
    uint32_t sent, delivered, dropped;
} sim_wifi_t;

/* ------------------------------------------------------------------------ */
/* Nodes                                                                     */

typedef enum {
    SIM_NODE_CENTRAL = 0, /* gw_core: logger, time master, GPS, LoRa, dashboard link */
    SIM_NODE_CAN,         /* node_core: a wired sensor controller (the IMU one also has rx) */
    SIM_NODE_CORNER,      /* corner_core: WiFi only */
    SIM_NODE_DASH,        /* dash_core: WiFi only */
} sim_node_kind_t;

typedef struct {
    const char     *name;
    uint8_t         id;
    sim_node_kind_t kind;
    bool            is_gateway; /* kind == SIM_NODE_CENTRAL */
    bool            powered;
    bool            booted;
    bool            traced;
    uint32_t        gen;     /* bumps on reboot: stale timer events are ignored */
    int64_t         boot_at; /* sim us; negative after a drift change re-anchors it */
    double          ppm;     /* crystal error */
    uint32_t        boot_count;
    uint8_t         reset_reason;

    sim_can_t        can;
    lbr_time_servo_t servo;

    /* Plan and sensors (every kind; the Central's are its GPS and dashboard link). */
    const lbr_node_plan_t *plan;
    const lbr_io_chip_t   *chip; /* esp32 or esp32c5 */
    lbr_sensor_t           sensors[SIM_MAX_SLOTS];
    lbr_acq_t              acq[SIM_MAX_SLOTS];
    sim_slot_t             slots[SIM_MAX_SLOTS];
    lbr_node_config_t      cfg;
    lbr_node_core_t       *core;      /* SIM_NODE_CAN */
    lbr_logger_t          *local_log; /* a node plan with local_log (none in 2026) */
    lbr_corner_rx_t       *corner_rx; /* the IMU Controller */
    lbr_corner_core_t     *corner;    /* SIM_NODE_CORNER */
    lbr_dash_core_t       *dash;      /* SIM_NODE_DASH */
    sim_wifi_t             wifi;      /* SIM_NODE_CORNER, SIM_NODE_DASH */
    int                    peer;      /* node index at the other end of the WiFi link */

    /* SD card (gateway, and nodes with a local log). */
    lbr_log_keeper_t    keeper;
    bool                sd_inserted;
    uint32_t            sd_fail_writes;
    FILE               *log_file;
    char                log_name[64]; /* last session file created */
    uint32_t            session_no;
    uint64_t            log_bytes;
    lbr_health_record_t last_health;

    /* Next local deadline per periodic job (local clock, us): one per slot,
     * then the SIM_JOB_* jobs. */
    uint64_t next_local[SIM_MAX_SLOTS + 7];
} sim_node_t;

/* Gateway extras. */
typedef struct {
    lbr_gw_core_t  *core;
    lbr_gw_config_t cfg;
    char            config_text[1024];
    uint16_t        epoch;

    lbr_logger_t log;
    uint8_t     *log_queue;
    uint8_t     *log_chunk;

    bool     sync_pending;
    uint8_t  sync_seq;
    uint32_t sync_timeouts;

    lbr_transport_t       usb, wifi, lora, dash;
    lbr_transport_stats_t usb_st, wifi_st, lora_st, dash_st;
    uint32_t              usb_seq, lora_seq;
    FILE                 *usb_file;
    uint64_t              lora_next_tx_local;
    lbr_lora_modem_t      modem;
    uint16_t              last_lora_records;
} sim_gw_t;

/* ------------------------------------------------------------------------ */
/* Vehicle model                                                             */

typedef enum {
    SIM_DRIVE_IDLE = 0,
    SIM_DRIVE_RACE,
} sim_drive_t;

typedef struct {
    sim_drive_t mode;
    double      speed, accel, heading, steer_deg, throttle, brake_kpa;
    double      lat_acc, yaw_rate, vert;
    double      travel[4];    /* suspension travel, mm (drives the corner IMUs and stress) */
    double      travel_v[4];  /* travel rate, mm/s */
    double      corner_az[4]; /* vertical acceleration at each corner, m/s^2 */
    double      roll_rate, pitch_rate;
    double      temp_cvt, temp_brake[4], temp_ambient;
    double      engine_rpm, cvt_rpm;
    double      fuel_pct;
    double      batt_corner[4], batt_dash; /* percent */
    double      x_m, y_m;                  /* position from the start, east/north */
    double      bump;                      /* filtered terrain input */
} sim_world_t;

/* ------------------------------------------------------------------------ */
/* Simulator                                                                 */

typedef struct {
    bool         seen;
    lbr_sample_t s;
    uint64_t     rx_t;
    uint8_t      source;
} sim_value_t;

typedef struct {
    float value;
    bool  set;  /* manual override of the model value */
    bool  open; /* electrical open circuit on this channel */
} sim_override_t;

struct sim {
    uint64_t    now; /* sim us */
    uint64_t    seed;
    uint64_t    rng;
    const char *out_dir;
    bool        quiet; /* scripted run: no trace/snapshot output */
    FILE       *out;   /* JSON lines */
    uint32_t    trace_budget;
    uint32_t    trace_dropped;

    sim_node_t  nodes[SIM_MAX_NODES];
    int         n_nodes;
    sim_gw_t    gw;
    sim_bus_t   bus;
    sim_world_t world;

    sim_value_t    values[SIM_MAX_SIGNALS];
    sim_override_t overrides[SIM_MAX_SIGNALS];
    sim_wire_t     recent[SIM_RECENT];
    size_t         recent_head;

    /* Events seen on the bus (EVENT/FAULT samples), counted per node/code. */
    uint32_t event_counts[SIM_MAX_NODES][0x80];
    uint32_t pending_events_head;

    /* The analysis view of the bus: what a log reader decodes. */
    lbr_bus_decoder_t decoder;

    /* WiFi datagrams in flight. */
    uint32_t wifi_pool_full;

    /* Scenario checks. */
    uint32_t checks_passed, checks_failed;
};

/* Lifecycle (sim.c) */
int  sim_init(sim_t *s, uint64_t seed, const char *out_dir, FILE *out, bool quiet);
void sim_free(sim_t *s);
void sim_run_until(sim_t *s, uint64_t t_us);
int  sim_command(sim_t *s, const char *line); /* <0: unknown/bad command */
void sim_snapshot(sim_t *s);

/* Helpers shared between the simulator files. */
int      sim_node_index(const sim_t *s, const char *name);
uint64_t sim_local_us(const sim_t *s, int node);
uint64_t sim_from_local(const sim_t *s, int node, uint64_t local_us);
double   sim_rand(sim_t *s); /* 0..1 */
void     sim_trace(sim_t *s, int node, const char *cmd, const char *fmt, ...)
    __attribute__((format(printf, 4, 5)));
void sim_note(sim_t *s, const char *fmt, ...) __attribute__((format(printf, 2, 3)));

/* Index of the node with protocol id `id`, or -1. */
int sim_node_by_id(const sim_t *s, uint8_t id);

/* Virtual CAN (vbus.c) */
void     vbus_init(sim_t *s, uint32_t bitrate);
bool     vbus_send(sim_t *s, int node, const lbr_can_frame_t *f);
void     vbus_event(sim_t *s, int kind); /* 0 = arbitrate, 1 = frame end */
void     vbus_kick(sim_t *s);            /* a queue became non-empty */
void     vbus_bus_off(sim_t *s, int node);
void     vbus_recover(sim_t *s, int node);
uint32_t vbus_frame_bits(const lbr_can_frame_t *f); /* exact, with bit stuffing */

/* Virtual WiFi (sim.c): a datagram from node `from` to node `to` over the
 * wireless end's link. */
bool wifi_send(sim_t *s, int from, int to, const uint8_t *buf, size_t len);

/* Vehicle model and sensor drivers (world.c) */
void world_init(sim_world_t *w);
void world_step(sim_t *s, double dt);
void world_bind_sensors(sim_t *s, int node);
bool world_signal_value(sim_t *s, lbr_signal_id_t sig, double *v);

/* Scheduler (sim.c) */
typedef enum {
    EV_BOOT = 0,
    EV_ACQ,
    EV_HEALTH,
    EV_GW_SYNC,
    EV_GW_SYNC_TIMEOUT,
    EV_GW_STREAM,
    EV_GW_LORA,
    EV_GW_PUMP,
    EV_GW_DASH,
    EV_BUS,
    EV_BUS_OFF_END,
    EV_WORLD,
    EV_WIFI,         /* slot = datagram pool index */
    EV_CORNER_CLOCK, /* corner node clock request */
    EV_CORNER_RX,    /* IMU Controller corner service */
} sim_ev_type_t;

void sim_schedule(sim_t *s, uint64_t t, sim_ev_type_t type, int node, int slot);

/* Callbacks from the bus into the nodes. */
void sim_deliver(sim_t *s, int node, const lbr_can_frame_t *f, uint64_t t);
void sim_tx_done(sim_t *s, int node, const lbr_can_frame_t *f, bool ok, uint64_t t);
