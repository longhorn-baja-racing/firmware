#pragma once

/*
 * Internal to the simulator: what its parts (sim_core.c, sim_platform.c,
 * sim_boot.c, sim_report.c, sim_cmd.c) share with each other. sim.h is the
 * interface to main.c and the vehicle model.
 */
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

#include <errno.h>
#include <math.h>
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>
#include <sys/stat.h>

#include "car_logic.h"
#include "central_plan.h"
#include "corner_plan.h"
#include "cockpit_plan.h"
#include "driver_input_plan.h"
#include "imu_plan.h"
#include "lbr/cobs.h"
#include "lbr/cockpit_link.h"
#include "lbr/nodes.h"
#include "lbr/packet.h"
#include "lbr/status.h"
#include "lbr/version.h"
#include "powertrain_plan.h"
#include "sim.h"
#include "stress_plan.h"
#include "temperature_plan.h"

#define GW            0 /* index of the Central in s->nodes */
#define BOOT_TIME_US  300000u
#define LOG_QUEUE     (64u * 1024u) /* CONFIG_LBR_LOG_QUEUE_KB on the gateway */
#define LOG_SYNC_MS   1000u
#define SYNC_TIMEOUT  50000u
#define WORLD_STEP_US 10000u
#define TRACE_BUDGET  400u
#define WIFI_POOL     512
#define WIFI_BASE_US  1000u /* one-way 5 GHz UDP latency, plus up to 2 ms jitter */

typedef struct {
    sim_t *s;
    int    n;
} hal_ctx_t;

typedef struct {
    bool    used;
    int     from, to;
    size_t  len;
    uint8_t data[LBR_PKT_MAX_SIZE];
} wifi_dgram_t;

extern hal_ctx_t               s_ctx[SIM_MAX_NODES];
extern const lbr_node_hal_t    k_node_hal;
extern const lbr_cockpit_hal_t k_cockpit_hal;
extern const lbr_gw_hal_t      k_gw_hal;
extern wifi_dgram_t            s_wifi[WIFI_POOL];

void        json_str(FILE *o, const char *t);
double      t_s(const sim_t *s);
const char *frame_desc(const lbr_can_frame_t *f, char *buf, size_t cap);
uint64_t    h_now(void *ctx);
uint64_t    h_l2s(void *ctx, uint64_t local);
void        path_in_out(const sim_t *s, const char *name, char *out, size_t cap);
void        keeper_start(sim_t *s, int i, lbr_logger_t *log, const char *config_text);
int         usb_send(lbr_transport_t *self, const uint8_t *pkt, size_t len);
int         wifi_send_pit(lbr_transport_t *self, const uint8_t *pkt, size_t len);
int         lora_send(lbr_transport_t *self, const uint8_t *pkt, size_t len);
int         cockpit_send(lbr_transport_t *self, const uint8_t *pkt, size_t len);
void        link_stats(lbr_transport_t *self, lbr_transport_stats_t *out);
void        wifi_deliver(sim_t *s, int slot);
void        schedule_local(sim_t *s, int node, int job, uint64_t local, sim_ev_type_t type);
void        schedule_acq(sim_t *s, int ni, size_t k);
void        boot_node(sim_t *s, int i);
void        power_off(sim_t *s, int i);
int64_t     sync_error_us(sim_t *s, int i);
bool        wireless(const sim_node_t *n);
bool        link_up(const sim_t *s, int i);
const char *state_of(const sim_t *s, int i);
double      bus_load_pct(const sim_t *s);
bool        gw_sees(const sim_t *s, int i);
bool        compare(double got, const char *op, double want);
