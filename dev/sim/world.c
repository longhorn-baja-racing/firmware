/*
 * Vehicle model and simulated sensor parts.
 *
 * The model drives a Baja car around a loop (or idles in the pit) and
 * produces the physical quantity behind every catalog signal. The simulated
 * parts then present it to the firmware the way real hardware would:
 *   - on-chip inputs (SIM_SLOT_IO): the quantity becomes a pin voltage or
 *     level and goes through the firmware's real lbr_io conversion, using the
 *     same input-spec syntax as menuconfig;
 *   - the GPS (SIM_SLOT_NMEA): the position becomes NMEA sentences that the
 *     firmware's real NMEA parser decodes;
 *   - digital parts (SIM_SLOT_DIRECT, e.g. the IMU): values in catalog units.
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "lbr/nodes.h"
#include "sim.h"

#define PI          3.14159265358979
#define WHEELBASE_M 1.6
#define TANK_L      4.0
#define START_LAT   30.2849 /* a loop near Austin, TX */
#define START_LON   (-97.7341)
#define UTC_BASE    1792300000u

static double clampd(double v, double lo, double hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

static double gauss(sim_t *s)
{
    double u = sim_rand(s) + 1e-12;
    double v = sim_rand(s);
    return sqrt(-2.0 * log(u)) * cos(2.0 * PI * v);
}

/* ------------------------------------------------------------------------ */
/* Vehicle model                                                             */

void world_init(sim_world_t *w)
{
    memset(w, 0, sizeof *w);
    w->mode         = SIM_DRIVE_RACE;
    w->temp_ambient = 31.0;
    w->temp_engine  = 31.0;
    w->temp_cvt     = 31.0;
    w->temp_gearbox = 31.0;
    w->temp_brake_f = 31.0;
    w->temp_brake_r = 31.0;
    w->fuel_pct     = 96.0;
    for (int i = 0; i < 4; i++) {
        w->travel[i] = 80.0;
    }
}

static double approach(double v, double target, double tau_s, double dt)
{
    return v + (target - v) * (dt / tau_s);
}

void world_step(sim_t *s, double dt)
{
    sim_world_t *w = &s->world;
    double       t = (double)s->now / 1e6;

    double v_target = w->mode == SIM_DRIVE_RACE ? 9.5 + 3.5 * cos(2.0 * PI * t / 20.0) : 0.0;
    w->accel        = clampd((v_target - w->speed) * 1.2, -6.0, 4.0);
    w->speed        = fmax(0.0, w->speed + w->accel * dt);
    if (w->mode == SIM_DRIVE_RACE) {
        w->throttle =
            w->accel >= -0.5 ? clampd(30.0 + w->accel * 15.0 + w->speed * 2.0, 0, 100) : 0;
        w->brake_kpa = w->accel < -0.8 ? -w->accel * 900.0 : 0.0;
        /* Corners come in the slow part of the lap: lateral load stays
         * around 0.4 g, as on a Baja course. */
        double slow  = clampd((13.0 - w->speed) / 7.0, 0.0, 1.0);
        w->steer_deg = 110.0 * sin(2.0 * PI * t / 40.0) * slow;
    } else {
        w->throttle  = 3.0;
        w->brake_kpa = w->speed > 0.1 ? 1500.0 : 0.0;
        w->steer_deg = approach(w->steer_deg, 0.0, 1.0, dt);
    }

    double road_wheel = (w->steer_deg / 12.0) * PI / 180.0; /* 12:1 steering ratio */
    double curvature  = tan(road_wheel) / WHEELBASE_M;
    w->yaw_rate       = w->speed * curvature;
    w->heading        = fmod(w->heading + w->yaw_rate * dt + 2.0 * PI, 2.0 * PI);
    w->lat_acc        = w->speed * w->yaw_rate;
    w->x_m += w->speed * sin(w->heading) * dt;
    w->y_m += w->speed * cos(w->heading) * dt;

    /* Terrain: filtered noise plus an occasional big hit. */
    double prev_bump = w->bump;
    w->bump          = 0.92 * w->bump + 0.08 * gauss(s) * 25.0 * (w->speed / 10.0);
    if (w->mode == SIM_DRIVE_RACE && sim_rand(s) < dt / 7.0) {
        w->bump += 70.0;
    }
    w->vert = (w->bump - prev_bump) / dt / 1000.0 * 4.0;

    for (int i = 0; i < 4; i++) {
        bool   left  = (i % 2) == 0;
        bool   front = i < 2;
        double roll  = w->lat_acc * 4.0 * (left ? 1.0 : -1.0);
        double pitch = w->accel * 3.0 * (front ? -1.0 : 1.0);
        double hit   = w->bump * (front ? 1.0 : 0.7) + gauss(s) * 2.0;
        w->travel[i] = clampd(80.0 + roll + pitch + hit, 0.0, 240.0);
    }

    w->temp_engine  = approach(w->temp_engine, 95.0 + w->throttle / 10.0, 60.0, dt);
    w->temp_cvt     = approach(w->temp_cvt, 60.0 + w->speed * 3.0, 90.0, dt);
    w->temp_gearbox = approach(w->temp_gearbox, 50.0 + w->speed * 2.0, 120.0, dt);
    w->temp_brake_f += (w->brake_kpa * 0.02 - (w->temp_brake_f - w->temp_ambient) * 0.03) * dt;
    w->temp_brake_r += (w->brake_kpa * 0.012 - (w->temp_brake_r - w->temp_ambient) * 0.03) * dt;
    w->fuel_pct = fmax(0.0, w->fuel_pct - (0.002 + w->throttle * 0.0004) * dt);
}

static double heading_deg(const sim_world_t *w)
{
    return w->heading * 180.0 / PI;
}

bool world_signal_value(sim_t *s, lbr_signal_id_t sig, double *v)
{
    const sim_world_t *w = &s->world;
    double             h = w->heading;
    switch (sig) {
        case LBR_SIG_ACCEL_X:
            *v = w->accel;
            return true;
        case LBR_SIG_ACCEL_Y:
            *v = w->lat_acc;
            return true;
        case LBR_SIG_ACCEL_Z:
            *v = 9.81 + w->vert;
            return true;
        case LBR_SIG_GYRO_X:
        case LBR_SIG_GYRO_Y:
            *v = 0.02 * gauss(s);
            return true;
        case LBR_SIG_GYRO_Z:
            *v = w->yaw_rate;
            return true;
        case LBR_SIG_ORIENT_QUAT_W:
            *v = cos(h / 2.0);
            return true;
        case LBR_SIG_ORIENT_QUAT_X:
        case LBR_SIG_ORIENT_QUAT_Y:
            *v = 0.0;
            return true;
        case LBR_SIG_ORIENT_QUAT_Z:
            *v = sin(h / 2.0);
            return true;
        case LBR_SIG_MAG_X:
            *v = 24.0 * cos(h);
            return true;
        case LBR_SIG_MAG_Y:
            *v = -24.0 * sin(h);
            return true;
        case LBR_SIG_MAG_Z:
            *v = -42.0;
            return true;
        case LBR_SIG_IMU_TEMP:
            *v = 36.0 + 0.1 * gauss(s);
            return true;
        case LBR_SIG_IMU_CALIB_STATE:
            *v = 3.0;
            return true;
        case LBR_SIG_FUEL_LEVEL:
            *v = w->fuel_pct;
            return true;
        case LBR_SIG_FUEL_VOLUME:
            *v = w->fuel_pct / 100.0 * TANK_L;
            return true;
        case LBR_SIG_TEMP_ENGINE:
            *v = w->temp_engine;
            return true;
        case LBR_SIG_TEMP_CVT:
            *v = w->temp_cvt;
            return true;
        case LBR_SIG_TEMP_GEARBOX:
            *v = w->temp_gearbox;
            return true;
        case LBR_SIG_TEMP_BRAKE_FRONT:
            *v = w->temp_brake_f;
            return true;
        case LBR_SIG_TEMP_BRAKE_REAR:
            *v = w->temp_brake_r;
            return true;
        case LBR_SIG_TEMP_AMBIENT:
            *v = w->temp_ambient;
            return true;
        case LBR_SIG_SUSP_TRAVEL_FL:
        case LBR_SIG_SUSP_TRAVEL_FR:
        case LBR_SIG_SUSP_TRAVEL_RL:
        case LBR_SIG_SUSP_TRAVEL_RR:
            *v = w->travel[sig - LBR_SIG_SUSP_TRAVEL_FL];
            return true;
        case LBR_SIG_THROTTLE_POS:
            *v = w->throttle;
            return true;
        case LBR_SIG_BRAKE_PRESS_FRONT:
            *v = w->brake_kpa;
            return true;
        case LBR_SIG_BRAKE_PRESS_REAR:
            *v = w->brake_kpa * 0.8;
            return true;
        case LBR_SIG_STEERING_ANGLE:
            *v = w->steer_deg;
            return true;
        case LBR_SIG_BRAKE_SWITCH:
            *v = w->brake_kpa > 200.0 ? 1.0 : 0.0;
            return true;
        case LBR_SIG_DASH_BUTTONS:
            *v = (double)w->buttons;
            return true;
        case LBR_SIG_GPS_SPEED:
            *v = w->speed;
            return true;
        case LBR_SIG_GPS_COURSE:
            *v = heading_deg(w);
            return true;
        default:
            break;
    }
    if (sig >= LBR_SIG_STRAIN_1 && sig <= LBR_SIG_STRAIN_8) {
        unsigned k    = sig - LBR_SIG_STRAIN_1;
        double   load = w->travel[k % 4] - 80.0;
        *v            = load * (6.0 + (double)k) + (k >= 4 ? w->lat_acc * 30.0 : 0.0) + gauss(s);
        return true;
    }
    return false;
}

/* ------------------------------------------------------------------------ */
/* NMEA receiver model                                                       */

static void civil_from_days(int64_t z, int *y, unsigned *m, unsigned *d)
{
    z += 719468;
    int64_t  era = (z >= 0 ? z : z - 146096) / 146097;
    unsigned doe = (unsigned)(z - era * 146097);
    unsigned yoe = (doe - doe / 1460 + doe / 36524 - doe / 146096) / 365;
    unsigned doy = doe - (365 * yoe + yoe / 4 - yoe / 100);
    unsigned mp  = (5 * doy + 2) / 153;
    *d           = doy - (153 * mp + 2) / 5 + 1;
    *m           = mp < 10 ? mp + 3 : mp - 9;
    *y           = (int)(yoe + (unsigned)era * 400u) + (*m <= 2);
}

static size_t sentence(char *out, size_t cap, const char *body)
{
    uint8_t sum = 0;
    for (const char *p = body; *p != '\0'; p++) {
        sum ^= (uint8_t)*p;
    }
    int n = snprintf(out, cap, "$%s*%02X\r\n", body, sum);
    return n > 0 && (size_t)n < cap ? (size_t)n : 0;
}

static void fmt_coord(char *out, size_t cap, double deg, bool lon)
{
    double a  = fabs(deg);
    int    d  = (int)a;
    double mi = (a - d) * 60.0;
    snprintf(out, cap, lon ? "%03d%08.5f,%c" : "%02d%08.5f,%c", d, mi,
             lon ? (deg < 0 ? 'W' : 'E') : (deg < 0 ? 'S' : 'N'));
}

/* One second's worth of a 10 Hz receiver: GGA, RMC, GSA for this epoch. */
static size_t nmea_epoch(sim_t *s, char *buf, size_t cap)
{
    const sim_world_t *w    = &s->world;
    double             lat  = START_LAT + w->y_m / 111320.0;
    double             lon  = START_LON + w->x_m / (111320.0 * cos(START_LAT * PI / 180.0));
    double             t    = (double)UTC_BASE + (double)s->now / 1e6;
    int64_t            secs = (int64_t)t;
    int                y;
    unsigned           mo, d;
    civil_from_days(secs / 86400, &y, &mo, &d);
    unsigned sod = (unsigned)(secs % 86400);
    char     hms[16], la[24], lo[24], body[160];
    snprintf(hms, sizeof hms, "%02u%02u%02u.%02u", sod / 3600, (sod / 60) % 60, sod % 60,
             (unsigned)((t - (double)secs) * 100.0));
    fmt_coord(la, sizeof la, lat, false);
    fmt_coord(lo, sizeof lo, lon, true);

    size_t n = 0;
    snprintf(body, sizeof body, "GNGGA,%s,%s,%s,1,10,0.9,%.1f,M,-22.0,M,,", hms, la, lo, 149.0);
    n += sentence(buf + n, cap - n, body);
    snprintf(body, sizeof body, "GNRMC,%s,A,%s,%s,%.2f,%.1f,%02u%02u%02u,,,A", hms, la, lo,
             w->speed / 0.514444, heading_deg(w), d, mo, (unsigned)(y % 100));
    n += sentence(buf + n, cap - n, body);
    n += sentence(buf + n, cap - n, "GNGSA,A,3,01,03,06,09,12,14,17,19,22,24,,,1.6,0.9,1.3");
    return n;
}

/* ------------------------------------------------------------------------ */
/* Simulated parts                                                           */

static double model_value(sim_t *s, lbr_signal_id_t sig, bool *have)
{
    double v = 0.0;
    if (sig < SIM_MAX_SIGNALS && s->overrides[sig].set) {
        *have = true;
        return s->overrides[sig].value;
    }
    *have = world_signal_value(s, sig, &v);
    return v;
}

static bool chan_open(const sim_t *s, lbr_signal_id_t sig)
{
    return sig < SIM_MAX_SIGNALS && s->overrides[sig].open;
}

/* Pin voltage an ADC input shows for physical value v (inverse of the spec). */
static double adc_mv(const lbr_io_chan_t *c, double v)
{
    if (c->u.adc.ntc) {
        double tk = v + 273.15;
        double r  = c->u.adc.r25 * exp(c->u.adc.beta * (1.0 / tk - 1.0 / 298.15));
        return c->u.adc.vsup_mv * r / (r + c->u.adc.rpull);
    }
    return c->u.adc.mv0 +
           (v - c->u.adc.v0) * (c->u.adc.mv1 - c->u.adc.mv0) / (c->u.adc.v1 - c->u.adc.v0);
}

static int part_init(lbr_sensor_t *self)
{
    sim_slot_t *sl = self->ctx;
    sim_trace(sl->sim, sl->node, "sensor.init", "%s%s", self->name,
              sl->unplugged ? " -> no response" : " -> ok");
    if (sl->unplugged) {
        return -1;
    }
    memset(sl->din, 0, sizeof sl->din);
    return 0;
}

static void read_io(sim_slot_t *sl, lbr_sensor_t *self, size_t i, lbr_reading_t *r, uint64_t local)
{
    sim_t               *s   = sl->sim;
    const lbr_io_chan_t *c   = &sl->chans[i];
    lbr_signal_id_t      sig = self->signals[i];
    bool                 have;
    double               v = model_value(s, sig, &have);

    switch (c->kind) {
        case LBR_IO_ADC: {
            double mv = chan_open(s, sig) || !have ? 0.0 : adc_mv(c, v);
            mv        = clampd(mv + gauss(s) * 1.5, 0.0, 3300.0);
            mv        = floor(mv); /* calibrated driver output is whole mV */
            lbr_io_adc_convert(c, (float)mv, r);
            sim_trace(s, sl->node, "adc.read", "gpio %d: %.0f mV -> %s", (int)c->u.adc.gpio, mv,
                      r->valid ? "ok" : lbr_unavail_reason_name(r->reason));
            break;
        }
        case LBR_IO_DIN: {
            uint32_t raw = chan_open(s, sig) || !have ? 0u : (uint32_t)v;
            uint32_t out = lbr_io_din_update(&sl->din[i], raw, local, c->u.din.debounce_ms);
            *r           = (lbr_reading_t){.valid = true};
            if (lbr_signal_info(sig)->type == LBR_VAL_BOOL) {
                r->v.b = out != 0;
            } else {
                r->v.u32 = out;
            }
            sim_trace(s, sl->node, "gpio.read", "%s level 0x%lx", lbr_signal_info(sig)->name,
                      (unsigned long)raw);
            break;
        }
        case LBR_IO_PULSE:
            *r = (lbr_reading_t){.valid = true, .v.f32 = have ? (float)v : 0.0f};
            break;
        default:
            *r = (lbr_reading_t){.valid = false, .reason = LBR_UNAVAIL_NOT_FITTED};
            break;
    }
}

static void read_direct(sim_slot_t *sl, lbr_signal_id_t sig, lbr_reading_t *r)
{
    sim_t *s = sl->sim;
    bool   have;
    double v = model_value(s, sig, &have);
    if (!have) {
        *r = (lbr_reading_t){.valid = false, .reason = LBR_UNAVAIL_NOT_FITTED};
        return;
    }
    if (chan_open(s, sig)) {
        *r = (lbr_reading_t){.valid = false, .reason = LBR_UNAVAIL_SENSOR_FAULT};
        return;
    }
    switch (lbr_signal_info(sig)->type) {
        case LBR_VAL_F32:
            *r = (lbr_reading_t){.valid = true, .v.f32 = (float)v};
            break;
        case LBR_VAL_I32:
            *r = (lbr_reading_t){.valid = true, .v.i32 = (int32_t)v};
            break;
        case LBR_VAL_BOOL:
            *r = (lbr_reading_t){.valid = true, .v.b = v != 0.0};
            break;
        default:
            *r = (lbr_reading_t){.valid = true, .v.u32 = (uint32_t)v};
            break;
    }
}

static int part_read(lbr_sensor_t *self, uint64_t t_us, lbr_reading_t *out)
{
    (void)t_us;
    sim_slot_t *sl    = self->ctx;
    sim_t      *s     = sl->sim;
    uint64_t    local = sim_local_us(s, sl->node);
    sl->reads++;

    if (sl->kind == SIM_SLOT_NMEA) {
        /* The receiver keeps talking at 10 Hz; a disconnected one goes quiet
         * and the parser reports NO_DATA once its data is stale. */
        if (!sl->unplugged && local - sl->nmea_last_us >= 100000u) {
            char   buf[512];
            size_t n = nmea_epoch(s, buf, sizeof buf);
            lbr_nmea_feed(&sl->nmea, (const uint8_t *)buf, n, local);
            sl->nmea_last_us = local;
            sim_trace(s, sl->node, "uart.read", "%u bytes of NMEA (GGA, RMC, GSA)", (unsigned)n);
        }
        lbr_nmea_fill(&sl->nmea, local, self->signals, self->n_signals, out);
        return 0;
    }
    if (sl->unplugged) {
        sim_trace(s, sl->node, "sensor.read", "%s -> no response", self->name);
        return -1;
    }
    for (size_t i = 0; i < self->n_signals; i++) {
        if (sl->kind == SIM_SLOT_IO) {
            read_io(sl, self, i, &out[i], local);
        } else {
            read_direct(sl, self->signals[i], &out[i]);
        }
    }
    if (sl->kind == SIM_SLOT_DIRECT) {
        sim_trace(s, sl->node, "sensor.read", "%s: %u channels", self->name,
                  (unsigned)self->n_signals);
    }
    return 0;
}

/* How each slot of the 2026 car is wired in the simulator. IO specs use the
 * menuconfig syntax (CONFIG_LBR_IO_<SLOT>) and are plausible wiring for
 * bench work, not a pin map. */
typedef struct {
    const char     *node;
    const char     *slot;
    sim_slot_kind_t kind;
    const char     *spec;
} wiring_t;

static const wiring_t k_wiring[] = {
    {"suspension", "travel", SIM_SLOT_IO,
     "adc gpio=32 mv0=200 v0=0 mv1=2400 v1=250 lo=100 hi=2500;"
     "adc gpio=33 mv0=200 v0=0 mv1=2400 v1=250 lo=100 hi=2500;"
     "adc gpio=34 mv0=200 v0=0 mv1=2400 v1=250 lo=100 hi=2500;"
     "adc gpio=35 mv0=200 v0=0 mv1=2400 v1=250 lo=100 hi=2500"},
    {"driver_inputs", "pedals", SIM_SLOT_IO,
     "adc gpio=36 mv0=300 v0=0 mv1=2300 v1=100 lo=150 hi=2450 min=-5 max=105;"
     "adc gpio=39 mv0=200 v0=0 mv1=2400 v1=13790 lo=100 hi=2500;"
     "adc gpio=34 mv0=200 v0=0 mv1=2400 v1=13790 lo=100 hi=2500"},
    {"driver_inputs", "steering", SIM_SLOT_IO,
     "adc gpio=35 mv0=200 v0=-180 mv1=2400 v1=180 lo=100 hi=2500"},
    {"driver_inputs", "brake_switch", SIM_SLOT_IO, "din gpio=4 active=low pull=up debounce_ms=10"},
    {"temperature", "drivetrain", SIM_SLOT_IO,
     "adc gpio=32 curve=ntc r25=10000 beta=3950 rpull=2200 vsup=3300 lo=50 hi=3250;"
     "adc gpio=33 curve=ntc r25=10000 beta=3950 rpull=2200 vsup=3300 lo=50 hi=3250;"
     "adc gpio=34 curve=ntc r25=10000 beta=3950 rpull=2200 vsup=3300 lo=50 hi=3250"},
    {"temperature", "brakes", SIM_SLOT_DIRECT, NULL},
    {"temperature", "ambient", SIM_SLOT_IO,
     "adc gpio=35 curve=ntc r25=10000 beta=3950 rpull=10000 vsup=3300 lo=50 hi=3250"},
    {"temperature", "aux", SIM_SLOT_IO, "-;-;-;-"},
    {"fuel_gauge", "fuel", SIM_SLOT_IO,
     "adc gpio=36 mv0=250 v0=0 mv1=2350 v1=100 lo=120 hi=2480;-"},
    {"dashboard", "buttons", SIM_SLOT_IO, "din gpio=4,5,18,19"},
    {"imu_gps", "gps", SIM_SLOT_NMEA, NULL},
};

static const wiring_t *wiring_for(const char *node, const char *slot)
{
    for (size_t i = 0; i < sizeof k_wiring / sizeof k_wiring[0]; i++) {
        if (strcmp(k_wiring[i].node, node) == 0 && strcmp(k_wiring[i].slot, slot) == 0) {
            return &k_wiring[i];
        }
    }
    return NULL; /* digital part */
}

void world_bind_sensors(sim_t *s, int node)
{
    sim_node_t *n = &s->nodes[node];
    lbr_node_build(n->plan, lbr_sensor_unconfigured, n->sensors, n->acq, &n->cfg);
    for (size_t i = 0; i < n->plan->n_slots && i < SIM_MAX_SLOTS; i++) {
        const lbr_slot_t *slot = &n->plan->slots[i];
        const wiring_t   *wr   = wiring_for(n->name, slot->name);
        sim_slot_t       *sl   = &n->slots[i];
        bool              keep = sl->unplugged; /* survives a reboot */
        memset(sl, 0, sizeof *sl);
        sl->sim       = s;
        sl->node      = node;
        sl->slot      = (int)i;
        sl->unplugged = keep;
        sl->kind      = wr != NULL ? wr->kind : SIM_SLOT_DIRECT;
        sl->spec      = wr != NULL ? wr->spec : NULL;
        lbr_nmea_init(&sl->nmea);
        if (sl->kind == SIM_SLOT_IO) {
            char err[96];
            if (lbr_io_parse(sl->spec, slot->signals, slot->n_signals, sl->chans, err,
                             sizeof err) != 0) {
                fprintf(stderr, "sim: bad wiring for %s.%s: %s\n", n->name, slot->name, err);
                exit(2);
            }
        }
        n->sensors[i] = (lbr_sensor_t){
            .name      = slot->name,
            .signals   = slot->signals,
            .n_signals = slot->n_signals,
            .init      = part_init,
            .read      = part_read,
            .simulated = true, /* flagged in heartbeats and session config */
            .params    = sl->spec,
            .ctx       = sl,
        };
    }
}
