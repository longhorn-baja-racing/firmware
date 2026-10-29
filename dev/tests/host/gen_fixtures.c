/*
 * Golden fixtures shared with lbr-dashboard (dev/tests/fixtures).
 *
 *   gen_fixtures write <dir>   regenerate every fixture
 *   gen_fixtures check <dir>   regenerate in memory, byte-compare with <dir>
 *
 * Binary fixtures are produced by the real codecs; each has a JSON file with
 * the expected decode. Any change to a wire format makes "check" fail until
 * the fixtures are regenerated on purpose and the dashboard is updated in
 * step. dev/tools/lbr_decode.py --selftest replays the same fixtures as an
 * independent (Python) decoder.
 */

#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "lbr/bytes.h"
#include "lbr/can.h"
#include "lbr/cobs.h"
#include "lbr/frames.h"
#include "lbr/health.h"
#include "lbr/logging.h"
#include "lbr/nodes.h"
#include "lbr/packet.h"
#include "lbr/protocol.h"
#include "lbr/transport.h"
#include "lbr/version.h"

/* ------------------------------------------------------------------------ */
/* Growable buffers                                                          */

typedef struct {
    uint8_t *data;
    size_t   len;
    size_t   cap;
} buf_t;

static void buf_put(buf_t *b, const void *p, size_t n)
{
    if (b->len + n + 1 > b->cap) {
        b->cap  = (b->len + n + 1) * 2;
        b->data = realloc(b->data, b->cap);
        if (b->data == NULL) {
            abort();
        }
    }
    memcpy(b->data + b->len, p, n);
    b->len += n;
    b->data[b->len] = 0;
}

static void jf(buf_t *b, const char *fmt, ...)
{
    char    tmp[4096];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf(tmp, sizeof tmp, fmt, ap);
    va_end(ap);
    if (n < 0 || (size_t)n >= sizeof tmp) {
        abort();
    }
    buf_put(b, tmp, (size_t)n);
}

static void json_str(buf_t *b, const char *s, size_t n)
{
    buf_put(b, "\"", 1);
    for (size_t i = 0; i < n; i++) {
        unsigned char c = (unsigned char)s[i];
        if (c == '"' || c == '\\') {
            jf(b, "\\%c", c);
        } else if (c == '\n') {
            jf(b, "\\n");
        } else if (c < 0x20 || c > 0x7E) {
            jf(b, "\\u%04x", c);
        } else {
            buf_put(b, &c, 1);
        }
    }
    buf_put(b, "\"", 1);
}

static void json_hex(buf_t *b, const uint8_t *p, size_t n)
{
    buf_put(b, "\"", 1);
    for (size_t i = 0; i < n; i++) {
        jf(b, "%02x", p[i]);
    }
    buf_put(b, "\"", 1);
}

/* ------------------------------------------------------------------------ */
/* JSON for decoded objects (shared by all expected-decode files)            */

static void json_sample(buf_t *b, const lbr_sample_t *s)
{
    const lbr_signal_info_t *info = lbr_signal_info(s->signal);
    jf(b, "{\"signal\": %u, \"name\": ", (unsigned)s->signal);
    json_str(b, info ? info->name : "?", strlen(info ? info->name : "?"));
    jf(b, ", \"source\": %u, \"type\": \"%s\", \"valid\": %s, \"synced\": %s, ", s->source,
       lbr_value_type_name(s->type), s->valid ? "true" : "false", s->synced ? "true" : "false");
    jf(b, "\"seq_inferred\": %s, \"seq\": %lu, \"t_us\": %llu, \"value_bits\": \"%08lx\"",
       s->seq_inferred ? "true" : "false", (unsigned long)s->seq, (unsigned long long)s->t_mono_us,
       (unsigned long)lbr_sample_value_bits(s));
    if (!s->valid) {
        jf(b, ", \"reason\": \"%s\"", lbr_unavail_reason_name((lbr_unavail_reason_t)s->v.u32));
    } else if (s->signal == LBR_SIG_EVENT || s->signal == LBR_SIG_FAULT) {
        jf(b, ", \"event\": \"%s\", \"event_arg\": %u",
           lbr_event_name((lbr_event_code_t)(s->v.u32 & 0xFFFFu)), (unsigned)(s->v.u32 >> 16));
    }
    jf(b, "}");
}

static void json_health(buf_t *b, const lbr_health_record_t *h)
{
    jf(b, "{\"node_id\": %u, \"state\": %u, \"flags\": %u, \"fw_version\": %lu, ", h->node_id,
       h->state, h->flags, (unsigned long)h->fw_version);
    jf(b, "\"protocol_version\": %u, \"boot_count\": %u, \"catalog_hash\": %lu, ",
       h->protocol_version, h->boot_count, (unsigned long)h->catalog_hash);
    jf(b, "\"config_crc\": %lu, \"uptime_ms\": %lu, \"reset_reason\": %u, ",
       (unsigned long)h->config_crc, (unsigned long)h->uptime_ms, h->reset_reason);
    jf(b, "\"acquire_hz\": %u, \"publish_hz\": %u, \"log_hz\": %u, ", h->acquire_hz, h->publish_hz,
       h->log_hz);
    jf(b, "\"queue_high_water\": %lu, \"overflow_count\": %lu, \"rejected_count\": %lu, ",
       (unsigned long)h->queue_high_water, (unsigned long)h->overflow_count,
       (unsigned long)h->rejected_count);
    jf(b, "\"write_error_count\": %lu, \"transport_error_count\": %lu, ",
       (unsigned long)h->write_error_count, (unsigned long)h->transport_error_count);
    jf(b, "\"can_tec\": %u, \"can_rec\": %u, \"can_bus_off_count\": %lu, ", h->can_tec, h->can_rec,
       (unsigned long)h->can_bus_off_count);
    jf(b, "\"can_tx_dropped\": %lu, \"can_rx_dropped\": %lu, \"can_bus_errors\": %lu, ",
       (unsigned long)h->can_tx_dropped, (unsigned long)h->can_rx_dropped,
       (unsigned long)h->can_bus_errors);
    jf(b, "\"sensor_status\": %lu, \"calibration_state\": %lu, ", (unsigned long)h->sensor_status,
       (unsigned long)h->calibration_state);
    jf(b, "\"last_sample_age_ms\": %lu, \"last_sd_write_age_ms\": %lu, ",
       (unsigned long)h->last_sample_age_ms, (unsigned long)h->last_sd_write_age_ms);
    jf(b, "\"time_step_count\": %lu, \"last_sync_error_us\": %ld}",
       (unsigned long)h->time_step_count, (long)h->last_sync_error_us);
}

/* Decode a frame as the Central's live views do. `dec` carries the sync
 * state across a capture (heartbeats, CORNER_STATUS), as a reader must. */
static void json_can_frame(buf_t *b, lbr_bus_decoder_t *dec, uint64_t t_us, uint8_t flags,
                           const lbr_can_frame_t *f)
{
    jf(b, "{\"t_us\": %llu, \"flags\": %u, \"id\": %lu, \"sender\": %u, \"dlc\": %u, \"data\": ",
       (unsigned long long)t_us, flags, (unsigned long)f->id,
       (flags & LBR_CAN_REC_FLAG_EXT) ? 0u : lbr_can_sender(f->id), f->dlc);
    json_hex(b, f->data, f->dlc);
    if (flags & LBR_CAN_REC_FLAG_EXT) {
        jf(b, "}"); /* not LBR traffic */
        return;
    }
    lbr_health_record_t h = {0};
    uint8_t             seq, code, state, hbf;
    uint8_t             args[7];
    uint16_t            epoch;
    uint64_t            tx;
    lbr_sample_t        s[LBR_FRAME_MAX_FIELDS];
    size_t              n = lbr_bus_decode(dec, f, t_us, s, LBR_FRAME_MAX_FIELDS);
    if (n > 0) {
        jf(b, ", \"samples\": [");
        for (size_t i = 0; i < n; i++) {
            jf(b, "%s", i ? ", " : "");
            json_sample(b, &s[i]);
        }
        jf(b, "]");
    } else if (lbr_can_decode_heartbeat(f, &state, &hbf) != 0) {
        jf(b, ", \"heartbeat\": {\"state\": %u, \"err_flags\": %u}", state, hbf);
    } else if (lbr_can_decode_time_sync(f, &seq, &epoch)) {
        jf(b, ", \"time_sync\": {\"seq\": %u, \"epoch\": %u, \"ms\": %lu}", seq, epoch,
           (unsigned long)lbr_get_u32(f->data));
    } else if (lbr_can_decode_time_follow_up(f, &seq, &tx)) {
        jf(b, ", \"time_follow_up\": {\"seq\": %u, \"tx_us\": %llu}", seq, (unsigned long long)tx);
    } else if (lbr_can_decode_command(f, &code, args)) {
        jf(b, ", \"command\": {\"code\": %u, \"args\": ", code);
        json_hex(b, args, 7);
        jf(b, "}");
    } else {
        int page = lbr_can_decode_health_page(f, &h);
        if (page >= 0) {
            jf(b, ", \"health_page\": %d", page);
        }
    }
    jf(b, "}");
}

/* ------------------------------------------------------------------------ */
/* Fixture content                                                           */

static lbr_sample_t mk(lbr_signal_id_t sig, uint8_t src, uint32_t seq, uint64_t t)
{
    const lbr_signal_info_t *info = lbr_signal_info(sig);
    lbr_sample_t             s    = {
        .signal    = sig,
        .source    = src,
        .type      = info ? info->type : LBR_VAL_RAW,
        .valid     = true,
        .synced    = true,
        .seq       = seq,
        .t_mono_us = t,
    };
    return s;
}

/* A representative set covering every value type, unavailability, events,
 * and counter edge cases. */
static size_t sample_set(lbr_sample_t *out)
{
    size_t n       = 0;
    out[n]         = mk(LBR_SIG_IMU_CH_ACCEL_Z, LBR_NODE_IMU, 0, 1000000);
    out[n++].v.f32 = -9.80665f;
    out[n]         = mk(LBR_SIG_GPS_LATITUDE, LBR_NODE_CENTRAL, 1, 1000100);
    out[n++].v.i32 = 302849870; /* 30.2849870 N */
    out[n]         = mk(LBR_SIG_GPS_SATELLITES, LBR_NODE_CENTRAL, 2, 1000200);
    out[n++].v.u32 = 11;
    out[n]         = mk(LBR_SIG_DASH_LINK, LBR_NODE_CENTRAL, 0xFFFFFFFFu, 2000000);
    out[n++].v.b   = true;
    out[n]         = mk(LBR_SIG_GPS_FIX, LBR_NODE_CENTRAL, 3, 2000300);
    out[n++].v.u32 = 3;
    out[n]         = mk(LBR_SIG_TEMP_CVT, LBR_NODE_TEMPERATURE, 7, 3000000);
    lbr_sample_set_unavailable(&out[n++], LBR_UNAVAIL_NOT_FITTED);
    out[n]         = mk(LBR_SIG_FAULT, LBR_NODE_CENTRAL, 0, 4000000);
    out[n++].v.u32 = lbr_event_value(LBR_EVT_NODE_LOST, LBR_NODE_STRESS);
    out[n]         = mk(LBR_SIG_STRESS_3, LBR_NODE_STRESS, 123456, 0x0000123456789ABCull);
    out[n].synced  = false;
    out[n++].v.f32 = 412.5f;
    return n;
}

static void gen_catalog(buf_t *j)
{
    jf(j, "{\n  \"protocol_version\": %u,\n  \"log_format_version\": %u,\n", LBR_PROTOCOL_VERSION,
       LBR_LOG_FORMAT_VERSION);
    jf(j, "  \"catalog_hash\": %lu,\n  \"signals\": [\n", (unsigned long)lbr_catalog_hash());
    for (size_t i = 0; i < lbr_signal_count(); i++) {
        const lbr_signal_info_t *s = lbr_signal_at(i);
        jf(j, "    {\"id\": %u, \"name\": \"%s\", \"type\": \"%s\", \"unit\": \"%s\", ",
           (unsigned)s->id, s->name, lbr_value_type_name(s->type), lbr_unit_name(s->unit));
        jf(j, "\"desc\": ");
        json_str(j, s->desc, strlen(s->desc));
        jf(j, "}%s\n", i + 1 < lbr_signal_count() ? "," : "");
    }
    jf(j, "  ],\n  \"frames\": [\n");
    for (size_t i = 0; i < lbr_frame_count(); i++) {
        const lbr_frame_def_t *d = lbr_frame_at(i);
        jf(j, "    {\"id\": %u, \"name\": \"%s\", \"sender\": %u, \"origin\": %u, ", d->can_id,
           d->name, d->sender, d->origin);
        jf(j, "\"rate_hz\": %u, \"dlc\": %u, \"counter_offset\": ", d->rate_hz, d->dlc);
        if (d->counter_offset == LBR_FRAME_NO_COUNTER) {
            jf(j, "null");
        } else {
            jf(j, "%u", d->counter_offset);
        }
        jf(j, ", \"fields\": [");
        for (size_t f = 0; f < d->n_fields; f++) {
            const lbr_field_def_t *fd = &d->fields[f];
            jf(j, "%s{\"signal\": %u, \"name\": \"%s\", \"offset\": %u, \"enc\": \"%s\", ",
               f ? ", " : "", (unsigned)fd->signal, lbr_signal_info(fd->signal)->name, fd->offset,
               lbr_field_enc_name((lbr_field_enc_t)fd->enc));
            jf(j, "\"scale\": %.9g}", (double)fd->scale);
        }
        jf(j, "]}%s\n", i + 1 < lbr_frame_count() ? "," : "");
    }
    jf(j, "  ],\n  \"can_ids\": {\"time_sync\": %u, \"time_follow_up\": %u, \"command\": %u, ",
       LBR_CAN_ID_TIME_SYNC, LBR_CAN_ID_TIME_FOLLOW_UP, LBR_CAN_ID_COMMAND);
    jf(j, "\"event_base\": %u, \"heartbeat_base\": %u, \"health_base\": %u, ",
       LBR_CAN_ID_EVENT_BASE, LBR_CAN_ID_HEARTBEAT_BASE, LBR_CAN_ID_HEALTH_BASE);
    jf(j, "\"time_sync_hz\": %u, \"bitrate\": 500000},\n", LBR_TIME_SYNC_HZ);
    jf(j, "  \"encodings\": [");
    for (unsigned e = 0; e < LBR_ENC_COUNT_; e++) {
        jf(j, "%s\"%s\"", e ? ", " : "", lbr_field_enc_name((lbr_field_enc_t)e));
    }
    jf(j, "],\n  \"heartbeat_err_flags\": {\"NOT_SYNCED\": %u, \"SENSOR\": %u, \"CAN\": %u, ",
       LBR_HB_ERR_NOT_SYNCED, LBR_HB_ERR_SENSOR, LBR_HB_ERR_CAN);
    jf(j, "\"CONFIG\": %u, \"STORAGE\": %u, \"SIM\": %u, \"LOSS\": %u, \"LINK\": %u},\n",
       LBR_HB_ERR_CONFIG, LBR_HB_ERR_STORAGE, LBR_HB_ERR_SIM, LBR_HB_ERR_LOSS, LBR_HB_ERR_LINK);
    jf(j, "  \"commands\": [{\"code\": %u, \"name\": \"SESSION_START\"}, ", LBR_CMD_SESSION_START);
    jf(j, "{\"code\": %u, \"name\": \"SESSION_STOP\"}, {\"code\": %u, \"name\": \"MARK\"}],\n",
       LBR_CMD_SESSION_STOP, LBR_CMD_MARK);
    jf(j, "  \"nodes\": [\n");
    for (unsigned id = 1; id < 0xFF; id++) {
        const char *name = lbr_node_name((uint8_t)id);
        if (name[0] != '?') {
            jf(j, "%s    {\"id\": %u, \"name\": \"%s\"}", id > 1 ? ",\n" : "", id, name);
        }
    }
    jf(j, "\n  ],\n  \"units\": [");
    for (unsigned u = 0; u < LBR_UNIT_COUNT_; u++) {
        jf(j, "%s\"%s\"", u ? ", " : "", lbr_unit_name((lbr_unit_t)u));
    }
    jf(j, "],\n  \"value_types\": [");
    for (unsigned t = 0; t < LBR_VAL_COUNT_; t++) {
        jf(j, "%s\"%s\"", t ? ", " : "", lbr_value_type_name((lbr_value_type_t)t));
    }
    jf(j, "],\n  \"unavailable_reasons\": [");
    for (unsigned r = 0; r <= LBR_UNAVAIL_BUS_ERROR; r++) {
        jf(j, "%s\"%s\"", r ? ", " : "", lbr_unavail_reason_name((lbr_unavail_reason_t)r));
    }
    jf(j, "],\n  \"events\": [");
    int first = 1;
    for (unsigned c = 1; c < 0x100; c++) {
        const char *name = lbr_event_name((lbr_event_code_t)c);
        if (name[0] != '?') {
            jf(j, "%s{\"code\": %u, \"name\": \"%s\"}", first ? "" : ", ", c, name);
            first = 0;
        }
    }
    jf(j, "]\n}\n");
}

static void gen_samples(buf_t *bin, buf_t *j)
{
    lbr_sample_t s[16];
    size_t       n = sample_set(s);
    jf(j, "{\"record_size\": %u, \"samples\": [\n", LBR_SAMPLE_WIRE_SIZE);
    for (size_t i = 0; i < n; i++) {
        uint8_t w[LBR_SAMPLE_WIRE_SIZE];
        lbr_sample_encode(&s[i], w, sizeof w);
        buf_put(bin, w, sizeof w);
        jf(j, "  ");
        json_sample(j, &s[i]);
        jf(j, "%s\n", i + 1 < n ? "," : "");
    }
    jf(j, "]}\n");
}

static void gen_health(buf_t *bin, buf_t *j)
{
    lbr_health_record_t h = {
        .node_id               = LBR_NODE_POWERTRAIN,
        .state                 = LBR_NODE_DEGRADED,
        .flags                 = LBR_HF_TIME_SYNCED | LBR_HF_CAN_UP | LBR_HF_CONFIG_VALID,
        .fw_version            = LBR_FW_VERSION,
        .protocol_version      = LBR_PROTOCOL_VERSION,
        .boot_count            = 17,
        .catalog_hash          = 0x01020304,
        .config_crc            = 0xA0B0C0D0,
        .uptime_ms             = 0xFFFFFF00u,
        .reset_reason          = LBR_RESET_WATCHDOG,
        .acquire_hz            = 200,
        .publish_hz            = 200,
        .log_hz                = 0,
        .queue_high_water      = 1250,
        .overflow_count        = 3,
        .rejected_count        = 1,
        .write_error_count     = 0,
        .transport_error_count = 2,
        .can_tec               = 96,
        .can_rec               = 0,
        .can_bus_off_count     = 1,
        .can_tx_dropped        = 12,
        .can_rx_dropped        = 0,
        .can_bus_errors        = 40,
        .sensor_status         = 0x0,
        .calibration_state     = 0x1,
        .last_sample_age_ms    = 4,
        .last_sd_write_age_ms  = LBR_AGE_NONE,
        .time_step_count       = 1,
        .last_sync_error_us    = -17,
    };
    uint8_t w[LBR_HEALTH_WIRE_SIZE];
    lbr_health_encode(&h, w, sizeof w);
    buf_put(bin, w, sizeof w);
    jf(j, "{\"record_size\": %u, \"health\": ", LBR_HEALTH_WIRE_SIZE);
    json_health(j, &h);
    jf(j, "}\n");
}

static lbr_sample_t fv(lbr_signal_id_t sig, float v)
{
    lbr_sample_t s = {.signal = sig, .type = LBR_VAL_F32, .valid = true};
    s.v.f32        = v;
    return s;
}

static lbr_sample_t uv(lbr_signal_id_t sig, uint32_t v)
{
    const lbr_signal_info_t *info = lbr_signal_info(sig);
    lbr_sample_t             s    = {.signal = sig, .type = info->type, .valid = true};
    s.v.u32                       = v;
    return s;
}

/* The bus traffic every capture-based fixture uses: one frame of each kind,
 * as the Central would log it. `t` is the receive time of each. Returns the
 * count. */
static size_t bus_set(lbr_can_frame_t *f, uint64_t *t, uint8_t *flags)
{
    /* Just before a 24-bit wrap of the event time stamp. (Counter wraps are
     * covered by the unit tests.) */
    const uint64_t base = (1ull << 24) * 3 - 500;
    size_t         n    = 0;
    lbr_sample_t   v[8];

    /* Heartbeats first: they say which clocks are synced. */
    lbr_can_encode_heartbeat(LBR_NODE_DRIVER_INPUT, LBR_NODE_RUN, 0, &f[n]);
    t[n] = base, flags[n++] = 0;
    lbr_can_encode_heartbeat(LBR_NODE_IMU, LBR_NODE_DEGRADED, LBR_HB_ERR_LINK, &f[n]);
    t[n] = base + 100, flags[n++] = 0;
    lbr_can_encode_heartbeat(LBR_NODE_STRESS, LBR_NODE_DEGRADED,
                             LBR_HB_ERR_NOT_SYNCED | LBR_HB_ERR_SENSOR, &f[n]);
    t[n] = base + 200, flags[n++] = 0;

    uint64_t k = (base + 600) / 10000u; /* 100 Hz grid point */
    v[0]       = fv(LBR_SIG_BRAKE_PEDAL, 12.5f);
    v[1]       = fv(LBR_SIG_GAS_PEDAL, 87.25f);
    v[2]       = fv(LBR_SIG_STEERING_ANGLE, -31.4f);
    lbr_can_encode_data(lbr_frame_def(0x100), v, k, &f[n]);
    t[n] = k * 10000u + 450, flags[n++] = 0;

    v[0] = fv(LBR_SIG_ENGINE_RPM, 3620.0f);
    v[1] = fv(LBR_SIG_CVT_RPM, 0.0f);
    lbr_can_encode_data(lbr_frame_def(0x200), v, k - 1, &f[n]);
    t[n] = k * 10000u + 600, flags[n++] = 0;

    v[0] = fv(LBR_SIG_FUEL_LEVEL, 63.4f);
    lbr_can_encode_data(lbr_frame_def(0x201), v, k / 100u, &f[n]);
    t[n] = k * 10000u + 800, flags[n++] = 0;

    /* A forwarded corner sample whose BR gyro Z did not fit its field. */
    v[0] = fv(LBR_SIG_IMU_FL_ACCEL_X, 1.25f);
    v[1] = fv(LBR_SIG_IMU_FL_ACCEL_Y, -0.5f);
    v[2] = fv(LBR_SIG_IMU_FL_ACCEL_Z, 9.81f);
    lbr_can_encode_data(lbr_frame_def(0x300), v, k - 3, &f[n]);
    t[n] = k * 10000u + 900, flags[n++] = 0;
    v[0] = fv(LBR_SIG_IMU_BR_GYRO_X, 0.125f);
    v[1] = fv(LBR_SIG_IMU_BR_GYRO_Y, -1.0f);
    v[2] = fv(LBR_SIG_IMU_BR_GYRO_Z, 50.0f);
    lbr_can_encode_data(lbr_frame_def(0x307), v, k - 3, &f[n]);
    t[n] = k * 10000u + 1000, flags[n++] = 0;

    /* Corner status: FL synced, BL down. */
    v[0] = fv(LBR_SIG_CORNER_FL_BATTERY, 81.0f);
    v[1] = fv(LBR_SIG_CORNER_FR_BATTERY, 77.0f);
    lbr_sample_set_unavailable(&v[2], LBR_UNAVAIL_NO_DATA);
    v[2].signal = LBR_SIG_CORNER_BL_BATTERY;
    v[2].type   = LBR_VAL_F32;
    v[3]        = fv(LBR_SIG_CORNER_BR_BATTERY, 79.0f);
    v[4]        = uv(LBR_SIG_CORNER_LINK, 0x0B);
    v[5]        = uv(LBR_SIG_CORNER_SYNCED, 0x01);
    lbr_can_encode_data(lbr_frame_def(0x30A), v, k / 100u, &f[n]);
    t[n] = k * 10000u + 1200, flags[n++] = 0;
    v[0] = fv(LBR_SIG_IMU_FL_ACCEL_X, 1.5f);
    v[1] = fv(LBR_SIG_IMU_FL_ACCEL_Y, -0.25f);
    v[2] = fv(LBR_SIG_IMU_FL_ACCEL_Z, 9.5f);
    lbr_can_encode_data(lbr_frame_def(0x300), v, k - 2, &f[n]);
    t[n] = k * 10000u + 1300, flags[n++] = 0;

    /* Stress (no counter, unsynced node) and temperatures. */
    v[0] = fv(LBR_SIG_STRESS_1, 412.0f);
    v[1] = fv(LBR_SIG_STRESS_2, -37.0f);
    lbr_sample_set_unavailable(&v[2], LBR_UNAVAIL_SENSOR_FAULT);
    v[2].signal = LBR_SIG_STRESS_3;
    v[2].type   = LBR_VAL_F32;
    v[3]        = fv(LBR_SIG_STRESS_4, 32767.0f);
    lbr_can_encode_data(lbr_frame_def(0x400), v, k, &f[n]);
    t[n] = k * 10000u + 1500, flags[n++] = 0;
    v[0] = fv(LBR_SIG_TEMP_BRAKE_FL, 212.3f);
    v[1] = fv(LBR_SIG_TEMP_BRAKE_FR, 208.7f);
    v[2] = fv(LBR_SIG_TEMP_BRAKE_BL, 151.0f);
    v[3] = fv(LBR_SIG_TEMP_BRAKE_BR, -12.5f);
    lbr_can_encode_data(lbr_frame_def(0x500), v, k / 10u, &f[n]);
    t[n] = k * 10000u + 1700, flags[n++] = 0;

    /* Central: GPS (always synced) and the dashboard link. */
    v[0]       = uv(LBR_SIG_GPS_LATITUDE, 0);
    v[0].v.i32 = 302849870;
    v[1]       = uv(LBR_SIG_GPS_LONGITUDE, 0);
    v[1].v.i32 = -977366810;
    lbr_can_encode_data(lbr_frame_def(0x600), v, k / 10u, &f[n]);
    t[n] = k * 10000u + 2000, flags[n++] = LBR_CAN_REC_FLAG_OWN_TX;
    v[0] = fv(LBR_SIG_GPS_SPEED, 12.34f);
    v[1] = fv(LBR_SIG_GPS_COURSE, 271.5f);
    v[2] = fv(LBR_SIG_GPS_ALTITUDE, 149.2f);
    lbr_can_encode_data(lbr_frame_def(0x601), v, k / 10u, &f[n]);
    t[n] = k * 10000u + 2100, flags[n++] = LBR_CAN_REC_FLAG_OWN_TX;
    v[0] = uv(LBR_SIG_GPS_UTC, 1790000000u);
    v[1] = uv(LBR_SIG_GPS_FIX, 3);
    v[2] = uv(LBR_SIG_GPS_SATELLITES, 11);
    v[3] = fv(LBR_SIG_GPS_HDOP, 0.9f);
    lbr_can_encode_data(lbr_frame_def(0x602), v, k / 10u, &f[n]);
    t[n] = k * 10000u + 2200, flags[n++] = LBR_CAN_REC_FLAG_OWN_TX;
    v[0] = fv(LBR_SIG_DASH_BATTERY, 66.0f);
    v[1] = uv(LBR_SIG_DASH_LINK, 1);
    lbr_can_encode_data(lbr_frame_def(0x60A), v, k / 100u, &f[n]);
    t[n] = k * 10000u + 2300, flags[n++] = LBR_CAN_REC_FLAG_OWN_TX;

    /* An event straddling the 24-bit wrap, time sync, a command, a health page. */
    lbr_sample_t e = mk(LBR_SIG_FAULT, LBR_NODE_TEMPERATURE, 70, base + 400);
    e.v.u32        = lbr_event_value(LBR_EVT_SENSOR_INIT_FAIL, 1);
    lbr_can_encode_event(&e, &f[n]);
    t[n] = base + 900, flags[n++] = 0;
    lbr_can_encode_time_sync(9, 0xBEEF, base + 5000, &f[n]);
    t[n] = base + 5000, flags[n++] = LBR_CAN_REC_FLAG_OWN_TX;
    lbr_can_encode_time_follow_up(9, base + 5000 - 130, &f[n]);
    t[n] = base + 5200, flags[n++] = LBR_CAN_REC_FLAG_OWN_TX;
    uint8_t args[7] = {42, 0, 0, 0, 0, 0, 0};
    lbr_can_encode_command(LBR_CMD_SESSION_START, args, &f[n]);
    t[n] = base + 5300, flags[n++] = LBR_CAN_REC_FLAG_OWN_TX;
    lbr_health_record_t h = {.uptime_ms = 61000, .state = LBR_NODE_RUN, .flags = LBR_HF_CAN_UP};
    lbr_can_encode_health_page(LBR_NODE_POWERTRAIN, &h, 0, &f[n]);
    t[n] = base + 6000, flags[n++] = 0;

    /* Not LBR traffic: an extended-id frame from some other device. */
    f[n] = (lbr_can_frame_t){.id = 0x18FEF100, .dlc = 3, .data = {1, 2, 3}};
    t[n] = base + 7000, flags[n++] = LBR_CAN_REC_FLAG_EXT;
    return n;
}

#define BUS_SET_MAX 32

/* A short bus capture as the Central records it: one 22-byte record per
 * frame. */
static void gen_can(buf_t *bin, buf_t *j)
{
    lbr_can_frame_t   f[BUS_SET_MAX];
    uint64_t          t[BUS_SET_MAX];
    uint8_t           fl[BUS_SET_MAX];
    size_t            n = bus_set(f, t, fl);
    lbr_bus_decoder_t dec;
    lbr_bus_decoder_init(&dec);
    jf(j, "{\"record_size\": %u, \"frames\": [\n", LBR_CAN_FRAME_REC_SIZE);
    for (size_t i = 0; i < n; i++) {
        uint8_t rec[LBR_CAN_FRAME_REC_SIZE];
        lbr_can_frame_rec_encode(t[i], fl[i], &f[i], rec, sizeof rec);
        buf_put(bin, rec, sizeof rec);
        jf(j, "%s  ", i ? ",\n" : "");
        json_can_frame(j, &dec, t[i], fl[i], &f[i]);
    }
    jf(j, "\n]}\n");
}

/* In-memory storage for the session log fixture. */
static int mem_write(void *ctx, const uint8_t *p, size_t n)
{
    buf_put(ctx, p, n);
    return 0;
}

static int mem_ok(void *ctx)
{
    (void)ctx;
    return 0;
}

static void json_log_record(buf_t *j, lbr_bus_decoder_t *dec, const lbr_log_record_t *r)
{
    static const char *const names[] = {
        [LBR_REC_SESSION] = "SESSION", [LBR_REC_SAMPLE] = "SAMPLE",
        [LBR_REC_HEALTH] = "HEALTH",   [LBR_REC_CAN_FRAME] = "CAN_FRAME",
        [LBR_REC_LOSS] = "LOSS",       [LBR_REC_TEXT] = "TEXT",
        [LBR_REC_END] = "END",
    };
    const char *name = r->type <= LBR_REC_END && names[r->type] ? names[r->type] : "?";
    jf(j, "{\"offset\": %lu, \"type\": \"%s\"", (unsigned long)r->offset, name);
    const uint8_t *p = r->payload;
    switch (r->type) {
        case LBR_REC_SESSION:
            jf(j, ", \"format_version\": %u, \"protocol_version\": %u, \"fw_version\": %lu, ",
               lbr_get_u16(p), lbr_get_u16(p + 2), (unsigned long)lbr_get_u32(p + 4));
            jf(j, "\"catalog_hash\": %lu, \"config_crc\": %lu, \"node_id\": %u, ",
               (unsigned long)lbr_get_u32(p + 8), (unsigned long)lbr_get_u32(p + 12), p[16]);
            jf(j, "\"reset_reason\": %u, \"boot_count\": %lu, \"start_time_us\": %llu, ", p[17],
               (unsigned long)lbr_get_u32(p + 20), (unsigned long long)lbr_get_u64(p + 24));
            jf(j, "\"config_text\": ");
            json_str(j, (const char *)p + LBR_LOG_SESSION_FIXED, r->len - LBR_LOG_SESSION_FIXED);
            break;
        case LBR_REC_SAMPLE: {
            lbr_sample_t s;
            lbr_sample_decode(p, r->len, &s);
            jf(j, ", \"sample\": ");
            json_sample(j, &s);
            break;
        }
        case LBR_REC_HEALTH: {
            lbr_health_record_t h;
            lbr_health_decode(p, r->len, &h);
            jf(j, ", \"health\": ");
            json_health(j, &h);
            break;
        }
        case LBR_REC_CAN_FRAME: {
            uint64_t        t;
            uint8_t         flags;
            lbr_can_frame_t f;
            lbr_can_frame_rec_decode(p, r->len, &t, &flags, &f);
            jf(j, ", \"frame\": ");
            json_can_frame(j, dec, t, flags, &f);
            break;
        }
        case LBR_REC_LOSS:
            jf(j, ", \"dropped_records\": %lu, \"dropped_bytes\": %lu, \"t_us\": %llu",
               (unsigned long)lbr_get_u32(p), (unsigned long)lbr_get_u32(p + 4),
               (unsigned long long)lbr_get_u64(p + 8));
            break;
        case LBR_REC_TEXT:
            jf(j, ", \"text\": ");
            json_str(j, (const char *)p, r->len);
            break;
        case LBR_REC_END:
            jf(j, ", \"t_us\": %llu, \"records_written\": %lu", (unsigned long long)lbr_get_u64(p),
               (unsigned long)lbr_get_u32(p + 8));
            break;
        default:
            break;
    }
    jf(j, "}");
}

static void gen_session_log(buf_t *bin, buf_t *j)
{
    static uint8_t   queue[4096];
    static uint8_t   chunk[LBR_LOG_MAX_RECORD + 64];
    lbr_logger_t     lg;
    lbr_log_config_t cfg = {queue, sizeof queue, chunk, sizeof chunk, 1000, NULL, NULL, NULL};
    lbr_log_init(&lg, &cfg);
    lbr_log_storage_t st      = {mem_write, mem_ok, mem_ok, bin};
    lbr_log_session_t session = {
        .fw_version    = LBR_FW_VERSION,
        .node_id       = LBR_NODE_CENTRAL,
        .reset_reason  = LBR_RESET_BROWNOUT,
        .boot_count    = 42,
        .start_time_us = 1500000,
        .config_text   = "node=central\nnode_id=6\ncan_bitrate=500000\n",
    };
    lbr_log_open(&lg, &st, &session);

    lbr_sample_t s[16];
    size_t       n = sample_set(s);
    for (size_t i = 0; i < 3; i++) {
        lbr_log_append_sample(&lg, &s[i]);
    }
    lbr_can_frame_t bf[BUS_SET_MAX];
    uint64_t        bt[BUS_SET_MAX];
    uint8_t         bfl[BUS_SET_MAX];
    size_t          bn = bus_set(bf, bt, bfl);
    for (size_t i = 0; i < bn && i < 5; i++) {
        lbr_log_append_can_frame(&lg, bt[i], bfl[i], &bf[i]);
    }
    lbr_health_record_t h = {.node_id   = LBR_NODE_CENTRAL,
                             .state     = LBR_NODE_RUN,
                             .flags     = LBR_HF_TIME_MASTER | LBR_HF_SD_LOGGING,
                             .uptime_ms = 2000};
    lbr_log_append_health(&lg, &h);
    uint8_t loss[LBR_LOG_LOSS_SIZE];
    lbr_put_u32(loss, 5);
    lbr_put_u32(loss + 4, 150);
    lbr_put_u64(loss + 8, 2500000);
    lbr_log_append_raw(&lg, LBR_REC_LOSS, loss, sizeof loss);
    lbr_log_append_text(&lg, "I (2500) central: fixture \"text\" record");
    for (size_t i = 3; i < n; i++) {
        lbr_log_append_sample(&lg, &s[i]);
    }
    lbr_log_close(&lg, 9000000);

    /* Power loss mid-record: append the first 9 bytes of another record. */
    uint8_t tail[64];
    size_t  tn = lbr_log_frame(LBR_REC_TEXT, (const uint8_t *)"lost", 4, tail, sizeof tail);
    (void)tn;
    buf_put(bin, tail, 9);

    lbr_log_reader_t  r;
    lbr_log_record_t  rec;
    lbr_bus_decoder_t dec;
    lbr_bus_decoder_init(&dec);
    lbr_log_reader_init(&r, bin->data, bin->len);
    jf(j, "{\"records\": [\n");
    size_t count = 0;
    while (lbr_log_reader_next(&r, &rec)) {
        jf(j, "%s  ", count++ ? ",\n" : "");
        json_log_record(j, &dec, &rec);
    }
    jf(j, "\n], \"truncated_tail\": %lu, \"corrupt_regions\": %lu, \"skipped_bytes\": %lu}\n",
       (unsigned long)r.truncated_tail, (unsigned long)r.corrupt_regions,
       (unsigned long)r.skipped_bytes);
}

typedef struct {
    buf_t             *j;
    int                n;
    lbr_bus_decoder_t *dec;
} pkt_ctx_t;

static void pkt_rec(uint8_t type, const uint8_t *p, size_t len, void *vctx)
{
    pkt_ctx_t *c = vctx;
    jf(c->j, "%s      {\"type\": %u, \"len\": %lu", c->n++ ? ",\n" : "", type, (unsigned long)len);
    if (type == LBR_REC_CAN_FRAME) {
        uint64_t        t;
        uint8_t         flags;
        lbr_can_frame_t f;
        lbr_can_frame_rec_decode(p, len, &t, &flags, &f);
        jf(c->j, ", \"frame\": ");
        json_can_frame(c->j, c->dec, t, flags, &f);
    } else if (type == LBR_REC_TEXT) {
        jf(c->j, ", \"text\": ");
        json_str(c->j, (const char *)p, len);
    }
    jf(c->j, "}");
}

/* A USB/WiFi byte stream: boot-ROM noise, then two COBS-framed packets. */
static void gen_stream(buf_t *bin, buf_t *j)
{
    const char noise[] = "ets Jun  8 2016 00:22:57\r\nrst:0x1 (POWERON_RESET)\r\n";
    buf_put(bin, noise, sizeof noise - 1);
    buf_put(bin, "\0", 1);

    lbr_can_frame_t   bf[BUS_SET_MAX];
    uint64_t          bt[BUS_SET_MAX];
    uint8_t           bfl[BUS_SET_MAX];
    size_t            bn = bus_set(bf, bt, bfl);
    lbr_bus_decoder_t dec;
    lbr_bus_decoder_init(&dec);
    jf(j, "{\"packets\": [\n");
    for (uint32_t k = 0; k < 2; k++) {
        uint8_t           pkt[LBR_PKT_MAX_SIZE];
        lbr_pkt_builder_t b;
        lbr_pkt_begin(&b, pkt, sizeof pkt, LBR_LINK_USB, LBR_NODE_CENTRAL, 0, 7000000 + k);
        if (k == 0) {
            for (size_t i = 0; i < 6 && i < bn; i++) {
                uint8_t rec[LBR_CAN_FRAME_REC_SIZE];
                lbr_can_frame_rec_encode(bt[i], bfl[i], &bf[i], rec, sizeof rec);
                lbr_pkt_add(&b, LBR_REC_CAN_FRAME, rec, sizeof rec);
            }
        } else {
            const char *t = "I (7000) central: stream fixture";
            lbr_pkt_add(&b, LBR_REC_TEXT, (const uint8_t *)t, strlen(t));
        }
        size_t len = lbr_pkt_finish(&b);
        lbr_pkt_restamp(pkt, len, LBR_LINK_USB, 0xFFFFFFFFu + k); /* seq wraps */
        uint8_t enc[LBR_COBS_MAX_ENCODED(LBR_PKT_MAX_SIZE)];
        size_t  e = lbr_cobs_encode(pkt, len, enc, sizeof enc);
        buf_put(bin, enc, e);
        buf_put(bin, "\0", 1);

        lbr_pkt_header_t h;
        pkt_ctx_t        ctx = {j, 0, &dec};
        jf(j, "%s  {\"link\": %u, \"source\": %u, \"seq\": %lu, \"t_us\": %llu, \"records\": [\n",
           k ? ",\n" : "", LBR_LINK_USB, LBR_NODE_CENTRAL, (unsigned long)(0xFFFFFFFFu + k),
           (unsigned long long)(7000000 + k));
        lbr_pkt_parse(pkt, len, &h, pkt_rec, &ctx);
        jf(j, "\n  ]}");
    }
    jf(j, "\n], \"bad_frames\": 1}\n");
}

/* ------------------------------------------------------------------------ */

typedef struct {
    const char *name;
    void (*gen)(buf_t *bin, buf_t *json);
    const char *bin_name;
} fixture_t;

static const fixture_t k_fixtures[] = {
    {"catalog.json", NULL, NULL},
    {"samples.json", gen_samples, "samples.bin"},
    {"health.json", gen_health, "health.bin"},
    {"can_capture.json", gen_can, "can_capture.bin"},
    {"session.json", gen_session_log, "session.lbr"},
    {"stream_usb.json", gen_stream, "stream_usb.bin"},
};

static int emit(const char *dir, const char *name, const buf_t *b, int write)
{
    char path[512];
    snprintf(path, sizeof path, "%s/%s", dir, name);
    if (write) {
        FILE *f = fopen(path, "wb");
        if (f == NULL || fwrite(b->data, 1, b->len, f) != b->len) {
            fprintf(stderr, "cannot write %s\n", path);
            return 1;
        }
        fclose(f);
        printf("wrote %s (%lu bytes)\n", path, (unsigned long)b->len);
        return 0;
    }
    FILE *f = fopen(path, "rb");
    if (f == NULL) {
        fprintf(stderr, "missing fixture %s\n", path);
        return 1;
    }
    buf_t  have = {0};
    char   tmp[4096];
    size_t n;
    while ((n = fread(tmp, 1, sizeof tmp, f)) > 0) {
        buf_put(&have, tmp, n);
    }
    fclose(f);
    int same = have.len == b->len && memcmp(have.data, b->data, b->len) == 0;
    free(have.data);
    if (!same) {
        fprintf(stderr,
                "fixture %s no longer matches the codec. If the format change is intended,\n"
                "bump the version, regenerate (cmake --build build-dev --target fixtures),\n"
                "and update lbr-dashboard in the same change.\n",
                path);
        return 1;
    }
    printf("PASS %s\n", name);
    return 0;
}

int main(int argc, char **argv)
{
    if (argc != 3 || (strcmp(argv[1], "write") != 0 && strcmp(argv[1], "check") != 0)) {
        fprintf(stderr, "usage: %s write|check <fixtures-dir>\n", argv[0]);
        return 2;
    }
    int write = strcmp(argv[1], "write") == 0;
    int fails = 0;
    for (size_t i = 0; i < sizeof k_fixtures / sizeof k_fixtures[0]; i++) {
        buf_t bin = {0}, json = {0};
        if (k_fixtures[i].gen == NULL) {
            gen_catalog(&json);
        } else {
            k_fixtures[i].gen(&bin, &json);
            fails += emit(argv[2], k_fixtures[i].bin_name, &bin, write);
        }
        fails += emit(argv[2], k_fixtures[i].name, &json, write);
        free(bin.data);
        free(json.data);
    }
    return fails == 0 ? 0 : 1;
}
