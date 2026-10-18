#include "lbr/sensor_io.h"

#include <ctype.h>
#include <math.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Pure side of the part-independent inputs: spec parsing and conversions.
 * No ESP-IDF headers here; sensor_io_esp.c owns the pins. */

#define CHAN_TEXT_MAX 160u
#define TOKEN_MAX     64u

static int fail(char *err, size_t err_len, const char *fmt, ...)
{
    if (err != NULL && err_len > 0) {
        va_list ap;
        va_start(ap, fmt);
        vsnprintf(err, err_len, fmt, ap);
        va_end(ap);
    }
    return -1;
}

static bool parse_float(const char *s, float *out)
{
    char *end = NULL;
    float v   = strtof(s, &end);
    if (end == s || *end != '\0' || !isfinite(v)) {
        return false;
    }
    *out = v;
    return true;
}

static bool parse_uint(const char *s, uint32_t lo, uint32_t hi, uint32_t *out)
{
    char         *end = NULL;
    unsigned long v   = strtoul(s, &end, 10);
    if (end == s || *end != '\0' || s[0] == '-' || v < lo || v > hi) {
        return false;
    }
    *out = (uint32_t)v;
    return true;
}

/* ESP32 (classic): GPIO 6..11 belong to the SPI flash, 20/24/28..31 do not
 * exist, and 34..39 are input-only without internal pulls. */
static bool gpio_exists(uint32_t g)
{
    return g <= 39u && !(g >= 6u && g <= 11u) && g != 20u && g != 24u && !(g >= 28u && g <= 31u);
}

static bool gpio_has_pulls(uint32_t g)
{
    return g < 34u;
}

static bool parse_gpio(const char *s, int8_t *out)
{
    uint32_t g;
    if (!parse_uint(s, 0, 39, &g) || !gpio_exists(g)) {
        return false;
    }
    *out = (int8_t)g;
    return true;
}

static bool parse_pull(const char *s, lbr_io_pull_t *out)
{
    if (strcmp(s, "none") == 0) {
        *out = LBR_IO_PULL_NONE;
    } else if (strcmp(s, "up") == 0) {
        *out = LBR_IO_PULL_UP;
    } else if (strcmp(s, "down") == 0) {
        *out = LBR_IO_PULL_DOWN;
    } else {
        return false;
    }
    return true;
}

/* Keys seen in one channel, to catch duplicates and missing required keys. */
enum {
    K_GPIO     = 1u << 0,
    K_AVG      = 1u << 1,
    K_CURVE    = 1u << 2,
    K_MV0      = 1u << 3,
    K_V0       = 1u << 4,
    K_MV1      = 1u << 5,
    K_V1       = 1u << 6,
    K_R25      = 1u << 7,
    K_BETA     = 1u << 8,
    K_RPULL    = 1u << 9,
    K_VSUP     = 1u << 10,
    K_LO       = 1u << 11,
    K_HI       = 1u << 12,
    K_MIN      = 1u << 13,
    K_MAX      = 1u << 14,
    K_SCALE    = 1u << 15,
    K_STALL    = 1u << 16,
    K_EDGE     = 1u << 17,
    K_PULL     = 1u << 18,
    K_ACTIVE   = 1u << 19,
    K_DEBOUNCE = 1u << 20,
};

typedef struct {
    const char *name;
    uint32_t    bit;
    uint8_t     kinds; /* bit per lbr_io_kind_t */
} key_def_t;

#define KIND_BIT(k) (uint8_t)(1u << (k))

static const key_def_t k_keys[] = {
    {"gpio", K_GPIO, KIND_BIT(LBR_IO_ADC) | KIND_BIT(LBR_IO_PULSE) | KIND_BIT(LBR_IO_DIN)},
    {"avg", K_AVG, KIND_BIT(LBR_IO_ADC)},
    {"curve", K_CURVE, KIND_BIT(LBR_IO_ADC)},
    {"mv0", K_MV0, KIND_BIT(LBR_IO_ADC)},
    {"v0", K_V0, KIND_BIT(LBR_IO_ADC)},
    {"mv1", K_MV1, KIND_BIT(LBR_IO_ADC)},
    {"v1", K_V1, KIND_BIT(LBR_IO_ADC)},
    {"r25", K_R25, KIND_BIT(LBR_IO_ADC)},
    {"beta", K_BETA, KIND_BIT(LBR_IO_ADC)},
    {"rpull", K_RPULL, KIND_BIT(LBR_IO_ADC)},
    {"vsup", K_VSUP, KIND_BIT(LBR_IO_ADC)},
    {"lo", K_LO, KIND_BIT(LBR_IO_ADC)},
    {"hi", K_HI, KIND_BIT(LBR_IO_ADC)},
    {"min", K_MIN, KIND_BIT(LBR_IO_ADC)},
    {"max", K_MAX, KIND_BIT(LBR_IO_ADC)},
    {"scale", K_SCALE, KIND_BIT(LBR_IO_PULSE)},
    {"stall_ms", K_STALL, KIND_BIT(LBR_IO_PULSE)},
    {"edge", K_EDGE, KIND_BIT(LBR_IO_PULSE)},
    {"pull", K_PULL, KIND_BIT(LBR_IO_PULSE) | KIND_BIT(LBR_IO_DIN)},
    {"active", K_ACTIVE, KIND_BIT(LBR_IO_DIN)},
    {"debounce_ms", K_DEBOUNCE, KIND_BIT(LBR_IO_DIN)},
};

static const key_def_t *find_key(const char *name)
{
    for (size_t i = 0; i < sizeof k_keys / sizeof k_keys[0]; i++) {
        if (strcmp(k_keys[i].name, name) == 0) {
            return &k_keys[i];
        }
    }
    return NULL;
}

static void chan_defaults(lbr_io_chan_t *c, lbr_io_kind_t kind)
{
    memset(c, 0, sizeof *c);
    c->kind = kind;
    switch (kind) {
        case LBR_IO_ADC:
            c->u.adc.gpio  = -1;
            c->u.adc.avg   = 4;
            c->u.adc.lo_mv = -INFINITY;
            c->u.adc.hi_mv = INFINITY;
            c->u.adc.min   = -INFINITY;
            c->u.adc.max   = INFINITY;
            break;
        case LBR_IO_PULSE:
            c->u.pulse.gpio     = -1;
            c->u.pulse.stall_ms = 2000;
            c->u.pulse.pull     = LBR_IO_PULL_NONE;
            break;
        case LBR_IO_DIN:
            c->u.din.pull        = LBR_IO_PULL_UP;
            c->u.din.debounce_ms = 10;
            break;
        default:
            break;
    }
}

static bool parse_din_gpios(const char *v, lbr_io_chan_t *c)
{
    char buf[TOKEN_MAX];
    snprintf(buf, sizeof buf, "%s", v);
    c->u.din.n_gpio = 0;
    char *save      = NULL;
    for (char *p = strtok_r(buf, ",", &save); p != NULL; p = strtok_r(NULL, ",", &save)) {
        if (c->u.din.n_gpio >= LBR_IO_MAX_DIN_PINS ||
            !parse_gpio(p, &c->u.din.gpio[c->u.din.n_gpio])) {
            return false;
        }
        c->u.din.n_gpio++;
    }
    return c->u.din.n_gpio > 0;
}

/* Apply one key=value to the channel. Returns false if the value is bad. */
static bool apply_key(lbr_io_chan_t *c, uint32_t bit, const char *v)
{
    uint32_t u;
    switch (bit) {
        case K_GPIO:
            if (c->kind == LBR_IO_DIN) {
                return parse_din_gpios(v, c);
            }
            return parse_gpio(v, c->kind == LBR_IO_ADC ? &c->u.adc.gpio : &c->u.pulse.gpio);
        case K_AVG:
            if (!parse_uint(v, 1, 64, &u)) {
                return false;
            }
            c->u.adc.avg = (uint8_t)u;
            return true;
        case K_CURVE:
            if (strcmp(v, "lin") == 0) {
                c->u.adc.ntc = false;
            } else if (strcmp(v, "ntc") == 0) {
                c->u.adc.ntc = true;
            } else {
                return false;
            }
            return true;
        case K_MV0:
            return parse_float(v, &c->u.adc.mv0);
        case K_V0:
            return parse_float(v, &c->u.adc.v0);
        case K_MV1:
            return parse_float(v, &c->u.adc.mv1);
        case K_V1:
            return parse_float(v, &c->u.adc.v1);
        case K_R25:
            return parse_float(v, &c->u.adc.r25);
        case K_BETA:
            return parse_float(v, &c->u.adc.beta);
        case K_RPULL:
            return parse_float(v, &c->u.adc.rpull);
        case K_VSUP:
            return parse_float(v, &c->u.adc.vsup_mv);
        case K_LO:
            return parse_float(v, &c->u.adc.lo_mv);
        case K_HI:
            return parse_float(v, &c->u.adc.hi_mv);
        case K_MIN:
            return parse_float(v, &c->u.adc.min);
        case K_MAX:
            return parse_float(v, &c->u.adc.max);
        case K_SCALE:
            return parse_float(v, &c->u.pulse.scale);
        case K_STALL:
            return parse_uint(v, 1, 60000, &c->u.pulse.stall_ms);
        case K_EDGE:
            if (strcmp(v, "rise") == 0) {
                c->u.pulse.falling = false;
            } else if (strcmp(v, "fall") == 0) {
                c->u.pulse.falling = true;
            } else {
                return false;
            }
            return true;
        case K_PULL:
            return parse_pull(v, c->kind == LBR_IO_DIN ? &c->u.din.pull : &c->u.pulse.pull);
        case K_ACTIVE:
            if (strcmp(v, "low") == 0) {
                c->u.din.active_high = false;
            } else if (strcmp(v, "high") == 0) {
                c->u.din.active_high = true;
            } else {
                return false;
            }
            return true;
        case K_DEBOUNCE:
            return parse_uint(v, 0, 1000, &c->u.din.debounce_ms);
        default:
            return false;
    }
}

/* Cross-key checks once a channel is fully parsed. */
static int check_chan(const lbr_io_chan_t *c, uint32_t seen, lbr_value_type_t type, const char *sig,
                      char *err, size_t err_len)
{
    if ((seen & K_GPIO) == 0) {
        return fail(err, err_len, "%s: gpio missing", sig);
    }
    switch (c->kind) {
        case LBR_IO_ADC: {
            if (type != LBR_VAL_F32) {
                return fail(err, err_len, "%s: adc needs an f32 signal", sig);
            }
            if (c->u.adc.gpio < 32) {
                return fail(err, err_len, "%s: adc gpio %d is not an ADC1 pin (32..39)", sig,
                            (int)c->u.adc.gpio);
            }
            const uint32_t lin = K_MV0 | K_V0 | K_MV1 | K_V1;
            const uint32_t ntc = K_R25 | K_BETA | K_RPULL | K_VSUP;
            if (c->u.adc.ntc) {
                if ((seen & ntc) != ntc || (seen & lin) != 0) {
                    return fail(err, err_len, "%s: curve=ntc needs r25 beta rpull vsup only", sig);
                }
                if (c->u.adc.r25 <= 0.0f || c->u.adc.beta <= 0.0f || c->u.adc.rpull <= 0.0f ||
                    c->u.adc.vsup_mv <= 0.0f) {
                    return fail(err, err_len, "%s: ntc parameters must be positive", sig);
                }
            } else {
                if ((seen & lin) != lin || (seen & ntc) != 0) {
                    return fail(err, err_len, "%s: curve=lin needs mv0 v0 mv1 v1 only", sig);
                }
                if (c->u.adc.mv0 == c->u.adc.mv1) {
                    return fail(err, err_len, "%s: mv0 and mv1 must differ", sig);
                }
            }
            if (c->u.adc.lo_mv >= c->u.adc.hi_mv || c->u.adc.min >= c->u.adc.max) {
                return fail(err, err_len, "%s: empty lo..hi or min..max range", sig);
            }
            return 0;
        }
        case LBR_IO_PULSE:
            if (type != LBR_VAL_F32) {
                return fail(err, err_len, "%s: pulse needs an f32 signal", sig);
            }
            if ((seen & K_SCALE) == 0 || c->u.pulse.scale <= 0.0f) {
                return fail(err, err_len, "%s: pulse needs scale > 0", sig);
            }
            if (c->u.pulse.pull != LBR_IO_PULL_NONE && !gpio_has_pulls((uint32_t)c->u.pulse.gpio)) {
                return fail(err, err_len, "%s: gpio %d has no internal pulls", sig,
                            (int)c->u.pulse.gpio);
            }
            return 0;
        case LBR_IO_DIN:
            if (type == LBR_VAL_BOOL) {
                if (c->u.din.n_gpio != 1) {
                    return fail(err, err_len, "%s: a bool signal takes one gpio", sig);
                }
            } else if (type != LBR_VAL_U32) {
                return fail(err, err_len, "%s: din needs a bool or u32 signal", sig);
            }
            for (uint8_t i = 0; i < c->u.din.n_gpio; i++) {
                if (c->u.din.pull != LBR_IO_PULL_NONE &&
                    !gpio_has_pulls((uint32_t)c->u.din.gpio[i])) {
                    return fail(err, err_len, "%s: gpio %d has no internal pulls", sig,
                                (int)c->u.din.gpio[i]);
                }
            }
            return 0;
        default:
            return 0;
    }
}

static char *trim(char *s)
{
    while (isspace((unsigned char)*s)) {
        s++;
    }
    size_t n = strlen(s);
    while (n > 0 && isspace((unsigned char)s[n - 1])) {
        s[--n] = '\0';
    }
    return s;
}

static int parse_chan(char *text, lbr_signal_id_t signal, lbr_io_chan_t *c, char *err,
                      size_t err_len)
{
    const lbr_signal_info_t *info = lbr_signal_info(signal);
    const char              *sig  = info != NULL ? info->name : "?";
    if (info == NULL) {
        return fail(err, err_len, "signal 0x%04x not in catalog", (unsigned)signal);
    }
    text = trim(text);
    if (strcmp(text, "-") == 0) {
        chan_defaults(c, LBR_IO_NONE);
        return 0;
    }

    char *save = NULL;
    char *tok  = strtok_r(text, " \t", &save);
    if (tok == NULL) {
        return fail(err, err_len, "%s: empty channel (use '-' for not fitted)", sig);
    }
    if (strcmp(tok, "adc") == 0) {
        chan_defaults(c, LBR_IO_ADC);
    } else if (strcmp(tok, "pulse") == 0) {
        chan_defaults(c, LBR_IO_PULSE);
    } else if (strcmp(tok, "din") == 0) {
        chan_defaults(c, LBR_IO_DIN);
    } else {
        return fail(err, err_len, "%s: unknown input kind '%s'", sig, tok);
    }

    uint32_t seen = 0;
    while ((tok = strtok_r(NULL, " \t", &save)) != NULL) {
        char *eq = strchr(tok, '=');
        if (eq == NULL || eq == tok || eq[1] == '\0') {
            return fail(err, err_len, "%s: expected key=value, got '%s'", sig, tok);
        }
        *eq                 = '\0';
        const key_def_t *k  = find_key(tok);
        const char      *va = eq + 1;
        if (k == NULL || (k->kinds & KIND_BIT(c->kind)) == 0) {
            return fail(err, err_len, "%s: key '%s' not valid here", sig, tok);
        }
        if ((seen & k->bit) != 0) {
            return fail(err, err_len, "%s: key '%s' given twice", sig, tok);
        }
        if (!apply_key(c, k->bit, va)) {
            return fail(err, err_len, "%s: bad value for %s: '%s'", sig, tok, va);
        }
        seen |= k->bit;
    }
    return check_chan(c, seen, info->type, sig, err, err_len);
}

int lbr_io_parse(const char *spec, const lbr_signal_id_t *signals, size_t n, lbr_io_chan_t *chans,
                 char *err, size_t err_len)
{
    if (spec == NULL || spec[0] == '\0') {
        return fail(err, err_len, "empty input spec");
    }
    if (strlen(spec) >= LBR_IO_SPEC_MAX) {
        return fail(err, err_len, "input spec longer than %u bytes", LBR_IO_SPEC_MAX - 1u);
    }
    for (const char *p = spec; *p != '\0'; p++) {
        if ((unsigned char)*p > 0x7Eu || ((unsigned char)*p < 0x20u && *p != '\t')) {
            return fail(err, err_len, "input spec is not printable ASCII");
        }
    }

    size_t      idx   = 0;
    const char *start = spec;
    for (;;) {
        const char *end = strchr(start, ';');
        size_t      len = end != NULL ? (size_t)(end - start) : strlen(start);
        if (idx >= n) {
            return fail(err, err_len, "%u channels given, slot has %u", (unsigned)(idx + 1),
                        (unsigned)n);
        }
        if (len >= CHAN_TEXT_MAX) {
            return fail(err, err_len, "channel %u spec too long", (unsigned)idx);
        }
        char text[CHAN_TEXT_MAX];
        memcpy(text, start, len);
        text[len] = '\0';
        if (parse_chan(text, signals[idx], &chans[idx], err, err_len) != 0) {
            return -1;
        }
        idx++;
        if (end == NULL) {
            break;
        }
        start = end + 1;
    }
    if (idx != n) {
        return fail(err, err_len, "%u channels given, slot has %u", (unsigned)idx, (unsigned)n);
    }

    /* One pin, one function. */
    int8_t pins[LBR_SENSOR_MAX_SIGNALS * LBR_IO_MAX_DIN_PINS];
    size_t np = 0;
    for (size_t i = 0; i < n; i++) {
        const lbr_io_chan_t *c = &chans[i];
        if (c->kind == LBR_IO_ADC) {
            pins[np++] = c->u.adc.gpio;
        } else if (c->kind == LBR_IO_PULSE) {
            pins[np++] = c->u.pulse.gpio;
        } else if (c->kind == LBR_IO_DIN) {
            for (uint8_t k = 0; k < c->u.din.n_gpio; k++) {
                pins[np++] = c->u.din.gpio[k];
            }
        }
    }
    for (size_t i = 0; i < np; i++) {
        for (size_t j = i + 1; j < np; j++) {
            if (pins[i] == pins[j]) {
                return fail(err, err_len, "gpio %d used twice", (int)pins[i]);
            }
        }
    }
    return 0;
}

void lbr_io_adc_convert(const lbr_io_chan_t *c, float mv, lbr_reading_t *out)
{
    out->valid  = false;
    out->reason = LBR_UNAVAIL_SENSOR_FAULT;
    out->v.u32  = 0;
    if (!isfinite(mv) || mv < c->u.adc.lo_mv || mv > c->u.adc.hi_mv) {
        return; /* open, shorted, or miswired sensor */
    }
    float v;
    if (c->u.adc.ntc) {
        if (mv <= 0.0f || mv >= c->u.adc.vsup_mv) {
            return;
        }
        float r = c->u.adc.rpull * mv / (c->u.adc.vsup_mv - mv);
        v       = 1.0f / (1.0f / 298.15f + logf(r / c->u.adc.r25) / c->u.adc.beta) - 273.15f;
    } else {
        v = c->u.adc.v0 +
            (mv - c->u.adc.mv0) * (c->u.adc.v1 - c->u.adc.v0) / (c->u.adc.mv1 - c->u.adc.mv0);
    }
    if (!isfinite(v)) {
        return;
    }
    if (v < c->u.adc.min || v > c->u.adc.max) {
        out->reason = LBR_UNAVAIL_OUT_OF_RANGE;
        return;
    }
    out->valid  = true;
    out->reason = LBR_UNAVAIL_UNSPECIFIED;
    out->v.f32  = v;
}

float lbr_io_pulse_update(lbr_io_pulse_t *st, uint32_t count, uint64_t edge_us, uint64_t now_us,
                          uint32_t stall_ms)
{
    if (count != st->ref_count) {
        if (st->have_ref && edge_us > st->ref_edge_us) {
            st->hz = (float)(uint32_t)(count - st->ref_count) * 1e6f /
                     (float)(edge_us - st->ref_edge_us);
        }
        st->ref_count   = count;
        st->ref_edge_us = edge_us;
        st->have_ref    = true;
    }
    if (!st->have_ref) {
        return 0.0f;
    }
    uint64_t since = now_us > st->ref_edge_us ? now_us - st->ref_edge_us : 0;
    if (since >= (uint64_t)stall_ms * 1000u) {
        /* Stopped: the next edge starts a fresh measurement. */
        st->have_ref = false;
        st->hz       = 0.0f;
        return 0.0f;
    }
    /* No edge for longer than the last period: the true rate is at most one
     * edge per `since`, so decay rather than hold a stale value. */
    if (since > 0) {
        float bound = 1e6f / (float)since;
        if (bound < st->hz) {
            return bound;
        }
    }
    return st->hz;
}

uint32_t lbr_io_din_update(lbr_io_din_t *st, uint32_t raw, uint64_t now_us, uint32_t debounce_ms)
{
    if (!st->seeded) {
        st->stable   = raw;
        st->pending  = raw;
        st->since_us = now_us;
        st->seeded   = true;
        return raw;
    }
    if (raw != st->pending) {
        st->pending  = raw;
        st->since_us = now_us;
    }
    if (st->pending != st->stable && now_us - st->since_us >= (uint64_t)debounce_ms * 1000u) {
        st->stable = st->pending;
    }
    return st->stable;
}
