#include "lbr/gw_config.h"

#include <stdarg.h>
#include <stdio.h>
#include <string.h>

typedef enum { K_BOOL, K_U16 } kind_t;

typedef struct {
    const char *key;
    kind_t      kind;
    size_t      offset;
    uint32_t    min;
    uint32_t    max;
} field_t;

#define F_BOOL(name)        {#name, K_BOOL, offsetof(lbr_gw_config_t, name), 0, 1}
#define F_U16(name, lo, hi) {#name, K_U16, offsetof(lbr_gw_config_t, name), lo, hi}

static const field_t k_fields[] = {
    F_BOOL(usb_enable),
    F_BOOL(wifi_enable),
    F_BOOL(ble_enable),
    F_BOOL(lora_enable),
    F_U16(lora_period_ms, 200, 60000),
    F_U16(lora_duty_permille, 1, 1000),
    F_U16(stream_batch_ms, 10, 1000),
    F_U16(node_timeout_ms, 500, 30000),
    F_BOOL(dash_enable),
    F_U16(dash_period_ms, 50, 1000),
};
#define N_FIELDS (sizeof k_fields / sizeof k_fields[0])

void lbr_gw_config_defaults(lbr_gw_config_t *c)
{
    *c = (lbr_gw_config_t){
        .usb_enable         = true,
        .wifi_enable        = true,
        .ble_enable         = true,
        .lora_enable        = true,
        .lora_period_ms     = 1000,
        .lora_duty_permille = 100,
        .stream_batch_ms    = 50,
        .node_timeout_ms    = 3000,
        .dash_enable        = true,
        .dash_period_ms     = 100,
    };
}

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

static bool parse_u32(const char *s, size_t n, uint32_t *out)
{
    if (n == 0 || n > 9) {
        return false;
    }
    uint32_t v = 0;
    for (size_t i = 0; i < n; i++) {
        if (s[i] < '0' || s[i] > '9') {
            return false;
        }
        v = v * 10u + (uint32_t)(s[i] - '0');
    }
    *out = v;
    return true;
}

static size_t trim(const char **s, size_t n)
{
    while (n > 0 && (**s == ' ' || **s == '\t')) {
        (*s)++;
        n--;
    }
    while (n > 0 && ((*s)[n - 1] == ' ' || (*s)[n - 1] == '\t' || (*s)[n - 1] == '\r')) {
        n--;
    }
    return n;
}

int lbr_gw_config_parse(const char *text, size_t len, lbr_gw_config_t *out, char *err,
                        size_t err_len)
{
    lbr_gw_config_t c;
    lbr_gw_config_defaults(&c);
    unsigned line_no = 0;
    size_t   pos     = 0;

    while (pos < len) {
        const char *line = text + pos;
        size_t      n    = 0;
        while (pos + n < len && text[pos + n] != '\n') {
            n++;
        }
        pos += n + 1;
        line_no++;

        for (size_t i = 0; i < n; i++) {
            if (line[i] == '#') {
                n = i;
                break;
            }
            if ((unsigned char)line[i] < 0x09 || (unsigned char)line[i] > 0x7E) {
                return fail(err, err_len, "line %u: non-ASCII or control byte", line_no);
            }
        }
        n = trim(&line, n);
        if (n == 0) {
            continue;
        }
        const char *eq = memchr(line, '=', n);
        if (eq == NULL) {
            return fail(err, err_len, "line %u: expected key=value", line_no);
        }
        const char *key  = line;
        size_t      klen = trim(&key, (size_t)(eq - line));
        const char *val  = eq + 1;
        size_t      vlen = trim(&val, n - (size_t)(eq - line) - 1);

        const field_t *f = NULL;
        for (size_t i = 0; i < N_FIELDS; i++) {
            if (strlen(k_fields[i].key) == klen && memcmp(k_fields[i].key, key, klen) == 0) {
                f = &k_fields[i];
                break;
            }
        }
        if (f == NULL) {
            return fail(err, err_len, "line %u: unknown key '%.*s'", line_no, (int)klen, key);
        }
        uint32_t v;
        if (!parse_u32(val, vlen, &v) || v < f->min || v > f->max) {
            return fail(err, err_len, "line %u: %s must be %lu..%lu", line_no, f->key,
                        (unsigned long)f->min, (unsigned long)f->max);
        }
        uint8_t *field = (uint8_t *)&c + f->offset;
        if (f->kind == K_BOOL) {
            *(bool *)field = v != 0;
        } else {
            *(uint16_t *)field = (uint16_t)v;
        }
    }
    *out = c;
    return 0;
}

size_t lbr_gw_config_format(const lbr_gw_config_t *c, char *buf, size_t cap)
{
    size_t len = 0;
    if (cap == 0) {
        return 0;
    }
    buf[0] = '\0';
    for (size_t i = 0; i < N_FIELDS && len < cap; i++) {
        const uint8_t *field = (const uint8_t *)c + k_fields[i].offset;
        unsigned long  v     = k_fields[i].kind == K_BOOL ? (*(const bool *)field ? 1u : 0u)
                                                          : *(const uint16_t *)field;
        int            w     = snprintf(buf + len, cap - len, "%s=%lu\n", k_fields[i].key, v);
        if (w < 0) {
            break;
        }
        len += (size_t)w;
    }
    return len < cap ? len : cap - 1;
}
