#include "lbr/nmea.h"

#include <ctype.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>

#define MAX_FIELDS   24
#define KNOTS_TO_MPS 0.514444f

void lbr_nmea_init(lbr_nmea_t *p)
{
    memset(p, 0, sizeof *p);
}

static bool recent(uint64_t at_us, uint64_t now_us)
{
    return at_us != 0 && now_us >= at_us && now_us - at_us < LBR_NMEA_STALE_US;
}

static int hex_val(char c)
{
    if (c >= '0' && c <= '9') {
        return c - '0';
    }
    if (c >= 'A' && c <= 'F') {
        return c - 'A' + 10;
    }
    if (c >= 'a' && c <= 'f') {
        return c - 'a' + 10;
    }
    return -1;
}

/* Strictly numeric field: digits with at most one '.', optional leading '-'. */
static bool parse_num(const char *s, double *out)
{
    if (s[0] == '\0') {
        return false;
    }
    bool dot = false;
    for (const char *q = s + (s[0] == '-' ? 1 : 0); *q != '\0'; q++) {
        if (*q == '.') {
            if (dot) {
                return false;
            }
            dot = true;
        } else if (!isdigit((unsigned char)*q)) {
            return false;
        }
    }
    *out = strtod(s, NULL);
    return isfinite(*out);
}

/* "ddmm.mmmm" / "dddmm.mmmm" plus hemisphere -> 1e-7 degree. */
static bool parse_coord(const char *v, const char *hemi, bool lon, int32_t *out)
{
    double raw;
    if (!parse_num(v, &raw) || raw < 0.0 || hemi[0] == '\0' || hemi[1] != '\0') {
        return false;
    }
    double deg = floor(raw / 100.0);
    double min = raw - deg * 100.0;
    if (min >= 60.0 || deg > (lon ? 180.0 : 90.0)) {
        return false;
    }
    double val = deg + min / 60.0;
    if (val > (lon ? 180.0 : 90.0)) {
        return false;
    }
    char h = hemi[0];
    if (lon ? (h != 'E' && h != 'W') : (h != 'N' && h != 'S')) {
        return false;
    }
    if (h == 'S' || h == 'W') {
        val = -val;
    }
    *out = (int32_t)lround(val * 1e7);
    return true;
}

uint32_t lbr_nmea_unix_time(unsigned year, unsigned month, unsigned day, unsigned hour,
                            unsigned min, unsigned sec)
{
    if (year < 1970 || year > 2105 || month < 1 || month > 12 || day < 1 || day > 31 || hour > 23 ||
        min > 59 || sec > 60) {
        return 0;
    }
    /* days_from_civil (proleptic Gregorian). */
    int      y    = (int)year - (month <= 2 ? 1 : 0);
    int      era  = y / 400;
    unsigned yoe  = (unsigned)(y - era * 400);
    unsigned mp   = (month + 9u) % 12u;
    unsigned doy  = (153u * mp + 2u) / 5u + day - 1u;
    unsigned doe  = yoe * 365u + yoe / 4u - yoe / 100u + doy;
    int64_t  days = (int64_t)era * 146097 + (int64_t)doe - 719468;
    return (uint32_t)(days * 86400 + (int64_t)hour * 3600 + (int64_t)min * 60 + (int64_t)sec);
}

static bool two_digits(const char *s, unsigned *out)
{
    if (!isdigit((unsigned char)s[0]) || !isdigit((unsigned char)s[1])) {
        return false;
    }
    *out = (unsigned)(s[0] - '0') * 10u + (unsigned)(s[1] - '0');
    return true;
}

static void on_gga(lbr_nmea_t *p, char **f, int nf, uint64_t now_us)
{
    if (nf < 10) {
        return;
    }
    double q = 0.0, sats = 0.0, hdop = 0.0, alt = 0.0;
    p->gga_quality = parse_num(f[6], &q) && q >= 0.0 && q < 10.0 ? (uint8_t)q : 0u;
    p->sats        = parse_num(f[7], &sats) && sats >= 0.0 && sats < 256.0 ? (uint8_t)sats : 0u;
    p->hdop_valid  = parse_num(f[8], &hdop) && hdop >= 0.0;
    p->hdop        = p->hdop_valid ? (float)hdop : 0.0f;
    p->gga_us      = now_us;
    int32_t lat, lon;
    if (p->gga_quality != 0 && parse_coord(f[2], f[3], false, &lat) &&
        parse_coord(f[4], f[5], true, &lon)) {
        p->lat_e7 = lat;
        p->lon_e7 = lon;
        p->pos_us = now_us;
        if (parse_num(f[9], &alt)) {
            p->alt_m  = (float)alt;
            p->alt_us = now_us;
        }
    }
}

static void on_rmc(lbr_nmea_t *p, char **f, int nf, uint64_t now_us)
{
    if (nf < 10) {
        return;
    }
    p->rmc_active = strcmp(f[2], "A") == 0;
    p->rmc_us     = now_us;
    if (!p->rmc_active) {
        return;
    }
    int32_t lat, lon;
    if (parse_coord(f[3], f[4], false, &lat) && parse_coord(f[5], f[6], true, &lon)) {
        p->lat_e7 = lat;
        p->lon_e7 = lon;
        p->pos_us = now_us;
    }
    double kn = 0.0, course = 0.0;
    if (parse_num(f[7], &kn) && kn >= 0.0) {
        p->speed_mps    = (float)kn * KNOTS_TO_MPS;
        p->course_valid = parse_num(f[8], &course) && course >= 0.0 && course < 360.0;
        p->course_deg   = p->course_valid ? (float)course : 0.0f;
        p->vel_us       = now_us;
    }
    unsigned hh, mm, ss, dd, mo, yy;
    if (strlen(f[1]) >= 6 && strlen(f[9]) == 6 && two_digits(f[1], &hh) &&
        two_digits(f[1] + 2, &mm) && two_digits(f[1] + 4, &ss) && two_digits(f[9], &dd) &&
        two_digits(f[9] + 2, &mo) && two_digits(f[9] + 4, &yy)) {
        uint32_t t = lbr_nmea_unix_time(2000u + yy, mo, dd, hh, mm, ss);
        if (t != 0) {
            p->utc_s  = t;
            p->utc_us = now_us;
        }
    }
}

static void on_gsa(lbr_nmea_t *p, char **f, int nf, uint64_t now_us)
{
    double mode = 0.0;
    if (nf < 3 || !parse_num(f[2], &mode) || mode < 1.0 || mode > 3.0) {
        return;
    }
    p->gsa_mode = (uint8_t)mode;
    p->gsa_us   = now_us;
}

static bool process_line(lbr_nmea_t *p, uint64_t now_us)
{
    p->line[p->len] = '\0';
    char *star      = strchr(p->line, '*');
    if (star == NULL || hex_val(star[1]) < 0 || hex_val(star[2]) < 0 || star[3] != '\0') {
        p->checksum_errors++;
        return false;
    }
    uint8_t sum = 0;
    for (const char *q = p->line; q < star; q++) {
        sum ^= (uint8_t)*q;
    }
    if (sum != (uint8_t)(hex_val(star[1]) * 16 + hex_val(star[2]))) {
        p->checksum_errors++;
        return false;
    }
    *star = '\0';

    char *f[MAX_FIELDS];
    int   nf = 0;
    char *s  = p->line;
    for (;;) {
        if (nf == MAX_FIELDS) {
            break;
        }
        f[nf++]     = s;
        char *comma = strchr(s, ',');
        if (comma == NULL) {
            break;
        }
        *comma = '\0';
        s      = comma + 1;
    }
    p->sentences++;
    p->last_good_us = now_us;

    /* Address: two-letter talker + three-letter type (proprietary P... ignored). */
    if (strlen(f[0]) != 5 || f[0][0] == 'P') {
        return true;
    }
    const char *type = f[0] + 2;
    if (strcmp(type, "GGA") == 0) {
        on_gga(p, f, nf, now_us);
    } else if (strcmp(type, "RMC") == 0) {
        on_rmc(p, f, nf, now_us);
    } else if (strcmp(type, "GSA") == 0) {
        on_gsa(p, f, nf, now_us);
    }
    return true;
}

uint32_t lbr_nmea_feed(lbr_nmea_t *p, const uint8_t *data, size_t n, uint64_t now_us)
{
    uint32_t good = 0;
    for (size_t i = 0; i < n; i++) {
        char c = (char)data[i];
        if (c == '$') {
            p->in_sentence = true;
            p->len         = 0;
        } else if (c == '\r' || c == '\n') {
            if (p->in_sentence && p->len > 0 && process_line(p, now_us)) {
                good++;
            }
            p->in_sentence = false;
        } else if (p->in_sentence) {
            if (p->len + 1 >= LBR_NMEA_LINE_MAX || (unsigned char)c < 0x20u ||
                (unsigned char)c > 0x7Eu) {
                p->overlong++;
                p->in_sentence = false;
            } else {
                p->line[p->len++] = c;
            }
        }
    }
    return good;
}

uint8_t lbr_nmea_fix_type(const lbr_nmea_t *p, uint64_t now_us)
{
    if (recent(p->gga_us, now_us)) {
        if (p->gga_quality == 0) {
            return 0;
        }
        if (p->gga_quality == 6) {
            return 1;
        }
        if (recent(p->gsa_us, now_us) && p->gsa_mode >= 2) {
            return p->gsa_mode;
        }
        return recent(p->alt_us, now_us) ? 3 : 2;
    }
    if (recent(p->rmc_us, now_us) && p->rmc_active) {
        return recent(p->gsa_us, now_us) && p->gsa_mode >= 2 ? p->gsa_mode : 2;
    }
    return 0;
}

static void set_f32(lbr_reading_t *r, bool ok, float v)
{
    *r = ok ? (lbr_reading_t){.valid = true, .v.f32 = v}
            : (lbr_reading_t){.valid = false, .reason = LBR_UNAVAIL_NO_DATA};
}

static void set_i32(lbr_reading_t *r, bool ok, int32_t v)
{
    *r = ok ? (lbr_reading_t){.valid = true, .v.i32 = v}
            : (lbr_reading_t){.valid = false, .reason = LBR_UNAVAIL_NO_DATA};
}

static void set_u32(lbr_reading_t *r, bool ok, uint32_t v)
{
    *r = ok ? (lbr_reading_t){.valid = true, .v.u32 = v}
            : (lbr_reading_t){.valid = false, .reason = LBR_UNAVAIL_NO_DATA};
}

void lbr_nmea_fill(const lbr_nmea_t *p, uint64_t now_us, const lbr_signal_id_t *signals, size_t n,
                   lbr_reading_t *out)
{
    bool    live = recent(p->last_good_us, now_us);
    uint8_t fix  = live ? lbr_nmea_fix_type(p, now_us) : 0;
    bool    pos  = live && fix >= 2 && recent(p->pos_us, now_us);
    bool    vel  = live && fix >= 2 && recent(p->vel_us, now_us);
    bool    gga  = live && recent(p->gga_us, now_us);
    for (size_t i = 0; i < n; i++) {
        lbr_reading_t *r = &out[i];
        switch (signals[i]) {
            case LBR_SIG_GPS_LATITUDE:
                set_i32(r, pos, p->lat_e7);
                break;
            case LBR_SIG_GPS_LONGITUDE:
                set_i32(r, pos, p->lon_e7);
                break;
            case LBR_SIG_GPS_ALTITUDE:
                set_f32(r, live && fix >= 3 && recent(p->alt_us, now_us), p->alt_m);
                break;
            case LBR_SIG_GPS_SPEED:
                set_f32(r, vel, p->speed_mps);
                break;
            case LBR_SIG_GPS_COURSE:
                set_f32(r, vel && p->course_valid, p->course_deg);
                break;
            case LBR_SIG_GPS_FIX:
                set_u32(r, live, fix);
                break;
            case LBR_SIG_GPS_SATELLITES:
                set_u32(r, gga, p->sats);
                break;
            case LBR_SIG_GPS_HDOP:
                set_f32(r, gga && p->hdop_valid, p->hdop);
                break;
            case LBR_SIG_GPS_UTC:
                set_u32(r,
                        live && p->rmc_active && recent(p->rmc_us, now_us) &&
                            recent(p->utc_us, now_us),
                        p->utc_s);
                break;
            default:
                *r = (lbr_reading_t){.valid = false, .reason = LBR_UNAVAIL_NOT_FITTED};
                break;
        }
    }
}
