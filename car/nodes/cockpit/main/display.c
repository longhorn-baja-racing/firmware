#include "display.h"

#include <math.h>
#include <stdio.h>

#include "esp_log.h"
#include "lbr/health.h"
#include "lbr/nodes.h"

static const char *TAG = "cockpit";

static const char *state_name(uint8_t state)
{
    static const char *const names[] = {
        [LBR_NODE_BOOT] = "BOOT",         [LBR_NODE_RUN] = "RUN",
        [LBR_NODE_DEGRADED] = "DEGRADED", [LBR_NODE_CONFIG_INVALID] = "BADCONFIG",
        [LBR_NODE_FAULT] = "FAULT",
    };
    return state < sizeof names / sizeof names[0] ? names[state] : "?";
}

void cockpit_display_init(void)
{
    ESP_LOGI(TAG, "cockpit display backend ready (panel hardware TBD)");
}

static void fmt_value(const lbr_value_table_t *values, lbr_signal_id_t sig, uint64_t now,
                      bool link_up, char *out, size_t cap)
{
    /* Do not let cached values look live after the Central link drops. */
    if (values == NULL || !link_up) {
        snprintf(out, cap, "--");
        return;
    }

    lbr_sample_t s;
    switch (lbr_value_table_get(values, sig, now, &s)) {
        case LBR_VALUE_OK:
            /* The dashboard signals are catalogued as F32. A malformed or
             * mismatched sample must not turn its raw bits into a reading. */
            if (s.type != LBR_VAL_F32 || !isfinite(s.v.f32)) {
                snprintf(out, cap, "--");
                break;
            }
            snprintf(out, cap, "%.1f", (double)s.v.f32);
            break;
        case LBR_VALUE_INVALID:
        case LBR_VALUE_STALE:
        default:
            snprintf(out, cap, "--");
            break;
    }
}

void cockpit_display_render(const lbr_value_table_t *values,
                            const lbr_cockpit_node_t nodes[LBR_COCKPIT_MAX_NODES], bool link_up,
                            uint64_t now_us)
{
    if (!link_up) {
        ESP_LOGW(TAG, "NO LINK to the Central");
    }
    char rpm[16], speed[16], fuel[16], tcvt[16];
    fmt_value(values, LBR_SIG_ENGINE_RPM, now_us, link_up, rpm, sizeof rpm);
    fmt_value(values, LBR_SIG_GPS_SPEED, now_us, link_up, speed, sizeof speed);
    fmt_value(values, LBR_SIG_FUEL_LEVEL, now_us, link_up, fuel, sizeof fuel);
    fmt_value(values, LBR_SIG_TEMP_CVT, now_us, link_up, tcvt, sizeof tcvt);
    ESP_LOGI(TAG, "rpm %s | spd %s m/s | fuel %s %% | cvt %s C", rpm, speed, fuel, tcvt);

    static const lbr_signal_id_t brake_signals[] = {
        LBR_SIG_TEMP_BRAKE_FL, LBR_SIG_TEMP_BRAKE_FR,
        LBR_SIG_TEMP_BRAKE_BL, LBR_SIG_TEMP_BRAKE_BR,
    };
    char brake_values[sizeof brake_signals / sizeof brake_signals[0]][16];
    for (size_t i = 0; i < sizeof brake_signals / sizeof brake_signals[0]; i++) {
        fmt_value(values, brake_signals[i], now_us, link_up, brake_values[i],
                  sizeof brake_values[i]);
    }
    ESP_LOGI(TAG, "brake C FL %s | FR %s | BL %s | BR %s", brake_values[0], brake_values[1],
             brake_values[2], brake_values[3]);

    static const lbr_signal_id_t stress_signals[] = {
        LBR_SIG_STRESS_1, LBR_SIG_STRESS_2, LBR_SIG_STRESS_3, LBR_SIG_STRESS_4,
        LBR_SIG_STRESS_5, LBR_SIG_STRESS_6, LBR_SIG_STRESS_7, LBR_SIG_STRESS_8,
    };
    char stress_values[sizeof stress_signals / sizeof stress_signals[0]][16];
    for (size_t i = 0; i < sizeof stress_signals / sizeof stress_signals[0]; i++) {
        fmt_value(values, stress_signals[i], now_us, link_up, stress_values[i],
                  sizeof stress_values[i]);
    }
    ESP_LOGI(TAG, "stress: %s %s %s %s %s %s %s %s", stress_values[0], stress_values[1],
             stress_values[2], stress_values[3], stress_values[4], stress_values[5],
             stress_values[6], stress_values[7]);

    char   line[192];
    size_t n = 0;
    for (size_t i = 0; i < LBR_COCKPIT_MAX_NODES && n < sizeof line; i++) {
        if (nodes[i].node_id == 0) {
            continue;
        }
        int w = snprintf(line + n, sizeof line - n, "%s:%s ", lbr_node_name(nodes[i].node_id),
                         state_name(nodes[i].state));
        if (w > 0) {
            n += (size_t)w;
        }
    }
    if (n > 0) {
        ESP_LOGI(TAG, "nodes %s", line);
    }
}
