#include "display.h"

#include <stdio.h>

#include "esp_log.h"
#include "lbr/health.h"
#include "lbr/nodes.h"

static const char *TAG = "dash";

static const char *state_name(uint8_t state)
{
    static const char *const names[] = {
        [LBR_NODE_BOOT] = "BOOT",         [LBR_NODE_RUN] = "RUN",
        [LBR_NODE_DEGRADED] = "DEGRADED", [LBR_NODE_CONFIG_INVALID] = "BADCONFIG",
        [LBR_NODE_FAULT] = "FAULT",
    };
    return state < sizeof names / sizeof names[0] ? names[state] : "?";
}

void dash_display_init(void)
{
    ESP_LOGI(TAG, "console display backend (panel hardware TBD)");
}

static void fmt_value(const lbr_value_table_t *values, lbr_signal_id_t sig, uint64_t now, char *out,
                      size_t cap)
{
    lbr_sample_t s;
    switch (lbr_value_table_get(values, sig, now, &s)) {
        case LBR_VALUE_OK:
            if (s.type == LBR_VAL_F32) {
                snprintf(out, cap, "%.1f", (double)s.v.f32);
            } else {
                snprintf(out, cap, "%lu", (unsigned long)s.v.u32);
            }
            break;
        case LBR_VALUE_INVALID:
            snprintf(out, cap, "n/a(%lu)", (unsigned long)s.v.u32);
            break;
        case LBR_VALUE_STALE:
            snprintf(out, cap, "stale");
            break;
        default:
            snprintf(out, cap, "--");
            break;
    }
}

void dash_display_render(const lbr_value_table_t *values,
                         const lbr_dash_node_t nodes[LBR_DASH_MAX_NODES], bool link_up,
                         uint64_t now_us)
{
    if (!link_up) {
        ESP_LOGW(TAG, "NO LINK to the Central");
    }
    char rpm[16], cvt[16], speed[16], fuel[16], tcvt[16];
    fmt_value(values, LBR_SIG_ENGINE_RPM, now_us, rpm, sizeof rpm);
    fmt_value(values, LBR_SIG_CVT_RPM, now_us, cvt, sizeof cvt);
    fmt_value(values, LBR_SIG_GPS_SPEED, now_us, speed, sizeof speed);
    fmt_value(values, LBR_SIG_FUEL_LEVEL, now_us, fuel, sizeof fuel);
    fmt_value(values, LBR_SIG_TEMP_CVT, now_us, tcvt, sizeof tcvt);
    ESP_LOGI(TAG, "rpm %s | cvt %s rpm | spd %s m/s | fuel %s %% | cvt %s C", rpm, cvt, speed, fuel,
             tcvt);

    char   line[192];
    size_t n = 0;
    for (size_t i = 0; i < LBR_DASH_MAX_NODES && n < sizeof line; i++) {
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
