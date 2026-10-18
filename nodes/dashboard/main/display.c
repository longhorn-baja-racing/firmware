#include "display.h"

#include <stdio.h>

#include "esp_log.h"
#include "lbr/nodes.h"

static const char *TAG = "dash";

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

void dash_display_render(const lbr_value_table_t *values, const lbr_node_table_t *nodes,
                         uint64_t now_us)
{
    char speed[16], eng[16], cvt[16], fuel[16], batt[16];
    fmt_value(values, LBR_SIG_GPS_SPEED, now_us, speed, sizeof speed);
    fmt_value(values, LBR_SIG_TEMP_ENGINE, now_us, eng, sizeof eng);
    fmt_value(values, LBR_SIG_TEMP_CVT, now_us, cvt, sizeof cvt);
    fmt_value(values, LBR_SIG_FUEL_LEVEL, now_us, fuel, sizeof fuel);
    fmt_value(values, LBR_SIG_BATT_VOLTAGE, now_us, batt, sizeof batt);
    ESP_LOGI(TAG, "spd %s m/s | eng %s C | cvt %s C | fuel %s %% | batt %s V", speed, eng, cvt,
             fuel, batt);

    char   line[160];
    size_t n = 0;
    for (size_t i = 0; i < LBR_NODE_TABLE_SIZE && n < sizeof line; i++) {
        const lbr_node_entry_t *e = &nodes->nodes[i];
        if (e->node_id == 0) {
            continue;
        }
        int w = snprintf(line + n, sizeof line - n, "%s:%s ", lbr_node_name(e->node_id),
                         e->present ? "up" : "LOST");
        if (w > 0) {
            n += (size_t)w;
        }
    }
    if (n > 0) {
        ESP_LOGI(TAG, "nodes %s", line);
    }
}
