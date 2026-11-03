#include "display_graphics.h"

#include <math.h>
#include <stdio.h>

#define DASHBOARD_BG       0x101820
#define TILE_BG            0x1B2833
#define TEXT_COLOR         0xF1F5F8
#define MUTED_COLOR        0x9AAAB7
#define FUEL_WARNING_COLOR 0xFF453A
#define TACH_NORMAL_COLOR  0x34C759
#define TACH_WARN_COLOR    0xFFC247
#define TACH_REDLINE       0xE9545C
#define STRESS_DANGER_RATIO 0.80f
#define TEMP_DANGER_C       100.0f

typedef struct {
    lv_obj_t *viewport;
    lv_obj_t *rpm_bar;
    lv_obj_t *rpm_value;
    lv_obj_t *speed_value;
    lv_obj_t *stress_tiles[COCKPIT_GRAPHICS_STRESS_COUNT];
    lv_obj_t *stress_value_labels[COCKPIT_GRAPHICS_STRESS_COUNT];
    lv_obj_t *temperature_tiles[COCKPIT_GRAPHICS_BRAKE_COUNT + 1];
    lv_obj_t *temperature_value_labels[COCKPIT_GRAPHICS_BRAKE_COUNT + 1];
    lv_obj_t *fuel_value;
    lv_obj_t *fuel_bar;
    lv_obj_t *fuel_warning;
    int32_t rpm;
} dashboard_t;

static dashboard_t s_dashboard;

static lv_obj_t *make_label(lv_obj_t *parent, const char *text, uint32_t color,
                            const lv_font_t *font)
{
    lv_obj_t *label = lv_label_create(parent);
    lv_label_set_text(label, text);
    lv_obj_set_style_text_color(label, lv_color_hex(color), 0);
    if (font != NULL) {
        lv_obj_set_style_text_font(label, font, 0);
    }
    return label;
}

static lv_obj_t *make_tile(lv_obj_t *parent)
{
    lv_obj_t *tile = lv_obj_create(parent);
    lv_obj_set_style_bg_color(tile, lv_color_hex(TILE_BG), 0);
    lv_obj_set_style_bg_opa(tile, LV_OPA_COVER, 0);
    lv_obj_set_style_radius(tile, 10, 0);
    lv_obj_set_style_border_width(tile, 2, 0);
    lv_obj_set_style_border_color(tile, lv_color_hex(0x344653), 0);
    lv_obj_set_style_pad_all(tile, 0, 0);
    return tile;
}

static lv_obj_t *label_tile(lv_obj_t *tile, const char *text, lv_coord_t y_offset)
{
    lv_obj_t *label = make_label(tile, text, TEXT_COLOR, &lv_font_montserrat_20);
    lv_obj_set_style_bg_color(label, lv_color_hex(DASHBOARD_BG), 0);
    lv_obj_set_style_bg_opa(label, LV_OPA_70, 0);
    lv_obj_set_style_radius(label, 4, 0);
    lv_obj_set_style_pad_hor(label, 3, 0);
    lv_obj_set_style_pad_ver(label, 1, 0);
    lv_obj_align(label, LV_ALIGN_CENTER, 0, y_offset);
    return label;
}

static void resize_viewport(lv_obj_t *parent, lv_obj_t *viewport)
{
    lv_coord_t parent_w = lv_obj_get_content_width(parent);
    lv_coord_t parent_h = lv_obj_get_content_height(parent);
    if (parent_w <= 0 || parent_h <= 0) return;

    lv_coord_t width = parent_w;
    lv_coord_t height = (lv_coord_t)((int32_t)width * 3 / 4);
    if (height > parent_h) {
        height = parent_h;
        width = (lv_coord_t)((int32_t)height * 4 / 3);
    }
    lv_obj_set_size(viewport, width, height);
    lv_obj_center(viewport);
}

static void parent_size_changed(lv_event_t *event)
{
    resize_viewport(lv_event_get_target(event), lv_event_get_user_data(event));
}

static lv_color_t stress_border_color(float value, float full_scale)
{
    float ratio = value / (full_scale * STRESS_DANGER_RATIO);
    if (ratio < 0.0f) ratio = 0.0f;
    if (ratio > 1.0f) ratio = 1.0f;
    return lv_color_make((uint8_t)(255.0f * ratio),
                         (uint8_t)(210.0f - 35.0f * ratio), 35);
}

static lv_color_t temperature_border_color(float celsius)
{
    float ratio = (celsius - 65.0f) / (TEMP_DANGER_C - 65.0f);
    if (ratio < 0.0f) ratio = 0.0f;
    if (ratio > 1.0f) ratio = 1.0f;
    return lv_color_make((uint8_t)(255.0f * ratio), (uint8_t)(100.0f + 80.0f * ratio),
                         (uint8_t)(255.0f * (1.0f - ratio)));
}

lv_obj_t *cockpit_graphics_create(lv_obj_t *parent)
{
    if (parent == NULL) return NULL;
    s_dashboard = (dashboard_t){0};

    lv_obj_t *viewport = lv_obj_create(parent);
    s_dashboard.viewport = viewport;
    lv_obj_remove_style_all(viewport);
    lv_obj_set_style_bg_color(viewport, lv_color_hex(DASHBOARD_BG), 0);
    lv_obj_set_style_bg_opa(viewport, LV_OPA_COVER, 0);
    lv_obj_set_style_pad_all(viewport, 10, 0);
    lv_obj_set_style_pad_column(viewport, 20, 0);
    lv_obj_set_flex_flow(viewport, LV_FLEX_FLOW_ROW);

    /* Left half: RPM is shown as a wide rectangular bar graph. */
    lv_obj_t *left = lv_obj_create(viewport);
    lv_obj_remove_style_all(left);
    lv_obj_set_size(left, LV_PCT(47), LV_PCT(100));
    lv_obj_set_style_pad_all(left, 0, 0);
    lv_obj_set_flex_flow(left, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(left, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER,
                          LV_FLEX_ALIGN_CENTER);
    make_label(left, "TACHOMETER (x1000 RPM)", MUTED_COLOR, &lv_font_montserrat_20);

    lv_obj_t *holder = lv_obj_create(left);
    lv_obj_remove_style_all(holder);
    lv_obj_set_width(holder, LV_PCT(100));
    lv_obj_set_flex_grow(holder, 1);
    lv_obj_set_flex_flow(holder, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(holder, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER,
                          LV_FLEX_ALIGN_CENTER);

    lv_obj_t *gauge_row = lv_obj_create(holder);
    lv_obj_remove_style_all(gauge_row);
    lv_obj_set_width(gauge_row, LV_PCT(100));
    lv_obj_set_flex_grow(gauge_row, 1);
    lv_obj_set_style_pad_column(gauge_row, 12, 0);
    lv_obj_set_flex_flow(gauge_row, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(gauge_row, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER,
                          LV_FLEX_ALIGN_CENTER);

    lv_obj_t *rpm_axis = lv_obj_create(gauge_row);
    lv_obj_remove_style_all(rpm_axis);
    lv_obj_set_size(rpm_axis, 76, LV_PCT(84));
    lv_obj_set_style_pad_row(rpm_axis, 0, 0);
    lv_obj_set_flex_flow(rpm_axis, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(rpm_axis, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER,
                          LV_FLEX_ALIGN_CENTER);
    /* Mark every 1000 RPM; emphasize the 7000-8000 RPM redline. */
    static const char *const rpm_ticks[] = {"8", "7", "6", "5", "4", "3", "2", "1", "0"};
    for (size_t i = 0; i < sizeof rpm_ticks / sizeof rpm_ticks[0]; i++) {
        lv_obj_t *tick_row = lv_obj_create(rpm_axis);
        lv_obj_remove_style_all(tick_row);
        lv_obj_set_width(tick_row, LV_PCT(100));
        lv_obj_set_flex_grow(tick_row, 1);
        lv_obj_set_style_pad_column(tick_row, 5, 0);
        lv_obj_set_flex_flow(tick_row, LV_FLEX_FLOW_ROW);
        lv_obj_set_flex_align(tick_row, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER,
                              LV_FLEX_ALIGN_CENTER);
        lv_obj_t *tick_label = make_label(tick_row, rpm_ticks[i], TEXT_COLOR,
                                          &lv_font_montserrat_20);
        lv_obj_set_width(tick_label, 24);
        lv_obj_set_style_text_align(tick_label, LV_TEXT_ALIGN_RIGHT, 0);
        /* Short axis tick visually joins the matching line inside the bar. */
        lv_obj_t *tick_mark = lv_obj_create(tick_row);
        lv_obj_remove_style_all(tick_mark);
        lv_obj_set_size(tick_mark, 12, 3);
        uint32_t tick_color = (i <= 1) ? TACH_REDLINE : TEXT_COLOR;
        lv_obj_set_style_bg_color(tick_mark, lv_color_hex(tick_color), 0);
        lv_obj_set_style_bg_opa(tick_mark, LV_OPA_COVER, 0);
    }

    s_dashboard.rpm_bar = lv_bar_create(gauge_row);
    lv_bar_set_orientation(s_dashboard.rpm_bar, LV_BAR_ORIENTATION_VERTICAL);
    lv_bar_set_range(s_dashboard.rpm_bar, 0, 8000);
    lv_bar_set_value(s_dashboard.rpm_bar, 0, LV_ANIM_OFF);
    lv_obj_set_size(s_dashboard.rpm_bar, 250, LV_PCT(84));
    lv_obj_set_style_bg_color(s_dashboard.rpm_bar, lv_color_hex(0x344653), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(s_dashboard.rpm_bar, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_radius(s_dashboard.rpm_bar, 14, LV_PART_MAIN);
    lv_obj_set_style_bg_color(s_dashboard.rpm_bar, lv_color_hex(TACH_NORMAL_COLOR), LV_PART_INDICATOR);
    lv_obj_set_style_bg_opa(s_dashboard.rpm_bar, LV_OPA_COVER, LV_PART_INDICATOR);
    lv_obj_set_style_radius(s_dashboard.rpm_bar, 14, LV_PART_INDICATOR);
    /* Yard lines across the bar, aligned to each 1000 RPM label. */
    lv_obj_t *rpm_lines = lv_obj_create(s_dashboard.rpm_bar);
    lv_obj_remove_style_all(rpm_lines);
    lv_obj_set_size(rpm_lines, LV_PCT(100), LV_PCT(100));
    lv_obj_set_style_pad_all(rpm_lines, 0, 0);
    lv_obj_set_flex_flow(rpm_lines, LV_FLEX_FLOW_COLUMN);
    for (size_t i = 0; i < sizeof rpm_ticks / sizeof rpm_ticks[0]; i++) {
        lv_obj_t *line_row = lv_obj_create(rpm_lines);
        lv_obj_remove_style_all(line_row);
        lv_obj_set_width(line_row, LV_PCT(100));
        lv_obj_set_flex_grow(line_row, 1);
        lv_obj_set_flex_flow(line_row, LV_FLEX_FLOW_COLUMN);
        lv_obj_set_flex_align(line_row, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER,
                              LV_FLEX_ALIGN_CENTER);
        lv_obj_t *line = lv_obj_create(line_row);
        lv_obj_remove_style_all(line);
        lv_obj_set_size(line, LV_PCT(100), 2);
        uint32_t line_color = (i <= 1) ? TACH_REDLINE : TEXT_COLOR;
        lv_obj_set_style_bg_color(line, lv_color_hex(line_color), 0);
        lv_obj_set_style_bg_opa(line, (i <= 1) ? LV_OPA_COVER : LV_OPA_70, 0);
    }
    lv_obj_align(rpm_lines, LV_ALIGN_CENTER, 0, 0);
    s_dashboard.rpm_value = make_label(s_dashboard.rpm_bar, "-- rpm", TEXT_COLOR,
                                       &lv_font_montserrat_24);
    lv_obj_set_style_bg_color(s_dashboard.rpm_value, lv_color_hex(DASHBOARD_BG), 0);
    lv_obj_set_style_bg_opa(s_dashboard.rpm_value, LV_OPA_80, 0);
    lv_obj_set_style_radius(s_dashboard.rpm_value, 6, 0);
    lv_obj_set_style_pad_hor(s_dashboard.rpm_value, 6, 0);
    lv_obj_set_style_pad_ver(s_dashboard.rpm_value, 3, 0);
    lv_obj_center(s_dashboard.rpm_value);
    s_dashboard.speed_value = make_label(holder, "-- km/h", TEXT_COLOR,
                                         &lv_font_montserrat_40);

    /* Right half: the 4 by 2 stress array, then temperatures and fuel. */
    lv_obj_t *right = lv_obj_create(viewport);
    lv_obj_remove_style_all(right);
    lv_obj_set_size(right, LV_PCT(49), LV_PCT(100));
    lv_obj_set_style_pad_all(right, 0, 0);
    lv_obj_set_style_pad_row(right, 4, 0);
    lv_obj_set_flex_flow(right, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(right, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER,
                          LV_FLEX_ALIGN_CENTER);

    lv_obj_t *stress_area = lv_obj_create(right);
    lv_obj_remove_style_all(stress_area);
    lv_obj_set_width(stress_area, LV_PCT(100));
    lv_obj_set_height(stress_area, LV_PCT(47));
    lv_obj_set_style_pad_all(stress_area, 0, 0);
    lv_obj_set_style_pad_row(stress_area, 4, 0);
    lv_obj_set_flex_flow(stress_area, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(stress_area, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER,
                          LV_FLEX_ALIGN_CENTER);
    make_label(stress_area, "STRESS", MUTED_COLOR, &lv_font_montserrat_20);
    for (size_t row = 0; row < 2; row++) {
        lv_obj_t *line = lv_obj_create(stress_area);
        lv_obj_remove_style_all(line);
        lv_obj_set_width(line, LV_PCT(100));
        lv_obj_set_flex_grow(line, 1);
        lv_obj_set_style_pad_column(line, 7, 0);
        lv_obj_set_flex_flow(line, LV_FLEX_FLOW_ROW);
        lv_obj_set_flex_align(line, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER,
                              LV_FLEX_ALIGN_CENTER);
        for (size_t col = 0; col < 4; col++) {
            size_t i = row * 4 + col;
            s_dashboard.stress_tiles[i] = make_tile(line);
            lv_obj_set_flex_grow(s_dashboard.stress_tiles[i], 1);
            lv_obj_set_height(s_dashboard.stress_tiles[i], LV_PCT(100));
            char name[4];
            (void)snprintf(name, sizeof name, "S%u", (unsigned)(i + 1));
            (void)label_tile(s_dashboard.stress_tiles[i], name, -20);
            s_dashboard.stress_value_labels[i] =
                label_tile(s_dashboard.stress_tiles[i], "--", 20);
        }
    }

    lv_obj_t *lower = lv_obj_create(right);
    lv_obj_remove_style_all(lower);
    lv_obj_set_width(lower, LV_PCT(100));
    lv_obj_set_flex_grow(lower, 1);
    lv_obj_set_style_pad_column(lower, 8, 0);
    lv_obj_set_flex_flow(lower, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(lower, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER,
                          LV_FLEX_ALIGN_CENTER);

    lv_obj_t *temps = lv_obj_create(lower);
    lv_obj_remove_style_all(temps);
    lv_obj_set_size(temps, LV_PCT(68), LV_PCT(100));
    lv_obj_set_style_pad_row(temps, 6, 0);
    lv_obj_set_style_pad_column(temps, 6, 0);
    lv_obj_set_flex_flow(temps, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(temps, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER,
                          LV_FLEX_ALIGN_CENTER);
    make_label(temps, "TEMPS", MUTED_COLOR, &lv_font_montserrat_20);
    for (size_t row = 0; row < 2; row++) {
        lv_obj_t *line = lv_obj_create(temps);
        lv_obj_remove_style_all(line);
        lv_obj_set_width(line, LV_PCT(100));
        lv_obj_set_flex_grow(line, 1);
        lv_obj_set_style_pad_column(line, 6, 0);
        lv_obj_set_flex_flow(line, LV_FLEX_FLOW_ROW);
        lv_obj_set_flex_align(line, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER,
                              LV_FLEX_ALIGN_CENTER);
        for (size_t col = 0; col < (row == 0 ? 3u : 2u); col++) {
            size_t i = row == 0 ? col : col + 3;
            s_dashboard.temperature_tiles[i] = make_tile(line);
            lv_obj_set_flex_grow(s_dashboard.temperature_tiles[i], 1);
            lv_obj_set_height(s_dashboard.temperature_tiles[i], LV_PCT(100));
            static const char *const temp_names[5] = {"CVT", "FL", "FR", "BL", "BR"};
            (void)label_tile(s_dashboard.temperature_tiles[i], temp_names[i], -20);
            s_dashboard.temperature_value_labels[i] =
                label_tile(s_dashboard.temperature_tiles[i], "--", 20);
        }
    }

    lv_obj_t *fuel = lv_obj_create(lower);
    lv_obj_set_flex_grow(fuel, 1);
    lv_obj_set_height(fuel, LV_PCT(100));
    lv_obj_set_style_bg_color(fuel, lv_color_hex(TILE_BG), 0);
    lv_obj_set_style_bg_opa(fuel, LV_OPA_COVER, 0);
    lv_obj_set_style_radius(fuel, 12, 0);
    lv_obj_set_style_border_width(fuel, 2, 0);
    lv_obj_set_style_border_color(fuel, lv_color_hex(0x344653), 0);
    lv_obj_set_style_pad_all(fuel, 6, 0);
    lv_obj_set_flex_flow(fuel, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(fuel, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER,
                          LV_FLEX_ALIGN_CENTER);
    make_label(fuel, "FUEL", MUTED_COLOR, &lv_font_montserrat_20);
    s_dashboard.fuel_value = make_label(fuel, "--%", TEXT_COLOR, &lv_font_montserrat_32);
    s_dashboard.fuel_bar = lv_bar_create(fuel);
    lv_bar_set_orientation(s_dashboard.fuel_bar, LV_BAR_ORIENTATION_VERTICAL);
    lv_bar_set_range(s_dashboard.fuel_bar, 0, 100);
    lv_bar_set_value(s_dashboard.fuel_bar, 0, LV_ANIM_OFF);
    lv_obj_set_size(s_dashboard.fuel_bar, 24, LV_PCT(55));
    lv_obj_set_style_bg_color(s_dashboard.fuel_bar, lv_color_hex(0x344653), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(s_dashboard.fuel_bar, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_radius(s_dashboard.fuel_bar, 8, LV_PART_MAIN);
    lv_obj_set_style_bg_color(s_dashboard.fuel_bar, lv_color_hex(TACH_NORMAL_COLOR), LV_PART_INDICATOR);
    lv_obj_set_style_bg_opa(s_dashboard.fuel_bar, LV_OPA_COVER, LV_PART_INDICATOR);
    lv_obj_set_style_radius(s_dashboard.fuel_bar, 8, LV_PART_INDICATOR);
    s_dashboard.fuel_warning = make_label(fuel, "LOW", FUEL_WARNING_COLOR,
                                          &lv_font_montserrat_20);
    lv_obj_add_flag(s_dashboard.fuel_warning, LV_OBJ_FLAG_HIDDEN);

    resize_viewport(parent, viewport);
    lv_obj_add_event_cb(parent, parent_size_changed, LV_EVENT_SIZE_CHANGED, viewport);
    lv_obj_update_layout(viewport);
    return viewport;
}

void cockpit_graphics_update(const cockpit_graphics_data_t *data)
{
    if (data == NULL || s_dashboard.viewport == NULL) return;
    char text[32];
    float rpm_max = (isfinite(data->rpm_max) && data->rpm_max > 0.0f)
                        ? data->rpm_max : 8000.0f;
    if ((data->valid_mask & COCKPIT_GRAPHICS_VALID_RPM) && isfinite(data->engine_rpm)) {
        s_dashboard.rpm = (int32_t)fmaxf(0.0f, fminf(data->engine_rpm, rpm_max));
        lv_bar_set_range(s_dashboard.rpm_bar, 0, (int32_t)rpm_max);
        lv_bar_set_value(s_dashboard.rpm_bar, s_dashboard.rpm, LV_ANIM_OFF);
        (void)snprintf(text, sizeof text, "%d rpm", (int)s_dashboard.rpm);
        lv_label_set_text(s_dashboard.rpm_value, text);
        float ratio = (float)s_dashboard.rpm / rpm_max;
        uint32_t color = ratio >= 0.85f ? TACH_REDLINE
                         : ratio >= 0.60f ? TACH_WARN_COLOR
                                          : TACH_NORMAL_COLOR;
        lv_obj_set_style_bg_color(s_dashboard.rpm_bar, lv_color_hex(color), LV_PART_INDICATOR);
    } else {
        s_dashboard.rpm = 0;
        lv_bar_set_value(s_dashboard.rpm_bar, 0, LV_ANIM_OFF);
        lv_label_set_text(s_dashboard.rpm_value, "-- rpm");
    }
    if ((data->valid_mask & COCKPIT_GRAPHICS_VALID_SPEED) && isfinite(data->speed_mps))
        (void)snprintf(text, sizeof text, "%.0f km/h", (double)(data->speed_mps * 3.6f));
    else
        (void)snprintf(text, sizeof text, "-- km/h");
    lv_label_set_text(s_dashboard.speed_value, text);

    float stress_scale = (isfinite(data->stress_full_scale) && data->stress_full_scale > 0.0f)
                             ? data->stress_full_scale : 1000.0f;
    for (size_t i = 0; i < COCKPIT_GRAPHICS_STRESS_COUNT; i++) {
        uint16_t bit = (uint16_t)(COCKPIT_GRAPHICS_VALID_STRESS_1 << i);
        bool valid = (data->valid_mask & bit) && isfinite(data->stress[i]);
        bool danger = valid && data->stress[i] >= stress_scale * STRESS_DANGER_RATIO;
        lv_color_t border = valid
                                ? (danger ? lv_color_hex(FUEL_WARNING_COLOR)
                                          : stress_border_color(data->stress[i], stress_scale))
                                : lv_color_hex(0x344653);
        lv_color_t background = danger ? lv_color_hex(FUEL_WARNING_COLOR)
                                       : lv_color_hex(TILE_BG);
        lv_obj_set_style_bg_color(s_dashboard.stress_tiles[i], background, 0);
        lv_obj_set_style_bg_opa(s_dashboard.stress_tiles[i], LV_OPA_COVER, 0);
        lv_obj_set_style_border_color(s_dashboard.stress_tiles[i], border, 0);
        if (valid) (void)snprintf(text, sizeof text, "%.0f", (double)data->stress[i]);
        else (void)snprintf(text, sizeof text, "--");
        lv_label_set_text(s_dashboard.stress_value_labels[i], text);
    }

    static const uint16_t temp_bits[5] = {
        COCKPIT_GRAPHICS_VALID_CVT_TEMP,
        COCKPIT_GRAPHICS_VALID_BRAKE_FL, COCKPIT_GRAPHICS_VALID_BRAKE_FR,
        COCKPIT_GRAPHICS_VALID_BRAKE_BL, COCKPIT_GRAPHICS_VALID_BRAKE_BR};
    float temps[5] = {data->cvt_temp_c, data->brake_temp_c[0], data->brake_temp_c[1],
                      data->brake_temp_c[2], data->brake_temp_c[3]};
    for (size_t i = 0; i < 5; i++) {
        bool valid = (data->valid_mask & temp_bits[i]) && isfinite(temps[i]);
        bool danger = valid && temps[i] >= TEMP_DANGER_C;
        lv_color_t border = valid
                                ? (danger ? lv_color_hex(FUEL_WARNING_COLOR)
                                          : temperature_border_color(temps[i]))
                                : lv_color_hex(0x344653);
        lv_color_t background = danger ? lv_color_hex(FUEL_WARNING_COLOR)
                                       : lv_color_hex(TILE_BG);
        lv_obj_set_style_bg_color(s_dashboard.temperature_tiles[i], background, 0);
        lv_obj_set_style_bg_opa(s_dashboard.temperature_tiles[i], LV_OPA_COVER, 0);
        lv_obj_set_style_border_color(s_dashboard.temperature_tiles[i], border, 0);
        if (valid) (void)snprintf(text, sizeof text, "%.0f C", (double)temps[i]);
        else (void)snprintf(text, sizeof text, "-- C");
        lv_label_set_text(s_dashboard.temperature_value_labels[i], text);
    }

    if ((data->valid_mask & COCKPIT_GRAPHICS_VALID_FUEL) && isfinite(data->fuel_percent)) {
        float fuel = fmaxf(0.0f, fminf(data->fuel_percent, 100.0f));
        (void)snprintf(text, sizeof text, "%.0f%%", (double)fuel);
        lv_bar_set_value(s_dashboard.fuel_bar, (int32_t)fuel, LV_ANIM_OFF);
        lv_obj_set_style_text_color(s_dashboard.fuel_value,
                                    lv_color_hex(fuel <= 20.0f ? FUEL_WARNING_COLOR : TEXT_COLOR), 0);
        lv_obj_set_style_bg_color(s_dashboard.fuel_bar,
                                  lv_color_hex(fuel <= 20.0f ? FUEL_WARNING_COLOR : TACH_NORMAL_COLOR),
                                  LV_PART_INDICATOR);
        if (fuel <= 20.0f) lv_obj_clear_flag(s_dashboard.fuel_warning, LV_OBJ_FLAG_HIDDEN);
        else lv_obj_add_flag(s_dashboard.fuel_warning, LV_OBJ_FLAG_HIDDEN);
    } else {
        (void)snprintf(text, sizeof text, "--%%");
        lv_bar_set_value(s_dashboard.fuel_bar, 0, LV_ANIM_OFF);
        lv_obj_set_style_text_color(s_dashboard.fuel_value, lv_color_hex(MUTED_COLOR), 0);
        lv_obj_add_flag(s_dashboard.fuel_warning, LV_OBJ_FLAG_HIDDEN);
    }
    lv_label_set_text(s_dashboard.fuel_value, text);
}
