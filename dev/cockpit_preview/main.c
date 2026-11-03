#define SDL_MAIN_HANDLED

#include <math.h>
#include <stddef.h>

#include "display_graphics.h"
#include "lvgl.h"

int main(void)
{
    lv_init();
    if (lv_sdl_window_create(960, 720) == NULL) {
        return 1;
    }

    lv_obj_t *screen = lv_screen_active();
    (void)cockpit_graphics_create(screen);

    /* Deliberately mixed readings exercise every dashboard color and warning. */
    cockpit_graphics_data_t sample = {
        .engine_rpm = 5200.0f,
        .rpm_max = 8000.0f,
        .speed_mps = 12.3f,
        .cvt_temp_c = 94.0f,
        .brake_temp_c = {72.0f, 88.0f, 101.0f, 110.0f},
        .fuel_percent = 18.0f,
        .stress = {80.0f, 220.0f, 360.0f, 470.0f, 610.0f, 740.0f, 860.0f, 980.0f},
        .stress_full_scale = 1000.0f,
        .valid_mask = 0xFFFFu,
    };

    uint32_t start_ms = lv_tick_get();
    for (;;) {
        float elapsed_s = (float)(lv_tick_get() - start_ms) / 1000.0f;
        /* A slow sine sweep moves the RPM needle through the dial and varies
         * speed alongside it. Independent sensor sweeps exercise the colors
         * and warning states across the rest of the dashboard. */
        sample.engine_rpm = 4200.0f + 3400.0f * sinf(elapsed_s * 0.7f);
        sample.speed_mps = 10.0f + 8.0f * sinf(elapsed_s * 0.32f - 0.8f);
        sample.cvt_temp_c = 78.0f + 30.0f * (0.5f + 0.5f * sinf(elapsed_s * 0.21f));
        for (size_t i = 0; i < COCKPIT_GRAPHICS_BRAKE_COUNT; i++) {
            sample.brake_temp_c[i] =
                45.0f + 75.0f * (0.5f + 0.5f * sinf(elapsed_s * 0.17f + (float)i * 1.1f));
        }
        sample.fuel_percent = 45.0f + 35.0f * sinf(elapsed_s * 0.4f);
        for (size_t i = 0; i < COCKPIT_GRAPHICS_STRESS_COUNT; i++) {
            sample.stress[i] =
                500.0f + 470.0f * sinf(elapsed_s * 0.38f + (float)i * 0.72f);
        }
        cockpit_graphics_update(&sample);
        lv_timer_handler();
        lv_delay_ms(20);
    }
}
