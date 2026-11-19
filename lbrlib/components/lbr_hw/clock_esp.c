#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "lbr/board.h"

/* The board's one shared clock: an lbr_clock_t (lbr/timebase.h) on esp_timer. */

/* Spinlock: the servo is touched from tasks on both cores. */
static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;

static uint64_t mono_us(void)
{
    return (uint64_t)esp_timer_get_time();
}

static void lock(void)
{
    portENTER_CRITICAL(&s_lock);
}

static void unlock(void)
{
    portEXIT_CRITICAL(&s_lock);
}

/* A zero servo is a valid, unsynced one, so no init call is needed at boot. */
static lbr_clock_t s_clock = {.mono_us = mono_us, .lock = lock, .unlock = unlock};

uint64_t lbr_time_mono_us(void)
{
    return mono_us();
}

void lbr_time_set_master(void)
{
    lbr_clock_set_master(&s_clock);
}

lbr_time_update_t lbr_time_sync_apply(uint64_t gateway_us, uint64_t local_rx_us, uint16_t epoch)
{
    return lbr_clock_sync_apply(&s_clock, gateway_us, local_rx_us, epoch);
}

uint64_t lbr_time_now_us(void)
{
    return lbr_clock_now_us(&s_clock);
}

uint64_t lbr_time_local_to_shared(uint64_t local_us)
{
    return lbr_clock_local_to_shared(&s_clock, local_us);
}

bool lbr_time_is_synced(void)
{
    return lbr_clock_is_synced(&s_clock);
}

void lbr_time_get_servo(lbr_time_servo_t *out)
{
    lbr_clock_get_servo(&s_clock, out);
}
