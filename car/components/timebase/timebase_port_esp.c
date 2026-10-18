#include "lbr/timebase.h"

#include "esp_timer.h"
#include "freertos/FreeRTOS.h"

/* Spinlock: the servo is touched from tasks on both cores. */
static portMUX_TYPE s_lock = portMUX_INITIALIZER_UNLOCKED;

uint64_t lbr_time_port_mono_us(void)
{
    return (uint64_t)esp_timer_get_time();
}

void lbr_time_port_lock(void)
{
    portENTER_CRITICAL(&s_lock);
}

void lbr_time_port_unlock(void)
{
    portEXIT_CRITICAL(&s_lock);
}
