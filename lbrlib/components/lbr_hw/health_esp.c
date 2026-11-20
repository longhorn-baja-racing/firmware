#include <stdio.h>

#include "lbr/board.h"
#include "lbr/mark.h"
#include "lbr/health.h"

#include "esp_attr.h"
#include "esp_system.h"

/*
 * Boot bookkeeping that survives software resets (RTC slow memory, not
 * cleared by panic/watchdog/brownout restarts). A cold power-on clears it, so
 * the counter restarts from 1 after a power cycle; the reset reason tells
 * the two apart.
 */

#define BOOT_MAGIC 0x4C425242u /* "LBRB" */

static RTC_NOINIT_ATTR uint32_t s_boot_magic;
static RTC_NOINIT_ATTR uint32_t s_boot_count;
static uint32_t                 s_this_boot;

lbr_reset_reason_t lbr_health_reset_reason(void)
{
    switch (esp_reset_reason()) {
        case ESP_RST_POWERON:
            return LBR_RESET_POWER_ON;
        case ESP_RST_SW:
            return LBR_RESET_SOFTWARE;
        case ESP_RST_PANIC:
        case ESP_RST_CPU_LOCKUP: /* double exception */
            return LBR_RESET_PANIC;
        case ESP_RST_INT_WDT:
        case ESP_RST_TASK_WDT:
        case ESP_RST_WDT:
            return LBR_RESET_WATCHDOG;
        case ESP_RST_BROWNOUT:
        case ESP_RST_PWR_GLITCH: /* the C5's supply-glitch detector: a dip, like a brownout */
            return LBR_RESET_BROWNOUT;
        case ESP_RST_DEEPSLEEP:
            return LBR_RESET_DEEP_SLEEP;
        case ESP_RST_EXT:
            return LBR_RESET_EXTERNAL;
        default:
            return LBR_RESET_UNKNOWN;
    }
}

uint32_t lbr_health_boot_count(void)
{
    if (s_this_boot == 0) {
        if (s_boot_magic != BOOT_MAGIC) {
            s_boot_magic = BOOT_MAGIC;
            s_boot_count = 0;
        }
        s_this_boot = ++s_boot_count;
    }
    return s_this_boot;
}

void lbr_board_print_mark(void)
{
    switch (lbr_health_reset_reason()) {
        case LBR_RESET_PANIC:
        case LBR_RESET_WATCHDOG:
        case LBR_RESET_BROWNOUT:
            return; /* do not hold up recovery with console output */
        default:
            fputs(lbr_mark_art, stdout);
            fflush(stdout);
            return;
    }
}
