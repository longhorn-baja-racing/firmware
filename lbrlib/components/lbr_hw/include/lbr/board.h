#pragma once

#include <stdbool.h>
#include <stdint.h>

#include "lbr/health.h"
#if __has_include("sdkconfig.h")
#include "sdkconfig.h"
#endif
#include "lbr/node.h"
#include "lbr/timebase.h"

/*
 * This board's hardware services (ESP32): its one shared clock, and why and
 * how often it booted. Board code (the roles, the ESP glue) uses these; the
 * pure layers get time and identity through their HAL tables instead, so they
 * run unchanged in host tests and the simulator.
 */

/* ------------------------------------------------------------------------ */
/* The board's shared clock (an lbr_clock_t on esp_timer, lbr/timebase.h)    */

/* Raw local monotonic microseconds (esp_timer). */
uint64_t lbr_time_mono_us(void);

/* Become the time master (the Central). */
void lbr_time_set_master(void);

/* Discipline the clock with one sync measurement. */
lbr_time_update_t lbr_time_sync_apply(uint64_t gateway_us, uint64_t local_rx_us, uint16_t epoch);

/* Disciplined shared time: use this to stamp samples. */
uint64_t lbr_time_now_us(void);

/* Convert a captured local timestamp to shared time. */
uint64_t lbr_time_local_to_shared(uint64_t local_us);

/* True once a sync frame has disciplined this board (or it is the master). */
bool lbr_time_is_synced(void);

/* Snapshot of the servo state for health reporting. */
void lbr_time_get_servo(lbr_time_servo_t *out);

/* ------------------------------------------------------------------------ */
/* Boot                                                                      */

/* Reset reason of the current boot, mapped to lbr_reset_reason_t. */
lbr_reset_reason_t lbr_health_reset_reason(void);

/* Persistent boot counter, incremented once per boot. */
uint32_t lbr_health_boot_count(void);

/* Print the programmer's mark (lbr/mark.h) on the console at boot. Skipped
 * after a panic, watchdog, or brownout reset so recovery is not held up. */
void lbr_board_print_mark(void);

/* ------------------------------------------------------------------------ */
/* On-chip inputs                                                            */

/*
 * Bind the part-independent input driver (lbr/sensor_io.h: ADC, pulse, and
 * switch inputs) to slot `slot` from a menuconfig spec string. An empty spec
 * leaves the slot unbound (NOT_FITTED); a bad one fails the sensor's init,
 * which is reported as a sensor fault. The board roles call this for every
 * slot the catalog marks input = "io".
 */
void lbr_bind_io(lbr_node *n, size_t slot, const char *spec);

/*
 * The driver every slot starts with until something binds it: NOT_FITTED on
 * race builds, the simulator on bench builds (CONFIG_LBR_SENSOR_SIM=y,
 * flagged in every heartbeat). The simulator is not even compiled into a
 * race build.
 */
#if CONFIG_LBR_SENSOR_SIM
#define LBR_UNBOUND_DRIVER lbr_sensor_sim
#else
#define LBR_UNBOUND_DRIVER lbr_sensor_unconfigured
#endif
