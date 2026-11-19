#pragma once

#include <stddef.h>

#include "lbr/logic.h"
#include "lbr/node.h"

/*
 * Writing the Central's firmware (car/nodes/central/main/main.c). The whole
 * app is:
 *
 *     #include "car_logic.h"
 *     #include "central_plan.h"      // generated from car/catalog.toml
 *     #include "lbr.h"
 *
 *     LBR_CENTRAL_MAIN(&lbr_plan_central, .logic = &car_logic);
 *
 * The Central (README "Central Controller"):
 *   - sniffs the whole CAN bus and writes the authoritative, time-aligned,
 *     full-fidelity microSD log (every frame, raw, with its receive time);
 *   - is the time master: it broadcasts TIME_SYNC + FOLLOW_UP;
 *   - samples its own slots (the GPS, the cockpit link state) into CAN frames,
 *     each bound as the catalog says (slot "input");
 *   - runs the car logic tasks (lbr/logic.h) and logs their outputs;
 *   - drives the live links: LoRa in-race subset, USB / WiFi full stream, BLE
 *     status, display data to the cockpit over WiFi;
 *   - tracks node presence and health, and reports missing and lost nodes.
 * Every path between them is a bounded, non-blocking queue: a slow or absent
 * live link never reduces what reaches the SD card. All decisions are in the
 * pure gateway core (lbr/gw_core.h), which the simulator runs unchanged.
 */

typedef struct {
    const lbr_node_plan_t *plan;  /* generated: lbr_plan_central */
    const lbr_task_list   *logic; /* the car logic (LBR_TASK_LIST); NULL for none */
} lbr_central_app_t;

#define LBR_CENTRAL_MAIN(plan_, ...)                                                               \
    void app_main(void);                                                                           \
    void app_main(void)                                                                            \
    {                                                                                              \
        static const lbr_central_app_t lbr_app_ = {.plan = (plan_), __VA_ARGS__};                  \
        lbr_central_main(&lbr_app_);                                                               \
    }                                                                                              \
    static_assert(true, "LBR_CENTRAL_MAIN takes a semicolon")

/* Start every Central service and run them. Never returns. */
[[noreturn]] void lbr_central_main(const lbr_central_app_t *app);
