#pragma once

#include "lbr/driver.h"
#include "lbr/node.h"
#if __has_include("sdkconfig.h")
#include "sdkconfig.h" /* firmware builds; host builds have no bench simulator option */
#endif

/*
 * Writing a sensor node (car/nodes/<node>/main/main.c). A whole node is:
 *
 *     #include "lbr/node_app.h"
 *     #include "powertrain_plan.h"          // generated from car/catalog.toml
 *
 *     static void init(lbr_node *n)
 *     {
 *         lbr_bind_io(n, POWERTRAIN_SLOT_SPEEDS, CONFIG_LBR_IO_SPEEDS);
 *         lbr_sensor_mypart(lbr_slot(n, POWERTRAIN_SLOT_FUEL), ...);
 *     }
 *
 *     LBR_NODE_MAIN(&lbr_plan_powertrain, .init = init);
 *
 * LBR_NODE_MAIN defines app_main: it builds the configuration from the
 * plan, gives every slot the unbound driver, lets init() bind the parts the
 * node has, validates everything, and runs the shared node runtime
 * (acquisition on the shared clock, CAN publishing, time sync, heartbeats,
 * fault reporting). It never returns. Slots init() leaves alone report
 * NOT_FITTED; a mistake in init() (a slot bound twice, a wrong slot, a
 * driver whose signals do not match its slot) puts the node in
 * CONFIG_INVALID with the reason in its log and heartbeat.
 */

typedef struct {
    const lbr_node_plan_t *plan;  /* generated: lbr_plan_<node> */
    void (*init)(lbr_node *node); /* bind drivers; optional */
    lbr_node_frame_cb on_frame;   /* listen_all plans only */
    void             *on_frame_ctx;
} lbr_node_app_t;

#define LBR_NODE_MAIN(plan_, ...)                                                                  \
    void app_main(void);                                                                           \
    void app_main(void)                                                                            \
    {                                                                                              \
        static const lbr_node_app_t lbr_app_ = {.plan = (plan_), __VA_ARGS__};                     \
        lbr_node_main(&lbr_app_);                                                                  \
    }                                                                                              \
    static_assert(true, "LBR_NODE_MAIN takes a semicolon")

/* Build, bind (app->init), validate, and run the node. Never returns. */
[[noreturn]] void lbr_node_main(const lbr_node_app_t *app);

/*
 * Bind the part-independent input driver (lbr/sensor_io.h: ADC, pulse, and
 * switch inputs) to slot `slot` from a menuconfig spec string. An empty spec
 * leaves the slot unbound (NOT_FITTED); a bad one fails the sensor's init,
 * which is reported as a sensor fault.
 */
void lbr_bind_io(lbr_node *n, size_t slot, const char *spec);

/*
 * The driver every slot gets until a real part is bound: NOT_FITTED on race
 * builds, the simulator on bench builds (CONFIG_LBR_SENSOR_SIM=y, flagged in
 * every heartbeat). The simulator is not even compiled into a race build.
 */
#if CONFIG_LBR_SENSOR_SIM
#define LBR_NODE_UNBOUND_DRIVER lbr_sensor_sim
#else
#define LBR_NODE_UNBOUND_DRIVER lbr_sensor_unconfigured
#endif
