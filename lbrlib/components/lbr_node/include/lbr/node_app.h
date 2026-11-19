#pragma once

#include "lbr/board.h"
#include "lbr/driver.h"
#include "lbr/node.h"

/*
 * Writing a sensor node (car/nodes/<node>/main/main.c). A whole node is:
 *
 *     #include "lbr.h"
 *     #include "powertrain_plan.h"          // generated from car/catalog.toml
 *
 *     LBR_NODE_MAIN(&lbr_plan_powertrain);
 *
 * and, for a slot read by a part driver (lbr/driver.h):
 *
 *     static bmi270_t chassis = {.addr = 0x68};
 *     static void init(lbr_node *n) { lbr_bind(n, IMU_SLOT_CHASSIS, &bmi270, &chassis); }
 *     LBR_NODE_MAIN(&lbr_plan_imu, .init = init);
 *
 * LBR_NODE_MAIN defines app_main: it builds the configuration from the plan,
 * binds every slot the catalog marks input = "io" from menuconfig
 * (CONFIG_LBR_IO_<SLOT>), lets init() bind the part drivers, validates
 * everything, and runs the shared node runtime (acquisition on the shared
 * clock, CAN publishing, time sync, heartbeats, fault reporting). It never
 * returns. Slots left unbound report NOT_FITTED; a mistake in init() (a slot
 * bound twice, a wrong slot, a driver whose channel count does not match)
 * puts the node in CONFIG_INVALID with the reason in its log and heartbeat.
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
