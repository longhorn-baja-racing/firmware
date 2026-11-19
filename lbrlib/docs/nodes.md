# Boards

Every board's firmware is one short `main.c` in `car/nodes/<node>/main/`. What a board has
(its slots, their signals and rates, how each slot is read) is declared in `car/catalog.toml`
and generated into its plan; the app only names its role, and binds part drivers if it has any.

| Board | Role | The whole `main.c` |
| --- | --- | --- |
| sensor nodes | `LBR_NODE_MAIN` | `LBR_NODE_MAIN(&lbr_plan_powertrain);` |
| the Central | `LBR_CENTRAL_MAIN` | `LBR_CENTRAL_MAIN(&lbr_plan_central, .logic = &car_logic);` |
| the cockpit | `LBR_COCKPIT_MAIN` | `LBR_COCKPIT_MAIN(&lbr_plan_cockpit, .display = &cockpit_console);` |

plus `#include "lbr.h"` and the node's generated `<node>_plan.h`.

## Sensor nodes

```c
#include "lbr.h"
#include "powertrain_plan.h"

LBR_NODE_MAIN(&lbr_plan_powertrain);
```

`LBR_NODE_MAIN` defines `app_main` and, in order:

1. builds the configuration from the plan, giving every slot the *unbound* driver: `NOT_FITTED`
   on race builds, simulated data on bench builds (`CONFIG_LBR_SENSOR_SIM=y`, flagged in every
   heartbeat so it is never mistaken for a measurement);
2. binds every slot the catalog marks `input = "io"` from menuconfig (`CONFIG_LBR_IO_<SLOT>`);
3. calls your `init()`, if any, to bind part drivers;
4. validates everything: the plan against the frame table, every driver's channel count, no slot
   bound twice. A mistake puts the node in `CONFIG_INVALID`: it still runs CAN and the heartbeat,
   so the fault shows on the bus with its reason in the node's log, instead of the node quietly
   running without the slot;
5. rejects a firmware image meant for another node (after a CAN update), and confirms a new image
   once it is on the bus and synced, or the bootloader rolls it back;
6. starts CAN, time sync, one acquisition task per slot on the shared clock's grid, the receive
   task, and the heartbeat. It never returns.

From then on, every sample is timestamped on the shared clock, sequenced, packed into its CAN
frame, and sent; faults are reported as events and in the heartbeat.

### Binding a part driver

A slot with no `input` in the catalog is read by a part driver you bind in `init()`:

```c
static bmi270_t chassis_imu = {.i2c_port = 0, .addr = 0x68};

static void init(lbr_node *n)
{
    lbr_bind(n, IMU_SLOT_CHASSIS, &bmi270, &chassis_imu);
}

LBR_NODE_MAIN(&lbr_plan_imu, .init = init);
```

| Call | |
| --- | --- |
| `lbr_bind(n, SLOT, &driver, &state)` | run a slot with a part driver ([drivers](drivers.md)); `state` must be `static` |
| `lbr_slot(n, SLOT)` | the raw `lbr_sensor_t` of a slot, for drivers written against the low-level interface |

Binding a slot twice, a slot the plan does not have, or a driver whose channel count differs
from the slot's makes the configuration invalid with the reason. A slot nothing binds reports
`NOT_FITTED`. `LBR_NODE_MAIN` also takes `.on_frame` and `.on_frame_ctx` for `listen_all`
plans (every received frame the runtime does not consume itself).

### A firmware for several nodes

The four corner hubs run one firmware. Their plans are an array in id order, and a Kconfig choice
picks the corner at build time:

```c
LBR_NODE_MAIN(&lbr_plan_corner[CONFIG_LBR_CORNER_POSITION]);
```

Each corner board is built with its position (`positions/<xx>.defaults`) and its node id is
stored on the board, so a corner image cannot be flashed onto the wrong corner over CAN.

## The Central

```c
#include "car_logic.h"
#include "central_plan.h"
#include "lbr.h"

LBR_CENTRAL_MAIN(&lbr_plan_central, .logic = &car_logic);
```

The Central role (`lbrlib/components/lbr_central`) is the time master and the recorder. It writes
the authoritative SD log, samples its own slots (the GPS, `input = "gps"`, and the cockpit's
battery and link, `input = "cockpit_link"`), runs the car logic ([car logic](logic.md)), drives the
LoRa, USB, WiFi, and BLE links and the cockpit link, and reports missing and lost nodes. Its
settings (GPS UART pins, the links, the SD card) are in menuconfig.

## The cockpit

```c
#include "cockpit_plan.h"
#include "display.h"
#include "lbr.h"

LBR_COCKPIT_MAIN(&lbr_plan_cockpit, .display = &cockpit_console);
```

The cockpit role (`lbrlib/components/lbr_cockpit`) joins the Central's access point, keeps the
values and node status the Central sends, reports its own slots (its battery, `input = "io"`) back,
and calls the display driver to draw. A display driver is a small table, like a part driver:

```c
const lbr_display cockpit_console = {
    .panel     = "console",
    .start     = console_start,     /* bring the panel up */
    .render    = console_render,    /* draw the values and node status */
    .period_ms = 500,
};
```

`render` gets the latest value of each display signal with its state (`OK`, `STALE`, `INVALID`,
`NONE`), each CAN node's status, and whether the Central is heard. It runs in its own task, so a
slow panel never holds up reception.

## Adding a node

1. Add it and its frames to `car/catalog.toml`, run `lbrgen.py`.
2. Copy an existing `car/nodes/<node>/` directory (top-level `CMakeLists.txt`, `sdkconfig.defaults`
   with the chip, `main/`), rename the project, and point `main.c` at the new plan.
3. Add its plan to `LBR_NODE_PLANS` in `dev/tests/host/CMakeLists.txt` and to `test_plans.c`, and
   a row in the simulator's node table (`dev/sim/sim_core.c`).
4. Add it to the CI firmware matrix and `dev/tools/lbr_deploy.py`.

## Pins

Pins are Kconfig settings that default to `-1` (unset) until the schematics confirm them; the
firmware refuses to drive an unset pin and reports it (`CAN_PINS_UNSET`, or a slot's init
failure). Never hard-code a GPIO. See [`docs/hardware.md`](../../docs/hardware.md).
