# Sensor nodes

A sensor node's firmware is one short file: `car/nodes/<node>/main/main.c`. The node's plan
(which slots, which signals, what rates) is generated from the catalog; `main.c` only says which
part is on which slot.

```c
#include "lbr.h"
#include "powertrain_plan.h"   /* generated: POWERTRAIN_SLOT_SPEEDS, lbr_plan_powertrain, ... */
#include "sdkconfig.h"

static void init(lbr_node *n)
{
    lbr_bind_io(n, POWERTRAIN_SLOT_SPEEDS, CONFIG_LBR_IO_SPEEDS);  /* pins + calibration from menuconfig */
    lbr_bind(n, POWERTRAIN_SLOT_FUEL, &fuel_sender, &fuel);        /* a part driver */
}

LBR_NODE_MAIN(&lbr_plan_powertrain, .init = init);
```

## What `LBR_NODE_MAIN` does

It defines `app_main` and, in order:

1. builds the configuration from the plan, giving every slot the *unbound* driver: `NOT_FITTED`
   on race builds, simulated data on bench builds (`CONFIG_LBR_SENSOR_SIM=y`, flagged in every
   heartbeat so it is never mistaken for a measurement);
2. calls your `init()` to bind the parts this node has;
3. validates everything: the plan against the frame table, every driver's channel count, no
   slot bound twice. A mistake puts the node in `CONFIG_INVALID`: it still runs CAN and the
   heartbeat, so the fault shows on the bus with its reason in the node's log, instead of the node
   quietly running without the slot;
4. rejects a firmware image meant for another node (after a CAN update), and confirms a new image
   once it is on the bus and synced, or the bootloader rolls it back;
5. starts CAN, time sync, one acquisition task per slot on the shared clock's grid, the receive
   task, and the heartbeat. It never returns.

From then on, every sample is timestamped on the shared clock, sequenced, packed into its CAN
frame, and sent; faults are reported as events and in the heartbeat. Your code is done.

## Binding parts

| Call | Use it for |
| --- | --- |
| `lbr_bind(n, SLOT, &driver, &state)` | a part with a driver ([drivers](drivers.md)); `state` must be `static` |
| `lbr_bind_io(n, SLOT, CONFIG_LBR_IO_<SLOT>)` | ADC, pulse, and switch inputs configured in menuconfig, no code ([drivers](drivers.md#inputs-that-need-no-code)); an empty setting leaves the slot unbound |
| `lbr_slot(n, SLOT)` | the raw `lbr_sensor_t` of a slot, for drivers written against the low-level interface |

Leave a slot alone and it reports `NOT_FITTED`. Binding a slot twice, a slot the plan does not
have, or a driver whose channel count differs from the slot's makes the configuration invalid
with the reason.

`LBR_NODE_MAIN` takes, after the plan, any of `.init`, and for `listen_all` plans `.on_frame`
and `.on_frame_ctx` (every received frame the runtime does not consume itself).

## A firmware for several nodes

The four corner hubs run one firmware. Their plans are an array in id order, and a Kconfig
choice picks the corner at build time:

```c
LBR_NODE_MAIN(&lbr_plan_corner[CONFIG_LBR_CORNER_POSITION], .init = init);
```

Each corner board is built with its position (`positions/<xx>.defaults`) and its node id is
stored on the board, so a corner image cannot be flashed onto the wrong corner over CAN.

## Adding a node

1. Add it and its frames to `car/catalog.toml`, run `lbrgen.py`.
2. Copy an existing `car/nodes/<node>/` directory (top-level `CMakeLists.txt`, `sdkconfig.defaults`
   with the chip, `main/`), rename the project, and write its `main.c`.
3. Add its plan to `LBR_NODE_PLANS` in `dev/tests/host/CMakeLists.txt` and to `test_plans.c`, and
   a row in the simulator's node table (`dev/sim/sim.c`).
4. Add it to the CI firmware matrix and `dev/tools/lbr_deploy.py`.

## Pins

Pins are Kconfig settings that default to `-1` (unset) until the schematics confirm them; the
firmware refuses to drive an unset pin and reports it (`CAN_PINS_UNSET`, or a slot's init
failure). Never hard-code a GPIO. See [`docs/hardware.md`](../../docs/hardware.md).
