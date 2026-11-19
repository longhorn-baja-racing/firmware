# Getting started

This walks you from a fresh clone to a new signal, read by a new driver, feeding a new piece of
car logic, checked in the simulator. No ESP32 needed until the last step.

## 1. Build and test on your laptop

You need CMake 3.21+, Ninja, gcc 14+ or clang 19+ (C23), and Python 3.11+.

```sh
cmake -S dev -B build-dev -G Ninja
cmake --build build-dev
ctest --test-dir build-dev --output-on-failure
```

Every test should pass. These run the real library code (everything except the `*_esp.c`
hardware glue) under AddressSanitizer and UndefinedBehaviorSanitizer.

## 2. Watch the car in the simulator

```sh
python3 dev/sim/ui/server.py          # then open http://127.0.0.1:8765/
```

You see every node on the car, the CAN bus, and the cockpit display. Click a node and unplug its
CAN cable: within 3 seconds the Central reports it lost, and every value that came from it reads
`NODE_LOST`. Plug it back in and it is found again. The simulator runs the same code the boards
run; only the hardware is pretend.

## 3. Add a signal

Say the powertrain controller gets an oil pressure sender. Open `car/catalog.toml`, find the
powertrain's `FUEL_LEVEL` frame, and add a frame for it:

```toml
[[frame]]
id = 0x202
name = "OIL_PRESSURE"
sender = "powertrain"
slot = "oil"
rate_hz = 10
signals = [
  { name = "OIL_PRESSURE", id = 0x0203, unit = "none", enc = "u16", scale = 0.01, desc = "Engine oil pressure, bar" },
]
```

and give the powertrain node that slot. The sender is a plain voltage on an ADC pin, so the slot
is read by on-chip inputs:

```toml
slots = [
  { name = "speeds", input = "io", hint = "pulse inputs (pickup or hall sensor)" },
  { name = "fuel", input = "io", hint = "an adc input (fuel sender)" },
  { name = "oil", input = "io", hint = "an adc input (0.5..4.5 V pressure sender)" },
]
```

Then regenerate:

```sh
python3 lbrlib/tools/lbrgen.py car/catalog.toml
```

The generator checks your entry (unique ids, the fields fit the frame, the rate divides one
second, the slot exists) and rewrites the generated C, the powertrain's plan, its menuconfig
settings (`main/Kconfig`, now with `CONFIG_LBR_IO_OIL`), and the tables in `docs/can-ids.md`. Build and run `ctest` again: `test_plans` checks the whole bus still fits its
bandwidth budget. (A real catalog change also means regenerating the golden fixtures and
updating the dashboard app; see [the catalog](catalog.md).)

The powertrain now publishes `OIL_PRESSURE`, reporting `NOT_FITTED` until its input is set.

## 4. Read it: no code, or a driver

For an ADC, pulse, or switch input you write no code at all. Set the pin and calibration in
`idf.py menuconfig` (menu "LBR powertrain inputs", `CONFIG_LBR_IO_OIL`):

```text
adc gpio=35 mv0=500 v0=0 mv1=4500 v1=10 lo=300 hi=4700
```

The powertrain binds it by itself at boot; `main.c` stays `LBR_NODE_MAIN(&lbr_plan_powertrain);`.
An open or shorted sender reads `SENSOR_FAULT` (outside `lo`..`hi`). See
[drivers](drivers.md#inputs-that-need-no-code).

For a part with its own protocol (I2C, SPI, a UART), leave `input` out of the slot and write a
driver: two functions and a table.

```c
typedef struct { int pin; } oil_sender_t;

static int oil_start(void *self) { return my_adc_setup(((oil_sender_t *)self)->pin); }

static int oil_read(void *self, lbr_channels *ch)
{
    float volts;
    if (my_adc_read(((oil_sender_t *)self)->pin, &volts) != 0) {
        lbr_ch_missing_all(ch, LBR_UNAVAIL_BUS_ERROR);
        return 0;
    }
    lbr_ch_f32(ch, 0, (volts - 0.5f) * 2.5f);   /* bar */
    return 0;
}

const lbr_driver oil_sender = {.part = "oil_sender", .channels = 1,
                               .start = oil_start, .read = oil_read};

static oil_sender_t oil = {.pin = 34};

static void init(lbr_node *n)
{
    lbr_bind(n, POWERTRAIN_SLOT_OIL, &oil_sender, &oil);
}

LBR_NODE_MAIN(&lbr_plan_powertrain, .init = init);
```

The library samples it at 10 Hz on the shared clock, sequences it, packs it into frame 0x202,
and reports a fault on the bus if the part stops answering. See [drivers](drivers.md).

## 5. Use it: car logic

Make an alarm for low oil pressure with the engine running: on below 0.8 bar, off again above
1.0 bar so it does not flicker. Create `car/logic/oil.c`:

```c
#include "car_logic.h"

LBR_OUTPUT(low_oil, "Alarms/LowOil", BOOL, NONE);

static bool s_low;

static void oil_periodic(lbr_cycle *c)
{
    lbr_bool running = lbr_gt(lbr_get(c, ENGINE_RPM), 1500.0f);
    lbr_bool low     = lbr_below(&s_low, lbr_get(c, OIL_PRESSURE), 0.8f, 1.0f); /* bar */
    lbr_out_bool(c, &low_oil, lbr_and(running, low));
}

LBR_TASK(oil, .rate_hz = 10, .periodic = oil_periodic);
```

If the engine is known to be off, the alarm is a plain "no". If the oil sensor or the
powertrain board is gone while the engine runs, the alarm is logged as unknown, with the
reason, never as "fine". Declare the task in `car/logic/include/car_logic.h` (`extern const lbr_task oil;`), add `&oil` to
the `LBR_TASK_LIST` in `car/logic/tasks.c`, and add `"oil.c"` to `car/logic/CMakeLists.txt`. Done: the Central runs it,
and `Alarms/LowOil` is in every log from now on. See [car logic](logic.md).

## 6. Test it in the simulator

Scenarios are short scripts in `dev/sim/scenarios/`. A check on an output looks like:

```text
at 4.0  expect out:Alarms/LowOil value == 0
at 4.0  set OIL_PRESSURE 0.5
at 5.0  expect out:Alarms/LowOil value == 1
at 5.0  disconnect powertrain
at 9.0  expect out:Alarms/LowOil status == NODE_LOST
end 9.0
```

Every scenario runs in `ctest` (as `sim_<name>`) and in CI. `dev/sim/scenarios/car_logic.sim`
is a complete example.

## 7. Look at the data

The simulated Central writes a real SD log. Export it like a log pulled off the car:

```sh
build-dev/sim/lbr_sim --out /tmp/run dev/sim/scenarios/car_logic.sim
python3 dev/tools/lbr_decode.py csv /tmp/run/00000001.lbrlog
```

Every sample, event, health field, and car logic output is a row; the dashboard app plots them.
See [logging and data](logging.md).

## 8. Flash a board

With ESP-IDF v6.1 exported:

```sh
cd car/nodes/powertrain
idf.py build
idf.py -p /dev/ttyUSB0 flash monitor
```

Pins are `menuconfig` settings that default to unset; the firmware refuses to drive an unset pin
and says so. Putting a release on the whole car is [`docs/deploying.md`](../../docs/deploying.md).
