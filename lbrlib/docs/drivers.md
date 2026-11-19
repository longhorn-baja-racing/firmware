# Drivers

A driver knows how to talk to one part. It never deals with time, sequence numbers, CAN, or
logging: the node runtime calls it at the slot's rate on the shared clock and does the rest.

## Inputs that need no code

Many sensors reach a node as a voltage, a pulse train, or a switch. Those are wired by
configuration alone, with no C at all:

1. mark the slot `input = "io"` in `car/catalog.toml` (`hint` says what is usually wired there);
2. run `lbrgen.py`: it writes the node's `main/Kconfig` with a `CONFIG_LBR_IO_<SLOT>` setting
   (menu "LBR <node> inputs") that lists the slot's channels in order;
3. set it in `idf.py menuconfig`, one `;`-separated entry per channel:

```text
adc gpio=34 mv0=310 v0=0 mv1=2980 v1=100      two-point linear calibration (pedals, fuel sender)
adc gpio=36 curve=ntc r25=10000 beta=3950     NTC thermistor
pulse gpio=27 scale=60 stall_ms=1000          frequency x scale (engine or CVT speed)
din gpio=4 active=low pull=up                 a switch
-                                             not fitted
```

The board binds the slot by itself; an empty setting leaves it `NOT_FITTED`. The setting is the
channel's calibration and is recorded in every session. The full grammar, the per-chip pin
rules, and the fault windows (`lo=`/`hi=` for open or shorted sensors, `min=`/`max=` for
out-of-range values) are in [`docs/sensors.md`](../../docs/sensors.md).

## Writing a driver

A driver is an `lbr_driver` table and two functions working on a state struct you own:

```c
#include "lbr.h"

typedef struct {
    int     i2c_port;
    uint8_t addr;
} bmi270_t;

static int bmi270_start(void *self)
{
    bmi270_t *imu = self;
    /* reset, check the chip id, configure ranges and the output data rate */
    return bmi270_init_registers(imu->i2c_port, imu->addr);   /* 0, or <0: not responding */
}

static int bmi270_read(void *self, lbr_channels *ch)
{
    bmi270_t *imu = self;
    int16_t   raw[6];
    if (bmi270_burst_read(imu->i2c_port, imu->addr, raw) != 0) {
        lbr_ch_missing_all(ch, LBR_UNAVAIL_BUS_ERROR);   /* one bad transaction */
        return 0;
    }
    for (size_t i = 0; i < 3; i++) {
        lbr_ch_f32(ch, i, raw[i] * ACCEL_M_PER_S2_PER_LSB);       /* accel X, Y, Z */
        lbr_ch_f32(ch, 3 + i, raw[3 + i] * GYRO_RAD_PER_S_PER_LSB); /* gyro X, Y, Z */
    }
    return 0;
}

const lbr_driver bmi270 = {
    .part     = "bmi270",
    .channels = 6,
    .start    = bmi270_start,
    .read     = bmi270_read,
};
```

and in the node's `init()`:

```c
static bmi270_t chassis_imu = {.i2c_port = 0, .addr = 0x68};
lbr_bind(n, IMU_SLOT_CHASSIS, &bmi270, &chassis_imu);
```

### The contract

| | |
| --- | --- |
| `.part` | the part name; recorded in every session's configuration (`sensor.<i>.params=`) |
| `.channels` | how many channels it produces. Must equal the slot's signal count, or the node is `CONFIG_INVALID` |
| `.start(self)` | bring the part up. Return 0, or <0 if it does not respond: every channel then reads `SENSOR_FAULT`, a `SENSOR_INIT_FAIL` fault goes on the bus, and the runtime retries every 2 s (`SENSOR_RECOVERED` when it comes back) |
| `.read(self, ch)` | one sample of every channel. Return 0, or <0 if the part stopped answering (handled like a failed start) |
| `.calibrated(self)` | optional: false while the part's calibration is missing; reported in the heartbeat |

Inside `read()`:

| Helper | |
| --- | --- |
| `lbr_ch_f32(ch, i, v)`, `lbr_ch_i32`, `lbr_ch_u32`, `lbr_ch_bool` | set channel i |
| `lbr_ch_missing(ch, i, why)` | channel i has no value this time, and why |
| `lbr_ch_missing_all(ch, why)` | every channel |
| `lbr_ch_count(ch)`, `lbr_ch_time_us(ch)` | the channel count; the sample time on the shared clock |

Channels are the slot's signals in catalog order (`ch->signals[i]` says which). A channel
`read()` does not set reads `NO_DATA`, so forgetting one is visible, never a zero or a stale
value.

### Rules

- **Report the catalog unit, in the part's own axes.** Convert raw counts in the driver;
  mounting transforms are session metadata the dashboard applies, never firmware.
- **Never invent a value.** No fresh, trustworthy reading means `lbr_ch_missing` with the reason
  that fits: `NO_DATA` (nothing new yet), `BUS_ERROR` (a transaction failed),
  `OUT_OF_RANGE` (physically impossible), `NOT_CALIBRATED`, `SENSOR_FAULT`.
- **Be quick.** `read()` runs once per sample period and must finish well inside it. Never wait
  on a bus without a timeout. Missed periods are counted as overruns.
- **Allocate nothing.** State lives in your struct, declared `static` in `main.c`.
- **Pins come from Kconfig**, defaulting to unset; refuse to start (`start()` < 0) while unset.
- **Keep the math testable.** Put raw-to-unit conversions in plain functions with no ESP-IDF
  calls, so a host test can check them.

### Where drivers live

A driver used by one node can live in that node's `main/`. A driver for a part several nodes
use goes in LBRLib: its hardware access in `lbrlib/components/lbr_hw/sensor_<part>_esp.c`, any
pure conversion math in `lbrlib/components/lbr_core/sensors/` (so host tests can check it),
and its `lbr_driver` table declared in a header in `lbr_hw/include/lbr/`. Both components build
every `.c` file in their directories; after adding one, run `idf.py reconfigure` (or a clean
build) so ESP-IDF sees it. The host build picks it up by itself.

## The low-level interface

`lbr_driver` sits on top of `lbr_sensor_t` (`lbr/sensor.h`), the interface the runtime calls.
The built-in drivers (`lbr_sensor_io`, the NMEA GPS, the simulator) implement it directly.
New drivers should use `lbr_driver`; `lbr_slot(n, SLOT)` hands you the raw sensor if you need it.
