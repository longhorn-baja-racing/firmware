# Adding a sensor driver

Sensor parts are chosen per project as the electronics settle. LBRLib is built so that a new
part touches exactly one driver file and one line in its node's `main.c`.

## How a node is put together

```text
car/catalog.toml                      nodes, signals, frames, slots: declared once
car/nodes/<node>/main/<node>_plan.c   generated from the catalog (pure data; do not edit)
car/nodes/<node>/main/main.c          binds a driver to each slot that has a part
lbrlib/components/lbr_core/sensors/   driver kit + part-independent drivers (pure)
lbrlib/components/lbr_hw/             their ESP32 side (ADC, pulse, GPIO, UART)
lbrlib/components/lbr_node/           runtime: timing, timestamps, seq, CAN, logs, heartbeat
```

Until a slot has a driver it runs the **unconfigured** driver: every channel publishes
`valid = false, NOT_FITTED`, so the dashboard already shows the channel as expected but absent.
With `CONFIG_LBR_SENSOR_SIM=y` unbound slots run the simulator instead (bench only; flagged in
every heartbeat and session config).

## Inputs that need no driver code

Many Baja sensors reach a node as a plain voltage, a pulse train, or a switch, whatever part is
picked. Those are wired by configuration alone: each slot of a node has a menuconfig string
(`CONFIG_LBR_IO_<SLOT>`, under "LBR <node> inputs") with one entry per channel, in the slot's
signal order, separated by `;`. Empty means not fitted.

| Entry | Input | Typical sensors |
| --- | --- | --- |
| `-` | not fitted | |
| `adc gpio=34 mv0=310 v0=0 mv1=2980 v1=100` | ADC1 pin, two-point linear calibration | potentiometers (pedals, steering), fuel sender, a battery divider |
| `adc gpio=36 curve=ntc r25=10000 beta=3950 rpull=10000 vsup=3300` | NTC thermistor to ground with a pull-up | CVT temperature |
| `pulse gpio=27 scale=60 stall_ms=1000` | edge-to-edge frequency x `scale` | engine or CVT speed (`scale` = 60 / pulses per rev) |
| `din gpio=4 active=low pull=up` | debounced switch | a bool signal |
| `din gpio=4,5,18,19` | several pins as a bitmask (bit i = i-th pin) | a bitmask signal |

Pins are checked against the node's chip: on the classic ESP32 (powertrain, driver_input,
temperature, stress) ADC1 is GPIO 32..39 and 34..39 have no internal pulls; on the ESP32-C5
(imu, central, cockpit, corners) ADC1 is GPIO 1..6 and GPIO 15..22 belong to flash.

ADC options: `avg=` samples per reading (default 4), `lo=`/`hi=` electrical window in mV (outside
it the channel reads `SENSOR_FAULT`: open or shorted sensor), `min=`/`max=` physical range
(outside it reads `OUT_OF_RANGE`). Pulse options: `edge=rise|fall`, `pull=up|down|none`. Digital
options: `active=low|high`, `pull=`, `debounce_ms=` (default 10). Full grammar:
`lbrlib/components/lbr_core/include/lbr/sensor_io.h`.

For battery slots, use `CONFIG_LBR_IO_BATTERY` and configure the voltage divider's measured
endpoints as voltage-to-percent values (`mv0/v0` and `mv1/v1`). This is a two-point linear
calibration; it does not model a chemistry-specific discharge curve. The selected pack's voltage
versus state-of-charge curve should be measured or taken from its data sheet before setting these
values. Keep the percent range at 0..100 so CAN encoding and warnings have consistent semantics.

The spec is the channel's calibration: it is recorded verbatim in every session config
(`sensor.<n>.params=`) and changes the config CRC. A spec that does not parse (wrong channel
count, a pin used twice, an ADC2 pin, a pull-up on an input-only pin, an `adc` entry on a bool
signal, ...) fails that slot's init, so it reports `SENSOR_INIT_FAIL` and `SENSOR_FAULT` instead of
looking not fitted; the reason is in the node's log output.

Limits of the on-chip ADC: about 150..2450 mV is its accurate range at 12 dB attenuation, so
divide 5 V sensors down; and it is too noisy for strain gauges (the `stress` node needs a bridge
amplifier/ADC part). Brake temperatures (likely thermocouples or IR) and the IMUs need part
drivers too.

**GPS** (on the `central` node) works with any module that outputs NMEA 0183: set `CONFIG_LBR_GPS_RX_GPIO`
(and baud). GGA, RMC, and GSA from any talker are used; bad checksums are counted and ignored, a
quiet receiver reads `NO_DATA` after 2 s, position needs a 2D fix, altitude a 3D fix, and UTC an
active RMC.

## Writing a driver

A part with its own protocol (an IMU, a strain amplifier, a thermocouple converter) needs a
driver: an `lbr_driver` table with `start()` and `read()` on a state struct, bound in the node's
`main.c` with `lbr_bind(n, SLOT, &driver, &state)`. The full guide, with an example, is
[`lbrlib/docs/drivers.md`](../lbrlib/docs/drivers.md).

## Rules every driver follows

- **Units:** report the catalog unit for each signal (`catalog.json`), converted from raw
  counts in the driver. Calibration constants come from config, not from code.
- **Sensor frame:** report values in the part's own axes. Mounting transforms are session
  metadata applied by the dashboard.
- **No fake zeros:** a channel without a fresh, trustworthy value is `valid = false` with the
  right reason (`NO_DATA`, `OUT_OF_RANGE`, `NOT_CALIBRATED`, `BUS_ERROR`, ...).
- **Bounded time:** `read()` must finish well inside one sample period. Never block on a bus.
  Missed periods are counted by the runtime as overruns.
- **Pins from Kconfig:** no hard-coded GPIO. Add `CONFIG_LBR_<NODE>_<FUNC>_GPIO` settings
  defaulting to `-1` and refuse to start while unset (see `docs/hardware.md`).

## Changing rates or channels

A slot's rate must equal the rate of the CAN frame its signals travel in, and every field of
that frame must come from the same slot (each sample is published exactly once, nothing is
decimated). Rates, channels, slots, and frames are all declared once, in
[`car/catalog.toml`](../car/catalog.toml) (with DAQ-WIRING.md section 7); a new signal is one
line in its frame. Regenerate and test:

```sh
python3 lbrlib/tools/lbrgen.py car/catalog.toml     # checks the layout, writes plans and tables
ctest --test-dir build-dev                         # test_plans: every plan, producer, bus budget
```

`lbrgen.py` refuses a catalog with a clear message (duplicate id, fields that do not fit the
frame, a slot no frame fills, ...); `test_plans` then checks every plan against the table, that
every frame has a producer, and the whole bus against its 50 % ceiling. A catalog change is a
protocol change: `cmake --build build-dev --target fixtures` and a matching update in the
dashboard app.
