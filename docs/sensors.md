# Adding a sensor driver

Sensor parts are chosen per project as the electronics settle. The framework is built so that a
new part touches exactly one driver file and one line in its node's `main.c`.

## How a node is put together

```text
car/nodes/<node>/main/<node>_plan.c   slots: which signals, at what rate, CAN decimation (pure data)
car/nodes/<node>/main/main.c          binds a driver to each slot that has a part
car/components/sensors/               driver interface + drivers
car/components/node/                  runtime: timing, timestamps, seq, CAN, logs, heartbeat
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
(imu, central, dashboard, corners) ADC1 is GPIO 1..6 and GPIO 15..22 belong to flash.

ADC options: `avg=` samples per reading (default 4), `lo=`/`hi=` electrical window in mV (outside
it the channel reads `SENSOR_FAULT`: open or shorted sensor), `min=`/`max=` physical range
(outside it reads `OUT_OF_RANGE`). Pulse options: `edge=rise|fall`, `pull=up|down|none`. Digital
options: `active=low|high`, `pull=`, `debounce_ms=` (default 10). Full grammar:
`car/components/sensors/include/lbr/sensor_io.h`.

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

1. Create `car/components/sensors/sensor_<part>.c` (hardware code may use ESP-IDF; keep any pure
   conversion math in a separate function so it can be host-tested).
2. Provide a constructor that fills an `lbr_sensor_t`:

   ```c
   void lbr_sensor_<part>(lbr_sensor_t *out, const lbr_signal_id_t *signals, size_t n,
                          const <part>_config_t *cfg);
   ```

3. Implement:
   - `init()`: bring the part up; return non-zero on failure. The runtime reports
     `SENSOR_INIT_FAIL`, marks every channel `SENSOR_FAULT`, and retries every 2 s.
   - `read(self, t_us, out)`: fill one `lbr_reading_t` per channel, in the slot's signal order.
     Return non-zero if the part stopped responding (the runtime re-inits it).
   - `is_calibrated()` (optional): false while calibration is missing or invalid.
4. Add the file to `car/components/sensors/CMakeLists.txt`, and bind it in the node's `main.c`:

   ```c
   lbr_sensor_<part>(&s_sensors[IMU_SLOT_CHASSIS], lbr_plan_imu.slots[IMU_SLOT_CHASSIS].signals,
                     lbr_plan_imu.slots[IMU_SLOT_CHASSIS].n_signals, &cfg);
   ```

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
decimated). So a rate or channel change is two edits: the node's plan and the frame table
(`car/components/protocol/frames.c`, with DAQ-WIRING.md section 7). Then run the host tests:
`test_plans` checks every plan against the table, that every frame has a producer, and the whole
bus against its 50 % ceiling. A new signal goes in its group in `signals.def` and in one frame,
followed by `cmake --build build-dev --target fixtures` and a matching update in
`lbr-dashboard`.
