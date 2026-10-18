# Adding a sensor driver

Sensor parts are chosen per project as the electronics settle. The framework is built so that a
new part touches exactly one driver file and one line in its node's `main.c`.

## How a node is put together

```text
nodes/<node>/main/<node>_plan.c   slots: which signals, at what rate, CAN decimation (pure data)
nodes/<node>/main/main.c          binds a driver to each slot that has a part
components/sensors/               driver interface + drivers
components/node/                  runtime: timing, timestamps, seq, CAN, logs, heartbeat
```

Until a slot has a driver it runs the **unconfigured** driver: every channel publishes
`valid = false, NOT_FITTED`, so the dashboard already shows the channel as expected but absent.
With `CONFIG_LBR_SENSOR_SIM=y` unbound slots run the simulator instead (bench only; flagged in
every heartbeat and session config).

## Writing a driver

1. Create `components/sensors/sensor_<part>.c` (hardware code may use ESP-IDF; keep any pure
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
4. Add the file to `components/sensors/CMakeLists.txt`, and bind it in the node's `main.c`:

   ```c
   lbr_sensor_<part>(&s_sensors[SUSPENSION_SLOT_TRAVEL], lbr_plan_suspension.slots[0].signals,
                     lbr_plan_suspension.slots[0].n_signals, &cfg);
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

Edit the node's plan and run the host tests: `test_plans` checks the node's own CAN budget,
signal ownership across nodes, and the whole bus. A channel decimated on CAN
(`publish_div > 1`) requires the node to keep a local log, so no sample is discarded unrecorded.
A new signal goes at the end of its group in `signals.def` (append only), followed by
`cmake --build build-host --target fixtures` and a matching update in `lbr-dashboard`.
