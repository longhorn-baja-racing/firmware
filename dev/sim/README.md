# LBR car simulator

Development only: nothing in `dev/` is ever built into firmware.

The simulator runs the **real firmware logic of every node** on a simulated car, so you can
watch the system work, break things on purpose, and check the behavior in automated scenarios
before any hardware exists.

## Run it

```sh
cmake -S dev -B build-dev -G Ninja && cmake --build build-dev
python3 dev/sim/ui/server.py          # open http://127.0.0.1:8765/
```

The browser view shows a top-down layout of the car: the eight nodes in their zones, every
sensor at its location, and the CAN trunk with frames moving along it (colored by priority).
Node outlines and sensor dots are colored by state (green ok, amber degraded / no data, red
fault, grey not fitted or off).

- **Click a node**: its state, time sync error, CAN controller (TEC/REC, drops, bus-off),
  heartbeat counters, slots, SD card and links, plus buttons to unplug its CAN cable, force
  bus-off, reboot, power it off, change its crystal error, unplug a part, or pull its SD card.
- **Click a sensor**: its live values, and controls to override the value, open the circuit,
  or unplug the part.
- **Click the bus**: load, frame rate, error frames, and noise injection.
- **Commands tab**: press "Trace commands" on a node to see every call its firmware makes into
  the hardware: `twai.send`, `adc.read`, `gpio.read`, `uart.read`, `sd.write`, `sd.fsync`,
  `time.apply`, `usb.write`, `lora.tx`, and so on.
- **Events tab**: every EVENT/FAULT seen on the bus, scenario check results, and (optionally)
  each node's log lines.
- **Cockpit tab**: what the driver's display shows, from the dashboard node's own tables.
- **Console tab**: any command below; `help` lists them.
- Header: pause, step 100 ms, speed (0.05x to 20x), race or pit (idle) driving, seed and
  restart, and **Play scenario**.

Everything the gateway writes is a real file in `sim-out/` (or `--out`): session logs
`NNNNNNNN.LBR`, the stress node's local logs, and the USB stream `usb_stream.bin`. Decode them
like the car's:

```sh
python3 dev/tools/lbr_decode.py log sim-out/00000001.LBR
python3 dev/tools/lbr_decode.py stream sim-out/usb_stream.bin
```

## What is real and what is simulated

| Real firmware code (unchanged) | Simulated |
| --- | --- |
| sensor-node runtime (`node_core.c`): validation, acquisition, retries, stamping, sequence numbers, decimation, events, heartbeats | the ESP32s: FreeRTOS tasks run instantly, no CPU load |
| gateway (`gw_core.c`): authoritative logging, node table, time master, USB/WiFi stream, LoRa policy | the CAN controllers and bus: bit-timed frames with stuffing, arbitration, TX queues, acceptance filter, error frames, TEC/REC, bus-off and recovery |
| time sync servo, CAN/health/sample codecs, node plans | each node's crystal (ppm error) and boot time |
| logger, log keeper (SD session, remount, LOSS records), packet framing, COBS, LoRa airtime and duty limiter | the SD card (a file; pull it or fail writes), USB/WiFi/LoRa links (counted, USB written to a file) |
| input-spec parsing and conversion (`sensor_io.c`), NMEA parser, dashboard value table | the vehicle and sensors: a Baja car lapping a loop; pots, transducers, NTCs, switches, IMU, GPS receiver |

Not covered: the `*_esp.c` hardware glue itself (TWAI driver, FATFS, radios, WiFi/BLE, OTA).
A passing scenario says the logic is right; hardware-in-the-loop tests are still needed.

Sensor wiring in the simulator uses the menuconfig input-spec syntax (see
`docs/sensors.md` and `world.c`) with plausible bench values; it is not a pin map.

## Commands

The same language in the UI console, in scenarios, and on `lbr_sim --interactive` stdin.

| Command | Effect |
| --- | --- |
| `run <ms>` | advance simulated time |
| `trace <node\|all\|none>` | stream a node's hardware commands |
| `unplug <node> <slot>` / `plug ...` | a part stops / starts responding (e.g. `unplug suspension travel`) |
| `open <SIGNAL>` / `close <SIGNAL>` | open circuit on one input / repair it |
| `set <SIGNAL> <value>` / `auto <SIGNAL>` | override the vehicle model / back to the model |
| `press <bit>` / `release <bit>` | dashboard buttons |
| `disconnect <node>` / `connect <node>` | CAN cable |
| `busoff <node>` | force the controller bus-off (it recovers by itself) |
| `noise <permille>` | corrupt that share of frames (error frames, retransmission) |
| `drift <node> <ppm>` | crystal error, up to +/-500 ppm |
| `reboot <node>`, `power <node> on\|off` | software reset / power cycle (power off loses unwritten log data) |
| `sd [node] remove\|insert\|fail <n>` | the gateway's (default) or a node's SD card |
| `drive race\|idle` | vehicle on the course or idling in the pit |
| `expect <target> <metric> <op> <value>` | a check (see below) |

Nodes: `gateway imu_gps suspension driver_inputs temperature fuel_gauge stress dashboard`.
Slots are the names in each node's plan; signals are catalog names (`SUSP_TRAVEL_FL`, ...).

### Checks

`expect <target> <metric> <op> <value>` with `op` one of `== != < <= > >=`:

| Target | Metrics |
| --- | --- |
| a node, or `all` (every sensor node; worst case reported) | `state` (RUN, DEGRADED, FAULT, BADCONFIG, BOOT, OFF), `seen` (the gateway sees it), `synced`, `sync_err_us`, `tx_dropped`, `bus_off`, `tec`, `overflow`, `steps`, `boot_count`, `event:<NAME>` (count seen on the bus, e.g. `event:SENSOR_INIT_FAIL`) |
| `bus` | `load` (percent, last second), `errors` |
| `gateway` | node metrics, plus `frames_logged`, `log_dropped`, `usb_pkts`, `lora_pkts`, `sync_fail` |
| `log` | `intact` (1 if the gateway's current session file reads back with no corruption), `frames` |
| a signal | `valid`, `value`, `age_ms`, `reason` (NONE, NOT_FITTED, NO_DATA, SENSOR_FAULT, OUT_OF_RANGE, ...) |

## Scenarios

`dev/sim/scenarios/*.sim` are scripts of `at <seconds> <command>` lines and one `end <seconds>`.
Every scenario runs in CI (`ctest`), and the UI can play them. Run one by hand:

```sh
build-dev/sim/lbr_sim dev/sim/scenarios/can_faults.sim            # exit status = failed checks
build-dev/sim/lbr_sim --trace suspension dev/sim/scenarios/sensor_faults.sim
build-dev/sim/lbr_sim --seed 7 --duration 30                       # just run, print a snapshot
```

| Scenario | Covers |
| --- | --- |
| `nominal` | boot, sync, all nodes RUN, bus load, no drops, log integrity, links |
| `sensor_faults` | unplugged part (retry and recovery), open circuit, out of range, GPS going quiet |
| `can_faults` | unplugged CAN cable (node lost / found), bus-off recovery, noisy bus |
| `time_sync` | +/-400 ppm crystals, gateway reboot and new epoch |
| `sd_card` | pulled card, remount and new session, write errors, the stress node's card |

Add a scenario for every bug the simulator finds or every behavior you change. Runs are
deterministic for a given `--seed`.

## Findings so far

- `imu_gps` overflowed a 32-frame CAN TX queue every second when all its slots and the
  heartbeat came due together; the queue is now 64 frames and `test_plans` checks every node's
  worst-case burst.
- Boot events and the first samples were rejected because the SD session opened after them;
  the log service now opens the first session before anything is logged.
- With the default LoRa settings (SF9, 125 kHz, 10 % duty, full packets) each packet is about
  1.2 s on air, so the pit gets one update about every 12 s (issue #6).

## Building on Windows

Use WSL or MSYS2 (MinGW-w64) with CMake and Ninja; the UI only needs Python 3.
