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

The browser view shows a top-down layout of the car as DAQ-WIRING.md wires it: the ten CAN
nodes on the trunk, the four wheel hub nodes on drops at the wheels (frames moving along it,
colored by priority), the dashboard in the cockpit with its 5 GHz WiFi link, and every sensor at
its location. Node outlines, links, and sensor dots are colored by state (green
ok, amber degraded / no data, red fault or lost, grey not fitted or off).

- **Click a node**: its state, time sync error, CAN controller (TEC/REC, drops, bus-off) or, for
  the dashboard, its WiFi link (datagrams, gaps), heartbeat counters, slots, SD card and links,
  plus buttons to unplug its CAN cable, force bus-off, take the dashboard out of WiFi range or
  add loss, reboot, power it off, change its crystal error, unplug a part, or pull the Central's
  SD card.
- **Click a sensor**: its live values, and controls to override the value, open the circuit,
  or unplug the part.
- **Click the bus**: load, frame rate, error frames, and noise injection.
- **Commands tab**: press "Trace commands" on a node to see every call its firmware makes into
  the hardware: `twai.send`, `adc.read`, `gpio.read`, `uart.read`, `sd.write`, `sd.fsync`,
  `time.apply`, `udp.send`, `udp.recv`, `usb.write`, `lora.tx`, and so on.
- **Events tab**: every EVENT/FAULT seen on the bus, scenario check results, and (optionally)
  each node's log lines.
- **Cockpit tab**: what the driver's display shows, from the dashboard core's own tables (fed
  over the simulated WiFi by the Central).
- **Console tab**: any command below; `help` lists them.
- Header: pause, step 100 ms, speed (0.05x to 20x), race or pit (idle) driving, seed and
  restart, and **Play scenario**.

Everything the Central writes is a real file in `sim-out/` (or `--out`): session logs
`NNNNNNNN.LBR` and the USB stream `usb_stream.bin`. Decode them
like the car's:

```sh
python3 dev/tools/lbr_decode.py log sim-out/00000001.LBR
python3 dev/tools/lbr_decode.py stream sim-out/usb_stream.bin
```

## What is real and what is simulated

| Real firmware code (unchanged) | Simulated |
| --- | --- |
| sensor-node runtime (`node_core.c`) and acquisition engine (`acq.c`): validation, grid-locked sampling, retries, frame packing, events, heartbeats | the ESP32s: FreeRTOS tasks run instantly, no CPU load |
| Central (`gw_core.c`): authoritative logging, node table, time master, GPS and dashboard-status slots, USB/WiFi stream, LoRa policy, dashboard link (`dash_link.c`) | the CAN controllers and bus: bit-timed 11-bit frames with stuffing, arbitration, TX queues, acceptance filter, error frames, TEC/REC, bus-off and recovery |
| CAN sensor nodes, wheel hubs included (`node_core.c`, the same runtime as on the car), the dashboard (`dash_core.c`) | the dashboard's 5 GHz WiFi: UDP datagrams with 1..3 ms latency, loss, and out-of-range; each node's crystal (ppm error) and boot time |
| time sync servo, CAN frame table and codecs, bus decoder, node plans | the SD card (a file; pull it or fail writes), USB/WiFi/LoRa links (counted, USB written to a file) |
| logger, log keeper (SD session, remount, LOSS records), packet framing, COBS, LoRa airtime and duty limiter, input-spec parsing and conversion (`sensor_io.c`, per chip), NMEA parser, value tables | the vehicle and sensors: a Baja car lapping a loop; pots, pickups, NTCs, five IMUs, the dashboard battery, GPS receiver |

Not covered: the `*_esp.c` hardware glue and the node apps' task wiring (TWAI driver, FATFS,
radios, WiFi/UDP sockets, BLE, OTA).
A passing scenario says the logic is right; hardware-in-the-loop tests are still needed.

Sensor wiring in the simulator uses the menuconfig input-spec syntax (see
`docs/sensors.md` and `world.c`) with plausible bench values; it is not a pin map.

## Commands

The same language in the UI console, in scenarios, and on `lbr_sim --interactive` stdin.

| Command | Effect |
| --- | --- |
| `run <ms>` | advance simulated time |
| `trace <node\|all\|none>` | stream a node's hardware commands |
| `unplug <node> <slot>` / `plug ...` | a part stops / starts responding (e.g. `unplug corner_fl imu`) |
| `open <SIGNAL>` / `close <SIGNAL>` | open circuit on one input / repair it |
| `set <SIGNAL> <value>` / `auto <SIGNAL>` | override the vehicle model / back to the model |
| `disconnect <node>` / `connect <node>` | CAN cable (CAN nodes) |
| `wifi <node> up\|down\|loss <permille>` | the dashboard's WiFi link: in range, out of range, random datagram loss |
| `busoff <node>` | force the controller bus-off (it recovers by itself) |
| `noise <permille>` | corrupt that share of frames (error frames, retransmission) |
| `drift <node> <ppm>` | crystal error, up to +/-500 ppm |
| `reboot <node>`, `power <node> on\|off` | software reset / power cycle (power off loses unwritten log data) |
| `sd [node] remove\|insert\|fail <n>` | the Central's (default) or a node's SD card |
| `drive race\|idle` | vehicle on the course or idling in the pit |
| `expect <target> <metric> <op> <value>` | a check (see below) |

Nodes: `central imu powertrain driver_input temperature stress corner_fl corner_fr corner_bl
corner_br dashboard`. Slots are the names in each node's plan (`imu`: chassis;
`powertrain`: speeds, fuel; `driver_input`: inputs; `temperature`: brakes, cvt; `stress`: stress;
`central`: gps, dash_status; corners: imu; `dashboard`: battery); signals are catalog
names (`ENGINE_RPM`, `IMU_FL_ACCEL_Z`, ...).

### Checks

`expect <target> <metric> <op> <value>` with `op` one of `== != < <= > >=`:

| Target | Metrics |
| --- | --- |
| a node, or a group: `all` (every node but the Central), `can` (the CAN sensor nodes, corners included), `corners` (the four wheel hub nodes), `wireless` (the dashboard); worst case reported | `state` (RUN, DEGRADED, FAULT, BADCONFIG, BOOT, OFF, LOST), `seen` (the Central hears it), `synced` (1 for the dashboard, which keeps no shared clock), `sync_err_us` (0 for the dashboard; 2147483647 while a node is off or not yet synced), `tx_dropped`, `bus_off`, `tec`, `overflow`, `steps`, `boot_count`, `transport_errors`, `event:<NAME>` (count seen on the bus, e.g. `event:SENSOR_INIT_FAIL`); the dashboard: `link`, `gaps` (datagrams from the Central that never arrived) |
| `bus` | `load` (percent, last second), `errors` |
| `central` | node metrics, plus `frames_logged`, `log_dropped`, `usb_pkts`, `lora_pkts`, `dash_pkts`, `sync_fail` |
| `log` | `intact` (1 if the Central's current session file reads back with no corruption), `frames` |
| a signal | `valid`, `value`, `age_ms`, `reason` (NONE, NOT_FITTED, NO_DATA, SENSOR_FAULT, OUT_OF_RANGE, ...), as a log reader decodes it from the bus |

## Scenarios

`dev/sim/scenarios/*.sim` are scripts of `at <seconds> <command>` lines and one `end <seconds>`.
Every scenario runs in CI (`ctest`), and the UI can play them. Run one by hand:

```sh
build-dev/sim/lbr_sim dev/sim/scenarios/can_faults.sim            # exit status = failed checks
build-dev/sim/lbr_sim --trace corner_fl dev/sim/scenarios/can_faults.sim
build-dev/sim/lbr_sim --seed 7 --duration 30                       # just run, print a snapshot
```

| Scenario | Covers |
| --- | --- |
| `nominal` | boot, CAN sync on all ten nodes, all nodes RUN, bus load under 50 %, no drops, log integrity, links, GPS and dashboard |
| `sensor_faults` | unplugged part (retry and recovery), open circuit, out of range (input and CAN field), the Central's GPS going quiet, a wheel hub IMU failing |
| `can_faults` | unplugged CAN cable (node lost / found), bus-off recovery, a wheel hub off the bus, noisy bus |
| `wifi` | dashboard datagram loss (counted), the dashboard out of range and back, low battery |
| `time_sync` | +/-400 ppm crystals, Central reboot and new epoch |
| `sd_card` | pulled card, remount and new session, write errors |

Add a scenario for every bug the simulator finds or every behavior you change. Runs are
deterministic for a given `--seed`.

## Findings so far

- `imu_gps` (2025 node set) overflowed a 32-frame CAN TX queue every second when all its slots and
  the heartbeat came due together; queues are now sized by `test_plans` (64 frames).
- Boot events and the first samples were rejected because the SD session opened after them;
  the log service now opens the first session before anything is logged.
- With the default LoRa settings (SF9, 125 kHz, 10 % duty, full packets) each packet is about
  1.2 s on air, so the pit gets one update about every 12 s (issue #6).
- The dashboard reports once a second, so a 0.5 s link timeout made the Central report it lost
  and found every second; its link has its own 3 s timeout.
- Four lost dashboard datagrams in a row trip the receive timeout, and restarting the sequence
  window then forgot the losses it still held; a restart now counts them
  (`lbr_seq_window_restart`).

## Building on Windows

Use WSL or MSYS2 (MinGW-w64) with CMake and Ninja; the UI only needs Python 3.
