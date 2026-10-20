# CAN ID map

Part of protocol version 3. Keep synchronized with `lbr-dashboard` and every node's firmware.
The map is the one in [`DAQ-WIRING.md`](../DAQ-WIRING.md) section 7; this file adds the bit-level
rules. The data frames are a schema, `car/components/protocol/frames.c`, exported as the
`frames` table of `dev/tests/fixtures/catalog.json` and folded into the catalog hash; the codec is
`car/components/can/include/lbr/can.h`; `dev/tests/fixtures/can_capture.*` is the byte-level
reference.

## Frames

Every LBR frame is a **classic CAN data frame with an 11-bit standard id**. Lower ids win
arbitration, so the map is ordered by priority: time and commands, events, driver inputs,
powertrain, IMUs, stress, temperatures, Central data, heartbeats, health detail. Every id has
exactly one transmitter, as classic CAN requires, so the sender of a frame follows from its id
(`lbr_can_sender`). Extended-id and remote frames are not LBR traffic; receivers count and ignore
them (`rx_foreign`).

| Id | Sender | Frame | Rate | dlc |
| --- | --- | --- | ---: | ---: |
| 0x001 | Central | `TIME_SYNC` | 10 Hz | 8 |
| 0x002 | Central | `TIME_FOLLOW_UP` | 10 Hz | 8 |
| 0x010 | Central | `COMMAND` | on event | 8 |
| 0x020 + n | node n (1..6) | `EVENT` / `FAULT` | on event | 8 |
| 0x100 .. 0x60A | per table | data frames (below) | per table | per table |
| 0x700 + n | node n | `HEARTBEAT` | 1 Hz | 2 |
| 0x780 + 16 n + p | node n | `HEALTH` page p (0..11) | 1 Hz | 8 |

### Node ids

Ids 1..6 are the wired controllers and match DAQ-WIRING's "CAN node N"; 7 and up never
transmit on CAN, but samples they take carry their id as the origin.

| Id | Node | Board | Link |
| --- | --- | --- | --- |
| 0x01 | `imu` | ESP32-C5 | CAN, corner WiFi access point |
| 0x02 | `powertrain` | ESP32 | CAN |
| 0x03 | `driver_input` | ESP32 | CAN |
| 0x04 | `temperature` | ESP32 | CAN |
| 0x05 | `stress` | ESP32 | CAN |
| 0x06 | `central` | ESP32-C5 | CAN, pit/dashboard WiFi access point, LoRa, USB, BLE |
| 0x07 | `dashboard` | ESP32-C5 | WiFi to the Central |
| 0x08 .. 0x0B | `corner_fl`, `corner_fr`, `corner_bl`, `corner_br` | Mini ESP32-C5 | WiFi to the IMU Controller |

## Data frames

| Id | Frame | Sender | Origin | Rate | Fields (byte: signal, encoding, scale) | Counter |
| --- | --- | --- | --- | ---: | --- | --- |
| 0x100 | `DRIVER_INPUTS` | driver_input | same | 100 | 0: BRAKE_PEDAL i16 0.01 %, 2: GAS_PEDAL i16 0.01 %, 4: STEERING_ANGLE i16 0.1 deg | 6 |
| 0x200 | `POWERTRAIN_SPEEDS` | powertrain | same | 100 | 0: ENGINE_RPM u16 1, 2: CVT_RPM u16 1 | 6 |
| 0x201 | `FUEL_LEVEL` | powertrain | same | 1 | 0: FUEL_LEVEL i16 0.1 % | - |
| 0x300, 0x302, 0x304, 0x306 | `IMU_xx_ACCEL` | imu | corner xx | 100 | 0/2/4: ACCEL_X/Y/Z i16 0.01 m/s^2 | 6 |
| 0x301, 0x303, 0x305, 0x307 | `IMU_xx_GYRO` | imu | corner xx | 100 | 0/2/4: GYRO_X/Y/Z i16 0.001 rad/s | 6 |
| 0x308 / 0x309 | `IMU_CH_ACCEL` / `IMU_CH_GYRO` | imu | imu | 100 | as the corners | 6 |
| 0x30A | `CORNER_STATUS` | imu | imu | 1 | 0..3: CORNER_FL..BR_BATTERY u8 1 %, 4: CORNER_LINK u8 bits, 5: CORNER_SYNCED u8 bits | - |
| 0x400 / 0x401 | `STRESS_1_4` / `STRESS_5_8` | stress | same | 100 | 0/2/4/6: STRESS_n i16 1 ue | - |
| 0x500 | `BRAKE_TEMPS` | temperature | same | 10 | 0/2/4/6: TEMP_BRAKE_FL/FR/BL/BR i16 0.1 degC | - |
| 0x501 | `CVT_TEMP` | temperature | same | 10 | 0: TEMP_CVT i16 0.1 degC | - |
| 0x600 | `GPS_POSITION` | central | same | 10 | 0: GPS_LATITUDE i32, 4: GPS_LONGITUDE i32 (1e-7 deg) | - |
| 0x601 | `GPS_MOTION` | central | same | 10 | 0: GPS_SPEED u16 0.01 m/s, 2: GPS_COURSE u16 0.01 deg, 4: GPS_ALTITUDE i16 0.1 m | 6 |
| 0x602 | `GPS_STATUS` | central | same | 10 | 0: GPS_UTC u32, 4: GPS_FIX u8, 5: GPS_SATELLITES u8, 6: GPS_HDOP u16 0.01 | - |
| 0x60A | `DASH_STATUS` | central | same | 1 | 0: DASH_BATTERY u8 1 %, 1: DASH_LINK u8 | - |

Scales marked TBD in DAQ-WIRING (pedals, steering, fuel, IMU, stress) are provisional until parts
are chosen. A scale change is a change to `frames.c`: the catalog hash moves, the fixtures are
regenerated, and `lbr-dashboard` updates in step. Every signal travels in exactly one frame, and
every field of a frame comes from one acquisition slot sampled at the frame's rate, so each
sample is published exactly once (checked by `test_plans`).

### Field encodings and unavailable values

Multi-byte values are little-endian. Physical value = raw x scale for `f32` signals; integer
signals travel unscaled.

| Encoding | Valid raw range | Reserved: unavailable, reason = raw - base |
| --- | --- | --- |
| `i16` | -32760 .. 32767 | base -32768 (-32768 .. -32761) |
| `u16` | 0 .. 65527 | base 0xFFF8 |
| `u8` | 0 .. 247 | base 0xF8 |
| `i32` | INT32_MIN + 8 .. INT32_MAX | base INT32_MIN |
| `u32` | 0 .. 0xFFFFFFF7 | base 0xFFFFFFF8 |

Reasons: 0 `UNSPECIFIED`, 1 `NOT_FITTED`, 2 `NO_DATA`, 3 `SENSOR_FAULT`, 4 `OUT_OF_RANGE`,
5 `NOT_CALIBRATED`, 6 `BUS_ERROR`. A valid reading that does not fit its field (or NaN) goes out as
`OUT_OF_RANGE` and is counted in the sender's `rejected_count`. A missing value is never a zero.

### Time grid and sample counter

Every node samples each slot on a grid of the shared clock: sample k of a stream is taken at
shared time k x period (period = 1000000 / rate us), and the platform wakes the slot at each grid
point. The frame's counter is k mod 65536.

- **Frames with a counter:** the receiver takes the k nearest its own receive time with those low
  16 bits (`lbr_can_grid_index`); the sample time is k x period, exact while delivery takes under
  32768 periods (5.5 minutes at 100 Hz). Gaps in k are lost samples.
- **Frames without a counter:** stamped at the last grid point at or before reception, which holds
  while the frame is delivered within one period (10 ms for stress). Their seq is inferred
  (`seq_inferred`) and gaps cannot be detected. If the stress frames ever need detection, the fix
  is a counter (three channels per frame) or CAN FD.
- Forwarded corner frames keep the corner's own k: the corner samples on the grid of its own
  disciplined clock, so WiFi latency never becomes measurement time.

### Synced or not

A frame alone does not say whether its origin's clock was on the shared base. Readers track it
(`lbr_bus_decoder_t`, and the same rules in `dev/tools/lbr_decode.py`):

- a CAN node is synced while its latest heartbeat lacks `NOT_SYNCED`;
- a corner is synced while the IMU Controller is, and the latest `CORNER_STATUS` sets the corner's
  `CORNER_SYNCED` bit (updated once a second);
- the Central is always synced (it is the time master);
- nothing heard yet means not synced.

## System frames

| Id | Frame | Payload |
| --- | --- | --- |
| 0x001 | `TIME_SYNC` | 0..3 uint32 ms since the Central booted (coarse, for simple listeners), 4 seq, 5..6 epoch, 7 reserved (0) |
| 0x002 | `TIME_FOLLOW_UP` | 0 seq, 1..7 uint56: shared time (us) at which `TIME_SYNC` seq finished transmitting |
| 0x010 | `COMMAND` | 0 code, 1..7 args. 0x01 `SESSION_START` (args 0..3: Central boot count), 0x02 `SESSION_STOP`, 0x03 `MARK` |
| 0x020 + n | `EVENT` / `FAULT` | 0..1 code, 2..3 arg, 4: bit 0 fault, bit 1 synced, bits 2..7 seq (6 bits, per node), 5..7 low 24 bits of the event time (us) |
| 0x700 + n | `HEARTBEAT` | 0 state (`lbr_node_state_t`), 1 error flags (below) |
| 0x780 + 16 n + p | `HEALTH` | one 8-byte health page (`docs/protocol.md`) |

The event time is rebuilt from the receiver's clock (`lbr_can_unwrap_time`), exact within
+/- 8.38 s. Sensor nodes ignore commands except to count and log them today.

Heartbeat error flags: bit 0 `NOT_SYNCED`, bit 1 `SENSOR` (a slot faulted or uncalibrated, or a
corner reports its IMU faulted), bit 2 `CAN` (error passive, not running, or a bus-off since the
last heartbeat), bit 3 `CONFIG`, bit 4 `STORAGE` (an expected log is not being written), bit 5
`SIM` (simulated data), bit 6 `LOSS` (samples, frames, or records lost since the last heartbeat),
bit 7 `LINK` (a wireless link is down).

Sensor nodes program their TWAI filter to ids 0x000..0x01F: they only receive the Central's time
sync and commands. The Central receives everything.

## Central log record (22 bytes)

The Central logs every frame as `t_us u64, id u32, flags u8 (bit 0 extended id, bit 1 sent by the
Central itself), dlc u8, data[8]` (zero padded). See `docs/protocol.md`.

## Bandwidth budget

Worst-case standard frame size (11-bit id, 8 data bytes, maximum bit stuffing, interframe space)
is **135 bits**. The load is fixed by the frame table: a node can only send its own frames at their
rates, which `lbr_node_config_validate` enforces at boot. The host tests check the whole bus
against a **system ceiling of 50 %** (DAQ-WIRING.md).

At **500 kbit/s** (from `test_plans`):

| Node | Load | Share |
| --- | ---: | ---: |
| `imu` | 136.8 kbit/s | 27.4 % |
| `powertrain` | 15.3 kbit/s | 3.1 % |
| `driver_input` | 15.2 kbit/s | 3.0 % |
| `temperature` | 3.8 kbit/s | 0.8 % |
| `stress` | 28.7 kbit/s | 5.7 % |
| `central` (incl. time sync) | 8.5 kbit/s | 1.7 % |
| **total** | **208.3 kbit/s** | **41.7 %** |

The simulator measures about 37 % with real bit stuffing. If the bus gets tight: lower rates in
the table, move to 1 Mbit/s (DAQ-WIRING.md), or CAN FD (needs CAN-FD-capable controllers).

The IMU Controller's TX queue is 127 frames (`CONFIG_LBR_CAN_TX_QUEUE_LEN`), every other node's 64:
forwarding four corners' datagrams at once can queue about 100 frames.

## To confirm with hardware (#15)

- [ ] bit rate (500 kbit/s) and sample point against harness length and stubs
- [ ] termination (120 ohm at the IMU Controller and the Central only) and the daisy-chain order
- [ ] transceiver part and its standby/silent pin handling
- [ ] measured bus load under 50 %
