# CAN ID / arbitration map

Part of protocol version 2. Keep synchronized with `lbr-dashboard` and every node's firmware;
`car/components/can/include/lbr/can.h` is the implementation and `dev/tests/fixtures/can_capture.*` the
byte-level reference.

## Frame format

Every LBR frame is a **classic CAN data frame with a 29-bit extended ID**:

| Bits | Field | Meaning |
| --- | --- | --- |
| 28..26 | priority | 0 wins arbitration .. 7 |
| 25..24 | kind | 0 sample, 1 system, 2..3 reserved |
| 23..16 | source | node id (`lbr_node_id_t`) |
| 15..0 | sub-id | signal id (kind 0) or system message id (kind 1) |

Lower IDs win arbitration, so priority dominates, then kind, then source. Because the source
node is part of the ID, every ID has exactly one transmitter, as classic CAN requires, and any
listener can attribute a frame without payload bytes. Standard-ID and remote frames are not
LBR traffic; receivers count and ignore them.

### Node ids

| Id | Node |
| --- | --- |
| 0x00 | invalid |
| 0x01 | `gateway` |
| 0x02 | `imu_gps` |
| 0x03 | `suspension` |
| 0x04 | `driver_inputs` |
| 0x05 | `temperature` |
| 0x06 | `fuel_gauge` |
| 0x07 | `stress` |
| 0x08 | `dashboard` (cockpit display) |
| 0xFF | reserved |

### Priorities

| Priority | Used for |
| ---: | --- |
| 0 | `FAULT` |
| 1 | time sync |
| 2 | `EVENT` |
| 3 | driver inputs, dashboard buttons |
| 4 | dynamics: IMU, suspension, strain, speed |
| 5 | main temperatures, fuel, GPS position/speed/course, orientation, magnetometer, electrical |
| 6 | heartbeats, slow status: IMU temperature/calibration, GPS fix quality/altitude/UTC, ambient and auxiliary temperatures |
| 7 | unused |

Each signal's priority is in the catalog (`can_prio`).

## Sample frames (kind 0, dlc 8)

| Bytes | Field |
| --- | --- |
| 0..3 | value bits (or the unavailable reason when not valid) |
| 4..6 | low 24 bits of the sample's shared timestamp (us) |
| 7 | bits 0..5 seq (low 6 bits); bit 6 synced; bit 7 valid |

The receiver rebuilds the full timestamp from its own shared clock at reception: the stamp is the
value nearest the receive time with those low 24 bits (`lbr_can_unwrap_time`). This is exact while
a frame is delivered within +/- 8.38 s of being stamped; nodes publish immediately, so real
latency is well under a millisecond. The value type is not on the wire: it is fixed per signal by
the catalog, and unknown signals decode as `raw`.

## System frames (kind 1)

| Sub-id | Message | Payload |
| --- | --- | --- |
| 0x0001 | `TIME_SYNC` (gateway) | dlc 3: `seq u8, epoch u16` |
| 0x0002 | `FOLLOW_UP` (gateway) | dlc 8: `seq u8`, then the gateway time (us, 56 bits) at which `TIME_SYNC` `seq` finished transmitting |
| 0x0100..0x010B | heartbeat page 0..11 | dlc 8: one health page (`docs/protocol.md`) |

The gateway sends the time-sync pair at 4 Hz (`LBR_TIME_SYNC_HZ`). Every node sends its 12
heartbeat pages once per second, so any single page can be lost without losing the others.

## Gateway log record (22 bytes)

The gateway logs every frame as `t_us u64, id u32, flags u8 (bit 0 extended, bit 1 sent by the
gateway itself), dlc u8, data[8]` (zero padded). See `docs/protocol.md`.

## Bandwidth budget

Worst-case frame size (extended ID, 8 data bytes, maximum bit stuffing, interframe space) is
**160 bits**. Every node's configuration is checked at boot against a per-node share
(`CONFIG_LBR_CAN_NODE_BUDGET_PERMILLE`, default 15%), and the host tests check every node plan plus
the gateway against a **system ceiling of 60%**. The rest is headroom for error frames,
retransmissions, and growth.

At the provisional **1 Mbit/s** (current plans, from `test_plans`):

| Node | Frames/s | Load | Share |
| --- | ---: | ---: | ---: |
| `imu_gps` | 814 | 130.2 kbit/s | 13.0% |
| `suspension` | 812 | 129.9 kbit/s | 13.0% |
| `driver_inputs` | 462 | 73.9 kbit/s | 7.4% |
| `temperature` | 71 | 11.4 kbit/s | 1.1% |
| `fuel_gauge` | 22 | 3.5 kbit/s | 0.4% |
| `stress` | 812 | 129.9 kbit/s | 13.0% (1 kHz logged locally, 100 Hz on CAN) |
| `dashboard` | 32 | 5.1 kbit/s | 0.5% |
| `gateway` | 20 | 3.2 kbit/s | 0.3% |
| **total** | **3045** | **487.2 kbit/s** | **48.7%** |

The same plans would need 97% of a 500 kbit/s bus, which is why 1 Mbit/s is the default. A
vehicle harness is far shorter than the ~40 m limit at that rate, but confirm against the final
harness (#15). If the bus gets tight: decimate on CAN behind a local log (as `stress` does), lower
rates in the plans, pack several channels per frame (a protocol change), or move to CAN FD (needs
CAN-FD-capable controllers; the classic ESP32 TWAI is not).

## To confirm with hardware (#15)

- [ ] bit rate (1 Mbit/s provisional) and sample point against harness length and stubs
- [ ] termination (120 ohm at both physical ends) and topology
- [ ] transceiver part and its standby/silent pin handling
