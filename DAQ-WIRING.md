# Longhorn Baja Racing, DAQ System Outline

Author: Cyrus Nilforooshan
Items marked **(proposed)** are a starting design, not settled. Items marked **TBD** are undecided.

## 1. Overview

- 10 controllers (ESP32 / ESP32-C5) on one CAN bus, powered from the car's central battery: 6 on the chassis and 4 wheel hub nodes (Mini ESP32-C5), one at each corner, each with its suspension IMU.
- 1 wireless Cockpit Controller (ESP32-C5) with its own battery, receiving data over WiFi from the Central Controller.
- Central Controller sends telemetry off the car over LoRa, logs the whole CAN bus, and has the GPS receiver wired to it.

## 2. Link types

- **Wired sensor link:** every sensor is wired directly to its controller. The interface for each sensor (analog/ADC, digital GPIO, I2C, SPI, UART, etc.) is **TBD** and will follow the sensor manufacturer's recommendation. Pin assignments are **TBD**. The diagram labels these "GPIO" only as a placeholder.
- **CAN:** backbone between all 10 controllers, wheel hub nodes included.
- **5 GHz WiFi:** Central Controller to Cockpit Controller only. Only ESP32-C5 boards are used for WiFi links.

## 3. Power

- **Central battery powers:** IMU Controller, Powertrain Controller, Driver Input Controller, Temperature Controller, Stress Controller, Central Controller, and the 4 wheel hub nodes (and their wired sensors, including the GPS receiver).
- **Own battery pack:** the Cockpit Controller (on a moving part). It has no wired connection to the rest of the car and communicates only over WiFi.
- **(proposed)** Run power and CAN together in one 4-wire harness to each controller: battery+, GND, CAN_H, CAN_L. Each controller gets a buck regulator from battery voltage down to 5 V / 3.3 V. Battery voltage is **TBD** (likely 12 V).
- The harness to each wheel hub node crosses the suspension travel: flex-rated cable, strain relief, and a sealed connector (**TBD** with the harness).
- **The Cockpit Controller must last a full Baja endurance race (4 hours) plus margin.** Battery capacity, charging method, and low-battery reporting are **TBD**.

## 4. Nodes and what connects to them

### ESP32-C5, IMU Controller (central battery, CAN node 1)
- Chassis IMU (wired)

### ESP32, Powertrain Controller (central battery, CAN node 2)
- Fuel Level Sensor (wired)
- Engine Speed (wired)
- CVT Speed (wired)

### ESP32, Driver Input Controller (central battery, CAN node 3)
- Brake Pedal Compression (wired)
- Gas Pedal Compression (wired)
- Steering Angle (wired)

### ESP32, Temperature Controller (central battery, CAN node 4)
- FL Brake Temp, FR Brake Temp, BL Brake Temp, BR Brake Temp (wired)
- CVT Temp (wired)

### ESP32, Stress Controller (central battery, CAN node 5)
- Stress Sensors 1 through 8 (wired)

### Wheel hub nodes (4x Mini ESP32-C5, central battery, CAN nodes 7 to 10)
- Mini ESP32-C5, FL wheel hub (CAN node 7): FL Suspension IMU (wired)
- Mini ESP32-C5, FR wheel hub (CAN node 8): FR Suspension IMU (wired)
- Mini ESP32-C5, BL wheel hub (CAN node 9): BL Suspension IMU (wired)
- Mini ESP32-C5, BR wheel hub (CAN node 10): BR Suspension IMU (wired)
- Each sends its own IMU data onto CAN. One firmware runs on all four; the position is set at build time.

### ESP32-C5, Central Controller (central battery, CAN node 6)
- LoRa Antenna / radio (wired; off-car telemetry)
- GPS receiver (wired; any NMEA 0183 module on a UART). Its position, speed, and fix go on CAN like any other sensor (0x600 to 0x602).
- Sends data to the Cockpit Controller over 5 GHz WiFi.
- Owns system time sync on CAN (see CAN ID map).
- Logs every CAN frame. The firmware writes the log to a microSD card on the Central Controller; the card hardware is **TBD**.

### ESP32-C5, Cockpit Controller (own battery)
- Receives data from the Central Controller over 5 GHz WiFi.
- No sensors attached.

## 5. Topology

```
                          [Chassis IMU] --> [ESP32-C5 IMU Controller]
                                                          | CAN
[FL Susp IMU]--[Mini C5 FL hub] --------------------------|
[FR Susp IMU]--[Mini C5 FR hub] --------------------------|  (routing to the hubs TBD)
[BL Susp IMU]--[Mini C5 BL hub] --------------------------|
[BR Susp IMU]--[Mini C5 BR hub] --------------------------|
                                                          | CAN
         Fuel Level, Engine Speed, CVT Speed --> [ESP32 Powertrain Controller]
                                                          | CAN
  Brake Pedal, Gas Pedal, Steering Angle --> [ESP32 Driver Input Controller]
                                                          | CAN
FL/FR/BL/BR Brake Temp, CVT Temp --> [ESP32 Temperature Controller]
                                                          | CAN
                     Stress Sensors 1-8 --> [ESP32 Stress Controller]
                                                          | CAN
               [GPS receiver] --> [ESP32-C5 Central Controller] --- [LoRa radio]
                                                          ~ 5GHz WiFi
                            [ESP32-C5 Cockpit Controller]

--  = wired, interface TBD per sensor
~~  = 5 GHz WiFi
|   = CAN (one shared bus; physical order, ends, and termination TBD with the harness)
[Cockpit] = own battery; everything else = central battery
```

## 6. CAN bus spec (proposed)

- One shared bus of 10 nodes. How it reaches the 4 wheel hubs (daisy-chained through them, or short drops off the trunk) is **TBD** with the harness.
- Bitrate: 500 kbit/s to start (robust on a vibrating, noisy car). Move to 1 Mbit/s only if bus load measures too high.
- Standard 11-bit IDs. Lower ID = higher priority on the bus.
- Each ESP32 uses its built-in CAN controller (TWAI) plus an external 3.3 V CAN transceiver chip (e.g. SN65HVD230 or similar).
- 120 ohm termination resistor at each physical end of the bus only (which nodes sit at the ends is **TBD**).
- Twisted pair for CAN_H/CAN_L.
- Multi-byte values are little-endian. Signed values are int16 unless noted.
- Target bus load under ~50%. Estimated load with the rates below is roughly 40% at 500 kbit/s; verify on the real bus.

## 7. CAN ID map (proposed)

There are no pre-assigned or standard IDs for this system. This scheme groups by priority (safety/driver inputs first, slow data last).

The firmware implements this map as a schema (`car/components/protocol/frames.c`, exported to `dev/tests/fixtures/catalog.json`), so the dashboard decodes frames from the table, not from hand-written layouts. Rows marked **(added)** are not in the original proposal: the firmware needs them for precise time sync, events/faults, the GPS on the Central, the cockpit link, and full health reporting. Bit-level details are in `docs/can-ids.md`. Every id has exactly one sender, the node that samples its data.

| ID | Sender | Content | Rate | Payload (bytes) |
|---|---|---|---|---|
| 0x001 | Central | Time sync | 10 Hz | 0-3: uint32 milliseconds since boot, 4: sync seq, 5-6: epoch (changes every Central boot) |
| 0x002 | Central | Time sync follow-up **(added)** | 10 Hz | 0: sync seq, 1-7: exact time (us) the matching 0x001 left the Central |
| 0x010 | Central | Command (start/stop logging, etc.) | on event | 0: command code, 1-7: args |
| 0x021-0x02A | node 1-10 | Event / fault **(added)** | on event | 0-1: code, 2-3: arg, 4: fault/synced/seq bits, 5-7: time (us, low 24 bits) |
| 0x100 | Driver Input | Pedals + steering | 100 Hz | 0-1: brake, 2-3: gas, 4-5: steering angle, 6-7: uint16 sample counter |
| 0x200 | Powertrain | Speeds | 100 Hz | 0-1: uint16 engine RPM, 2-3: uint16 CVT RPM, 6-7: uint16 sample counter |
| 0x201 | Powertrain | Fuel level | 1 Hz | 0-1: fuel level |
| 0x300 | FL hub | FL IMU accel | 100 Hz | 0-5: accel X/Y/Z, 6-7: uint16 sample counter |
| 0x301 | FL hub | FL IMU gyro | 100 Hz | 0-5: gyro X/Y/Z, 6-7: uint16 sample counter |
| 0x302 | FR hub | FR IMU accel | 100 Hz | same as 0x300 |
| 0x303 | FR hub | FR IMU gyro | 100 Hz | same as 0x301 |
| 0x304 | BL hub | BL IMU accel | 100 Hz | same as 0x300 |
| 0x305 | BL hub | BL IMU gyro | 100 Hz | same as 0x301 |
| 0x306 | BR hub | BR IMU accel | 100 Hz | same as 0x300 |
| 0x307 | BR hub | BR IMU gyro | 100 Hz | same as 0x301 |
| 0x308 | IMU Ctrl | Chassis IMU accel | 100 Hz | same as 0x300 |
| 0x309 | IMU Ctrl | Chassis IMU gyro | 100 Hz | same as 0x301 |
| 0x400 | Stress | Stress sensors 1-4 | 100 Hz | 0-7: four int16 values |
| 0x401 | Stress | Stress sensors 5-8 | 100 Hz | 0-7: four int16 values |
| 0x500 | Temperature | Brake temps | 10 Hz | 0-7: FL, FR, BL, BR as int16 (0.1 deg C units) |
| 0x501 | Temperature | CVT temp | 10 Hz | 0-1: int16 (0.1 deg C units) |
| 0x600 | Central | GPS position **(added)** | 10 Hz | 0-3: int32 latitude, 4-7: int32 longitude (1e-7 deg) |
| 0x601 | Central | GPS motion **(added)** | 10 Hz | 0-1: uint16 speed (0.01 m/s), 2-3: uint16 course (0.01 deg), 4-5: int16 altitude (0.1 m), 6-7: uint16 sample counter |
| 0x602 | Central | GPS status **(added)** | 10 Hz | 0-3: uint32 UTC (Unix s), 4: fix type, 5: satellites, 6-7: uint16 HDOP (0.01) |
| 0x60A | Central | Cockpit status **(added)** | 1 Hz | 0: cockpit battery %, 1: link OK |
| 0x701 | IMU Ctrl | Heartbeat | 1 Hz | 0: state, 1: error flags |
| 0x702 | Powertrain | Heartbeat | 1 Hz | same |
| 0x703 | Driver Input | Heartbeat | 1 Hz | same |
| 0x704 | Temperature | Heartbeat | 1 Hz | same |
| 0x705 | Stress | Heartbeat | 1 Hz | same |
| 0x706 | Central | Heartbeat | 1 Hz | same |
| 0x707-0x70A | FL, FR, BL, BR hub | Heartbeat | 1 Hz | same |
| 0x700 + 16n + p | node n (1-10) | Health detail page p (0-11) **(added)** | 1 Hz | 0-7: one page of the full health record (versions, counters, bus errors, sync error) |

Scaling/units for pedals, steering, fuel, IMU, and stress values are **TBD** until sensors are chosen; the firmware uses provisional scales (pedals 0.01 %, steering 0.1 deg, fuel 0.1 %, accel 0.01 m/s^2, gyro 0.001 rad/s, stress 1 microstrain) recorded in the frame table, so changing one is a one-line schema change. Rates are starting points.

Firmware conventions on top of the table:

- **Sample counter = time.** Every node samples on a grid of the shared clock (sample k at k x period), so the uint16 counter is k mod 65536 and the Central rebuilds each sample's exact time from it. Frames without a counter (0x201, 0x400/0x401, 0x500/0x501, 0x600, 0x602, 0x60A) are stamped at the last grid point before reception, which holds while a frame is delivered within one period.
- **Missing values are never zero.** Each integer field reserves its 8 lowest (signed) or highest (unsigned) raw values for "unavailable, because ...": not fitted, no data, sensor fault, out of range, not calibrated, bus error. A reading that does not fit its field is sent as out of range.
- **Heartbeat error flags** (byte 1 of 0x70n): bit0 clock not synced, bit1 sensor fault, bit2 CAN errors, bit3 config rejected, bit4 storage, bit5 simulated data, bit6 data lost this second, bit7 cockpit WiFi link down (Central).
- **Node presence.** The Central tracks every node by its heartbeat and reports a node that goes quiet (and comes back) as an event, wheel hubs included.
- Planned load from this table: about 43 % of 500 kbit/s (the host tests check it stays under 50 %).

## 8. Wireless links (proposed)

The Cockpit Controller is the only wireless node.

- **Central -> Cockpit:** 5 GHz, UDP, Central pushes display data at a modest rate (e.g. 10-20 Hz). No TCP, to avoid retransmit stalls.
- **Band fallback:** ESP32-C5 is dual-band, so keep the band configurable (5 GHz default, 2.4 GHz fallback) and test both on the car.
- **Battery reporting:** the cockpit reports its battery level so it can be logged and shown.

What the firmware does today (`car/components/wlink`, exercised by the simulator in `dev/sim`):

- UDP (ports 47801 cockpit, 47802 cockpit status), one CRC-checked packet per datagram with a per-sender sequence number, so lost datagrams are counted.
- The Central sends display data 10 times a second; the cockpit reports its battery once a second.
- A cockpit silent for 3 s is reported lost (event + 0x60A bit) and found again when it comes back.
- Band: 5 GHz by default, 2.4 GHz with one menuconfig switch (`CONFIG_LBR_WIFI_5GHZ`).
- Every CAN node, wheel hubs included, syncs to the Central over CAN within about 50 us.

## 9. LoRa telemetry

- LoRa bandwidth is very low (a few kbit/s at most), so only a summary is sent off-car (e.g. RPM, speeds, temps, fuel, cockpit battery, heartbeat status at a few Hz). Full-rate data is logged on the car. Exact telemetry packet contents are **TBD**.

## 10. Sensor count summary

- IMUs: 5 (4 suspension IMUs on the wheel hub nodes, 1 chassis IMU wired to the IMU Controller)
- GPS: 1 (wired to the Central Controller)
- Powertrain: 3 (fuel level, engine speed, CVT speed)
- Driver inputs: 3 (brake pedal, gas pedal, steering angle)
- Temperatures: 5 (4 brake, 1 CVT)
- Stress sensors: 8
- Total: 25 sensors, 10 CAN controllers (4 of them wheel hub nodes), 1 cockpit node, 1 LoRa radio

## 11. Still TBD

- Sensor part numbers, interfaces (ADC / GPIO / I2C / SPI / UART), and scaling
- Pin assignments on each ESP32
- CAN transceiver part and connector type
- Battery voltage, regulators, fuses
- CAN routing to the 4 wheel hubs, the bus ends, and the hub harness (flex cable, connectors)
- Battery capacity and charging for the cockpit
- Data logging hardware (the firmware logs to a microSD card on the Central Controller; the log format is in `docs/protocol.md`)
- GPS module part and antenna placement
- LoRa packet contents and rate
