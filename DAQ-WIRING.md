# Longhorn Baja Racing, DAQ System Outline

Author: Cyrus Nilforooshan
Items marked **(proposed)** are a starting design, not settled. Items marked **TBD** are undecided.

## 1. Overview

- 6 wired controllers (ESP32 / ESP32-C5) on one CAN bus, powered from the car's central battery.
- 4 wireless suspension nodes (Mini ESP32-C5) on the moving suspension, each with its own battery, sending IMU data over WiFi to the IMU Controller.
- 1 wireless Dashboard Controller (ESP32-C5) with its own battery, receiving data over WiFi from the Central Controller.
- Central Controller sends telemetry off the car over LoRa, logs the whole CAN bus, and has the GPS receiver wired to it.

## 2. Link types

- **Wired sensor link:** every sensor is wired directly to its controller. The interface for each sensor (analog/ADC, digital GPIO, I2C, SPI, UART, etc.) is **TBD** and will follow the sensor manufacturer's recommendation. Pin assignments are **TBD**. The diagram labels these "GPIO" only as a placeholder.
- **CAN:** backbone between the 6 wired controllers.
- **5 GHz WiFi:** corner nodes to IMU Controller, and Central Controller to Dashboard Controller. Only ESP32-C5 boards are used for WiFi links.

## 3. Power

- **Central battery powers:** IMU Controller, Powertrain Controller, Driver Input Controller, Temperature Controller, Stress Controller, Central Controller (and their wired sensors, including the GPS receiver).
- **Own battery packs:** the 4 Mini ESP32-C5 suspension nodes (on moving suspension) and the Dashboard Controller (on a moving part). These have no wired connection to the rest of the car and communicate only over WiFi.
- **(proposed)** Run power and CAN together in one 4-wire harness to each wired controller: battery+, GND, CAN_H, CAN_L. Each wired controller gets a buck regulator from battery voltage down to 5 V / 3.3 V. Battery voltage is **TBD** (likely 12 V).
- **Battery-powered nodes must last a full Baja endurance race (4 hours) plus margin.** Battery capacity, charging method, and low-battery reporting are **TBD**.

## 4. Nodes and what connects to them

### Wireless suspension nodes (4x Mini ESP32-C5, own battery)
- Mini ESP32-C5, FL Suspension: FL Suspension IMU (wired)
- Mini ESP32-C5, FR Suspension: FR Suspension IMU (wired)
- Mini ESP32-C5, BL Suspension: BL Suspension IMU (wired)
- Mini ESP32-C5, BR Suspension: BR Suspension IMU (wired)
- Each sends IMU data over 5 GHz WiFi to the IMU Controller.

### ESP32-C5, IMU Controller (central battery, CAN node 1)
- Receives the 4 suspension nodes over 5 GHz WiFi.
- One IMU wired directly to it (labeled "FL Suspension IMU" in the diagram; treated here as a chassis IMU; see Open questions).
- Forwards all IMU data onto CAN.

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

### ESP32-C5, Central Controller (central battery, CAN node 6)
- LoRa Antenna / radio (wired; off-car telemetry)
- GPS receiver (wired; any NMEA 0183 module on a UART). Its position, speed, and fix go on CAN like any other sensor (0x600 to 0x602).
- Sends data to the Dashboard Controller over 5 GHz WiFi.
- Owns system time sync on CAN (see CAN ID map).
- Logs every CAN frame. The firmware writes the log to a microSD card on the Central Controller; the card hardware is **TBD**.

### ESP32-C5, Dashboard Controller (own battery)
- Receives data from the Central Controller over 5 GHz WiFi.
- No sensors attached.

## 5. Topology

```
[FL Susp IMU]--[Mini C5 FL] ~~\
[FR Susp IMU]--[Mini C5 FR] ~~~+~~ 5GHz WiFi ~~> [ESP32-C5 IMU Controller] <-- [Chassis IMU]
[BL Susp IMU]--[Mini C5 BL] ~~~+                          |
[BR Susp IMU]--[Mini C5 BR] ~~/                           | CAN  (120 ohm termination at this end)
                                                          |
         Fuel Level, Engine Speed, CVT Speed --> [ESP32 Powertrain Controller]
                                                          | CAN
  Brake Pedal, Gas Pedal, Steering Angle --> [ESP32 Driver Input Controller]
                                                          | CAN
FL/FR/BL/BR Brake Temp, CVT Temp --> [ESP32 Temperature Controller]
                                                          | CAN
                     Stress Sensors 1-8 --> [ESP32 Stress Controller]
                                                          | CAN
               [GPS receiver] --> [ESP32-C5 Central Controller] --- [LoRa radio]
                                                          |   (120 ohm termination at this end)
                                                          ~ 5GHz WiFi
                            [ESP32-C5 Dashboard Controller]

--  = wired, interface TBD per sensor
~~  = 5 GHz WiFi
|   = CAN (one shared bus, daisy-chained in this order)
[Mini C5 *] and [Dashboard] = own battery; everything else = central battery
```

## 6. CAN bus spec (proposed)

- One shared bus, daisy-chained in the order shown above.
- Bitrate: 500 kbit/s to start (robust on a vibrating, noisy car). Move to 1 Mbit/s only if bus load measures too high.
- Standard 11-bit IDs. Lower ID = higher priority on the bus.
- Each ESP32 uses its built-in CAN controller (TWAI) plus an external 3.3 V CAN transceiver chip (e.g. SN65HVD230 or similar).
- 120 ohm termination resistor at each physical end of the bus only (IMU Controller end and Central Controller end).
- Twisted pair for CAN_H/CAN_L.
- Multi-byte values are little-endian. Signed values are int16 unless noted.
- Target bus load under ~50%. Estimated load with the rates below is roughly 35-40% at 500 kbit/s; verify on the real bus.

## 7. CAN ID map (proposed)

There are no pre-assigned or standard IDs for this system. This scheme groups by priority (safety/driver inputs first, slow data last).

The firmware implements this map as a schema (`car/components/protocol/frames.c`, exported to `dev/tests/fixtures/catalog.json`), so the dashboard decodes frames from the table, not from hand-written layouts. Rows marked **(added)** are not in the original proposal: the firmware needs them for precise time sync, events/faults, the GPS on the Central, the dashboard link, and full health reporting. Bit-level details are in `docs/can-ids.md`.

| ID | Sender | Content | Rate | Payload (bytes) |
|---|---|---|---|---|
| 0x001 | Central | Time sync | 10 Hz | 0-3: uint32 milliseconds since boot, 4: sync seq, 5-6: epoch (changes every Central boot) |
| 0x002 | Central | Time sync follow-up **(added)** | 10 Hz | 0: sync seq, 1-7: exact time (us) the matching 0x001 left the Central |
| 0x010 | Central | Command (start/stop logging, etc.) | on event | 0: command code, 1-7: args |
| 0x021-0x026 | node 1-6 | Event / fault **(added)** | on event | 0-1: code, 2-3: arg, 4: fault/synced/seq bits, 5-7: time (us, low 24 bits) |
| 0x100 | Driver Input | Pedals + steering | 100 Hz | 0-1: brake, 2-3: gas, 4-5: steering angle, 6-7: uint16 sample counter |
| 0x200 | Powertrain | Speeds | 100 Hz | 0-1: uint16 engine RPM, 2-3: uint16 CVT RPM, 6-7: uint16 sample counter |
| 0x201 | Powertrain | Fuel level | 1 Hz | 0-1: fuel level |
| 0x300 | IMU Ctrl | FL IMU accel | 100 Hz | 0-5: accel X/Y/Z, 6-7: uint16 sample counter |
| 0x301 | IMU Ctrl | FL IMU gyro | 100 Hz | 0-5: gyro X/Y/Z, 6-7: uint16 sample counter |
| 0x302 | IMU Ctrl | FR IMU accel | 100 Hz | same as 0x300 |
| 0x303 | IMU Ctrl | FR IMU gyro | 100 Hz | same as 0x301 |
| 0x304 | IMU Ctrl | BL IMU accel | 100 Hz | same as 0x300 |
| 0x305 | IMU Ctrl | BL IMU gyro | 100 Hz | same as 0x301 |
| 0x306 | IMU Ctrl | BR IMU accel | 100 Hz | same as 0x300 |
| 0x307 | IMU Ctrl | BR IMU gyro | 100 Hz | same as 0x301 |
| 0x308 | IMU Ctrl | Chassis IMU accel | 100 Hz | same as 0x300 |
| 0x309 | IMU Ctrl | Chassis IMU gyro | 100 Hz | same as 0x301 |
| 0x30A | IMU Ctrl | Corner node status | 1 Hz | 0-3: battery % for FL/FR/BL/BR, 4: link-OK bitmask (bit0 FL ... bit3 BR), 5: clock-synced bitmask **(added)** |
| 0x400 | Stress | Stress sensors 1-4 | 100 Hz | 0-7: four int16 values |
| 0x401 | Stress | Stress sensors 5-8 | 100 Hz | 0-7: four int16 values |
| 0x500 | Temperature | Brake temps | 10 Hz | 0-7: FL, FR, BL, BR as int16 (0.1 deg C units) |
| 0x501 | Temperature | CVT temp | 10 Hz | 0-1: int16 (0.1 deg C units) |
| 0x600 | Central | GPS position **(added)** | 10 Hz | 0-3: int32 latitude, 4-7: int32 longitude (1e-7 deg) |
| 0x601 | Central | GPS motion **(added)** | 10 Hz | 0-1: uint16 speed (0.01 m/s), 2-3: uint16 course (0.01 deg), 4-5: int16 altitude (0.1 m), 6-7: uint16 sample counter |
| 0x602 | Central | GPS status **(added)** | 10 Hz | 0-3: uint32 UTC (Unix s), 4: fix type, 5: satellites, 6-7: uint16 HDOP (0.01) |
| 0x60A | Central | Dashboard status **(added)** | 1 Hz | 0: dashboard battery %, 1: link OK |
| 0x701 | IMU Ctrl | Heartbeat | 1 Hz | 0: state, 1: error flags |
| 0x702 | Powertrain | Heartbeat | 1 Hz | same |
| 0x703 | Driver Input | Heartbeat | 1 Hz | same |
| 0x704 | Temperature | Heartbeat | 1 Hz | same |
| 0x705 | Stress | Heartbeat | 1 Hz | same |
| 0x706 | Central | Heartbeat | 1 Hz | same |
| 0x780 + 16n + p | node n (1-6) | Health detail page p (0-11) **(added)** | 1 Hz | 0-7: one page of the full health record (versions, counters, bus errors, sync error) |

Scaling/units for pedals, steering, fuel, IMU, and stress values are **TBD** until sensors are chosen; the firmware uses provisional scales (pedals 0.01 %, steering 0.1 deg, fuel 0.1 %, accel 0.01 m/s^2, gyro 0.001 rad/s, stress 1 microstrain) recorded in the frame table, so changing one is a one-line schema change. Rates are starting points.

Firmware conventions on top of the table:

- **Sample counter = time.** Every node samples on a grid of the shared clock (sample k at k x period), so the uint16 counter is k mod 65536 and the Central rebuilds each sample's exact time from it. Frames without a counter (0x201, 0x30A, 0x400/0x401, 0x500/0x501, 0x600, 0x602, 0x60A) are stamped at the last grid point before reception, which holds while a frame is delivered within one period.
- **Missing values are never zero.** Each integer field reserves its 8 lowest (signed) or highest (unsigned) raw values for "unavailable, because ...": not fitted, no data, sensor fault, out of range, not calibrated, bus error. A reading that does not fit its field is sent as out of range.
- **Heartbeat error flags** (byte 1 of 0x70n): bit0 clock not synced, bit1 sensor fault, bit2 CAN errors, bit3 config rejected, bit4 storage, bit5 simulated data, bit6 data lost this second, bit7 wireless link down.
- Planned load from this table: about 42 % of 500 kbit/s (the host tests check it stays under 50 %).

## 8. Wireless links (proposed)

- **Corner nodes -> IMU Controller:** 5 GHz. IMU Controller acts as the access point/receiver; corner nodes send UDP (or ESP-NOW if supported on the chosen band in the ESP-IDF version used). No TCP, to avoid retransmit stalls.
- **Timestamp at the source:** corner nodes sample at a fixed rate and attach a sample counter (and ideally a synced timestamp) to every sample. WiFi latency jitters, so receive time must not be used as sample time.
- **Batching:** corner nodes may send several samples per packet to save power; the IMU Controller unpacks and forwards them onto CAN.
- **Link loss:** if a corner node drops out, the IMU Controller keeps running and reports it via 0x30A link-OK bits. Corner nodes may buffer locally and resend.
- **Central -> Dashboard:** 5 GHz, UDP, Central pushes display data at a modest rate (e.g. 10-20 Hz).
- **Band fallback:** ESP32-C5 is dual-band, so keep the band configurable (5 GHz default, 2.4 GHz fallback) and test both on the car.
- **Battery reporting:** corner nodes and dashboard report battery level so it can be logged and shown.

What the firmware does today (`car/components/wlink`, exercised by the simulator in `dev/sim`):

- UDP on both links (ports 47801 corners, 47802 dashboard, 47803 dashboard status), one CRC-checked packet per datagram with a per-sender sequence number, so lost datagrams are counted.
- Corner nodes send 5 samples per datagram (20 datagrams/s) and repeat the previous batch in each one, so a single lost datagram loses nothing. The IMU Controller forwards each sample onto CAN once and counts samples that never arrive.
- Corner clocks sync to the IMU Controller with a request/reply over WiFi (round-trip corrected), typically within 1 ms, well inside the 10 ms sample grid. CAN nodes sync to the Central within about 50 us.
- A corner silent for 0.5 s (the dashboard for 3 s) is reported lost (event + 0x30A / 0x60A bit) and found again when it comes back.
- Band: 5 GHz by default, 2.4 GHz with one menuconfig switch (`CONFIG_LBR_WIFI_5GHZ`).

## 9. LoRa telemetry

- LoRa bandwidth is very low (a few kbit/s at most), so only a summary is sent off-car (e.g. RPM, speeds, temps, fuel, battery/heartbeat status at a few Hz). Full-rate data is logged on the car. Exact telemetry packet contents are **TBD**.

## 10. Sensor count summary

- IMUs: 5 (4 suspension corners via wireless nodes, 1 chassis IMU wired to the IMU Controller)
- GPS: 1 (wired to the Central Controller)
- Powertrain: 3 (fuel level, engine speed, CVT speed)
- Driver inputs: 3 (brake pedal, gas pedal, steering angle)
- Temperatures: 5 (4 brake, 1 CVT)
- Stress sensors: 8
- Total: 25 sensors, 6 CAN controllers, 4 wireless corner nodes, 1 dashboard node, 1 LoRa radio

## 11. Still TBD

- Sensor part numbers, interfaces (ADC / GPIO / I2C / SPI / UART), and scaling
- Pin assignments on each ESP32
- CAN transceiver part and connector type
- Battery voltage, regulators, fuses
- Battery capacity and charging for corner nodes and dashboard
- Data logging hardware (the firmware logs to a microSD card on the Central Controller; the log format is in `docs/protocol.md`)
- GPS module part and antenna placement
- LoRa packet contents and rate
