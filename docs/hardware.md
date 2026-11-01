# Hardware

> **Status: not finalized.** Per-node GPIO assignments are TBD and must come from each node's
> schematic. Do not hard-code pins or assign line semantics until the electronics team confirms
> them. Every pin below is a Kconfig setting that defaults to `-1`; the firmware refuses to drive
> a pin that is still `-1` and reports it (heartbeat state, `CAN_PINS_UNSET` fault, log text).

## Nodes

The node set, power, and topology are [`DAQ-WIRING.md`](../DAQ-WIRING.md). Parts are TBD; the
signal list of each node is fixed by its plan (`car/nodes/<name>/main/<name>_plan.c`).

| Node | Board | Power | Links | Sensors (parts TBD) | SD card |
| --- | --- | --- | --- | --- | --- |
| `imu` | ESP32-C5 | central battery | CAN (end of the trunk) | chassis IMU | no |
| `powertrain` | ESP32 | central battery | CAN | fuel level, engine speed, CVT speed | no |
| `driver_input` | ESP32 | central battery | CAN | brake pedal, gas pedal, steering angle | no |
| `temperature` | ESP32 | central battery | CAN | four brake temperatures, CVT temperature | no |
| `stress` | ESP32 | central battery | CAN | stress sensors 1..8 (bridge amplifier/ADC part) | no |
| `central` | ESP32-C5 | central battery | CAN (end of the trunk), LoRa, USB, BLE, pit/dashboard WiFi access point | GPS (NMEA) | yes (authoritative log; hardware TBD) |
| `corner` (x4) | Mini ESP32-C5 | central battery | CAN | suspension IMU | no |
| `dashboard` | ESP32-C5 | own battery | WiFi to the Central | own battery level | no |

One `corner` firmware runs on all four wheel hub nodes; `CONFIG_LBR_CORNER_POSITION` (FL, FR, BL,
BR) picks the node id (7..10). Each node's chip is set in its `sdkconfig.defaults`
(`CONFIG_IDF_TARGET`); `idf.py set-target` is only needed to change it.

## Pin settings (Kconfig, all default -1)

Set per node with `idf.py menuconfig` or in that node's `sdkconfig.defaults` once confirmed.

| Function | Settings | Nodes |
| --- | --- | --- |
| CAN (TWAI) | `CONFIG_LBR_CAN_TX_GPIO`, `CONFIG_LBR_CAN_RX_GPIO` | the ten CAN nodes |
| microSD over SPI | `CONFIG_LBR_SD_SPI_{MOSI,MISO,SCLK,CS}_GPIO`, `CONFIG_LBR_SD_CD_GPIO` | central |
| microSD over SDMMC | `CONFIG_LBR_SD_BUS_SDMMC`, `CONFIG_LBR_SD_SDMMC_4BIT` | alternative (classic ESP32 slot-1 pins) |
| LoRa (SX127x, SPI) | `CONFIG_LBR_LORA_{SCLK,MOSI,MISO,CS,RST,DIO0}_GPIO` | central (on the C5 it shares the SD card's SPI bus: SCLK/MOSI/MISO must equal the SD pins, or LoRa start fails; own CS) |
| USB serial | UART0 through the module's USB-UART bridge (or the C5's USB serial) | central |
| GPS (NMEA) | `CONFIG_LBR_GPS_{UART_NUM,RX_GPIO,TX_GPIO,BAUD}` | central |
| On-chip ADC / pulse / digital inputs | `CONFIG_LBR_IO_<SLOT>` input specs (pins inside the spec, see [`sensors.md`](sensors.md)) | powertrain, driver_input, temperature, dashboard battery |
| Sensor buses (I2C/SPI parts) | per driver, when parts are chosen | IMUs, stress, brake temperatures |

`dev/ci/sdkconfig.linkcheck` contains placeholder pins used only so CI can link every
hardware path. It is **not** a pin map.

## Wireless

| Setting | Meaning | Nodes |
| --- | --- | --- |
| `CONFIG_LBR_WIFI_5GHZ` | 5 GHz band (default), off = 2.4 GHz | central, dashboard |
| `CONFIG_LBR_WIFI_SSID`, `_PASSWORD`, `_CHANNEL` | the Central's access point (pit link + dashboard) | central, dashboard |
| `CONFIG_LBR_DASH_ENABLE` | push display data to the dashboard | central |

The password defaults to empty, which keeps the network off; set it in a local sdkconfig and never
commit it. The dashboard link is UDP; the protocol is in [`protocol.md`](protocol.md) ("Dashboard
link"). Every other node is on CAN only.

## CAN bus

- Bit rate: `CONFIG_LBR_CAN_BITRATE`, **500 kbit/s** (DAQ-WIRING.md; see the budget in
  [`can-ids.md`](can-ids.md)); every node must match.
- One shared bus of ten nodes: imu, powertrain, driver_input, temperature, stress, central, and
  the four wheel hub nodes; twisted pair, 120 ohm termination at the two physical ends only. How
  the bus reaches the wheel hubs (daisy-chained through them, or short drops off the trunk) and
  which nodes sit at the ends are TBD with the harness.
- Transceiver: 3.3 V part (e.g. SN65HVD230, TJA1051T/3) so TX/RX connect directly to the ESP32.
- Proposed harness: battery+, GND, CAN_H, CAN_L in one 4-wire run to each controller, wheel hubs
  included, with a buck regulator per controller (battery voltage TBD). The run to each wheel hub
  crosses the suspension travel: flex-rated cable, strain relief, and a sealed connector.

## Board and memory

- 4 MB flash assumed by `car/nodes/sdkconfig.common`; the Central uses two 1.875 MB OTA slots.
  Confirm the modules.
- The Central runs WiFi, BLE, CAN, SD, LoRa, and GPS together; its WiFi IRAM optimizations are
  moved to flash (`car/nodes/central/sdkconfig.defaults`).
- The TWAI interrupt is cache-safe (`CONFIG_TWAI_ISR_CACHE_SAFE`), so CAN reception and time
  sync keep running while flash is written (OTA, config push, boot counter).
- The dashboard, the only battery-powered node, must last a 4 hour endurance race plus margin. Its
  WiFi runs without power save for latency; capacity, charging, and low-battery behavior are TBD.

## To verify against the final schematic and harness before bring-up (#15)

- [ ] per-node GPIO for every setting above (classic ESP32 analog inputs on ADC1 pins 32..39,
      ESP32-C5 on ADC1 pins 1..6; ADC2 cannot be used with WiFi; C5 pins 15..22 belong to flash)
- [ ] dividers that keep every analog input inside 150..2450 mV, and conditioning (pull-ups,
      filtering) on pulse pickups
- [ ] voltage levels and pull-ups on CAN, SD, sensor buses
- [ ] CAN bit rate, sample point, termination, and topology (routing to the four wheel hubs, stub
      lengths if they hang off the trunk)
- [ ] the wheel hub harness: flex life over the suspension travel, connector sealing, and
      shielding near the brakes
- [ ] SD card-detect, LoRa reset/DIO0 wiring, power control of radios and sensors
- [ ] the Central's shared SPI bus (SD card + LoRa radio on the ESP32-C5's one SPI host)
- [ ] brownout threshold vs the vehicle supply (brownout resets are reported next boot)
- [ ] 5 GHz range from the cockpit to the Central, on the car, and the 2.4 GHz fallback
