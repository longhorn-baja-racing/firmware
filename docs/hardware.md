# Hardware

> **Status: not finalized.** Per-node GPIO assignments are TBD and must come from each node's
> schematic. Do not hard-code pins or assign line semantics until the electronics team confirms
> them. Every pin below is a Kconfig setting that defaults to `-1`; the firmware refuses to drive
> a pin that is still `-1` and reports it (heartbeat state, `CAN_PINS_UNSET` fault, log text).

## Nodes

One ESP32 per sensor project. Zones and parts are TBD; the signal list of each node is fixed by
its plan (`car/nodes/<name>/main/<name>_plan.c`).

| Node | Zone | Sensors (parts TBD) | SD card |
| --- | --- | --- | --- |
| `gateway` | cockpit | none: CAN logger, time master, LoRa/USB/WiFi/BLE | yes (authoritative log) |
| `imu_gps` | chassis, near CoG | IMU (accel, gyro, orientation, magnetometer), GPS receiver | no |
| `suspension` | chassis | four shock/linkage travel sensors | no |
| `driver_inputs` | cockpit / pedal box | throttle position, front/rear brake pressure, steering angle, brake switch | no |
| `temperature` | engine bay | engine, CVT, gearbox, brake, ambient, four auxiliary | no |
| `fuel_gauge` | tank | fuel level sender | no |
| `stress` | frame / suspension members | up to eight strain gauge channels | yes (full-rate redundancy) |
| `dashboard` | cockpit | driver display and buttons | no |

Unassigned signals already in the catalog: `ENGINE_RPM`, `VEHICLE_SPEED`, `WHEEL_SPEED_*`,
`BATT_VOLTAGE`, `BATT_CURRENT`. Give each an owner node (or a new node) before it is wired.

## Pin settings (Kconfig, all default -1)

Set per node with `idf.py menuconfig` or in that node's `sdkconfig.defaults` once confirmed.

| Function | Settings | Nodes |
| --- | --- | --- |
| CAN (TWAI) | `CONFIG_LBR_CAN_TX_GPIO`, `CONFIG_LBR_CAN_RX_GPIO` | all |
| microSD over SPI | `CONFIG_LBR_SD_SPI_{MOSI,MISO,SCLK,CS}_GPIO`, `CONFIG_LBR_SD_CD_GPIO` | gateway, stress |
| microSD over SDMMC | `CONFIG_LBR_SD_BUS_SDMMC` (fixed ESP32 slot-1 pins), `CONFIG_LBR_SD_SDMMC_4BIT` | alternative |
| LoRa (SX127x, SPI3) | `CONFIG_LBR_LORA_{SCLK,MOSI,MISO,CS,RST,DIO0}_GPIO` | gateway |
| USB serial | UART0 through the module's USB-UART bridge | gateway |
| GPS (NMEA) | `CONFIG_LBR_GPS_{UART_NUM,RX_GPIO,TX_GPIO,BAUD}` | imu_gps |
| On-chip ADC / pulse / digital inputs | `CONFIG_LBR_IO_<SLOT>` input specs (pins inside the spec, see [`sensors.md`](sensors.md)) | sensor nodes |
| Sensor buses (I2C/SPI parts) | per driver, when parts are chosen | sensor nodes |

`dev/ci/sdkconfig.linkcheck` contains placeholder pins used only so CI can link every
hardware path. It is **not** a pin map.

## CAN bus

- Bit rate: `CONFIG_LBR_CAN_BITRATE`, **1 Mbit/s provisional** (see the budget in
  [`can-ids.md`](can-ids.md)); every node must match.
- Termination: 120 ohm at both physical ends of the trunk only.
- Transceiver: 3.3 V part (e.g. SN65HVD230, TJA1051T/3) so TX/RX connect directly to the ESP32.

## Board and memory

- ESP32 (classic) with **4 MB flash** assumed by `car/nodes/sdkconfig.common`; the gateway uses two
  1.875 MB OTA slots. Confirm the module.
- The gateway runs WiFi, BLE, CAN, SD, and LoRa together. With everything linked it needs the
  WiFi IRAM optimizations moved to flash (set in `car/nodes/gateway/sdkconfig.defaults`).
- The TWAI interrupt is cache-safe (`CONFIG_TWAI_ISR_CACHE_SAFE`), so CAN reception and time
  sync keep running while flash is written (OTA, config push, boot counter).

## To verify against the final schematic and harness before bring-up (#15)

- [ ] per-node GPIO for every setting above (analog sensors on ADC1 pins 32..39 only; ADC2
      cannot be used)
- [ ] dividers that keep every analog input inside 150..2450 mV, and conditioning (pull-ups,
      filtering) on pulse pickups
- [ ] voltage levels and pull-ups on CAN, SD, sensor buses
- [ ] CAN bit rate, sample point, termination, and topology
- [ ] SD card-detect, LoRa reset/DIO0 wiring, power control of radios and sensors
- [ ] brownout threshold vs the vehicle supply (brownout resets are reported next boot)
