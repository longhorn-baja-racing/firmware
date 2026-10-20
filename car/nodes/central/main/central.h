#pragma once

/*
 * Central Controller (ESP32-C5, CAN node 6). Responsibilities (README
 * "Central Controller"):
 *   - sniff the entire CAN bus and write the authoritative, time-aligned,
 *     full-fidelity microSD log (every frame, raw, with its receive time);
 *   - be the time master: broadcast TIME_SYNC + FOLLOW_UP;
 *   - sample its GPS and the dashboard link state into its own CAN frames;
 *   - drive the live links: LoRa in-race subset; USB / WiFi full stream;
 *     BLE status; display data to the cockpit dashboard over WiFi;
 *   - track node presence and health and report losses.
 * Every path between them is a bounded, non-blocking queue: a slow or absent
 * live link never reduces what reaches the SD card.
 */
void central_run(void);
