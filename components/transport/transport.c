#include "lbr/transport.h"

/*
 * No concrete transports yet. Implementations (LoRa over SPI, USB serial, WiFi,
 * BLE) live here or in node-local code and populate an lbr_transport_t vtable.
 * This translation unit keeps the component buildable until then.
 */
