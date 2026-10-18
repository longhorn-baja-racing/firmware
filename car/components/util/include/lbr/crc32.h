#pragma once

#include <stddef.h>
#include <stdint.h>

/*
 * CRC-32 (IEEE 802.3: reflected polynomial 0xEDB88320, init 0xFFFFFFFF,
 * final xor 0xFFFFFFFF). Check value: crc32("123456789") == 0xCBF43926.
 * Used for log records and transport packets (CAN has its own hardware CRC).
 */
uint32_t lbr_crc32(const void *data, size_t len);

/* Incremental form: start with crc = 0, feed chunks, result equals one-shot. */
uint32_t lbr_crc32_update(uint32_t crc, const void *data, size_t len);
