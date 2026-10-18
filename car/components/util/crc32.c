#include "lbr/crc32.h"

/* Nibble-table CRC: 64 bytes of table, fast enough for log/transport rates. */
static const uint32_t s_table[16] = {
    0x00000000u, 0x1DB71064u, 0x3B6E20C8u, 0x26D930ACu, 0x76DC4190u, 0x6B6B51F4u,
    0x4DB26158u, 0x5005713Cu, 0xEDB88320u, 0xF00F9344u, 0xD6D6A3E8u, 0xCB61B38Cu,
    0x9B64C2B0u, 0x86D3D2D4u, 0xA00AE278u, 0xBDBDF21Cu,
};

uint32_t lbr_crc32_update(uint32_t crc, const void *data, size_t len)
{
    const uint8_t *p = data;
    crc              = ~crc;
    while (len--) {
        crc ^= *p++;
        crc = (crc >> 4) ^ s_table[crc & 0x0Fu];
        crc = (crc >> 4) ^ s_table[crc & 0x0Fu];
    }
    return ~crc;
}

uint32_t lbr_crc32(const void *data, size_t len)
{
    return lbr_crc32_update(0, data, len);
}
