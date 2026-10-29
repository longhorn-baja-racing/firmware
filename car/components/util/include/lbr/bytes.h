#pragma once

#include <stdint.h>
#include <string.h>

/*
 * Little-endian load/store helpers. Every multi-byte field in the LBR wire,
 * CAN, and log formats is little-endian, independent of host byte order.
 */

static inline void lbr_put_u16(uint8_t *p, uint16_t v)
{
    p[0] = (uint8_t)v;
    p[1] = (uint8_t)(v >> 8);
}

static inline void lbr_put_u24(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)v;
    p[1] = (uint8_t)(v >> 8);
    p[2] = (uint8_t)(v >> 16);
}

static inline void lbr_put_u32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)v;
    p[1] = (uint8_t)(v >> 8);
    p[2] = (uint8_t)(v >> 16);
    p[3] = (uint8_t)(v >> 24);
}

static inline void lbr_put_u64(uint8_t *p, uint64_t v)
{
    lbr_put_u32(p, (uint32_t)v);
    lbr_put_u32(p + 4, (uint32_t)(v >> 32));
}

static inline uint16_t lbr_get_u16(const uint8_t *p)
{
    return (uint16_t)(p[0] | ((uint16_t)p[1] << 8));
}

static inline uint32_t lbr_get_u24(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16);
}

static inline uint32_t lbr_get_u32(const uint8_t *p)
{
    return (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
}

static inline uint64_t lbr_get_u64(const uint8_t *p)
{
    return (uint64_t)lbr_get_u32(p) | ((uint64_t)lbr_get_u32(p + 4) << 32);
}

/* Bit-exact float <-> u32 (no aliasing UB). */
static inline uint32_t lbr_f32_bits(float f)
{
    uint32_t u;
    memcpy(&u, &f, sizeof u);
    return u;
}

static inline float lbr_bits_f32(uint32_t u)
{
    float f;
    memcpy(&f, &u, sizeof f);
    return f;
}
