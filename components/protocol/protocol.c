#include "lbr/protocol.h"

/*
 * Stub implementations. The byte layout is not frozen yet (docs/protocol.md).
 * These return 0 (error/not-encoded) so callers compile and link; replace with
 * the real serialization + checksum once the shared spec defines it.
 */

size_t lbr_sample_encode(const lbr_sample_t *s, uint8_t *buf, size_t cap)
{
    (void)s;
    (void)buf;
    (void)cap;
    return 0; /* TODO */
}

size_t lbr_sample_decode(const uint8_t *buf, size_t len, lbr_sample_t *out)
{
    (void)buf;
    (void)len;
    (void)out;
    return 0; /* TODO */
}
