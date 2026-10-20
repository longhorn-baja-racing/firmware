#include "lbr/ringbuf.h"

static void ring_write(lbr_ringbuf_t *rb, const uint8_t *src, size_t len)
{
    for (size_t i = 0; i < len; i++) {
        rb->mem[rb->head] = src[i];
        rb->head          = (rb->head + 1 == rb->cap) ? 0 : rb->head + 1;
    }
}

static void ring_read(const lbr_ringbuf_t *rb, size_t from, uint8_t *dst, size_t len)
{
    for (size_t i = 0; i < len; i++) {
        dst[i] = rb->mem[from];
        from   = (from + 1 == rb->cap) ? 0 : from + 1;
    }
}

void lbr_ringbuf_init(lbr_ringbuf_t *rb, uint8_t *mem, size_t cap)
{
    rb->mem        = mem;
    rb->cap        = cap;
    rb->head       = 0;
    rb->tail       = 0;
    rb->used       = 0;
    rb->high_water = 0;
}

bool lbr_ringbuf_push(lbr_ringbuf_t *rb, const uint8_t *rec, size_t len)
{
    lbr_iov_t iov = {rec, len};
    return lbr_ringbuf_pushv(rb, &iov, 1);
}

bool lbr_ringbuf_pushv(lbr_ringbuf_t *rb, const lbr_iov_t *iov, size_t n)
{
    size_t len = 0;
    for (size_t i = 0; i < n; i++) {
        len += iov[i].len;
    }
    if (len == 0 || len > 0xFFFFu || len + LBR_RINGBUF_OVERHEAD > lbr_ringbuf_free(rb)) {
        return false;
    }
    uint8_t hdr[2] = {(uint8_t)len, (uint8_t)(len >> 8)};
    ring_write(rb, hdr, sizeof hdr);
    for (size_t i = 0; i < n; i++) {
        ring_write(rb, iov[i].data, iov[i].len);
    }
    rb->used += len + LBR_RINGBUF_OVERHEAD;
    if (rb->used > rb->high_water) {
        rb->high_water = rb->used;
    }
    return true;
}

size_t lbr_ringbuf_peek_len(const lbr_ringbuf_t *rb)
{
    if (rb->used == 0) {
        return 0;
    }
    uint8_t hdr[2];
    ring_read(rb, rb->tail, hdr, sizeof hdr);
    return (size_t)hdr[0] | ((size_t)hdr[1] << 8);
}

size_t lbr_ringbuf_pop(lbr_ringbuf_t *rb, uint8_t *out, size_t cap)
{
    size_t len = lbr_ringbuf_peek_len(rb);
    if (len == 0 || len > cap) {
        return 0;
    }
    size_t from = (rb->tail + LBR_RINGBUF_OVERHEAD) % rb->cap;
    ring_read(rb, from, out, len);
    rb->tail = (from + len) % rb->cap;
    rb->used -= len + LBR_RINGBUF_OVERHEAD;
    return len;
}
