#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/*
 * Bounded byte ring of variable-length records. Each record is stored with a
 * 2-byte length prefix and is pushed/popped whole (all or nothing), so a full
 * ring rejects a record instead of splitting it. Push never blocks: the caller
 * counts the rejection. Not thread-safe; callers serialize access.
 */
typedef struct {
    uint8_t *mem;
    size_t   cap;
    size_t   head; /* next write offset */
    size_t   tail; /* next read offset */
    size_t   used; /* bytes in use including length prefixes */
    size_t   high_water;
} lbr_ringbuf_t;

#define LBR_RINGBUF_OVERHEAD 2u

void lbr_ringbuf_init(lbr_ringbuf_t *rb, uint8_t *mem, size_t cap);

/* Returns true if the record was stored. `len` must be 1..65535. */
bool lbr_ringbuf_push(lbr_ringbuf_t *rb, const uint8_t *rec, size_t len);

/* Push one record assembled from `n` pieces (scatter/gather). */
typedef struct {
    const uint8_t *data;
    size_t         len;
} lbr_iov_t;

bool lbr_ringbuf_pushv(lbr_ringbuf_t *rb, const lbr_iov_t *iov, size_t n);

/* Length of the oldest record, or 0 if empty. */
size_t lbr_ringbuf_peek_len(const lbr_ringbuf_t *rb);

/* Pop the oldest record into `out`. Returns its length, or 0 if empty or if
 * `cap` is too small (the record then stays queued). */
size_t lbr_ringbuf_pop(lbr_ringbuf_t *rb, uint8_t *out, size_t cap);

static inline size_t lbr_ringbuf_free(const lbr_ringbuf_t *rb)
{
    return rb->cap - rb->used;
}

static inline bool lbr_ringbuf_empty(const lbr_ringbuf_t *rb)
{
    return rb->used == 0;
}
