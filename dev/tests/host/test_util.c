#include <stdlib.h>

#include "lbr/bytes.h"
#include "lbr/cobs.h"
#include "lbr/crc32.h"
#include "lbr/ringbuf.h"
#include "lbr_test.h"

static void test_crc32_check_value(void)
{
    CHECK_EQ_U(lbr_crc32("123456789", 9), 0xCBF43926u);
    CHECK_EQ_U(lbr_crc32("", 0), 0u);
    uint32_t inc = lbr_crc32_update(lbr_crc32_update(0, "1234", 4), "56789", 5);
    CHECK_EQ_U(inc, 0xCBF43926u);
}

static void test_bytes_le(void)
{
    uint8_t b[8];
    lbr_put_u64(b, 0x0102030405060708ull);
    CHECK_EQ_U(b[0], 0x08);
    CHECK_EQ_U(b[7], 0x01);
    CHECK_EQ_U(lbr_get_u64(b), 0x0102030405060708ull);
    lbr_put_u24(b, 0xABCDEF);
    CHECK_EQ_U(lbr_get_u24(b), 0xABCDEFu);
    CHECK(lbr_bits_f32(lbr_f32_bits(-1.5f)) == -1.5f);
}

static void cobs_roundtrip(const uint8_t *in, size_t len)
{
    uint8_t enc[1024];
    uint8_t dec[1024];
    size_t  n = lbr_cobs_encode(in, len, enc, sizeof enc);
    CHECK(n > 0);
    CHECK(n <= LBR_COBS_MAX_ENCODED(len));
    for (size_t i = 0; i < n; i++) {
        CHECK(enc[i] != 0);
    }
    size_t m = lbr_cobs_decode(enc, n, dec, sizeof dec);
    CHECK_EQ_U(m, len);
    CHECK(memcmp(in, dec, len) == 0);
}

static void test_cobs(void)
{
    uint8_t zeros[10] = {0};
    cobs_roundtrip(zeros, sizeof zeros);
    uint8_t mixed[] = {0x11, 0x00, 0x00, 0x22, 0x33, 0x00};
    cobs_roundtrip(mixed, sizeof mixed);

    /* Runs around the 254-byte block boundary. */
    uint8_t big[600];
    for (size_t len = 250; len < 520; len++) {
        for (size_t i = 0; i < len; i++) {
            big[i] = (uint8_t)(1 + i % 255);
        }
        cobs_roundtrip(big, len);
    }
    for (size_t i = 0; i < sizeof big; i++) {
        big[i] = (uint8_t)(rand() & 0xFF);
    }
    cobs_roundtrip(big, sizeof big);

    uint8_t out[4];
    CHECK_EQ_U(lbr_cobs_encode(big, 100, out, sizeof out), 0u); /* too small */
}

static void test_cobs_stream_resync(void)
{
    uint8_t       rxbuf[64];
    uint8_t       frame[64];
    lbr_cobs_rx_t rx;
    lbr_cobs_rx_init(&rx, rxbuf, sizeof rxbuf);

    uint8_t payload[] = {1, 0, 2, 3};
    uint8_t enc[16];
    size_t  n = lbr_cobs_encode(payload, sizeof payload, enc, sizeof enc);

    /* Garbage (a malformed frame), then a good frame. */
    uint8_t garbage[] = {0x05, 0x01, 0x00};
    size_t  got       = 0;
    for (size_t i = 0; i < sizeof garbage; i++) {
        got = lbr_cobs_rx_push(&rx, garbage[i], frame, sizeof frame);
    }
    CHECK_EQ_U(got, 0u);
    CHECK_EQ_U(rx.frames_bad, 1u);
    for (size_t i = 0; i < n; i++) {
        CHECK_EQ_U(lbr_cobs_rx_push(&rx, enc[i], frame, sizeof frame), 0u);
    }
    got = lbr_cobs_rx_push(&rx, 0, frame, sizeof frame);
    CHECK_EQ_U(got, sizeof payload);
    CHECK(memcmp(frame, payload, sizeof payload) == 0);
    CHECK_EQ_U(rx.frames_ok, 1u);

    /* Oversized frame is discarded and counted, then the stream recovers. */
    for (int i = 0; i < 100; i++) {
        lbr_cobs_rx_push(&rx, 0x7F, frame, sizeof frame);
    }
    CHECK_EQ_U(lbr_cobs_rx_push(&rx, 0, frame, sizeof frame), 0u);
    CHECK_EQ_U(rx.frames_bad, 2u);
    for (size_t i = 0; i < n; i++) {
        lbr_cobs_rx_push(&rx, enc[i], frame, sizeof frame);
    }
    CHECK_EQ_U(lbr_cobs_rx_push(&rx, 0, frame, sizeof frame), sizeof payload);
}

static void test_ringbuf(void)
{
    uint8_t       mem[32];
    uint8_t       out[32];
    lbr_ringbuf_t rb;
    lbr_ringbuf_init(&rb, mem, sizeof mem);
    CHECK(lbr_ringbuf_empty(&rb));

    uint8_t a[10];
    memset(a, 0xAA, sizeof a);
    CHECK(lbr_ringbuf_push(&rb, a, sizeof a));  /* 12 used */
    CHECK(lbr_ringbuf_push(&rb, a, sizeof a));  /* 24 used */
    CHECK(!lbr_ringbuf_push(&rb, a, sizeof a)); /* would need 36: rejected whole */
    CHECK_EQ_U(rb.high_water, 24u);
    CHECK_EQ_U(lbr_ringbuf_pop(&rb, out, 5), 0u); /* too small: stays queued */
    CHECK_EQ_U(lbr_ringbuf_pop(&rb, out, sizeof out), 10u);

    /* Wrap-around with a scatter push. */
    uint8_t   p1[]   = {1, 2, 3};
    uint8_t   p2[]   = {4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15};
    lbr_iov_t iov[2] = {{p1, sizeof p1}, {p2, sizeof p2}};
    CHECK(lbr_ringbuf_pushv(&rb, iov, 2));
    CHECK_EQ_U(lbr_ringbuf_pop(&rb, out, sizeof out), 10u);
    CHECK_EQ_U(lbr_ringbuf_peek_len(&rb), 15u);
    CHECK_EQ_U(lbr_ringbuf_pop(&rb, out, sizeof out), 15u);
    for (int i = 0; i < 15; i++) {
        CHECK_EQ_U(out[i], (unsigned)(i + 1));
    }
    CHECK(lbr_ringbuf_empty(&rb));
    CHECK_EQ_U(lbr_ringbuf_pop(&rb, out, sizeof out), 0u);
    CHECK(!lbr_ringbuf_push(&rb, a, 0));
}

int main(void)
{
    RUN(test_crc32_check_value);
    RUN(test_bytes_le);
    RUN(test_cobs);
    RUN(test_cobs_stream_resync);
    RUN(test_ringbuf);
    TEST_MAIN_END();
}
