#include "lbr/cobs.h"

size_t lbr_cobs_encode(const uint8_t *in, size_t len, uint8_t *out, size_t cap)
{
    if (cap < LBR_COBS_MAX_ENCODED(len)) {
        return 0;
    }
    size_t  code_pos = 0;
    size_t  o        = 1;
    uint8_t code     = 1;
    for (size_t i = 0; i < len; i++) {
        if (in[i] == 0) {
            out[code_pos] = code;
            code_pos      = o++;
            code          = 1;
        } else {
            out[o++] = in[i];
            if (++code == 0xFF) {
                out[code_pos] = code;
                code_pos      = o++;
                code          = 1;
            }
        }
    }
    out[code_pos] = code;
    return o;
}

size_t lbr_cobs_decode(const uint8_t *in, size_t len, uint8_t *out, size_t cap)
{
    size_t i = 0;
    size_t o = 0;
    while (i < len) {
        uint8_t code = in[i++];
        if (code == 0) {
            return 0;
        }
        for (uint8_t k = 1; k < code; k++) {
            if (i >= len || in[i] == 0 || o >= cap) {
                return 0;
            }
            out[o++] = in[i++];
        }
        if (code != 0xFF && i < len) {
            if (o >= cap) {
                return 0;
            }
            out[o++] = 0;
        }
    }
    return o;
}

void lbr_cobs_rx_init(lbr_cobs_rx_t *rx, uint8_t *buf, size_t cap)
{
    rx->buf        = buf;
    rx->cap        = cap;
    rx->len        = 0;
    rx->overflow   = false;
    rx->frames_ok  = 0;
    rx->frames_bad = 0;
}

size_t lbr_cobs_rx_push(lbr_cobs_rx_t *rx, uint8_t byte, uint8_t *out, size_t out_cap)
{
    if (byte != 0) {
        if (rx->len < rx->cap) {
            rx->buf[rx->len++] = byte;
        } else {
            rx->overflow = true;
        }
        return 0;
    }

    size_t n = 0;
    if (rx->len > 0) {
        n = rx->overflow ? 0 : lbr_cobs_decode(rx->buf, rx->len, out, out_cap);
        if (n > 0) {
            rx->frames_ok++;
        } else {
            rx->frames_bad++;
        }
    }
    rx->len      = 0;
    rx->overflow = false;
    return n;
}
