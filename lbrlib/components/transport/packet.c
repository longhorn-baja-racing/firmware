#include "lbr/packet.h"

#include <string.h>

#include "lbr/bytes.h"
#include "lbr/crc32.h"

void lbr_pkt_begin(lbr_pkt_builder_t *b, uint8_t *buf, size_t cap, uint8_t link, uint8_t source,
                   uint32_t seq, uint64_t t_us)
{
    b->buf   = buf;
    b->cap   = cap;
    b->len   = LBR_PKT_HEADER_SIZE;
    b->count = 0;
    buf[0]   = LBR_PKT_VERSION;
    buf[1]   = link;
    buf[2]   = source;
    buf[3]   = 0;
    lbr_put_u32(buf + 4, seq);
    lbr_put_u64(buf + 8, t_us);
}

size_t lbr_pkt_room(const lbr_pkt_builder_t *b)
{
    size_t used = b->len + LBR_PKT_REC_HDR + LBR_PKT_CRC_SIZE;
    if (b->count == 0xFF || used >= b->cap) {
        return 0;
    }
    size_t room = b->cap - used;
    return room > 0xFF ? 0xFF : room;
}

bool lbr_pkt_add(lbr_pkt_builder_t *b, uint8_t type, const uint8_t *payload, size_t len)
{
    if (b->count == 0xFF || len > 0xFF ||
        b->len + LBR_PKT_REC_HDR + len + LBR_PKT_CRC_SIZE > b->cap) {
        return false;
    }
    b->buf[b->len]     = type;
    b->buf[b->len + 1] = (uint8_t)len;
    if (len > 0) {
        memcpy(b->buf + b->len + LBR_PKT_REC_HDR, payload, len);
    }
    b->len += LBR_PKT_REC_HDR + len;
    b->count++;
    return true;
}

size_t lbr_pkt_finish(lbr_pkt_builder_t *b)
{
    if (b->count == 0) {
        return 0;
    }
    b->buf[3] = b->count;
    lbr_put_u32(b->buf + b->len, lbr_crc32(b->buf, b->len));
    return b->len + LBR_PKT_CRC_SIZE;
}

void lbr_pkt_restamp(uint8_t *pkt, size_t len, uint8_t link, uint32_t seq)
{
    if (len < LBR_PKT_HEADER_SIZE + LBR_PKT_CRC_SIZE) {
        return;
    }
    pkt[1] = link;
    lbr_put_u32(pkt + 4, seq);
    lbr_put_u32(pkt + len - LBR_PKT_CRC_SIZE, lbr_crc32(pkt, len - LBR_PKT_CRC_SIZE));
}

bool lbr_pkt_parse(const uint8_t *buf, size_t len, lbr_pkt_header_t *hdr, lbr_pkt_record_cb cb,
                   void *ctx)
{
    if (len < LBR_PKT_HEADER_SIZE + LBR_PKT_CRC_SIZE || buf[0] != LBR_PKT_VERSION) {
        return false;
    }
    size_t body = len - LBR_PKT_CRC_SIZE;
    if (lbr_get_u32(buf + body) != lbr_crc32(buf, body)) {
        return false;
    }
    /* Check the record framing fully before calling back. */
    size_t  pos   = LBR_PKT_HEADER_SIZE;
    uint8_t count = 0;
    while (pos < body) {
        if (pos + LBR_PKT_REC_HDR > body || pos + LBR_PKT_REC_HDR + buf[pos + 1] > body) {
            return false;
        }
        pos += LBR_PKT_REC_HDR + buf[pos + 1];
        count++;
    }
    if (count != buf[3]) {
        return false;
    }
    hdr->version = buf[0];
    hdr->link    = buf[1];
    hdr->source  = buf[2];
    hdr->count   = buf[3];
    hdr->seq     = lbr_get_u32(buf + 4);
    hdr->t_us    = lbr_get_u64(buf + 8);
    for (pos = LBR_PKT_HEADER_SIZE; pos < body; pos += LBR_PKT_REC_HDR + buf[pos + 1]) {
        if (cb != NULL) {
            cb(buf[pos], buf + pos + LBR_PKT_REC_HDR, buf[pos + 1], ctx);
        }
    }
    return true;
}
