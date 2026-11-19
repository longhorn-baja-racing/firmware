#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/*
 * Transport packet: the one framing every live link uses (LoRa, USB, WiFi).
 * A packet batches records of the same types the log uses (protocol.h
 * lbr_record_type_t), so a consumer decodes live data and logs identically.
 *
 *   off size field
 *   0   1    version (LBR_PKT_VERSION)
 *   1   1    link (lbr_link_kind_t) that carried it
 *   2   1    source node id (the gateway)
 *   3   1    record count
 *   4   4    packet seq, per link, modulo 2^32: drops/duplicates/reordering
 *   8   8    t_us, shared time base when the packet was built
 *   16  ..   records: type u8, len u8, payload[len]
 *   end 4    crc32 over everything before it
 *
 * On byte streams (USB serial, WiFi TCP) each packet is COBS-encoded and
 * terminated by 0x00, so a receiver resynchronizes at the next delimiter.
 * LoRa packets are sent as-is (one radio packet = one LBR packet).
 */

#define LBR_PKT_VERSION     1u
#define LBR_PKT_HEADER_SIZE 16u
#define LBR_PKT_CRC_SIZE    4u
#define LBR_PKT_REC_HDR     2u
#define LBR_PKT_MAX_SIZE    1024u
#define LBR_PKT_LORA_MAX    255u

typedef struct {
    uint8_t *buf;
    size_t   cap;
    size_t   len;
    uint8_t  count;
} lbr_pkt_builder_t;

/* Start a packet in `buf` (cap includes room for the CRC). */
void lbr_pkt_begin(lbr_pkt_builder_t *b, uint8_t *buf, size_t cap, uint8_t link, uint8_t source,
                   uint32_t seq, uint64_t t_us);

/* Append a record. False (packet unchanged) if it does not fit. */
bool lbr_pkt_add(lbr_pkt_builder_t *b, uint8_t type, const uint8_t *payload, size_t len);

/* Room left for one more record payload (0 if none). */
size_t lbr_pkt_room(const lbr_pkt_builder_t *b);

/* Seal with the CRC. Returns the packet length (0 if no records). */
size_t lbr_pkt_finish(lbr_pkt_builder_t *b);

/* Set the link and per-link seq of a finished packet and re-seal its CRC.
 * Links call this as a packet is handed to them, so packet seq numbers are
 * contiguous per link and a gap at the receiver means loss in transit. */
void lbr_pkt_restamp(uint8_t *pkt, size_t len, uint8_t link, uint32_t seq);

typedef struct {
    uint8_t  version;
    uint8_t  link;
    uint8_t  source;
    uint8_t  count;
    uint32_t seq;
    uint64_t t_us;
} lbr_pkt_header_t;

typedef void (*lbr_pkt_record_cb)(uint8_t type, const uint8_t *payload, size_t len, void *ctx);

/* Validate and walk a packet. False if the CRC, version, or record framing
 * is wrong (then no callback is made). */
bool lbr_pkt_parse(const uint8_t *buf, size_t len, lbr_pkt_header_t *hdr, lbr_pkt_record_cb cb,
                   void *ctx);
