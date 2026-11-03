#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/*
 * The cockpit display's wireless link (DAQ-WIRING.md section 8), pure
 * (host-tested). It is the only node off the CAN bus:
 *
 *   Central -> cockpit   display values, node status   (LBR_LINK_COCKPIT)
 *   cockpit -> Central   cockpit battery
 *
 * Every datagram is one transport packet (lbr/packet.h: header, records,
 * CRC-32) over UDP, no TCP, so a lost datagram never stalls the next. The
 * packet seq is per sender, so the receiver counts lost datagrams. The
 * Central side is lbr/cockpit_link.h, the cockpit side lbr/cockpit_core.h.
 */

#define LBR_WLINK_COCKPIT_PORT 47801u /* cockpit listens */
#define LBR_WLINK_STATUS_PORT  47802u /* Central listens for the cockpit */

#define LBR_WLINK_LINK_TIMEOUT_US    500000u  /* the Central sends 10 datagrams/s */
#define LBR_WLINK_COCKPIT_TIMEOUT_US 3000000u /* the cockpit reports once a second */

/* ------------------------------------------------------------------------ */
/* Datagram sequence                                                         */

/* Which of the last 32 sequence numbers arrived. WiFi can reorder: a late
 * datagram is not a gap, so a number counts as lost only when it leaves the
 * window unseen. */
typedef struct {
    bool     have;
    uint32_t last; /* newest seq seen */
    uint32_t bits; /* bit i: seq last - i arrived */
} lbr_seq_window_t;

typedef enum {
    LBR_SEQ_NEW = 0,   /* newer than anything so far */
    LBR_SEQ_LATE,      /* older, not seen before (reordered) */
    LBR_SEQ_DUPLICATE, /* seen before */
    LBR_SEQ_OLD,       /* too old to tell (or the sender restarted) */
} lbr_seq_result_t;

/* Account `seq`; adds datagrams known lost to *gaps. */
lbr_seq_result_t lbr_seq_window_take(lbr_seq_window_t *w, uint32_t seq, uint32_t *gaps);

/* The sender went quiet or restarted its count: forget the window, adding
 * the numbers it still held unseen (sent before the newest one, never
 * arrived) to *gaps. The next lbr_seq_window_take starts afresh. */
void lbr_seq_window_restart(lbr_seq_window_t *w, uint32_t *gaps);
