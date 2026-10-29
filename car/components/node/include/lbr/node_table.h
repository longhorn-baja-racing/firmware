#pragma once

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "lbr/can.h"
#include "lbr/health.h"

/*
 * Live bus topology, built from received frames: which CAN nodes are
 * present, when each was last heard, its latest heartbeat (state and error
 * flags) and health record. Used by the Central (NODE_FOUND / NODE_LOST
 * events, telemetry health, the cockpit dashboard's node display). Pure;
 * host-tested.
 */

#define LBR_NODE_TABLE_SIZE 16u

typedef struct {
    uint8_t             node_id; /* 0 = free slot */
    bool                present;
    uint64_t            first_seen_us;
    uint64_t            last_seen_us;
    uint64_t            last_health_us;
    uint16_t            pages_seen; /* bitmask of health pages received */
    bool                hb_seen;
    uint8_t             hb_state; /* latest heartbeat: lbr_node_state_t */
    uint8_t             hb_flags; /* latest heartbeat: LBR_HB_ERR_* */
    uint32_t            frames;
    lbr_health_record_t health;
} lbr_node_entry_t;

typedef struct {
    lbr_node_entry_t nodes[LBR_NODE_TABLE_SIZE];
    uint64_t         timeout_us; /* silent this long => lost */
    uint32_t         table_full; /* frames from nodes that did not fit (counted) */
} lbr_node_table_t;

typedef enum {
    LBR_NT_NONE = 0,
    LBR_NT_FOUND, /* first frame from a node, or back after being lost */
} lbr_node_table_event_t;

void lbr_node_table_init(lbr_node_table_t *t, uint64_t timeout_us);

/* Account one received frame. */
lbr_node_table_event_t lbr_node_table_on_frame(lbr_node_table_t *t, const lbr_can_frame_t *f,
                                               uint64_t rx_us);

/* Mark nodes silent for longer than the timeout as lost. Writes up to `cap`
 * newly lost node ids to `lost` and returns how many. */
size_t lbr_node_table_expire(lbr_node_table_t *t, uint64_t now_us, uint8_t *lost, size_t cap);

const lbr_node_entry_t *lbr_node_table_find(const lbr_node_table_t *t, uint8_t node_id);
