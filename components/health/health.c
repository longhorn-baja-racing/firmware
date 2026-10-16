#include "lbr/health.h"

/*
 * Health records are plain data; builders/accumulators live here as the node
 * firmware grows (e.g. snapshotting queue high-water marks and CAN error
 * counters into an lbr_health_record_t each heartbeat tick).
 */
