#pragma once

/*
 * Firmware version shared by every node build. Bump on release; the packed
 * form travels in heartbeats and session metadata. The git description of the
 * exact build is recorded separately (esp_app_desc) in the session config.
 */
#define LBR_FW_VERSION_MAJOR 0u
#define LBR_FW_VERSION_MINOR 2u
#define LBR_FW_VERSION_PATCH 0u

#define LBR_FW_VERSION                                                                             \
    ((LBR_FW_VERSION_MAJOR << 16) | (LBR_FW_VERSION_MINOR << 8) | LBR_FW_VERSION_PATCH)
