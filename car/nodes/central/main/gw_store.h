#pragma once

#include <stddef.h>

#include "lbr/gw_config.h"

/*
 * Runtime config persistence (NVS). The stored text is validated on load;
 * anything invalid falls back to defaults and is reported, never applied.
 */

/* Load the stored config. Returns 0 if a valid stored config was used, 1 if
 * none was stored (defaults), <0 if the stored one was invalid (defaults). */
int gw_store_load(lbr_gw_config_t *out);

/* Validate and store a pushed config (takes effect next session). */
int gw_store_put(const char *text, size_t len, char *err, size_t err_len);

/* Text of the config in effect for this session. */
size_t gw_store_get(char *buf, size_t cap);
