#pragma once

#include "lbr.h"

/* The cockpit's display driver (lbr/cockpit_app.h). The panel and its GPIO map
 * are still TBD, so this one reports the display state to the console; a real
 * panel driver (see display_graphics.c for the LVGL layout) replaces it. */
extern const lbr_display cockpit_console;
