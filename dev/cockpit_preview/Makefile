CC ?= cc
LVGL_DIR ?= ../../build-cockpit-preview/lvgl
OUTPUT ?= ../../build-cockpit-preview/cockpit_preview

LVGL_SOURCES := $(shell find "$(LVGL_DIR)/src" -name '*.c')
PREVIEW_SOURCES := main.c ../../car/nodes/cockpit/main/display_graphics.c
CPPFLAGS += -DLV_CONF_INCLUDE_SIMPLE=1 -I. -I../../car/nodes/cockpit/main -I$(LVGL_DIR)
CFLAGS += -std=c99 -O2 -Wall -Wextra
LDLIBS += -lSDL2 -lm

.PHONY: all clean

all: $(OUTPUT)

$(OUTPUT): $(PREVIEW_SOURCES) $(LVGL_SOURCES) lv_conf.h
	mkdir -p $(dir $@)
	$(CC) $(CPPFLAGS) $(CFLAGS) $(PREVIEW_SOURCES) $(LVGL_SOURCES) $(LDFLAGS) $(LDLIBS) -o $@

clean:
	rm -f $(OUTPUT)
