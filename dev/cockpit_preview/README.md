# Cockpit dashboard desktop preview

This preview runs the cockpit's existing LVGL dashboard in an SDL desktop
window. It compiles `car/nodes/cockpit/main/display_graphics.c` directly, so it
does not change the cockpit firmware display path. Synthetic RPM, speed, CVT
and brake temperatures, fuel level, and all stress readings vary over time to
exercise the gauges, color ranges, and low-fuel warning.

## Build and run

Requirements: Make, a C compiler, SDL2 development files, and network access the
first time LVGL is fetched.

From the repository root:

```sh
git clone --depth 1 --branch v9.2.2 https://github.com/lvgl/lvgl.git build-cockpit-preview/lvgl
make -C dev/cockpit_preview
./build-cockpit-preview/cockpit_preview
```

Resize the window to check the dashboard's centered 4:3 viewport. Close the
window to exit. The LVGL source and preview binary are kept in the ignored build
directory; no LVGL source is vendored into the firmware tree.
