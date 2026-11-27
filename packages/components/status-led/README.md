# status-led

The WS2812 status LED layer shared by the Waveshare ESP32-S3-Zero projects:
RMT init with DMA off (DMA breaks after TinyUSB init on the S3), blink timing
and a blocking flash. Each project keeps its own `status_led` component with
its mode enum and a table of colours indexed by it.

## Why it has no CMakeLists.txt

robocar and the thinkpack boxes compile every directory under
`packages/components`. A component here would pull `led_strip` into their
builds. Instead each consumer's `components/status_led/CMakeLists.txt` adds
`status_led_core.c` to its own `SRCS` and this `include/` to its
`PRIV_INCLUDE_DIRS`, and keeps the `espressif/led_strip` dependency in its own
`idf_component.yml`. Because no `EXTRA_COMPONENT_DIRS` entry names this
directory, `tools/check-component-extra-paths.py` cannot see the consumers:
list `components/status-led` in each one's `extra_paths` in
`.github/project-matrix.json` by hand.

## API

```c
#include "status_led_core.h"

static const status_led_pattern_t k_patterns[] = {
    [MY_LED_OFF] = {0, 0, 0, 0},
    [MY_LED_BUSY] = {0, 0, 32, 500 * 1000}, /* blue, 500 ms on / 500 ms off */
    [MY_LED_ERROR] = {32, 0, 0, 0},         /* red, solid */
};

status_led_core_init(21, k_patterns, sizeof(k_patterns) / sizeof(k_patterns[0]));
status_led_core_set_mode(MY_LED_BUSY);
status_led_core_update();                  /* from a loop at ~50 Hz or faster */
status_led_core_flash(0, 32, 32, 200);     /* blocks 200 ms; the mode resumes */
```

A mode at or past the table's length shows dark.

## Consumers

release-please has no package for this directory, so a commit here releases
none of the consumers. A behaviour change needs a `fix` or `feat` commit under
each consumer's path to ship (log-udp has the same gap, #1081).


- `packages/input-gaming/xbox-switch-bridge` (its host suite,
  `components/status_led/test/`, runs this file through the bridge's table)
- `packages/usb-tools/facedancer-espdancer-fw`
- `packages/networking/it-troubleshooter`
