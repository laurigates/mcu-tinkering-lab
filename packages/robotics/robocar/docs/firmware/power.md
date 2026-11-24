# Power switch and pack cutoff

Firmware for the socketed carrier's soft latch and pack divider (ADR-038,
[#1271](https://github.com/laurigates/mcu-tinkering-lab/issues/1271)). The hardware is
in [the carrier README](../../hardware/carrier/README.md#the-power-switch).
`FEATURE_POWER_LATCH_ENABLED` in `main/config.h` is **0 by default**: a
hand-wired robot leaves D7 open and D3 floating, and a floating pad reads as
noise that could shut it down. Set it to 1 for a robot built on the carrier.

| File | Does |
|---|---|
| `power_core.c` | Pure, host-tested (`test/test_power_core.c`): divider conversion, the low-pack monitor, the long-press detector |
| `power_latch.c` | HOLD (`POWER_HOLD_PIN`, GPIO44): a constructor drives it high before `app_main`, `power_latch_init()` repeats it on `app_main`'s first line |
| `pack_sense.c` | ADC1 oneshot read of `PACK_SENSE_PIN` (GPIO4, ADC1_CH3), 16 samples averaged, 6 dB, curve-fitting calibration |
| `power_manager.c` | The `power` task (Core 1, 100 ms): pack monitor, button on the MCP23017 (`POWER_BUTTON_EXPANDER_PIN`, internal pull-up, low = pressed), the shutdown |

## What it does

- **Boot.** HOLD goes high as early as software can, so the button can be
  released. Whether the constructor ran before `app_main` is in the boot log
  (`Power latch: HOLD high before app_main`). How long the button must be held
  is set by the ROM and second-stage bootloader, which no firmware controls, and
  is not measured.
- **Pack.** The filtered voltage is on the OLED (`HW OK 7.4V`, dropped when
  faults fill the row), on `robocar/power` every 10 s
  (`{"pack_mv":7421,"state":"ok"}`) and as a `pack_mv` extra on every
  GlitchTip event.
- **Cutoff.** `POWER_PACK_LOW_MV` 6600 (3.3 V per cell), cleared above
  `POWER_PACK_CLEAR_MV` 7000, after `POWER_PACK_MIN_LOW_MS` 5000 below it. The
  pack voltage is filtered first, so the milliseconds-long sag at a motor start
  (#991) does not count. A reading under `POWER_PACK_ABSENT_MV` 4000 is no pack
  (USB power only) and never trips. The 3.3 V per cell is the example from the
  issue: the BMS's cutoff is not recorded in this repo and no discharge log has
  set the value.
- **Button.** Held for `POWER_BUTTON_LONG_PRESS_MS` 2000 switches off. The press
  that switched the robot on is ignored until its first release.
- **Shutdown** (`shutdown_robot()`, once): wheels held coasting
  (`reactive_controller_set_hold()`), head centred by a head lease, a line on
  `robocar/power`, the spoken line (waits for playback, at most
  `POWER_SPEAK_MAX_MS`), then HOLD low. After `POWER_OFF_GRACE_MS` a robot that
  is still running (USB power, or the button still held, which keeps the carrier
  on) goes to deep sleep. A press on RESET or a power cycle boots it again.

## Don't

- Do not log from `power_latch_init()`: it runs ahead of
  `event_report_boot_capture()`.
- Do not write motors or the head from `power_manager.c` except through
  `reactive_controller_set_hold()` and `reactive_controller_head_hold()`.
- Do not feed the monitor from ADC2: it is unusable while Wi-Fi runs.
