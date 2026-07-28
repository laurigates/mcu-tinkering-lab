# Read the Sensor PID Before Reasoning About Any Camera Register

The `esp32-camera` component binds a **different driver per sensor**, and those
drivers do not share a register map, an address width, or even the meaning of
the `sensor_t` function arguments. A comment in a pin header saying which sensor
is fitted is **not evidence**. `esp_camera_init()` logs the detected PID at
boot; that is the only thing that settles it.

Cost of skipping this (robocar-unified, 2026-07): `camera_pins.h` claimed
"OV2640" from the day it was written, the board is actually an **OV3660**, and
an entire multi-agent investigation into dark frames reasoned from OV2640
register semantics — arriving at a plausible, well-evidenced, wrong conclusion.
The real defect was visible only once the PID was printed.

## The trap: identically-named setters, incompatible arguments

`set_gainceiling()` is the worst offender, because both drivers accept the same
`gainceiling_t` and neither validates:

| | OV2640 | OV3660 / OV5640 |
|---|---|---|
| Argument | **enum index**: `0` = 2x … `6` = 128x | **raw 10-bit ceiling**, gain x16 |
| Written to | `COM9[7:5]` | `0x3A18` / `0x3A19` |
| Power-on default | driver forces `GAINCEILING_2X` in `esp_camera_init()` | `0x00F8` = 248 = **15.5x** |
| So `set_gainceiling(s, 0)` means | "2x ceiling" — low but functional | **"zero gain ceiling"** — AGC cannot brighten anything |

That one call, copied from any of the thousands of OV2640 examples online, is
what made this robot's frames dark. It is not a range error the driver can
reject — `0` is valid in both worlds and means something reasonable in one.

Register **addressing** differs too, so a read is just as unsafe:

- **OV2640**: 8-bit addresses, bank-switched. `get_reg()` encodes the bank in
  bit 8, so sensor-bank registers are addressed `0x1xx`.
- **OV3660/OV5640**: 16-bit addresses (`0x3500`, `0x350A`, `0x3A18`), read via
  `SCCB_Read16`. A `mask <= 0xFF` selects the single-byte path.

Reading one map through the other driver returns **plausible numbers, not
errors** — the observed result was `gain=0 exp=0`, which is itself a specific
and actionable diagnosis ("auto-exposure is not converging"). A silent
mis-read is worse than a failed read.

## The rule

1. **Confirm the PID before touching a sensor register.** From a boot log
   (`Camera PID=0x____`, and `esp_camera_sensor_get()->id.PID` at runtime), not
   from a header comment, a schematic PDF, or a product page.
2. **Branch on the PID** in any code that reads or composes sensor registers,
   and return failure — not zeros — for a sensor whose map you do not know.
3. **Never render an unread sensor as `gain=0 exp=0`.** Emit "UNREADABLE", or
   omit the fields entirely, so a downstream reader cannot mistake a failed I2C
   transaction for a measurement.
4. **Treat `set_gainceiling()` / `set_agc_gain()` / `set_aec_value()` argument
   ranges as sensor-dependent.** Expose the valid range from a helper that
   consults the PID rather than hardcoding `0..6`.

Known PIDs (`managed_components/espressif__esp32-camera/driver/include/sensor.h`):
`OV2640_PID = 0x2640`, `OV3660_PID = 0x3660`, `OV5640_PID = 0x5640`,
`OV7670_PID = 0x7670`, `OV7725_PID = 0x7725`, `NT99141_PID`, `GC032A_PID`, …

## Where this bites

- **XIAO ESP32-S3 Sense** ships variants; do not assume the OV2640 that most
  blog posts describe. This one is an OV3660.
- Any project under `packages/` using `esp32-camera` — `robocar/unified`,
  `robocar/camera`, `camera-vision/*`. The pin header of one is routinely
  copied to the next, comment included.
- Copy-pasted "tuning blocks" from ESP32-CAM examples. They are OV2640-shaped
  by default and silently mean something else on a newer sensor.

## Verify

```sh
just <project>::monitor | grep -i "camera PID\|Detected"
```

Then, before trusting any exposure reasoning, check that the numbers move when
the scene does. A gain that never changes is a mis-read, not a stuck AGC.

## Related

- `esp-idf-sdkconfig.md` — `CONFIG_CAMERA_JPEG_MODE_FRAME_SIZE`, the other
  camera setting whose default silently breaks capture at higher gain
- `~/.claude/rules/diagnose-at-the-failure-point.md` — the general form: the
  entity named in an error (or a header comment) is a claim to verify, not a
  fact to reason from
