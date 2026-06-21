# LEGO Boost Move Hub ⟵ Xbox (direct, custom firmware)

Experimental track: pair an **Xbox Series controller** *directly* with a **LEGO
Boost Move Hub** (set 17101) by writing **custom hub firmware** — no host PC and
no second microcontroller in the middle.

> **Status: research / feasibility.** No firmware is built yet. Start with the
> options write-up and feasibility spike in
> [`docs/movehub-custom-firmware-options.md`](docs/movehub-custom-firmware-options.md).

## Why this is a separate track

The working, supported version is the browser bridge in the sibling
[`lego-boost-xbox`](../lego-boost-xbox/) package (Xbox → browser → Web Bluetooth
→ hub). That middleman **stays** — it is the reliable fallback. This package is a
clean-room, separate effort to remove the middleman entirely, kept apart so the
two never entangle.

## The short version

- The Move Hub is an **STM32F070RB** (Cortex-M0, 128 KB flash, 16 KB RAM) plus a
  separate **ST BlueNRG-MS** BLE coprocessor.
- Pybricks doesn't offer Xbox-controller support on the Move Hub — but the
  blocker is **flash space** (MicroPython leaves only ~106 KiB, already >95%
  full), **not** the radio: the BlueNRG-MS supports the BLE **central** role and
  Pybricks' own Move Hub driver already calls the central ACI functions.
- So a **dedicated** firmware (no MicroPython) should be able to scan for, pair
  with, and read an Xbox controller directly, then drive the motors.

See [`docs/movehub-custom-firmware-options.md`](docs/movehub-custom-firmware-options.md)
for the full hardware findings, the platform/language/framework options
(extend Pybricks · bare-metal STM32 + BlueNRG-MS ACI · Zephyr · libopencm3), the
recommendation, and the step-by-step feasibility spike.

## Related

- [`lego-boost-xbox`](../lego-boost-xbox/) — the working browser-bridge middleman
  (keep using this today)
- [`xbox-switch-bridge`](../xbox-switch-bridge/) — Xbox BLE host (Bluepad32)
  reference for the HID report layout
