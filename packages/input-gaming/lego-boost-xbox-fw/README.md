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

## Feasibility spike (thin)

Tooling to kill-or-confirm the two unknowns — **does it fit flash** and **does
the BlueNRG driver have the pairing path** — using the *extend Pybricks* option
as the spike vehicle. This is **hardware-in-the-loop**: the recipes build/flash
firmware and run test scripts on *your* physical Move Hub + Xbox controller.

```sh
just boost-fw::setup     # clone pybricks-micropython into external/ (gitignored)
just boost-fw::build     # build vanilla movehub firmware in a container
just boost-fw::flash     # install on the hub via pybricksdev (hold the button)
just boost-fw::run motor_test     # Phase 2 baseline: spin the motors
just boost-fw::restore   # back to official Pybricks (code.pybricks.com)
```

| Path | What it is |
|---|---|
| `justfile` | spike recipes (setup, build, flash, run, enable-xbox, restore) |
| `docker/Dockerfile` | ARM toolchain + poetry build container |
| `spike/*.py` | `motor_test` (Phase 2), `remote_test` (Phase 1, needs a handset), `xbox_test` (Phase 3) |
| `patches/` | the Phase-3 "enable Xbox for movehub" patch (produced during the spike) |
| `docs/spike-log.md` | running record of results → the go/no-go decision |

Prereqs on the host: Docker, and `pybricksdev` (`pipx install pybricksdev`).
The three phases and their pass/fail gates are tracked in
[`docs/spike-log.md`](docs/spike-log.md).

## Related

- [`lego-boost-xbox`](../lego-boost-xbox/) — the working browser-bridge middleman
  (keep using this today)
- [`xbox-switch-bridge`](../xbox-switch-bridge/) — Xbox BLE host (Bluepad32)
  reference for the HID report layout
