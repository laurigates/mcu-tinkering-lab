# Feasibility spike log — direct Xbox → Move Hub pairing

Running record of the thin spike (see
[`movehub-custom-firmware-options.md`](movehub-custom-firmware-options.md) for the
options and [the plan](../README.md) for scope). Fill each phase's result in as
you run it on the hardware. The Phase 3 verdict drives the go/no-go decision.

## Goal recap

Confirm whether the Move Hub can pair with and read an Xbox controller directly,
via custom Pybricks firmware. Two unknowns to settle:

1. **Flash fit** — does enabling the Xbox feature link within the Move Hub's
   ~106 KiB free flash? (Build-time signal: a linker overflow = no.)
2. **Pairing path** — does the BlueNRG driver implement the LE pairing/bonding
   the Xbox HID service needs? (The `Remote` works without pairing.)

## Phase 1 — Central path confidence check (optional, zero build)

- Hardware needed: LEGO Powered Up handset. Skipped if unavailable.
- Command: `just boost-fw::run remote_test`
- **Result:** _TBD_
- Notes: _TBD_

## Phase 2 — Build + flash + recovery baseline

- Commands: `just boost-fw::setup` → `build` → `flash` → `run motor_test` → `restore`
- Build output / firmware size: _TBD_
- Flash + motor behavior: _TBD_
- Recovery to official Pybricks verified: _TBD_
- Notes (toolchain version, any docker/Dockerfile adjustments): _TBD_

## Phase 3 — Enable Xbox for movehub; learn fit + gap; STOP

- What was changed (capture as `patches/enable-xbox-movehub.patch`): _TBD_
- Build outcome — pick one:
  - [ ] **(a) Linker flash overflow** → Option A (extend Pybricks) is dead; pivot
        to Option B (bare-metal + BlueNRG-MS ACI).
  - [ ] **(b) Builds & fits** → firmware size / `.map` flash usage: _TBD_
- If (b): `just boost-fw::run xbox_test` — connect? pair/bond? read input?
  Exact stop point: _TBD_
- **Verdict & recommendation:** _TBD_
