# PRP: Carrier Power Design: Switch, Cutoff, Monitoring and Charging

**Status**: planned
**Source**: [ADR-038](../decisions/ADR-038-carrier-power-architecture.md), tracking issue [#1274](https://github.com/laurigates/mcu-tinkering-lab/issues/1274)
**Priority**: P2
**Confidence**: 6/10 (steps 1–3 use parts and blocks already in the repo; step 4's IC and step 5's pack are not chosen)

---

## Goal

Carry out ADR-038 on the carriers. The pack's BMS stays the backup. Firmware
reads the pack and ends a discharge first, the robocar carriers get an off
switch that draws nothing, and the robocar charges from USB-C. ADR-038 holds
the decision and its reasons. This file holds the steps, the files each one
touches, and the gate that says it is done.

## Steps

Each step is one issue under #1274, in this order.

### 1. Balancebot reads its pack and stops the motors (#747)

- **Files:** `packages/robotics/balancebot/src/pin_config.h` (`PIN_PACK_ADC 26`),
  `src/cli.c` (`stat`), a pure-C threshold module with a host test (the
  project keeps decision logic SDK-free, `packages/robotics/balancebot/CLAUDE.md`),
  `hardware/carrier/scripts/check_pins.py` (`ROLE_NETS`, net `PACK_SENSE`).
- **Behaviour:** warn below 10.5 V; below about 9 V for a minimum duration,
  FAULT with the motors off, as the tilt cutoff does. Handle both divider
  ratios: 100k/27k on the delivered rev1 boards, 100k/22k on rev3. Report
  "absent" on a hand-wired build, where D0 floats.
- **Done when:** `just balancebot::test` covers the thresholds, hysteresis and
  minimum duration; `just balancebot::carrier-check` passes; on the robot a
  bench supply lowered through 9 V faults it, and the run log saves that run
  (ADR-037).

### 2. Robocar carriers: switch, latch, divider (#1270)

- **Blocks:** `docs/reference/blocks/pack-input-pfet-switched.toml` in place of
  the input stage, `soft-latch.toml` on its `SW` port, optionally
  `button-input.toml` for a chassis button. Both are first uses, so ERC, the
  netlist and the board check exercise them for the first time.
- **Files:** `packages/robotics/robocar/hardware.toml` (socketed) and/or
  `packages/robotics/robocar/integrated/hardware.toml`, each carrier's
  `hardware/carrier/board.toml`, `WIRING.md`, the carrier README.
- **Pins:** `HOLD` on a native GPIO driven early in boot; `SENSE` on any input
  (the integrated carrier's MCP23017 is a candidate); the divider on an ADC1
  pin. The socketed `hardware.toml` lists D3 (GPIO4, ADC1) free and D7
  (GPIO44, no ADC) spare.
- **Done when:** `just robocar::carrier-check` (and
  `just robocar-integrated::carrier-check` if that carrier is changed)
  passes with its stamp committed, `just kicad::check` passes, and a
  `pcb-reviewer` pre-fab review has no open blocker.

### 3. Robocar firmware: latch and shutdown (#1271)

- **Files:** `packages/robotics/robocar/main/` (a pure module for the pack
  thresholds with a host test under `test/`, then the ADC and GPIO wrapper),
  `pin_config.h`.
- **Behaviour:** drive `HOLD` high early in boot; a long press on `SENSE`
  switches off; below the threshold (above the BMS's cell cutoff, with
  hysteresis and a minimum duration) park the servos, stop the motors, speak,
  then release `HOLD`. Publish the pack voltage on the OLED and over MQTT.
- **Done when:** `just robocar::test` covers the threshold logic; the button
  boots and switches off the robot; a bench supply lowered through the
  threshold shuts it down once, and a short dip does not. Measure and record
  how long the button must be held at boot.

### 4. 2S USB-C charger block (#1272)

- **Choose:** BQ25887 or MP2672A (or another), on input current, charge and
  balance current, thermal regulation, power path, external parts and
  package; then LCSC stock and Basic/Extended for the IC and inductor
  (`.claude/rules/part-sourcing.md`, `aliexpress-parts` skill).
- **Files:** `docs/reference/blocks/<ic>-2s-usb-charger.toml` with the
  datasheet reasoning in comments, new parts in `docs/reference/parts/`, then
  the robocar carrier's `hardware.toml`.
- **Constraints:** the charger's USB-C port must not reach the XIAO's 5V pin,
  which is the XIAO's VBUS (#1252). The pack's BMS stays, so decide how the
  charger's balancing and the BMS's coexist.
- **Done when:** `just hardware::test` and `tools/kicad/test_blocks_registry.py`
  pass; on its first carrier, the gates of step 2.

### 5. Balancebot switch, latch and charging (#1273)

- **Starts with** the 3S/4S decision from #1262's push tests, recorded in
  PRD-012.
- **Then:** a 3S/4S variant of the switched input and latch with a gate clamp
  like the present D2, ratings for 16.8 V plus hot-plug ringing, firmware
  thresholds for the cell count, and the charging decision (external, or
  USB-PD input with a buck/boost charger).

## Out of scope

- A BMS on any carrier (rejected in ADR-038).
- Fuel gauging beyond a voltage reading.
- The shared carrier standard's catalogue and checks (#970). This plan uses
  its blocks but does not write the standard.
