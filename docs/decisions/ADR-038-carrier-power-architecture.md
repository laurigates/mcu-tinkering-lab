# ADR-038: Carrier Power Design: the BMS Stays on the Pack, the Switch, Cutoff and Charging Move Onto the Carrier

**Status**: accepted
**Date**: 2026-11-24
**Confidence**: 6/10 (the split rests on where each protection has to sit to
work; the charger ICs and the 3S/4S balancebot pack are not chosen, and the
switched-input and soft-latch blocks have never been on a board)
**Issue**: [#1274](https://github.com/laurigates/mcu-tinkering-lab/issues/1274)
**Extends**: [ADR-031](ADR-031-circuit-blocks.md)

---

## Context

The robots run from 18650 packs: robocar from 2S (7.4 V nominal, 8.4 V full,
XT30 on J3), balancebot from 3S (9.0–12.6 V, JST-XH on J1;
[PRD-012](../requirements/PRD-012-balancebot.md), #739, #744). Each pack carries
a protection board (BMS) wired to its cells and is charged off the robot. The
question was whether the BMS and charging should move onto the carriers.

What the carriers do today:

- **Input protection.** A PTC fuse on every carrier; a reverse-polarity
  P-FET on the integrated robocar carrier (`pack-input-pfet-ptc`) and on
  balancebot, which adds a gate zener for 3S
  (`packages/robotics/balancebot/hardware/carrier/README.md`).
- **No low-battery cutoff on the socketed robocar carrier**
  (`packages/robotics/robocar/hardware/carrier/README.md`, pre-fab review
  item 6). The LM2596 module grounds its ON/OFF pin on its own board and keeps
  drawing as the cells fall past their minimum. The integrated carrier's
  TPS563208 enable divider stops at about 4.0–4.6 V, too coarse to count. On
  both, the pack's BMS is the only thing that ends a discharge.
- **Monitoring.** Balancebot has a pack divider on D0 that firmware does not
  read yet (#747), so a flat pack shows only as the BMS cutting power and the
  robot falling. Robocar has no divider.
- **No power switch** on any carrier: the pack is unplugged to turn a robot off.
- **Designed, unused:** `pack-input-pfet-switched` (back-to-back P-FETs and
  the PTC; the switch carries about 76 µA and nothing when off) and
  `soft-latch` (one button on, firmware holds, firmware off), both for 2S
  (`docs/reference/blocks/`).

Two outside constraints:

- [PRD-015](../requirements/PRD-015-teuvo-for-other-owners.md) FR-036 point 5:
  portable batteries must be user-removable from 18.2.2027 (Battery Regulation
  Art 11), so the pack is a unit of its own.
- The `aliexpress-parts` skill (trap 15): many listings with "BMS" in the title
  are chargers or boost boards that protect only their own output. A BMS
  protects each cell, with per-cell over-charge, over-discharge and
  over-current thresholds.

## Decision

Battery handling is four jobs, and each goes where it can work:

| Job | Where | Why |
|---|---|---|
| **Cell protection and balancing (BMS)** | **On the pack**, a board wired to the cells | It protects the cells wherever the pack is |
| **Power switch and low-battery shutdown** | **On the carrier**: `pack-input-pfet-switched` driven by `soft-latch`, with firmware switching off | Firmware can end a discharge before the BMS has to, and off draws nothing |
| **Pack-voltage monitoring** | **On the carrier**: a divider into an ADC pin, read by firmware | Firmware needs the voltage to warn, stop and switch off |
| **Charging** | **Robocar (2S): on the carrier**, a 5 V USB-C charger that balances. **Balancebot: external** until its cell count is settled | 2S from 5 V is a common charger shape; 3S/4S from USB needs PD input and 15–25 W of heat |

### The BMS stays on the pack

- **It protects the cells off the robot.** On the pack it covers charging,
  storage, transport and a dropped pack shorting across its lead. On the
  carrier it would protect the cells only while plugged in, and a bare 3S
  pack with an exposed lead would have nothing between a short and the cells.
- **It keeps safety circuitry out of every carrier revision.** On the carrier
  the BMS needs every cell tap brought to the board and the full pack current
  through its FETs and sense resistor. Each layout change would then touch it.
- **The pack is a removable unit** (Battery Regulation Art 11, above), and the
  protection belongs with what is removed.

### The carrier switches and cuts off

Firmware becomes the first line of the low-battery cutoff; the BMS becomes the
backup.

- **Robocar** (#1270, #1271): `pack-input-pfet-switched` replaces the input
  stage, `soft-latch` drives it, and a divider feeds an ADC1 pin. Firmware
  holds the latch at boot, switches off on a long press, and below a threshold
  above the BMS's cell cutoff parks the servos, stops the motors and releases
  the latch. The threshold has hysteresis and a minimum duration, so a
  motor-start sag (#991) does not trip it.
- **Balancebot** (#747): firmware reads the existing divider and stops the
  motors with a FAULT near the DRV8825's 8.2 V minimum, about 9 V on 3S. A
  FAULT also makes the run log save the run (ADR-037), which a BMS cutoff does
  not. The switch and latch wait for the cell count (#1273), since the 2S
  blocks need a 3S/4S variant with a gate clamp like the present input stage.

### Charging per robot

- **Robocar** (#1272): a circuit block for a 5 V USB-C charger that charges 2S
  and balances. Candidates to evaluate, none checked yet: TI BQ25887 and MPS
  MP2672A. At about 10 W from a 5 V / 2 A port that is roughly 1 A into the
  pack (estimate). The charger's USB-C port is separate from the XIAO's, which
  stays the data port; the XIAO's 5V pin is its VBUS (#1252), so the charger
  must not feed it. The pack's BMS stays, so the block decides how its
  balancing and the BMS's coexist.
- **Balancebot** (#1273): an external charger until #1262's tests settle 3S
  or 4S. On-board charging there means USB-PD input (9–20 V), a buck/boost
  charger with an inductor and sense resistor, and 15–25 W of heat on a small
  carrier. Chargers that balance more than two cells are rare, so the pack's
  BMS would still balance. An IP2368-based design is one candidate.

### Order

1. #747: balancebot firmware reads the divider and stops the motors.
2. #1270: robocar carriers get the switch, latch and divider.
3. #1271: robocar firmware holds the latch and shuts down on a low pack.
4. #1272: the 2S USB-C charger block, then on a robocar carrier.
5. #1273: balancebot switch, latch and charging after the 3S/4S decision.

The steps, files and gates are in
[`docs/prompts/carrier-power-architecture.md`](../prompts/carrier-power-architecture.md).

## Consequences

**Positive**

- A flat pack ends with the robot switching itself off, or on balancebot
  stopping its motors, instead of the BMS cutting power mid-motion.
- An off switch that draws nothing, so a pack left in the robot is not drained
  by the regulator.
- The cells are protected on and off the robot by one board that never
  changes with the carrier.
- The robocar charges from one USB-C cable, which also suits the open-hardware
  build pack (PRD-015 FR-036).

**Negative**

- **Protection in two places.** The BMS's thresholds and the firmware's must be
  set against each other: firmware's cutoff above the BMS's, so the BMS
  stays the backup. A BMS bought later with different thresholds changes that
  margin.
- **More parts and a first use.** The switched input and the latch add two
  FETs, a diode pair, resistors and a button to each robocar carrier, and the
  blocks have only been checked for structure, never by ERC or on a board.
- **The latch needs firmware.** A hang with the latch held cannot be undone by
  the button; unplugging the pack is the override (`soft-latch.toml`). The
  button must be held through the ESP32-S3's boot, for a time not yet
  measured.
- **Pins.** The robocar carrier needs a native GPIO for `HOLD`, one input for
  `SENSE` and an ADC1 pin for the divider; the socketed carrier lists D3 free
  and D7 spare (#1270).
- **A charger on the robocar carrier adds heat and an inductor**, and its
  balancing has to be worked out against the BMS's.
- **Balancebot stays behind robocar** until its pack is decided.

## Alternatives Considered

- **BMS on the carrier**: rejected. It protects the cells only while plugged
  in, puts every cell tap and the full pack current through each carrier, and
  makes every carrier revision a change to safety circuitry.
- **Keep everything external** (the state before this ADR): rejected. It
  leaves the socketed robocar with no cutoff and no robot with an off switch,
  and leaves the BMS as the only end to a discharge.
- **Monitoring and firmware cutoff only, no switch**: rejected as the end
  state, though it is balancebot's first step (#747). Without a switch the
  regulator still draws from a pack left in a robot that has stopped.
- **On-board 3S/4S charging for balancebot now**: deferred. The cell count is
  open, and the input, ratings and heat all follow from it.

## Related

- [ADR-031](ADR-031-circuit-blocks.md): the circuit blocks the switch, latch
  and charger are written as
- [ADR-037](ADR-037-balancebot-untethered-run-log.md): a firmware FAULT saves
  the run log; a BMS cutoff does not
- [PRD-012](../requirements/PRD-012-balancebot.md),
  [PRD-015](../requirements/PRD-015-teuvo-for-other-owners.md)
- #970 (shared carrier design standard), #1252 (USB and pack order), #991
  (robocar brownout), #1262 (balancebot push tests)
