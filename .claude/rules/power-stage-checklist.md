---
paths:
  - 'packages/**/hardware.toml'
  - 'packages/**/hardware/**'
  - 'packages/**/WIRING.md'
  - 'docs/reference/parts/**'
---
# A Battery-Powered Power Stage: Check These Against the Datasheet

The integrated robocar carrier's buck converter (PR #807) passed ERC, DRC,
parity and the board check, and an adversarial review still found five power
problems, none of them visible to a gate. Every one came from a number that is
in the datasheet and was not read. Work through this list for any regulator
fed from a battery, and cite the table or section for each answer: do not
recall a datasheet value (`analog-fault-isolation.md`, "Read the datasheet; do
not recall it").

| # | Check | How | What the integrated carrier found |
|---|---|---|---|
| 1 | **Dropout over the pack's whole discharge** | Minimum VIN = VOUT / maximum duty cycle (or VOUT plus dropout, for a linear or non-synchronous part); compare it with the pack's voltage at the end of discharge | The TPS563201's 75 % maximum duty cycle needs 6.7 V for 5 V: 3.3 V per cell, above a 2S pack's end of discharge. The docs said 6.0–8.4 V |
| 2 | **Low-battery cutoff** | The EN or UVLO threshold against the cell's discharge limit. Size the divider against EN's internal resistance and its high and low thresholds; note when the falling threshold is unspecified | EN was tied to VBAT, so the converter would drain the pack to about 1.7 V per cell. A 100k/30k divider now turns it on near 6.2 V and off near 4.9 V. Check the guaranteed turn-on too: VENH max 1.6 V across R26 ∥ REN min 600 kΩ (SLVSD90B §5.5) puts 100k/30k guaranteed on only by 7.2 V; 100k/36k is guaranteed on by 6.3 V |
| 3 | **Input protection** | Fuse or PTC rating against the pack's fault current, and every place VBAT is routed (test headers included) | No fuse, with VBAT on a 0.8 mm track to a test header. Fix: a 3 A PTC at the connector |
| 4 | **Sink capability** | Does the regulator sink current? Pulse-skip and DCM parts do not. With motors or an H-bridge on its rail, braking pushes energy back through the body diodes into the output capacitance | The TPS563201 is pulse-skip; the rail held the amplifier (6 V absolute maximum) and the MCU. Fix: the forced-continuous TPS563208, same pins |
| 5 | **Current budget against the minimum current limit** | Sum the stall currents (motors, servos) and the peaks (amplifier, radio) against the converter's **minimum** current limit, not its typical rating. Know what happens past it (hiccup restarts brown out the MCU) | Two motor stalls alone (about 1.7 A each, a simulation assumption) passed the 3.3 A minimum. The real stall current is still unmeasured |
| 6 | **Output capacitance against the loop's table** | A constant-on-time or D-CAP-style loop specifies a capacitance range: count it at DC bias, since an 0805 ceramic keeps about half its value at 5 V. Count any bulk capacitor whose ESR is low enough to join the loop | Two 22 µF 0805s can fall under TI's 20 µF minimum, so a third was added. Bulk polymer was rejected: TI caps low-ESR capacitance at 68 µF |
| 7 | **Inductor tolerance and saturation** | The value's tolerance against the table's minimum, Isat against the peak current at the **maximum** current limit plus half the ripple, and Irms | A 3.3 µH ±30 % part can fall below the 3.3 µH minimum. Fix: 4.7 µH ±20 %, with Isat 6.8 A against a 5.5 A peak |
| 8 | **The datasheet's layout rules** | Read the layout section: the input loop, vias at the input and output capacitors, the feedback trace away from the switch node, ground width | The buck's ground reached the battery through 23 mm of 0.19–0.25 mm track and one via (the `carrier-pcb` skill, trap 29) |
| 9 | **Output setpoint at the feedback tolerance's low end** | Compute VOUT at VFB minimum, subtract the trunk's drop at load, and compare the result with every downstream part's operating-range rows | TPS563208 VFB is 749/768/787 mV in continuous mode (SLVSD90B §5.5). R12 = 56 kΩ gives 5.07 V typical but 4.94 V at VFB minimum, which puts the TB6612FNG in its 0.4 A row (5 V > VM ≥ 4.5 V) instead of 1.0 A (VM ≥ 5 V; Toshiba datasheet, Operating Range). The trunk's ~50 mV at 2 A adds to it |
| 10 | **Input capacitor order** | Which capacitor the datasheet treats as the decoupling capacitor, and whether that one sits nearest VIN | TI asks for a ceramic over 10 µF as the decoupling capacitor and calls the 0.1 µF optional (SLVSD90B §7.2.2.4). The 100 nF sat nearest VIN and the 10 µF farther out; swapped |
| 11 | **Input path vias and necks against input current** | Input current at minimum VIN and full load; every via and the narrowest track on the path from the connector to the input capacitors, against the net class | The buck input (~2.5 A at end of charge) reached its capacitors through one 0.6/0.3 mm via although its class named 1.0/0.5 mm, and the track necked to 0.6 mm over ~14 mm: about 26 °C rise at 2.5 A by the IPC-2221 estimate |

Record each answer where the part is chosen (the `hardware.toml` comment above
it, and the project README's design notes), including what is still unverified
and needs a bench measurement.

## Related

- `analog-fault-isolation.md`: measure the effect, read the datasheet, confirm
  the instrument can see it
- `.claude/agents/pcb-reviewer.md`: the adversarial review that found these
- The `carrier-pcb` skill: `[fanout]` and `required` vias for the layout half
