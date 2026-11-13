# TB6612FNG — Dual DC Motor Driver

**Manufacturer:** Toshiba
**Used in:** Robocar main controller (left/right DC motor H-bridge control)

## Key Specs

Values and page numbers are from Toshiba's current revision (footer dated
2026-05-13), which the Toshiba link below served on 2026-11-13 although its
file name says 20141001. The SparkFun mirror is the 2007-06-30 revision; it has
the same pages and the same values except the operating range of VM and the
current breakpoint that goes with it (see below the table).

| Parameter | Value | Source |
|-----------|-------|--------|
| Channels | 2 (dual H-bridge) | p. 1, p. 2 |
| Motor supply VM, operating | 2.5–13.5 V (typ. 5 V) | Operating Range, p. 3 |
| Motor supply VM, absolute maximum | 15 V | Absolute Maximum Ratings, p. 3 |
| Output current, operating | 1.0 A max per channel at VM ≥ 4.5 V; 0.4 A max at 4.5 V > VM ≥ 2.5 V, without PWM operation | Operating Range, p. 3 |
| Output current, absolute maximum | 1.2 A per channel (average); peak 2 A (tw = 20 ms continuous pulse, duty ≤ 20 %) or 3.2 A (tw = 10 ms single pulse) | Absolute Maximum Ratings, p. 3 |
| Logic supply Vcc | 2.7–5.5 V operating (typ. 3 V); absolute maximum 6 V | Operating Range and Absolute Maximum Ratings, p. 3 |
| ON resistance | 0.5 Ω typical (upper + lower, VM ≥ 5 V) | p. 1 |
| Switching freq | Up to 100 kHz | Operating Range, p. 3 |
| Control modes | CW, CCW, short brake, stop | p. 1, p. 4 |
| Package | SSOP24 (0.65mm pitch) | p. 1 |
| Protection | Thermal shutdown, undervoltage lockout | p. 1 |

**The VM minimum.** The 2007-06-30 revision contradicts itself: its pin table
on p. 2 gives VM1–VM3 as "Motor supply (2.5V~13.5V)", while its Operating Range
table on p. 3 gives VM a 4.5 V minimum, with 1.0 A per channel at VM ≥ 5 V and
0.4 A at 5 V > VM ≥ 4.5 V. The 2026-05-13 revision drops the range from the pin
table (p. 2 says only "Motor supply") and puts the minimum at 2.5 V in the
Operating Range table, with the 1.0 A row starting at 4.5 V and the 0.4 A row
limited to operation without PWM. That revision still contradicts itself: its
p. 7 Typical Application Diagram labels the VM supply "+4.5 V to 13.5 V", so
the 2.5 V minimum comes only from the p. 3 Operating Range table, where VM
below 4.5 V is limited to 0.4 A without PWM. In the 2007-06-30 copy, p. 7
("+4.5V ~13.5V") agrees with its p. 3. Which revision first made the change
was not checked.

**In this repo.** Both robocar carriers run VM from the 5 V rail
(`packages/robotics/robocar/hardware.toml`, the LM2596 module set to 5.0 V;
`packages/robotics/robocar/integrated/hardware.toml`, the TPS563208 at 5.07 V
typical, 4.94–5.19 V), inside the operating range and, under the 2026-05-13
revision's 4.5 V breakpoint, in its 1.0 A row. Under the 2007-06-30 revision's
5 V breakpoint the integrated carrier's 4.94 V low end falls in the 0.4 A row,
which is what `.claude/rules/power-stage-checklist.md` (row 9), the integrated
carrier's README and its `hardware.toml` reason from. The motors' stall
current is an unmeasured assumption of about 1.7 A each
([WIRING.md](../../../packages/robotics/robocar/WIRING.md), "The 5 V current
budget"), above both the 1.0 A operating maximum and the 1.2 A absolute
maximum average.

## Datasheets & References

- **Datasheet PDF (Toshiba official, revision 2026-05-13 when read on 2026-11-13):** <https://toshiba.semicon-storage.com/info/TB6612FNG_datasheet_en_20141001.pdf?did=10660&prodName=TB6612FNG>
- **Product page:** <https://toshiba.semicon-storage.com/us/semiconductor/product/motor-driver-ics/brushed-dc-motor-driver-ics/detail.TB6612FNG.html>
- **SparkFun mirror (revision 2007-06-30):** <https://cdn.sparkfun.com/datasheets/Robotics/TB6612FNG.pdf>
