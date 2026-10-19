# Robocar Unified: integrated carrier

The [robocar-unified](../unified/) carrier PCB with the chips on the board itself:
no breakout modules and no buck module. It exists to compare the two approaches.
The firmware, the pin header (`../unified/main/pin_config.h`) and the three head
boards are robocar-unified's, unchanged. There is no firmware here.

> **PROVISIONAL: do not order.** The board is placed by hand, routed by Freerouting
> and passes every carrier gate with KiCad 10.0.6 (ERC, DRC with schematic parity,
> netlist parity, the board check and its stamp), but nobody has reviewed the
> layout or the tracks, and the open items under [Design notes](#design-notes) stand.

## What changes

| Socketed carrier (`../unified`) | Integrated carrier (here) |
|---|---|
| A3 Adafruit TCA9548A breakout | U5 TCA9548APWR (TSSOP-24), C5, upstream I2C pull-ups R14/R15, RESET pull-up R16 |
| A2 Adafruit PCA9685 breakout | U6 PCA9685PW (TSSOP-28), C4; series R17-R24 on the LED and servo channels (220 Ω, 47 Ω on green and blue) |
| A1 SparkFun TB6612FNG breakout | U7 TB6612FNG (SSOP-24), C15 (VCC), C16 (VM) |
| A4 Adafruit MAX98357A breakout | U8 MAX98357AETE+ (TQFN-16), C17, C18, ferrite bead FB1 on its supply |
| A5 CJMCU-2317 (MCP23017) module | U9 MCP23017-E/SS (SSOP-28), C6 |
| J4 lead to an LM2596 module | U10 TPS563208 (SOT-23-6, 3 A), L1 4.7 µH, C9-C14 and C20, feedback R12/R13, EN cutoff divider R25/R26, input bulk C19 |
| (no fuse) | F1 3 A PTC between the XT30 and VBAT |
| C1-C3 470 µF electrolytics (8 x 10.5 mm) | C1-C3 22 µF tantalums (3528), [below](#bulk-capacitors) |
| Servos and LEDs on the PCA9685 breakout's 3-pin columns | J20/J21 servo headers, J22/J23 RGB LED headers |
| Speaker on the MAX98357A breakout's screw terminal | J19 JST-XH 2 |

Everything else keeps its reference, value and role: the connectors, the test
headers, the LDO, the mux channel pull-ups, the buzzer, the mounting holes and
the three head boards (U1/J2, and the spares U3/J16 and U4/J17). Net names are the socketed carrier's, so the two netlists compare
by name.

## Footprint comparison

The laid-out boards, as sawn apart:

| Main board | Socketed | Integrated |
|---|---|---|
| Size | about 70.5 x 100 mm | 64.5 x 76 mm |
| Area | about 7050 mm² | 4902 mm² (−30 %) |

| Whole PCB | Socketed | Integrated |
|---|---|---|
| Outline | 100 x 100 mm | 94 x 100 mm |

Both PCBs carry the same three head boards, the 29.5 mm square for the pan-tilt
cradle, in a column on the right: the integrated column is the socketed one moved
6 mm left. The integrated main board ends at y 76, so the strip below it
(64.5 x 24 mm) is waste; the outline stays 100 mm tall for the heads. Saw on the
waste side of the y 76 line: J3's courtyard ends 0.1 mm short of it. The integrated main board keeps
the socketed one's layout habits (connectors along the front and rear edges, M3
holes in the corners, 0.5 mm edge keepouts), so the difference is the chips, not a
tighter layout style. Its parts' courtyards fill half of it; the socketed board's
fill three quarters.

The parts alone:

```bash
just robocar-unified-integrated::compare
```

prints each carrier's courtyard area, grouped into main board, head board and
mounting holes, then every main-board part only one carrier has
(`tools/kicad/area.py`). Each part counts its courtyard's bounding box: for a
breakout socket that is the module's outline, for a bare chip the package and its
pads. The sum is a lower bound on board area; it leaves out routing and the gaps a
placement leaves. As of this commit:

| Main board | Socketed | Integrated |
|---|---|---|
| Parts | 39 | 75 |
| Courtyard area | 5357 mm² | 2280 mm² (−57 %) |
| SMD / through-hole parts | 19 / 20 | 56 / 19 |
| Pads | 200 | 312 |

Where the area goes:

- The 30 parts the two main boards share take 1337 mm². Most of it is the
  connectors, the 2x10 GPIO mirror header and the M3 holes' surroundings, none of
  which shrink.
- The five breakout sockets and the buck lead take 3687 mm²; the PCA9685 breakout
  alone is 63 x 26 mm (1625 mm²). Their replacements, 42 parts, take about 890 mm²:
  the six chips 356 mm², the buck's inductor 46 mm² and input bulk capacitor
  67 mm², 28 0805 passives and the ferrite bead about 200 mm², the input fuse,
  and the new servo, LED and speaker connectors 197 mm².
- The three 470 µF electrolytics (332 mm²) became 22 µF tantalums (52 mm²).
- The socketed carrier's buck converter is off the board: the LM2596 module (about
  43 x 21 mm) sits on the chassis at the end of J4's lead. The integrated buck
  (U10, L1, eight capacitors, four resistors) adds about 220 mm² of courtyard
  to the board and removes the module.

Other differences the numbers do not show:

- **Assembly.** Every new part is SMD. TSSOP and SSOP at 0.65 mm pitch hand-solder
  with flux and care; the MAX98357A's TQFN with its exposed pad needs hot air or
  JLCPCB assembly. The socketed carrier needs no fine-pitch soldering.
- **Repair.** A dead breakout unplugs; a dead chip is a rework job.
- **Debug.** The breakouts' own pins are no longer there to probe. J11 and J12
  (supply and I2C test headers) are unchanged; there is no test point on the
  motor-driver inputs or the I2S lines.
- **Height.** The breakouts sat on sockets, about 10-15 mm above the board. The
  tallest integrated parts are the headers and the XT30; C19 is 7.7 mm.

## JLCPCB assembly

Every SMD part on the front carries an LCSC number (#808), so JLCPCB can
assemble the whole main board. `just robocar-unified-integrated::carrier-fab`
writes `output/assembly/bom-jlc.csv` and `cpl-jlc.csv`: 23 BOM lines, 56
placements, one per SMD part. The chips and the inductor take their number
from their registry entry (`docs/reference/parts/`), the passives from
`hardware.toml`.

| Ref | Part | LCSC | Library |
|---|---|---|---|
| U5 | TCA9548APWR | C130026 | Extended |
| U6 | PCA9685PW,118 | C2678753 | Extended |
| U7 | TB6612FNG (SSOP-24, 5.3 mm) | C88224 | Extended |
| U8 | MAX98357AETE+T | C910544 | Extended |
| U9 | MCP23017-E/SS | C506653 | Extended |
| U10 | TPS563208DDCR | C468250 | Extended |
| U2 | AMS1117-3.3 | C6186 | Basic |
| L1 | Bourns SRN6045TA-4R7M | C2044594 | Extended |
| F1 | BSMD1812-300-16V, 3 A hold, 16 V | C883162 | Extended |
| FB1 | Murata BLM21PG221SN1D, 220 Ω, 2 A, 45 mΩ | C85840 | Extended |
| C19 | 100 µF 35 V electrolytic, 6.3 x 7.7 mm | C2836437 | Extended |
| C1-C3, C7, C8 | 22 µF 16 V tantalum, 3528 | C8020 | Extended |
| C13, C14, C20 | 22 µF 25 V X5R 0805 | C45783 | Basic |
| C9, C10, C16, C17 | 10 µF 25 V X5R 0805 | C15850 | Basic |
| C4-C6, C11, C12, C15, C18 | 100 nF 50 V X7R 0805 | C49678 | Basic |
| R2-R11, R14, R15 | 2.2 kΩ 0805 | C17520 | Basic |
| R13, R16 | 10 kΩ | C17414 | Basic |
| R12 | 54.9 kΩ | C17747 | Extended |
| R25 | 100 kΩ | C149504 | Basic |
| R26 | 30 kΩ | C17621 | Basic |
| R17, R20, R23, R24 | 220 Ω | C17557 | Basic |
| R18, R19, R21, R22 | 47 Ω | C17714 | Basic |
| R1 | 100 Ω | C17408 | Basic |

Stock and library type are JLCPCB's listing on 2026-10-19. The thinnest stock
is the inductor (179) and the MCP23017 (424); check both before ordering. 54.9 kΩ
has no Basic part; it stays, as TI's Table 7-2 value, at one Extended fee.

### Placement rotation

JLCPCB places each part as LCSC's own footprint draws it, and LCSC draws most of
these chips with pin 1 bottom-left where KiCad has it top-left.
[`docs/reference/jlcpcb-rotations.toml`](../../../docs/reference/jlcpcb-rotations.toml)
holds the offset per LCSC number, derived by `kicad.lcsc_rotation` from LCSC's
footprint pad by pad, and `kicad.jlc` adds it to KiCad's rotation in
`cpl-jlc.csv`: 270° for U5, U7, U8, U9 and U10, 180° for U2, 0° for U6
(the PCA9685PW's LCSC footprint has pin 1 top-left) and for every two-pad part.
Without it the CPL would have put five chips at 90° and the LDO at 180°.
`carrier-fab` fails on a placed LCSC number with no checked offset. JLCPCB's
preview is still the last check: compare each chip's pin-1 dot with the
silkscreen before paying.

### Fiducials

Three fiducials, FID1-FID3 (1 mm copper, 2 mm mask opening), sit on the main
board in an asymmetric triangle: top-left (3.5, 9.5), top-right (60.5, 10.0)
and bottom-left (3.5, 66.5), each 2 mm clear of every courtyard. JLCPCB does not
require them for a board this size; they let the machine correct for board
stretch, which the 0.5 mm-pitch TQFN is least tolerant of.

### What is still hand-soldered

The through-hole parts: the XT30 (J3), the motor, speaker, rangefinder and OLED
connectors (J5-J9, J19), the ribbon header (J1), the servo and LED headers
(J20-J23), the test and ground headers (J11-J15), the GPIO mirror header (J18),
the buzzer (BZ1), and on the heads the XIAO sockets (U1, U3, U4) and their
ribbon headers (J2, J16, J17). All are 2.5 mm pitch or wider.

### Cost

The order is five PCBs, the minimum, of which 2 or 5 can be assembled. At
94 x 100 mm and two layers the PCB falls under JLCPCB's under-10 x 10 cm
promotion, and coupons and other offers stack with it, so the fees change from
order to order; only the live quote under [Before ordering](#before-ordering)
gives the price. The part prices are fixed:

- **Parts:** about $11.50 per board at JLCPCB's listed unit prices
  (2026-10-19). The chips are most of it: PCA9685 $3.33, MCP23017 $1.77,
  MAX98357A $1.32, TB6612FNG $1.15, TCA9548A $0.81. Attrition parts come on
  top.
- **Extended parts:** 12 BOM lines, each with a per-order loading fee unless an
  offer waives it: the six chips, L1, F1, FB1, C19, the tantalums (C8020) and
  R12. The other 11 lines are Basic.

## Design notes

The circuit choices that are not a copy of a breakout:

- **Buck converter.** TPS563208: 4.5-17 V in, 3 A out, matching the LM2596
  module's rating. It is the forced-continuous sibling of the TPS563201, with
  the same pins and the same TI SLVSD90B Table 7-2 values, chosen because it
  sinks current as well as sourcing it: a motor that brakes or reverses pushes
  energy back into 5 V through the TB6612FNG, and a pulse-skipping converter
  would leave that to charge the rail (whose absolute maximum, at the
  MAX98357A, is 6 V). For 5 V: R1 54.9 kΩ over R2 10 kΩ (VFB 0.768 V, so
  4.98 V); L1 4.7 µH ±20 % (Bourns SRN6045TA-4R7M: Isat 6.8 A, Irms 4.5 A,
  above the 5.5 A peak at the 5.1 A maximum current limit), since the 3.3 µH
  part is ±30 % and can fall below the table's 3.3 µH minimum; 3 x 22 µF 0805
  out (C13, C14, C20), since an 0805 keeps roughly half its capacitance at
  5 V and two can fall under the table's 20 µF minimum.
- **Pack voltage.** The converter's maximum duty cycle is 75 % (SLVSD90B
  7.2.2.4), so 5 V needs VIN ≥ 6.7 V, 3.3 V per cell; below that 5 V sags with
  the pack. EN goes through a 100 kΩ / 30 kΩ divider (R25/R26): EN rises at
  1.38-1.47 V typical (1.6 V maximum) and falls at 1.05-1.2 V, so the buck turns
  on at about 6.2 V (6.9 V at the limit) and off at about 4.9 V, and stops
  draining a flat pack. It is coarse, with no specified falling threshold; a
  pack BMS is the real protection. F1, a 3 A hold PTC at the XT30, limits a
  short on VBAT.
- **5 V current budget.** Nothing in the repo names the drive motors; the
  simulation assumes 3 Ω windings and a 2 A stall (`../simulation/config/
  robot_config.yaml`), which is about 1.7 A each at 5 V. Two stalled motors
  (3.4 A) alone pass the buck's 3.3 A minimum current limit, before the
  servos (about 0.7 A each at stall), the head (about 0.5 A) and the amplifier
  (about 1 A peaks). Past the limit the converter hiccups (10 ms restarts)
  and resets the XIAO. Measure the real motors' stall current; if it is near
  this, ramp and cap the PWM duty in firmware, or give the motors their own
  supply.
- **Bulk capacitors.** D-CAP2 regulates on the output ripple, and its stability
  rests on the L x Cout double pole; TI bounds Cout at 68 µF of low-ESR
  capacitance. The socketed carrier's three 470 µF electrolytics made up for an
  LM2596 module at the end of a lead; next to this buck, a low-ESR polymer of the
  same value would sit inside the loop and could make it unstable, and so could
  the electrolytics. C1-C3 are 22 µF MnO2 tantalums (3528, the LDO's part). Their
  ESR (about 1.5 Ω) is not enough to take them out of the loop: at the LC
  double pole, near 18 kHz, the tantalums on 5V look like about 0.4 Ω in
  parallel, about the ceramics' reactance there, so they damp the pole rather
  than move it. That is probably benign but outside TI's table: bench-test a
  0.2-2 A load step before trusting it. On AMP_5V the tantalum damps FB1
  against C17, a ferrite and a ceramic alone being a resonant LC. The input side has no such
  limit, so C19 (100 µF 25 V, 6.3 x 7.7 mm) is an aluminium electrolytic on VBAT:
  its ESR damps the ringing of the battery lead's inductance against the ceramic
  input capacitors when the XT30 is plugged in, which a polymer would not, and a
  tantalum takes that surge badly. The motors' stall current now draws on the
  buck's 3 A and C2 rather than a 470 µF reservoir; check for brown-outs on the
  bench.
- **Strapping.** Mux, PWM driver and expander address pins go straight to GND
  (0x70, 0x40, 0x20, as `pin_config.h` expects). The PCA9685's OE and EXTCLK go to
  GND, the MCP23017's RESET to 3V3. The MAX98357A keeps the socketed carrier's
  straps: SD_MODE to VDD (left channel), GAIN_SLOT to GND (12 dB).
- **PCA9685 outputs.** The Adafruit breakout has a 220 Ω resistor on every output;
  they are kept only on the channels that leave the board (LEDs, servos). The six
  motor-driver channels go straight to the TB6612FNG.
- **Amplifier supply.** The MAX98357A takes 5 V through a ferrite bead (FB1) as
  its own net, AMP_5V, with SD_MODE, its decoupling and its 22 µF bulk capacitor.
  It keeps the class-D switching off the motor and logic rail, and it is what lets
  5 V have tracks wider than the amplifier's pins allow (below). Choose a bead
  rated for at least 2 A.
- **RGB LEDs.** Common anode on 3V3, the PCA9685's own supply, so an output driven
  high leaves no forward voltage across the LED. The LEDs' resistors are on this
  board (R17-R22): 220 Ω on red (about 6 mA at 2 V), 47 Ω on green and blue,
  whose 2.9-3.2 V forward voltage leaves next to nothing across 220 Ω from 3V3
  (about 4-6 mA through 47 Ω).
- **I2C pull-ups** are 2.2 kΩ, upstream (R14/R15) and on every mux channel
  (R2-R11), as on the socketed carrier since #810: at 400 kHz, Fast-mode's
  300 ns rise time allows 10 kΩ only below about 35 pF of bus. RESET's R16
  stays 10 kΩ.
- **LCSC numbers** are set on every SMD part: see
  [JLCPCB assembly](#jlcpcb-assembly).

## Layout

`hardware/carrier/board.toml` holds the placement, the fixed tracks, the net
classes, the pour and the silkscreen. The buck converter follows TI's layout
order: SW straight into L1, the input capacitors beside VIN with their GND pads
facing U10's GND pin, the bootstrap capacitor along VBST's side to the switch
node at L1, the feedback divider next to them.

Freerouting gives each net one track width, so a net class is the widest track every
pin on its nets accepts:

| Class | Nets | Track | Limited by |
|---|---|---|---|
| input_power | VBAT, BUCK_SW | 0.8 mm | TPS563201 pads, 0.6 mm at 0.95 mm pitch |
| motor_power | +5V, MOT_* | 0.45 mm | TB6612FNG pads, 0.4 mm at 0.65 mm pitch |
| head_power | HEAD*_+5V, HEAD*_GND | 0.5 mm | |
| logic_power | +3V3 | 0.35 mm | |
| signal | everything else | 0.25 mm | MAX98357A pads, 0.25 mm at 0.5 mm pitch |

Tracks laid down before routing (`[[tracks]]`, locked on the board):

- **5V trunk**, 1.5 mm on F.Cu from L1's output pad west and north to C2, beside
  the TB6612FNG's VM pins and C16.
- **Buck GND**, 0.4 mm from U10's GND pin, under the package between its pin
  rows, to input capacitor C11's GND pad: the input loop, TI's first layout rule.
- **U8 ground ties**, from the MAX98357A's GND pins (3, 11, 15) and GAIN_SLOT
  (2) straight across to its exposed pad.

GND is poured on both layers of the main board (each head's ground on its
head), and each region stops 1 mm short of a cut line. There is no 5 V pour:
the first layout poured 5 V on F.Cu above GND, which cut F.Cu GND into islands
around every chip in the power half and left their ground pads joined to the
B.Cu plane only through 0.19-0.25 mm tracks (the board had one GND via). Now
`[fanout]` puts a 0.6/0.3 mm via, on a 0.4 mm stub, beside every SMD pad of a
poured net after routing, outward from its part, wherever it clears
everything; the exposed pad takes its via in the pad. `route_board.py` fails
if a `required` pad (the buck, the motor driver's PGND and VM capacitors, the
amplifier's and the LDO's capacitors) finds no room; a logic chip's ground
pin or strap without one keeps its routed track. Freerouting necks tracks down to 0.15 mm at some
fine-pitch pads, so the `.kicad_pro` sets a 0.15 mm minimum track width (JLCPCB's
two-layer minimum is 0.127 mm).

```bash
just kicad::setup                                 # KiCad 10.0.6, Freerouting, Java 25
just robocar-unified-integrated::carrier-board    # place, route, pour
just robocar-unified-integrated::carrier-check    # every gate, then the stamp
```

## Files

| Path | What it is |
|---|---|
| [`hardware.toml`](hardware.toml) | Parts and nets; the join's input. Pin roles come from `../unified/main/pin_config.h` |
| `hardware/carrier/board.toml` | Placement, keepouts, fixed tracks, net classes, pour, fanout, silkscreen |
| `hardware/carrier/robocar-unified-integrated-carrier/` | The KiCad project. The `.kicad_sch` and both library tables are generated, the `.kicad_pcb` comes from `carrier-board`, and `.check.json` is the stamp `carrier-check` writes; the `.kicad_pro` is hand-kept, copied from the socketed carrier's, with the ERC change below and a 0.15 mm minimum track |
| `docs/reference/parts/*-tssop24.toml` and siblings | Registry entries for the bare chips, the inductor and the new connectors |
| `docs/reference/jlcpcb-rotations.toml` | JLCPCB's placement offset for every LCSC number, read by `carrier-fab` |

The `.kicad_pro` sets ERC's pin-conflict matrix to OK for an "Unspecified" pin
against any pin type but No-connect. KiCad's MAX98357A symbol types its exposed pad
(pin 17) Unspecified, so tying it to GND otherwise fails ERC (`--severity-all`)
against the ground net's power-input, passive and input pins. It is the only
Unspecified pin in the vendored library, so the change covers that pad and nothing
else. An ERC exclusion would be narrower
still, but KiCad matches one only against its own serialisation of the marker,
which `kicad-cli` cannot write.

Never edit the schematic in KiCad: `just kicad::check` fails on any difference from
what `hardware.toml` generates. Change `hardware.toml` and run:

```bash
just robocar-unified-integrated::carrier-schematic
```

## Before ordering

A pre-fabrication review (adversarial, against the datasheets and the routed
board) found the ground vias missing and the issues fixed above. Still open:

- Measure the drive motors' stall current at 5 V against the buck's 3.3 A
  minimum current limit (see the current budget above).
- Bench-test a 0.2-2 A load step on 5 V with the tantalums fitted.
- Confirm whether the pack has a BMS; the EN divider is only a coarse cutoff.
- Get a live JLCPCB quote for 2 and 5 assembled boards (upload the zip, the
  BOM and the CPL), check the placement preview part by part against
  [the rotation table](#placement-rotation), and record the quote here.
- Saw on the waste side of each cut line.
