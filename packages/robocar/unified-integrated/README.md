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
| A2 Adafruit PCA9685 breakout | U6 PCA9685PW (TSSOP-28), C4; 220 Ω series R17-R24 on the LED and servo channels |
| A1 SparkFun TB6612FNG breakout | U7 TB6612FNG (SSOP-24), C15 (VCC), C16 (VM) |
| A4 Adafruit MAX98357A breakout | U8 MAX98357AETE+ (TQFN-16), C17, C18, ferrite bead FB1 on its supply |
| A5 CJMCU-2317 (MCP23017) module | U9 MCP23017-E/SS (SSOP-28), C6 |
| J4 lead to an LM2596 module | U10 TPS563201 (SOT-23-6, 3 A), L1 3.3 µH, C9-C14, feedback R12/R13, input bulk C19 |
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
| Parts | 39 | 71 |
| Courtyard area | 5357 mm² | 2234 mm² (−58 %) |
| SMD / through-hole parts | 19 / 20 | 52 / 19 |
| Pads | 200 | 304 |

Where the area goes:

- The 30 parts the two main boards share take 1337 mm². Most of it is the
  connectors, the 2x10 GPIO mirror header and the M3 holes' surroundings, none of
  which shrink.
- The five breakout sockets and the buck lead take 3687 mm²; the PCA9685 breakout
  alone is 63 x 26 mm (1625 mm²). Their replacements, 38 parts, take 845 mm²: the
  six chips 356 mm², the buck's inductor 46 mm² and input bulk capacitor 67 mm²,
  25 0805 passives and the ferrite bead 179 mm², and the new servo, LED and
  speaker connectors 197 mm².
- The three 470 µF electrolytics (332 mm²) became 22 µF tantalums (52 mm²).
- The socketed carrier's buck converter is off the board: the LM2596 module (about
  43 x 21 mm) sits on the chassis at the end of J4's lead. The integrated buck
  (U10, L1, seven capacitors, two resistors) adds about 190 mm² of courtyard to
  the board and removes the module.

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

## Design notes

The circuit choices that are not a copy of a breakout:

- **Buck converter.** TPS563201: 4.5-17 V in, 3 A out, matching the LM2596
  module's rating. TI SLVSD90B Table 7-2 for 5 V: R1 54.9 kΩ over R2 10 kΩ (VFB
  0.768 V, so 4.98 V), L 3.3 µH, and 20-68 µF of output capacitance (C13, C14,
  2 x 22 µF 0805). EN is tied to VBAT (EN and VIN share the 17 V limit). Check
  the inductor's saturation current against the motors' stall current, and the
  22 µF ceramics' capacitance at 5 V bias, which an 0805 can halve.
- **Bulk capacitors.** D-CAP2 regulates on the output ripple, and its stability
  rests on the L x Cout double pole; TI bounds Cout at 68 µF of low-ESR
  capacitance. The socketed carrier's three 470 µF electrolytics made up for an
  LM2596 module at the end of a lead; next to this buck, a low-ESR polymer of the
  same value would sit inside the loop and could make it unstable, and so could
  the electrolytics. C1-C3 are 22 µF MnO2 tantalums (3528, the LDO's part): their
  ESR, around 1 Ω, keeps them out of the loop while they still feed the servo,
  motor and amplifier current steps, and on AMP_5V it damps FB1 against C17, a
  ferrite and a ceramic alone being a resonant LC. The input side has no such
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
  its own net, AMP_5V, with SD_MODE, its decoupling and its 470 µF bulk capacitor.
  It keeps the class-D switching off the motor and logic rail, and it is what lets
  5 V have tracks wider than the amplifier's pins allow (below). Choose a bead
  rated for at least 2 A.
- **RGB LEDs.** Common anode on 3V3, the PCA9685's own supply, so an output driven
  high leaves no forward voltage across the LED. The LEDs' resistors are on this
  board (R17-R22).
- **LCSC numbers** are set only where the socketed carrier already checked them
  (10 kΩ, 100 Ω, 100 nF, the 22 µF tantalums, the LDO). The new chips, the
  inductor, C19, 220 Ω, 54.9 kΩ, 10 µF and 22 µF ceramics still need parts chosen
  against JLCPCB's stock.

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

Two tracks are laid down before routing (`[[tracks]]`, locked on the board):

- **5V trunk**, 1.5 mm on F.Cu from L1's output pad west and north to C2, beside
  the TB6612FNG's VM pins and C16.
- **Buck GND**, 0.4 mm from U10's GND pin, under the package between its pin
  rows, to input capacitor C11's GND pad: the input loop, TI's first layout rule.

5 V is also poured on F.Cu over most of the main board (x 8.5-53, y 26-75.5),
above GND, which keeps B.Cu (and the rest of F.Cu). The trunk sits inside the
pour, so the pour cannot break into islands between L1 and the motor driver, and
the pour fills one body around it. GND's 0.25 mm tracks only need to reach every
pad, since the B.Cu plane joins them; the routed 0.45 mm 5 V tracks are the last
millimetres into fine-pitch pins. Freerouting necks tracks down to 0.15 mm at some
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
| `hardware/carrier/board.toml` | Placement, keepouts, net classes, pour, silkscreen |
| `hardware/carrier/robocar-unified-integrated-carrier/` | The KiCad project. The `.kicad_sch` and both library tables are generated, the `.kicad_pcb` comes from `carrier-board`, and `.check.json` is the stamp `carrier-check` writes; the `.kicad_pro` is hand-kept, copied from the socketed carrier's, with the ERC change below and a 0.15 mm minimum track |
| `docs/reference/parts/*-tssop24.toml` and siblings | Registry entries for the bare chips, the inductor and the new connectors |

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

- Review the placement and Freerouting's tracks.
- Check the inductor's saturation current against the motors' stall current, the
  output ceramics' capacitance at 5 V bias, and on the bench that the 22 µF
  tantalums hold 5 V through a motor stall.
- Choose LCSC parts for the new chips, the inductor, the ferrite bead and the new
  passive values.
