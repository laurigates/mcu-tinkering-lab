# Robocar Unified: integrated carrier

The [robocar-unified](../unified/) carrier PCB with the chips on the board itself:
no breakout modules and no buck module. It exists to compare the two approaches.
The firmware, the pin header (`../unified/main/pin_config.h`) and the head board
are robocar-unified's, unchanged. There is no firmware here.

> **Schematic only.** `hardware/carrier/board.toml` has `stage = "schematic"`: there
> is no board layout yet. The schematic passes ERC and netlist parity with KiCad
> 10.0.6 (`just robocar-unified-integrated::carrier-check`). Do not order anything
> from this directory.

## What changes

| Socketed carrier (`../unified`) | Integrated carrier (here) |
|---|---|
| A3 Adafruit TCA9548A breakout | U5 TCA9548APWR (TSSOP-24), C5, upstream I2C pull-ups R14/R15, RESET pull-up R16 |
| A2 Adafruit PCA9685 breakout | U6 PCA9685PW (TSSOP-28), C4; 220 Ω series R17-R24 on the LED and servo channels |
| A1 SparkFun TB6612FNG breakout | U7 TB6612FNG (SSOP-24), C15 (VCC), C16 (VM) |
| A4 Adafruit MAX98357A breakout | U8 MAX98357AETE+ (TQFN-16), C17, C18 |
| A5 CJMCU-2317 (MCP23017) module | U9 MCP23017-E/SS (SSOP-28), C6 |
| J4 lead to an LM2596 module | U10 TPS563201 (SOT-23-6, 3 A), L1 3.3 µH, C9-C14, feedback R12/R13 |
| Servos and LEDs on the PCA9685 breakout's 3-pin columns | J20/J21 servo headers, J22/J23 RGB LED headers |
| Speaker on the MAX98357A breakout's screw terminal | J19 JST-XH 2 |
| Three head boards (two spares) | One head board |

Everything else keeps its reference, value and role: the connectors, the test
headers, the bulk capacitors, the LDO, the mux channel pull-ups, the buzzer and the
mounting holes. Net names are the socketed carrier's, so the two netlists compare
by name.

## Footprint comparison

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
| Parts | 39 | 69 |
| Courtyard area | 5357 mm² | 2441 mm² (−54 %) |
| SMD / through-hole parts | 19 / 20 | 50 / 19 |
| Pads | 200 | 300 |

Where the area goes:

- The 33 parts the two main boards share take 1670 mm². Most of it is the
  connectors, the three 470 µF electrolytics, the 2x10 GPIO mirror header and the
  M3 holes' surroundings, none of which shrink.
- The five breakout sockets and the buck lead take 3687 mm²; the PCA9685 breakout
  alone is 63 x 26 mm (1625 mm²). Their replacements, 36 parts, take 771 mm²: the
  six chips 356 mm², the buck's inductor 46 mm², 24 0805 passives 172 mm², and the
  new servo, LED and speaker connectors 197 mm².
- The socketed carrier's buck converter is off the board: the LM2596 module (about
  43 x 21 mm) sits on the chassis at the end of J4's lead. The integrated buck
  (U10, L1, six capacitors, two resistors) adds about 120 mm² of courtyard to the
  board and removes the module.

The socketed main board is about 70 x 100 mm after sawing, so its courtyards fill
about 76 % of it. At that packing the integrated parts would fit about 3200 mm²,
around 57 x 57 mm, but a board laid out with the same connector rows along the
front and rear edges will not pack that tightly. The layout is the real answer.

Other differences the numbers do not show:

- **Assembly.** Every new part is SMD. TSSOP and SSOP at 0.65 mm pitch hand-solder
  with flux and care; the MAX98357A's TQFN with its exposed pad needs hot air or
  JLCPCB assembly. The socketed carrier needs no fine-pitch soldering.
- **Repair.** A dead breakout unplugs; a dead chip is a rework job.
- **Debug.** The breakouts' own pins are no longer there to probe. J11 and J12
  (supply and I2C test headers) are unchanged; there is no test point on the
  motor-driver inputs or the I2S lines.
- **Height.** The breakouts sat on sockets, about 10-15 mm above the board. The
  tallest integrated parts are the 470 µF electrolytics (10.5 mm) and the headers.

## Design notes

The circuit choices that are not a copy of a breakout:

- **Buck converter.** TPS563201: 4.5-17 V in, 3 A out, matching the LM2596
  module's rating. From TI SLVSD90B: VFB 0.768 V, so 56.2 kΩ over 10 kΩ gives
  5.08 V; the 5 V efficiency curve uses 3.3 µH. EN is tied to VBAT (EN and VIN
  share the 17 V limit). The input and output capacitors (2 x 10 µF, 100 nF,
  2 x 22 µF) are TI's usual D-CAP2 values; **check them against the datasheet's
  application section** before laying out, along with the inductor's saturation
  current against the motors' stall current.
- **Strapping.** Mux, PWM driver and expander address pins go straight to GND
  (0x70, 0x40, 0x20, as `pin_config.h` expects). The PCA9685's OE and EXTCLK go to
  GND, the MCP23017's RESET to 3V3. The MAX98357A keeps the socketed carrier's
  straps: SD_MODE to VDD (left channel), GAIN_SLOT to GND (12 dB).
- **PCA9685 outputs.** The Adafruit breakout has a 220 Ω resistor on every output;
  they are kept only on the channels that leave the board (LEDs, servos). The six
  motor-driver channels go straight to the TB6612FNG.
- **RGB LEDs.** Common anode on 3V3, the PCA9685's own supply, so an output driven
  high leaves no forward voltage across the LED. The LEDs' resistors are on this
  board (R17-R22).
- **LCSC numbers** are set only where the socketed carrier already checked them
  (10 kΩ, 100 Ω, 100 nF, the 470 µF and 22 µF capacitors, the LDO). The new chips,
  the inductor, 220 Ω, 56.2 kΩ, 10 µF and 22 µF ceramics still need parts chosen
  against JLCPCB's stock.

## Files

| Path | What it is |
|---|---|
| [`hardware.toml`](hardware.toml) | Parts and nets; the join's input. Pin roles come from `../unified/main/pin_config.h` |
| `hardware/carrier/board.toml` | KiCad project name, paper size, `stage = "schematic"` |
| `hardware/carrier/robocar-unified-integrated-carrier/` | The KiCad project. The `.kicad_sch` and both library tables are generated; the `.kicad_pro` is hand-kept, copied from the socketed carrier's, with one ERC change (below) |
| `docs/reference/parts/*-tssop24.toml` and siblings | Registry entries for the bare chips, the inductor and the new connectors |

The `.kicad_pro` sets ERC's pin-conflict matrix to OK for an "Unspecified" pin
against any pin type but No-connect. KiCad's MAX98357A symbol types its exposed pad
(pin 17) Unspecified, so tying it to GND otherwise fails ERC (`--severity-all`)
against the ground net's power-input, passive and input pins. It is the only Unspecified pin in the vendored library, so
the change covers that pad and nothing else. An ERC exclusion would be narrower
still, but KiCad matches one only against its own serialisation of the marker,
which `kicad-cli` cannot write.

Never edit the schematic in KiCad: `just kicad::check` fails on any difference from
what `hardware.toml` generates. Change `hardware.toml` and run:

```bash
just robocar-unified-integrated::carrier-schematic
```

## Next steps

Lay out the board: add `[outline]`, `[placement]`, `[net_classes]`, `[signal]`,
`[pour]` and `[silk]` to `board.toml` (the socketed carrier's
`../unified/hardware/carrier/board.toml` is the template), remove `stage`, and run
`carrier-board` then `carrier-check`. Keep the buck's input capacitors, switch node
and inductor tight, and give the TB6612FNG's VM and PGND pins short, wide copper.
