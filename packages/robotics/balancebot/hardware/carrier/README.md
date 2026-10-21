# Balancebot carrier PCB

A 90 × 62 mm, 2-layer carrier that replaces the hand-wired harness in
[`WIRING.md`](../../WIRING.md). Every module sits in a socket or plugs into a
connector, so any of them can be swapped without desoldering. The passives are
SMD on the top side; the sockets and connectors are through-hole.

KiCad 10 project: [`balancebot-carrier/`](balancebot-carrier/). Its schematic is
generated from [`hardware.toml`](../../hardware.toml) and the parts registry
([`docs/reference/parts/`](../../../../../docs/reference/parts/)) by `tools/kicad`
([ADR-027](../../../../../docs/decisions/ADR-027-kicad-carrier-generation.md)):
never edit `balancebot-carrier.kicad_sch` in KiCad; change `hardware.toml` or a
registry entry and regenerate. The board is placed and routed from the same
netlist and [`board.toml`](board.toml) (placement, silkscreen, net classes, pour).

## What is on it

| Ref | Part | Connects |
|---|---|---|
| U1 | Seeed XIAO RP2350, in two 1×7 female headers | USB-C toward the top edge |
| A1, A2 | Pololu-format DRV8825 carriers, in 1×8 female headers | Left / right stepper |
| J3 | GY-521 (MPU6050), in a 1×8 female header | Lies flat, component side up, over the outlined area; the header is on its rear edge |
| J1 | JST-XH 2-pin (B2B-XH-A) | 3S pack (through its BMS); BAT+ on pad 1 (the square pad, right), marked "+" below it |
| F1 | 3 A hold PTC, 1812 (BSMD1812-300-24V) | Pack input: limits a short on VBAT |
| Q1 | ZXMP4A16G P-MOSFET, SOT-223 | Reverse-polarity switch: drain (tab) to F1, source to VBAT |
| R4 | 10 kΩ, 0805 | Q1's gate to GND |
| J2 | JST-XH 4-pin | LM2596 module: IN+, IN−, OUT+, OUT− |
| J4, J5 | JST-XH 4-pin | Motor coils A1, A2, B1, B2 |
| C1, C2 | 100 µF 35 V SMD electrolytic, 6.3 × 7.7 mm | VMOT bulk at each driver |
| R1 | 10 kΩ, 0805 | nENABLE pull-up to 3V3 (motors off until firmware drives it low) |
| D1 | B5819W (SMD 1N5819), SOD-123 | LM2596 output → XIAO 5V pad, so USB cannot back-feed the buck |
| R2, R3, C3 | 100 kΩ, 27 kΩ, 100 nF, 0805 | Pack voltage divider into D0 (GPIO26, ADC0); C3 sits at the pin |
| H1–H4 | M3 | Mounting holes on a 82 × 54 mm rectangle |

The DRV8825 straps are on the board: M0 and M1 to 3V3 and M2 to GND (1/8
microstep), nSLEEP and nRESET to 3V3, FAULT left open. The Pololu carrier has
no VDD pin. Pin 2 is FAULT, which settles the hardware half of #738.

The divider reads 12.6 V (full 3S) as 2.68 V and draws about 0.1 mA from the
pack. Firmware does not read it yet.

### Pack input protection

J1's BAT+ reaches VBAT through F1 and Q1; the hand-wired harness has neither.
The values below are from the datasheets, not measured on this board.

- **Q1** is a P-MOSFET high-side switch: drain to F1, source to VBAT, gate to
  GND through R4. With the pack the right way round its body diode conducts
  first, the gate sits up to 12.6 V below the source and the channel turns on.
  Reversed, the gate is above the source and the body diode is reverse-biased,
  so nothing downstream sees the reversed pack. ZXMP4A16G (Diodes DS33584
  Rev. 6-2): VDSS −40 V, VGSS ±20 V, RDS(on) ≤ 60 mΩ at VGS −10 V. A full 3S
  pack puts |VGS| at 12.6 V, 7.4 V under the rating, so there is no gate
  zener. At F1's 3 A hold current Q1 dissipates up to 0.54 W, about a 34 °C
  rise at the datasheet's 62.5 °C/W (25 × 25 mm of 1 oz copper; the board
  gives it less). The robot's real pack current is not measured yet.
- **F1** is a 3 A hold, 6 A trip PTC (BSMD1812-300-24V, BHFUSE datasheet;
  Ihold falls to 2.55 A at 40 °C). It is the 24 V part of the family the
  integrated robocar carrier uses at 16 V: 16 V leaves 3.4 V over a full 3S
  pack, 24 V leaves 11.4 V for a hot-plug overshoot into 200 µF of bulk
  capacitance.
- **Power-stage checklist** (`.claude/rules/power-stage-checklist.md`): items
  1–2 and 4–7 (dropout, low-battery cutoff, sink capability, current limit,
  capacitance, inductor) belong to the off-board LM2596 module; 3 (input
  protection) is F1 and Q1; 8 (layout) is the fixed 1.5 mm PACK_IN, PACK_FUSED
  and VBAT tracks in [`board.toml`](board.toml).
- **Low-battery cutoff.** The DRV8825 runs from 8.2 V (TI SLVSA73F, section 1),
  and the pack's BMS cuts off lower than that. Firmware should stop the motors
  near 9 V on the pack divider (#747).

### VBAT and ground

VBAT is laid down as fixed F.Cu tracks, 1.5 mm wide except the 1.0 mm feed to
the divider (R2), with no via: Q1's source to A1's VMOT and C1, to the buck
lead's IN+ (J2), and from there round the east edge to C2 and A2's VMOT
(129 mm from Q1's source to A2's VMOT). GND is poured on both layers, with a
via beside each SMD ground pad and stitching vias on a 5 mm grid wherever both
layers' pour holds one (118 GND vias). The pour stays 3.5 mm from each M3
hole's centre.

## Check before ordering

These come from library footprints and from memory, not from your parts. Hold
each part against the board render (`just balancebot::carrier-fab`, then open
the Gerbers, or open the board in KiCad):

1. **Battery lead polarity.** JST-XH has no polarity of its own: the board
   prints "+" below pad 1 (BAT+, the square pad, on the right) and "−" below
   pad 2. Crimp the pack lead to match. A reversed lead is blocked by Q1
   rather than reaching VBAT, but check it anyway. JST-XH is rated 3 A, which
   leaves little headroom with both steppers drawing at once.
2. **DRV8825 pin order.** The footprint is the Pololu layout: one row GND, FAULT,
   A2, A1, B1, B2, GND, VMOT; the other EN, M0, M1, M2, RST, SLP, STEP, DIR.
   Confirmed 2026-10-14 against the HW-216 carriers on hand (labels on the
   bottom: EN faces VMOT, DIR faces GND). A module also fits its socket
   turned 180°, so the board prints VMOT, EN and DIR at those ends of each
   socket: **the "VMOT" text must be at the module's VMOT pin** before you
   power it.
3. **GY-521 pin order.** VCC, GND, SCL, SDA, XDA, XCL, AD0, INT, with pin 1 at
   the left end of J3. Confirmed 2026-10-14 against the module on hand. With J3
   on the rear edge of its outline and the module component side up, VCC lands
   on pin 1 and the X arrow points to FWD.
4. **The buck lead.** J2 is labelled IN+, −, 5V, − (the registry's IN+, IN−,
   OUT+, OUT−). Meter the LM2596 lead's four wires against those labels
   before the first plug-in: OUT+ on IN+ puts the pack on the buck's output.
5. **The motor leads.** J4 and J5 are labelled A1, A2, B1, B2 by their pads.

## Assembly notes

The board is meant to be hand-soldered (a bare PCB order). The SMD parts use
KiCad's hand-solder pads, and each symbol still carries an LCSC part number,
so JLCPCB assembly remains possible:

- **JLCPCB assembly**: `carrier-fab` writes `output/assembly/bom-jlc.csv` and
  `cpl-jlc.csv` with only the ten SMD parts. Upload them with the Gerber zip
  (Economic PCBA, top side). Everything else is hand-soldered afterwards.
  C1/C2 (C2836437), F1 (C7500481) and Q1 (C141324) are JLC Extended parts
  (a feeder fee each); the rest are Basic.
  JLC's rotation convention differs from KiCad's for some packages, so check
  the polarity of C1, C2 and D1 in JLC's placement preview: C1/C2 "+" on the
  pad marked "+" on the silkscreen, D1's cathode bar on the left (pad 1,
  XIAO 5V side). Correct the rotation there if the preview shows one flipped.
- **By hand**: skip the two CSVs. Any 1 A / 40 V Schottky in SOD-123 and any
  100 µF ≥ 25 V 6.3 mm SMD electrolytic fit. Solder Q1's tab (its drain)
  as well as its three pins.

- Set the LM2596 to 5.0–5.3 V on the bench **before** plugging it into J2. D1
  drops about 0.3 V, so the XIAO's 5V pad sees roughly 4.7–5.0 V.
- Mount the GY-521 flat, component side up, on straight pins pushed down into
  J3, so the module covers the outlined area in front of the header and its X
  arrow points to the board's FWD mark; the firmware computes pitch from X and
  Z (Z up). The carrier must sit at axle height for
  the IMU placement the build guide describes.
- Set each DRV8825's Vref before connecting motors, as in the build guide.

## Regenerate and check

```sh
just balancebot::carrier-schematic  # schematic + library tables from hardware.toml (no KiCad)
just balancebot::carrier-board      # place from scratch, autoroute (Freerouting), pour GND
just balancebot::carrier-check      # ERC, DRC + schematic parity, netlist parity, board check,
                                    # then the check stamp CI verifies
just balancebot::carrier-fab        # Gerbers + drill files → output/balancebot-carrier-fab.zip
just balancebot::carrier-lib        # socket XIAO footprint + symbol from Seeed's DIP ones
```

`carrier-check` and `carrier-fab` need KiCad 10.0.6 (`kicad-cli`).
`carrier-board` also needs KiCad's bundled Python (it provides `pcbnew`), Java
25 or later, and the Freerouting 2.5.0 jar at
`~/.cache/freerouting/freerouting-2.5.0.jar` (or `FREEROUTING_JAR`). Freerouting
does not route the same way twice, so a re-run gives a different, equally
checked board: commit the board you checked.

`carrier-check` fails if the committed schematic or board no longer matches the
join (ADR-021): `kicad.parity` compares KiCad's own netlist of the schematic
with the join's, and `kicad.boardcheck` compares every board pad's net, value,
LCSC number and footprint path with it. So renumbering a pin in
`src/pin_config.h` without regenerating and re-routing fails here. Only when
every gate passes does it write `balancebot-carrier.check.json`, which records
the hashes of the files it checked; CI (`carrier-check.yml`, no KiCad) fails
when the committed files differ from it.

### Why the board is built by script

- **Placement** is the table in [`board.toml`](board.toml); parts and nets come
  from the netlist `tools/kicad` builds from `hardware.toml`.
- **The XIAO footprint** is derived from Seeed's `XIAO-RP2350-DIP`
  ([OSHW-XIAO-Series @ cc7f3bb](https://github.com/Seeed-Studio/OSHW-XIAO-Series),
  MIT), which is kept unmodified in
  [`tools/kicad/lib/seeed-xiao/`](../../../../../tools/kicad/lib/seeed-xiao/),
  by `tools/kicad/derive_xiao.py`. The DIP version mills a cutout under the
  module and plates holes for its bottom pads; a socketed XIAO needs neither.
- **`balancebot-carrier.kicad_dru`** lets a GND thermal relief resolve to one
  spoke. `route_board.py` refuses to pour unless every GND pad also has a routed
  track, so the spoke is never the only connection.

## Ordering

The board options and form labels below are from JLCPCB's order page as it was
when this was written; if the page differs, follow the page and check the
detected size and layer count.

1. **Settle the pre-order checks** above (#748): battery lead polarity,
   DRV8825 rows and VMOT marks, GY-521 header, buck and motor leads.
2. **Start from the committed board.** Opening the project in KiCad and saving
   rewrites the `.kicad_pcb`. `git status` must show nothing under
   `hardware/carrier/`; `git restore` the board if KiCad changed it and you did
   not mean to.
3. **Generate the files:**

   ```sh
   just balancebot::carrier-fab
   ```

   It runs `carrier-check` (ERC, DRC with schematic parity, netlist parity,
   the board check) first and stops if any fails. Outputs:

   | File | For |
   |---|---|
   | `output/balancebot-carrier-fab.zip` | The board: Gerbers and drill files |
   | `output/assembly/bom-jlc.csv` | Assembly only: the ten SMD parts with LCSC numbers |
   | `output/assembly/cpl-jlc.csv` | Assembly only: their positions and rotations |

4. **Upload the zip** at jlcpcb.com (Order now). It should detect 2 layers and
   90 × 62 mm; stop if it does not. Then:

   | Option | Choose |
   |---|---|
   | Base material | FR-4 |
   | Layers | 2 |
   | Quantity | 5 (the minimum) |
   | Thickness | 1.6 mm |
   | Surface finish | HASL, or lead-free HASL |
   | Outer copper | 1 oz (the track widths assume it) |
   | Mark on PCB / order number | JLCPCB prints one by default; remove it for a fee, or leave it |

   Check the outline, the four M3 holes and the "+"/"−" beside J1 in the
   Gerber preview.

5. **Choose who solders the ten SMD parts** (R1–R4, C1–C3, D1, F1, Q1):

   - **JLCPCB**: turn on *PCB Assembly*, *Economic*, top side. Upload
     `bom-jlc.csv` and `cpl-jlc.csv` when asked. All ten should match; C1/C2,
     F1 and Q1 are Extended parts and add a feeder fee each. In the placement
     preview, check C1, C2 and D1's polarity, and Q1's tab, as described in
     [Assembly notes](#assembly-notes) before confirming.
   - **By hand**: leave *PCB Assembly* off and buy the parts listed in
     [Assembly notes](#assembly-notes).

6. **Buy the through-hole parts**, which you solder either way. LCSC orders can
   usually ship with the JLCPCB order:

   | Qty | Part |
   |---|---|
   | 2 | 1×7 female header, 2.54 mm (XIAO) |
   | 5 | 1×8 female header, 2.54 mm (four for the DRV8825s, one for the GY-521) |
   | 1 | JST-XH B2B-XH-A, with a mating 2-pin housing and crimps (battery) |
   | 3 | JST-XH B4B-XH-A, with mating 4-pin housings and crimps |
   | 4 | M3 standoffs and screws |

7. **Check out.** Choose shipping and pay.

Minimums on this board, for the fab's capability check (measured on the
routed board): 0.225 mm track (three necked NENABLE segments at A1), 0.6/0.3 mm
via (0.15 mm annular ring), 0.32 mm annular ring on through-hole pads, 1.0 mm
silkscreen text and 0.15 mm silkscreen lines.
