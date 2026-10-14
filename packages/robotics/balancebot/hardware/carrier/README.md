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
| J3 | GY-521 (MPU6050), in a 1×8 female header | Lies flat over the outlined area |
| J1 | AMASS XT30PW-M | 3S pack (through its BMS) |
| J2 | JST-XH 4-pin | LM2596 module: IN+, IN−, OUT+, OUT− |
| J4, J5 | JST-XH 4-pin | Motor coils A1, A2, B1, B2 |
| C1, C2 | 100 µF 35 V SMD electrolytic, 6.3 × 7.7 mm | VMOT bulk at each driver |
| R1 | 10 kΩ, 0805 | nENABLE pull-up to 3V3 (motors off until firmware drives it low) |
| D1 | B5819W (SMD 1N5819), SOD-123 | LM2596 output → XIAO 5V pad, so USB cannot back-feed the buck |
| R2, R3, C3 | 100 kΩ, 27 kΩ, 100 nF, 0805 | Pack voltage divider into D0 (GPIO26, ADC0) |
| H1–H4 | M3 | Mounting holes on a 82 × 54 mm rectangle |

The DRV8825 straps are on the board: M0 and M1 to 3V3 and M2 to GND (1/8
microstep), nSLEEP and nRESET to 3V3, FAULT left open. The Pololu carrier has
no VDD pin. Pin 2 is FAULT, which settles the hardware half of #738.

The divider reads 12.6 V (full 3S) as 2.68 V and draws about 0.1 mA from the
pack. Firmware does not read it yet.

## Check before ordering

These come from library footprints and from memory, not from your parts. Hold
each part against the board render (`just balancebot::carrier-fab`, then open
the Gerbers, or open the board in KiCad):

1. **XT30 polarity.** KiCad's footprint puts "−" on pad 1 and "+" on pad 2; the
   board's silkscreen follows it. Check that the + and − moulded into your XT30
   body match the silkscreen when the connector sits in its outline.
2. **DRV8825 pin order.** The footprint is the Pololu layout: one row GND, FAULT,
   A2, A1, B1, B2, GND, VMOT; the other EN, M0, M1, M2, RST, SLP, STEP, DIR.
   Unbranded carriers usually copy it, but check yours.
3. **GY-521 pin order.** Assumed VCC, GND, SCL, SDA, XDA, XCL, AD0, INT with
   pin 1 at the left end of J3. This one is from memory; read it off your
   module's silkscreen before ordering.

## Assembly notes

The SMD passives use KiCad's hand-solder 0805 pads, and each symbol carries an
LCSC part number, so the same board works both ways:

- **JLCPCB assembly**: `carrier-fab` writes `output/assembly/bom-jlc.csv` and
  `cpl-jlc.csv` with only the seven SMD parts. Upload them with the Gerber zip
  (Economic PCBA, top side). Everything else is hand-soldered afterwards. All
  parts are JLC Basic except C1/C2 (C2836437, Extended: one feeder fee).
  JLC's rotation convention differs from KiCad's for some packages, so check
  the polarity of C1, C2 and D1 in JLC's placement preview: C1/C2 "+" on the
  pad marked "+" on the silkscreen, D1's cathode bar on the left (pad 1,
  XIAO 5V side). Correct the rotation there if the preview shows one flipped.
- **By hand**: skip the two CSVs. Any 1 A / 40 V Schottky in SOD-123 and any
  100 µF ≥ 25 V 6.3 mm SMD electrolytic fit.

- Set the LM2596 to 5.0–5.3 V on the bench **before** plugging it into J2. D1
  drops about 0.3 V, so the XIAO's 5V pad sees roughly 4.7–5.0 V.
- Mount the GY-521 flat with its X arrow toward the board's FWD mark; the
  firmware computes pitch from X and Z. The carrier must sit at axle height for
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

1. **Settle the pre-order checks** above (#748): XT30 polarity, DRV8825 rows,
   GY-521 header.
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
   | `output/assembly/bom-jlc.csv` | Assembly only: the seven SMD parts with LCSC numbers |
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

   Check the outline, the four M3 holes and the "+"/"−" beside the XT30 in the
   Gerber preview.

5. **Choose who solders the seven SMD parts** (R1–R3, C1–C3, D1):

   - **JLCPCB**: turn on *PCB Assembly*, *Economic*, top side. Upload
     `bom-jlc.csv` and `cpl-jlc.csv` when asked. All seven should match; C1/C2
     (C2836437) are an Extended part and add one feeder fee. In the placement
     preview, check C1, C2 and D1's polarity as described in
     [Assembly notes](#assembly-notes) before confirming.
   - **By hand**: leave *PCB Assembly* off and buy the parts listed in
     [Assembly notes](#assembly-notes).

6. **Buy the through-hole parts**, which you solder either way. LCSC orders can
   usually ship with the JLCPCB order:

   | Qty | Part |
   |---|---|
   | 2 | 1×7 female header, 2.54 mm (XIAO) |
   | 5 | 1×8 female header, 2.54 mm (four for the DRV8825s, one for the GY-521) |
   | 1 | AMASS XT30PW-M, horizontal PCB male |
   | 3 | JST-XH B4B-XH-A, with mating 4-pin housings and crimps |
   | 4 | M3 standoffs and screws |

7. **Check out.** Choose shipping and pay.

Minimums on this board, for the fab's capability check: 0.3 mm track,
0.6/0.3 mm via, 0.2 mm annular ring, 1.0 mm silkscreen text.
