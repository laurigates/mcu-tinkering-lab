# robocar-unified carrier PCB

> **PROVISIONAL: do not order yet.** The pre-order checks below are still owed.
> The silkscreen says so too, and [`board.toml`](board.toml) marks every
> unmeasured dimension (#750).

One 100 × 100 mm, 2-layer PCB that carries two boards and replaces the
hand-wired harness in [`WIRING.md`](../../WIRING.md):

- the **main board**, on the 2WD chassis plate: the motor driver, the PWM
  driver, the I2C mux, the amplifier, the 3V3 regulator, the power and
  peripheral connectors, and the test headers;
- the **head board**, in the top-right corner, in the pan-tilt kit's top
  cradle: the XIAO ESP32-S3 Sense and the head end of the ribbon.

A 3 mm copper-free band under a silkscreen saw line ("CUT", on both faces)
separates them. JLCPCB makes one rectangle with no V-cuts, tabs or slots; the
head is sawn off after delivery (see [Separating the head](#separating-the-head)).
A 2 × 7 IDC ribbon joins the two boards.

KiCad 10 project: [`robocar-unified-carrier/`](robocar-unified-carrier/). Its
schematic is generated from [`hardware.toml`](../../hardware.toml) and the parts
registry ([`docs/reference/parts/`](../../../../../docs/reference/parts/)) by
`tools/kicad`
([ADR-027](../../../../../docs/decisions/ADR-027-kicad-carrier-generation.md));
the board is placed and routed from the same netlist and
[`board.toml`](board.toml), which holds every position, the band and the
silkscreen. Never edit the `.kicad_sch` or `.kicad_pcb` in KiCad; change those
files and regenerate (see [Regenerate and check](#regenerate-and-check)).

## What is on it

Each module sits in female headers or plugs into a connector, so any of them can
be swapped without desoldering. The passives and the regulator are SMD on the
top side; everything else is through-hole.

| Ref | Part | Board | Notes |
|---|---|---|---|
| U1 | XIAO ESP32-S3 Sense, in two 1×7 female headers | head | USB-C toward the top edge, between the M2 holes |
| J2 | 2×7 pin header, unshrouded | head, **back face** | The head end of the ribbon |
| H5, H6 | M2 holes, 2.2 mm | head | Through the cradle into heat-set inserts |
| J1 | 2×7 pin header, unshrouded | main | The main end of the ribbon, beside the band |
| A1 | SparkFun TB6612FNG, in 2 × 1×8 female headers | main | Motor outputs on its left header |
| A2 | Adafruit PCA9685, in female headers on its JP3 side header and its channel 8–13 PWM pins | main | Motor control lines; servo and LED columns stay on the module |
| A3 | Adafruit TCA9548A, in 2 × 1×12 female headers | main | Every I2C device sits behind it |
| A4 | Adafruit MAX98357A, in a 1×7 female header | main | The speaker stays on the module's terminal |
| U2 | AMS1117-3.3, SOT-223 | main | The main board's 3V3; the XIAO's 3V3 pad stays open |
| C1–C3 | 470 µF 16 V electrolytic, 8 × 10.5 mm | main | Bulk at the PCA9685 V+, TB6612FNG VM and MAX98357A Vin |
| C4, C5 | 100 nF, 0805 | main | PCA9685 VCC, TCA9548A VIN |
| C7, C8 | 22 µF 16 V tantalum, 3528 | main | AMS1117 input and output |
| R1 | 100 Ω, 0805 | main | Piezo series resistor |
| R2–R9 | 10 kΩ, 0805 | main | Pull-ups on mux channels 0, 1, 3 and 4 |
| J3 | XT30 (AMASS XT30PW-M) | main | Pack in; pad 1 is "−" |
| J4 | JST-XH 4-pin | main | LM2596 module: IN+, IN−, OUT+, OUT− |
| J5, J6 | JST-XH 2-pin | main | Left and right motor |
| J7, J8 | JST-XH 4-pin | main | RCWL-1601 front and rear, I2C mode: VCC, SCL, SDA, GND |
| J9 | JST-XH 4-pin | main | SSD1306 0.91" OLED: GND, VCC, SCL, SDA |
| J10 | JST-XH 2-pin | main | Piezo: + on pin 1 |
| J11–J15 | 1×4 and 1×2 pin headers | main | Test headers, see [Debug headers](#debug-headers) |
| H1–H4 | M3 holes | main | Chassis plate |

The per-part values, LCSC numbers and the reasons for each are in
[`hardware.toml`](../../hardware.toml); the pin orders are in the registry entries
it names.

## The ribbon survives a reversed plug

The headers are unshrouded, so the IDC plug fits either way round, which takes
pin *n* to pin 15 − *n*. The pinout in
[`idc-2x07-xiao-cable.toml`](../../../../../docs/reference/parts/idc-2x07-xiao-cable.toml)
pairs each pin with one it can safely swap with: 5 V lands on 5 V, GND on GND,
SDA on SCL, UART0 TX on the unconnected pin 9, and every other XIAO output on
another main-board input. **A reversed plug damages nothing and does not work.**
If the robot is dead after reassembly, check the plug orientation first: both
ends print a pin-1 mark.

## Debug headers

Through-hole pin headers on the main board, each pin's net printed beside it.

| Ref | Pins | For |
|---|---|---|
| J11 | +3V3, +5V, GND, VBAT | Meter probes. VBAT's only neighbour is GND, so a slipped probe cannot put the pack on a logic rail |
| J12 | GND, SDA, SCL, MUX_RST | Logic analyser on the upstream I2C bus and the mux reset, on the main board's side of the ribbon |
| J13 | GND, GND | Scope ground clip, beside the MAX98357A and C3 |
| J14 | GND, GND | Scope ground clip, beside the AMS1117 |
| J15 | GND, UART0 TX | USB-serial adapter: its GND to GND, its RX to TX |

**The UART0 tap.** The ribbon carries the XIAO's D6 (GPIO43, UART0 TX) to J15.
The ESP32-S3 ROM bootloader prints its boot messages there at every reset, at
115200 baud, with no firmware change: reset reasons and boot mode, readable
when the USB-C console is unavailable. The application's logs go to the
USB-Serial-JTAG console on the USB-C connector, not to UART0 (see
[`main/pin_config.h`](../../main/pin_config.h)), so J15 shows them only if the
firmware is changed to mirror its logs there. Nothing on the carrier drives or
loads the line.

## Pre-order checks still owed (Lauri)

Each is a measurement or a part in hand that no file in this repo can settle.
Correct [`board.toml`](board.toml) or the registry entry, then regenerate and
re-check.

1. **Cradle M2 holes.** H5 and H6 are 22.5 mm apart in the head's top corners.
   Drill the cradle to match, or measure where it has room and move them.
2. **J2 and the plug behind the head.** J2 is on the back face; check that the
   IDC plug and the ribbon's bend clear the cradle and the pan servo.
3. **Camera orientation.** With U1's USB-C toward the head's top edge, check the
   Sense camera faces forward when the head is in the cradle.
4. **Chassis plate.** Measure the plate's usable area and its M3 holes; the
   outline and H1–H4 are proposals.
5. **Ribbon length** for the full pan and tilt travel, from J1 beside the band.
6. **RCWL-1601 and OLED pin orders** (J7, J8, J9) against the fitted modules:
   both were read from listing photos, not continuity-checked. Check each
   RCWL-1601 has 10 kΩ on R4 (I2C mode) and R5 empty.
7. **Hold each fitted breakout against the board render** (TB6612FNG,
   PCA9685, TCA9548A, MAX98357A): headers on the footprints, outline inside its
   courtyard.
8. **Polarity** of the XT30 pack lead (J3 pad 1 is "−", next to the silkscreen
   "−"), the LM2596 lead (J4 pin 1 IN+), and the piezo lead (J10 pin 1 +).
9. **Saw kerf against the band.** The band is 3 mm with the cut line in the
   middle; a blade wider than about 1 mm leaves no bare margin.

## Assembly notes

- **Set the LM2596 to 5.0 V on the bench before connecting it to J4.** The
  module's output is adjustable and arrives set to whatever the factory left.
- **The heat-set inserts go in the cradle, not the PCB.** H5 and H6 are plain
  2.2 mm clearance holes for M2 screws.
- The SMD parts carry LCSC numbers, so JLCPCB can place them (see
  [Ordering](#ordering)); by hand, any 470 µF ≥ 16 V 8 × 10.5 mm SMD
  electrolytic and 22 µF ≥ 16 V 3528 tantalum fit.

## Separating the head

Score-and-snap cannot make this cut: the line is L-shaped, and a snap along one
leg cracks into the main board at the corner. Saw it.

1. **Buy a jeweller's saw frame with fine blades** (or a coping saw with a fine
   blade).
2. **Clamp the board** flat, the band just past the bench edge.
3. **Cut in the waste band, along the silk line.** Each leg ends in the 2 mm
   relief hole at the inside corner, so the blade turns in the hole instead of
   binding in the corner.
4. **Sand the head to 29.5 × 29.5 mm** on sandpaper on a flat surface, checking
   with calipers. The cradle's inner width is 30.0 mm.

FR-4 dust contains glass fibre: wet-sand, and wear a mask.

## Regenerate and check

```sh
just robocar-unified::carrier-schematic  # schematic + library tables from hardware.toml (no KiCad)
just robocar-unified::carrier-board      # place from scratch, autoroute (Freerouting), pour
just robocar-unified::carrier-check      # ERC, DRC + schematic parity, netlist parity, board check,
                                         # then the check stamp CI verifies
just robocar-unified::carrier-fab        # carrier-check, then Gerbers, drill, BOM and CPL
```

The tool requirements, and why the board is built by script, are in
[the balancebot carrier README](../../../../robotics/balancebot/hardware/carrier/README.md#regenerate-and-check)
and the `carrier-pcb` skill. Freerouting does not route the same way twice:
commit the board you checked.

## Ordering

Not until the pre-order checks are done and the PROVISIONAL text is gone from
`board.toml`'s `[silk] texts`.

1. **Start from the committed board.** `git status` must show nothing under
   `hardware/carrier/`.
2. **Generate the files:** `just robocar-unified::carrier-fab`. It stops if any
   carrier gate fails. Outputs, under `robocar-unified-carrier/output/`:

   | File | For |
   |---|---|
   | `robocar-unified-carrier-fab.zip` | The board: Gerbers and drill files |
   | `assembly/bom-jlc.csv` | Assembly only: the SMD parts with LCSC numbers |
   | `assembly/cpl-jlc.csv` | Assembly only: their positions and rotations |

3. **Upload the zip** at jlcpcb.com. It should detect 2 layers and
   100 × 100 mm. FR-4, 1.6 mm, 1 oz copper (the track widths assume it), HASL.
   In the Gerber preview check the outline, the relief hole at the band's
   inside corner, both "CUT" lines, and J2's outline on the bottom silkscreen.
4. **Assembly (optional):** *PCB Assembly*, *Economic*, top side, with the two
   CSVs. C1–C3 (C96179) and C7–C8 (C8020) are Extended parts, so two feeder
   fees; the rest are Basic. JLCPCB's stock and part status were checked on
   2026-10-16 (the commit that set C96179 records the figures). **Check
   polarity in the placement preview** before confirming: the CPL carries
   KiCad's rotations, and a community rotation table
   ([JLCKicadTools `cpl_rotations_db.csv`](https://github.com/matthewlai/JLCKicadTools/blob/master/jlc_kicad_tools/cpl_rotations_db.csv))
   lists a 180° correction for all three footprints involved: `CP_Elec_8x10.5`
   (C1–C3), `CP_EIA-3528` (C7, C8) and `SOT-223` (U2). Each "+" must land on
   the pad marked "+" on the silkscreen, and U2's tab must face J14; correct
   the rotation in the preview where one is flipped.
5. **Buy the through-hole parts**, soldered by hand either way:

   | Qty | Part |
   |---|---|
   | 2 | 1×7 female header, 2.54 mm (XIAO) |
   | 2 | 1×8 female header (TB6612FNG) |
   | 2 | 1×12 female header (TCA9548A) |
   | 1 | 1×7 female header (MAX98357A) |
   | 1 | 1×6 female header, plus six single female sockets from a breakaway strip for the channel 8–13 PWM pins (PCA9685; how those pins reach the carrier is pre-order check 7) |
   | 2 | 2×7 pin header, 2.54 mm (J1, J2) |
   | 2 | 2×7 IDC plug (female, 2.54 mm), and 14-way 1.27 mm ribbon |
   | 2 | 1×4 pin header (J11, J12) |
   | 3 | 1×2 pin header (J13, J14, J15) |
   | 1 | XT30 PCB socket (XT30PW-M) |
   | 3 | JST-XH 2-pin (B2B-XH-A), with housings and crimps (two motors, piezo) |
   | 4 | JST-XH 4-pin (B4B-XH-A), with housings and crimps (LM2596, two RCWL-1601, OLED) |
   | 4 | M3 screws and standoffs (chassis plate) |
   | 2 | M2 screws, and 2 M2 heat-set inserts (cradle) |
