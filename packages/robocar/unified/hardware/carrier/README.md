# robocar-unified carrier PCB

> **PROVISIONAL: do not order yet.** The pre-order checks below are still owed.
> The silkscreen says so too, and [`board.toml`](board.toml) marks every
> unmeasured dimension (#750).

One 100 × 100 mm, 2-layer PCB that carries four pieces and replaces the
hand-wired harness in [`WIRING.md`](../../WIRING.md):

- the **main board**, x 0–70.5 mm, screwed to the chassis (no ready-made plate; the chassis is drilled to match H1–H4): the motor driver,
  the PWM driver, the I2C mux, the GPIO expander and its mirror header, the
  amplifier, the 3V3 regulator, the power and peripheral connectors, the
  buzzer and the test headers;
- three **head boards** in a column on its right, each 29.5 × 29.5 mm, for the
  pan-tilt kit's top cradle: a XIAO ESP32-S3 Sense socket and the head end of
  the ribbon. They differ only in where the ribbon header sits (see
  [The three heads](#the-three-heads)); fit the XIAO to the one that suits the
  cradle.

Copper-free bands of 3 mm or more under silkscreen saw lines ("CUT", on both
faces) separate the pieces. JLCPCB makes one rectangle with no V-cuts, tabs or
slots; the pieces are sawn apart after delivery (see
[Separating the pieces](#separating-the-pieces)). A 2 × 7, 2.0 mm-pitch IDC
ribbon joins the main board to the fitted head.

KiCad 10 project: [`robocar-unified-carrier/`](robocar-unified-carrier/). Its
schematic is generated from [`hardware.toml`](../../hardware.toml) and the parts
registry ([`docs/reference/parts/`](../../../../../docs/reference/parts/)) by
`tools/kicad`
([ADR-027](../../../../../docs/decisions/ADR-027-kicad-carrier-generation.md));
the board is placed and routed from the same netlist and
[`board.toml`](board.toml), which holds every position, the bands and the
silkscreen. Never edit the `.kicad_sch` or `.kicad_pcb` in KiCad; change those
files and regenerate (see [Regenerate and check](#regenerate-and-check)).

## What is on it

Each module sits in female headers or plugs into a connector, so any of them can
be swapped without desoldering. The passives and the regulator are SMD on the
top side; everything else is through-hole.

| Ref | Part | Board | Notes |
|---|---|---|---|
| U1, U3, U4 | XIAO ESP32-S3 Sense socket, two 1×7 female headers | heads 1, 2, 3 | USB-C toward the head's top edge; fit one XIAO |
| J2, J16, J17 | 2×7 pin header, 2.0 mm, unshrouded | heads 1, 2, 3 | The head end of the ribbon; see [The three heads](#the-three-heads) |
| H5–H10 | M2 holes, 2.2 mm, ISO 7380 courtyard | heads 1, 2, 3 | Through the cradle into heat-set inserts; M2 button heads only |
| J1 | 2×7 pin header, 2.0 mm, unshrouded | main | The main end of the ribbon, in the front row; its pins are 21–33 mm in from the main board's right edge. Lauri's 10–15 cm ribbon length was for J1's earlier spot beside the head column, so check it still reaches |
| A1 | SparkFun TB6612FNG, in 2 × 1×8 female headers | main | Motor outputs on its left header |
| A2 | Adafruit PCA9685, in female headers on its JP3 side header and its channel 8–13 PWM pins | main | Motor control lines; servo and LED columns stay on the module |
| A3 | Adafruit TCA9548A, in 2 × 1×12 female headers | main | Every I2C device sits behind it |
| A4 | Adafruit MAX98357A, in a 1×7 female header | main | The speaker stays on the module's terminal |
| A5 | CJMCU-2317 (MCP23017), in female headers under all 30 holes | main | On mux channel 2, address 0x20; see [The GPIO expander](#the-gpio-expander) |
| J18 | 2×10 pin header, 2.54 mm | main | The expander's GPIOs, in the module's own order; see [The GPIO expander](#the-gpio-expander) |
| U2 | AMS1117-3.3, SOT-223 | main | The main board's 3V3; the XIAO's 3V3 pad stays open |
| C1–C3 | 470 µF 16 V electrolytic, 8 × 10.5 mm | main | Bulk at the PCA9685 V+, TB6612FNG VM and MAX98357A Vin |
| C4, C5 | 100 nF, 0805 | main | PCA9685 VCC, TCA9548A VIN |
| C7, C8 | 22 µF 16 V tantalum, 3528 | main | AMS1117 input and output |
| R1 | 100 Ω, 0805 | main | Buzzer series resistor |
| R2–R11 | 2.2 kΩ, 0805 | main | Pull-ups on mux channels 0–4 (R10, R11 on channel 2); see [Pre-fab review](#pre-fab-review-810) |
| F1 | 3 A hold PTC fuse, 16 V, 1812 | main | Between J3 and everything else, beside J3 |
| J3 | XT30 (AMASS XT30PW-M) | main | Pack in; pad 1 is "−" |
| J4 | JST-XH 4-pin | main | LM2596 module: IN+, IN−, OUT+, OUT− |
| J5, J6 | JST-XH 2-pin | main | Left and right motor, side by side at A1's motor outputs |
| J7, J8 | JST-XH 4-pin | main | RCWL-1601 front and rear, I2C mode: VCC, SCL, SDA, GND; side by side at the front edge |
| J9 | JST-XH 4-pin | main | SSD1306 0.91" OLED: GND, VCC, SCL, SDA |
| BZ1 | 12 mm buzzer, 7.6 mm pin pitch | main | Soldered to the board; + on pad 1, beside the silkscreen "+" |
| J11–J15 | 1×4 and 1×2 pin headers | main | Test headers, see [Debug headers](#debug-headers) |
| H1–H4 | M3 holes | main | Chassis (drilled to match) |

The per-part values, LCSC numbers and the reasons for each are in
[`hardware.toml`](../../hardware.toml); the pin orders are in the registry entries
it names.

## The three heads

All three have the same outline, the same two M2 holes in the top corners
(22.5 mm apart, the positions Lauri confirmed on 2026-10-16), and the XIAO with
its USB-C toward the top edge, which points the Sense camera forward. The
cradle's sides carry clips, so the header sits at the top or the bottom edge,
never at a side (Lauri, 2026-10-16). Only one head gets a XIAO; the other two
stay spare.

| Head | Refs | Header | XIAO | For |
|---|---|---|---|---|
| 1 | U1, J2, H5, H6 | Back face, bottom edge | Between the holes, above the header | The plug and ribbon behind the head, leaving the bottom edge |
| 2 | U3, J16, H7, H8 | Front face, bottom edge | Between the holes, above the header | The plug in front, below the camera, if nothing behind the head clears it |
| 3 | U4, J17, H9, H10 | Back face, top edge, between the holes | Below the holes | The ribbon leaving over the top, if the bottom edge is blocked |

The holes use KiCad's ISO 7380 M2 footprint, whose courtyard is a 3.5 mm button
head: on heads 1 and 2 the XIAO socket sits 0.26 mm from it on each side. Fit
M2 button-head screws (ISO 7380) or another head no larger than 3.5 mm.

Each head's spare copy of the XIAO socket is wired to its own header (nets
`HEAD2_*`, `HEAD3_*`); the main board has a single end, J1. Moving the ribbon
to another head moves the robot to it.

## The ribbon survives a reversed plug

The headers are unshrouded 2.0 mm-pitch pin headers, mated by a 2.0 mm IDC plug
on 1.0 mm-pitch flat ribbon. The plug fits either way round, which takes
pin *n* to pin 15 − *n*. The pinout in
[`idc-2x07-xiao-cable.toml`](../../../../../docs/reference/parts/idc-2x07-xiao-cable.toml)
pairs each pin with one it can safely swap with: 5 V lands on 5 V, GND on GND,
SDA on SCL, UART0 TX on the unconnected pin 9, and every other XIAO output on
another main-board input. **A reversed plug damages nothing and does not work.**
If the robot is dead after reassembly, check the plug orientation first: every
header prints a pin-1 mark.

## The GPIO expander

The CJMCU-2317 module (A5) sits in female headers under all 30 of its holes,
so it plugs in firmly and every pin reaches the board. It is turned 180°: its
single column (I2C and supply) faces the mux, its double column of GPIOs faces
the right edge. Hole positions and which column is which are Lauri's
measurement of the fitted module (2026-10-16), recorded on
[its board page](../../../../../docs/reference/boards/cjmcu-2317.md).
It fits upside down too, which shorts 3V3: fit it chip side up (see
[Assembly notes](#assembly-notes)). The silkscreen inside its socket outline
names every hole, and "CHIP SIDE UP" is printed below it.

- **Address 0x20, hard-strapped.** A0, A1 and A2 go straight to GND, which is
  `MCP23017_ADDR` in [`main/pin_config.h`](../../main/pin_config.h). A direct
  tie overrides whatever bias the module itself puts on those pins.
- **RESET tied high.** RESET goes straight to 3V3, so the expander always runs;
  the firmware has no reset line for it.
- **Bus.** SDA and SCL go to TCA9548A channel 2 (`I2C_BUS_CHANNEL_MCP23017`),
  pulled up to 3V3 by R10 and R11 like the other channels. The two NC pins stay
  unconnected.
- **Mirror header J18.** A 2×10 pin header beside the module repeats its double
  column in the same physical order and orientation, for jumper wires:

  | Row | Column B (inner) | Column A (outer, board edge) |
  |---|---|---|
  | 1 (bottom) | 3V3 | GND |
  | 2 | INTB | INTA |
  | 3–10 | GPB0–GPB7 | GPA0–GPA7 |

  The silkscreen prints "B" and "A" over the top end and "3V3" and "GND" under
  row 1. The nets are `EXP_GPA0`…`EXP_GPB7`, `EXP_INTA` and `EXP_INTB`. GPA7
  and GPB7 are outputs only (MCP23017 datasheet DS20001952D, p. 11).

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

1. **The header behind or in front of the head.** For each head you mean to
   use, check the IDC plug and the ribbon's bend clear the cradle and the pan
   servo. On head 2 (header on the front) also fit the XIAO in its socket and
   push the plug onto J16: the XIAO's edge is 0.58 mm from the header body, the
   plug is wider than the header and stands about as tall as the socketed
   XIAO, so it may touch the XIAO or only leave toward the bottom edge. This
   can wait for the plugs: it does not block ordering, because heads 1 and 3
   do not depend on it and head 2 is the variant being tried.
2. **RCWL-1601 mode resistors.** Check each rangefinder has 10 kΩ on R4 (I2C
   mode) and R5 empty.
3. **Hold each fitted breakout against the board render** (TB6612FNG,
   PCA9685, TCA9548A, MAX98357A, CJMCU-2317): headers on the footprints,
   outline inside its courtyard. The CJMCU-2317's outline (about 21 × 26 mm)
   is an estimate from a listing photo; its hole positions are measured.
   With the module seated, push a 2.54 mm jumper housing onto J18's column B
   (the column next to the module): by the estimated outline the module's
   edge is about 0.9 mm from the housing, so a larger module stops it.
4. **Polarity** of the XT30 pack lead (J3 pad 1 is "−", next to the silkscreen
   "−"), the LM2596 lead (J4 pin 1 IN+), and the buzzer (BZ1 "+" on pad 1).
5. **Saw kerf against the bands.** A blade wider than about 1 mm leaves no bare
   margin beside the cut line in a 3 mm band.
6. **The CJMCU-2317's own bias and pull-ups**
   ([board page checks 3–5](../../../../../docs/reference/boards/cjmcu-2317.md#check-on-the-physical-board)):
   what A0–A2 and RESET are tied to on the module, and whether its two 10 kΩ
   parts are SDA/SCL pull-ups. The carrier's direct ties win either way; the
   check says whether a tie fights a module pull-up (0.33 mA each at 10 kΩ)
   and whether channel 2 ends up with two pull-ups in parallel (about 1.8 kΩ
   with R10 and R11 at 2.2 kΩ, which still sinks under 2 mA).
   Also check the unmarked part beside the chip is a capacitor across VDD and
   VSS (board page check 6). The carrier fits no decoupling capacitor of its
   own (WIRING.md's C6) and relies on that part: decoupling belongs at the
   chip, and a carrier capacitor would sit beyond a socket contact and the
   module's traces, beside the AMS1117's 22 µF output capacitor on the same
   rail. If the part is not a capacitor, add a 100 nF 0805 at A5's
   single-column VCC before ordering.

## Pre-fab review (#810)

The integrated carrier's adversarial review (#807) found problems every gate
passed. Each was checked against this carrier; `just robocar-unified::carrier-review`
prints the layout figures quoted here.

| # | Finding | Here |
|---|---|---|
| 1 | SMD ground pads reach the B.Cu plane only through thin tracks | **Fixed.** The committed board had one GND via. `[fanout]` in [`board.toml`](board.toml) adds a via beside every SMD GND pad and fails the route if C1–C5, C7, C8 or U2's GND pin gets none |
| 2 | Copper and parts close to the saw cuts | **Fixed for copper:** pours stop 1 mm short of each cut (#811), and head 2's top band now reaches 1 mm past its cut. One track, HEAD3_MOTOR_STBY, stays 0.73 mm from the y 70.5 cut: a wider band there left HEAD3_PIEZO unroutable past J17, and J17 has no room to move. That is above the board's 0.5 mm copper-to-edge clearance, and the saw runs in the waste strip. **Parts:** every SMD part is on the main board, at least 20 mm from a cut. The through-hole parts within 2 mm of a cut are H7 (1.0 mm, an M2 hole the cradle fixes), J17 (1.5 mm), J16 (1.6 mm), and A4 and J18 (1.7 mm, socket headers); flex cracks SMD ceramics, and none of these is one |
| 3 | Silkscreen under 1.0 mm | **Fixed where it fits:** R2–R11 and the HEAD 1 and HEAD 2 labels are now 1.0 mm. MUX_RST, J18's labels, HEAD 3 and the CJMCU-2317 pin names inside its socket stay at 0.8 mm: at 1.0 mm DRC finds them overlapping a neighbour's outline or pads. Strokes are 0.16 mm or more |
| 4 | I2C pull-ups too weak for 400 kHz | **Fixed on the channels:** R2–R11 go from 10 kΩ to 2.2 kΩ (the reasoning is on them in `hardware.toml`). **Upstream** of the mux the only pull-ups are the TCA9548A breakout's 10 kΩ, across the 10–15 cm ribbon: probe SDA and SCL on J12 with a scope at the first power-up. If the 30–70 % rise time is over 300 ns, set `I2C_MASTER_FREQ_HZ` to 100 kHz (1000 ns allowed) |
| 5 | No fuse on the pack input | **Fixed:** F1, a 3 A hold, 16 V PTC beside J3, ahead of the LM2596 lead and J11's VBAT pin |
| 6 | No low-battery cutoff | **Not fixed on the carrier.** The LM2596's ON/OFF pin (pin 5) turns it off above about 1.3 V (TI SNVS124G, § 6), but the common module ties it to ground on its own board, and J4 carries only IN and OUT. Use a pack with a BMS, whose cell undervoltage cutoff ends the discharge; WIRING.md says so |
| 7 | Buck dropout | **Nothing to build.** WIRING.md's figure stands: the LM2596 needs about 1.25 V of headroom at 3 A (SNVS124G, Figure 7-6), so 5 V sags below about 6.3 V of pack |
| 8 | RGB green and blue nearly dark | **Not applicable.** The LEDs are harness parts on the PCA9685's own 3-pin columns, not on the carrier. Their common anode goes to the column's V+ (5 V), so 5 V less a 3.0 V forward drop across the breakout's 220 Ω gives about 9 mA. Check brightness on the bench |
| 9 | Decoupling far from its pins | **Kept.** C4 is 3.6 mm from the PCA9685's VCC pin and C5 4.5 mm from the TCA9548A's VIN, both measured to the socket pin: the breakouts carry their own decoupling at the chip. C1–C3 sit beside the V+, VM and Vin pins they serve, and C7 and C8 beside U2 |
| 10 | The 5 V current budget | **Documented, not measured.** The LM2596's current limit is 3.4 A minimum over temperature (SNVS124G, § 7.9). Two stalled motors at the simulation's 1.7 A each reach it before the servos, head and amplifier are counted. Measure the real motors' stall current; WIRING.md records the budget |

## Assembly notes

- **The CJMCU-2317 goes in chip side up.** Solder its male pins so they leave
  the board on the side away from the chip: the chip faces up, away from the
  carrier, and the printed pin labels on the module's back face the carrier.
  Upside down (flipped top to bottom) it still fits the socket, with its rows
  reversed, and that puts its VCC on GND and SDA and A2 on 3V3: the 3V3 rail
  then feeds through the MCP23017's protection diodes. Before seating it,
  compare the carrier's silkscreen inside A5 ("VCC GND" at the end nearest
  J18's "3V3 GND", "A2" … "VCC" beside the single column) with the labels on
  the module's back. The silkscreen shows the chip side, so the back's labels
  read mirrored left to right, but row by row they must match: the
  `VCC/GND` row at the "VCC GND" end.
- **Set the LM2596 to 5.0 V on the bench before connecting it to J4.** The
  module's output is adjustable and arrives set to whatever the factory left.
- **The heat-set inserts go in the cradle, not the PCB.** H5–H10 are plain
  2.2 mm clearance holes for M2 button-head screws.
- The SMD parts carry LCSC numbers, so JLCPCB can place them (see
  [Ordering](#ordering)); by hand, any 470 µF ≥ 16 V 8 × 10.5 mm SMD
  electrolytic and 22 µF ≥ 16 V 3528 tantalum fit.

## Separating the pieces

Every cut is straight across the board or across the head column, so no cut
ends in an inside corner and none needs a relief hole.

1. **Buy a jeweller's saw frame with fine blades** (or a coping saw with a fine
   blade).
2. **Clamp the board** flat, the cut line just past the bench edge.
3. **Cut the head column off** along the vertical line at x 70.5, in the band
   on the main board's side of the line, the full 100 mm.
4. **Cut the column into its three heads** along the four horizontal lines.
   Each head's lines are its own edges; keep the kerf in the waste strips
   between heads (y 29.5–35.25 and 64.75–70.5), where the "CUT" labels are.
5. **Sand each head back to its lines, 29.5 × 29.5 mm,** on sandpaper on a flat
   surface, checking with calipers. The cradle's inner size is 30.0 × 30.0 mm.

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
   In the Gerber preview check the outline, the five "CUT" lines, and J2's
   and J17's outlines on the bottom silkscreen.
4. **Assembly (optional):** *PCB Assembly*, *Economic*, top side, with the two
   CSVs. C1–C3 (C96179), C7–C8 (C8020) and F1 (C883162) are Extended parts,
   so three feeder fees; the rest are Basic. JLCPCB's stock and part status
   were checked on 2026-10-16, F1 and the 2.2 kΩ pull-ups (C17520) on
   2026-10-18 (the comments in `hardware.toml` record the figures). **Check
   polarity in the placement preview** before confirming: the CPL carries
   KiCad's rotations, and a community rotation table
   ([JLCKicadTools `cpl_rotations_db.csv`](https://github.com/matthewlai/JLCKicadTools/blob/master/jlc_kicad_tools/cpl_rotations_db.csv))
   lists a 180° correction for all three footprints involved: `CP_Elec_8x10.5`
   (C1–C3), `CP_EIA-3528` (C7, C8) and `SOT-223` (U2). Each "+" must land on
   the pad marked "+" on the silkscreen, and U2's tab must face C7, toward
   the front edge; correct the rotation in the preview where one is flipped.
5. **Buy the through-hole parts**, soldered by hand either way:

   | Qty | Part |
   |---|---|
   | 2 | 1×7 female header, 2.54 mm (XIAO, for the one head fitted) |
   | 2 | 1×8 female header (TB6612FNG) |
   | 2 | 1×12 female header (TCA9548A) |
   | 1 | 1×7 female header (MAX98357A) |
   | 1 + 1 | 2×10 and 1×10 female header, 2.54 mm, or three 1×10 (the CJMCU-2317's 30 holes) |
   | 1 | 1×10 male pin header, 2.54 mm (the CJMCU-2317's third column: the module ships with two 1×10 strips) |
   | 1 | 2×10 male pin header, 2.54 mm (J18) |
   | 1 | 1×6 female header, plus six single female sockets from a breakaway strip for the channel 8–13 PWM pins (PCA9685; how those pins reach the carrier is pre-order check 3) |
   | 2 | 2×7 pin header, 2.0 mm pitch (J1 and the fitted head's J2, J16 or J17) |
   | 2 | 2×7 IDC plug (female, 2.0 mm pitch), and 14-way flat ribbon at 1.0 mm pitch, 10–15 cm per cable (Lauri, 2026-10-16) |
   | 2 | 1×4 pin header (J11, J12) |
   | 3 | 1×2 pin header (J13, J14, J15) |
   | 1 | XT30 PCB socket (XT30PW-M) |
   | 2 | JST-XH 2-pin (B2B-XH-A), with housings and crimps (two motors) |
   | 1 | 12 mm board-mount buzzer, 7.6 mm pin pitch (BZ1) |
   | 4 | JST-XH 4-pin (B4B-XH-A), with housings and crimps (LM2596, two RCWL-1601, OLED) |
   | 4 | M3 screws and standoffs (chassis) |
   | 2 | M2 button-head screws, ISO 7380 (or any head of 3.5 mm diameter or less), and 2 M2 heat-set inserts (cradle) |
