# robocar carrier PCB

> **Before paying for an order**, Lauri does the bench checks in
> [Pre-order checks still owed](#pre-order-checks-still-owed-lauri);
> [`board.toml`](board.toml) marks every unmeasured dimension (#750).

The **main board**: one 100 × 100 mm, 2-layer PCB, screwed to the chassis (no
ready-made plate; the chassis is drilled to match H1–H4), that replaces the
hand-wired harness in [`WIRING.md`](../../WIRING.md). It carries the motor
driver, the PWM driver, the I2C mux, the GPIO expander and its mirror header,
the amplifier, the 3V3 regulator, the pack's power switch, soft latch and USB-C
charger, the power and peripheral connectors, the buzzer and the test headers.

The XIAO ESP32-S3 Sense rides on a **head board** in the pan-tilt kit's top
cradle. The three head variants are a PCB of their own, the
[head panel](../../head/README.md), ordered apart from this one. A 2 × 8,
2.0 mm-pitch IDC ribbon joins J1 here to the fitted head.

Until #1280 (Lauri, 2026-11-25) the heads were a column on this board's right,
sawn off after delivery; the main board needed their 30 mm for the switch,
latch and charger (ADR-038). The main board is now the whole panel and needs no
sawing. Both KiCad projects come from one [`hardware.toml`](../../hardware.toml):
`board.toml` `[project] side = "main"` here, `side = "head"` there
(`tools/kicad/netlist.py`).

KiCad 10 project: [`robocar-carrier/`](robocar-carrier/). Its
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
| J1 | 2×8 pin header, 2.0 mm, unshrouded | main | The main end of the ribbon, in the front row (pins at x 37.5–51.5). Lauri's 10–15 cm ribbon length was for J1's spot beside the old head column, about 22 mm further right, so check it still reaches. The head end is on the [head panel](../../head/README.md) |
| A1 | SparkFun TB6612FNG, in 2 × 1×8 female headers | main | Motor outputs on its left header |
| A2 | Adafruit PCA9685, in female headers on its JP3 side header and its channel 8–13 PWM pins | main | Motor control lines; servo and LED columns stay on the module |
| A3 | Adafruit TCA9548A, or the wider purple TCA9548A/PCA9548A clone, in female headers | main | Every I2C device sits behind it. The clone's rows are one pitch wider, so the upstream (VIN…SC1) side has a second row of holes one pitch toward the rear, each joined to the hole beside it; see [The TCA9548A socket](#the-tca9548a-socket) |
| A4 | Adafruit MAX98357A, in a 1×7 female header | main | The speaker stays on the module's terminal |
| A5 | CJMCU-2317 (MCP23017), in female headers under all 30 holes | main | On mux channel 2, address 0x20; see [The GPIO expander](#the-gpio-expander) |
| J18 | 2×10 pin header, 2.54 mm | main | The expander's GPIOs, in the module's own order; see [The GPIO expander](#the-gpio-expander) |
| U2 | AMS1117-3.3, SOT-223 | main | The main board's 3V3; the XIAO's 3V3 pad stays open |
| D1 | B5819W Schottky, SOD-123 | main | Between the main board's 5V and the ribbon's 5V conductors (net `XIAO_5V`), beside J1: the XIAO's 5V pad is its USB-C VBUS |
| C1–C3 | 470 µF 16 V electrolytic, 8 × 10.5 mm | main | Bulk on the 5V trunk for the PCA9685 V+ and TB6612FNG VM (C1, C2), and beside the MAX98357A Vin (C3) |
| C4–C6 | 100 nF, 0805 | main | PCA9685 VCC, TCA9548A VIN, MCP23017 VCC (C6, beside the CJMCU-2317's socket) |
| C7, C8 | 22 µF 16 V tantalum, 3528 | main | AMS1117 input and output |
| R1 | 100 Ω, 0805 | main | Buzzer series resistor |
| R2–R11 | 2.2 kΩ, 0805 | main | Pull-ups on mux channels 0–4 (R10, R11 on channel 2); see [Pre-fab review](#pre-fab-review-810) |
| R12, R13 | 2.2 kΩ, 0805 | main | Pull-ups on the upstream I2C bus (the XIAO's SDA and SCL), beside C5 in the row behind A3 |
| F1 | 4 A hold PTC fuse, 16 V, 1812 (BSMD1812-400-16V) | main | After the power switch Q2, ahead of everything else on VBAT, at the rear edge between Q2 and J4; see [The power switch](#the-power-switch) |
| Q1, Q2 | DMP3013SFV P-MOSFET, PowerDI3333-8 (C264098) | main | Back to back: Q1 blocks a reversed pack, Q2 is the switch; see [The power switch](#the-power-switch) |
| R14, R15 | 100 kΩ (C149504), 10 kΩ (C17414), 0805 | main | Q1/Q2 gate pull-up and gate resistor |
| Q3, R16 | 2N7002 (C8545), 10 kΩ 0805 (C17414) | main | The latch's hold FET and its HOLD pull-down |
| D2, D3 | SD101AW Schottky, SOD-123 (C3031887) | main | The latch's button-to-switch and button-to-SENSE diodes; 0.2 µA leakage: a B5819W's would pull the off switch's gate toward its threshold through R14 (`soft-latch` block header) |
| R32 | 10 kΩ, 0805 (C17414) | main | `PWR_SENSE` pull-up to 3V3, so D2's leakage does not read as a held button against the MCP23017's weak pull-up |
| J19 | JST-XH 2-pin | main | Power button lead (a chassis push button, either way round) |
| R17, R18, C9 | 100 kΩ (C149504), 22 kΩ (C17560), 100 nF (C49678), 0805 | main | Pack-voltage divider into D3 and its filter |
| J3 | XT30 (AMASS XT30PW-M) | main | Pack in, at the rear edge on the right, mating face to the rear; pad 1 is "−" |
| U5, L1 | BQ25887RGER (VQFN-24, C2761614), 1 µH 1008 inductor (C435392) | main | The pack's USB charger; see [The USB-C charger](#the-usb-c-charger) |
| C10–C19, R19–R31 | 0805, except C13–C15 (22 µF 25 V X7R, 1210, C309062) | main | The charger block's capacitors and resistors (the block's header lists values and LCSC numbers) |
| F2 | 2 A hold PTC fuse, 16 V, 1812 (C883156) | main | The charger's BAT branch, between U5's BAT capacitors and J3's "+" (`CHG_BAT` to `PACK_IN`) |
| C20 | 100 µF 16 V electrolytic, 6.3 × 5.8 mm (Panasonic EEE-FK1C101P, C128532) | main | Damps the pack lead's ring against the BAT branch's ceramics at plug-in; see [The USB-C charger](#the-usb-c-charger) |
| D4, R33 | Red LED 0805 (NCD0805R1, C84256), 1.5 kΩ 0805 (C4310) | main | Charge status on U5's STAT: on while charging, off when done, blinking at 1 Hz on a fault |
| J20 | USB-C receptacle, USB 2.0 16-pin (HRO TYPE-C-31-M-12, C165948) | main | Charge-only port at the right edge; not the XIAO's USB-C, and not connected to its 5V pin |
| J21 | JST-XH 3-pin | main | Pack balance lead: only pin 2, the cells' midpoint, is wired |
| J4 | JST VH 4-pin (B4P-VH), 3.96 mm | main | LM2596 module: IN+, IN−, OUT+, OUT−, at the rear edge turned 180° so IN+ is its right-hand pin, nearest F1; the silkscreen prints "O- O+ I- I+" above the pins, left to right, and "SET LM2596 TO 5.0V FIRST" beside it |
| J5, J6 | JST-XH 2-pin | main | Left and right motor, side by side at A1's motor outputs |
| J7, J8 | JST-XH 4-pin | main | RCWL-1601 front and rear, I2C mode: VCC, SCL, SDA, GND (silkscreen "V C D G"); side by side at the front edge |
| J9 | JST-XH 4-pin | main | SSD1306 0.91" OLED: GND, VCC, SCL, SDA (silkscreen "G V C D") |
| BZ1 | 12 mm buzzer, 7.6 mm pin pitch | main | Soldered to the board; + on pad 1, beside the silkscreen "+" |
| J11–J15 | 1×4 and 1×2 pin headers | main | Test headers, see [Debug headers](#debug-headers) |
| H1–H4 | M3 holes | main | Chassis (drilled to match), at the four corners; H2 and H4 moved from x 65 to x 96 with #1280 |

The per-part values, LCSC numbers and the reasons for each are in
[`hardware.toml`](../../hardware.toml); the pin orders are in the registry entries
it names.

## The three heads

The heads are the [head panel](../../head/README.md#the-three-heads)'s three
variants: where each one puts the ribbon header, and the cradle's M2 holes.
Moving the ribbon to another head moves the robot to it; the main board has a
single end, J1.

## The ribbon survives a reversed plug

The headers are unshrouded 2.0 mm-pitch pin headers, mated by a 2.0 mm IDC plug
on 1.0 mm-pitch flat ribbon. The plug fits either way round, which takes
pin *n* to pin 17 − *n*. The pinout in
[`idc-2x08-xiao-cable.toml`](../../../../../docs/reference/parts/idc-2x08-xiao-cable.toml)
pairs each pin with one it can safely swap with: 5 V lands on 5 V, GND on GND,
SDA on SCL, the pack divider's D3 (the one XIAO input) on the unconnected
pin 9, and every other XIAO output on another main-board input. UART0 TX lands
on the latch's HOLD and idles high, so a reversed plug latches the robot on and
firmware cannot switch it off: unplug the pack. The ribbon grew from 14 to 16 ways for the power
latch's HOLD and the divider (ADR-038, #1270), because the carrier's only spare
XIAO pads, D3 and D7, are on the head and the switch is on the main board. **A reversed plug damages nothing and does not work, and the robot stays on until the pack is unplugged.**
If the robot is dead after reassembly, check the plug orientation first: every
header prints a pin-1 mark.

## The power switch

ADR-038 / #1270. The pack input is the circuit block
[`pack-input-pfet-switched`](../../../../../docs/reference/blocks/pack-input-pfet-switched.toml)
driven by [`soft-latch`](../../../../../docs/reference/blocks/soft-latch.toml)
(both first uses on a board), plus a pack-voltage divider. J3 pin 2 (+) goes to
Q1, then Q2 and F1 to VBAT.

- **Switch.** Q1 and Q2 are DMP3013SFV, back to back, gates tied (`PACK_GATE`).
  The switch carries about 76 µA of gate current and nothing when off. 2S
  ratings against this pack: 30 V and ±25 V VGSS against 8.4 V full and a 16.8 V
  hot-plug ring; F1 is 16 V. VGS = −0.91 × the pack, so −5.6 V at 6.2 V, inside
  the −4.5 V RDS(on) row (DS40141 Rev. 5-2). Channel loss at the 3.4 A stall
  current is about 0.2 W per FET (estimate). The block has no gate clamp, so it
  is 2S only; the balancebot's 3S/4S version is #1273.
- **F1** is the 4 A hold part of #810 (the block's own is 3 A), now after the
  FETs. The 23 mm of `PACK_IN` from J3 to Q1's drain has no fuse ahead of it;
  the pack's BMS covers a short there. The charger's branch off `PACK_IN` has
  its own fuse, F2.
- **Latch.** One button (J19, a chassis push button on a JST-XH lead) closes
  `PWR_BTN` to GND, which turns the pack on. The XIAO boots with the button still
  held, and firmware drives HOLD (D7, GPIO44) high as early as it can; the 10 kΩ
  pull-down R16 keeps it low while the MCU is unpowered, against GPIO44's
  internal pull-up at reset. Driving HOLD low
  switches the robot off once the button is released. A firmware hang with HOLD
  high cannot be undone by the button: unplug the pack. Not measured: how long
  the button has to be held at boot.
- **Button on a lead, not on the board.** The repo's tact switch entry has no
  LCSC number until its JLCPCB rotation is checked by hand, and a chassis button
  is where a person reaches. `PWR_BTN` sits at the pack voltage less one Schottky
  drop while the robot is off (about 76 µA behind it), so the lead has no TVS;
  the `button-input` block's 5 V TVS and 3V3 pull-up cannot be used there.
- **SENSE** (button read-back, for a long-press switch-off) goes to the
  MCP23017's GPB0 (net `PWR_SENSE`, also on J18 pin 6) because no XIAO pad is
  left. A robot without the expander fitted cannot be switched off from the
  button, and the button cannot wake it from deep sleep; the pack-voltage cutoff
  does not need it.
- **Divider.** R17 (100 kΩ, from VBAT after the switch) over R18 (22 kΩ), C9
  100 nF at the ribbon's J1 end, into D3 (GPIO4, ADC1_CH3) via ribbon pin 8: 8.4 V reads
  1.51 V, 6.0 V reads 1.08 V, 69 µA at 8.4 V. The source impedance is 18 kΩ, so
  C9 supplies the sampling charge. The ratio is balancebot's rev3 ratio.

Final nets and pins for the firmware (#1271): `PWR_HOLD` is `POWER_HOLD_PIN`,
GPIO44 (D7, ribbon pin 11); `PACK_SENSE` is `PACK_SENSE_PIN`, GPIO4 (D3, ribbon
pin 8); `PWR_SENSE` is `POWER_BUTTON_EXPANDER_PIN`, MCP23017 GPB0 (pin 8).

**Layout** (#1280). The pack path runs along the rear edge, right to left, all
on F.Cu with no via: J3's "+" to Q1's drain on a fixed 2 mm track, Q1 and Q2
facing each other (common source tied pin to pin, joined across by a 1.0 mm
segment between the two gate pads), Q2's drain into F1 at 2 mm, and F1 into
J4's IN+ at 2 mm. R14 and R15 sit north of the FETs; the gates sit at
opposite corners of the pair, so PACK_GATE reaches Q1's through B.Cu. The latch
(D2, D3, Q3, R16) and the button lead J19 are above J3; the divider (R17, R18,
C9) is at the front, by J1 and J15. `board.toml` gives the reasons for each
fixed track.

## The USB-C charger

ADR-038 / #1272. The circuit block
[`bq25887-2s-usb-charger`](../../../../../docs/reference/blocks/bq25887-2s-usb-charger.toml)
(first use on a board) charges the 2S pack from a 5 V USB-C port on J20 and
balances the cells. The block's header holds the datasheet reasoning, the IC
comparison against the MP2672A, and every number below.

- **The port reaches nothing but the charger.** `CHG_VBUS` joins J20's four
  VBUS pins, U5's VBUS pin and C10, and no rail names it besides. The XIAO's 5V
  pad is its own VBUS (#1252), so the netlist test
  (`RobocarChargerTests`) pins `CHG_VBUS` to those pins. J20's CC1 and CC2 each
  have a 5.1 kΩ pull-down (R30, R31): a sink that asks for 5 V and negotiates
  nothing. D+, D− and SBU are open. J20's shield and GND pins go to GND.
- **BAT reaches `PACK_IN` through F2**, on the pack side of Q1, so the pack
  charges with the robot off, through J3 pin 2 and the pack's BMS. The charge
  current does not go through F1 or the switch FETs.
- **Plug-in ring.** With Q1 and Q2 off, about 86 µF of ceramic sits on
  `CHG_BAT` (C16, C17, and C13–C15 through the charger's sense path). Against
  the pack lead's inductance (about 0.3 µH, assumed) it rings at plug-in, to an
  estimated 10–11 V against BAT's and SNS's 12 V absolute maximum (SLUSD89B
  §7.1). C20, a 100 µF electrolytic whose 0.36 Ω impedance at 100 kHz is near
  the ring's characteristic impedance, damps it. Calculated, not measured:
  scope `PACK_IN` at XT30 insertion on the first board.
- **Charge-status LED.** D4 lights from REGN through R33 (1.5 kΩ) into STAT,
  which is open drain: on while charging, off when the charge ends or is
  disabled, blinking at 1 Hz on any fault (SLUSD89B §6). About 1.9 mA, and
  2.4 mA at most against STAT's 6 mA sink rating (§7.1). REGN is up only while
  the port has power, so D4 is dark with no USB plugged in.
- **Input and charge current.** R19 (820 Ω) caps the input at 1.35 A typical,
  1.56 A at the part's maximum (datasheet constant KILIM). That gives about
  0.83 A into the pack (0.96 A at the maximum), where ADR-038's estimate was
  1 A from 2 A of input. For 2 × 18650 of 2500–3500 mAh that is 3–4.5 hours
  (estimate, not measured).
- **No power path.** With the robot on, the robot's load comes out of the
  charge current first, and the charger cannot see the current fall at the end
  of the cycle, so it holds 4.20 V per cell until its 12 hour timer ends it.
  Charge with the robot off.
- **Balancing.** R21–R24 (4 × 200 Ω in parallel) set about 82 mA, from the
  part's own defaults, with no host. The cells' midpoint comes in on J21 pin 2.
  **J21 pins 1 and 3 are the raw cell terminals and stay unwired**: a wire from
  either to GND or `PACK_IN` would bypass the pack's BMS switch. The pack's BMS
  stays, and both circuits only discharge the higher cell, so they do not
  fight. The BMS's over-charge cutoff must sit above 4.221 V per cell or it
  cuts an ordinary charge; this pack's BMS is not recorded in the repo.
- **No cell temperature sensing.** R27 (10 kΩ) stands in for an NTC the pack
  does not carry, so TS always reads "normal". Only the BMS (if it has an NTC
  input) and the part's own thermal regulation (120 °C) limit a hot charge.
- **Heat.** About 0.45 W in U5 (0.51 W at the maximum) and 0.1 W in L1 at 1.35 A
  input; 16 °C rise at the die at the datasheet's JEDEC θJA of 32.4 °C/W, which
  is a floor for this board. U5's exposed pad (pin 25) needs a ground pour and
  vias. No enclosure exists to assess the case.
- **Sourcing** (kicad.lcsc, 2026-11-24): U5 is extended with 401 in stock, so a
  run of 100 does not fit. J20 (C165948) and L1 are extended. J21 is a loose
  B3B-XH-A (LCSC C144394), through-hole, hand-fitted.

**Layout** (#1280, after SLUSD89B §11.1's priority list, with its Figure 81 as
the pattern). U5 is unturned: its GND, PMID and VBUS pins face north, SW, SNS
and BAT east, the setting pins south. J20 sits flush with the right edge.
Every wide net meets U5's 0.5 mm-pitch pins through a fixed track no wider
than 0.3 mm at the pin (`board.toml` gives the reason for each). Path lengths
below are `just robocar::carrier-measure path` on the committed board:

- SNS: pins 15 and 16 to C13, a 1210 just east of the SNS pins (2.77 mm from
  U5:15), its GND pad at the GND corner, tied 1.0 mm wide to C11's GND; then
  the bulk C14 and C15.
- SW: pins 17 and 18 east under C13's body to L1 beside it (5.59 mm from
  U5:17, 6.99 mm before the second review round), then on to C19.
- PMID: C11 stands straight over the PMID pins, its GND pad over U5's GND pins
  19 and 20, which tie into the exposed pad; the pad takes five 0.5/0.3 mm
  vias. C12 stands beside L1's PMID pad, its GND pad tied to C13's. U5:21 to
  L1:2 is 12.68 mm through both capacitors (18.68 mm before).
- VBUS: pin 23 up beside the PMID capacitors to C10.
- BAT: pins 13 and 14 down to C16 and C17, then on a fixed 1.0 mm F.Cu track
  to F2 and C20, and J3's "+".
- BTST: pin 12 and C19 sit on opposite sides of the BAT and SNS chains, so on
  two layers BTST is fixed tracks through two fixed vias: down south of pin
  12, east on B.Cu under the BAT capacitors, up beside C19 (9.33 mm, 2 vias).
  A shorter loop would put C19 at pin 12 and run SW to it across BAT and SNS,
  on B.Cu.
- No routing on B.Cu under the power stage (U5, the PMID and SNS
  capacitors, L1), except BTST's last stretch to its via by C19, which crosses
  north-south under the SNS track.

The setting resistors stand in a row 2 mm below U5's setting pins, in pin
order, left to right: R19 (ILIM), R20 (MIDSENSE), R24, R23, R22, R21 (CBSET to
MID), then C18 (REGN) below pin 11. The second row holds R27, R26 and R25 (the
TS divider), the I2C pull-ups R28 and R29, and the STAT LED's R33 and D4. Of
these only D4 carries a reference on the silkscreen (2.1 mm pitch; C11, C12
and C16 have none either). CC1 and CC2 run to R30 and R31 north of the
receptacle.

Two of the setting nets are fixed tracks too, because Freerouting left them
unrouted in most routes. CBSET runs from pin 10 along the bottom lane of the
channel under U5, dropping into R21-R24; TS, ILIM and MIDSENSE leave on the
lanes above it. REGN runs straight down from pin 11 to C18, then down the gap
west of C18 and along the lane between the two rows to R25, R28, R29 and R33,
all on F.Cu. C18's GND pad ties to C16's. With the pull-ups west of U5 (the
first round), REGN had to cross the other setting nets on a B.Cu loop that,
with BTST and a routed CHG_BAT, fenced the BAT and REGN capacitors' B.Cu
return off from U5's pad.

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

## The TCA9548A socket

A3 takes either the Adafruit TCA9548A (#2717) or the purple TCA9548A/PCA9548A
clone (Lauri, 2026-11-02). The PCA9548A is NXP's pin-compatible part. The
clone has the same 24 pins in the same order (its top-face silkscreen, on
[the board page](../../../../../docs/reference/boards/adafruit-tca9548a.md#wider-clone)),
but its rows are 17.78 mm apart instead of Adafruit's 15.24 mm. The channel
2–7 row (pads 13–24) stays where it was, and both modules use it. The
upstream row (VIN…SC1, pads 1–12) has a second row of holes one pitch toward
the rear, at y 28.24. Each of those holes has the pad number of the hole in
front of it, so the board joins each pair with copper and the netlist is the
same for both modules. A 2×12 female header on that side takes either
module: Adafruit's pins go in the front row, the clone's in the rear row.

- **The clone's outline is a caliper reading, centred by assumption.** 21.8 ×
  31.2 mm (Lauri, 2026-11-02); the footprint centres it on its holes, because
  the offsets to the edges were not measured. Its courtyard reaches y 30.5 and
  x 32.8. The socket moved 0.3 mm toward the front, so J7's and J8's labels
  are now 0.8 mm. The pull-up row (C5, R12, R13, R2–R9) moved behind it to
  y 32.45 at a 2.9 mm pitch, with 0.8 mm references, and J11 and J12 moved
  0.7 mm right.
- **Nothing on the carrier depends on the clone's own pull-ups or straps.**
  A0–A2 stay unconnected (address 0x70 needs them pulled down on the module)
  and RST is driven by the XIAO. The photo shows six 10 kΩ parts on the clone,
  but which pins they pull has not been traced: see pre-order check 7.
- **Either module can be fitted turned 180°**, which puts VIN on a channel pin.
  "VIN" is printed beside pad 1 for Adafruit's and behind its rear-row twin
  for the clone's.

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
when the USB-C console is unavailable. From the start of `app_main` the
firmware then mirrors every `ESP_LOG` line there too, at the same 115200 8N1
([`main/log_mirror.c`](../../main/log_mirror.c), issue #798), so one adapter
setting reads the boot log and the application log in a single capture. The
console itself stays on the USB-Serial-JTAG on the USB-C connector: command
input, command replies, the `snap`/`mic dump` frames and Improv packets go
there only, because they are written to stdout rather than logged. The mirror
is best effort: a line that does not fit its queue is dropped and counted, and
J15 shows `[log_mirror: N line(s) dropped]` where it happened; a line longer
than `LOG_MIRROR_LINE_MAX` arrives cut, ending in `...`. Nothing on the carrier
drives or loads the line.

## Pre-order checks still owed (Lauri)

Each is a measurement or a part in hand that no file in this repo can settle.
Do them before paying for the order (Lauri, 2026-10-21). Correct
[`board.toml`](board.toml) or the registry entry, then regenerate and
re-check.

1. **The heads' own checks** (the plug against the cradle, the saw kerf) are
   in the [head panel README](../../head/README.md#pre-order-checks-lauri).
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
   "−"), the LM2596 lead (J4 pin 1 IN+, under the silkscreen "I+"), and the
   buzzer (BZ1 "+" on pad 1).
   The LM2596 lead is now a JST VH 4 (3.96 mm): crimp it with SVH-41T-P1.1
   contacts on AWG 18–16 wire into a VHR-4N housing, at least on OUT+ and
   OUT−.
5. **The ribbon's reach** from J1, now about 22 mm further from the head's
   side of the robot than when Lauri sized it (see the J1 row above).
6. **The CJMCU-2317's own bias and pull-ups**
   ([board page checks 3–5](../../../../../docs/reference/boards/cjmcu-2317.md#check-on-the-physical-board)):
   what A0–A2 and RESET are tied to on the module, and whether its two 10 kΩ
   parts are SDA/SCL pull-ups. The carrier's direct ties win either way; the
   check says whether a tie fights a module pull-up (0.33 mA each at 10 kΩ)
   and whether channel 2 ends up with two pull-ups in parallel (about 1.8 kΩ
   with R10 and R11 at 2.2 kΩ, which still sinks under 2 mA).
   The carrier now fits C6, a 100 nF 0805 beside A5's single-column VCC pin
   (WIRING.md's C6), whether or not the unmarked part beside the chip is a
   capacitor (board page check 6), so that check no longer blocks the order.
7. **The TCA9548A clone, if it is the one fitted.** Measure from its VIN hole
   to the nearest end and side edges: the footprint assumes 1.63 mm and
   2.01 mm (its 21.8 × 31.2 mm outline centred on the holes). Check with a
   meter that A0–A2 read low (pull-downs, address 0x70) and that RST is
   pulled up to VIN, because the carrier leaves A0–A2 unconnected.

## Pre-fab review (#810)

This review and the one below were of the four-piece panel before #1280. The
heads have moved to the [head panel](../../head/README.md); the rows about
them (2, 3 and N-1) describe that board now, and the main board's layout has
changed on its right half and rear edge since.

The integrated carrier's adversarial review (#807) found problems every gate
passed. Each was checked against this carrier; `just robocar::carrier-review`
prints the layout figures quoted here.

| # | Finding | Here |
|---|---|---|
| 1 | SMD ground pads reach the B.Cu plane only through thin tracks | **Fixed.** The committed board had one GND via. `[fanout]` in [`board.toml`](board.toml) adds a via beside every SMD GND pad and fails the route if C1, C2, C4, C5, C7, C8 or U2's GND pin gets none. C3's GND pad takes a fixed track to J13's through-hole pad instead, so `carrier-review` lists it as having no via (see [Pre-order review](#pre-order-review-2026-10-21)) |
| 2 | Copper and parts close to the saw cuts | **Fixed for copper:** pours stop 1 mm short of each cut (#811), and head 2's top band now reaches 1 mm past its cut. One head 3 track stays about 0.7 mm from the y 70.5 cut (0.71 mm, HEAD3_UART0_TX, on the committed board; Freerouting picks the net): a wider band there left HEAD3_PIEZO unroutable past J17, and J17 has no room to move. That is above the board's 0.5 mm copper-to-edge clearance, and the saw runs in the waste strip. **Parts:** every SMD part is on the main board, at least 20 mm from a cut. The through-hole parts within 2 mm of a cut, measured from the courtyard, are J17 (0.96 mm from the y 70.5 cut), H7 (1.01 mm, an M2 hole the cradle fixes), J16 (1.56 mm), and A4 and J18 (1.7 mm, socket headers); flex cracks SMD ceramics, and none of these is one |
| 3 | Silkscreen under 1.0 mm | **Fixed where it fits:** R10, R11 and the HEAD 1 and HEAD 2 labels are now 1.0 mm. J18's labels, HEAD 3, the CJMCU-2317 pin names inside its socket and, since the TCA9548A clone's second row (2026-11-02), C5, R2–R9, R12, R13 and J7's and J8's labels stay at 0.8 mm: at 1.0 mm DRC finds them overlapping a neighbour's outline or pads. Text strokes are 0.15 mm or more, JLCPCB's minimum: 40 footprint texts (references, J3's "−" and "+", BZ1's "+") are 0.15 mm, the resized references and `[silk] texts` 0.16 mm. Footprint silkscreen lines are widened to 0.15 mm since `pcb/silk.py` (some were 0.10–0.127 mm) |
| 4 | I2C pull-ups too weak for 400 kHz | **Fixed on the channels:** R2–R11 go from 10 kΩ to 2.2 kΩ (the reasoning is on them in `hardware.toml`). **Upstream** of the mux the breakout's 10 kΩ now has R12 and R13 (2.2 kΩ, fitted) in parallel, about 1.8 kΩ, across the 10–15 cm ribbon (N-4 below). Still probe SDA and SCL on J12 with a scope at the first power-up; if the 30–70 % rise time is over 300 ns, set `I2C_MASTER_FREQ_HZ` to 100 kHz (1000 ns allowed) |
| 5 | No fuse on the pack input | **Fixed:** F1, a 4 A hold, 16 V PTC beside J3 (the 3 A part first fitted derated below the load, SF-4 below), ahead of the LM2596 lead and J11's VBAT pin |
| 6 | No low-battery cutoff | **Fixed on the carrier (ADR-038, #1270; laid out in #1280).** The LM2596's ON/OFF pin (pin 5) is tied to ground on the common module's own board, and J4 carries only IN and OUT, so the carrier cuts the pack input instead: Q1/Q2 are a P-MOSFET switch driven by a push-button soft latch, firmware holds HOLD (D7) high and releases it to switch off, and a 100 kΩ / 22 kΩ divider (R17, R18) puts the switched pack on D3 for the firmware cutoff (#1271). The pack's BMS stays as the backup; see [The power switch](#the-power-switch) |
| 7 | Buck dropout | **Nothing to build.** WIRING.md's figure stands: the LM2596 needs about 1.25 V of headroom at 3 A (SNVS124G, Figure 7-6), so 5 V sags below about 6.3 V of pack |
| 8 | RGB green and blue nearly dark | **Not applicable.** The LEDs are harness parts on the PCA9685's own 3-pin columns, not on the carrier. Their common anode goes to the column's V+ (5 V), so 5 V less a 3.0 V forward drop across the breakout's 220 Ω gives about 9 mA. Check brightness on the bench |
| 9 | Decoupling far from its pins | **Kept.** C4 is 3.6 mm from the PCA9685's VCC pin and C5 5.3 mm from the TCA9548A's VIN (4.5 mm before the clone's second row moved it behind the socket, 2026-11-02), both measured to the socket pin: the breakouts carry their own decoupling at the chip; C6 is 5.5 mm from the CJMCU-2317's VCC pin. C3 is 6.9 mm of track from the MAX98357A's Vin. C1 and C2 are bulk on the 5V trunk, not beside the pins: 52.1 mm of track from the PCA9685's V+ (C1) and 36.8 mm from the TB6612FNG's VM (C2), with no via, because no 8 × 10.5 mm capacitor fits beside either pin (the modules' outlines and J5, J6 surround them). C7 and C8 sit beside U2 |
| 10 | The 5 V current budget | **Documented, not measured.** The LM2596's current limit is 3.4 A minimum over temperature (SNVS124G, § 7.9). Two stalled motors at the simulation's 1.7 A each reach it before the servos, head and amplifier are counted. Measure the real motors' stall current; WIRING.md records the budget |

## Pre-order review (2026-10-21)

An adversarial review of the board on `main` found the problems below; Lauri
chose which to fix. Every "before" figure is from that board, every "after"
from the committed one, measured on the routed copper per layer (only vias and
through-hole pads join the layers).

| # | Finding | Before | After |
|---|---|---|---|
| BL-1 | The board said not to order it | "PROVISIONAL: do not order" on the silkscreen and in `board.toml` | Removed. Lauri's [bench checks](#pre-order-checks-still-owed-lauri) come before paying |
| BL-2 | The ribbon put +5V straight on the XIAO's 5V pad, its USB-C VBUS: a USB cable and the LM2596 feed each other | J1:1 and J1:14 on +5V | D1, a B5819W (C8598), from +5V to a new `XIAO_5V` net on J1's 5V pins, 3.3 mm from J1:14. The XIAO sees 5V less about 0.4 V; USB reverse-biases D1 |
| SF-1 | Every 5V path to the motors, servos and head changed layer through one 1.0/0.5 mm via, the LDO and amplifier through another | Via (42.35, 63.40) alone joined J4 to A1:8, A2:1, C1, C2, C7, J11 and J1; via (49.46, 73.75) to U2 and A4; via (36.41, 89.59) to C3. J4:3 to A1:8 76.2 mm, to A2:1 85.0 mm, to U2:3 30.7 mm, to A4:1 49.1 mm (0.6 mm at the narrowest) | A fixed F.Cu trunk ([`board.toml`](board.toml) `[[tracks]]`), no via: J4:3 to C1 to C2 at 1.5 mm, C2 to A1:8 at 2.0 mm, on to A2:1 at 1.5 mm; the LDO and amplifier branch at 1.0 mm from C2 by C7, U2:3, A4:3 and A4:1 to C3. J4:3 to A1:8 74.4 mm, to A2:1 79.9 mm, to U2:3 63.8 mm, to A4:1 85.2 mm, all 0 vias. The +5V vias left are on the routed feed to D1 and J11 |
| SF-2 | The LM2596 lead was a JST XH, 3 A per contact on AWG 22 (JST XH sheet), under up to about 3.4 A on OUT+ with both motors stalled | XH 4, 2.5 mm | JST VH 4 (B4P-VH, 3.96 mm): 10 A AC/DC on AWG 16, wire AWG 22–16 (JST VH sheet, LCSC C160317's datasheet). Same pin order, registry `lm2596-buck-module-vh4` |
| SF-3 | The TB6612FNG, MAX98357A and TCA9548A fit their sockets turned 180°, which puts a supply on GND | No pin names at the sockets | 1.0 mm names inside each outline: A1 "VM", "PWMA" (pads 8, 9) and "GND" (pads 1, 16); A4 "Vin", "LRC" (pads 1, 7); A3 "VIN", "SC7" (pads 1, 24) |
| SF-4 | F1 derated below the load: BSMD1812-300-16V holds 2.55 A at 40 °C and 2.28 A at 50 °C (BHFUSE BSMD1812 datasheet, Thermal Derating Chart) against 2.4–3.1 A into the LM2596 at full load | 3 A hold, 6 A trip, C883162 | BSMD1812-400-16V, C47263807 (Extended, 15,582 in stock on 2026-10-21): 4 A hold, 8 A trip, 16 V; 3.48 A at 40 °C, 3.16 A at 50 °C. Same 1812 footprint, JLCPCB rotation 0. At 50 °C the 3.1 A full load leaves 2 % below the 3.16 A hold, so a hot robot at full load can trip it; not changed |
| SF-4 (tracks) | F1 guarantees a trip only at 8 A, but PACK_IN and VBAT were routed at 1.0 mm. IPC-2221 puts 1.0 mm of external 1 oz copper at about +32 °C at 4 A and about +155 °C at 8 A (estimates) | PACK_IN J3:2 to F1:1 16.1 mm at 1.0 mm; VBAT F1:2 to J4:1 30.3 mm, 0.75 mm at the narrowest (two segments, 7.2 mm); 0 vias each | Fixed F.Cu `[[tracks]]` at 2.0 mm, no via: PACK_IN from J3:2 down and under the XT30, below its pins, to F1:1 (21.0 mm of centreline); VBAT from F1:2 between A1's GND pins and J3's pads (0.55 mm from each) to J4:1 (32.5 mm). J11's VBAT pin stays on the routed 1.0 mm B.Cu feed from J4:1 (85.7 mm, 1 via), which carries only a meter probe |
| SF-5 | Four claims in the #810 table did not reproduce | C1–C3 "beside" their pins (41.4, 22.7 and 44.1 mm of track); J17 "1.5 mm" from a cut; strokes "0.16 mm or more"; the CPL "carries KiCad's rotations" | C3 moved beside A4's Vin, 6.9 mm of track. C1 and C2 cannot sit beside their pins (no room); the table now gives their trunk distances, 52.1 and 36.8 mm with no via. J17 0.96 mm, strokes 0.15 mm, and the ordering step, corrected above and below |
| SF-6 | Silkscreen under JLCPCB's 0.15 mm, polarity marks too small | Footprint lines 0.10–0.127 mm; the CJMCU-2317 warning at 0.8 mm | Footprint lines widened to 0.15 mm by build_board.py; "+" at 1.0 mm beside C1, C2, C3, C7 and C8 pad 1; "CHIP SIDE UP / LABELS FACE DOWN" at 1.0 mm |
| SF-7 | Off-board leads had no pin names | — | 1.0 mm: J7, J8 "V C D G"; J9 "G V C D"; J4 "I+ I- O+ O-", and "SET LM2596 TO 5.0V FIRST" beside J4 |
| N-1 | Head 3's ground pour was mostly removed as islands | HEAD3_GND 57.7 mm² on F.Cu, 62.7 mm² on B.Cu | A fixed B.Cu ring round head 3 from U4's GND pin to J17:13 and an F.Cu track down its right side, plus `[pour] stitch`: 95.1 mm² on F.Cu, 452.1 mm² on B.Cu. Stitching put 12 vias on head 1, 13 on head 2, and one on head 3, whose F.Cu outer fill is still fenced off by routed tracks; an F.Cu ring as well left three head 3 signals unroutable |
| N-4 | Upstream I2C rise time: the breakout's 10 kΩ alone on the bus from the ribbon | No carrier pull-ups; SDA 78.6 mm and SCL 54.6 mm of track, plus the ribbon | R12, R13, 2.2 kΩ (C17520), fitted, beside C5: about 1.8 kΩ with the breakout's. SDA 115.8 mm and SCL 63.7 mm of track, the pull-ups' included, on the board of that review; the 2026-11-02 re-route for the TCA9548A clone has not been re-measured |
| N-9 | The CJMCU-2317's own decoupling is unverified | None | C6, 100 nF 0805 (C49678), fitted, 5.5 mm from A5's VCC pin |

The repo has no do-not-fit convention: every carrier part is in the BOM and
CPL, so R12, R13 and C6 are fitted.

`carrier-review` on the committed board (#1280, after the second pre-fab
review round) reports seven flags, none gating:

- C10 (VBUS) sits 4.3 mm from U5:23, and C19 6.8 mm from U5:12 and 8.2 mm
  from U5:17 (the bootstrap loop above).
- C4, C5 and C6 are 3.6, 5.3 and 5.5 mm from their modules' pins (each module
  decouples at its chip).
- C6:2 and J20's four GND pads have no via within 2.5 mm. J20's pads tie to
  its through-hole shield, which joins both layers, and join the pour solid
  (`[pour] solid_pads`).

The main board has 229 GND vias (stitching, fanout and the vias route_board
puts in pockets of fill), and `carrier-measure plane` finds one GND plane
component of 5 mm² or more.

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
- The SMD parts carry LCSC numbers, so JLCPCB can place them (see
  [Ordering](#ordering)); by hand, any 470 µF ≥ 16 V 8 × 10.5 mm SMD
  electrolytic and 22 µF ≥ 16 V 3528 tantalum fit.

## Separating the pieces

Nothing to separate: since #1280 the main board is the whole panel. The head
panel is sawn into its three heads; see its
[README](../../head/README.md#separating-the-heads).

## Regenerate and check

```sh
just robocar::carrier-schematic  # schematic + library tables from hardware.toml (no KiCad)
just robocar::carrier-board      # place from scratch, autoroute (Freerouting), pour
just robocar::carrier-check      # ERC, DRC + schematic parity, netlist parity, board check,
                                         # then the check stamp CI verifies
just robocar::carrier-fab        # carrier-check, then Gerbers, drill, BOM and CPL
```

The tool requirements, and why the board is built by script, are in
[the balancebot carrier README](../../../../robotics/balancebot/hardware/carrier/README.md#regenerate-and-check)
and the `carrier-pcb` skill. Freerouting does not route the same way twice:
commit the board you checked.

## Ordering

Lauri does the [pre-order checks](#pre-order-checks-still-owed-lauri) before
paying for the order.

1. **Start from the committed board.** `git status` must show nothing under
   `hardware/carrier/`.
2. **Generate the files:** `just robocar::carrier-fab`. It stops if any
   carrier gate fails or anything that feeds the board has uncommitted
   changes, and writes `<sha7> <pcb8>` (the commit and the stamp's
   board-hash prefix) on the silkscreen under the title, on a plotting copy
   of the board (`board.toml` `[fab] provenance_at`). A
   `CARRIER_FAB_ALLOW_DIRTY=1` run writes `<sha7>*<pcb8>`, the same width,
   and `-dirty` in the file names; never order from it. Outputs, under
   `robocar-carrier/output/`, named for the commit's first 7 hex
   digits:

   | File | For |
   |---|---|
   | `robocar-carrier-fab-<sha7>.zip` | The board: Gerbers and drill files |
   | `assembly/robocar-carrier-bom-jlc-<sha7>.csv` | Assembly only: the SMD parts with LCSC numbers |
   | `assembly/robocar-carrier-cpl-jlc-<sha7>.csv` | Assembly only: their positions and rotations |
   | `robocar-carrier-fab-<sha7>.csv` | The hand-off row: the commit and the zip's sha256 |

3. **Upload the zip** at jlcpcb.com. It should detect 2 layers and
   100 × 100 mm. FR-4, 1.6 mm, 1 oz copper (the track widths assume it), HASL.
   In the Gerber preview check the outline and the `<sha7> <pcb8>` text under
   the title (it must match the zip's name).
4. **Assembly (optional):** *PCB Assembly*, *Economic*, top side, with the two
   CSVs. Fourteen LCSC numbers are Extended, so fourteen feeder fees: C1–C3
   (C96179), C7–C8 (C8020), F1 (C47263807), F2 (C883156), Q1–Q2 (C264098, 237
   in stock), D2–D3 (C3031887), U5 (C2761614, 371 in stock), L1 (C435392), J20
   (C165948), C13–C15 (C309062), C19 (C107154), C20 (C128532), R25 (C17739)
   and R26 (C21841); the rest are Basic or Preferred (`python3 -m kicad.lcsc
   --project packages/robotics/robocar`, 2026-11-26). JLCPCB's stock and part status were checked on
   2026-10-16, the 2.2 kΩ pull-ups (C17520) on 2026-10-18, and F1, D1, C6,
   R12 and R13 on 2026-10-21 (the comments in `hardware.toml` record the
   figures). The CPL's rotations already include each LCSC part's offset
   from [`jlcpcb-rotations.toml`](../../../../../docs/reference/jlcpcb-rotations.toml)
   (`kicad.jlc`; U2 goes out at 270°, KiCad's 90° plus 180°). **Verify
   polarity in the placement preview** before confirming: each "+" on the pad
   marked "+" on the silkscreen (C1–C3, C7, C8), D1's cathode bar on its pad toward the rear edge (pad 1, `XIAO_5V`),
   and U2's tab toward C7.
5. **Buy the through-hole parts**, soldered by hand either way:

   | Qty | Part |
   |---|---|
   | 2 | 1×8 female header (TB6612FNG) |
   | 1 + 1 | 1×12 and 2×12 female header, 2.54 mm (TCA9548A: the 2×12 on the upstream side takes either module) |
   | 1 | 1×7 female header (MAX98357A) |
   | 1 + 1 | 2×10 and 1×10 female header, 2.54 mm, or three 1×10 (the CJMCU-2317's 30 holes) |
   | 1 | 1×10 male pin header, 2.54 mm (the CJMCU-2317's third column: the module ships with two 1×10 strips) |
   | 1 | 2×10 male pin header, 2.54 mm (J18) |
   | 1 | 1×6 female header, plus six single female sockets from a breakaway strip for the channel 8–13 PWM pins (PCA9685; how those pins reach the carrier is pre-order check 3) |
   | 1 | 2×8 pin header, 2.0 mm pitch (J1; the head's is on the head panel's list) |
   | 2 | 2×8 IDC plug (female, 2.0 mm pitch), and 16-way flat ribbon at 1.0 mm pitch, 10–15 cm per cable (Lauri, 2026-10-16; 2×7 and 14-way before ADR-038) |
   | 2 | 1×4 pin header (J11, J12) |
   | 3 | 1×2 pin header (J13, J14, J15) |
   | 1 | XT30 PCB socket (XT30PW-M) |
   | 1 | JST VH 4-pin header B4P-VH (C160317 at LCSC), a VHR-4N housing and four SVH-41T-P1.1 crimps for AWG 20–16 (J4, the LM2596 lead) |
   | 3 | JST-XH 2-pin (B2B-XH-A), with housings and crimps (two motors, J19 the power button) |
   | 1 | JST-XH 3-pin (B3B-XH-A, C144394), with a housing and one crimp for pin 2 (J21, the pack balance lead's midpoint) |
   | 1 | Panel push button, momentary, normally open, for J19 |
   | 1 | 12 mm board-mount buzzer, 7.6 mm pin pitch (BZ1) |
   | 3 | JST-XH 4-pin (B4B-XH-A), with housings and crimps (two RCWL-1601, OLED) |
   | 4 | M3 screws and standoffs (chassis) |
