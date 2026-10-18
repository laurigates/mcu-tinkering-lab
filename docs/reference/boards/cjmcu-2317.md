# CJMCU-2317 MCP23017 Breakout

Breakout for the MCP23017 16-bit I2C I/O expander. robocar-unified uses it as
optional GPIO expansion on TCA9548A channel 2 (`I2C_BUS_CHANNEL_MCP23017`,
`MCP23017_ADDR` in `packages/robocar/unified/main/pin_config.h`). For the
silicon see [`io--mcp23017.md`](../datasheets/io--mcp23017.md).

| | |
|---|---|
| Silkscreen | `CJMCU-2317` on the component face, `CJMCU` on the back |
| Chip marking | `MCP23017` / `E/SS` / `2049M1U`: a 28-lead SSOP (DS20001952D p. 1, p. 26) |
| Seller | TENSTAR ROBOT, AliExpress item **1005008592833333**, <https://www.aliexpress.com/item/1005008592833333.html> |
| Also called | "1953W" in #662. That marking appears on none of the six listing photos. |

> **Partly bench-verified.** The pin order was read from the listing's six
> product photos. Lauri checked the hole geometry and which column of the
> double row is inner and which outer on the fitted module on 2026-10-16
> (see [Pitch and outline](#pitch-and-outline)). The outline, the on-board
> parts and their roles are still read from photos only.
> `.claude/rules/board-layout-from-vendor-files.md` does not count photos as
> authoritative. CJMCU publishes no board file, and no schematic for this
> board turned up online. Do the open checks under
> [Check on the physical board](#check-on-the-physical-board) before a carrier
> is ordered.

## Physical layout

**Viewed from the component side** with the chip text upright and
`CJMCU-2317` along the bottom edge. The board has three columns of
through-holes, ten rows each:

- **Right edge:** one column of control and power pads.
- **Left edge:** two columns, one pitch apart. *Outer* is at the board edge
  and *inner* sits one pitch in, towards the chip.

The labels are printed only on the back. Turning the board over to read them
swaps the columns: the double column appears on the right and the single
column on the left. Each back-face label in the double column (`B0/A0`)
covers both holes in its row.

The single column's order is read straight from the back-face silkscreen. It
also agrees with the chip: with the pin-1 dot at the lower left, A0–A2 and
RESET (SSOP pins 15–18) sit at the chip's top right, and VDD/VSS/SCL/SDA
(pins 9–13) at its bottom right.

**Inner vs outer, verified by Lauri on 2026-10-16.** In every row of the
double column the first-named signal is on the inner hole and the
second-named on the outer one: row 1 is VCC inner and GND outer, row 2 INTB
inner and INTA outer, and rows 3–10 GPBn inner and GPAn outer. The outer
column (board edge) therefore holds GND, INTA and GPA0–7, and the inner
column holds VCC, INTB and GPB0–7. This is the reading this page assumed from
the back-face silkscreen before the check.

`Side`/`Pos` follow [`tools/hardware/layout.py`](../../../tools/hardware/layout.py):
`L`/`R` edge, `Pos 1` at the top. `Row` says which column a left-edge pad is in.
`layout.py` has no way to express two columns on one edge. The repeated `L`
positions are therefore deliberate: `parse_layout` rejects this table with a
"claimed by both" error and does not draw a wrong picture. Do not add a `board =`
key for this page to `hardware.toml` until the layout model can represent an
inner row. The order is now checked on a board, but `parse_layout` still has
no inner row, so the caveat stands for that reason alone. The carrier socket
takes its pad positions from `tools/kicad/lib/breakout-sockets/sockets.toml`
(`pad_xy`) instead of from this table.

| Pin | Side | Pos | Row | Silkscreen (back) | Notes |
|-----|------|-----|-----|-------------------|-------|
| GND | L | 1 | outer | `VCC/GND` | Verified (Lauri, 2026-10-16) |
| VCC | L | 1 | inner | `VCC/GND` | Verified (Lauri, 2026-10-16) |
| INTA | L | 2 | outer | `ITB/ITA` | Interrupt, PORTA |
| INTB | L | 2 | inner | `ITB/ITA` | Interrupt, PORTB |
| GPA0 | L | 3 | outer | `B0/A0` | Driver pin 0 |
| GPB0 | L | 3 | inner | `B0/A0` | Driver pin 8 |
| GPA1 | L | 4 | outer | `B1/A1` | |
| GPB1 | L | 4 | inner | `B1/A1` | |
| GPA2 | L | 5 | outer | `B2/A2` | |
| GPB2 | L | 5 | inner | `B2/A2` | |
| GPA3 | L | 6 | outer | `B3/A3` | |
| GPB3 | L | 6 | inner | `B3/A3` | |
| GPA4 | L | 7 | outer | `B4/A4` | |
| GPB4 | L | 7 | inner | `B4/A4` | |
| GPA5 | L | 8 | outer | `B5/A5` | |
| GPB5 | L | 8 | inner | `B5/A5` | |
| GPA6 | L | 9 | outer | `B6/A6` | |
| GPB6 | L | 9 | inner | `B6/A6` | |
| GPA7 | L | 10 | outer | `B7/A7` | Output only (DS20001952D p. 11) |
| GPB7 | L | 10 | inner | `B7/A7` | Output only (DS20001952D p. 11) |
| A2 | R | 1 | single | `A2` | Address bit 2 |
| A1 | R | 2 | single | `A1` | Address bit 1 |
| A0 | R | 3 | single | `A0` | Address bit 0 |
| RESET | R | 4 | single | `RESET` | Active low |
| NC | R | 5 | single | `NC/SO` | SSOP pin 14. Unused on the MCP23017 (SO on the MCP23S17) |
| NC | R | 6 | single | `NC/CS` | SSOP pin 11. Unused on the MCP23017 (CS on the MCP23S17) |
| SDA | R | 7 | single | `SDA/SI` | I2C data |
| SCL | R | 8 | single | `SCL/SCK` | I2C clock |
| GND | R | 9 | single | `GND` | |
| VCC | R | 10 | single | `VCC` | 1.8–5.5 V. robocar-unified feeds it 3.3 V |

### Pitch and outline

Measured by Lauri on the fitted module, 2026-10-16:

- **Row pitch** 2.54 mm in all three columns.
- **Outer to inner column** one pitch, 2.54 mm, centre to centre.
- **Inner to single column** 15.24 mm (6 pitches), centre to centre.

So, from the component side with the outer column's row-1 hole (GND) at the
origin, +x to the right and +y down: the outer column is at x 0, the inner at
x 2.54 and the single column at x 17.78, and row *n* of each is at
y 2.54 × (*n* − 1), 0 to 22.86. The carrier socket
(`CJMCU_2317_Socket`) lists exactly these positions.

- **Outline, estimated, not measured:** the outline 21 × 26 mm, scaled from
  hand-held photo 1 against the 2.54 mm pitch. The socket footprint centres
  it on the holes for its courtyard. It is not a dimension to place against
  without checking the module against the board render.
- The listing ships two 10-pin male header strips (photos 1–3 and 6), which
  fill only 20 of the 30 holes.
- **Bottom corners.** In photo 1 the two bottom-corner holes (outer `B7/A7`
  and `VCC`) look slightly different from the rest. They may be square pin-1
  pads, or it may be lighting. Not confirmed.

### On-board parts (seen, not traced)

| Part | Marking | Where | Likely role (unverified) |
|------|---------|-------|--------------------------|
| 4-element resistor array | `103` (4 × 10 kΩ) | Top right, above the chip's pins 15–19 | Bias for A0, A1, A2 and RESET |
| Two chip resistors | `103` (10 kΩ). The photo shows it rotated as `E01` | Bottom right, below the chip | SDA/SCL pull-ups |
| One unmarked chip part, capacitor-like | none | Beside the two resistors | VDD decoupling |

The array's role is an inference from where it sits. RESET has to be held
high for the chip to run at all, so if the array's common side is one rail it
must be VCC. Then A0–A2 would also be pulled **high**, giving address
**0x27**, not the 0x20 the firmware uses. The photos can show neither
the array's common net nor which pins it lands on. Measure them.

## Check on the physical board

Check with the module unpowered, using a continuity or resistance meter, before
a carrier is ordered:

1. **Identity.** Confirm the fitted module is this one: `CJMCU-2317` on the
   component face and an `MCP23017` (not `MCP23S17`) SSOP. #662 called it
   "1953W". Find out where that marking is, or whether it is a different board.
2. **Inner vs outer.** Done: verified by Lauri, 2026-10-16 (first-named
   signal on the inner hole).
3. **Address pins.** Measure each of A0, A1 and A2 to VCC and to GND. The
   robocar-unified carrier ties all three straight to GND, which sets 0x20
   whatever the module does; a 10 kΩ pull-up on the module would then draw
   0.33 mA per pin and nothing else. On a harness the bias decides the
   address: about 10 kΩ to VCC means 0x27, floating means the harness must tie
   them (DS20001952D p. 11).
4. **RESET.** Confirm a pull-up to VCC (about 10 kΩ expected). The carrier
   ties RESET straight to 3V3; on a harness, if none is fitted, the wiring
   must hold RESET high.
5. **I2C pull-ups.** Check whether the two 10 kΩ parts sit on SDA/SCL to VCC.
   The carrier fits its own 2.2 kΩ pull-ups on channel 2; if the module has
   them too, the pair is about 1.8 kΩ per line.
   The TCA9548A breakout has no pull-ups on its downstream channels: in
   Adafruit's `Adafruit TCA9548A.brd` at commit `c5bdfe4c` (the file
   [`adafruit-tca9548a.md`](adafruit-tca9548a.md) is drawn from), R1/R2 pull
   up the upstream SDA/SCL only, and every `nSDA`/`nSCL` net joins just the
   chip and the header pad. If these two parts are not
   SDA/SCL pull-ups, the carrier must pull channel 2 up.
6. **Decoupling.** Confirm the unmarked part is a capacitor across VDD/VSS. If
   so, the optional C6 in `WIRING.md` is redundant.
7. **Pitch and outline.** The pitch and both column spacings are done
   (Lauri, 2026-10-16). Still open: the board outline and the hole
   diameters. Hold the module against the carrier render to check the
   outline sits inside the socket's courtyard.

## Source

The hole geometry and the inner/outer assignment are Lauri's measurement on
the fitted module, 2026-10-16. The order was read from the six photos on
AliExpress listing
1005008592833333 (TENSTAR ROBOT, retrieved 2026-10-14). Photos 3, 4 and 6 show the
back-face silkscreen. Photos 1, 2, 5 and 6 show the component face and the chip
marking. No vendor board file or schematic was found, so `tools/breakout-pinout.py`
does not apply. The chip-side cross-check uses Microchip DS20001952D, p. 1
(Package Types) and p. 11 (Table 2-1).

Other boards sold as "CJMCU-2317" or "MCP23017 module" are not covered by this
page unless they carry the same silkscreen in the same order.
