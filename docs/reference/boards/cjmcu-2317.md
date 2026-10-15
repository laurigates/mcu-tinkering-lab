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

> **Not bench-verified.** Every position on this page comes from the
> listing's six product photos. Nothing was read from a board file or checked
> with a continuity meter. `.claude/rules/board-layout-from-vendor-files.md`
> does not count photos as authoritative. CJMCU publishes no board file, and
> no schematic for this board turned up online. Do the checks under
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

**Inner vs outer is a reading-order assumption.** On the back face each row
reads `label, inner hole, outer hole`, so this page maps the first name to the
inner hole and the second to the outer one. Under that reading the outer
column (board edge) holds GND, INTA and GPA0–7, and the inner column holds
VCC, INTB and GPB0–7. The photos cannot resolve the traces on either face
well enough to confirm it.

`Side`/`Pos` follow [`tools/hardware/layout.py`](../../../tools/hardware/layout.py):
`L`/`R` edge, `Pos 1` at the top. `Row` says which column a left-edge pad is in.
`layout.py` has no way to express two columns on one edge. The repeated `L`
positions are therefore deliberate: `parse_layout` rejects this table with a
"claimed by both" error and does not draw a wrong picture. Do not add a `board =`
key for this page to `hardware.toml` until the layout model can represent an
inner row and the order has been checked on a board.

| Pin | Side | Pos | Row | Silkscreen (back) | Notes |
|-----|------|-----|-----|-------------------|-------|
| GND | L | 1 | outer | `VCC/GND` | Assumed. Check that it beeps to R9 |
| VCC | L | 1 | inner | `VCC/GND` | Assumed. Check that it beeps to R10 |
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

- **Pitch.** 2.54 mm is assumed, not measured. The listing ships two
  10-pin male header strips (photos 1–3 and 6). The inner and outer columns
  sit one row pitch apart in photos 1 and 5. Two 10-pin strips fill only 20 of
  the 30 holes.
- **Outline.** Scaled against a 2.54 mm pitch, photo 1 gives roughly
  21 × 26 mm. That figure comes from a hand-held photo and is not a
  dimension to place against.
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
2. **Inner vs outer.** Check whether the left-column row-1 holes beep to the
   right column's `GND` (R9) or `VCC` (R10). Then check whether the outer or
   the inner row-3 hole goes to SSOP pin 21 (GPA0) or pin 1 (GPB0).
   Correct the `Row` column if it is the other way round.
3. **Address pins.** Measure each of A0, A1 and A2 to VCC and to GND. If they read
   about 10 kΩ to VCC, the module answers at 0x27 unless the carrier ties them
   to GND. If they float, the carrier must tie them (DS20001952D p. 11).
4. **RESET.** Confirm a pull-up to VCC (about 10 kΩ expected). If none is
   fitted, the carrier must hold RESET high.
5. **I2C pull-ups.** Check whether the two 10 kΩ parts sit on SDA/SCL to VCC.
   The TCA9548A breakout has no pull-ups on its downstream channels: in
   Adafruit's board file R1/R2 pull up the upstream SDA/SCL only, and every
   `nSDA`/`nSCL` net joins just the chip and the header pad
   ([`adafruit-tca9548a.md`](adafruit-tca9548a.md)). If these two parts are not
   SDA/SCL pull-ups, the carrier must pull channel 2 up.
6. **Decoupling.** Confirm the unmarked part is a capacitor across VDD/VSS. If
   so, the optional C6 in `WIRING.md` is redundant.
7. **Pitch and outline.** Measure the header pitch, the inner-to-outer column
   spacing, the board outline and the hole diameters.

## Source

The order was read from the six photos on AliExpress listing
1005008592833333 (TENSTAR ROBOT, retrieved 2026-10-14). Photos 3, 4 and 6 show the
back-face silkscreen. Photos 1, 2, 5 and 6 show the component face and the chip
marking. No vendor board file or schematic was found, so `tools/breakout-pinout.py`
does not apply. The chip-side cross-check uses Microchip DS20001952D, p. 1
(Package Types) and p. 11 (Table 2-1).

Other boards sold as "CJMCU-2317" or "MCP23017 module" are not covered by this
page unless they carry the same silkscreen in the same order.
