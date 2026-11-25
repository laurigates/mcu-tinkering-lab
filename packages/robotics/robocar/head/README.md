# robocar head panel PCB

The three head boards that carry the XIAO ESP32-S3 Sense in the pan-tilt kit's
top cradle: one 29.5 × 100 mm, 2-layer PCB, sawn into three 29.5 × 29.5 mm
heads after delivery. Each head is a XIAO socket and the head end of the 2 × 8
ribbon; they differ only in where the ribbon header sits. Fit the XIAO to the
one that suits the cradle.

The heads were a column on the right of the
[main board's panel](../hardware/carrier/README.md) until #1280 (Lauri,
2026-11-25), when the main board needed their area for the power switch, soft
latch and USB charger (ADR-038). The layout is that column's, moved 70.5 mm
left: x = 0 here was x = 70.5 there.

This directory holds no firmware and no `hardware.toml` of its own. The KiCad
project, [`hardware/carrier/robocar-head/`](hardware/carrier/robocar-head/), is
the head side of the parent's [`hardware.toml`](../hardware.toml): its
[`board.toml`](hardware/carrier/board.toml) says `[project] side = "head"` and
`join = ".."`, and `tools/kicad/netlist.py` keeps the XIAO sockets (U1, U3,
U4), the ribbon headers (J2, J16, J17) and the cable's `head_parts`, the M2
holes (H5–H10). Never edit the `.kicad_sch` or `.kicad_pcb` in KiCad.

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

Each head's copy of the XIAO socket is wired to its own header (nets `HEAD_*`,
`HEAD2_*`, `HEAD3_*`). The main board has a single end, J1, so moving the
ribbon to another head moves the robot to it. The ribbon's pinout and why a
reversed plug is harmless are in the main board's
[README](../hardware/carrier/README.md#the-ribbon-survives-a-reversed-plug).

## Layout changes since the shared panel (#1280)

- **The left edge is milled now, not sawn**, so copper comes to 0.5 mm of it,
  1 mm closer than beside the old vertical cut.
- **The 2 × 8 headers** (ADR-038) are 2 mm longer than the 2 × 7 ones, at the
  same pin 1. J16 sits 1 mm left of its old position.
- **Signal tracks are 0.2 mm.** The headers' 1.35 mm pads at 2.0 mm pitch
  leave 0.65 mm between pins: a 0.2 mm track passes there with 0.2 mm
  clearance each side, a 0.3 mm one does not. At 0.3 mm head 2 left one or two
  signals unrouted in 25 attempts. The heads' signals are I2C, I2S, UART0 and
  slow control lines.

## Pre-order checks (Lauri)

1. **The header behind or in front of the head.** For each head you mean to
   use, check the IDC plug and the ribbon's bend clear the cradle and the pan
   servo. On head 2 (header on the front) also fit the XIAO in its socket and
   push the plug onto J16: the plug is wider than the header and stands about
   as tall as the socketed XIAO, so it may touch the XIAO or only leave toward
   the bottom edge. This can wait for the plugs: heads 1 and 3 do not depend
   on it.
2. **Saw kerf against the bands.** A blade wider than about 1 mm leaves no bare
   margin beside the cut line in a 3 mm band.

## Separating the heads

Every cut is straight across the panel, so no cut ends in an inside corner and
none needs a relief hole.

1. **Buy a jeweller's saw frame with fine blades** (or a coping saw with a fine
   blade).
2. **Clamp the panel** flat, the cut line just past the bench edge.
3. **Cut the panel into its three heads** along the four horizontal lines. Each
   head's lines are its own edges; keep the kerf in the waste strips between
   heads (y 29.5–35.25 and 64.75–70.5), where the "CUT" labels are.
4. **Sand each head back to its lines, 29.5 × 29.5 mm,** on sandpaper on a flat
   surface, checking with calipers. The cradle's inner size is 30.0 × 30.0 mm.

FR-4 dust contains glass fibre: wet-sand, and wear a mask. The heat-set
inserts go in the cradle, not the PCB: H5–H10 are plain 2.2 mm clearance holes.

## Regenerate and check

```sh
just robocar-head::carrier-schematic  # schematic + library tables (no KiCad)
just robocar-head::carrier-board      # place from scratch, autoroute, pour
just robocar-head::carrier-check      # ERC, DRC + parity, netlist parity, board check, stamp
just robocar-head::carrier-fab        # carrier-check, then Gerbers and drill files
```

A change to the parent's `hardware.toml` regenerates both this project and the
main board's; run both checks.

## Ordering

1. **Generate the files** with `just robocar-head::carrier-fab` from a clean
   tree. The panel has no SMD parts, so there is no assembly BOM to use.
2. **Upload the zip** at jlcpcb.com: 2 layers, 29.5 × 100 mm, FR-4, 1.6 mm,
   1 oz, HASL. The order selects high-precision silkscreen (the 0.8 mm
   HEAD 3 label); `carrier-fab` prints the reminder. In the Gerber preview
   check the four "CUT" lines and J2's and J17's outlines on the bottom
   silkscreen.
3. **Buy the parts** for the one head fitted:

   | Qty | Part |
   |---|---|
   | 2 | 1×7 female header, 2.54 mm (the XIAO's socket) |
   | 1 | 2×8 pin header, 2.0 mm pitch (J2, J16 or J17) |
   | 2 | M2 button-head screws, ISO 7380 (or any head of 3.5 mm diameter or less), and 2 M2 heat-set inserts (cradle) |

   The ribbon and its plugs are on the main board's list.
