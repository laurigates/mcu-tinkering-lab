# Balancebot perfboard prototype

The [carrier PCB](../carrier/) laid out on a 50 × 70 mm pad-per-hole proto
board (18 × 24 holes), for building a prototype before ordering boards. It is
the same circuit: parts and nets come from the carrier netlist that
`hardware.toml` and the [parts registry](../../../../../docs/reference/parts/)
give (the one the carrier schematic is written from), so the firmware pin map
applies unchanged.

- [`perfboard.svg`](perfboard.svg): the component side, then one mirrored
  solder-side panel per build step (pack input, power, logic power, motor
  coils, signals).
  Each panel shows only that step's wires, coloured per net with a legend, and
  numbered as in the wire list. Wires are maze-routed along the channels
  between holes, so no wire is drawn over a pad it does not connect to, and no
  two wires in a step share a channel.
- [`perfboard-print.svg`](perfboard-print.svg): both sides at 1:1, in
  millimetres, for printing at 100 %. Check the 50 mm bar with a ruler first.
  The red hole-grid outline is a cut line for a placement sticker, and the
  crosshairs sit on the four corner holes, so the print lines up with the
  board. Wire numbers are only in `perfboard.svg`; they do not fit at 2.54 mm.
- [`perfboard.md`](perfboard.md): the wire list. Tick each line off as you
  solder it.
- [`continuity.md`](continuity.md): what must beep after each step, and the
  rail pairs that must not before first power-up.

Holes are named from the **component side**: columns A–R left to right, rows
1–24 top to bottom, with robot forward toward row 1. The solder-side drawing is
mirrored so it matches what you see when the board is flipped; the hole names
are not.

## Differences from the PCB

| Ref | On the PCB | On the perfboard |
|---|---|---|
| J1 | JST-XH 2-pin (2.5 mm) | Plain 1×2 2.54 mm male header for the pack's JST-XH lead, same pin order |
| J2, J4, J5 | JST-XH (2.5 mm) | Plain 1×4 2.54 mm male headers, same pin order |
| C1, C2 | 6.3 mm SMD electrolytic | 8 mm radial, 2.5 mm leads, ≥ 25 V |
| C3, R1–R3 | 0805 SMD | Ceramic disc; resistors stood upright |
| D1 | B5819W (SOD-123) | 1N5819, the same diode in DO-41, stood upright |
| D2 | BZT52C15 15 V zener (SOD-123) | 1N4744A, a 15 V zener in DO-41, stood upright |
| F1 | 3 A hold PTC, 1812 SMD | Radial PTC, 3 A hold, ≥ 16 V, leads bent to 2.54 mm |
| Q1 | ZXMP4A16G (SOT-223) | A TO-220 P-FET with the same G, D, S pin order and VGS ≥ ±20 V (IRF9540N, IRF4905) |
| R4 | 0805 SMD | Resistor stood upright |
| H1–H4 | M3 holes | Use the proto board's own corner holes |

## Build order

1. **Sockets first:** 2 × 1×7 female headers for the XIAO (columns A and G,
   rows 1–7), 4 × 1×8 for the DRV8825s, 1 × 1×8 for the GY-521 (row 7, I–P).
   Fit the modules while soldering the sockets so the rows stay parallel.
2. **Low parts** in the middle column: R1, R2, R3, C3. They sit between the
   drivers, not under them. Then the pack input below A1: Q1 in column F
   (G at F18, D at F19, S at F20), F1 at H19–H20 and R4 at J19–J20.
3. **D1 and D2:** each cathode band goes on the square pad: D1's at K23,
   D2's at F16 (D2's anode is at F15, above it).
4. **C1, C2:** the + lead (longer) goes on the square pad.
5. **Headers** along the bottom edge.
6. **Wires**, one step at a time: `perfboard.md` lists them in the same five
   steps and numbers as the drawing, with the gauge for each. Heavy nets
   (the pack input, VBAT, GND, motor coils) need 20 AWG or heavier and are drawn thicker. A
   length-1 wire can be a solder bridge.
7. **Check before power:** work through `continuity.md`, which lists every
   net's pads and the rail pairs (VBAT, XIAO_5V, +3V3, GND) that must read
   open. Then plug in the LM2596 only after setting it to 5.0–5.3 V on the
   bench.

## Things a perfboard changes

- **The GY-521 lies flat on its socket**, component side up, over the area to
  the right of the XIAO. The socket is on row 7 with VCC at I7, so the module
  extends toward row 1 and its X arrow points forward. Nothing is under it.
- **A2 faces the same way as A1**, so its motor pins are on its left (column L)
  and the right-motor wires cross to J5. That keeps both VMOT pins at the
  bottom (B17 and L17), each three holes above its bulk cap's + pad (C20, L20).
- The three pin orders still to confirm before the PCB order (#748) matter here
  too: battery lead polarity, the DRV8825 rows and the GY-521 header.

## Regenerate

```sh
just balancebot::perfboard          # rewrite the drawings, wire list and checklist
just balancebot::perfboard-check    # fail if they are stale or the layout breaks
```

Both run the shared generator in
[`tools/kicad/perfboard/`](../../../../../tools/kicad/perfboard/)
(`python3 -m kicad.perfboard`). It needs no KiCad, and CI runs the check
(`carrier-check.yml`). What it reads:

- [`perfboard.toml`](perfboard.toml): board size, where each part sits, the
  build steps and their nets, the wire gauges, and the rails the checklist
  pairs;
- the parts registry's `[perfboard]` tables: each part's through-hole stand-in
  and its value, and whether it is raised on a socket or tall
  (the "Differences from the PCB" table above);
- the carrier's [`board.toml`](../carrier/board.toml): the heavy nets are its
  `power` net class;
- pad offsets and courtyards from the vendored KiCad footprint files
  (`tools/kicad/lib/`), never typed in.

It refuses to write anything if two parts share a hole, a pad falls off the
board, a tall part sits under a raised module, or the wires leave any net
split or join two nets. That last check is a union-find over the generated
wire list, separate from the code that plans the wires.

### Routing

Each wire is routed on a half-pitch grid. Holes sit on even coordinates and the
channels between them on odd ones. A wire leaves its pad by a half-pitch stub
and then stays in the channels. The router is Dijkstra with costs for length,
bends and crossings (`STEP_COST`, `BEND_COST`, `CROSS_COST`). It routes each
step in `ORDER_TRIES` wire orders (shortest-first, then seeded shuffles) and
keeps the cheapest, so a run is repeatable. A separate `check_routes` re-walks
every path and fails if a wire jumps, ends on the wrong hole, passes over a
hole, shares a channel with another wire in its step, or turns where another
wire runs. Wires in different steps are not checked against each other: each
panel is drawn on its own, and insulated wires may cross on the board.

Wire-list lengths are measured along the route, so a wire between two holes in
one row is 3, not 2: it goes around the hole between them.
