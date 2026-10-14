# Balancebot perfboard prototype

The [carrier PCB](../carrier/) laid out on a 50 × 70 mm pad-per-hole proto
board (18 × 24 holes), for building a prototype before ordering boards. It is
the same circuit: parts and nets come from the carrier netlist that
`hardware.toml` and the [parts registry](../../../../../docs/reference/parts/)
give (the one the carrier schematic is written from), so the firmware pin map
applies unchanged.

- [`perfboard.svg`](perfboard.svg): the component side, then one mirrored
  solder-side panel per build step (power, logic power, motor coils, signals).
  Each panel shows only that step's wires, coloured per net with a legend, and
  numbered as in the wire list. Wires are maze-routed along the channels
  between holes, so no wire is drawn over a pad it does not connect to, and no
  two wires in a step share a channel.
- [`perfboard.md`](perfboard.md): the wire list. Tick each line off as you
  solder it.

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
| H1–H4 | M3 holes | Use the proto board's own corner holes |

## Build order

1. **Sockets first:** 2 × 1×7 female headers for the XIAO (columns A and G,
   rows 1–7), 4 × 1×8 for the DRV8825s, 1 × 1×8 for the GY-521 (row 1, I–P).
   Fit the modules while soldering the sockets so the rows stay parallel.
2. **Low parts** in the middle column: R1, R2, R3, C3. They sit between the
   drivers, not under them.
3. **D1:** the cathode band goes on the square pad, K23.
4. **C1, C2:** the + lead (longer) goes on the square pad.
5. **Headers** along the bottom edge.
6. **Wires**, one step at a time: `perfboard.md` lists them in the same four
   steps and numbers as the drawing, with the gauge for each. Heavy nets
   (VBAT, GND, motor coils) need 20 AWG or heavier and are drawn thicker. A
   length-1 wire can be a solder bridge.
7. **Check before power:** continuity on every GND pin, and no continuity
   between VBAT, 3V3 and GND. Then plug in the LM2596 only after setting it to
   5.0–5.3 V on the bench.

## Things a perfboard changes

- **The GY-521 lies flat on its socket**, over the area to the right of the
  XIAO, with its X arrow toward row 1 (forward). Nothing is under it.
- **A2 faces the same way as A1**, so its motor pins are on its left (column L)
  and the right-motor wires cross to J5. That keeps both VMOT pins at the
  bottom (B17 and L17), each three holes above its bulk cap's + pad (C20, L20).
- The three pin orders still to confirm before the PCB order (#748) matter here
  too: battery lead polarity, the DRV8825 rows and the GY-521 header.

## Regenerate

```sh
just balancebot::perfboard          # rewrite perfboard.svg and perfboard.md
just balancebot::perfboard-check    # fail if they are stale or the layout breaks
```

Both run the shared generator in
[`tools/kicad/perfboard/`](../../../../../tools/kicad/perfboard/)
(`python3 -m kicad.perfboard`). It needs no KiCad, and CI runs the check
(`carrier-check.yml`). What it reads:

- [`perfboard.toml`](perfboard.toml): board size, where each part sits, the four
  build steps and their nets, and the wire gauges;
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
