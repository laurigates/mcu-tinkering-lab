"""A carrier laid out on a pad-per-hole perfboard: drawing and wire list.

    PYTHONPATH=tools python3 -m kicad.perfboard [project ...] [--check] [--allow-none]

A perfboard project has a git-tracked `hardware/perfboard/perfboard.toml`. Its
parts and nets are the carrier netlist (`kicad.netlist`: hardware.toml's
carrier view plus the parts registry), so the perfboard and the carrier PCB
are one circuit. Each part is placed on a 2.54 mm hole grid, each net becomes
a tree of wires on the solder side, routed over free holes (and between holes
where pads are in the way) with splices and optional bare buses, the result is
checked, and four files are written
beside perfboard.toml:

- perfboard.svg        the component side, then one mirrored solder-side panel per build step
- perfboard-print.svg  both sides at 1:1 in mm, with a cut line and corner marks, for printing
- perfboard.md         the wire list, one line per connection, hole to hole
- continuity.md        what must beep after each step, and which rails must not

No KiCad is needed: pad offsets and courtyards are read from the vendored
footprints (`tools/kicad/lib/`).

Where each fact comes from:

- perfboard.toml: board size and title, placement, build steps, wire gauges,
  conductors per hole, buses,
  optional per-ref `[swaps.REF]` footprint/value and `skip` overrides, and the
  `[continuity] isolated` rails;
- the parts registry `[perfboard]` table: the through-hole stand-in footprint
  and value, the height class (low, raised on a socket, tall), and a body
  outline for modules whose footprint does not show it;
- the carrier's board.toml: the heavy nets, as the net class
  `[wires] heavy_class` names.

Override precedence: perfboard.toml, then the registry, then the netlist's
footprint and value. A part with no pins (a mounting hole) is skipped.

Modules: `config` (perfboard.toml), `grid` (hole geometry and names), `place`
(parts on the grid and their checks), `wires` (the plan's union-find check,
numbering by step), `route` (the hole router and its independent
check), `render_svg`, `render_print`, `wirelist`, `continuity`, `__main__` (the
CLI).

Import nothing here: `python3 -m kicad.perfboard` loads `__main__` itself.
"""
