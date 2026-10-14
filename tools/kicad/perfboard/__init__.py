"""A carrier laid out on a pad-per-hole perfboard: drawing and wire list.

    PYTHONPATH=tools python3 -m kicad.perfboard [project ...] [--check] [--allow-none]

A perfboard project has a git-tracked `hardware/perfboard/perfboard.toml`. Its
parts and nets are the carrier netlist (`kicad.netlist`: hardware.toml's
carrier view plus the parts registry), so the perfboard and the carrier PCB
are one circuit. Each part is placed on a 2.54 mm hole grid, each net becomes
point-to-point wires on the solder side, every wire is maze-routed through the
channels between holes, the result is checked, and two files are written
beside perfboard.toml:

- perfboard.svg  the component side, then one mirrored solder-side panel per build step
- perfboard.md   the wire list, one line per connection, hole to hole

No KiCad is needed: pad offsets and courtyards are read from the vendored
footprints (`tools/kicad/lib/`).

Where each fact comes from:

- perfboard.toml: board size and title, placement, build steps, wire gauges,
  and optional per-ref `[swaps.REF]` footprint/value and `skip` overrides;
- the parts registry `[perfboard]` table: the through-hole stand-in footprint
  and value, the height class (low, raised on a socket, tall), and a body
  outline for modules whose footprint does not show it;
- the carrier's board.toml: the heavy nets, as the net class
  `[wires] heavy_class` names.

Override precedence: perfboard.toml, then the registry, then the netlist's
footprint and value. A part with no pins (a mounting hole) is skipped.

Modules: `config` (perfboard.toml), `grid` (hole geometry and names), `place`
(parts on the grid and their checks), `wires` (wire plan, its union-find
check, numbering by step), `route` (the channel router and its independent
check), `render_svg`, `wirelist`, `__main__` (the CLI).

Import nothing here: `python3 -m kicad.perfboard` loads `__main__` itself.
"""
