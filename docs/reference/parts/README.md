# Parts registry

One TOML file per physical part a carrier PCB can mount. A project's
`hardware.toml` names an entry with `part = "<slug>"` (the file stem), on a
`[parts.<id>]` table or on `[source]` for the MCU. `tools/kicad` reads the
entry to turn the pin names `hardware.toml` uses, such as a breakout's
silkscreen names (`drv_left.STEP`, `imu.SDA`), into a KiCad symbol, a footprint
and symbol pin numbers. The result is the carrier netlist
(`PYTHONPATH=tools python3 -m kicad.netlist <project>`).

Two parts on the same connector can need two entries. The pin-*name* map
belongs to the module on the cable, not to the connector, so the LM2596 lead
and the stepper lead are separate entries on the same JST-XH 4.

## Format

```toml
[part]
name        = "..."                # required, display
source      = "..."                # required: where the pin order came from
symbol      = "Lib:Name"           # required; vendored under tools/kicad/lib/
footprint   = "Lib:Name"           # required; vendored
value       = "..."                # optional default Value
lcsc        = "C123"               # optional default LCSC (JLCPCB) number
no_connect  = ["FAULT"]            # optional: pin names allowed to stay unconnected
pin_numbers = "board-dip-ccw"      # optional; replaces [pins] (MCU sockets)

[pins]                             # pin name -> symbol pin number(s): int, str or list
GND = [1, 7]

[perfboard]                        # optional; the perfboard layout's stand-in
footprint = "Lib:Name"             # through-hole stand-in; default: [part] footprint
value     = "..."                  # default: the instance's value
height    = "low"                  # "low" | "raised" | "tall"; default "low"
body      = [x0, y0, x1, y1]       # outline in holes from pin 1; default: F.CrtYd box
```

A `hardware.toml` instance can override `value` and `lcsc`. Without a `value`
anywhere, the instance's `name` is used.

Every LCSC number, here or on an instance, needs a checked JLCPCB placement
rotation for its footprint in
[`../jlcpcb-rotations.toml`](../jlcpcb-rotations.toml): `kicad.fab` refuses to
write a CPL without one, and `test_jlc` fails on any LCSC number in use that
has none. `PYTHONPATH=tools python3 -m kicad.lcsc_rotation <C123> <Lib:Name>`
derives the entry.

`pin_numbers = "board-dip-ccw"` takes the numbers from the part's board page
under `docs/reference/boards/`: the MCU's `[source] board`, or the part's own
`board`. The page's Side/Pos columns give the number, counter-clockwise like a
DIP. Side L position *n* is pin *n*, and Side R position *n* is pin 2N+1−*n*,
where N is the number of pads per side. A pad whose row has a GPIO may stay off
every net, as a spare I/O. A power pad (5V, GND, 3V3) must be on a rail.

## Where a pin order comes from

`source` must say where the pin order came from: a vendor board file, the KiCad
library, the fitted part's silkscreen (`gy-521`), or memory plus the issue that
tracks checking it.
A recalled pin order reads exactly like a checked one. For a breakout, take it
from the vendor's board file, as
[`.claude/rules/board-layout-from-vendor-files.md`](../../../.claude/rules/board-layout-from-vendor-files.md)
explains. A part with a vendor-sourced board page should derive its entry from
that page, as the XIAO socket does with `pin_numbers`. The robocar carrier will
do the same with its breakout pages (follow-up to #750).

## What `tools/kicad` checks

`tools/kicad/registry.py` validates every entry against the vendored library,
in the unit tests and in CI:

- unknown tables and keys are rejected;
- `symbol`, `footprint` and `[perfboard] footprint` resolve under
  `tools/kicad/lib/`; vendor a new one with `just kicad::vendor`;
- every symbol pin number is mapped exactly once, and is a pad of both
  footprints;
- a `[pins]` name that is also a symbol pin's name maps to that pin;
- the symbol has one unit and no De Morgan body, and no hidden `power_in` pins;
- every `no_connect` name is a pin, and a `no_connect`-type symbol pin is listed;
- **polarity**: every `"+"`/`"-"`, `"K"`/`"A"` on a part with a `"K"`, and
  `"G"`/`"D"`/`"S"` on a part with all three (a MOSFET), is proved. The proof is the symbol's own pin name (`D_Schottky` names K and A),
  the footprint's silkscreen (the XT30 prints "−" beside pad 1), or a pinned
  convention in `registry.CONVENTIONS` (`C_Polarized` pin 1 is "+"). An
  unproved pin fails. A reversed electrolytic or diode would otherwise agree
  with the netlist, the board and DRC alike.

`tools/kicad/test_registry.py` holds a failing mutation for each rule.
