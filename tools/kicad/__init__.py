"""KiCad carrier boards generated from the hardware join (#750).

    <project>/hardware.toml ─┐ hardware.join(view=CARRIER)
    docs/reference/parts/    ─┼─> netlist ─> .kicad_sch, board checks, perfboard
    tools/kicad/lib/         ─┘ (vendored KiCad 10.0.6 symbols and footprints)

Stdlib only (Python 3.11+), so CI checks a carrier without KiCad installed.
Modules, one responsibility each:

- `errors`      KicadError, the one exception raised
- `paths`       REPO_ROOT, a project's carrier directory and board.toml, the
                git-tracked carrier projects, and the shared project-argument
                handling every per-project CLI uses
- `sexpr`       s-expression spans and blocks of KiCad files; not a full parser
- `library`     where a symbol library or footprint file is found: the vendored
                copy first, the installed KiCad only for `vendor`
- `symbols`     a symbol's pins, flags and extent, and its `lib_symbols` embedding
- `footprints`  a footprint's pads, courtyard, attributes and user text
- `registry`    the parts registry (docs/reference/parts/): pin maps, checked
                against the vendored library, polarity included
- `netlist`     CLI: the carrier netlist, from the join's carrier view and the
                registry; KiCad's no-connect net names
- `parity`      CLI (local, kicad-cli): the committed schematic's netlist
                against the join's
- `vendor`      CLI: pin, compare and rebuild the vendored library subtrees
- `derive_xiao` CLI: the socket XIAO parts derived from Seeed's DIP ones

Import nothing here: `python3 -m kicad.<module>` must not pay for, or fail on,
modules it does not use.
"""
