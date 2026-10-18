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
- `schematic`   CLI: the carrier `.kicad_sch` and library tables, written from
                the netlist; `--check` fails when they are stale
- `libtables`   the project sym-lib-table / fp-lib-table, every nickname
                pointed at the vendored copy
- `boardcheck`  CLI: the committed `.kicad_pcb` footprints and pad nets against
                the join, and board.toml against the netlist; `--relink`
- `stamp`       CLI: the check stamp; `--write` locally after every gate,
                verified KiCad-free in CI
- `check`       CLI (local, kicad-cli): every local carrier gate in order,
                ending in `stamp --write`
- `area`        CLI: each part's courtyard area, grouped into main board, head
                and mechanical; given two projects, the main-board parts only
                one has
- `boardinput`  CLI: the JSON the pcbnew scripts read (netlist, placement,
                silkscreen, net classes, pour), from the join and board.toml
- `fab`         CLI (local, kicad-cli): gerbers, drill files and their zip,
                BOM and CPL, then `jlc`
- `jlc`         CLI: kicad-cli's BOM and CPL cut to JLCPCB's assembly format
- `vendor`      CLI: pin, compare and rebuild the vendored library subtrees
- `derive_xiao` CLI: the socket XIAO parts derived from Seeed's DIP ones
- `sockets`     CLI: breakout socket footprints from the board pages' pad
                centres (`lib/breakout-sockets/`)
- `perfboard`   CLI (a subpackage): a carrier laid out on a pad-per-hole
                perfboard, drawing and wire list, from the netlist and
                `hardware/perfboard/perfboard.toml`
- `testfixture` for the unit tests: the frozen balancebot inputs behind
                #749's netlist export (`testdata/balancebot-749/`)

`pcb/` is not part of the package: `build_board.py` (place, assign nets,
silkscreen), `route_board.py` (Freerouting, then the pour) and `dsn.py` (the
DSN net-class rewrite) run as scripts under KiCad's bundled Python 3.9, read
`boardinput`'s JSON, and import nothing from here (test_pcb_py39 pins that).

Import nothing here: `python3 -m kicad.<module>` must not pay for, or fail on,
modules it does not use.
"""
