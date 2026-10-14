# ADR-027: KiCad Carrier Boards Generated From the Hardware Join

**Status**: accepted
**Date**: 2026-10-13
**Source**: issue #750, conversation 2026-10-13
**Confidence**: 8/10 (the KiCad file format is pinned to 10.0.6)
**Extends**: [ADR-021](ADR-021-hardware-source-of-truth.md)

---

## Context

ADR-021 made `hardware.toml`, the pin header and the board page one join that
every pin document is generated from. The balancebot carrier PCB (#749) was the
first consumer outside it. It restated the join twice:

- a hand-typed netlist, drawn into `balancebot-carrier.kicad_sch`;
- `check_pins.py`, whose `ROLE_NETS` table mapped each `pin_config.h` role to
  the carrier net it was meant to land on.

Both could drift from the join, and one did. The board wires the pack-voltage
divider into D0, while `pin_config.h` listed D0 as spare. `check_pins.py`
passed, because it only checked the roles it listed, and D0 had no role.

Nothing in CI read the carrier at all. A pin moved in `pin_config.h` left the
schematic and board stale, with every check green.

## Decision

The carrier schematic is generated from the join, and CI checks the committed
carrier against it without KiCad installed.

1. **`build`-filtered views of `hardware.toml`.** Entries can carry
   `build = "harness"` or `build = "carrier"`; entries without one are in both
   views. `join()` defaults to the harness view, so every existing docs
   consumer (WIRING.md tables, pinout images, schematics, build guides) reads
   what it read before. `tools/kicad` asks for the carrier view. Carrier-only
   facts live in the same file rather than an overlay, so there is one source.
2. **A parts registry**, `docs/reference/parts/<slug>.toml`: each part's KiCad
   symbol, footprint, value, LCSC number and pin map, with the source the pin
   map was read from. Polarised pins must be proved by the symbol's pin names,
   the footprint's silkscreen or a pinned convention; an unproved one fails.
3. **An in-repo schematic writer**, `tools/kicad/schematic.py`: stdlib only,
   deterministic (UUIDs are uuid5 of the carrier directory and the item), and
   byte-identical across Python 3.11, 3.13 and 3.14. Connections are global
   labels on pin stubs; no-connects and power flags are written explicitly.
4. **KiCad-free CI** (`carrier-check.yml`): the `tools/kicad` unit tests, the
   vendored library's hashes, regenerate-and-diff of the schematic and library
   tables, a board parity check (`kicad.boardcheck`: footprints, values, LCSC
   numbers, paths and every pad's net against the join), the check stamp, and
   regenerate-and-diff of each perfboard layout's drawing and wire list
   (`kicad.perfboard --check`, which also fails a layout that breaks its
   placement, wiring or routing checks). ERC, DRC and Freerouting run locally. `just <project>::carrier-check` runs
   ERC, DRC with schematic parity, netlist parity and the board check, and
   only when all pass writes the stamp CI verifies.
5. **A vendored library subset** under `tools/kicad/lib/`: the KiCad 10.0.6
   symbols and footprints the carriers use (`kicad-stock/`, CC-BY-SA 4.0) and
   the Seeed XIAO parts (`seeed-xiao/`, MIT), each pinned file by file in a
   `MANIFEST.tsv`. Each carrier's project library tables point the stock
   nicknames (`Device`, `Resistor_SMD`, ...) at the subset, shadowing the
   installed libraries, so local ERC and DRC read the bytes CI pins.
6. **pcbnew is confined to `tools/kicad/pcb/`.** Placement, routing and the
   DSN rewrite run under KiCad's bundled Python 3.9 and read a JSON file
   (`kicad.boardinput`) built from the netlist and the project's `board.toml`.
   Nothing else imports pcbnew.

### The check stamp

`<name>.check.json` beside the carrier project records the sha256 of the
committed `.kicad_sch` and `.kicad_pcb` and KiCad's own netlist of that
schematic, normalised. Only `kicad.check` writes it, as its last step. CI
(`kicad.stamp`) fails when either hash differs from the committed file or the
recorded netlist differs from the join's.

It closes two holes the KiCad-free checks leave:

- `schematic --check` compares the writer's output with itself, so a writer
  bug plus a regenerated schematic would pass. The stamp needs KiCad to have
  read those bytes and produced the join's netlist.
- `boardcheck` reads pad nets only. Text-editing a pad's `(net ...)` greens it
  while the copper still joins the old pads; the board hash then no longer
  matches the stamp.

It does not stop a deliberate forgery (anyone can compute two hashes); it
stops accidental drift.

## Scope vs ADR-023

[ADR-023](ADR-023-schematic-rendering.md) owns the *documentation* schematics
(schemdraw, `docs/schematics/`), and they stay. This ADR is about a
*fabrication* output: the KiCad project a board is ordered from. The two read
the same join and do not replace each other.

ADR-023 rejected KiCad for documentation partly as "~6 GB in CI". That
overstates it: `kicad/kicad:10.0.6` is 771 MiB compressed and 2.6 GB
unpacked; only the `-full` variant (3D models) is larger. The image is
amd64-only for 10.x, runs ERC or DRC in about 6 s under Rosetta, and has no
Java, so it could not run Freerouting either. It is not used: CI here needs
no KiCad.

## Alternatives Considered

- **kicad-mcp-pro's `sch_build_circuit`, pinned.** Driven headless, in
  process, it reproduced #749's netlist exactly. Rejected: it is an
  undocumented internal API of a 7,000-line module with 26 PyPI releases in 60
  days; it draws UUIDs from an unseeded `uuid4`, so output differs on every
  run; it writes no no-connects and hard-codes `in_bom`; and it needs Python
  3.13 or later. Each fix would be a patch or post-step around an API the
  package does not publish.
- **kiutils, kicad-skip.** Unmaintained since 2024, and GPL-3.0 and LGPL-2.1.
  kiutils drops KiCad 10 fields on a round trip; kicad-skip warns on keys it
  cannot parse.
- **kicad-sch-api.** Drops every global label on a load/save round trip (28
  nets became 46, ERC went from 0 to 127 violations).
- **kipy (kicad-python).** No schematic API on KiCad 10; its IPC API needs a
  running GUI.
- **A separate carrier overlay file** next to `hardware.toml`. Rejected: two
  sources for one board's pins.
- **ERC and DRC in CI, in the `kicad/kicad` container.** Deferred. The stamp
  gives CI evidence that KiCad checked the committed bytes without a 2.6 GB
  pull on every run.
- **Keep the hand-drawn schematic and add only a parity check.** Rejected: the
  schematic would still be hand-maintained, and every pin change would need a
  KiCad session to redraw it.

## Consequences

- The generated `.kicad_sch` is not hand-editable: `schematic --check` fails on
  any edit. Changes go into `hardware.toml` or the parts registry.
- The committed routed board stays valid. KiCad 10.0.6 matches footprints to
  symbols by reference, not UUID, so DRC with schematic parity reports 0 / 0 /
  0 against the generated schematic without re-routing. The board's footprint
  paths were relinked once to the writer's UUIDs, so the GUI's "Update PCB from
  Schematic" and cross-probing stay consistent.
- An MIT repository carries a CC-BY-SA subtree. `tools/kicad/lib/kicad-stock/`
  keeps the upstream `LICENSE.md` and a modification notice; the root README
  states the exception.
- Shadowing the stock nicknames means KiCad's library chooser, in these
  projects, lists only the vendored subset. A new part goes through the
  registry and `just kicad::vendor`.
- KiCad 11 drops the SWIG `pcbnew` module. Only `tools/kicad/pcb/` uses it.
- A change to a carrier's schematic or board needs a local KiCad 10.0.6 run
  (`just <project>::carrier-check`, which refreshes the stamp) before CI is
  green. That includes a writer change: it cannot pass CI until KiCad has read
  its output. Any such change already needed KiCad (a re-place and re-route, or
  ERC on the new output), so the added cost is committing the stamp.
- `boardcheck` and the stamp stay out of pre-commit. `test.yml` runs
  pre-commit and `auto-fix.yml` acts on its failures, and a fixer without
  KiCad could only green those two by hand-editing the board or the stamp.
  For the same reason the unit tests in the `kicad-carrier` hook never compare
  a live carrier with #749's netlist export: they build from a frozen copy of
  the balancebot inputs (`tools/kicad/testfixture.py`). The unit suite still
  holds the committed schematic and perfboard files equal to their generators'
  output, so after a legitimate pin move the hook goes green once both
  `just <project>::carrier-schematic` and `just <project>::perfboard` have
  run; CI's stamp step stays red until `carrier-check` has run locally.

## Related

- [ADR-021](ADR-021-hardware-source-of-truth.md): the join this extends
- [ADR-023](ADR-023-schematic-rendering.md): the documentation schematics
- #749 (the balancebot carrier), #748 (pin orders to check before ordering),
  #747 (firmware reads the pack divider)
- `.claude/skills/carrier-pcb/`: the workflow and the KiCad 10 traps
- `packages/robotics/balancebot/hardware/carrier/README.md`

## Implementation notes

Views (`tools/hardware`):

| Entry | Harness view (default) | Carrier view |
|---|---|---|
| no `build` key | in | in |
| `build = "harness"` | in | out |
| `build = "carrier"` | out | in |
| `[[rails]]` without `from` | continues the rail of that name | same |
| `[net_names]` | ignored | renames nets on the board |

Module map (`tools/kicad`; the package docstring has the full list):

| Module | Does |
|---|---|
| `registry`, `netlist` | parts registry; the carrier netlist from the join's carrier view |
| `schematic`, `libtables` | the `.kicad_sch` and library tables; `--check` |
| `parity`, `check` | local, kicad-cli: netlist parity; every local gate, then the stamp |
| `boardcheck`, `stamp` | KiCad-free board parity and stamp verification (CI) |
| `boardinput`, `pcb/` | JSON for the pcbnew scripts; place, route, DSN rewrite |
| `fab`, `jlc` | Gerbers, drill, BOM and CPL; JLCPCB's assembly format |
| `vendor`, `derive_xiao` | the pinned library subtrees; the socket XIAO parts |
| `perfboard/` | KiCad-free: the carrier netlist laid out on a perfboard (`hardware/perfboard/perfboard.toml`): drawing and wire list; `--check` |

Per project: `hardware/carrier/board.toml` (KiCad project name, placement,
silkscreen, net classes, pour) and the generated KiCad project beside it.
Recipes come from `tools/kicad.just` (`carrier-schematic`, `carrier-board`,
`carrier-check`, `carrier-fab`); `just kicad::check` runs what CI runs.
