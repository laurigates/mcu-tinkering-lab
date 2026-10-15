---
name: carrier-pcb
description: "Generate, check and fabricate a project's KiCad carrier PCB from hardware.toml with tools/kicad, and the KiCad 10 traps that read as success. Use when adding a part to the parts registry, regenerating or checking a carrier, running Freerouting, or editing tools/kicad."
user-invocable: true
argument-hint: "[topic: registry | regenerate | board | traps]"
allowed-tools: Bash, Read, Grep, Glob, Edit
---

# Carrier PCBs: Generated From the Join, Checked Without KiCad in CI

A carrier's schematic is written from `hardware.toml` and the parts registry by
`tools/kicad`; its board is placed and routed by scripts; CI checks both
without KiCad. The design and its reasons are
[ADR-027](../../../docs/decisions/ADR-027-kicad-carrier-generation.md).

**Verified with** KiCad 10.0.6 on macOS and in `kicad/kicad:10.0.6`, Freerouting
2.5.0, 2026-10-13 to 2026-10-14 (#749, #750). The file format and
several traps below are version-specific: after a KiCad upgrade, re-run
`just kicad::vendor-check` and the "gate can fail" checks before trusting them.

**Consumers in this repo:**

- `tools/kicad/`: the package (module map in its `__init__.py`), its tests,
  and `lib/` (the vendored, pinned library subset)
- `tools/kicad.just`: the per-project recipes; `tools/kicad/justfile`: `just kicad::*`
- `docs/reference/parts/`: the parts registry
- `packages/robotics/balancebot/hardware/carrier/`: the first carrier;
  `board.toml` holds placement, silkscreen, net classes and the pour
- `packages/robocar/unified/hardware/carrier/`: head and main boards on one PCB,
  split by a `[[keepouts]]` band under a `[silk] cut_lines` saw line, with a
  `[pour]` region per ground net (`GND`, `HEAD_GND`)
- `.github/workflows/carrier-check.yml` (CI) and the `kicad-carrier` pre-commit hook

## Workflow

1. Edit `hardware.toml` (carrier-only entries take `build = "carrier"`) or a
   registry entry.
2. `just <p>::carrier-schematic` writes the `.kicad_sch` and library tables.
   Never edit the schematic in KiCad: `schematic --check` fails on any edit.
3. `just <p>::carrier-check` (KiCad 10.0.6): ERC, DRC with schematic parity,
   netlist parity with the join, the board check, then the check stamp.
4. If the board check fails, the board is stale: `just <p>::carrier-board`
   re-places and re-routes it from scratch. Re-run `carrier-check` and commit
   that board with its stamp.
5. `just <p>::carrier-fab` (runs `carrier-check` first): Gerbers, drill, BOM
   and CPL. `just kicad::check` runs what CI runs.

## Adding a registry entry

- `source` is mandatory: where the pin order came from (vendor board file,
  KiCad library, or memory plus the issue tracking its check). A recalled pin
  order reads exactly like a checked one.
- The symbol and footprint must be vendored: add them, then `just kicad::vendor`
  (rebuilds `tools/kicad/lib/kicad-stock/` from the installed KiCad).
- **Polarity tripwire.** Every "+"/"−" (and "K"/"A") must be proved by the
  symbol's pin names, the footprint's silkscreen, or `registry.CONVENTIONS`.
  An unproved one fails, because a reversed part agrees with every other check
  built from the same mistake.
- Format and rules: `docs/reference/parts/README.md`.

## Traps

### From #749 and `kicad-scripting.md`

1. **`board.Remove()` corrupts SWIG for the rest of the process.** Every later
   `pcbnew` call returns an untyped `SwigPyObject`, and the error lands on
   whichever call comes next. Build a fresh `NewBoard()` in a scratch
   directory (it writes a `.kicad_pro` beside itself) and `Save()` to the real
   path.
2. **A vendor footprint can carry board-shape geometry.** Seeed's
   `XIAO-*-DIP` mills an Edge.Cuts cutout under the module; on a socketed
   carrier that is a hole in the board. `grep -c Edge.Cuts` before reusing a
   vendor footprint; `kicad.derive_xiao` derives the socket version.
3. **XT30 pad 1 is "−".** ERC, DRC and every net check pass with VBAT on pad 1,
   because the error is consistent everywhere. The registry's polarity
   tripwire now pins it; still check the physical part's moulding (#748).
4. **ERC JSON positions are in units of 100 mm**, although the report's
   `coordinate_units` says `mm`. Multiply by 100; the result lands on the
   1.27 mm grid.
5. **Freerouting does not reproduce a route.** Commit the board you checked,
   not a re-run; compare netlists, never board files; fix a class of problem,
   not one run's geometry.
6. **Claude Code only re-reads `.mcp.json` at startup.** A kicad-mcp-pro
   update or an `.mcp.json` edit needs a session restart before the `kicad`
   MCP tools change. (From issue #750; nothing in the repo recorded it.)

### Found while building `tools/kicad`

7. **DRC parity matches footprints to symbols by reference, not UUID.**
   Rewriting every symbol UUID still gives 0 parity issues; a swapped or
   renamed ref silently re-pairs footprints, and stale UUID paths go unnoticed.
   `kicad.boardcheck` checks the paths.
8. **ERC `lib_symbol_mismatch` misses pin-type and pin-length changes** in an
   embedded symbol. The shadowed, manifest-pinned subset makes this moot for
   the carriers; `just kicad::vendor-check` reports drift from the installed
   KiCad.
9. **A wrong `${KIPRJMOD}/../` depth** in a library table shows up only as
   `lib_symbol_issues` / `lib_footprint_issues`, which `--severity-all` makes
   fatal. `kicad.libtables` computes the depth.
10. **KiCad's bundled Python is 3.9, with no `tomllib`.** The pcbnew scripts
    in `tools/kicad/pcb/` read JSON; `test_pcb_py39` parses them as 3.9.
11. **Freerouting 2.5.0 needs Java 25 or later** (class file version 69); mise
    `temurin-17` cannot run it. The jar is `$FREEROUTING_JAR`, else
    `~/.cache/freerouting/freerouting-2.5.0.jar`.
12. **ERC needs a PWR_FLAG** on every net with a `power_in` pin and no driver.
    The writer adds one.
13. **The KiCad GUI rewrites `.kicad_pro` `top_level_sheets` to `[]`.**
    `schematic --check` catches it.
14. **kicad-mcp-pro's schematic writer** draws UUIDs from an unseeded `uuid4`
    and writes no no-connects. Never use it for committed files.
15. **The derived XIAO socket keeps `(attr smd)`**, so U1 appears in
    `--smd-only` position exports. `jlc.py` drops it because it has no LCSC
    number. Fixing the attribute changes the board footprint (follow-up issue).
16. **DRC `--schematic-parity` exits 0 with "0 schematic parity issues" when
    it cannot load the schematic.** Only stdout says `Failed to fetch schematic
    netlist`; the report and JSON equal a real pass. `kicad.check` greps stdout,
    requires `Found 0 schematic parity issues`, and checks the schematic exists.
17. **No-connect pins are nets on the board**, named
    `unconnected-(<ref>-<pin name>-Pad<num>)`. A board built without them fails
    DRC parity with `Pad missing net given by schematic`. `netlist.nc_net_name`
    gives the names; `boardcheck` and the stamp check them.
18. **A project sym/fp-lib-table entry shadows the global library of the same
    nickname.** The carriers rely on it to pin the stock subset; elsewhere it is
    how a stale local copy silently wins.
19. **A passive-only MCU socket hides an unpowered MCU from ERC.** The Seeed
    socket's pins are all `passive`, and implied no-connects cover GPIO pads
    only, so `netlist` requires every power pad to be on a rail.

## Verify the gate can fail

Before trusting a carrier gate, mutate a scratch copy and watch it fail (move
a pin in `pin_config.h`, swap two refs, rename the schematic away, text-edit a
pad net, move `board.toml` away). Restore from a `cp` snapshot, never
`git checkout --`.

## Related

- `.claude/rules/kicad-scripting.md`: the edit-time pointer to this skill
- `.claude/rules/board-layout-from-vendor-files.md`: a breakout's pin order
  comes from its vendor's board file
- ADR-021 (the join), ADR-023 (documentation schematics), ADR-027 (this)
- `packages/robotics/balancebot/hardware/carrier/README.md`: the balancebot
  carrier, its pre-order checks and ordering steps
