# ADR-031: Circuit Blocks: a Stage Designed Once, Used From Any hardware.toml

**Status**: accepted
**Date**: 2026-11-06
**Confidence**: 8/10 (the first two blocks reproduce the integrated carrier's
netlist, schematic, stamp and board check byte-identical; the 3S buck block
and balancebot's input stage are still to come)
**Issue**: [#976](https://github.com/laurigates/mcu-tinkering-lab/issues/976), under [#970](https://github.com/laurigates/mcu-tinkering-lab/issues/970)
**Extends**: [ADR-021](ADR-021-hardware-source-of-truth.md),
[ADR-027](ADR-027-kicad-carrier-generation.md)

---

## Context

Each carrier's `hardware.toml` spells out its power stages part by part:
robocar-integrated's TPS563208 buck is thirteen `[parts.*]` tables,
four internal rails and three port lists folded into other rails, with about
fifty lines of datasheet reasoning in comments. The pack-input stage (PTC,
reverse-polarity P-FET, gate resistor) was designed for that carrier and
back-ported to balancebot by hand (#822). The next carrier would copy both
again, and a correction found on one board would not reach the others.

#970 asks for reusable protection blocks as part of a carrier design
standard. This ADR decides the mechanism.

## Decision

A **block** is `docs/reference/blocks/<slug>.toml`: registry parts without
refs, the rails between them, and named **ports**, each a list of block pins.
The reasoning for every value lives in the block file's comments.

A project uses a block with a `[blocks.<id>]` table and connects a port by
writing `<id>.<PORT>` wherever a part pin may go:

```toml
[blocks.buck]
block = "tps563208-5v"
net_prefix = "BUCK"                    # internal rail SW becomes BUCK_SW
refs = { ic = "U10", l = "L1", ... }   # exactly one per block part
net_names = { FB = "BUCK_FB" }         # optional: name one internal rail
parts.fb_top = { value = "56k" }       # optional: name, note, value, lcsc, buy
build = "carrier"                      # optional: on every expanded entry

[[rails]]
name = "VBAT"
from = "input.OUT"
to = ["buck.VIN", "bulk_in.+"]
```

`tools/hardware/blocks.py` **expands blocks when the file is read**
(`load_sidecar`), before the join sees it:

- block part `<key>` becomes `[parts.<id>_<key>]` with its ref;
- each internal rail is appended to `[[rails]]` as `<net_prefix>_<name>`
  unless `net_names` renames it;
- each `<id>.<PORT>` in a `from`/`to` of `rails`, `nets`, `channel_nets`,
  `bus_nets` or `outputs` becomes the port's pins. A multi-pin port in a rail's
  `from` keeps its first pin there and moves the rest to `to`; anywhere else a
  single-valued field must name a one-pin port.

Every reader of a `hardware.toml` goes through `load_sidecar`: the join
(`hardware.model.join`) and `kicad.lcsc.used`. Nothing downstream knows
blocks exist, so ERC, the netlist, the schematic writer, the check stamp, the
fab BOM and the rotation table work unchanged.

**Refs stay per project.** They belong to the board: placement, fixed tracks
and silkscreen in `board.toml` name them. A block never allocates refs.

**What stays in the project:** the rails joining the block to the rest of the
circuit, parts the block's datasheet does not call for (an input bulk
capacitor sized to the pack leads, a test header), and the comments on why
this board picked an override.

**Expansion fails loudly** on an unknown block, a missing or extra ref, a ref
that is not `[A-Z]+[0-9]+`, an override of a part the block lacks or of a
fixed key (`part`, `kind`, `ref`), an unknown port, a port that is not
connected anywhere, a part key or rail name that collides with the project's,
and a block pin naming no block part.

### Parameters are overrides

A block has no formula parameters. A project changes a value by overriding
it (`parts.fb_top = { value = "62k", lcsc = "C..." }`) and says why in its own
comment. Computing a divider from a target voltage would put datasheet
equations in code, where review cannot see the choice of E-series value, the
tolerance band or the JLC Basic part. The block's comments give the equation;
the override records the decision.

### Footprint and symbol policy

Blocks reference registry parts, and the registry keeps its rule: symbols and
footprints come from **KiCad stock first**, vendored and pinned by
`tools/kicad/lib/kicad-stock/MANIFEST.tsv`. A third-party footprint enters one
at a time, in its own pinned subtree like `breakout-sockets`, with its source
and licence recorded. Whole third-party libraries are not adopted:
CDFER/JLCPCB-Kicad-Library (reviewed 2026-11-06) names footprints after
EasyEDA packages, which would invalidate every registry pin map and every
`jlcpcb-rotations.toml` entry. Single parts from it that KiCad lacks (the
WS2812B-1615, the HX 16-pin USB-C) are candidates under the one-at-a-time
rule. JLCPCB part data comes per part from `kicad.lcsc`'s cache, not from a
copied parts database.

## Consequences

**Gained**

- A stage's reasoning sits once, beside its values. A correction to the
  block reaches every carrier on the next regeneration, and the carrier check
  shows which boards it changed.
- robocar-integrated's `hardware.toml` is 200 lines shorter: it lost the part
  tables and rails and keeps only the board-specific notes.
- The netlist, schematic, stamp and board check of a re-expressed project are
  byte-identical: the netlist sorts components by ref and nets by name, so the
  order expansion appends entries in does not reach any generated file.

**Costs**

- One more file to read when following a net: `buck.VIN` in a rail means the
  block's VIN port, not a pin of a part called `buck`.
- Part keys inside a block become `<id>_<key>`; the IC of `[blocks.buck]` is
  `buck_ic`, not `buck`. Only `kicad.lcsc`'s where-used column shows part keys.
- A block change regenerates every carrier that uses it. That is the point,
  but it also means a value tweak for one board belongs in that board's
  override, not in the block.

**Out of scope here**

- The 3S buck block (the TPS563208's input range stops below a 3S hot-plug
  ring) and balancebot's input stage (fuse first, gate zener, ZXMP4A16G) are
  separate blocks, tracked in #976.
- Placement or layout fragments in `board.toml`. Blocks describe the circuit;
  layout stays per board.

## Alternatives considered

- **Expand in the join only.** `kicad.lcsc` and the view tests also read the
  raw TOML, so they would have seen no block parts. Expanding at load, in one
  function every reader calls, keeps them consistent.
- **A block as a KiCad hierarchical sheet.** The schematic is generated and
  not hand-edited (ADR-027); a sheet would split the one-sheet schematic and
  the check stamp for no gain the join does not already give.
- **Templated part keys and net names** (`{prefix}_SW` inside the block).
  `net_prefix` plus `net_names` covers the existing names (the input stage's
  `PACK_RP` and `PFET_GATE` do not share a prefix) with less syntax.

## Related

- `tools/hardware/blocks.py`, `tools/hardware/test_blocks.py`
- `docs/reference/blocks/README.md`: the format
- `docs/reference/parts/README.md`: the registry blocks draw from
- `.claude/skills/carrier-pcb/SKILL.md`
