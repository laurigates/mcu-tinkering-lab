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
  `board.toml` holds placement, silkscreen, net classes and the pour, VBAT as
  fixed `[[tracks]]` with no via, `[fanout]` and `[pour] stitch` vias, and a
  `.kicad_dru` rule that keeps the pour off the mounting holes
- `packages/robotics/robocar/hardware/carrier/` and
  `packages/robotics/robocar/head/hardware/carrier/`: one hardware.toml split
  into two PCBs (#1280, trap 57). The main board (`side = "main"`) is the whole
  100 × 100 mm panel, with the pack switch, latch and BQ25887 charger and the
  fixed tracks that meet the charger's 0.5 mm-pitch pins. The head panel
  (`side = "head"`, `join = ".."`) is three head boards split by
  `[[keepouts]]` bands under `[silk] cut_lines` saw lines, with a `[pour]`
  region per head ground (`HEAD_GND`, `HEAD2_GND`, `HEAD3_GND`); the spare
  heads come from `[[cables]] spare_heads` (ADR-021). Two head IDC headers sit
  on the back face (`[placement]` entry `[x, y, rot, "back"]`, mirrored about
  its anchor, so the pads run the other way: at rotation 90 they run toward -x)
- `packages/robotics/robocar/integrated/hardware/carrier/`: the same carrier with
  the breakout chips and a buck converter on the board, for comparison.
  `just robocar-integrated::compare` runs `kicad.area`, the
  courtyard-area comparison against the socketed carrier. A carrier with no
  board yet takes `stage = "schematic"` in `board.toml`: boardcheck, the stamp
  and the local DRC skip it. It also shows the layout features the router
  does not provide: `[[tracks]]` (fixed tracks build_board.py lays down
  locked, here a 1.5 mm 5V trunk, the buck's GND and an exposed pad's ground
  ties), `[fanout]` (a via beside every SMD pad of a poured net, added after
  routing; `required` pads fail the route without one; an `own` pad takes a
  via of its own even when a fixed-track chain already reaches one) and `[pour]` regions
  with their own `layers` and `priority`. It is also the mouse-bite carrier:
  `[panel] mode = "mousebites"` (#809, `kicad.panel`) turns the cut lines into
  Edge.Cuts slots and board-only MB<n> footprints of NPTH holes, all inside
  the keepout bands; `boardcheck` compares both with board.toml
- `just kicad::setup` (`tools/kicad/setup.sh`): KiCad 10.0.6 on Ubuntu 24.04,
  Freerouting 2.5.0 and a Java 25 JRE in `~/.cache/freerouting`. With
  `--docker` (or when Launchpad no longer serves the .deb, #1146) it writes
  `kicad-cli` and `kicad-python` wrappers around `kicad/kicad:10.0.6-full` and
  prints the exports; that is how a cloud session runs every carrier recipe
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
   that board with its stamp. `just <p>::carrier-board-checked` does both in a
   loop and stops at the first run that passes every gate (Freerouting does not
   route the same way twice).
5. Before fabrication: `just <p>::carrier-review` prints the layout metrics the
   gates do not check (vias per poured net, ground pads without one, pour
   islands, narrowest power tracks, decoupling distances, cut-line clearances,
   silkscreen size, pour pieces only tracks join to the plane).
   `just <p>::carrier-measure <path|cuts|vias-in-pads|clearance-to-holes|plane|resistance>`
   measures per layer, joining layers only at vias and through-hole pads
   (`kicad.measure` over `pcb/dump.py`'s JSON; `--json`, and `--ref <rev>` for
   the board at another commit, read by `git show`; its printed coordinates,
   the `[fanout] in_pad` exemptions and the poured-net list still come from
   today's `board.toml`). Numbers in a README or PR
   come from its output, not a scratch script or a hand copy: a layer-blind
   script once reported "no via" on a path through two. Then run the
   `pcb-reviewer` agent (`.claude/agents/pcb-reviewer.md`) for an adversarial
   review against the datasheets, and `.claude/rules/power-stage-checklist.md` for any regulator.
   Run it as a loop:
   - Verify each finding against the board before acting on it: the review is
     model output, and its numbers can be wrong.
   - Fix in `board.toml`, `hardware.toml` or `tools/kicad`, re-route
     (`carrier-board-checked`), then re-measure with `just <p>::carrier-measure
     <subcommand>` where it exists (`path`, `cuts`, `vias-in-pads`, `plane`,
     `resistance`, `clearance-to-holes`); add `--ref <git-ref>` for the before
     side of the table.
   - Write the README's before/after table only from measurement output.
   - Re-run the reviewer on the fixed board. On the integrated carrier every
     fix round introduced a regression the next pass found: a 0.6/0.3 mm VBAT
     via after walling off F.Cu, a neck shared by the servo and head feeds,
     and a split top-half GND after widening +5V.
6. `just <p>::carrier-fab` (runs `carrier-check` first): Gerbers, drill, BOM
   and CPL, named for the commit (`<name>-r<N>-fab-<sha7>.zip`,
   `<name>-r<N>-bom-jlc-<sha7>.csv`, `<name>-r<N>-cpl-jlc-<sha7>.csv`) beside a
   hand-off row (`<name>-r<N>-fab-<sha7>.csv`: the commit and the zip's sha256).
   It refuses uncommitted changes to anything that feeds the board
   (`CARRIER_FAB_ALLOW_DIRTY=1` exports anyway, with `-dirty` in the file
   names), and plots from a temporary copy of the board that carries
   `r<N> <sha7> <pcb8>` (`[fab] revision`, the commit and the stamp's `board_sha256` prefix) on
   F.SilkS at `board.toml`'s `[fab] provenance_at`; KiCad's DRC on that copy
   fails the run if the text lands on a pad, other silkscreen or the edge. A
   dirty run writes `r<N> <sha7>*<pcb8>`: KiCad's stroke font gives `*` a space's
   advance, so the text is as wide as the clean one and passes DRC alike. The committed board and stamp
   are only read (`kicad.provenance`). A new carrier needs `[fab] revision`
   and `provenance_at` before its first fab run. Once a revision is in
   `hardware/carrier/orders.toml` (`kicad.revision record`, after payment),
   a changed board needs a higher `[fab] revision`; the `kicad-carrier`
   hook, CI and `carrier-fab` refuse it until then. `just kicad::check` runs what CI
   runs. The CPL adds each LCSC
   part's checked rotation offset (trap 32); a new LCSC number needs an entry
   in `docs/reference/jlcpcb-rotations.toml` first. `carrier-fab` first runs
   `python3 -m kicad.lcsc --project <p>`, which asks JLCPCB about each LCSC
   number (cached a day under `~/.cache/mcu-tinkering-lab/lcsc`, nothing
   committed) and stops on one that is unknown, not orderable (trap 51) or
   out of stock; extended parts only warn. `just kicad::parts` checks every number
   in use (`--json`, `--refresh`, `--offline`). Quoting and ordering the
   result: the `jlcpcb-order` skill, whose § 1 decides which SMD parts
   JLCPCB places: any part not owned and not sold on AliExpress stays on the
   PCBA BOM (`.claude/rules/part-sourcing.md`).

## Using a circuit block

A stage that already exists as a block (`docs/reference/blocks/`, ADR-031)
goes in as `[blocks.<id>]` with `block`, `net_prefix` and one ref per block
part; connect its ports by writing `<id>.<PORT>` in the project's rails. Change
a value for one board with an override (`parts.<key> = { value = ... }`) and a
comment, not by editing the block: a block edit regenerates every carrier
that uses it. Part keys expand to `<id>_<key>`, so the IC of `[blocks.buck]`
is `buck_ic`. Re-expressing an existing stage as a block must leave
`kicad.netlist`, `kicad.schematic --check` and `kicad.stamp` unchanged.

## Adding a registry entry

- **Choose a part that can be bought** before drawing it: owned, placeable by
  JLCPCB, or sold on AliExpress under its exact part number, and current, not
  NRND or obsolete (`.claude/rules/part-sourcing.md`). Record it as `buy`
  (`aliexpress:<id>`, `lcsc:C<n>`, `owned:<what>`) on the entry, or on the
  instance for a value-specific part; `lcsc` alone means only JLCPCB supplies
  it. `just kicad::sourcing` fails on a BOM part with neither.
- `source` is mandatory: where the pin order came from (vendor board file,
  KiCad library, or memory plus the issue tracking its check). A recalled pin
  order reads exactly like a checked one.
- The symbol and footprint must be vendored: add them, then `just kicad::vendor`
  (rebuilds `tools/kicad/lib/kicad-stock/` from the installed KiCad). Then run
  the whole `just kicad::test`, not the module you touched: `test_vendor` pins
  the subset's symbol and footprint counts, and a partial run let a vendored
  polyfuse reach CI with stale counts.
- **Polarity tripwire.** Every "+"/"−" (and "K"/"A") must be proved by the
  symbol's pin names, the footprint's silkscreen, or `registry.CONVENTIONS`.
  An unproved one fails, because a reversed part agrees with every other check
  built from the same mistake.
- Format and rules: `docs/reference/parts/README.md`.

## A socketed sensor's orientation

A board with an IMU in a socket declares it, so the layout and the firmware
mount cannot drift apart (8d18cd8d put J3 where the GY-521 only fits face
down):

- The socket's `sockets.toml` entry gives the sensor's `axes = { x = [dx,
  dy], z = "component" | "solder" }` in the footprint's y-down frame. The
  generator validates it and leaves it out of the footprint.
- `board.toml` gives `[outline] forward` (`"+x"`, `"-x"`, `"+y"` or `"-y"`;
  the board is taken as mounted top face up) and `[imu] ref`, `mount` (the
  firmware's `mount up|down`, default `"up"`) and `face` (the module in its
  socket, default `"up"`; `"down"` is turned over about the pad row).
- `kicad.boardcheck` derives the sensor-to-robot rotation (`kicad.mount`)
  from those and the placement, and fails unless it is the declared mount,
  naming the mount that would match or printing the matrix when neither does.
  Placement rotations must be multiples of 90 degrees, on the front face.

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
    `~/.cache/freerouting/freerouting-2.5.0.jar`. `just kicad::setup` installs
    a Temurin 25 JRE in `~/.cache/freerouting/jre`, and `_carrier-route` puts
    it first on the PATH.
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

### Found laying out the socketed robocar carrier (#750)

20. **A module socket can accept the module upside down.** Asymmetric column
    spacing rules out a rotated or left-right-mirrored module, but a top-bottom
    flip (component side down) keeps every column on its x and reverses the
    rows. On the CJMCU-2317 that puts its VCC on GND and 3V3 into its I/O
    pins. Every check passes, because the error is in the person fitting the
    module. Any socket for a module labelled on one face needs a
    silkscreen orientation mark and pin names beside the pads (#803).
    boardcheck now enforces it (`kicad.orientation`): for every module socket
    (`Breakout_Sockets:*`, `Module:Pololu_Breakout-*`) it finds each turn or
    flip that maps the pads onto themselves and fails when one changes a pad's
    number or keeps them all but moves the F.Fab outline (a 1xN row flipped
    across itself: the GY-521 face down on the 8d18cd8d balancebot) and no
    `[silk] texts` entry tagged `for = "<ref>"` sits within 4 mm of the
    courtyard and on the board. `[silk] module_outlines = ["<ref>"]` draws the
    footprint's own F.Fab outline on F.SilkS (less the edges its strip
    outline already draws), never a hand copy. `carrier-silk` adds both
    without touching copper.
21. **A back-face footprint in KiCad 10:** `FOOTPRINT.Flip()` segfaults before
    `board.Add()`, and `FLIP_DIRECTION_LEFT_RIGHT` followed by
    `SetOrientationDegrees(rot)` mirrors the footprint's *local y* about its
    anchor, so the pads run the other way from pad 1. Set reference and silk
    positions after the flip, or the flip mirrors them too.
    `build_board.py` does it in that order.
22. **Freerouting does not honour KiCad's copper-to-edge clearance.** It keeps
    only its class clearance from the boundary, so on a tight board it routes
    tracks inside `.kicad_pro`'s 0.5 mm `min_copper_edge_clearance` and DRC
    fails. Line the outline with 0.5 mm `[[keepouts]]`, leaving out any edge a
    courtyard reaches, because a keepout also refuses footprints.
23. **Check a fit against the stock footprint's courtyard, not the part's
    size.** `MountingHole_2.2mm_M2` has a 5 mm courtyard, so a XIAO socket did
    not fit between two holes that an M2 screw head clears;
    `MountingHole_2.2mm_M2_ISO7380` (4 mm) did. Read the courtyard from the
    vendored `.kicad_mod` before promising a layout.

### Found laying out the integrated robocar carrier

24. **Freerouting gives each net one track width, and necks it into a
    fine-pitch pin.** A power net reaching an SSOP's 0.4 mm pins does not cap
    the rail: the integrated carrier's +5V routes at 0.8 mm and lands on the
    TB6612FNG's VM pins at full width, 0.202 mm from the signal pins beside
    them (DRC's minimum is 0.2 mm). At 0.45 mm,
    the width once assumed to be the cap, its servo and head feed necked to
    0.337 mm instead. A 0.5 mm-pitch TQFN's 0.25 mm pads are another matter:
    the integrated carrier gives the MAX98357A its own AMP_5V net behind a
    ferrite bead. Necking goes below 0.2 mm, so such a board needs a smaller
    `min_track_width`. A wider class also re-cuts the pour: at 0.8 mm the
    +5V runs split the integrated carrier's top-half GND, so re-check the pour
    islands (`carrier-review`) after any class change.
25. **`kicad-cli` cannot write an ERC exclusion.** KiCad matches an exclusion
    only against its own serialisation of the marker, and a hand-built key
    (`pin_to_pin|x|y|uuid|uuid|...`) loads as a separate excluded marker while
    the real one still fails. For a symbol's pin-type quirk, change the
    project's ERC `pin_map` instead, and check that no other pin has that type.
26. **The KiCad PPA keeps only the newest 10.0.x**, and Launchpad's `+files/`
    URL for the superseded 10.0.6 .deb answered 404 on 2026-11-12 and
    2026-11-26 (#1146). `tools/kicad/setup.sh` still pins those packages by
    sha256, and falls back to the `kicad/kicad:10.0.6-full` image, pinned by
    digest, when docker is present.
27. **An extra `)` in the DSN hides everything after it, and Freerouting
    says nothing.** Until #1280 `dsn.rewrite` cut the net class at the first
    `"    )\n"`, which also matches the tail of the rule's six-space `)`, and
    left the class's own `)` behind. `(pcb ...)` then closed before
    `(wiring ...)`, so Freerouting never read a `[[tracks]]` track and routed
    straight through them. That was read as "Freerouting 2.5.0 drops the
    wiring section", and route_board.py walled every fixed segment off with a
    keepout and dropped each track's first pad from the netlist: the
    workarounds behind traps 56 and 58, which then boxed pads in themselves.
    `dsn.check_balanced` now fails the route on an unbalanced DSN. A
    `[[tracks]]` end sits on a pad of its net or on a `[[vias]]` via of it, so
    a fixed track can change layer. Reading the wiring is not the same as
    counting it as a connection: trap 59.
28. **One boxed-in pad can make Freerouting abandon a whole net.** A keepout
    that took U10's GND pin's only F.Cu way out left 30-50 GND connections
    unrouted across the board, deterministically, though every other pad had
    room and B.Cu was free. Bisect by editing the DSN (`output/carrier.dsn`)
    and running the jar on copies; dropping one pin from the net is the
    quickest test of a suspect. The fix was a `[[tracks]]` entry for that pin.
    The same happens to a pad a new keepout boxes in: a fixed track between
    fine-pitch pins can enclose its neighbour (the MAX98357A's GAIN_SLOT pin
    beside a GND tie), and fanout vias placed before routing, as keepouts,
    boxed in enough pads that GND, +3V3 and BUCK_SW all collapsed. Hence
    `[fanout]` runs after routing, and an exposed pad inside a ring of pins
    leaves the netlist (`in_pad`): Freerouting cannot reach it.
29. **A poured net's SMD pads need vias, and no gate notices.** Freerouting
    adds a via only where a track changes layer, and a pour on the pads' layer
    can be cut into islands (here by a +5V pour above GND), so a whole power
    section's grounds reached the B.Cu plane through 0.19-0.25 mm tracks and
    one via, with ERC, DRC, parity and the stamp all green. `[fanout]` adds the
    vias; `required` turns a missing one on a power path into a failure.
    After any `[fanout]` or `[[tracks]]` change, read Freerouting's
    "Auto-routing stage completed ... (N unrouted" line, not just DRC: when it
    gives up on GND, change the netlist (`in_pad`, a tie track whose first pad
    leaves it), not the keepouts.
30. **Do not commit while a KiCad or Freerouting job uses the working tree.**
    pre-commit stashes the unstaged files for the length of its hooks, so a
    `carrier-board` run reading `board.toml` mid-commit gets the committed copy:
    one route here placed a single fanout via and failed for no reason in the
    design. Commit after the run, or from a separate worktree.
31. **A pour across a hand-saw cut line is copper the saw bridges.**
    boardcheck fails a `[pour]` region within 1.0 mm of a `[silk] cut_lines`
    line (a whole-board pour meets every line). It also fails silkscreen text
    under 1.0 mm, JLCPCB's standard minimum. A board whose labels need 0.8 mm
    sets `[silk] high_precision = true`, and `carrier-fab` then prints an
    `ORDER:` line to select JLC's paid high-precision silkscreen.
    build_board.py keeps reference strokes at 0.16 mm or more.

### Found adding mouse-bite panels (#809)

34. **A keepout rule area refuses a non-plated hole.** KiCad counts an NPTH pad
    as a pad, so mouse-bite holes inside a copper keepout band fail DRC
    (`items_not_allowed`). build_board.py subtracts each tab's area from the
    rule areas (`SHAPE_POLY_SET.BooleanSubtract`) rather than relaxing the
    bands, which still keep every real part's pads out.
35. **A board-only footprint is invisible to schematic parity** and needs no
    library: an id with no library nickname raises no `lib_footprint_issues`.
    That is what makes the tabs footprints at all; boardcheck identifies them
    by their MB<n> reference and checks them against `[panel]` itself. The id
    must still have a name: with an empty one KiCad exports the DSN image as
    `""`, Freerouting writes `(component ` back with no name, and
    `ImportSpecctraSES` returns False after a complete route (every attempt
    reads "SES import failed"). build_board.py uses `LIB_ID("", "MouseBites")`.

### Found preparing JLCPCB assembly (#808)

32. **JLCPCB rotates by LCSC's footprint, not KiCad's.** Its pick-and-place
    puts each part down as LCSC's (EasyEDA) footprint draws it, and LCSC draws
    most TSSOP, SSOP, QFN and SOT-23-6 parts with pin 1 bottom-left, where
    KiCad's are top-left: KiCad's rotation passed straight through puts the
    chip 90 degrees off, and the SOT-223 LDO 180. ERC, DRC, parity and the
    stamp all pass. The offset belongs to the LCSC number, not the footprint:
    LCSC draws the PCA9685PW with pin 1 top-left (offset 0) and the
    TCA9548APWR on the same TSSOP bottom-left (270). `kicad.jlc` adds the
    offset from `docs/reference/jlcpcb-rotations.toml` and fails on an LCSC
    number without one; `python3 -m kicad.lcsc_rotation <C...> <Lib:Fp>`
    derives an entry by turning LCSC's pads onto KiCad's, `--check` re-derives
    them all (network). Pad numbers prove pin 1 lands on pin 1, not which end
    LCSC marks "+": read its silkscreen for polarised two-pad parts.
33. **EasyEDA's component API refuses a browser User-Agent** with an HTML
    error page under HTTP 200 and Python's default with a 403; curl's is
    served. `lcsc_rotation.fetch` sends curl's.

### Found in the integrated carrier's layout reviews

36. **Freerouting ignores a class's `use_via` where it does not fit.** Every
    via padstack in the DSN is open to every net, and it drops to the 0.6/0.3
    mm signal via, deterministically, at the same spot on every run; removing
    that via from the structure's `(via ...)` list changes nothing. Moving
    parts to make room did not help either. `route_board.py` grows each
    undersized via to its class's size and lets DRC judge the fit. On the
    robocar main board (#1280) U2's +3V3 via, grown from 0.6 to 0.8 mm, came
    0.16 mm from a GND track in 8 of 12 routes; that class now uses the signal
    via.
37. **A fanout via can land in a neighbouring pin's pad of its own net.**
    Same-net SMD pads were no obstacle, so U7:4's diagonal candidate sat
    inside U7:3, where the hole drains paste at reflow. Fanout vias now clear
    every pad but their own, and the route fails on any via inside an SMD pad
    but an `in_pad`, checked after fanout so both kinds are covered.
38. **Freerouting routes "redundant" GND tracks across the board** (it does
    not know the pour), and they block a via or a power track where you need
    it. `[[route_keepouts]]` (one layer, router-only, no rule area) moves them.
39. **On macOS each Freerouting run steals the focus** unless the JVM runs
    with `-Djava.awt.headless=true -Dapple.awt.UIElement=true`, even with its
    GUI off.
40. **A path script that ignores layers lies.** Joining a B.Cu track end to an
    F.Cu track it only crosses reported "no via" for a path through a via, and
    that claim reached the README. Measure per layer, with vias and
    through-hole pads as the only joins between layers.

### Found in the balancebot carrier's pre-order review

41. **KiCad's stock footprints draw their silkscreen at 0.12 mm** (some 0.10
    or 0.127 mm), under JLCPCB's 0.15 mm minimum, polarity marks included (an
    electrolytic's "+", a SOD-123's cathode bar). `carrier-review` measured
    only text strokes, so it reported 0.15 mm over 0.10 mm lines. build_board.py
    now widens every footprint silkscreen stroke under 0.15 mm to it
    (`pcb/silk.py`), which changes each carrier's silkscreen the next time it
    is rebuilt, and `carrier-review` reports the thinnest line.
42. **A fanout via placed small and grown afterwards can land on its own
    pad.** route_board.py grows a `[fanout]` via to its net class's size after
    placing it; placed at 0.6 mm, C3's grew to 1.0 mm over C3's own GND pad and
    the route failed the via-in-pad check. Give `[fanout] via_um` the class's
    via size. For pour islands that no SMD pad reaches (the space between a
    socketed module's pin rows), `[pour] stitch = { pitch_mm, via_um,
    exclude }` adds a via at each grid point where every pour layer's fill
    holds it (`pcb/stitch.py`).

### Found in the integrated carrier's sixth layout review

43. **A 0.4 mm fanout stub cannot leave a 0.25 mm pin at 0.5 mm pitch.** The
    stub starts at the pad's centre, so its end cap alone came 0.175 mm from
    the next pin on every ray, and the MAX98357A's GND pins never took a via.
    `fanout.stub_width` caps the stub at the pad's narrow side.
44. **Reserving room for a fanout via can collapse the route.** A
    `[[route_keepouts]]` square for U5:12's via took that pin's exits, and
    Freerouting left 33-49 GND connections unrouted (trap 28 again). A
    reservation must leave the pad itself reachable; when it cannot, the via
    is not worth the route.
45. **A stitching grid joins only pour the other layer's main plane
    overlaps.** A piece enclosed by tracks on both layers stays tracks-only:
    at 2.5 mm pitch, 237 stitching vias joined none of three such pieces.
    `carrier-measure plane` tells them apart; only the route can fix them.
46. **LCSC can number a package's pads differently from KiCad.** KiCad's
    PowerDI3333-8 draws the drain and its fingers as one pad 5; LCSC numbers
    the fingers 5-8 and the drain pad 8, and `lcsc_rotation` found no turn.
    Derive it on chosen pads with a map, `--pads 1,2,3,4,5=8`; the entry
    records `pads`. The pins alone would not do: collinear pads cannot tell a
    turn from a mirror, so `--pads` refuses them. Two shapes have no `--pads`
    answer: a footprint that repeats pad numbers (KiCad's TS-1187A tact
    switch is 1, 1, 2, 2 where LCSC numbers 1-4, and `kicad_pads` keeps one
    centre per number), and one whose only distinct pads are collinear plus
    unnumbered mounting pads (the MSK12C02 slide switch). Check those by hand
    and leave `lcsc` off the entry until the rotation entry exists.

### Found enforcing JLCPCB's silkscreen rules (#877)

47. **JLCPCB deletes silkscreen within 0.15 mm of a pad, and KiCad's defaults
    pass it.** `silk_over_copper` fires only on actual overlap of the mask
    opening. The carriers' `.kicad_dru` rule "silkscreen 0.15 mm clear of
    pads" (`(condition "B.Type == 'Pad'")`, `silk_clearance` 0.15 mm) moves
    that limit out by 0.15 mm (measured to 0.01 mm in KiCad 10.0.6 and
    10.0.7). Two lookalikes do nothing. Any `(layer ...)` clause disables the
    rule, and the board-wide `min_silk_clearance` spaces silkscreen only from
    silkscreen. Turned on, the rule found 43 deletions on the robocar boards,
    among them the labels R18-R24 against U6's pins and the XIAO outline
    over J2.
48. **Clipping a footprint's silkscreen makes it differ from its library
    copy.** `build_board.clip_silk` cuts lines back from every pad on the
    board, its own or another footprint's, and moves each original to
    Cmts.User; `Remove()` is off-limits (trap 1). Any change of geometry or
    layer raises `lib_footprint_mismatch`; a stroke width change does not,
    which is why widening (trap 41) never did. DRC exclusions do not help,
    because kicad-cli counts excluded findings in its exit code. So
    `clip_silk` marks each footprint it edits with a hidden `Silk` field,
    which KiCad does not compare, and `kicad.check` excuses the mismatch only
    on marked footprints.
49. **A silkscreen fix need not reroute the board.** `just
    <project>::carrier-silk` (`build_board.py --silk-only`) re-applies
    `[silk]` positions, sizes and unique text positions, and clips the
    footprint lines, on the routed board. Tracks, vias and zones stay byte
    for byte. A second run clips nothing more, so the `Silk` field appears
    only on a board that was clipped from its committed state. Re-applying a
    position also rounds away KiCad's 1 nm drift (−3.900001 → −3.9).

### Found checking LCSC parts against JLCPCB

50. **JLCPCB's parts search is fuzzy.** `selectSmtComponentList` with
    `"keyword": "C17414"` also lists C20622501, extended and out of stock,
    whose MPN is "C17414"; an unknown number lists 41 unrelated parts. The
    first hit, or any hit, is not the part: `kicad.lcsc` keeps only the entry
    whose `componentCode` matches, and asks the detail endpoint (which
    answers `{"code": 400, "data": null}` under HTTP 200 for an unknown
    number) for everything but `preferredComponentFlag`.
51. **`componentStatus` "yes" does not mean orderable.** On 2026-11-06
    C9900140859 answered `componentStatus` "yes" with `isBuyComponent` "0"
    and `noBuyReason` "This product is no longer manufactured.". Gating on
    the status alone passes a discontinued part that still has leftover
    stock: `kicad.lcsc` fails `isBuyComponent` "0" and prints the reason.

### Found vendoring new parts (PR #988)

52. **`just kicad::vendor` refuses when the installed KiCad is newer than
    `vendor.TAG`.** On 2026-11-07 the Mac had 10.0.7 and the repo pins
    10.0.6, and `--check-installed` showed two vendored footprints differ
    between them (Fiducial_1mm_Mask2mm, TQFN-16), so copying from the install
    is not safe. Call `kicad.vendor.update(footprint_roots=[root])` instead,
    with `root` holding the already-vendored footprints plus the new ones
    fetched from `gitlab.com/kicad/libraries/kicad-footprints/-/raw/10.0.6/`.
    Symbols cannot come from the tag the same way: kicad-symbols 10.x keeps
    one file per symbol (`<Lib>.kicad_symdir/<Sym>.kicad_sym`) and the
    single `<Lib>.kicad_sym` URL is a 404. Take symbols from the install only
    when `--check-installed` reports no symbol drift, and compare each new
    block against its `.kicad_symdir` file at the tag.
53. **Many KiCad parts are derived symbols, and the vendor rejects
    `extends`.** `Transistor_FET:2N7002` extends `Q_NMOS_GSD`. Name the base
    symbol in the registry entry (same pins, G 1, S 2, D 3) and say so in
    `source`.

### Found splitting the robocar carrier into two PCBs (#1280)

54. **Freerouting says which pad it could not reach, at DEBUG.** Its `--help`
    lists no log option, but `--logging.console.level=DEBUG` is a
    `LoggingSettings` key like `--gui.enabled=false`. route_board.py passes it,
    keeps the whole log in `output/freerouting.log` and prints each
    `insert trace failed ... target: (x,-y)` point in board mm
    (`dsn.stuck_points`; DSN units are 0.1 µm, y negated). When a whole net
    collapses (trap 28), read that line before bisecting the DSN: on #1280 it
    named J20's GND pad, walled in by a fixed VBUS track's keepout, where 36
    GND connections had gone unrouted. `MazeSearchEngine.init: Failed - no
    accessible expansion doors found` is the same story from the start side.
55. **A 0.3 mm signal track cannot pass between 2.0 mm-pitch header pins.**
    `PinHeader_2x08_P2.00mm` pads are 1.35 mm, leaving 0.65 mm: 0.3 mm plus
    0.2 mm clearance each side needs 0.7. Every lower-row pin then has to
    leave round the header's ends, and the robocar head 2 left one or two
    signals unrouted in 25 attempts until `[signal] track_um = 200`. Check
    the pad gap against track + 2 x clearance before blaming placement.
56. **A footprint can stack two pads of one net at one spot** (a USB-C
    receptacle's A1/B12, A12/B1, A4/B9, A9/B4). Each is its own pin to
    Freerouting. While route_board.py dropped a fixed track's first pad
    (trap 27) it had to drop both; with the fixed wiring read, a track over
    the spot joins both.
57. **A carrier split in two needs `side` on both board.toml files.** With
    `[project] side = "main"` the netlist keeps every part but the heads;
    `side = "head"`, in a project with no hardware.toml, needs `join` naming
    the directory whose hardware.toml it reads, and the cable's `head_parts`
    for the heads' other parts (mounting holes). A net with nodes on both
    PCBs fails the build.
58. **A keepout boxes in pads as surely as copper does.** Before trap 27's
    fix, each fixed segment's keepout (half the width plus the widest class
    clearance, flush with the ends) left 0.6 mm of an 0805 pad, and R15's
    gate pad, C10's VBUS and GND pads and C19's BTST pad were each walled in:
    "the maze search algorithm could not be created" or "no connection was
    found". `[[route_keepouts]]` still do this: a B.Cu keepout also blocks
    through vias (Freerouting's `ObstacleArea.isObstacle` takes any foreign
    trace or via), so a pad under one leaves on the other layer or not at all.
    Read route_board's `Freerouting:` lines, which name the pads, and notch the
    keepout where a fixed track has to cross it.

59. **Freerouting joins a fixed wire to a pin only on the pin's exact
    centre, and KiCad's DSN export misses it by half a micrometre.** It writes
    wire coordinates in whole µm but places pins from their image offsets,
    which sit on half µm (an 0805's pads at ±1.0375 mm), and board.toml's
    points are rounded to 0.01 mm besides. Every fixed track then counts as
    unconnected: Freerouting never finishes a route, so its optimizer never
    runs, and the board comes back with dangling vias and pockets of pour
    (#1280: with the charger's fixed tracks, 1-20 unconnected items and
    dangling vias across the whole board in every route). build_board.py ends
    each fixed track on its pad's centre with a short segment inside the pad,
    and `dsn.snap_wire_ends` moves each wire end within 2 µm of a pin of its
    net onto the centre exactly. The test is Freerouting's DEBUG
    `Pass #1: N incompletes` block, per net: CHG_PMID's five fixed-joined pads
    read 4 incompletes before and 0 after. Two fixed tracks join only where
    they share a corner: a branch that meets another track mid-segment counts
    as unconnected too (#1280, REGN's lane to R28), so give the polyline a
    corner at every junction.
60. **Freerouting's "could not be routed" list describes its final board,
    not the best one it saves.** The two differ after a stagnation stop; read
    the DEBUG `Pass #N: ... incompletes` block of the pass whose score the
    SES restored, or KiCad's DRC on the imported board.
61. **KiCad 10's starved-thermal check reads the `.kicad_pro`'s
    `min_resolved_spokes`, not a `.kicad_dru` rule, and the zone fill ignores
    custom `zone_connection` rules.** The report says "zone min spoke count 2"
    beside a rule that asks for 1; no rule form changed it (#1280). Set the
    project value. For pads that should join the pour solid, list them in
    `[pour] solid_pads` (route_board.py sets the pad's own zone connection).
    A thermal that reaches only a pocket of fill fenced off by routed tracks
    is "connected to isolated island": route_board.py's island vias tie each
    such pocket to the other layer's plane where a via fits inside the fill.
62. **A pad's number is not its identity.** A SOT-223's tab and its middle
    pin are both pad 2. #1301's `check_track_ends` keyed pads by
    (ref, number), so balancebot's tab-to-pin track read as "starts and ends
    on one pad" and every re-route stopped before Freerouting ran (#1314).
    Key a pad by its position too, wherever two pads are compared.
63. **The perfboard tests read a frozen copy of balancebot, not the live
    project.** `test_render.py` lays out `testfixture.shared()`, so a label
    collision in the live `perfboard.svg` can pass every test. Render the
    live drawing and look at it (`kicad.perfboard <project>`, then a
    headless screenshot). `just <p>::perfboard` also recompiles the step
    guide's PDF through `mise exec typst@<version>`; without mise that step
    fails after the SVG is written, the PDF stays stale, and CI's
    "Verify committed PDFs match Typst source" catches it. Typst 0.15.0 from
    its GitHub release, with the recipe's flags, reproduces main's PDF byte
    for byte (#1314).

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
