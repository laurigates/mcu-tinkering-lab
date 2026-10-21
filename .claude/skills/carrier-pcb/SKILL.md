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
- `packages/robocar/unified/hardware/carrier/`: a main board and three head
  boards on one PCB, split by `[[keepouts]]` bands under `[silk] cut_lines` saw
  lines, with a `[pour]` region per ground net (`GND`, `HEAD_GND`,
  `HEAD2_GND`, `HEAD3_GND`). The spare heads come from `[[cables]]
  spare_heads` (ADR-021). Two head IDC headers sit on the back face
  (`[placement]` entry `[x, y, rot, "back"]`, mirrored about its anchor, so the
  pads run the other way: at rotation 90 they run toward -x)
- `packages/robocar/unified-integrated/hardware/carrier/`: the same carrier with
  the breakout chips and a buck converter on the board, for comparison.
  `just robocar-unified-integrated::compare` runs `kicad.area`, the
  courtyard-area comparison against the socketed carrier. A carrier with no
  board yet takes `stage = "schematic"` in `board.toml`: boardcheck, the stamp
  and the local DRC skip it. It also shows the layout features the router
  does not provide: `[[tracks]]` (fixed tracks build_board.py lays down
  locked, here a 1.5 mm 5V trunk, the buck's GND and an exposed pad's ground
  ties), `[fanout]` (a via beside every SMD pad of a poured net, added after
  routing; `required` pads fail the route without one) and `[pour]` regions
  with their own `layers` and `priority`. It is also the mouse-bite carrier:
  `[panel] mode = "mousebites"` (#809, `kicad.panel`) turns the cut lines into
  Edge.Cuts slots and board-only MB<n> footprints of NPTH holes, all inside
  the keepout bands; `boardcheck` compares both with board.toml
- `just kicad::setup` (`tools/kicad/setup.sh`): KiCad 10.0.6 on Ubuntu 24.04,
  Freerouting 2.5.0 and a Java 25 JRE in `~/.cache/freerouting`
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
   silkscreen size). Then run the `pcb-reviewer` agent
   (`.claude/agents/pcb-reviewer.md`) for an adversarial review against the
   datasheets, and `.claude/rules/power-stage-checklist.md` for any regulator.
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
   and CPL. `just kicad::check` runs what CI runs. The CPL adds each LCSC
   part's checked rotation offset (trap 32); a new LCSC number needs an entry
   in `docs/reference/jlcpcb-rotations.toml` first.

## Adding a registry entry

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

### Found laying out the robocar-unified carrier (#750)

20. **A module socket can accept the module upside down.** Asymmetric column
    spacing rules out a rotated or left-right-mirrored module, but a top-bottom
    flip (component side down) keeps every column on its x and reverses the
    rows. On the CJMCU-2317 that puts its VCC on GND and 3V3 into its I/O
    pins. Every check passes, because the error is in the person fitting the
    module. Any socket for a module labelled on one face needs a
    silkscreen orientation mark and pin names beside the pads (#803).
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

24. **Freerouting gives each net one track width**, so a net class can be no
    wider than the narrowest pin pitch on any of its nets allows: 0.25 mm at a
    0.5 mm-pitch TQFN, 0.45 mm at 0.65 mm-pitch SSOP. A power net that reaches a
    fine-pitch pin caps the whole rail; give that chip its own net (the
    integrated carrier's AMP_5V behind a ferrite bead) rather than thinning
    the rail. Freerouting also necks tracks below the class width at fine-pitch
    pads, so such a board needs a smaller `min_track_width` than 0.2 mm.
25. **`kicad-cli` cannot write an ERC exclusion.** KiCad matches an exclusion
    only against its own serialisation of the marker, and a hand-built key
    (`pin_to_pin|x|y|uuid|uuid|...`) loads as a separate excluded marker while
    the real one still fails. For a symbol's pin-type quirk, change the
    project's ERC `pin_map` instead, and check that no other pin has that type.
26. **The KiCad PPA keeps only the newest 10.0.x**, but Launchpad still serves
    superseded builds under the PPA's `+files/` URL; `tools/kicad/setup.sh`
    pins 10.0.6's packages by sha256.
27. **Freerouting 2.5.0 drops a DSN's `(wiring ...)` section**, fixed and
    protected wires included, and routes other nets straight through a
    `[[tracks]]` track. route_board.py walls each segment off with a DSN
    `keepout`, flush with the polyline's ends so the end pads' centres stay
    reachable (a keepout over a pad centre leaves that pad unroutable), and
    drops the pad under each track's first point from the DSN netlist
    (`dsn.drop_pins`): the track already joins it to the pad under its last
    point, and both ends must sit on pads of the track's net.
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
    line (a whole-board pour meets every line) and silkscreen text under
    0.8 mm. JLCPCB prints 1.0 mm tall, 0.15 mm stroke legibly; build_board.py
    keeps reference strokes at 0.16 mm or more.

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
