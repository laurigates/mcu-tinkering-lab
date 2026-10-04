# Schematics

Text-defined circuit schematics for MCU Tinkering Lab projects, rendered with
[Schemdraw](https://schemdraw.readthedocs.io/).

Each circuit lives in `circuits/` as a small Python module that imports
reusable component blocks from `components.py` and returns a
`schemdraw.Drawing` from a `draw()` function. `render.py` discovers them and
writes SVG + PNG to `images/`.

## Layout

```
docs/schematics/
├── circuits/              # One .py per circuit (defines draw())
│   └── gamepad_synth.py
├── components.py          # Reusable chip/breakout factories (ESP32, MAX98357A, ...)
├── routing.py             # Manhattan auto-router (Router) used by circuits for nets
├── hardware_nets.py       # MCU net endpoints by pin role, from a hardware.toml join
├── test_hardware_nets.py  # pytest suite for hardware_nets.py
├── test_routing.py        # pytest suite for routing.py
├── metrics.py             # Routing-quality metrics per circuit (crossings, ...)
├── test_metrics.py        # pytest suite for metrics.py
├── guide_deps.py          # Which Typst documents embed a rendered image (render-all)
├── test_guide_deps.py     # pytest suite for guide_deps.py
├── conftest.py            # Shared fixture: each real circuit routed once per session
├── images/                # Generated SVG + PNG (committed so GitHub renders them)
├── render.py              # Batch-render every circuit in circuits/
├── justfile               # `just schematics::render`, `::clean`, ...
└── pyproject.toml         # uv-managed deps: schemdraw, cairosvg, pytest (dev)
```

## Quick start

```bash
# From repo root
just schematics::render-all          # renders every circuit + recompiles guides that embed one
just schematics::render              # renders every circuit
just schematics::render-one gamepad_synth
just schematics::clean
```

Or directly:

```bash
cd docs/schematics
uv sync
uv run python render.py
```

## Adding a new circuit

1. Add any missing component factories to `components.py`. Keep pin sets
   minimal and name pins as the firmware does.
2. Create `circuits/<name>.py` with a `draw() -> schemdraw.Drawing` function.
   Reference `circuits/balancebot.py` as a template: place every
   component first, then its power/ground tags, then create a `Router(d)`
   and call `.wire(a, b)` for each point-to-point net and `.finish()` once
   after the last one (see "Routing" below). Other hand-drawn local stubs
   (LED branches, bus fan-outs) may come before or after the nets.
3. Run `just schematics::render-all`. The SVG + PNG land in `images/`, and
   any build guide that embeds one is recompiled (see "Embedded in a build
   guide" below).
4. Link the rendered PNG from the project's README or WIRING.md:
   ```markdown
   ![Wiring](../../docs/schematics/images/<name>.png)
   ```

## Routing

`routing.py` provides `Router`, an obstacle-aware Manhattan (orthogonal)
auto-router — point it at a drawing and ask it to connect two pin anchors,
and it grid-searches an orthogonal path around every component already
placed, instead of the diagonal or hand-tuned-per-wire lines that
`schemdraw.elements.Wire` alone produces (schemdraw has no routing/collision
model of its own: see the module docstring in `routing.py` for the full
rationale). This is similar in spirit to the auto-routed connectors in
[quick-connections](https://github.com/niknah/quick-connections), adapted to
schemdraw's static SVG output.

```python
from routing import Router

router = Router(d)
router.wire(esp.GPIO5, amp.BCLK, net="i2s")
router.finish()
```

- **Route, then finish**: `.wire(...)` routes a net and records it — it
  marks the net's cells at once, so later nets still steer around it — but
  draws nothing. `.finish()` adds every recorded wire to the drawing, in
  the order `.wire(...)` was called, and belongs straight after the last `.wire(...)`: where it
  is called is where the wires sit in the SVG's paint order. `.wire(...)`
  returns a handle whose `.points` is the routed polyline and whose
  `.element` is the drawn `Path` once `.finish()` has run. The split exists
  because hops, nudging and net ordering depend on the finished set of
  wires (#492, ADR-023). `.finish()` also picks the *routing* order (#494):
  the router is greedy, so whichever net routes first takes the best path
  and the rest detour round it. It re-routes the recorded nets under a
  fixed list of candidate orders (`routing.ORDERINGS`: as written,
  shortest-first, longest-first, by net class, reversed), scores each
  finished set — a weighted sum of collinear overlaps (1000 each), tight
  parallel pairs (20), crossings (4) and length, by `metrics.py`'s own
  rulers — and keeps the lowest, the authored order winning any tie.
  `router.ordering` names the one chosen. The score sees routed wires only: a tight pair or crossing
  against a hand-drawn lead is not counted, so a future ordering could
  trade a wire-wire pair for a wire-lead one unseen. All three circuits
  measured 0 wire-lead overlaps and 0 wire-lead tight pairs at #494, and
  `test_no_routed_wire_runs_on_or_beside_a_lead_in_real_circuits` holds
  them there. The drawing order does not change with it, so a handle's
  `.points` can differ after `.finish()` but its place in the SVG cannot.
  A circuit that forgets `.finish()` fails `render.py`, `metrics.py` and
  the tests with an error naming it, rather than rendering with no wires.

- **Crossings and junctions are marked by `.finish()`** (#493). Where two
  wires cross, the horizontal one hops over the vertical one with a small
  arc — KiCad's `ShouldHopOver` convention, so exactly one of the pair
  hops. No hop is drawn at a wire's own endpoint (a T or a shared end is a
  connection) or where more than two wires meet (that is a junction). A
  filled dot goes on every junction: three or more wire ends at one point,
  or one wire ending on another's interior. Hand-drawn `elm.Wire`/`elm.Line`
  leads already in the drawing count too — a routed wire hops a lead on
  either axis, since the lead is never redrawn — so call `.finish()` *after*
  the hand-drawn stubs; a lead added later is invisible to it, and
  `test_routing.py` fails on any crossing it left unmarked. A lead ending on
  a `Vdd`/`Ground` tag ends on the tag, so only that end is discounted: a
  wire running through the point is not dotted, but a real T there is.
  Hops are two quarter-circle cubic Béziers
  (`HOP_RADIUS = 0.15`), never the SVG `A` arc: schemdraw's matplotlib
  backend turns `A` into a MOVETO. Crossings closer than a hop diameter —
  adjacent grid columns — merge into one bridge. `.finish()` may run more
  than once; a wire an earlier call drew is redrawn in place if a later
  wire or lead crosses it. The marks must also *read*: `test_routing.py`
  fails on a hop within `HOP_RADIUS + JUNCTION_RADIUS` of a dot (the dot
  hides the arc and draws a connection that is not there), a hop near
  another wire's corner or shrunk by its own segment end, a dot joining
  two net classes, or two hand-drawn leads crossing (nothing would hop
  them). Hand-drawn leads placed before `Router(d)` routes also count as
  occupied, so a net is not drawn on top of one.
- **Place components, then route**: call `Router(d)` and every `.wire(...)`
  *after* every component in the circuit is placed, so each net has full
  obstacle awareness. A wire routed before a later component exists can't
  avoid it.
- **What counts as an obstacle**: any placed component's bounding box
  (`Ic`, `Motor`, `Speaker`, `Resistor`, `LED`, ...). `Wire`, `Line`
  (and `Arrow`, a `Line` subclass), `Vdd`, and `Ground` are excluded — they're
  leads and single-terminal annotation symbols, not physical bodies a real
  wire needs to route around. A wire drawn across a power or ground tag
  still reads as a connection to the rail, so the search charges
  `tag_penalty` per grid point on or inside a tag's body (label excluded)
  that the wire is not wired to — every tag except one whose stub leads to
  the wire's own end pin (#591). A soft cost, so a pin beside its chip's own
  GND tag stays reachable; and like every obstacle it only sees tags already
  drawn, which is why tags go in before `Router(d)`.
- **Real components placed after routing** (e.g. a resistor/LED branch
  hanging off a GPIO the router doesn't touch) aren't obstacles for nets
  routed earlier — if a later-placed real component's footprint would cross
  an already-routed wire, place that component (and its branch) *before*
  the `Router(d)` call instead, same as any other component.
- **Two-terminal nets only**: `Router.wire()` connects exactly two points.
  A multi-endpoint bus (e.g. one GPIO driving a vertical trunk that fans out
  to several chips' pins) is a T-junction, not a point-to-point net, and
  stays hand-drawn with `elm.Wire(...)` — see the nENABLE trunk in
  `circuits/balancebot.py` for an example. Colour it with
  `net_color("<class>")` and `.finish()` dots its junctions.
- **Tuning**: `Router(d, grid=0.25, clearance=0.3, stub=0.75,
  turn_penalty=4.0, overlap_penalty=6.0, tag_penalty=20.0)` — defaults suit this repo's
  `unit=2.0`-scale circuits. Lower `turn_penalty` allows more bends in
  exchange for tighter routing; raise `clearance` if a wire hugs a chip
  outline too closely. `overlap_penalty` is charged in full for running on
  top of an earlier wire and a quarter of it per earlier wire one grid step
  beside, so raising it spreads parallel nets apart at the cost of length
  and crossings. Nothing is charged two steps away: that is the spacing a
  bus settles into, and no circuit improved when it was taxed (#494). Do
  not raise `clearance` to separate *wires* — it pushes more of them
  through component bodies instead (#491).
- `test_routing.py` covers the router directly (orthogonality, obstacle
  avoidance, fast failure on an unreachable goal) and re-checks every wire
  every real circuit actually draws. Run every suite with
  `just schematics::test` (or `uv run --group dev pytest`) after touching
  `routing.py` or any `circuits/*.py`; CI runs the whole directory.
  Tests that only *read* a real circuit take the `real_circuits` fixture in
  `conftest.py`, which routes each circuit once per session and fails any
  test that changes a shared drawing or its metrics; a test that must route
  afresh (determinism, a monkeypatched `Router` default) routes its own copy
  with `draw_circuit(load_circuit(...))` or in a fresh interpreter (#594).
- **Measuring a router change**: `metrics.py` reports, per circuit, total
  wire length, length inside component bodies (own and foreign), length
  over power/ground tags the wire is not wired to, crossings,
  tight parallel pairs, collinear overlaps, junctions (routed-wire ends
  only) and the hops and dots actually drawn — each defined
  exactly in its module docstring and pinned by `test_metrics.py`. Run
  `just schematics::metrics` (`--json` for machine output) before and after
  a routing change and quote both in the commit, rather than judging the
  SVG by eye.

## Conventions

- **Pin ordering**: schemdraw renders L/R side pins *bottom-to-top* in the
  order given. Component factories list pins bottom-first so visual order
  matches the intuitive top-to-bottom reading order.
- **Alignment**: chips that connect to each other keep matching pin counts on
  facing sides so default spacing aligns them when placed at the same
  y-center. E.g. both the ESP32-S3-Zero and MAX98357A factories expose 4 pins
  on the left and 3 on the right.
- **Labels**: factories don't set a center label — individual circuits add
  `.label('Name', loc='bot', ofst=0.4)` to avoid collisions with pin labels.
- **Physical layouts** (ADR-023 stage 6, #495): the two conventions above
  describe `layout="schematic"` symbols, whose pin order was chosen for the
  router. A board with a vendor-sourced layout also has a `layout="physical"`
  symbol — currently the XIAO ESP32-S3 Sense, TCA9548A, PCA9685, TB6612FNG
  and MAX98357A — drawn by `components.physical_module()`: every pad of the
  real header, on the real edge, in the real order as seen from the
  component side, `PITCH = 1.0` drawing units apart (true 2.54 mm would be
  too tight for the router). The pad list is read from
  `docs/reference/boards/<board>.md` through `tools/hardware/layout.py`,
  never typed into `components.py`, and each breakout page records the
  vendor board file it was read from with `tools/breakout-pinout.py`
  (`.claude/rules/board-layout-from-vendor-files.md`). A name the board
  repeats is anchored by edge and position (`tb["GND.L3"]`,
  `pca["SCL.R3"]`); the XIAO's GPIO pads are anchored by firmware name
  (`xiao.GPIO5`). Do not rotate or mirror a physical symbol — place the
  other parts around its pin order instead. The default stays `"schematic"`,
  so a circuit changes only when it opts in. A new board gets a page under
  `docs/reference/boards/` (a `Pin | Side | Pos` table) before it gets a
  physical symbol.
- **Colors — by net class, never by literal**: every `router.wire()` passes
  `net=` one of the classes in `routing.NET_COLORS`, which alone decides the
  colour; there is no `color=` argument. Hand-drawn leads that carry a net
  use `net_color("<class>")` rather than a literal, so they cannot drift from
  the palette. Colour is by class rather than by graph colouring (ADR-023): a
  graph colouring reassigns colours whenever an unrelated net moves, turning
  every render into a large SVG diff. The palette is Okabe-Ito, which stays
  distinguishable under the common colour-vision deficiencies (its yellow is
  left out — too faint on white):

  | Class | Colour | For |
  |---|---|---|
  | `power` | vermillion `#D55E00` | supply rails and their tags |
  | `ground` | black `#000000` | ground leads and tags |
  | `i2c` | blue `#0072B2` | SDA/SCL, including behind a mux |
  | `i2s` | bluish green `#009E73` | BCLK/LRC/DIN |
  | `pwm` | orange `#E69F00` | PWM duty outputs (PCA9685 channels, LEDC tones) |
  | `sensor` | reddish purple `#CC79A7` | sensor trigger/echo/interrupt lines |
  | `signal` | sky blue `#56B4E9` | any other digital GPIO (enable, step/dir) |
  | `load` | grey `#666666` | driver output into a motor coil or speaker |

  A wire routed with no `net=` is drawn black and fails the tests for the
  real circuits. The one literal left is the gray `SD_MODE` annotation arrow
  in `robocar_unified.py`: it marks a pin left floating, not a net.
- **Output format**: SVG is the source of truth — schemdraw writes
  byte-deterministic SVG, so `git diff` on it is reliable signal that the
  rendered output is out of sync with `circuits/<name>.py`. The PNG is
  generated alongside via cairosvg for easier embedding in READMEs, but its
  bytes vary with the host's cairo encoder version, so it's *not* a reliable
  diff target.

## Source of truth

Each circuit file cites the authoritative wiring document at the top (usually
the project's `WIRING.md`). Keep the schematic and that document in sync when
pins change.

A project with a `hardware.toml` (ADR-021) does not transcribe its MCU wiring.
`robocar_unified.py` loads the join live at render time and asks
`hardware_nets.JoinedNets` for each net by pin role —
`router.wire(*nets.ends("I2S_BCLK_PIN"), net="i2s")` — so the XIAO pad comes
from `pin_config.h` and the board reference, and the part pin from the
`[[nets]]` entry. Chip addresses, the PWM frequency and the PCA9685 and mux
channel numbers in its labels come from the same headers. Placement, net order
and net colour stay in the circuit. After the last MCU wire,
`nets.check_all_drawn()` fails the render for any `[[nets]]` entry the drawing
left out, so a net added to `hardware.toml` cannot ship a schematic without it.

## Freshness check

`.github/workflows/schematics-check.yml` re-renders all circuits on every PR
that touches `docs/schematics/**` — or an input a circuit reads at render
time: the board references in `docs/reference/boards/`, `tools/hardware/`,
and the `main/pin_config.h` and `hardware.toml` that robocar-unified's wiring
and labels come from — and fails if `images/*.svg` would change. A circuit that starts
reading a new file needs that file on the workflow's trigger paths, or the
SVG can go stale with no check running.
The workflow is SVG-only on purpose (PNG drift is encoder-version noise, not
content drift); the workflow surfaces PNG diffs as `::notice` only.

When the check fails, run `just schematics::render-all` locally and commit
the regenerated SVG and PNG, plus any PDF it recompiled.

## Embedded in a build guide

A Typst document can embed a rendered PNG — the robocar-unified build guide
embeds `images/robocar_unified.png` — and its committed PDF then carries the
image's bytes. Re-rendering that circuit leaves the PDF stale, and
`build-guide-check.yml` fails on it even though this directory's own check
passes (issue #595).

`just schematics::render-all` closes that gap: it renders, then recompiles
every document whose source references `docs/schematics/images/`, each through
its project's own `build-guide` recipe (which pins the Typst CLI and the flags
CI uses). The documents are found by `guide_deps.py` reading the sources, so a
new guide that embeds a schematic needs no edit here. Plain
`just schematics::render` renders only, and afterwards names any project whose
PDF it has just made stale. The rule that governs the guard is
`.claude/rules/build-guide-drift-guard.md` § 1c.
