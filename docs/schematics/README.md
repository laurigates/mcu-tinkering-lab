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
├── test_routing.py        # pytest suite for routing.py
├── images/                # Generated SVG + PNG (committed so GitHub renders them)
├── render.py              # Batch-render every circuit in circuits/
├── justfile               # `just schematics::render`, `::clean`, ...
└── pyproject.toml         # uv-managed deps: schemdraw, cairosvg, pytest (dev)
```

## Quick start

```bash
# From repo root
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
   Reference `circuits/gamepad_synth.py` as a template: place every
   component first, then create a `Router(d)` and call `.wire(a, b)` for
   each point-to-point net (see "Routing" below), then add any hand-drawn
   local stubs (power tags, LED branches, bus fan-outs).
3. Run `just schematics::render`. The SVG + PNG land in `images/`.
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
router.wire(esp.GPIO5, amp.BCLK, color="steelblue")
```

- **Place components, then route**: call `Router(d)` and every `.wire(...)`
  *after* every component in the circuit is placed, so each net has full
  obstacle awareness. A wire routed before a later component exists can't
  avoid it.
- **What counts as an obstacle**: any placed component's bounding box
  (`Ic`, `Motor`, `Speaker`, `Resistor`, `LED`, ...). `Wire`, `Line`
  (and `Arrow`, a `Line` subclass), `Vdd`, and `Ground` are excluded — they're
  leads and single-terminal annotation symbols, not physical bodies a real
  wire needs to route around.
- **Real components placed after routing** (e.g. a resistor/LED branch
  hanging off a GPIO the router doesn't touch) aren't obstacles for nets
  routed earlier — if a later-placed real component's footprint would cross
  an already-routed wire, place that component (and its branch) *before*
  the `Router(d)` call instead, same as any other component.
- **Two-terminal nets only**: `Router.wire()` connects exactly two points.
  A multi-endpoint bus (e.g. one GPIO driving a vertical trunk that fans out
  to several chips' pins) is a T-junction, not a point-to-point net, and
  stays hand-drawn with `elm.Wire(...)` — see the nENABLE trunk in
  `circuits/balancebot.py` for an example.
- **Tuning**: `Router(d, grid=0.25, clearance=0.3, stub=0.75,
  turn_penalty=4.0, overlap_penalty=6.0)` — defaults suit this repo's
  `unit=2.0`-scale circuits. Lower `turn_penalty` allows more bends in
  exchange for tighter routing; raise `clearance` if a wire hugs a chip
  outline too closely.
- `test_routing.py` covers the router directly (orthogonality, obstacle
  avoidance, fast failure on an unreachable goal) and re-checks every wire
  every real circuit actually draws. Run it with
  `uv run --group dev pytest test_routing.py` after touching `routing.py` or
  any `circuits/*.py`.

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
- **Colors**: signal buses use `steelblue`; power/ground use default black.
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

## Freshness check

`.github/workflows/schematics-check.yml` re-renders all circuits on every PR
that touches `docs/schematics/**` and fails if `images/*.svg` would change.
The workflow is SVG-only on purpose (PNG drift is encoder-version noise, not
content drift); the workflow surfaces PNG diffs as `::notice` only.

When the check fails, run `just schematics::render` locally and commit both
the regenerated SVG and PNG.
