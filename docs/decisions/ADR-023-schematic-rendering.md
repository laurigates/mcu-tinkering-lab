# ADR-023: Schematic Rendering — Stay on Schemdraw, Draw Physical Footprints, Mark Every Crossing

**Status**: accepted
**Date**: 2026-08-30
**Source**: conversation 2026-08-30
**Confidence**: 7/10

---

## Context

`docs/schematics/` renders every circuit with schemdraw plus a hand-written
Manhattan auto-router (`routing.py`). The rendered SVG is committed and
drift-guarded by `schematics-check.yml`, and the PNG is what `WIRING.md` embeds
and the build guide prints. It is the drawing someone actually wires from.

Four defects, each observed in the current output rather than inferred:

**Wires pass through chip bodies at both ends of a net.** `Router.wire()`
excludes, for the *entire* A\* search, every obstacle box containing either
endpoint (`routing.py:303-308`). The exclusion exists for a real reason — a
second box overlapping the goal point would make the goal unreachable outright —
but it is far too broad: the box is dropped for the whole search, not just near
the pin, so a wire may cut straight across the component it terminates on.
Measured 31.99 units of wire inside component bodies.

**Three near-parallel verticals cannot be told apart.** The router charges
`overlap_penalty` for reusing a cell along the same axis, and that penalty never
fires. `_astar` anchors its lattice at each net's own start point (`ox, oy =
start`, `routing.py:181`) while `_mark_occupied` keys cells by absolute
coordinate (`round(x / grid)`). Two nets therefore sample two lattices offset by
a fraction of a grid step, their rounded keys rarely collide, and the parallel
term is inert. The rendered SVG is byte-identical at `overlap_penalty` 6, 50 and
1000, and twelve different net orderings produced identical total wire length.

**A crossing and a connection are drawn with the same mark.** Nothing in the
renderer draws a hop-over or a junction dot, so two wires that merely cross look
exactly like two wires that join.

**The XIAO symbol does not resemble the board.** `xiao_esp32s3_sense()` declares
12 pins as 3 left / 9 right with `size=(3.5, 10)`. The physical header is 14
positions, 7 per side, with the power pins on the right. D6/D7 (GPIO43/GPIO44)
are absent from the factory entirely. Somebody counting pads on the real board
cannot count them on the drawing.

## Decision

Stay on schemdraw. Fix the router, then draw crossings, then make the symbols
physical.

1. **Keep schemdraw and `routing.py`.** No layout engine, no EDA tool. The
   alternatives all either move components (which ADR-021 § *What stays
   hand-authored* explicitly protects) or cost more than the defect.
2. **Repair the two router defects** — test the stub points rather than the raw
   endpoints when building the obstacle list, and give the A\* lattice a single
   global origin so `overlap_penalty` becomes load-bearing.
3. **Split `Router.wire()` into record-then-`finish()`.** Hops, nudging and net
   ordering are properties of the finished set of wires; `wire()` currently
   routes and draws in one call, so none of them can be computed.
4. **Mark every crossing** — hop arcs where one wire jumps another, junction
   dots where they join, and colour by net class (power, ground, I2C, PWM, I2S,
   sensor).
5. **Draw stylised-physical footprints** — pins in true header order and on the
   true side, at `PITCH = 1.0` drawing units per pad, with pin data read from
   `docs/reference/boards/<board>.md` rather than re-entered.

### Sequencing is a constraint, not a preference

Fixing the obstacle predicate alone raises crossings from 19 to 31 and tight
parallel pairs from 2 to 13: wires no longer take the shortcut through the chip,
so they route around it and meet each other instead. Physical pin order adds
more of both, because the pins are no longer ordered for the router's
convenience.

So the crossing visuals (stage 4) must ship before physical pin order (stage 6).
Landing them in the other order makes the diagrams harder to read at every
intermediate commit.

## Consequences

**Pin order stops being chosen for the router.** `components.py`'s conventions
are explicit that it is today — *"Chips that connect to each other keep the same
pin count on facing sides so default auto-spacing aligns the pins"* — and at
least six factories carry a comment justifying their order on routing grounds:
*"SCL is listed above SDA … the two bus wires run parallel instead of crossing"*
(line 88), *"listing GPIO9/8/7 in that order keeps the three wires parallel"*
(line 94), and similar at lines 135, 261, 291 and 340. Physical order forfeits
all of it.

**The cost of that forfeit is real but unmeasured.** The crossing and
parallel-pair figures quoted above come from a prototype harness that was not
preserved, so they are the right order of magnitude and not a baseline. Stage 5
re-measures against the post-stage-2 tree rather than carrying these numbers
forward.

**`overlap_penalty` starts mattering, which is a behaviour change.** It is
currently inert, so every existing rendered SVG was produced with the parameter
doing nothing. Repairing it moves wires. Each stage therefore re-renders and
commits, and `schematics-check.yml` diffs the result.

**Colour is assigned by net class, not by graph colouring.** A graph-colouring
scheme reassigns arbitrary colours when an unrelated net moves, which turns
every render into a large diff and makes the byte-comparison gate useless as a
review signal. Class colouring is stable under unrelated edits. Of the 21
`router.wire()` calls in `circuits/robocar_unified.py`, 15 pass `steelblue`
explicitly and 6 pass no colour at all.

## Alternatives considered

**libavoid** (Adaptagrams) — the orthogonal connector router behind Inkscape and
Dunnart, with genuine hop and hyperedge support. Installed independently and
measured: it works. It has **no PyPI wheel under any name**, so adopting it means
either vendoring a C++ build or asking every contributor to compile it, and the
WASM build exposes neither `JunctionRef` nor `HyperedgeRerouter` — the two
classes that would be the reason to take it. Held in reserve rather than
rejected: if hand-rolled hops prove inadequate, this is where to look next.

**elkjs, OGDF, HOLA** — all are *layout* engines: they choose node positions.
That destroys hand-authored placement, which ADR-021 protects on purpose and
which #462 restates as explicitly out of scope. `knownLayoutAlgorithms()` returns
11 algorithms and `alg.libavoid` is not among them, so elkjs cannot even be used
as a routing-only front-end to the router above.

**KiCad 10** — the one tool in this survey that ships hop-overs as a feature. It
is ~6 GB in CI, both Python schematic-authoring libraries predate KiCad 9, and
its symbols are logical rather than physical, so it would not deliver the
footprint half of this ADR either.

**pyelk** — no `edgeRouting` option, so orthogonal routing is not configurable.

**pyorthogonalrouting** — takes no obstacle parameter at all, which is the entire
job `routing.py` does. Also AGPL, against this repo's MIT.

**Fritzing part libraries / Adafruit's published parts** — CC BY-SA, and the
share-alike term reaches a rendered diagram that embeds the artwork. Ruled out by
Lauri on licence grounds. The best external hit rate measured against the parts
this repo actually uses was 6 of 13 in any case, so most footprints would still
be hand-drawn.

**SVG `A` (elliptical arc) for hop-overs** — schemdraw's matplotlib backend
degrades an `A` command to `MOVETO`, which silently breaks the path. Hops use two
quarter-circle cubic Béziers with the standard circle constant
`k = 0.5522847`.

## Related

- [ADR-021](ADR-021-hardware-source-of-truth.md) — the board × header × parts
  join; epic #458. ADR-021 owns *where pin data comes from*, this ADR owns *how
  it is drawn*. Stage 6 consumes #459's parser.
- #462 — schematic consumes the join for pin labels and net endpoints. Physical
  footprints are a change to symbol *geometry*: neither placement (which #462
  excludes) nor labels (which #462 covers).
- #463 — already documents the symbol-size/fixture coupling that stages 1 and 6
  both disturb.
- `docs/reference/boards/xiao-esp32s3.md` — carries physical header order
  (`Side`/`Pos`) for all 14 positions. `xiao-esp32s3-sense.md` states the same
  external pinout and links to it.
- `docs/schematics/README.md` — router and circuit-authoring conventions.
