---
name: enclosure-cad
description: Generate, check, render and quote 3D-printed cases for the KiCad carriers with tools/enclosure (build123d), and the traps that read as success. Use when adding a case for a board, changing tools/enclosure or an enclosure.toml, rendering a board in its case, or pricing cases at JLC3DP.
argument-hint: <project path, e.g. packages/robotics/balancebot>
allowed-tools: Bash, Read, Grep, Glob, Edit, Write
---

# Carrier Cases: build123d From the Committed Board

`tools/enclosure` is a uv project (build123d 0.13.0, Python 3.14.8) that
builds two cases for each piece of a carrier board: an open **tray** and a
closed **box** with lid. It reads the board, never a second copy of it.

| Fact | Source |
|---|---|
| Footprint positions, courtyards, mounting holes, board thickness | the committed `.kicad_pcb`, through the stdlib `kicad.sexpr` reader |
| Part heights and pin depth under the board | a `kicad-cli pcb export step` of that board |
| Heights of parts with no 3D model | the project's `hardware/enclosure/enclosure.toml` `[heights]`, each with a `source` |
| Pieces, wall openings, lid windows, chassis tabs | `enclosure.toml` `[[pieces]]` |

```sh
just enclosure::test                 # pytest: synthetic pieces + the balancebot board
just enclosure::gen <project> ...    # STEP/STL/3MF per piece and style, report.json
just enclosure::render <project>     # isometric line drawings (SVG/PNG)
just enclosure::assembly <project>   # Blender renders: the populated board in each case
```

## The design (2026-10-23)

- The board screws onto short solid standoffs: a self-tapping screw into a
  blind pilot hole (M3 2.5 mm, M2 1.6 mm), `min_standoff` 4 mm of thread.
  Resin takes a thread; it does not take a heat-set insert.
- The case bolts to the chassis through two floor tabs (M3 clearance), or
  none: `mount = "none"` (robocar head 1, held by its cradle).
- The box lid screws into four bosses merged into the wall corners. Each
  starts above the parts and board screw heads under it; a corner with under
  `MIN_THREAD` (4 mm) of thread loses its boss, and fewer than two is an error.
- Box cable windows close 1 mm above the connector plus a mated plug
  (`plug`, 6 mm), with `MIN_WALL` above; the tray's low walls keep open
  notches. A back-side connector's window is below the board.

## The gates `gen` enforces (it exits non-zero on either)

1. **Clash**: case ∩ (board STEP moved into case coordinates + a stand-in box
   per model-less part) must be empty within the piece.
2. **Wall thickness**: rays cast inward from 20 000 surface points
   (`enclosure/thickness.py`, fixed seed); any part under `MIN_WALL` (2.0 mm)
   fails, with the point printed.

Both have controls in the suite (a part through the wall must clash; a 0.5 mm
plate must read 0.5 mm), and every geometry fix has a test that fails on the
pre-fix code.

## Traps

1. **build123d must stay out of `tools/kicad`.** Its unittest discover runs on
   stdlib-only Python in CI and pre-commit; one import breaks both.
2. **KiCad 10's 3D library has no Pololu breakout, 1812 fuse or breakout-socket
   models, and the XIAO sockets point at `${AMZPATH}`.** The STEP silently
   lacks them; `load_pieces` refuses a top-side part with no height rather
   than treating it as flat. Estimates stay marked as estimates.
3. **The STEP labels models by footprint name, not reference.** Heights are
   assigned by which courtyard holds the model's centre; a model that lands on
   none raises, which is also the check on the placement transform.
4. **JLC3DP's upload check misses real defects and flags non-defects.** It
   passed a lid with a 0.15 mm fin (a camera window cut through a pillar) and
   flagged "Thin walls detected" on lids whose thinnest wall measured 1.6 mm,
   although its page asks only for over 1.2 mm. Trust the thickness gate, and
   keep `MIN_WALL` at 2.0.
5. **A cylinder tangent to a plane makes a 3MF the mesher rejects** ("3mf mesh
   is invalid") while the B-rep is valid. `_untangent` grows a boss 0.1 mm
   within 0.05 mm of tangency.
6. **A circular cut through a thin band leaves points that taper to nothing**
   (the lid lip round a boss: 0.004 mm). Cut bands square.
7. **A blind hole's skin counts as a wall.** It is `SKIN = MIN_WALL`.
8. **`kicad-cli` writes non-ASCII to stderr**; decode as UTF-8 with
   `errors="replace"`, or `subprocess` fails under an ASCII locale.

## Renders (`enclosure.assembly` + `blender/render_assembly.py`)

`kicad-cli pcb export glb` gives the populated board with real colours; it
imports into Blender in metres, top-left corner at the origin, bottom face at
z 0 (the STEP's frame). Scale ×1000 and apply the clash check's offset.

- **Blender `-b -P` exits 0 when the script raises.** Pass
  `--python-exit-code 1`.
- Blender 5's boolean solvers are `FLOAT`, `EXACT`, `MANIFOLD`; `FAST` is gone.
- `read_factory_settings` deletes materials made before it: make them after.

## Pricing

JLC3DP (9600 resin, white) priced every tray at €0.58–1.76 and a box at
€1.42–4.34 on 2026-10-22/23; the quoted volume matched `report.json` to
0.01 cm³. Upload and cart steps are in the `jlcpcb-order` skill.

## Related

- `carrier-pcb` skill: the boards these cases fit
- `jlcpcb-order` skill: JLC3DP upload, product description, cart
- `.claude/rules/enclosure-generation.md`: the edit-time pointer here
