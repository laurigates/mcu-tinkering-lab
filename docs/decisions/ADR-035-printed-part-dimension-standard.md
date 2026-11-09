# ADR-035: A Shared Dimension Standard for Printed Parts

**Status**: proposed
**Date**: 2026-11-09
**Source**: conversation 2026-11-06/09 (Lauri: printed parts should fit
together between projects while prototyping, "sort of like how Legos work,
but not to that degree")
**Confidence**: 6/10 (the fastener sizes are ISO; every fit is the value an
existing part was built with, and none has been measured on a coupon yet)
**Relates to**: [ADR-027](ADR-027-kicad-carrier-generation.md) (the carrier
cases in `tools/enclosure` are its consumers)

---

## Context

Three generators make printed parts: `tools/enclosure` (carrier cases, JLC3DP
resin), the pan-tilt `print/design.py` (JLC3DP PA12, MJF) and the phone
microscope `design.py` (FDM). Each defined its own copy of the same numbers:

| Number | `case.py` | pan-tilt `print/` | microscope |
|---|---|---|---|
| Thinnest wall | `MIN_WALL = 2.0` | `WALL = 2.0` | (README: 2 mm) |
| M3 clearance | `3.4` | radius `1.7` | `M3_CLEAR = 3.4` |
| M2 pilot / tap drill | `1.6` | `PILOT = 1.6` | |
| M3 nut | | pocket `5.8`, stand-in `5.5 x 2.4` | `5.5 x 2.4` |
| Slide fit, per side | `LIP_FIT = 0.3` | `KEY_FIT = 0.3` | |
| Running gap | | `GAP = 0.5` | |

Nothing kept them equal, and nothing gave a new part a place to start. A
bracket from one project had no reason to bolt to a plate from another.

LEGO interchangeability rests on moulding tolerances near ±0.01 mm. Printed
parts hold ±0.1 to ±0.3 mm, and the error depends on the process, the
material and the orientation (JLC3DP: MJF ±0.2 mm or ±0.2 %, holes under 5 mm
0.2 to 0.3 mm undersize; [`docs/reference/jlc-blog/3dp-enclosure-mechanism.md`](../reference/jlc-blog/3dp-enclosure-mechanism.md)
B1, B2). Part geometry cannot be standardised to that degree. The interfaces
between parts can: a hole pattern, a fastener, and named fits.

## Decision

### 1. `tools/cadstd` is the one source

A package under `tools/`, imported as `cadstd`:

| Module | Holds | Imports |
|---|---|---|
| `fasteners` | M2, M2.5, M3, M4: clearance (ISO 273 medium), tap drill (ISO 2306), nut (ISO 4032), socket (ISO 4762) and button (ISO 7380) heads | stdlib |
| `fits` | `MIN_WALL`; the fit classes per process; heat-set insert holes per process | stdlib |
| `grid` | the mounting grid: pitch, plate size, slot length, `off_grid()` | stdlib |
| `shapes` | build123d cutters (`clearance_hole`, `pilot_hole`, `insert_hole`, `nut_pocket`, `slot`) and `grid_plate` | build123d |
| `coupon` | the calibration coupon | build123d |

The stdlib modules may be imported from `tools/kicad` and `tools/hardware`,
which must not import build123d (`enclosure-generation.md`). `tools/enclosure`
and `tools/mechanism` both pin `build123d==0.13.0`, so `cadstd` has no venv of
its own: its shape tests run in `tools/mechanism`'s (`just cadstd::test`), and
the stdlib tests run in the `cadstd` pre-commit hook, and so in CI.

### 2. Fits are named, per side, and per process

A fit is the gap between two faces, per side; a bore for diameter d is
`hole(d, kind, process) = d + 2 * gap`. Parts ask for a class by name:

| Class | Use | Gap (all three processes, unmeasured) | Taken from |
|---|---|---|---|
| `close` | located, then clamped: a captive nut, a split collar on its rod | 0.15 | pan-tilt nut pocket 5.8 AF; microscope rod bore 10.3 |
| `slide` | assembles by hand, stays put: a lid lip, a key in its slot | 0.3 | `case.py` `LIP_FIT`; pan-tilt `KEY_FIT` |
| `running` | moves against its neighbour in use | 0.5 | pan-tilt `GAP` |

Processes: `fdm` (home printer, PLA or PETG), `sla` (JLC3DP resin, the
cases), `mjf` (JLC3DP PA12, the pan-tilt). They start with identical values
because no measurement yet separates them. Each value carries its `source`
and a `measured` flag. There is no `press` class: no part relies on an
interference fit, and JLC3DP calls press fits "not reliable as-printed" in
MJF (B3 in the reference above).

Heat-set insert holes depend on the process, so they are per process:
`mjf` M2 3.2 (the pan-tilt's), M3 4.2 (JLC3DP's 4.2 to 4.3); `fdm` M3 4.0
(an estimate); `sla` none, since resin cracks round a hot insert.
`insert_hole()` raises an error for a process or size without one, so no
part gets a hole that nothing will fill.

### 3. The calibration coupon measures a process

`just cadstd::coupon <process>` writes a plate and a peg. The plate has a
ladder of seven holes for the 4 mm peg, at per-side gaps from 0.05 to 0.5,
and a row of hardware features: M3 clearance hole and slot, M3 and M2 pilots,
an M3 nut pocket, and the process's insert holes. Both rows sit on the grid
pitch. Printing peg and plate on the same process measures the print-on-print
fit that printed assemblies have. A measured value goes into
`fits.PROCESSES` with `measured=True`. The coupon passes the wall gate it is
tested against (2.0 mm).

### 4. The default thinnest wall is 2.0 mm

This is `case.py`'s value, and the cases fail under it. JLC3DP flagged lids
measured at 1.6 mm, and 2.0 is above every process minimum JLC3DP publishes
(SLA 0.8, MJF 1.0, FDM 1.2). A design may go under it only where it records
why: the pan-tilt's thinnest MJF walls are 1.0 mm, listed part by part in its
README ("Walls").

### 5. The mounting grid: 10 mm, M3

A part that mounts to, or carries, other parts puts its mounting holes on a
10 mm grid of M3 clearance holes:

- Holes sit at cell centres. An n-cell plate is `n * 10 - 0.5` mm (0.25 off
  each end), so plates butted end to end keep every hole on one grid.
- One side of every grid joint is slotted ±0.5 mm along the joint, so print
  error and a second hole pattern do not fight.
- Plates are 3 or 5 mm thick, so M3 x 8 and M3 x 10 cover most joints.
- An odd-shaped thing (a board, a servo, a motor) gets an adapter: its own
  holes on one face, grid holes on the other.

The pitch leaves `MIN_WALL` between neighbouring M3 clearance or insert holes
and to a plate edge (`test_cadstd.py` holds this). 10 mm matches MakerBeam's
10 mm M3 extrusion, so aluminium beam and printed parts mix.

### 6. The grid is gated where a design declares it

`mechanism.check` reads an optional `GRID_HOLES = {part: [(x, y), ...]}` from
a `design.py` and fails (`grid_off` in `check.json`) if any hole is not a
whole number of pitches from the part's first. A design builds those holes
from `cadstd.grid.points`, so what it declares is what it cuts. A design
without `GRID_HOLES` is not checked and its `check.json` keeps its old shape.

## Migration

Every value a consumer now reads from `cadstd` equals what it had before.
`tools/cadstd/test_cadstd.py` pins those values, so changing one in `cadstd`
fails a test instead of moving a committed part without notice.

| Consumer | Now from `cadstd` |
|---|---|
| `tools/enclosure/enclosure/case.py` | `MIN_WALL`, M3 and M2 pilots, M3 clearance, `LIP_FIT = fit("slide", "sla")` |
| pan-tilt `print/design.py` | `WALL`, `PILOT`, `INSERT_HOLE`, `GAP`, `KEY_FIT`, the M3 nut pocket and stand-in, the M3 clearance |
| phone microscope `design.py` | `ROD_FIT`, M3 clearance, head and nut, M4 head and nut |

Check: every printed part of both designs (18) had identical volume, area and
bounding box to 1e-6 mm before and after the change. `just enclosure::test`
gave 48 passed, 2 skipped, before and after; `just mechanism::test` passed
its 20 existing tests before and after, plus the 3 new grid tests.

Values left where they are:

- The pan-tilt tournament `entries/` are the tournament record and are not edited.
- Microscope `M4_CLEAR = 4.4`: 0.1 mm under ISO 273 medium (4.5). Kept so the
  printed parts do not change; the next revision takes `M4.clearance`.
- Pan-tilt values tied to one bought part: the servo windows (`FIT`,
  `FIT_END`, `FIT_TAB`), the bearing pockets and crush ribs (`TB_*`), and the
  nut pocket depth (`nut_t + 0.2`).
- `case.py` `SCREW_HEAD` (6.0 x 2.6): a keep-clear envelope round a pan head,
  not a nominal size.

## Alternatives considered

- **LEGO Technic as the base grid** (8 mm pitch, Ø4.8 holes): gives access to
  axles, gears and beams, and the repo has LEGO Boost projects. Rejected as
  the base: the holes need per-printer tuning, cross-axle holes print poorly,
  and pin friction fits are sensitive to small errors. A Technic adapter
  plate on the 10 mm grid remains possible.
- **Gridfinity (42 mm) or Multiboard (25 mm)**: built for storage and wall
  mounting, too coarse for mechanisms.
- **Universal fit numbers**: one 0.3 mm does not describe FDM, resin and MJF.
  The per-process table starts with equal values but can diverge once measured.
- **A venv for `cadstd`**: a third copy of build123d and OCP for a package
  whose consumers already pin it.

## Open questions

- **Running gap on MJF.** JLC3DP's guideline gives 0.6 mm between moving
  parts in MJF and 0.5 in SLA and FDM. The pan-tilt (and so `running`) uses
  0.5. JLC's figure is for parts printed in place; separately printed parts
  may need less. The coupon decides; until then 0.5 stays, so the ordered
  pan-tilt is unchanged.
- **M2 pilot in resin.** 1.6 mm is just above JLC3DP's 1.5 mm resin minimum
  hole, with about 0.3 mm shrink (C3 in the reference). The coupon has an M2
  pilot; the `sla` coupon tests it.
- **`mechanism.check` reports the thinnest wall but does not fail on it**;
  only clashes, and now off-grid holes, fail. `tools/enclosure` fails under
  2.0 mm. Making the mechanism gate fail on walls under `MIN_WALL` would fail
  the pan-tilt, whose 1.0 mm MJF webs are deliberate (its README, "Walls").
  It needs a per-design floor first.

## Consequences

- A new printed part takes its holes, fits and wall from `cadstd` and names
  its process; the numbers in it are then the part's own dimensions only.
- Measuring a process once (the coupon) updates every part built for it on
  its next build.
- A reusable bracket or plate states its grid holes, and `mechanism::check`
  holds them to the grid.
- `cadstd` is now a dependency of `tools/enclosure` and `tools/mechanism`:
  both put `tools/` on `PYTHONPATH` (their justfiles and test conftests).
