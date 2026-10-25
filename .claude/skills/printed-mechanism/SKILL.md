---
name: printed-mechanism
description: Design, gate, render and order 3D-printed servo mechanisms (pan-tilts, brackets, linkages, eyebrows) with tools/mechanism (build123d), including the multi-agent design tournament. Use when designing or changing a printed part that moves or carries a servo, comparing mechanism designs, or ordering one from JLC3DP.
argument-hint: <design dir, e.g. packages/robocar/unified/hardware/pan-tilt/print>
allowed-tools: Bash, Read, Grep, Glob, Edit, Write
---

# Printed Mechanisms: Envelopes, a Clash Sweep, and Renders

`tools/mechanism` checks any directory holding a `design.py`. Cases for a
carrier board are the `enclosure-cad` skill; this one is for parts that move.
The worked example is robocar-unified's MG90D pan-tilt
(`packages/robocar/unified/hardware/pan-tilt/`): five tournament entries, the
refined winner in `print/`, and the record in its README.

```sh
just mechanism::setup                 # libosmesa6 for headless VTK, and the venv
just mechanism::test                  # pytest: clash, servo envelopes, one render
just mechanism::check <design dir>    # clash sweep + wall per printed part -> output/check.json
just mechanism::showcase <design dir> # assembled x5, travel poses, exploded x2, parts
just mechanism::quickview <design dir># matplotlib, while iterating
```

## The design contract

`design.py` exposes:

| Name | Meaning |
|---|---|
| `printed_parts() -> dict[str, Part]` | every part to order |
| `assembly(*pose) -> list[Body]` | printed parts and stand-ins at one pose, world frame |
| `JOINTS`, `SWEEP`, `REST`, `SHOWCASE_POSES` | optional; default a pan/tilt: pan ±90 by 30, tilt ±45, rest (0, 0), one view at (60, 30) |

A `Body` has a `group`: bodies that move together. Only pairs from different
groups are clash-tested, and the exploded render lifts whole groups. `allow`
excuses a named pair for a deliberate interface (a spline in its hub); every
excuse is listed in `check.json`, so read that list in review.

Stand-ins are keep-out envelopes from `mechanism.servos` (MG90D, SG90, horns),
each dimension with its source and "estimate" where nobody measured. A
project adds its own (the pan-tilt's head board is in its `common/fixtures.py`).

## Before designing: write the shared fixtures yourself

When several designs will be compared (a tournament, or before/after), write
the stand-ins and gates first and make every design use them. Otherwise each
designer models the servo slightly differently and the comparison measures
the models, not the designs. Make the dimensions a design depends on
parameters at the top of its `design.py`, and list them in its README under
"measure before ordering".

## Mechanism checklist

Every item below was a real finding on the pan-tilt, most by an inspector after
the designer reported success.

1. **Horn capture and torque path.** Which stock horn, which screw, and does
   torque reach the part through the spline and horn, not through friction?
2. **The opposite-side pivot.** A tilt stage hung off the spline alone
   cantilevers the servo's bearing. Put a pin or bearing on the far side.
3. **Cables over the whole sweep.** Model servo leads and ribbons as bodies
   and sweep them; the pan-tilt's taut-ribbon model found a ribbon through
   the head mount when looking down. A taut model is still not the slack loop:
   say so and hand-sweep a real cable before closing up.
4. **Fasteners in their own group.** A screw in the same group as the servo it
   sits beside is never checked against it; the brow-pod screw clashed with
   the SG90's tab and the gate passed.
5. **Driver access per assembly step.** Walk the steps in order with a hex key
   or screwdriver: can each screw be reached when its step comes? Chassis
   screws under a fitted yoke and a cap whose screws touch down before the
   bearing seats were both missed by the gates.
6. **Tolerance stacks need a take-up.** A gap set by four toleranced parts
   (tab thickness, horn height, MJF ±0.3 mm twice) needs a shim, slot or
   feeler-set adjustment, not a modelled 0.4 mm.
7. **Sanding only removes material.** Print a pin oversize and sand to fit, or
   sleeve an undersize one; "undersize, then sand to a slide fit" cannot work.
8. **Walls against the process, not the case convention.** `MIN_WALL` 2.0 mm is
   for resin cases; MJF holds about 1.0 mm, and a tapped hole beside a window
   loses about 0.2 mm more to the thread. Print a coupon of the thinnest webs
   and the press-fit ribs before the full set.

## Materials at JLC3DP

- **MJF PA12-HP** for rigid structure (dyed black is cheap); **PA11** for
  anything that flexes at assembly (snap-fits), since PA12 snap-fits fatigue.
- **Resin** for cosmetic parts only: it cracks as a snap-fit and does not hold
  heat-set inserts (`enclosure-cad`). Self-tapping into MJF holds; tap and wick
  thin CA for screws beside thin webs.
- Price ballpark used on the pan-tilt: about $0.28/g of MJF PA12 with a $1
  minimum per part (an agent's estimate, check the quote). One order with a
  material per line, and a geometry-identical cheap fallback for any
  cosmetic upgrade, so styling never holds up the structure.

## The design tournament

`.claude/workflows/design-tournament.js` runs N designers with distinct
briefs → an independent inspector each → a revision → a panel of judges with
distinct lenses → the winner refined with grafts from the runners-up → an
adversarial review → fixes. Run it only when the user asks for a multi-agent
run. Write the shared fixtures first (above), and pass the brief, entrants,
lenses and paths as `args`; the script's header documents them. The pan-tilt
run used 36 agents and about 5.3M subagent tokens and hit a session limit
once; see `.claude/rules/multi-agent-workflows.md` for surviving that.

## Environment traps

1. **tools/enclosure pins Python 3.14.8**, which uv cannot download in cloud
   sessions. tools/mechanism needs only >=3.13 and loads the wall gate from
   tools/enclosure by path. To run tools/enclosure's own tests on 3.13:
   `cd tools/enclosure && PYTHONPATH=..:. python -m pytest` in a venv with
   build123d 0.13.0, trimesh, rtree and pytest.
2. **Headless VTK needs `libosmesa6`** (no X, no EGL): without it pyvista warns
   about OSMesa and EGL and renders nothing. `just mechanism::setup`.
3. **Blender runs headless from PyPI** (`bpy` 5.2.3 on Python 3.13): Cycles CPU
   about 7 s for 800×600 at 32 samples, Workbench 1 s, EEVEE about 46 s through
   software GL. That suits product renders; VTK is the right tool for checking
   a mechanism (under 3 s a view, feature edges, exact geometry). tools/enclosure's
   recipes still call the macOS Blender app.
4. **matplotlib sorts faces, not pixels**: parts show through each other in
   `quickview`. Send `showcase` images to people.
5. **One ray can lie about a wall.** A sample on a convex rim cast its ray along
   a tessellated side facet and read 0.01 mm on a 1.0 mm yoke. The gate now
   checks its thinnest readings against a ring of nearby points; a reading
   under half the ring's median is a graze (`thickness.py`, tested on the
   yoke's own facets).

## Related

- `enclosure-cad` skill: cases for carrier boards, the wall gate's home
- `jlcpcb-order` skill: JLC3DP upload and cart
- `docs/reference/servo-selection.md`: why the MG90D does not fit the 9 g kits
