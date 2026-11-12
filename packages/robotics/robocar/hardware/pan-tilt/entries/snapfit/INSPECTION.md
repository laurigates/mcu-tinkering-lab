# Inspection: `snapfit` (Snap & Flex)

Independent inspection, 2026-10-25. I did not take the README's word for
anything. Scratch probes are in
`/tmp/claude-0/-home-user-mcu-tinkering-lab/fb62d1db-4321-5a2a-995b-4b8db64d8229/scratchpad/insp/`.
I changed nothing in this entry except this file and the regenerated `output/`.

## 1. Gates reproduce

- `python design.py` runs in about 6 s. It writes 5 STEP/STL pairs with the
  volumes the README states: base 20.76, yoke 13.20, cradle 7.90, brows 0.81
  cm³ each. All solids are valid. The tilt axis is at z 64.9.
- `python -m common.check entries/snapfit` gives **zero clashes** over pan ±90 ×
  tilt ±45. Its `check.json` is byte-identical in content to the committed
  one. The thinnest walls are 1.79 / 1.51 / 1.50 / 1.60 / 1.60 mm.
- `python -m common.quickview entries/snapfit`: the four PNGs regenerate and match
  the description.
- Excused pairs: only `pan-servo / pan-horn` and `tilt-servo / tilt-horn`, the
  spline inside its hub. That is honest. I found no hidden clash behind them.

**Caveat on what "zero clashes" proves.** `check.py` skips pairs in the same
group, and this entry puts each servo and horn in the group of the part that
holds it. So no servo-in-pocket, horn-in-pocket or head-in-cradle fit is tested
by the gate. The check allows this, and every entry can do it, but the claim
"zero clashes" says nothing about those fits. I tested them separately (§2.6).

## 2. Mechanism

### 2.1 Assembly order and access: mostly sound

- **Pan servo.** It drops in through the Ø46.1 ring bore. The bore was sized
  from the tab corner and the lead boot, and I checked that. The tabs rest on
  the 2.1 mm pocket walls, and the lead is threaded out through a closed
  5.2 × 10 mm window in the ring wall before the servo goes in. A Dupont plug
  fits that window.
- **Pan horn screw.** It is reachable from straight above, through the Ø4.5 hole
  in the yoke, after step 5 and before the cradle. Nothing sits over the pan axis
  at that point.
- **Tilt horn screw.** It is reachable coaxially through the yoke's +Y arm pivot
  hole (Ø6.4, about 37 mm from the hub) before the cradle goes on. A PH0 shaft
  fits through it. The README is right that this is the only time.
- **Cradle.** To fit the cradle, the arm has to spring out about 4.1–4.5 mm:
  1.6 mm of pocket depth plus 2.5 mm of stub engagement. The cradle is pushed
  +Y to clear the horn arm, lowered, then pushed -Y onto the horn. The arm
  strain is about 2.8 % (3·δ·t / 2L², t 3.0, L 26.8), which matches the README.

### 2.2 MAJOR: the tilt servo is held against its reaction torque at one end only

In `slide_cradle()`, the lower tab (the end away from the shaft) is boxed in:
the back rail behind it, the flexure nose in front of it. The **upper tab, at
the shaft end, has only the back rail.** Its groove is open toward +y_s (world
+X, the slide-in direction), and nothing closes it. A tab in a groove can turn
in the groove's plane, and that plane contains the tilt axis. So for one sign of
tilt torque the servo body pivots about the lower tab, and its shaft moves
forward about 21·θ.

The only thing that resists this is the shaft itself: through the horn, the
cradle, the Ø6 stub and the yoke arm. The servo's reaction torque therefore goes
through the press-fit horn pocket and the stub. At 1 kg·cm that is a side load
of about 5 N on the shaft. The slop in that path is the stub's 0.4 mm diametral
clearance plus the 0.15 mm horn pocket. Over a 21 mm lever that is about 1.5°
of head play, and it reverses whenever the torque sign changes: on every nod,
and when the head moves through balance.

Fix: give the upper groove a second catch (a mirror of the lower flexure), or a
front stop that the servo clicks past.

### 2.3 MAJOR: each flexure finger touches its end wall along a zero-width line (all three slide cradles)

In `slide_cradle()`, the 0.8 mm slit is cut at `x ∈ [f_lo − 0.8, f_lo]`, which
is inside the end wall. The finger is the ledge at `x ≥ f_lo, z ≤ g0`. The end
wall above the slit is solid out to `x = f_lo` for `z ≥ g0`. So the finger's top
corner and the wall's corner meet exactly at `(f_lo, g0)`, along the finger's
whole ~13 mm length.

I sectioned it (`insp/finger.png`), and the STLs confirm it: trimesh reports a
non-manifold edge shared by four faces in `yoke.stl` at y −29.4, z 43.65,
x −4…8.9, and two in `cradle.stl` at the brow pod (x −15.1, z 116.25). For the
same reason `yoke.stl` and `cradle.stl` are **not watertight**.

On MJF a zero gap fuses. The "flexure" is then held along two edges, and it has
to tear the fused line before its nose can drop 0.8 mm. Expect the tilt servo
and both SG90s not to slide in, or the finger to crack free on the first
insertion. JLC's mesh check may also flag the files.

Fix: start the slit at `f_lo` and run it to `f_lo + 0.8`, or drop the finger's
top by the slit width at that edge.

### 2.4 MAJOR: the head board's latch pegs have no lead-in in the insertion direction

The pegs (`_cradle_board`) point along +X, into the board face, and their cone
tapers toward +X. The board drops in along −Z. Its flat bottom edge meets the
top line of the Ø1.9 cylinder first (x 0…0.6, where the board overlaps it), and
the contact normal there is pure Z. There is no cam force to push the tongue
back. The README's "the board's bottom edge pushes it back" is false. To fit the
board you have to hold both tongues back by about 1.7 mm by hand while you slide
it down. They sit behind the board and are reachable only from the rear. With
the brow deck fitted, they are also under the deck.

Fix: add a ramp on the top side of each peg facing +Z, or chamfer the tongue so
that the board's edge deflects it.

### 2.5 MAJOR: the -Y cheek cuts into the J2 keep-out (hidden by grouping)

Cradle ∩ head (same group, so the gate never sees it) is 16.7 mm³. Seven of
that are the two pegs in the solid board, which is expected, because the fixture
has no holes. The other **9.7 mm³ is the -Y cheek inside the J2/IDC keep-out**,
at board frame x −3…0, y −15.5…−15.1, z −0.5…7.5.

The notch stops at `GROOVE_FLOOR` (−15.1). The fixture's keep-out reaches −15.5,
and a 14-way 2.0 mm IDC plug is about 18–19 mm long, so it overhangs the board
edge by about that much. The README says the cheek "is notched for the IDC plug".
It is notched, but 0.4 mm short. Deepen the notch to y ≤ −15.6.

### 2.6 MAJOR: zero tolerance budget in the pan stack (fixture-estimate sensitivity)

How I tested sensitivity to the fixture estimates:

- **Envelope margin 0.5.** I wrapped `fx.servo(margin=0.5)` (the design binds
  `fx.servo` at call time) and swept pan ±90/0 × tilt ±45/0. Inter-group clashes:
  **none**. In-host overlaps (my own check): base/pan-servo 293, yoke/tilt-servo
  150, cradle/brow-servo 156 mm³ each. With 0.2–0.3 mm fits that is expected.
  It means every pocket depends on the stand-in dimensions to about 0.2 mm.
- **C and F +1 mm** (whole body 1 mm longer below the tabs): base/pan-servo
  overlaps 110 mm³. The servo hangs on its tabs only **0.5 mm** above the floor.
  It bottoms out, lifts the horn and so lifts the yoke, and the fingers have only
  **0.2 mm** of axial play under the lip. So the yoke will not snap on.
- **C +1 only** (case top 1 mm higher above the tabs): base/pan-servo 150 mm³,
  yoke/tilt-servo 80 mm³. The yoke would have to sit 1 mm higher, so the fingers
  cannot reach under the lip. **That would stop the build.** Measure C − F, the
  tab-to-case-top distance (8.7 mm on paper). The README's "Measure before
  ordering" table lists F and the spline height but not C or C − F.
- **Tab thickness 2.9** (+0.4): the tilt servo's tabs do not fit their grooves
  (20 mm³). The README flags this.
- **Headline size 22.8 × 12.2**: fits.

The servo does not bottom out at the nominal size, but a published-figure error
of +0.3 mm in the vertical stack means the design is not assemblable. Suggested
fix: make the snap fingers' catch longer (axial play of 1–1.5 mm), and take up
the play with a compliant printed wave spring or a shim under the yoke. Or
fasten the pan servo by its tabs, so that the horn height no longer sets the
snap.

### 2.7 Horn capture and torque path: OK

- **Pan.** Spline → cross horn → the yoke's underside pocket (0.15 mm clearance;
  the stand-in fits with zero overlap). The horn is held on the spline by the
  yoke, and the yoke by the fingers (0.2 + 0.3 mm play). The pan servo is held
  down by the same stack. It has no screws, which is acceptable once 2.6 is
  fixed.
- **Tilt.** Spline → single-arm horn → the cheek pocket, 1.6 mm deep. The horn
  can back off the spline by no more than the 0.4 mm `RUN` gap before the cheek
  meets the arm. The +Y stub is a real opposite-side pivot, so the spline does
  not carry the head as a cantilever.

  Weak point: the pivot arm is designed to spring 4.5 mm in Y. By its own
  bending (E ≈ 1.8 GPa, 12 × 3 mm section, 26.8 mm long), about **9 N** of
  sideways push on the head toward +Y (1.2 mm of arm deflection) lets the cradle
  slide off the horn arm, and about **19 N** pops the stub. A knock against a
  wall or a hand can do that. That is minor to major: the head drops but nothing
  breaks.

### 2.8 Cables: plausible, not modelled

- **Pan lead.** It is static and leaves through the rear window. Fine.
- **Tilt lead.** It runs to the clip on the yoke's -Y edge and pans with the
  yoke. Fine. The 25 cm lead may be short from z ≈ 45 at a radius of about
  50 mm.
- **IDC ribbon.** From J2 it goes back about 25 mm to the hook on the rear
  tongue. The hook is a 2.2 × 17 mm slot, open at +Y for sideways entry, at
  radius ≈ 31.6 mm, outside the fingers (R 28.1), and the ribbon's width lies
  along Y. In tilt, it bends about its easy axis, so that is fine. In pan, the
  hanging length sweeps around the base at radius ≈ 31. At ±60° that puts it
  right over the M3 ears and screw heads at (−6, ±31), which stand about 4.7 mm
  proud. Beyond about ±70° the ribbon drapes across them. No geometry checks the
  ribbon. Minor: move the ears to ±45° behind, or give the ribbon a guide.
- **SG90 leads.** They need extensions; the README says so.

## 3. Printability (JLC3DP MJF)

- **PA11 for the parts that snap** (yoke and cradle) is the right choice, with
  enough elongation for 1.6–2.8 % strains. PA12 for the base and brows is fine.
  Resin is correctly avoided.
- **Walls.** 1.5 mm minimum is at or above the MJF minimum JLC states. The 0.8 mm
  slits and the 0.9 mm lead-clip slit are above the ~0.6 mm fusing gap. The
  exception is the line contact in 2.3.
- **Trapped powder.** Every part is one solid with one shell, so there are no
  closed cavities.
- **Mesh.** `yoke.stl` and `cradle.stl` are non-watertight because of the
  non-manifold edges in 2.3. Fix the geometry; do not rely on JLC's repair.
- **Prices.** The figures ($0.275/g, the PA11 premium) are not verified. The
  README says to upload the parts for a quote.

## 4. Fit to the robot

- **Chassis.** Two Ø3.2 holes, 62 mm apart across the car, 6 mm behind the pan
  axis, and an optional Ø10 lead hole. That is usable. The README does not say
  where the pan axis goes on the plate relative to the front or the carrier.
  Minor.
- **Head M2 holes.** They take printed Ø1.9 pegs (holes 2.2 mm) instead of
  screws. That works, subject to 2.4.
- **J2 keep-out.** See 2.5.
- **Height.** The tilt axis is at z 64.9 and the brow pod tops out at about
  z 119 above the plate. On a small 2WD chassis that is a high centre of
  gravity. The brows sit about 27 mm above the camera's "eye". That is a
  character choice, but it is worth checking with Lauri.

## Verdict

The gates reproduce: zero clashes, and walls of 1.5 mm or more. The concept is
coherent: a captured snap ring as the pan bearing, an opposite-side tilt stub,
and no tools except the M3 screws. Four defects need fixing before it is
ordered:

1. The flexure finger fused to its end wall along a line, which also makes the
   STLs non-manifold.
2. The head latch pegs with no lead-in.
3. The J2 notch 0.4 mm short.
4. No vertical tolerance in the pan snap stack.

Also fix the tilt servo's restraint at one end only. All five are local
edits to `slide_cradle()`, `_cradle_board()` and the finger/lip constants.

## Revision 2 (designer's response, 2026-10-25)

All five major findings are fixed in `design.py`. The gates were re-run:
`common.check` reports **zero clashes** over pan ±90 × tilt ±45, with the same
two excused spline pairs, and `common.quickview` regenerated the four PNGs. All
five STLs are now **watertight, single-body** meshes (trimesh). The thinnest
walls are base 1.50, yoke 1.50, cradle 1.00 (a latch-peg tip, see 2.4) and
brows 1.60 mm. The volumes are base 20.2, yoke 18.5, cradle 8.6 and brows
0.81 cm³ each. The tilt axis is at z 65.9, 1 mm higher than before.

| Finding | Change |
|---|---|
| **2.3 Finger fused to its end wall** | `slide_cradle()` now cuts the 0.8 slit from the end wall's face **toward the finger** (`[wall, wall ± 0.8]`), so the wall and the finger are 0.8 apart everywhere. The noses start 0.3 past the slit. `yoke.stl` and `cradle.stl` are watertight now, with no edge shared by more than two faces. |
| **2.2 Upper tab held at one end only** | `slide_cradle()` builds a flexure finger and nose in **both** blocks, so each tab is boxed in by the back rail and a nose. This applies to the tilt servo and both SG90s. Upper finger: 3.5 wide, 2.0 thick, about 12.9 long, roughly 1.4 % strain at 0.8 mm deflection. Probe: push the tilt servo 0.6 mm toward the open end, and the lower and upper tabs each meet a nose (0.71 mm³ each). Before, only the lower one did. |
| **2.4 Latch pegs without lead-in** | The Ø1.9 peg (now reaching 1.5 into the board) has its top cut by a plane falling toward +X. Its top line runs from the peg top at the tongue face down to 1.0 above its bottom at the tip, a 28° ramp. The board's back-bottom edge meets that ramp, not a horizontal line, so it cams the tongue back. tan 28° = 0.53, which beats nylon-on-FR-4 friction (about 0.3). The lower half stays round and holds the board in its hole. Cost: the peg tip is 1.00 mm tall, the cradle's new thinnest feature, at JLC's MJF small-feature minimum. `PEG_TIP_H` 1.2 gives a 24° ramp if JLC objects. |
| **2.5 -Y cheek in the J2 keep-out** | The notch now runs to `J2 y-min − 0.15` (−15.65) and in X up to +0.1, past the board's back face. cradle ∩ J2 keep-out = **0 mm³**. cradle ∩ head is 6.6 mm³, all of it the two pegs in the fixture's hole-less board. |
| **2.6 No vertical tolerance in the pan stack** | The servo is taken out of the snap stack. (a) The yoke now rests on the ring top (`RING_TOP = YOKE_B − 0.1`), and the fingers catch 0.2 under the lip. Both are printed dimensions. (b) Both horn pockets have `HORN_SLACK` = 1.0 mm of axial float either way. The yoke plate is 2 mm thicker, and the -Y cheek is 2 mm thicker so the tilt pocket keeps a 2.0 skin. (c) The pan servo is held by **four flat tab hooks** rising from the floor, not by the yoke. The hooks are 22 mm beams with about 0.4 % strain, and they catch 0.3 over the tab top. The pocket walls became side walls plus end pillars, each 0.8 clear of the hooks. (d) The floor under the case is recessed 1.5 mm, so the servo hangs 2.0 clear. Screw count is unchanged at 2 M3. The horn centre screws are now *recommended*, to keep a floating horn on its spline. |

**Sensitivity probe** (`sens.py` in this folder). Each servo is placed by its
tabs, as a real one would sit. I checked overlaps with the printed part that
holds it and how much of each horn arm sits inside its pocket:

| Variant | base/pan-servo | yoke/pan-servo | yoke/pan-horn | yoke/tilt-servo | cradle/tilt-horn | Arms engaged |
|---|---|---|---|---|---|---|
| nominal | 0 | 0 | 0 | 0 | 0 | 100 % / 100 % |
| C ± 0.5, ± 1.0 (C − F changes) | 0 | 0 | 0 | 0 | 0 | 100 % |
| C and F + 1, + 2 (body longer below the tabs) | 0 | 0 | 0 | 0 | 0 | 100 % |
| 22.8 × 12.2 case | 0 | 0 | 0 | 0 | 0 | 100 % |

The brow servos and horns also overlap the cradle and brows by 0 mm³ at nominal.

The probe does not cover tab thickness. A tab more than about 0.3 mm thicker
than 2.5 still jams in the grooves and holds the pan hooks open. The README
lists it, and C − F and the spline height are now in its measure-first table.

**Not changed** (minor findings, still open): the ribbon passes over the M3
ears beyond about ±70° of pan. The arm's sideways retention is about 9 N.
Where the pan axis goes on the chassis plate still needs to be stated.
