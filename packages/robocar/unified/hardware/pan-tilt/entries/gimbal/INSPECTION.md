# Inspection: `gimbal`

Independent inspection, 2026-10-24. I re-ran everything and probed the geometry
with my own scripts. I did not rely on the README's claims.

## 1. Does it reproduce?

- `python design.py` runs in about 12 s and writes 4 × STEP/STL. Volumes are
  tower 16.88, yoke 10.43, outrigger 2.49 and cradle 5.30 cm³, and all four
  are valid. Lens height 76.5 mm. Panned CoM offset is x +0.10, y +8.21 mm.
- `python -m common.check entries/gimbal` exits 0 with **zero clashes and zero
  `allow` excuses** over pan ±90 × tilt ±45. The thinnest walls are tower
  1.10, yoke 1.10, outrigger 2.2 and cradle 1.6 mm.
- `python -m common.render entries/gimbal`: the four PNGs regenerate.
- Each STL is one watertight shell with no internal voids, so MJF powder can
  escape.
- The "no excuses" claim is honest. `bored_horn()` removes the spline from
  the fixture horn, and that is a fair model of a hub on a spline.

Gates reproduce: **yes**. The gates do not test whether the parts can be
assembled, and that is where this entry fails.

## Blockers

### B1. The head board's two M2 screws cannot be driven

Each cradle bridge (`BRIDGE_X0..X1`, lens-frame x −22.0 to −19.5, |y| ≥ 9.5,
z −13.2 to 6) sits 0.7 mm in front of a button head. Both head holes, at
(y +11.75, z −9.65) and (y −10.75, z −9.65), fall inside the bridges' y and z
range. A Ø2 driver shaft along the screw axis cuts 7.9 mm³ out of the cradle
at each hole, and a Ø3.5 bit cuts 24–26 mm³ (`probe2.py`). The screws cannot
be fitted beforehand either. The board only goes into the cradle by sliding
down in Z, and that slide is clear, but shanks already in the board would
have to enter the inserts sideways.

**Fix:** a Ø4.5 hole through each bridge on the screw axis. Alternatively,
end the bridges above z −12 and carry the cheeks some other way.

### B2. The pan cross horn can be neither installed nor screwed to the yoke

- **Screw access.** The four arm screws at r = 9 have their heads under the
  horn, so they must be driven from below (README step 2). The tower's
  opening from below is the servo case cavity: x −16.75 to 6.15, y ±6.3. Only
  the screw at (−9, 0) is inside it. A vertical Ø3 driver to (+9, 0) cuts
  31 mm³ of the tower, and drivers to (0, ±9) cut 97 mm³ each.
- **Insertion.** The fixture cross horn spans 32 mm. Under the chamber is a
  12.6 × 22.9 mm slot. Over it are the Ø27 lip and then the Ø25 bore of the
  6705ZZ. The horn cannot be screwed to the spigot first either, because the
  spigot has to pass down through that Ø25 bore. With the horn's arms 16 mm
  from the axis, nothing passes.

**Fix:** use the **disc horn** (Ø21 in the fixture, smaller than the Ø25
bore). Sink it in a pocket in the spigot's underside, and screw it in on the
bench before the yoke is dropped through the bearing. The pocket walls then
carry the torque. This also helps M1.

## Major

### M1. Pan axial over-constraint

The yoke is located axially twice. Its land sits on the 6705ZZ inner race,
and its spigot is screwed rigidly to the horn, which is pressed onto a spline
whose servo is screwed to the tower ledge. MJF tolerance is ±0.3 on each of
several stacked dimensions, so one of two things happens:

- the horn only part-engages the spline, which raises the risk of stripping
  it; or
- the shaft lifts the yoke off the race, so the servo shaft carries the
  overturning moment the bearing was meant to carry.

The README notes this but designs nothing to absorb it.

**Fix:** a torque-only coupling that lets the horn float axially in its
pocket, with the centre screw only retaining it.

### M2. Servo fits rest on the smaller of TowerPro's two figures, with 0.15 mm per end on a ±0.3 mm process

- The fixture uses 22.6 × 12.0. With TowerPro's headline 22.8 × 12.2 swapped
  into the stand-ins, the nominal check fails: tower/pan_servo 3.8 mm³ and
  yoke/tilt_servo 1.8 mm³ (`margin.py big`).
- With a 0.25 mm fixture margin: tower/pan_servo 43 mm³ and yoke/tilt_servo
  34 mm³.
- With a 0.5 mm margin: 417 and 110 mm³.

Part of the margin result comes from the grown tabs sitting on their own
ledge, but the case-end fit is the real limit. The README's fix
(`FIT_END = 0.25`) brings the tab-pilot web down to 1.0 mm, JLC's floor.

The tab span is similar: the tower's tab cavity is 32.1 mm. An MG90D with
E = 32.3 (the SG90's figure) would not enter it.

### M3. A 1.1 mm web beside every servo tab pilot

The two pilots on each servo, four in all, are Ø1.8 self-tap holes that sit
1.1 mm from the window or case cavity the servo body occupies. A self-tapper
pushes radially. In PA12 that web will bulge into the 0.15 mm end clearance
and pinch the case, or split. The tilt servo's two screws also bite only the
3 mm upright.

**Fix:** pre-drill to Ø1.9–2.0. Where the far side can be reached, use M2
machine screws with nuts. The tower's +X pilot already breaks into the horn
chamber. Test-print one coupon before ordering the set.

### M4. The pan servo's base is 0.5 mm above the chassis plate, and nothing checks it

The servo hangs from its tabs on the ledge, so a real F (base to tab
underside) more than 0.5 mm over the 19.8 estimate puts the servo on the
acrylic. MJF's ±0.3 on the ledge height uses up most of that 0.5 mm. The
tower then rocks, or the tab screws fight the plate.

`assembly()` has no chassis-plate body, so `common.check` cannot see this. A
fixture with F + 1 still passes the gate.

**Fix:** a 2 mm relief under the servo, or a plate stand-in in `assembly()`.

### M5. Ribbon and servo leads are not modelled or guided

Beyond the J2 keep-out, the IDC ribbon and the servo leads exist only in
README prose:

- a loose 40 mm U behind the cradle;
- cable-tied to the +Y upright;
- a ¾-turn helix of a 14 mm-wide ribbon around a stationary tower for ±90°
  pan.

The loop drags over the flange corners and the four M3 heads. No printed part
has a clip, channel or strain-relief feature. Between the cradle's +Y outer
cheek (y 18.25) and the upright (y 20.4) there is no path, so everything has
to go behind the cradle. Whether the ribbon survives the full sweep is
unverified.

## Minor

- **Firmware change.** The flipped head (`HEAD_FLIPPED`) needs
  `set_vflip(1)` and `set_hmirror(1)` in the firmware. The entry does not make
  that change and only states it. USB-C then faces down, with about 21 mm to
  the yoke flange at tilt 0.
- **The nodal claim rests on estimates.** It depends on
  `PUPIL_BEHIND_STACK_FRONT` and the fixture's assumption that the lens sits
  at the stack's centre. Both are estimates, so the "no parallax" benefit is
  unproven. In return the lens sits at 76.5 mm, with a swept envelope of
  about Ø96.
- **Stiffness.** The yoke upright is a 3 mm plate, and the servo window
  leaves 2.7 mm rails. The servo hangs outboard, which loads the upright in
  its weak direction, and the outrigger post is the same. The cradle ties the
  two into a portal, but expect some camera shake on rough floors.
- **6705ZZ land.** Ø27.0 against the inner race OD and the ZZ shield. Check
  the bearing's d1 so the rotating land cannot rub the stationary shield.
- **MJF holes.** Ø1.8 pilots in MJF print undersized and fill with powder.
  Plan to drill them.
- **Price.** "$0.275/g with a $1 floor" is unverified, and dyeing costs
  extra. Re-quote in the JLC3DP instant quote.
- **Pan centring.** The horn is fixed to the yoke before the spline engages,
  so pan zero lands on an 18° spline step, up to ±9° off. The firmware has
  limits but no trim.
- **Tilt centre screw.** Driving it needs a PH0 driver at least 45 mm long
  with a shaft under Ø5.6, through the MR128ZZ bore. Re-tightening it later
  means removing the outrigger and the head.
- **Servicing the pan servo** means unbolting the tower from the chassis,
  with nuts under the acrylic next to the main board.
- **Placement on the robot.** The entry gives no position for the 35 × 32 mm
  M3 pattern relative to the carrier board or the front edge of the chassis.
- **No eyebrows.** The SG90 eyebrows were optional and are not provided.

## What is good

- Every gate reproduces exactly, with no excuses. The horn bore is modelled
  honestly.
- The tilt has a real opposite-side idler (MR128ZZ on an outrigger pin), so
  the tilt spline is not a cantilever.
- The tilt torque path is spline → horn → pocket walls.
- The tilt servo goes in from outside through the upright window with its
  tabs on the outer face, and both its tab screws and its lead are reachable.
- Driving the tilt centre screw through the idler bearing's bore is a neat
  answer to the access problem.
- The yoke is cut by a computed sweep of everything that tilts, and the lens
  height is derived from that sweep rather than guessed.
- MJF PA12 is the right choice here. It takes heat-set inserts, needs no
  supports, and every part is one open shell, so no powder is trapped.
- The README's measure-before-ordering table is thorough and specific.

## Scripts

My probe scripts are in
`/tmp/claude-0/-home-user-mcu-tinkering-lab/fb62d1db-4321-5a2a-995b-4b8db64d8229/scratchpad/gimbal/`
(not part of the entry):

- `margin.py`: re-runs the sweep with an enlarged servo envelope.
- `probe.py`: checks screwdriver access to the pan horn and prints the key
  heights.
- `probe2.py`: checks screwdriver access to the head screws and the board's
  insertion path.

## Revision (designer, 2026-10-25)

Everything below was re-checked with the inspector's scripts (`margin.py`) and
with `probe_rev.py` and `margin_split.py`, which sit beside them in the same
scratchpad folder. `common.check` gives **zero clashes and zero excuses** over
pan ±90 × tilt ±45, and `assembly()` now includes the chassis plate, the
keeper, the keeper screws and the two tied-down ribbon stretches. Lens height
is unchanged at 76.5 mm. The panned CoM is 0.24 mm from the pan axis in X.

| Issue | What changed | Evidence now |
|---|---|---|
| **B1** head screws cannot be driven | The bridges and inner cheeks stop 0.35 mm above a Ø3.6 driver envelope on each screw axis (`HEAD_DRIVER_D`, `CHEEK_TAB_Z` = −7.5). A Ø3.6 channel runs forward through the cradle on each axis. The pivot boss is trimmed to the cheek bottom. | Ø2.0, Ø3.0 and Ø3.5 rods along both screw axes cut **0.00 mm³** of the cradle. In the assembled pose they also cut 0.00 of the yoke, outrigger and tilt servo. The XIAO stack allows a shaft up to Ø3.7 on the −Y screw. The cradle's thinnest wall is now 1.5 mm, under the MR128ZZ pocket. |
| **B2** cross horn cannot be installed or screwed on | Pan coupling redesigned. The cross horn's arms are cut to Ø20 (`HORN_TRIM_R`), and the horn goes on the spline from above through the open bearing seat (chamber Ø28.6), with its centre screw driven from above. The horn has no arm screws. A new printed **keeper** ring (Ø26.6) bolts under the yoke's spigot with 4 × M2 × 8, from below, **on the bench**. Its through cross slot takes the arm stubs, and the slot walls carry the torque. I used a trimmed cross horn rather than the disc horn: a disc gives the pocket walls no flat to push on, so its torque would go back through screws. | A Ø20.4 disc lowered down the pan axis cuts 0.00 mm³ of the tower, and so does a Ø5 driver to the centre screw. Nothing is screwed from below once the tower is assembled. |
| **M1** pan axial over-constraint | The keeper clamps the 6705ZZ's inner race against the yoke's land. The spigot is 0.2 mm short of the race width (`SPIGOT_SHORT`) so the clamp lands on the race. The horn floats **±1.0 mm** axially in the keeper's slot (`KEY_FLOAT`). Yoke, bearing and keeper are one subassembly, and the outer race sits in a seat with three crush ribs (0.15 mm interference). The bearing locates the yoke axially and carries the moment. The servo shaft sees torque only. | Arms at 33.4–35.0, slot 32.4–36.0 (world z). The residual risk, an outer race retained only by the crush fit (plus optional CA), is listed in the README. |
| **M2** windows fitted to the smaller figure | `FIT_END` 0.15 → **0.35** (window 23.3), `FIT` stays 0.3 (12.6), and a new `FIT_TAB` 0.6 (tab cavity **32.7**, which takes the SG90's 32.3). The pocket roof now sits 0.5 mm above the case top (`CASE_TOP_CLEAR`). | `margin.py big` (22.8 × 12.2): **0 poses with a clash** (was 3.8 / 1.8 mm³). `margin.py tallF`: 0. At ±0.25 only the tabs overlap (26 mm³ per servo, the margin growing each tab into its own ledge; the case body is 0.00, per `margin_split.py`). At ±0.5 the case is 13.0 wide against a 12.6 window, which is beyond the fit clearance by construction. |
| **M3** 1.1 mm web beside self-tapped pilots | All 12 M2 holes are **tapped** (Ø1.6 drill, M2 × 0.4 tap) for M2 machine screws: tab screws M2 × 8, keeper M2 × 8, outrigger M2 × 6. A tap removes material, so nothing pushes the web sideways. With the larger end clearance the web is now **1.0 mm**. That web is set by the servo's drawing (hole centre 2.15 mm from the case end), so no mount gets more. The servo case is 0.35 mm away and stops the web bulging. Nuts on the far side do not fit: an M2 nut beside the case end would sit inside the case. The tilt upright's 3 mm gives 7 threads for a screw whose only job is to stop pull-out, since the window takes the torque reaction. | The gate reads 1.0 mm on the tower and the yoke, both at a tab hole. The README asks for a printed coupon before ordering the set. |
| **M4** pan servo 0.5 mm above the plate, unchecked | `PAN_SERVO_LIFT` 0.5 → **2.0**. The lowered bearing seat keeps the lens at 76.5. `assembly()` has a `chassis_plate` stand-in (group `tower`). | `margin.py tallF` (F + 1) passes with the plate present. Up to F + 2.0 clears. |
| **M5** ribbon and leads not modelled or guided | The yoke gets a **ribbon shelf** on its rear edge (2 tie slots) and a straight **lead gutter** under its rim from the tilt servo (100°) toward the shelf (148°), with a tie slot. The tower gets a **ribbon post** at the rear (2 × 2 tie slots), which is the strain relief before the carrier. The tied-down stretches on the shelf and post are bodies in `assembly()`. `ribbon_spans()` prints each free span's chord over the full sweep. | Clash gate: ribbon_shelf and ribbon_post clash with nothing. Tilt span chord 18.7–70.1 mm over ±45° (28.6–63.6 over ±30°), so leave 80 mm. Pan span chord 12.2–42.0 mm over ±90°, so leave 60 mm. At ±90° a taut pan chord passes 1.1 mm outside the tower cylinder and above the box, flange and M3 heads. The swept head clears the shelf top by ≥ 9 mm. The free spans themselves are still not swept: a flexible ribbon has no single shape to check. |

Minor items:

- The tie slots, the gutter and the post were fitted to MJF's 1.0 mm. A
  curved gutter first read 0.14–0.26 mm at its end faces. That reading is the
  ray gate's tangent effect on a convex wall's end, but the gutter is straight
  now, so the gate has nothing to argue about.
- The README now states the firmware pan offset needed for the 18° spline step
  and the horn-slot backlash (about ±1°, file the slot to fit).
- The price rate is still unverified and the README says so.
- The eyebrows remain out of scope.
