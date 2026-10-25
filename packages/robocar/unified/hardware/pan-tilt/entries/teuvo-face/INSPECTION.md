# Inspection: teuvo-face

Independent re-check, 2026-10-25. I ran `design.py --brows`, `common.check` and
`common.render` again from `hardware/pan-tilt/`. To test insertion paths and
fixture sensitivity I also ran probe scripts that import `design.py` and
`common/` read-only (kept in the session scratchpad, not here). Nothing in
`common/` or `design.py` was changed.

## 1. Gates reproduce

- `design.py` rebuilds all seven parts. All are valid, watertight, single-shell
  meshes with no enclosed voids, so no powder or resin is trapped. The torque
  budget matches the README: 23.4 g, 6.1 mm, 0.044 kg·cm.
- `common.check` returns **0 clashes, rc 0**. The regenerated `check.json` is
  identical to the committed one.
- `--brows` matches the README: the only touch is brow -18° against brow -18°.
- The renders regenerate unchanged. I looked at all four.
- The five excuses are honest: four spline/horn-hub pairs and the M3 shank
  through the 683ZZ bore. Each horn and its holder are in the same group,
  though, so the gate never tests the horn-in-pocket fits. I checked them
  myself: pan horn/frame, tilt horn/eye and brow horn/brow all have 0 mm³
  overlap. The eye/pivot-screw overlap (10 mm³) is the thread inside its
  pilot, which is correct.

**But zero clashes at the assembled poses does not mean it assembles.** The
gate tests where the parts end up, not how they get there. I swept each
stand-in along its documented insertion path. Three of those paths are
blocked.

## 2. Findings

### Blockers

**B1. Head 1 cannot be fitted into the eye cradle.** README step 6 says to push
it up from below. The eye's bottom bar (`make_eye`, `box(10, BEZEL_X1, ±ARM_OUT,
-13, -11)`) sits directly under the XIAO stack, which spans x -4.4…21.6 in
tilt-frame coordinates.

| Path | Overlap with the eye | What blocks it |
|---|---|---|
| From below | 413 mm³ from 2 mm of travel onward | Stack footprint (11.6 × 17.8) on the 2 mm bar |
| From above | 81–432 mm³ | J2 (header plus plug) hits the back plate |
| From behind or in front | Blocked | Back plate, bezel |

**Fix (tested):** delete the bottom bar where it lies under the stack, or start
it at x ≥ 21.7 so it only carries the bezel. With that change, insertion from
below is clear the whole way (0 mm³).

**B2. The tilt servo cannot slide into the +Y pod past the forehead bosses.**
The four `cyl_z(±POST_X, ±POST_Y, a+7 … zi1)` bosses hang from the pod top at
z 75–79 world, x 2.3–6.7, y 33–37.5. The upper MG90D tab reaches z 78.45 across
its full 12 mm width.

- Sliding the servo in along -Y clashes the frame by 0.2 / 38 / 46 / 14 mm³ at
  2 / 4 / 6 / 8 mm short of seated. The cable boot plays no part: the result is
  the same with `cable=False`.
- Dropping the servo 4 mm to pass under the bosses does not help. The case
  enters the bulkhead window (6.2 mm before seating) before the tabs clear the
  bosses (2 mm before seating).

**Fix (tested):** delete the four bosses. The pod-top pilots stay. The insertion
sweep is then 0 mm³ at every offset. The bosses do nothing today anyway; see M1.

**B3. The pan servo cannot be lowered into the base with its lead.** The lead
boot sits at z 8–12, behind the pocket's -X end. The rear window (z 6.5–13)
only lines up with it once the servo is seated.

- On the way down, the boot hits solid plinth wall between z 13 and the ledge
  at 22.8. Lowering the stand-in clashes 23 / 68 / 84 / 59 mm³ at 2 / 4 / 8 / 12
  mm above seated. Without the boot it clears at every offset.
- The lead cannot be bent flat into the gap instead. The end clearance is
  0.3 mm and the lead is about 1 mm thick.
- The obvious fix, opening the window up to the ledge, cuts the rear tab pilot
  at x -18.75, y 0. That pilot is directly above the lead exit.
- The lead exit height is also a fixture estimate. On SG90/MG90-style cases the
  lead often leaves just under the tab.

**Fix:** caliper the exit height first. Then cut an open-topped notch beside the
pilot, for example y +1.2…+5.5, and route the lead sideways. An alternative is
an open rear end with a screw-on keeper for the rear tab.

### Major

**M1. The forehead hangs on about 1.9 mm of thread.** The M2 × 20 heads sit on
the cavity floor, 31.1 mm above the tilt axis. That puts the tips at 11.1 mm,
inside the 2 mm pod-top wall (11–13 mm). The screws never reach the bosses
(7–11 mm).

- So about 40 g of resin and two SG90s (and their reaction torque) are held by
  four M2 screws engaging about 1.9 mm of PA12. One knock will strip them.
- **Fix:** use M2 × 25, or shorten the posts or the plate. Better: once B2
  removes the bosses, thicken the pod top or move the posts outside the pod
  interior.

**M2. The README says the design tolerates F up to 20.8 mm. It does not.** I
re-ran the sweep with an MG90D stand-in 1 mm taller below the tabs (C 29.5,
F 20.8).

- The pan pocket absorbs it.
- The **tilt servo's base hits the +Y ear cap rim** at every pose (24 mm³). The
  servo base sits at y 50.9 and the cap's 2.5 mm press rim reaches y 51.5, so
  there is only 0.6 mm of room.
- **Fix:** lengthen the pod (`POD_Y1`) by about 1.5 mm, or shorten the rim.

**M3. Every servo lead and the ribbon leave the turning stage far off the pan
axis, and none of them is modelled.**

- The tilt lead and both brow leads leave through the pod rear-wall slots at
  |y| 40–52, about 46 mm from the axis. From there they have to reach the
  chassis.
- At pan ±60, each lead's anchor point moves about 48 mm. At ±90 the leads wrap
  round the face.
- The brow leads come out of the forehead's open back about 100 mm above the
  plate, with no tie point drawn.
- **Fix:** route all three leads down beside the neck, close to the axis.
  Model at least the lead path and a tie point on the frame floor near r ≈ 10.

**M4. The ribbon is too short and its pan routing is not checked.**

- README step 10 asks for about 40 mm of tilt loop, then about 60 mm of pan
  service loop below the gate, then the run to J1.
- The carrier README gives Lauri's ribbon as 10–15 cm. The loops alone use
  100 mm, plus about 40 mm of drop from the gate (z 39–43) to the plate, plus
  the run to J1.
- So **a new, longer ribbon (about 20 cm, crimped) needs to be on the BOM.**
- The 14 mm flat ribbon drops off the back edge of the 52 mm neck ring. Every
  pan move drags it about 27 mm × θ(rad) across the plinth and flange. It will
  not twist kindly at ±90.

**M5. The servo case clearance (0.3 mm a side) leaves no margin for size
variation.**

- With the fixture margin at 0.5 the check fails everywhere: base/pan servo
  338 mm³, frame/tilt servo 236 mm³, forehead/each SG90 402 mm³.
- Part of that is an artefact: the margin also thickens the tabs into the
  ledges they rest on, which every entry would suffer.
- The rest is real. A servo 0.25 mm over the published width will not go in,
  and MJF holes print undersize.
- **Fix:** make `CASE_CLR` 0.4–0.5 for MJF, or say in the README that the
  pockets may need filing.

### Minor

- **m1. The 683ZZ shim and spacer touch the shields.** The README asks for a
  3 × 5 shim, and allows a spacer up to Ø4.5, that "bears on the inner race
  only". A 683 inner-ring land is about 4.2 mm across and a ZZ shield bore is
  about 4.5. A Ø5 shim rubs the shield and will drag or bind under clamp load.
  **Fix:** specify Ø4.0 shims and spacer, or use a flanged MF63ZZ / F683ZZ.
- **m2. The axial position of the tilt horn is set by a tolerance chain.** The
  chain runs bearing seat, then spacer, then eye width, then horn hub.
  - The spline-to-eye-pocket clearance is 0.7 mm. With the servo 1 mm taller
    above its tabs, the spline bottoms in the eye pocket (5.4 mm³).
  - MJF ±0.3 mm on each link can preload the servo bearing or cost engagement.
  - **Fix:** shim the spacer at assembly.
- **m3. There is almost no margin for fitting the eye onto the spline.** The
  eye must shift 3.5 mm in -Y to clear the spline. The gap to the cheek is
  4.0 mm. That is 0.5 mm of margin before MJF tolerance.
- **m4. Tab screws are hard to reach.**
  - The lower tilt-tab screw (z ≈ 49) has the tilt lead boot (y 42–46,
    z 45–51) in its driver path from the pod end.
  - Each SG90's outer tab screw (y ±33.4) has that servo's own lead boot in its
    driver path from the back.
  - The leads can be pushed aside, but say so in the assembly steps.
- **m5. Do step 5 before step 3.** Press the 683ZZ in before the frame goes on
  the pan horn, so the press force does not go into the pan servo.
- **m6. The tab-pilot webs are thin: 1.1 mm (MJF) and 1.3 mm (SLA 9600).**
  - They are above the process minimums.
  - Self-tapping a 2 mm screw into a Ø1.5 pilot beside a 1.3 mm web of brittle
    9600 resin will probably crack the forehead.
  - **Fix:** print the forehead in MJF too, or open the pilots to Ø1.7 for
    resin.
- **m7. The head screws are slightly long.** An M2 × 5 into a 1.6 mm board
  plus a 3 mm plate pokes out 0.4 mm behind the plate. Use M2 × 4 or a 3.5 mm
  plate.
- **m8. USB-C cannot be reached.** The XIAO's USB-C faces up, and there is only
  about 16 mm under the forehead. Plugging in a cable needs tilt at -45 or the
  forehead off. OTA exists, but say so.
- **m9. Some material and price claims are unverified.** The material name
  "PA12S-HP" and the $0.275/g MJF rate should be confirmed against a JLC3DP
  upload quote. Resin prices are guesses, as the README itself says.
- **m10. The chassis mounting does not say where on the chassis.** The
  42 × 25 M3 pattern and the drilling note are fine. But the README does not
  say where on the acrylic plate the base goes relative to the main board and
  J1. That position also sets the ribbon length (M4).
- **m11. The bezel aperture depends on the unmeasured lens position.** It is
  Ø17 at 0.4–4 mm in front of the estimated lens. The README already lists
  this under measure-first.

## 3. What is good

- A real opposite-side pivot (683ZZ plus M3) means the tilt load is not
  cantilevered on the spline.
- The torque path goes through the spline, and the double horn is pocketed and
  screwed into the eye.
- The tilt axis runs through the camera stack, which keeps the centre of mass
  6 mm off the axis and parallax small.
- The brows sit on the pan stage, so no servo lead crosses the tilt joint.
- The critical servo dimensions (F, E, tab thickness, hole inset, shaft offset)
  are parameters at the top of `design.py`, and the README has a sensible
  measure-first list.
- No heat-set inserts in resin. No closed cavities, so nothing traps powder or
  resin.
- The README is honest about the omitted spline screw, the asymmetric mass and
  the forehead entering the field of view.

## Verdict

The gate result reproduces, but as drawn the design does not assemble.

- **Three blockers.** Head 1 cannot enter the eye (B1), the tilt servo cannot
  pass the forehead bosses (B2), and the pan servo's lead cannot pass the plinth
  wall (B3).
- **Two are one-line geometry fixes,** both checked here: drop the bottom bar
  under the stack, and drop the four pod-top bosses.
- **B3 needs the lead exit measured first,** then a notch beside the rear tab
  pilot.
- **M1 (forehead screws) and M4 (ribbon length) must be fixed before
  ordering.**

## Revision (designer, 2026-10-25)

Every blocker and major issue is fixed in `design.py`, and the gates were
re-run. `design.py --brows` rebuilds eight valid parts; `keeper` is new.
`common.check` gives **0 clashes, rc 0**, with 8 excuses: the 5 from before
plus 3 lead-into-boot contacts. The renders are regenerated. A probe kept in
the session scratchpad repeats your insertion sweeps and sensitivity sweeps;
every result is listed below.

| Issue | What changed | Result |
|---|---|---|
| **B1** head 1 cannot enter the eye | `make_eye()`: the bottom bar now starts at `STACK_FRONT + 0.1` (x 21.7), so it only carries the bezel. The side struts at \|y\| 15.25–19.5 stay | Head 1 pushed up from 40 mm below: 0 mm³ at every 2 mm step |
| **B2** tilt servo blocked by the forehead bosses | The four bosses are deleted. The pod roof is now 6 mm solid (`POD_TOP_T`), and the forehead pilots go straight into it | Servo slid in along −Y from 34 mm out: 0 mm³ at every step |
| **B3** pan lead boot cannot pass the plinth | `_keeper_slot()` cuts a 4.8 mm slot behind the pocket, open at the top and the rear and down into the lead window. The slot takes the rear tab pilot with it. A new printed `keeper` (MJF PA12, 0.29 cm³) slides in from behind on two ribs in grooves, so it cannot lift. It refills the slot to ledge height and carries the rear tab pilot (1.0 mm web). The tab screw pins it along X | Pan servo lowered from 32 mm above with the rigid fixture boot: 0 mm³ at every step. Keeper slid in from 10 mm behind: 0 mm³ against the base and the seated servo. The lead exit is on the README's measure-first list (item 7) |
| **M1** forehead on 1.9 mm of thread | With the roof 6 mm thick (z 11–17 above the axis), the forehead posts stand on it, and the M2 × 20 from the cavity floor (31.1) ends at 11.1: **5.9 mm of thread** per screw, 0.65 mm above the tilt servo's upper tab. The BOM says not to use longer screws | Arithmetic from `POD_TOP`, `FH_Z`, `SG90`; the servo-tab clearance is inside the clash sweep |
| **M2** F 20.8 hits the ear-cap rim | `POD_Y1` 54 → 56 | Sweep with an MG90D 1 mm taller below the tabs (C 29.5, F 20.8) at pan −90/0/90 × tilt −45/0/45: clear. The servo base clears the rim by 1.6 mm |
| **M3** leads leave at r ≈ 46 and are not modelled | New route (see README step 9a). The brow leads come down the back of their pods into low rear-wall slots. The tilt lead drops from its boot. All go down a hole in the pod floor and into a channel under the floor plate, which is now full neck depth. The channel runs inward into the neck's front quadrant pockets. A 2.5 mm cable tie through two floor slots holds the bundle at **r ≈ 12 mm**. The bundle drops to the plinth from there. A flat land stops the channel leaving a knife edge where it crosses the ring. Everything outboard of the tie is fixed to the pan stage. The `lead_brow_l/r`, `lead_tilt` and `lead_tie_l/r` bodies model the route to the tie in their own `leads` group | 0 clashes against the frame, eye, head and both servos at all 35 poses. The tie point moves about 12 mm at ±60° (from 48 mm). At ±60 it stays clear of the pan case. At ±90 it reaches the case's edge (README risk) |
| **M4** ribbon too short, drags on the ring | The ribbon now turns down through a new floor slot under the gate bridge (x −22.5…−20, 16.5 wide). It leaves the stage at r ≈ 21 inside the neck instead of over the ring's back edge. The `ribbon` body models it from under the bridge to just above the pan servo's rear tab. The BOM has a **200 mm** crimped 14-way ribbon with the length budget. *Mounting* places the base within 40 mm of J1 (your m10). The 20 cm ribbon's extra capacitance on the I2C bus is added as a risk that points to the carrier's J12 rise-time check | 0 clashes at all poses. A first version grazed the pan servo's rear tab (0.03 mm³ at pan ±30), so the modelled run now ends above the tab |
| **M5** no margin for servo size variation | `CASE_CLR` is split. `CASE_CLR_W` 0.5 across the width (pockets, windows and pod interior). `CASE_CLR_L` 0.4 at the ends, where the tab pilot webs set the limit: 1.0 mm, the MJF minimum. The README says to file the ends if a servo is longer still | Sweep with every servo 0.45 mm wider and 0.3 mm longer than published: clear. Each SG90 slid into the forehead from behind: 0 mm³ at every step |

Gate numbers that changed:

- Volumes: base 10.6 cm³, frame 29.6, eye 8.6, forehead 19.0, ear cap 2.7,
  keeper 0.29.
- Thinnest walls: base, frame and keeper 1.0 mm (the tab-pilot webs), forehead
  1.2 mm. The rest are unchanged.
- Tilt budget: 22.7 g, CoM 5.7 mm from the axis, 0.043 kg·cm, 49× stall.

Minor findings not addressed in this pass: m10 is covered under M4. The others
stand as you wrote them.
