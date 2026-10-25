# Inspection: `wildcard` (Majakka)

Independent inspection, 2026-10-25. I did not rely on the README's own claims.
Scripts are in the inspector's scratchpad. They are not part of the entry.

## 1. Do the gates reproduce?

Yes.

- `python design.py` writes the 8 STEP and STL pairs. Every part is valid and
  the tilt table matches the README.
- `python -m common.check entries/wildcard` gives **zero clashes** over pan
  ±90 and tilt ±45, and the result is byte-identical to the committed
  `output/check.json`.
- `common.render` writes the 4 PNGs.
- The two excuses are honest. Each is a stock horn on its own servo's spline
  (`pan_horn / pan_servo`, `tilt_horn / tilt_servo`). Nothing else is excused,
  and the gear mesh has no excuse.
- With every servo envelope grown by **margin 0.5** (`fixtures.servo` patched),
  the full sweep still gives zero clashes.
- With an MG90D that is **1 mm taller below the tabs** (C+1, F+1, A+1), the
  sweep gives zero clashes at pan 0/±90 and tilt 0/±45. However, the tilt
  servo's base then touches the base's inner -X wall with 0.0 mm left (case end
  at x -44.5 against the wall at x -44.5). The check reports no clash only
  because touching faces have zero volume.

## 2. Mechanism and assembly

### BLOCKER: the tilt fork cannot be assembled

The fork has three parts that each need a different approach direction, and
the parts around it allow none of them:

- **Pressing the horn onto the spline needs the fork to move ~4 mm along X.** I
  slid the fork along +X: it hits the base's pivot pillar after **0.5 mm**
  (24.6 mm³ of overlap at 1 mm, 200 mm³ at 4 mm). The far pivot boss ends at
  x 12.0 and the pillar starts at x 12.5. The horn hub runs 0.5..4.5 mm above
  the case on a 4 mm spline, so it cannot be put on or taken off in 0.5 mm.
- **The tilt servo cannot come to the horn instead.** Its tabs bear on the
  bulkhead's front face. Moving the servo -X to clear the spline drives the
  31.5 mm tab span into the 3 mm bulkhead, which has a 23.2 mm window.
- **The two dowels trap the fork in X**, because the dowels lie along the same
  axis as the spline. With both dowels in the groove, any X shift drives one
  dowel tip (|x| 4.5) through the collar core (R 4.2).
- **The near dowel cannot be pressed in afterwards.** README step 8 says to
  press both dowels "from outside the plates". Outside the horn plate
  (x -10.5) there is 4.5 mm to the servo case top (x -15), with the horn hub
  and arms in that gap. The dowel is 6 mm long.
- **The far dowel and pivot dowel are the only accessible pins**, through the
  front service window and the pillar.
- **The fork cannot be screwed to a horn already on the spline.** The two
  M2×5 screws go into the horn at y -5 and -17, and the far fork plate, which
  is solid at those points, blocks the screwdriver.
- **The collar cannot be fitted after the fork.** The plunger goes in from
  below, and its Ø14 flange cannot pass between dowel tips 9 mm apart.

The design works only as a CAD pose. Possible fixes:
- Pivot the fork on two dowels in the base, both pressed from outside, and
  drive it from the horn through a radial slot or a pin in a slot rather than
  coaxially.
- Or make the pillar a separate screwed cap with a U-slot, so the boss drops
  in from above. This still needs a different answer for the near dowel, such
  as a pin printed into the fork or a screw driven in from inside the tine.

### MAJOR: the tilt servo cannot be "lowered from above" as the README says

The tilt servo case is 28.5 mm long. The deck opening runs only from the inner
wall at x -44.5 to x -17.7, which is 26.8 mm. The case top end, from x -17.7
to -15, plus the spline, sits under a solid deck strip.
- Lifting the servo straight up clashes with the deck: 122 mm³ at +20 mm.
- Shifting it -X first is impossible because there is 1 mm to the wall.

A lazy BFS over pitch, X and Z (5° and 0.5 mm steps, base only, lead not
modelled) found a way out. The servo has to be swung through about **70°**,
spline end up, through the opening. So the servo can probably be inserted, but
only by a near-vertical swing with zero design clearance at the deck edge. The
README's assembly step 2 is wrong. A pocket in the deck, or a deck edge moved
3 mm forward, would turn this into a straight drop.

### MAJOR: the pinion's height has ±0.5 mm of room and depends on three estimates

The pinion clears the deck by 0.5 mm below and the turret floor by 0.5 mm above:
- Moving it down 0.8 mm puts 9 mm³ into the base.
- Moving it up 0.8 mm puts 12 mm³ into the turret.

Its height is set by:
- the lintel height (from `tab_underside`, an estimate in effect);
- `height - tab_underside` (case above the tabs);
- the disc horn's seat height (fixture estimate 4.5 mm);
- MJF ±0.3 mm on the lintels.

None of these is measured, and the README's "measure first" list does not tie
them to this clearance.

### MAJOR: gear centre distance is not controlled

- The pan servo sits in its pocket with `FIT` 0.3 mm per side, and the pocket's
  short axis (servo width) is the gear centre line. So the centre distance can
  move ±0.3 mm before MJF tolerance on the bearing bore is added.
- The designed backlash of 0.15 mm of arc is only about 0.2 mm of radial
  play.
- If the servo screws pull it 0.3 mm toward the turret, the gears bind; if
  away, the pan has ~1.5° of slop.

Fixes: locate the servo along Y with a datum (pocket width tight on one side,
or slotted tab holes to set the mesh), or add backlash.

### MAJOR: the far fork pivot is over-constrained

- The far fork plate turns on the Ø4 dowel in a Ø4.3 journal (0.15 mm per
  side), which must be coaxial with the tilt spline.
- The tilt servo is located only by its tabs and a window with 0.3 mm per side
  of fit. A shaft-offset error of about 1 mm (the fixture's 6.0 is an
  estimate) pushes the case against the window edge and moves the shaft about
  0.7 mm off the dowel axis.
- Either fault binds the fork or side-loads the spline. The README covers the
  shaft offset only as "moves the tilt servo's body into the window's edge".

### MAJOR: the unshrouded gear mesh is exposed to the head ribbon

- The turret gear (tip R 13.6) sits on the deck. The only cover above it is
  the turret plate (x -6..19.5 in the turret frame) and an R10 disc. The mesh
  point is fixed at world (0, 12.5).
- At pan 0 to +90 the plate covers the mesh. At **pan -60 to -90** it does not:
  the mesh lies outside both the plate and the disc.
- In that range the head faces -Y, so J2 and the ribbon point +Y, toward the
  pinion.
- The ribbon is the one cable that crosses the pan joint. With its README
  30 mm loop draped over the deck, it can be fed into the mesh.

A printed guard over the pinion (fixed to the base) is needed.

### MAJOR: the firmware changes are not acknowledged as changes

- `design.py` and the README say to flip the pan sign "in the console, `servo`
  command". `main.c`'s `servo` command has no direction or invert option
  (usage: `servo | pan|tilt <deg> | on|off | limit | exercise | freq`).
- The 1:1 reversed pan and the non-linear tilt table (servo 0.82° per head
  degree, +37.6/-33.1 at ±45) both need code in `servo_controller.c`.
- Until that code exists, `head_aim` pans the wrong way, a positive feedback
  loop that winds the head to its limit, and tilts about 20 % too far.

### MAJOR: the head board will not sit flat on the cradle

- Head 1 carries the XIAO in two 1×7 **through-hole** female headers
  (`docs/reference/parts/seeed-xiao-esp32s3-socket.toml`). Their pins and
  solder joints stand about 1–2 mm proud of the back face at head y ±7.6,
  z ~9–24.
- The cradle's back plate (x -3..0, z 8..29.5, full width) is flat against the
  back face with no relief.
- Screwing the M2 button heads into the inserts will bow the board over the
  pins.

`fixtures.head()` does not model back-side THT protrusions, so the gate cannot
see this. It is a fixture gap, but this entry is the one with a full-face
plate.

### Questions asked by the inspection brief

- **Is the horn captured, and does the torque go through the spline?** Yes:
  - The disc horn is screwed (3× M2) to the pinion.
  - The trimmed double horn is screwed (2× M2) to the fork.
  - Both are on their splines with the horn screw.
- **Is the tilt load cantilevered on the spline alone?** No, and this is the
  entry's real strength. The head hangs on two M3 cheek pivots, and pan loads
  go into two 6801ZZ bearings. The fork's far side rides on a dowel, though see
  the over-constraint above.
- **Do the servo leads survive the sweep?** Yes. Both servos are fixed, so
  neither lead crosses a moving joint. The pan servo's lead boot (rigid in the
  fixture) clashes with the -X lintel while the servo is lowered (50–83 mm³ at
  +5..+10 mm). The real lead bends, so this is only a note.
- **Does the IDC ribbon survive the sweep?**
  - J2's keep-out clears everything over the sweep.
  - At tilt 0, the ribbon must first run straight back past the turret plate
    edge (x -6, 1.5 mm beyond the keep-out) before it can bend down.
  - The real ribbon beyond the keep-out is not modelled. Pan travel depends on
    how much loop is left, and on the gear-mesh exposure above.
- **Screwdriver access to each horn screw:**
  - Pan: through the pinion's Ø5.2 hole, clear from above.
  - Tilt: through the pillar's Ø3.9 press hole. The horn screw head has to be
    pre-placed in the fork's Ø4.6 pocket and must pass the Ø4.3 far journal.
  - The tilt servo's tab screws are reachable through the service window and
    the Ø5 hole, but only before the fork is in. Taking the tilt servo out
    means taking the fork out, which is the blocker above.

## 3. Printability (JLC3DP MJF PA12-HP)

| Finding | Severity |
|---|---|
| Walls of 1.0–1.35 mm meet JLC's recommended 1.0 mm for MJF. The pan servo's tab pilots sit **1.05 mm** from the lintel face. A self-tapping screw of about 2.2 mm leaves about 0.7–0.9 mm, and with MJF ±0.3 the lintel can split. Nylon is tough and the load is small. | minor |
| The hub is line-to-line in the bearings: `BRG_D/2` = 6.0 against a 12.0 bore, with MJF ±0.3. If the hub prints oversize it will not enter the bearings. If undersize, the turret rocks by up to 0.15 mm per side over a 1.5 mm lip span, which defeats the point of the bearings. The README only says "sand or ream". Give the hub a deliberate −0.1 mm and plan to sand to fit. | major |
| The E-clip groove (Ø9) is printed in PA12 on a hollow hub: the corner wall over the 4.4 mm square bore is 1.4 mm. A DIN 6799 size-9 clip needs a groove of 9 h11. At ±0.3 it will either fail to seat or pop off. A printed groove is a weak retention for the only thing holding the turret down. | minor |
| Ø1.6 blind pilots (5–6 mm deep) and the Ø2.9 dowel press holes are below or at MJF's dependable hole size. Expect powder in them, and drill or ream after delivery. There are no closed cavities. | minor |
| The pivot screws are M3 button heads that turn in Ø3.3 cheek journals, with the threads in the journal and the screws backed off on threadlocker from heat-set inserts. This works, but it wears and creeps. A shoulder screw or a smooth sleeve would be better. | minor |
| The PA12 MJF material choice is right for press-fit bearings, inserts and sliding parts. The per-part prices ($0.275/g, about $1 floor) are unverified estimates. Total is about $18. | n/a |

## 4. Fit to the robot

- **Chassis mounting:** four M3 holes in 9 mm ears, 2.5 mm thick, on a
  75.0 × 51.3 mm rectangle. The pattern is fine.
  - The footprint is 84 × 61 mm and the lens sits 70.6 mm above the plate.
    Whether it fits beside the main board is unverified.
  - The open floor pit puts the fork 1.3 mm above the acrylic. Chassis slots
    and nut heads under the pit can jam the tilt.
  - The plunger and E-clip go in from below, so the base must come off the
    chassis for any service.
- **Head board M2 holes:** inserts on the fixture's `HEAD_HOLES_YZ`, which is
  correct. See the THT pin problem above.
- **J2 keep-out:** clear over the whole sweep. The head is offset 6.5 mm to
  -Y for it.
- **Eyebrows:** none. The README explains why, which is honest.

## Summary

The concept is well argued and better than the alternatives:
- both servos stay fixed;
- bearings carry the pan;
- the head turns on two pivots;
- no servo lead crosses a moving joint;
- the gates honestly pass.

But it is not buildable as drawn:
- The tilt fork cannot be assembled. The spline axis, the dowel axis and the
  pillar all lie on X, which leaves no assembly direction.
- The tilt servo needs a 70° swing that the README does not describe.
- Several ±0.3–0.5 mm interfaces (pinion height, gear centre distance,
  hub-to-bearing fit, fork coaxiality) have no adjustment.
- The pan sign and the tilt table need firmware work, while the README claims
  a console setting covers the sign.

## Revision (designer's response, revision 2)

The gates were re-run after every change: `output/check.json` has zero clashes
over pan ±90 and tilt ±45, with the same two excuses (each stock horn on its
own spline). The assembly claims below were checked by moving each part out
along its assembly path in CAD, against the parts already in place at that
step. The probe script is in the designer's scratchpad, not in the entry.

| Issue | What changed |
|---|---|
| **BLOCKER: the fork cannot be assembled** | **Fixed by removing the fork.** It is now a one-sided `lever`: one plate on the tilt horn, with one Ø3 x 8 dowel pressed into a boss on the bench. Gone are the far plate, the crossbar, the Ø4 pivot dowel and the base pillar. The **collar is a separate part**. It goes in last: up through the floor pit at y +9.5, then slid -Y so the dowel enters the groove from the side. The dowel tip at \|x\| 4.5 never meets the R4.2 core. Then the square rod drops in from **above** through the hub into the collar's socket, and an M2 goes up through the collar's counterbore. Order: tilt servo, then lever (pressed onto the spline along X with nothing in front: 0 mm³ over 0.5–6 mm of +X travel), then turret, then collar, then rod. The horn screw is driven through a new Ø6 hole in the front wall on the spline axis. Probes: lever +X 0 mm³; collar up then slide 0 mm³; rod down from +60 mm 0 mm³. |
| MAJOR: the tilt servo cannot be dropped straight in | Fixed. The deck opening now runs to x -14.4, past the case top at -15 by FIT + 0.3, with a 4.6 mm notch for the spline. It stays 1.9 mm off the bearing housing. Lifting the servo +Z from its seat, 0.5 to 40 mm, gives 0 mm³. README step 2 now matches. |
| MAJOR: the pinion has ±0.5 mm of vertical room | Fixed. The pinion now has **±1.5 mm** to the deck and to the new hood (`PINION_Z_CLEAR`). The turret gear's face is 7 mm, against the pinion's 5, so the pinion stays fully engaged within ±1 mm. The pinion moved ±1.5 mm gives 0 mm³ against the base, hood and turret. "Measure first" item 1 now ties F, C and the horn seat to this clearance. |
| MAJOR: the gear centre distance is uncontrolled | Fixed. A printed **Y datum wall** on the pan servo's -Y side (0.05 mm nominal gap) now sets the centre distance. An M2 **jack screw** through the +Y wall holds the case against it. The servo moved 0.1 mm into the datum gives 19 mm³, so the face is real. Backlash went from 0.15 to **0.3 mm** of arc, which allows about 0.4 mm of centre-distance error before binding. The cost is about 1.4° of nominal pan play, stated under Risks with a tape-shim fix. |
| MAJOR: the far pivot is over-constrained | **Removed with the fork.** Nothing is coaxial with the tilt shaft any more. A shaft-offset error now only shifts the lever's zero, which centring takes out. The cost: the spline carries the rod's push through the dowel. By my estimate that is under 1 N, about 0.07 kg·cm of the servo's 2.2, and the README says so instead of claiming no cantilever. |
| MAJOR: the gear mesh is exposed to the ribbon | Fixed. There is a new fixed **`hood`** part, MJF PA12, two M2 into bosses under the deck. Its lid covers the pinion and the mesh point (0, 12.5): it runs to R11.5 from the pan axis, 1 mm off the turret neck. Its skirt wraps the pinion and is open only where the turret gear runs, 1.4 mm or more off the tips. The turret plate was raised 4 mm to pass over it, so the lens is now at z 74.9. The hood has its own clash group, so the gate tests it against everything, the base and servos included. It goes in after the turret at pan -90 (0 mm³ down from 40 mm). |
| MAJOR: firmware changes presented as a console setting | Fixed in the docs. The README has a "Firmware changes needed" section. It says plainly that `servo_controller.c` needs a pan sign and the tilt lookup, both in `servo_angle_to_count()`, and what happens without them (the pan runs away, the tilt is about 20 % over). The `design.py` comment no longer mentions the console. I did not write the firmware: the brief keeps the entry inside its folder. |
| MAJOR: head board will not sit flat | Fixed. The cradle's back plate is 2 mm back from the board (`STANDOFF`). The board rests on four pads: the two M2 bosses (R2.6 in front of the plate) and two plain pads at head y ±12.5, z 13, outboard of both socket rows. The M2 inserts now go in from the back, and the head screws become M2 x 8. The 14 socket pins, modelled as Ø2 x 2 mm cylinders, give 0 mm³ against the cradle. |
| MAJOR: hub line-to-line in the bearings | Fixed. The hub is **Ø11.85** (`HUB_D`). The README now says to sand it to a slide fit and bond the inner rings with Loctite 638/641, which fills the gap, so the turret does not rock. |

Minor items carried forward, not changed: the 1.05 mm lintel pilot wall, the
printed E-clip groove (the README now gives a fallback), the powder in small
pilots, and the M3 pivots turning on their threads.

One new finding from this work, about the fixtures and the gate:
`common.check` never tests a servo against its own pocket, because both are in
the `base` group. The inspector's servo-insertion findings were therefore
invisible to the gate. I probed the servo fits separately.
