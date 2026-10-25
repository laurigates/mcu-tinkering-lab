# Inspection: workhorse

An independent inspection, 2026-10-24. I did not take the README's word for
anything: I re-ran the generator and both gates, read `design.py` line by line,
and probed the geometry with scripts of my own. The scripts only read the
entry. They live in the session scratchpad, not in the repo.

## 1. Reproduction

- `python design.py` writes all five parts. Every part is `valid=True`, and the
  volumes and bounding boxes match the README (base 9.30, yoke 26.65,
  cradle 9.78, brow_bracket 3.71 and brow 0.92 cm³).
- `python -m common.check entries/workhorse` exits 0 with **zero clashes**
  over pan -90..90 and tilt -45..45. It lists four excuses, which are two pairs:
  the pan horn on the pan servo and the tilt horn on the tilt servo. Both are
  spline-in-hub interfaces, so they are honest.
- Each STL is a single watertight shell. No part has a closed internal cavity,
  so no powder or resin gets trapped.
- **Gates reproduce: yes.**

### What the gate cannot see (my own probes)

- **Same-group pairs are never tested.** The base and the pan servo share a
  group, and so do the yoke and the tilt servo. So the gate never checks that a
  servo fits its own pocket. I tested every pair, same group included:
  - At margin 0 the only overlap is the SG90 spline inside its own horn hub
    (45 mm³ each). That is the same interface the MG90D pairs excuse.
  - At **margin 0.5** no pair from different groups clashes. But the pan servo
    overlaps its base pocket by 236 mm³, the tilt servo overlaps its cheek
    window by 153 mm³, and each SG90 overlaps its bracket window by 104 mm³.
    The cause is `FIT` = 0.3 per side, below 0.5. The TowerPro headline size is
    22.8 x 12.2 mm, against the fixture's 22.6 x 12. With the headline size the
    fit is only 0.2 per side, and MJF's tolerance is ±0.3 mm.
- **A taller MG90D (F and C both +1 mm):** no clash. The pan servo has exactly
  1.0 mm (`PAN_GAP`) below it, so F up to 20.8 mm still seats on the tabs. Above
  20.8 mm the case bottoms out and the tabs lift off the rim. That is tolerable.
- **On the tilt side, the dimension that matters is C − F − tab thickness, not
  height.** The horn pocket has 0.2 mm of axial slack and the pivot has 0.5 mm
  of end play. If the real tab is 0.5 mm thinner than the estimated 2.5 mm, the
  servo pushes the cradle about 0.3 mm toward -Y. That still fits, with little
  margin.

## 2. Mechanism and assembly

### Major: the tilt horn's arm screws hit the yoke cheek

**Severity: major, close to a blocker.**

- `cradle()` drills Ø1.5 through-pilots at r = 10 and 16 mm. The README (step 5
  and the BOM) says to drive PB2 x 4-5 screws "from the horn side" through the
  single horn into the cradle arm.
- So the screw heads sit on the servo-facing side of the horn arm. That arm
  surface is flush with the cradle arm's outer face (`Y_ARM_P`), and the cheek
  is only `GAP` = 1.0 mm away.
- The heads are not modelled. I added PB2 pan-head stand-ins and swept tilt:
  - Ø3.8 x 1.3 mm heads clash with the yoke **at rest**. The r = 16 head
    overlaps the cheek beside the window edge at x -16.9. The r = 10 head clashes
    from |tilt| ≥ 30°, where it leaves the window footprint (up to 3.4 mm³).
  - DIN 7981 ST2.2 heads are 1.6 mm high, so the interference is worse.
  - Only heads 1.0 mm high or thinner clear.
- With 1.3 mm heads, the cradle stack between the cheeks measures
  45.1 mm (boss to screw head), against 45.0 mm between the cheeks. The cradle
  either does not drop in at step 6, or it sits pressed against the -Y cheek
  with zero end play and drags.
- **Fix:** drive the arm screws from inside the cradle. Use Ø2.2 clearance in
  the arm, with the screw tapping into the drilled-out horn holes, so the heads
  sit behind the board plane at head x -6 and -12, clear of the board. Or set
  `GAP` to at least 2.5 mm, or counterbore the heads into the horn side.
- The pan horn's screws, driven up into the yoke, are fine: I swept them with
  the same heads and found no clash.

### Major: the brow-bracket M2 x 8 screw reaches the PCB

- The bracket's counterbore seats the screw head at head x -8.0. The cradle's
  blind pilot stops at x -1.2, and the board's back face is at x 0.
- An M2 x 8 (length under the head) therefore ends exactly at the board face,
  so it would punch through the 1.2 mm pilot floor or strip it. The tip lands
  at y ±11.25, z 16 on head 1, about 3 mm outboard of the XIAO socket rows.
- The README says "M2x8 stops short of the board", which is false.
- **Fix:** use an M2 x 6 (tip at x -2) and change the BOM.

### Major: the cradle back plate presses on the XIAO socket pin tails

- The back plate is a solid 5 mm slab. It lies flat against the back of the
  board from head z 8.5 to 29.5.
- The XIAO sits in through-hole female headers on the front face. Their pin
  tails and solder fillets stand about 1-2 mm proud on the back, at
  y ≈ 0.5 ± 7.6 and z ≈ 9-24.5. That is inside the plate's footprint.
- Tightening the two M2 screws then bows the board over 14 solder joints, or the
  head never seats.
- `fixtures.head()` does not model the back-side tails, so the gate cannot
  catch this. It is still the design's job.
- **Fix:** add a relief pocket about 3 mm deep under the two socket rows, or
  stand the board off on two bosses around the M2 holes.

### Tilt support and torque path: good

- The tilt is **not** cantilevered on the spline. The -Y side has an M3 screw
  clamping a 4/3.1 brass sleeve to a boss on the cradle, and the sleeve turns in
  a Ø4.1 PA12 bore.
- The screw head bears only on the sleeve end, because the sleeve stands 0.5 mm
  proud of the cheek. That is a nice detail.
- Torque passes spline → horn hub → horn arm → pocket walls on both axes, not
  through the screws.
- The pan stage *is* carried on the pan spline alone, with no thrust ring. That
  is the norm for 9 g kits and acceptable with the MG90D's bearings. But the
  moving mass is about 80 g (the yoke alone is about 27 g of PA12, plus the
  outboard tilt servo, cradle, head and brows), with its centre of mass offset
  toward +Y. **Minor.**

### Servo insertion and screwdriver access: works

- **Pan servo:** it slides in sideways through the open -Y side of the pocket.
  The lead root travels in the slot, which is open to -Y below z 14. The tabs
  ride over the full-width end walls.
- **Tilt servo:** it goes shaft-first into a closed window from outboard. Only
  the 8.7 mm above the tabs passes the cheek, and the lead stays outboard.
- **Pan horn screw:** access is through the Ø5.6 hole on the axis, before the
  cradle goes on.
- **Tilt horn screw (step 7):** a PH0 shaft (3 mm) goes through the Ø4.1 bore
  and the Ø3.4 M3 hole, along the axis. The horn screw itself has to be placed
  through the open cradle first, because its head is wider than 3.4 mm. This is
  fiddly but possible, and only before the head is fitted. **Minor:** say so in
  the README.
- **Horn arm holes:** stock horn holes are about 1 mm, meant for pushrod wire.
  The PB2 screws need them drilled to about 2 mm. The README only says "drill
  if your horns differ". **Minor.**

### Cables

**Ribbon: major risk, unproven.**

- The ribbon runs from the J2 plug down to a 17 x 3 mm slot at world x -27.5,
  where it is tied to the bar behind it. That part is sensible: tilt flexes
  about 16 mm of free loop.
- Below the plate the problem starts. The slot orbits the pan axis at a 27.5 mm
  radius. The ribbon's lower end is fixed at J1, about 35 mm lower.
- At ±60-90° pan, the 14 mm-wide ribbon is twisted and dragged round the
  base's rear corner, which is only 4.5 mm behind the slot at pan 0. The pan
  lead's slot exits on the same corner.
- No guide, no clock-spring loop and no clash body exists for it. The README
  itself says the current 10-15 cm ribbon is probably too short.

**Servo leads: minor.**

- The tilt servo lead leaves the outboard end, about 45 mm out at the tilt-axis
  height. The README sends it to tie slots at y 12.5 and 17.5, which are inboard
  of the cheek, so it has to cross over the cheek or round its rear. That works,
  but it is unspecified.
- The brow leads simply "follow the ribbon".

**Eyebrows: minor.**

- The bracket bolts on behind the cradle. The SG90s point forward through
  windows above the board top, and the brows rotate in the face plane.
- The upper tab of each SG90 is glued, not screwed.
- Each brow covers its horn hub with 2 mm of resin, so the stock SG90 horn
  screw (about 4 mm) will not reach the spline. Specify an M2/PB2 x 6.
- The brows sit about 14 mm behind the camera's front face and raise Teuvo to
  about 120 mm. That is a styling choice, but a tall, top-heavy tower on a
  small 2WD.

## 3. Printability at JLC3DP

- **Material choice:** MJF PA12-HP for the structure and SLA black resin for the
  cosmetic brows. That fits the rules: PA12 takes self-tapping screws and
  survives knocks, and the repo's own enclosure notes say resin is poor with
  inserts. No supports are needed, and parts are exported in their world pose,
  which the README says.
- **Thinnest wall: 1.1 mm** on the base and the yoke. Each is the ligament
  between a Ø1.5 tab pilot and the servo pocket or window. JLC's MJF minimum is
  about 1.0 mm, and I did not re-verify that figure online. A local ligament
  beside a self-tapping screw in ductile PA12 is acceptable, but marginal.
  **Minor.** Opening the pilot to 1.6 mm makes the ligament thinner, so the
  better fix is to move the screw outward if calipers allow it.
- **Hole sizes:** Ø4.1 bore (to be reamed), Ø3.4 for M3, Ø1.7 M2 pilots and
  Ø1.5 servo pilots. All are sensible for MJF ±0.3 mm. The hex nut pocket is
  5.8 AF, open on the arm's inner face, and the head board blocks the nut from
  falling out. Good.
- **Prices** ($0.275/g MJF) are the entrant's figures. I did not re-quote them,
  so treat them as estimates. Two processes (MJF plus SLA) mean two per-part
  minimums.

## 4. Fit to the robot

- **Chassis mount:** 4 x M3 on a 20 x 34 mm rectangle in a 3 mm flange. Each
  hole has about 2.3 mm of material to the flange edge. A pan-lead hole through
  the floor needs a matching chassis hole. The pattern is sound, but the README
  does not say where on the plate it goes relative to the main board and J1,
  and that matters for the ribbon length.
- **Head-board M2 holes:** the pilots are coaxial with H5/H6. With an M2 x 6
  button head, about 4.4 mm engages the 5 mm plate. Good.
- **J2 keep-out:** the back plate starts 1 mm above the keep-out, and the
  sweep clears it at ±45° (`HEAD_DROP` 24.5). Plugging the ribbon before
  mounting the head is required, and the README says so.
- **Height:** the tilt axis is 65.5 mm above the chassis and the head top about
  83 mm, or about 120 mm with the brows. The swing radius is about 52 mm, set by
  the outboard tilt servo. Check this against anything mounted near the head
  column. **Minor.**

## Verdict

- The gates reproduce cleanly, and the architecture is the strongest kind:
  a U-yoke, the horn captured in a pocket, and a real second pivot on brass.
- Three unmodelled hardware details would stop or damage a first build, and
  none needs a redesign:
  - the tilt horn screw heads in the 1 mm cheek gap;
  - an M2 x 8 that reaches the PCB;
  - a flat back plate on the XIAO socket tails.
- The ribbon's path through the pan sweep is the largest open risk.

## Revision (designer, 2026-10-25)

All four major issues were fixed in `design.py`. `python design.py`, then
`common.check` (exit 0, **zero clashes** over pan ±90 and tilt ±45, the same
four spline excuses and no new ones) and `common.render` were re-run.
`output/check.json` and the renders are current. I also re-ran a probe of
every pair at pan/tilt (0,0), (60,30), (-90,-45) and (90,45), same group
included. Only the intended interfaces overlap: the splines in their hubs,
the horns in their pockets, and the board on the cradle.

### Tilt horn arm screws in the cheek gap: fixed

- The screws are now driven **from inside the cradle**. `cradle()` has Ø2.2
  clearance holes through the +Y arm, with Ø4.4 x 1.0 counterbores on the
  inner face.
- They are PB2 x 4 pan heads (Ø4.0 x 1.6). Each taps into the single horn's
  arm hole, drilled to 1.6. The tip ends flush with the horn's servo-side face
  at `Y_ARM_P`, so nothing stands in the 1.0 mm gap. Boss to arm face, the
  cradle is 43.5 mm between cheeks 45.0 mm apart, with 0.5 mm of pivot end
  play and the 1.0 mm gap, as designed.
- Hole radii moved from 10/16 to **11/17.5**
  (`HORN_SCREW_R`). At r 10 the two counterbores left a 0.6 mm web. At r 11
  the PH0 shaft clears the back plate and the -Y arm by 0.5 mm when driven
  along Y from the -Y side, on the bench at step 5.
- The heads are modelled as `tilt_horn_screws` (tilt group), so the sweep now
  checks them against the yoke and the tilt servo: no clash. The pan horn's
  heads are modelled too (`pan_horn_screws`), against the pan servo: no clash.
- The brow bracket would have hit the r 11 head. Its bottom edge moved from
  head z 10 to 14.5 (`BROW_BRACKET_Z0`), and the probe confirms it clears the
  head by 0.5 mm.

### Brow-bracket screw reaching the PCB: fixed

- The BOM, README and code comment now say **M2 x 6**. The head seats at head
  x -8, so the tip ends at x -2, 0.8 mm short of the 1.2 mm pilot floor.
- The screws moved from y ±11.25, z 16 to **y ±12, z 18** (`BROW_SCREW_YZ`),
  in both the cradle pilots and the bracket holes. This keeps them 1.7 mm
  clear of the new socket relief pocket, and clear of the horn screw heads.

### Back plate on the XIAO socket pin tails: fixed

- `cradle()` cuts a **2.5 mm deep relief pocket** into the back plate's board
  face. It spans y -8.4..9.4 and runs from below the plate's bottom edge up to
  z 25.7 (`SOCKET_ROW_Y/Z` ± `TAIL_R` 1.3).
- The board now bears only on the two side strips outside the rows
  (|y| > 9.4 to the board edge) and on the top band around the M2 screws.
- The thinnest wall there is about 1.1 mm, at the pocket's corner next to each
  M2 board-screw pilot. The ray gate reports the cradle at 1.2 mm.
- The README lists the row positions and the tail height as things to measure.

### Ribbon through the pan sweep: fixed by design, still unproven on the bench

- **Pan servo turned shaft-rearward** (`PAN_LOC` gains `Rot(Z=180)`). The case
  and lead now point forward, and the lead exits through the front wall. That
  lets the base's rear end be a **drum** of r 12.4 round the pan axis.
- **Ribbon slot** moved from x -27.5 to **16.4 mm behind the axis**. It is
  now 15 mm wide across Y, so the ribbon's flat face faces the axis, with a
  cable-tie slot and a 2.2 mm bar behind it.
- In the ±90° sweep the slot never passes over the base. Its sideways travel
  at ±60° is 16 mm instead of 27 mm.
- **Designed path**:
  1. The ribbon drops vertically.
  2. Just above the flange, a taped 45° fold turns it into an edgewise run.
  3. The run wraps round the drum toward +Y.
  4. It leaves tangentially along a new flat +Y side of the base and goes
     forward toward J1.
- Pan changes only the length lying on the drum: 7-34 mm over ±60°, and
  0-41 mm over ±90°. There is no twist and no corner.
- A 2 mm, 19 mm tall **fence** at r 20-22 round the rear 112°, with straight
  tangent tails, keeps the ribbon on the flange. It leaves the -Y side open,
  because the pan servo slides in from there.
- **Modelled**: `ribbon_drop` (pan group, 2 mm thick for the fold's two
  layers, 15 mm wide, from the flange to the plate) and `ribbon_wrap` (base
  group, the wrapped band and the forward run). Both are in the sweep, with
  zero clashes and no excuses.
- **Length**: the README now gives a budget of about 160 mm plus the run to
  J1, and asks for a **20-25 cm** ribbon.
- **Mounting** changed to 35 x 37 mm (x -19/+16, y ±18.5). The README says to
  keep the +Y side clear for the ribbon's run.
- **Still open**: the keep-outs are rigid. Whether the real ribbon slides
  along the drum when pushed (panning toward -) rather than bowing against the
  fence needs a bench check with `servo exercise`. This is listed in the
  README's risks.

### Minor points also addressed

- The README now says to drill the horn holes: 2.2 mm for the double horn,
  1.6 mm for the single.
- It says to put the tilt horn's centre screw in the hub from inside the
  cradle at step 5.
- It specifies a PB2/M2 x 6 brow horn screw.
- It routes the tilt and brow servo leads with the ribbon.

### Not changed

- `FIT` stays at 0.3 per side; see the README's measurement table.
- The 1.1 mm tab-pilot ligaments on the base and yoke are unchanged. They are
  mirrored on the base, which is the same geometry.
- The pan stage still rides on the pan spline alone.
