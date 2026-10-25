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
