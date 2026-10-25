# The Workhorse: a U-bracket pan-tilt for two MG90D servos

A conventional two-axis pan-tilt sized for the TowerPro MG90D. The tilt cradle
is carried on **both** sides: the servo spline on +Y, and an M3 screw in a brass
sleeve running in a nylon plain bearing on -Y. It has three structural parts,
all MJF PA12-HP, and needs only a screwdriver to assemble. It holds head 1 by its
two existing M2 holes, so it replaces the kit without changes to the board.
An optional bolt-on bracket carries two SG90s as **Teuvo's eyebrows**.

![rest](output/render-rest.png) ![front](output/render-rest-front.png)
![corner](output/render-corner.png) ![back](output/render-rest-back.png)

Gates: `output/check.json` shows **zero clashes** over pan -90..90 and tilt
-45..45, with the eyebrows fitted. Four excuses, each a deliberate interface:
the pan horn on the pan servo's spline, and the tilt horn on the tilt servo's
spline.

## Concept

```
            brows (SG90 x2, optional)
          [==]  [==]
           |bracket|
     +-----+-------+-----+       tilt axis: world Y at z 65.5
  -Y | pin |  head |horn | +Y    (4 mm behind the board face,
cheek|  )==| cradle|==(  |cheek   12 mm above its bottom edge)
     |     +-------+     |[MG90D tilt, outboard]
     +=========yoke======+       yoke rides on the pan horn
              | base |           MG90D pan, upright in the base
          ====+======+====       4 x M3 to the chassis plate
```

- **base**: the pan servo stands upright in a pocket with its tabs on the rim,
  held by its own two screws. The pocket is **open on the -Y side**, so the servo
  slides in sideways and the lead's root never has to pass a wall. The lead
  leaves through a slot in the rear wall and the floor. The flange takes four
  M3 screws.
- **yoke**: a U. Its 6 mm plate has a pocket in the underside that captures the
  stock **double horn** (both arms along Y, so torque goes through the pocket
  walls, not the screws). There is a Ø5.6 hole on the axis for the horn
  screw. The +Y cheek (8.1 mm) holds the tilt servo in a closed window, with the
  body outboard and the tabs on the outer face. The -Y cheek (6 mm) is a Ø4.1
  plain bearing.
- **cradle**: a back plate behind the board, which starts above the J2 plug,
  and two side arms. The +Y arm has a pocket for the stock **single horn**,
  pointing rearward. The -Y arm has a 0.5 mm boss (the sleeve's seat), an M3
  clearance hole, and a captive M3 nut in a hex pocket on its inner face.
- The tilt axis runs through the head's mass (x 4, z 12 in head coordinates),
  so the servo carries almost no gravity torque, and the camera lens sits only
  ~24 mm from the axis, which keeps parallax small for ADR-025's aiming.
- **Eyebrows (optional)**: `brow_bracket` is a flat plate that screws to the
  back of the cradle with two M2x8. Two SG90s pass through its windows with
  their shafts pointing forward, above the board's top edge. Each `brow` is a
  black resin bar that slips over an SG90 single horn: a 10 x 29.5 mm brow
  that the servo rotates in the face plane, so it can scowl, lift or tilt. Set
  `EYEBROWS = False` in `design.py` to drop them. The cradle's two blind M2
  pilots for the bracket are always printed, so the brows can be added later.
  PCA9685 channels 14 and 15.

Envelope: the tilt axis is 65.5 mm above the chassis plate and the head top is
about 83 mm. The brow bracket top is about 120 mm. The assembly is about 85 mm
wide (-Y pivot screw head to the tilt servo's outer end) and swings in a
~54 mm radius.

## Printed parts

The parts are exported in their **world pose** (pan 0, tilt 0). MJF is a powder
bed process: there are no supports and JLC3DP chooses the orientation, so the
pose in the file does not matter. Prices use JLC3DP's MJF-PA12 rate of
**$0.275/g** (JLC3DP price cut, 2025-08-13) and ~1.0 g/cm³. They are estimates:
quote the STLs before ordering, and expect a per-part minimum.

| Part | Qty | Process / material | Colour | Volume | Thinnest | Est. price | Why |
|---|---|---|---|---|---|---|---|
| base | 1 | MJF PA12-HP | grey (or dyed black) | 9.3 cm³ | 1.1 mm | ~$2.6 | Takes the servo's self-tapping screws without cracking, and its isotropic strength suits an upright cup |
| yoke | 1 | MJF PA12-HP | grey (or dyed black) | 26.7 cm³ | 1.1 mm | ~$7.4 | Nylon is a self-lubricating bushing for the brass sleeve. Stiff 6-8 mm cheeks. The closed servo window and cheeks need no supports |
| cradle | 1 | MJF PA12-HP | grey (or dyed black) | 9.8 cm³ | 1.2 mm | ~$2.7 | Holds the M2 board screws, the horn screws and a captive nut. Tough enough to survive a knock to the head |
| brow_bracket | 1 (optional) | MJF PA12-HP | grey | 3.7 cm³ | 1.3 mm | ~$1.0 | Flat plate that takes the SG90 screws |
| brow | 2 (optional) | SLA, JLC black resin | black | 0.9 cm³ each | 1.4 mm | ~$1 each | Cosmetic. Smooth, crisp, and black like 1950s film-star eyebrows. No load on them |

About $14 for the three structural parts, plus about $3 for the brows.

Why not resin for the mechanism? It is brittle under self-tapping screws and
impact, and it wears in a pivot. The repo's enclosure work found it "takes a
thread, not a heat-set insert", which is fine for a case and marginal for a
servo mount. Why not FDM? Its layer lines run across the cheeks in any upright
orientation.

**Wall thickness.** The ray gate's thinnest readings are **1.1 mm**, on the
base and the yoke: the ligament between each servo-tab pilot (Ø1.5) and the
servo pocket or window. This is below the repo's 2.0 mm resin-case convention,
on purpose:

- JLC3DP states a 1.0 mm minimum wall for MJF PA12-HP and ±0.3 mm tolerance up
  to 100 mm.
- The ligament is local, beside a hole. It is not a free-standing wall.
- Its position follows the tab-hole estimate. It moves outward by however much
  the real tab hole sits farther from the case.

Every other wall is at least 1.2 mm (the cradle's blind pilots stop 1.2 mm
short of the board).

## Hardware BOM

| Item | Qty | Where |
|---|---|---|
| TowerPro MG90D | 2 | pan (in base), tilt (+Y cheek) |
| Stock double horn (bag) | 1 | pan, captured in the yoke's underside pocket |
| Stock single horn (bag) | 1 | tilt, captured in the cradle's +Y arm pocket |
| Horn centre screw (bag, M2/PB2 x 4) | 2 | one per servo spline |
| Servo tab screws (bag, 2 mm self-tapping, ~8 mm) | 4 | 2 into the base rim, 2 into the +Y cheek's outer face |
| PB2 x 4-5 mm self-tapping (horn screws) | 4 | 2 through the double horn up into the yoke, 2 through the single horn into the cradle |
| M3 x 14 socket head, ISO 4762 | 1 | tilt pivot |
| Brass tube 4.0 OD x 0.45 wall (3.1 ID), cut to **7.0 mm** | 1 | pivot sleeve: turns in the -Y cheek's Ø4.1 bore |
| M3 hex nut, ISO 4032 | 1 | captive in the cradle's -Y arm |
| M2 x 6 button head, ISO 7380 | 2 | head 1 to the cradle (self-tap into Ø1.7 pilots) |
| M3 x 10 + M3 nut (or M3 x 8 into a tapped plate) | 4 | base to the chassis plate |
| Small cable ties (2.5 mm) | 3 | ribbon to the yoke's rear bar, tilt lead to the cheek's rear foot |
| *Eyebrows:* SG90 | 2 | brow bracket, PCA9685 ch 14 and 15 |
| *Eyebrows:* SG90 single horn + centre screw (bag) | 2 | brows press over them |
| *Eyebrows:* M2 x 8 socket or button head | 2 | brow bracket to the back of the cradle (1 mm counterbore) |
| *Eyebrows:* 2 mm self-tapping (bag) | 2 | lower SG90 tabs |

If you cannot get a shoulder-type sleeve, an ISO 7379-style 4 mm shoulder bolt
with an M3 thread works in place of the tube and screw. Its thread must be long
enough to reach the nut, about 7 mm.

## Assembly (screwdriver: PH0 x 60 mm or longer, 3 mm shaft; 2.5 mm hex)

1. **Ream** the yoke's -Y bore with a 4.1 mm drill (MJF ±0.3 mm). Check the
   brass sleeve turns freely in it. Clear the M2 and M3 holes if powder remains.
2. **Centre both MG90Ds** electrically (`servo pan 0`, `servo tilt 0`) before
   fitting any horn.
3. **Base**: slide the pan servo in from the open -Y side, lead first into the
   rear slot, tabs gliding over the rim. Fit both tab screws. Screw the base to
   the chassis (see "Mounting").
4. **Yoke**: drive two PB2 screws up through the double horn's outer holes
   into the yoke's underside pilots (horn arms along Y). Press the yoke onto
   the pan spline facing forward (+X). Drive the horn centre screw down through
   the Ø5.6 hole on the axis.
5. **Cradle**: screw the single horn into the +Y arm pocket, arm pointing
   rearward, with two PB2 screws. Push the M3 nut into the -Y arm's hex pocket.
6. Drop the cradle between the cheeks. Push the **tilt servo** into the +Y
   cheek window from outside, shaft first, so the spline enters the horn hub
   with the cradle level. Fit the two tab screws into the cheek's outer face.
7. **Tilt horn screw**: pass the PH0 through the empty -Y bore, the -Y arm's
   M3 hole and the empty cradle (no head yet) to the horn centre screw, and
   drive it.
8. **Pivot**: put the brass sleeve in the bore. Pass the M3 x 14 through it
   into the nut, and tighten until the sleeve is clamped to the boss. Then the
   sleeve turns with the cradle and the cradle swings freely with 0.5 mm of
   end play.
9. **Head**: plug the ribbon into J2 first, then fit head 1 to the cradle with
   two M2 x 6 button heads from the front.
10. *Eyebrows*: fit the SG90s through the bracket windows from the front, lead
    first through the notch. Screw the lower tab, and put a dab of hot glue on
    the upper tab. Screw the bracket to the back of the cradle with 2 x M2 x 8.
    Centre the SG90s, fit their horns pointing outward and press the brows on.
    The horn screw goes through the Ø2.4 hole in the brow.

**Cables**

- **Ribbon**: from the J2 plug it runs rearward and down into the slot in the
  yoke plate's rear (17 x 3 mm). Tie it to the 3 mm bar behind the slot as
  strain relief, so tilt flexes only the 16 mm of free loop above the plate.
  Below the plate, leave a service loop of 40-50 mm before J1 on the main
  board, because pan winds it around the base. Lauri's 10-15 cm ribbon is
  probably too short now: J1 has moved and the head sits 65 mm up.
- **Tilt servo lead**: it exits the outboard end. Run it rearward and tie it in
  the two slots at the +Y cheek's rear foot, then let it hang beside the ribbon.
- **Pan lead**: static. It leaves through the base's rear slot.
- **SG90 leads**: they run down behind the bracket, then follow the ribbon.

**Mounting**: 4 x M3 on a **20 x 34 mm** rectangle (20 mm along X, 34 across).
Its centre is 5.3 mm behind the pan axis. Drill 3.2-3.4 mm in the acrylic plate
where the head should stand, and add a 6 mm hole under the rear slot for the
pan lead if it should go below the plate.

## Measure before ordering (calipers on the parts in hand)

All of these are parameters at the top of `design.py`. Change them and rerun
`python design.py` and the two gates.

| Parameter | Now | Source | Why it matters |
|---|---|---|---|
| `tab_underside` (F) | 19.8 | TowerPro page | Base pocket depth: the servo's tabs must sit on the rim, and the case must not bottom out |
| `tab_span` (E) | 31.5 | TowerPro page | Tab pilot spacing on the base and the cheek |
| `tab_hole_inset` | 2.3 | estimate | Tab pilot position, which sets the 1.1 mm ligament |
| `shaft_from_end` | 6.0 | estimate | Shaft position in the pocket and window. Wrong by 1 mm, the spline misses the axis |
| `tab_thickness` | 2.5 | estimate | Where the tilt servo's case top sits relative to the cradle arm |
| Case B x D | 22.6 x 12 | TowerPro (headline 22.8 x 12.2) | Covered by `FIT` 0.3 per side. If yours measures 22.8 x 12.2, the fit is 0.2 |
| Horn hub height, arm thickness | 4.0 / 1.6 | fixture estimate | Pocket depths, and the 1 mm arm-to-cheek gap |
| Horn arm hole radii | 14 (double), 10 and 16 (single) | guess | The PB2 pilots. If your horns differ, drill through the horn's outermost holes into the part on assembly |
| SG90 shaft offset, tab span | fixture | estimate | Brow bracket windows |

## Risks

- **Ligament at the tab pilots (1.1 mm)**. A 2 mm self-tapping screw could split
  it toward the pocket. Drive the screws gently, or open the pilots to 1.6 mm.
  If the tab holes measure farther out than 2.3 mm from the tab ends, this
  improves.
- **MJF tolerance ±0.3 mm** on the Ø4.1 bore, the horn pockets and the hex
  pocket. Ream the bore. A tight horn pocket can be scraped, and a loose one is
  shimmed by the horn screws.
- **Height**. The J2 plug and ribbon keep-out (16 mm behind the board) swing
  23 mm below the tilt axis at ±45°, so the plate sits 24.5 mm under the axis.
  Firmware limits tilt to ±30°: with that, `HEAD_DROP` could drop to about 20 mm.
- **Ribbon fatigue at pan ±60°**. The ribbon twists around the base below the
  yoke. Watch for chafe on the base's rear corner, and lengthen the loop if
  needed.
- **The upper SG90 tab is glued, not screwed**. The lead's root has to pass a
  notch exactly where the upper tab's screw would go.
- **The pivot screw can loosen** if the sleeve slips in the bore. Use a
  nylon-insert nut if it does. There is room: the pocket is 3 mm deep against
  the nut's 2.4.
- The **eyebrows** add about 22 g, mostly high on the tilt stage. That is
  ~60 g·cm against the MG90D's ~2100 g·cm, so it is fine. They do make Teuvo
  about 37 mm taller.

## Fixture notes (common/ not edited)

- `servo()` models the lead as a rigid 6 mm "boot" beyond the end face. That is
  why the base is open on one side and the brow windows have notches. A real
  lead bends, but its root does protrude, so the design treats it as real.
- `head()`'s J2 keep-out (16 x 18 x 8 mm) is the dominant constraint on height.
  If the IDC plug is replaced by a low-profile right-angle plug, measure it and
  the tilt axis can come down.
- The carrier README says head 1's M2 holes go "through the cradle into
  heat-set inserts". This cradle uses Ø1.7 self-tapping pilots in PA12 instead.
  M2 x 6 ISO 7380, as the courtyard requires.

## Regenerate

```sh
cd packages/robocar/unified/hardware/pan-tilt/entries/workhorse
/tmp/claude-0/b123/.venv/bin/python design.py           # output/*.step, *.stl
cd ../.. && /tmp/claude-0/b123/.venv/bin/python -m common.check entries/workhorse
/tmp/claude-0/b123/.venv/bin/python -m common.render entries/workhorse
```
