# Majakka: a pan-tilt where neither servo moves

Tournament entry `wildcard`. Head 1 (29.5 mm board, XIAO ESP32-S3 Sense) on two
TowerPro MG90D servos. Both servos are screwed into a fixed base and never move.
Only the head turns, like the lamp in a lighthouse (*majakka*). Eight MJF PA12
parts from JLC3DP, two 6801ZZ bearings, three steel dowels, one E-clip, and
small screws.

![rest](output/render-rest.png) ![front](output/render-rest-front.png)
![corner, pan 60 tilt 30](output/render-corner.png) ![back](output/render-rest-back.png)

Gates (`output/check.json`): **zero clashes** over pan -90..90 and tilt -45..45.
Two excuses, both the stock horn on its own servo's spline (`pan_horn /
pan_servo`, `tilt_horn / tilt_servo`). The gear mesh needs no excuse: the
involute teeth are modelled with 0.15 mm backlash and do not intersect at any
swept pose. Every part is a valid solid, and the thinnest wall is 1.0 mm (see
[Walls](#walls)).

## Concept

In every kit-style pan-tilt, and in the other entries, the tilt servo rides on
the pan stage. Its 13 g swings with every pan move, and its 25 cm lead flexes
through the pan joint all day. Here the tilt goes **up the pan axis** instead:

```
            head 1 on the cradle, pivots on 2 x M3 in the turret cheeks
                 \__ crank (9 mm) __ link (10 mm) __ bracket
                                                       |
   turret (pans) ===[gear]=== hub in 2 x 6801ZZ  [ square plunger slides
        ^                                          |  up and down inside
   pinion on the pan MG90D (1:1)                   |  the hollow hub ]
                                                   |
                                       grooved collar on the plunger
                                       held by 2 dowels on a fork
                                       on the tilt MG90D's horn
   ----------------------- fixed base ------------------------------------
```

- **Pan.** The pan MG90D stands upright off-axis. A 20-tooth module 1.25
  pinion on its disc horn drives an identical gear on the turret, so the turret
  turns 1:1 (the opposite way: flip the pan channel's sign). The turret's hub
  runs in two 6801ZZ bearings pressed back to back against a lip in the base
  and is held down by an E-clip. The bearings carry the head's weight and its
  tipping moment. The pan servo's spline sees only the tooth force.
- **Tilt.** The hub has a 4.4 mm square bore. A 4 mm square plunger slides in
  it and turns with the turret. At the bottom, below the hub, the plunger has a
  collar with a groove. A fork screwed to the tilt MG90D's horn holds two 3 mm
  steel dowels in that groove, from both sides, like a gearbox shift fork. When
  the turret pans, the collar turns under the dowels and nothing else happens,
  so **pan and tilt are fully decoupled**. When the tilt servo turns, the fork
  lifts or lowers the plunger. At the top, a bracket on the plunger pushes a
  10 mm link up to a 9 mm crank behind the head, and the head nods.
- **The head is carried on both sides.** The cradle swings on two M3 screws in
  the turret's cheeks. No servo shaft holds the head, and the fork's far side
  turns on a 4 mm dowel in the base, so the tilt servo's shaft has no
  cantilever either.

What this buys:

| | Majakka | Tilt servo on the pan stage |
|---|---|---|
| Servo leads crossing the pan joint | none (only the head's ribbon) | the tilt lead, flexed on every pan |
| Mass that pans | head + cradle + turret (~23 g) | the same plus a 13 g servo |
| What holds the head's weight | ball bearings (pan), two pivots (tilt) | the servo shafts |
| Pan travel limit | the ribbon's slack | the tilt lead's slack and the ribbon |

What it costs: a bigger base (66 x 61 mm and 35 mm tall, 84 mm long with its ears, with both servos inside
it), more parts, and a tilt that is not 1:1 (below). The base is the price of
keeping both servos still.

**Tilt mapping.** The crank and link make the tilt a little non-linear. The
servo turns about 0.82 degrees per degree of head tilt near centre and less
towards look-up. `design.py` prints the table (`tilt_to_servo()`); for the
firmware it is a fixed lookup between head angle and servo angle:

| head tilt (deg, + = look up) | -45 | -30 | -15 | 0 | +15 | +30 | +45 |
|---|---|---|---|---|---|---|---|
| tilt servo (deg) | +37.6 | +24.6 | +12.3 | 0 | -12.2 | -23.7 | -33.1 |

The firmware limits (pan ±60, tilt ±30) sit well inside the ±90 / ±45 this
clears. `head_aim` works in head angles, so the table belongs in the servo
write path (`servo_controller.c`), one place, and `servo tilt <deg>` keeps
meaning a head angle.

**Geometry.** World frame of `common/fixtures.py`, z 0 on the chassis plate,
the pan axis at x = y = 0. The tilt axis is at x 11.5, z 61.8, in the plane of
the board's back face and 8 mm above its bottom edge. **The lens is 70.6 mm
above the chassis plate** and 39 mm ahead of the pan axis. The head is 6.5 mm
to the right of the pan axis (-Y): that keeps its J2 ribbon keep-out clear of
the plunger, which is on the pan axis. Everything is computed in `_layout()`
from the part stack and the J2 keep-out's lowest point over ±46° tilt.

## Parts (JLC3DP)

All parts are **MJF PA12-HP (nylon)**. Why MJF for all of them:

- The base takes two **press-fit ball bearings** and **self-tapping servo
  screws**, and the cradle takes **heat-set inserts**. PA12 does all three.
  Resin cracks under a press fit and does not hold a heat-set insert (the
  `enclosure-cad` skill's trap).
- The plunger, the fork's groove and the cradle pivots are **sliding
  surfaces**. PA12 on steel and PA12 on PA12 wear well with a dry PTFE lube.
- The gears: MJF PA12 is the usual material for printed small spur gears, and
  the module 1.25 teeth have a 1.35 mm tip land (tips cut 0.15 m short for it).
- MJF needs no supports, so print orientation is free. JLC3DP recommends 1.0 mm
  walls for MJF nylon ([design guideline](https://jlc3dp.com/help/article/3D-Printing-Design-Guideline)).

Colour: MJF PA12 comes grey; order the base and turret **dyed black** for
Teuvo's 1950s cabinet look, and the cradle grey. Prices are estimates from
JLC3DP's MJF-PA12 rate of $0.275/g ([Aug 2025 price cut](https://jlc3dp.com/blog/drastic-material-price-cuts-and-adjusted-post-processing-service-fees);
PA12 ~1.01 g/cm³), with an assumed ~$1 floor per part. Get a real quote from
the STLs before ordering.

| Part | Volume | Thinnest | Est. price | Print orientation | Notes |
|---|---|---|---|---|---|
| `base` | 35.0 cm³ | 1.05 mm | ~$9.70 | floor down | Box with bearing housing, lintels and bulkhead for the servos, fork pillar, 4 x M3 ears |
| `turret` | 7.9 cm³ | 1.32 mm | ~$2.20 | gear down (teeth in XY) | Hub, gear, plate, two cheeks with M3 journals |
| `pinion` | 2.2 cm³ | 1.35 mm | ~$1.00 | flat | Screws to the pan servo's disc horn |
| `cradle` | 3.2 cm³ | 1.00 mm | ~$1.00 | back face down | Holds head 1 by its M2 holes, carries the crank |
| `fork` | 1.2 cm³ | 1.20 mm | ~$1.00 | plates vertical | Two plates and a crossbar; the dowels press in |
| `plunger` | 1.2 cm³ | 1.20 mm | ~$1.00 | lying flat | 4 mm square rod with the grooved collar |
| `bracket` | 0.3 cm³ | 1.00 mm | ~$1.00 | tab flat | Socket on the plunger top, tab for the link pin |
| `link` | 0.15 cm³ | 1.35 mm | ~$1.00 | flat | Two Ø2.3 holes 10 mm apart |
| **total** | 51 cm³ | | **~$18** + shipping | | |

`printed_parts()` returns every part in its **world pose at pan 0, tilt 0**,
and `python design.py` writes `output/<part>.step` and `.stl` in that pose.
Rotate to the orientation above in JLC3DP's viewer if you care; MJF does not
need it.

## Hardware

| Qty | Item | Where |
|---|---|---|
| 2 | TowerPro MG90D, with its horn bag and screws | pan (upright) and tilt (on its side) |
| 1 | MG90D **disc horn** (Ø21) and its horn screw | pan servo, under the pinion |
| 1 | MG90D **double horn**, both arms cut to **9 mm** from the centre, and its horn screw | tilt servo, under the fork |
| 2 | **6801ZZ** ball bearing, 12 x 21 x 5 | base housing, pressed in from each side against the lip |
| 1 | **DIN 6799 E-clip, size 9** (groove Ø9, for a 12 mm shaft) | hub groove under the lower bearing |
| 2 | Steel dowel **Ø3 x 6**, ISO 2338 m6 | fork pins in the collar groove (press fit, 2.9 mm holes) |
| 1 | Steel dowel **Ø4 x 12** | fork's far pivot, pressed into the base pillar from outside |
| 2 | M3 heat-set insert, 4 mm long (Ø4.0 bore) | cradle pivot bosses |
| 2 | M3 x 8 button head (ISO 7380) + low-strength threadlocker | pivots: through the cheek journals (Ø3.3) into the inserts |
| 2 | M2 heat-set insert (Ø3.2 bore) | cradle back plate, behind head 1's H5/H6 holes |
| 2 | M2 x 6 button head (the head's existing screws) | head 1 to the cradle |
| 2 | M2 x 6 pan head, self-tapping into Ø1.6 pilots | crank pin and bracket pin (the link turns on the shank) |
| 1 | M2 x 8 pan head, self-tapping | bracket down into the plunger's end |
| 3 | M2 x 5 self-tapping | pinion to the disc horn (drill the horn's holes 1.5 mm at r 7, 120° apart) |
| 2 | M2 x 5 self-tapping | fork plate to the cut double horn (holes at r 6) |
| 4 | MG90D tab screws (in the servo bag, ~2 x 8 self-tapping) | 2 per servo into Ø1.6 pilots |
| 4 | M3 x 12 + nut + washer | base ears to the acrylic chassis plate |
| — | Dry PTFE lube | plunger, collar groove, cheek journals |

**Chassis.** The four M3 holes are at x -51.5 and +23.5, y -22.0 and +29.3
(a **75.0 x 51.3 mm** rectangle; the pan axis is 23.5 mm in from the front
pair and 22 mm from the right pair). Drill **3.4 mm** holes in the acrylic
plate (a step drill or a sharp brad-point at low speed; back it with wood) and
bolt from above with nuts underneath. Nothing passes through the plate, but the
fork's lowest point is 1.3 mm above it: keep the area under the base's floor
pit (x ±14.5 mm, y -24.5..10 around the pan axis) clear of wiring and nut
heads.

## Assembly

1. **Bearings.** Press one 6801ZZ into the housing from below and one from
   above, each against the lip. MJF holes run ±0.3 mm: if one is loose, a drop
   of retaining compound; if tight, warm the housing with a hot-air gun.
2. **Tilt servo.** Lower it into the bulkhead's open U from above (the deck is
   open over it), shaft pointing forward (+X), tabs against the bulkhead's
   front face. Two tab screws from the front, through the service window in the
   front wall and the Ø5 driver hole beside the pillar. Do this before the fork
   goes in: the fork sits in front of the screws.
3. **Pan servo.** Drop it into the opening behind the bearing housing, spline
   up, tabs on the two lintels. Two tab screws from above. Route both leads out
   of the slot in the rear wall.
4. **Centre both servos** (1500 µs, from a servo tester or `servo pan 0` /
   `servo tilt 0` with the horns off).
5. **Pinion.** Screw the disc horn to the pinion's underside (three M2 x 5),
   press it on the pan spline, and drive the horn screw through the pinion's
   Ø5.2 centre hole.
6. **Turret.** Hold the turret facing forward and lower the hub through both
   bearings; the gear meshes with the pinion. From below, through the floor pit,
   snap the E-clip into the hub's groove. Check the turret turns freely by
   hand.
7. **Plunger.** From below, through the floor pit, push the plunger up into
   the hub's square bore until the collar stops under the hub. Screw the
   bracket onto its top end (M2 x 8, from above; the bracket's tab points to
   the left, +Y).
8. **Fork.** Cut the double horn's arms to 9 mm. Screw it to the fork's horn
   plate (two M2 x 5). Put the fork around the collar and press both Ø3 dowels
   in from outside the plates, into the groove (the cut horn arm clears the
   near dowel by 0.5 mm). Press the horn onto the tilt spline with the fork
   level and drive the horn screw through the far plate's Ø4.3 journal. Then
   press the Ø4 x 12 dowel through the front wall's pillar into that journal.
9. **Cradle and head.** Melt the two M2 inserts and the two M3 inserts into
   the cradle. Screw head 1 on with its own M2 button heads. Put the link on
   the bracket tab (M2 x 6 from the head's right side, -Y). Hold the cradle
   between the cheeks and fit the two M3 x 8 pivots through the cheeks into the
   inserts, threadlocker on the threads, backed off to 0.1 mm end play. Join the
   link to the crank (M2 x 6 from the left, +Y, past the cheek).
10. **Ribbon.** Plug the 14-way IDC onto J2 last. Let it drop behind the
    turret plate, leave a loop of about 30 mm, and tie it to the base's rear
    wall with a cable tie through the lead slot. The loop is the pan travel:
    check ±90 by hand before powering up.

## Measure first (calipers on the parts in hand)

The shared fixtures carry estimates. Each of these is a parameter at the top of
`design.py` or in `fixtures.MG90D`; re-run `python design.py` and the gates
after changing one.

1. **MG90D tab span E (31.5) and tab hole spacing.** The lintel pilots and the
   bulkhead pilots are placed from `servo_tab_holes()`. The pan servo's pilot
   is 1.05 mm from the lintel's inner face (the thinnest wall in the base), so
   a hole that is really further out splits the lintel.
2. **Below-tab depth F (19.8)** and **case height C (28.5).** F sets the
   lintel height. C sets the pinion's height, and so the gear, the turret and
   the head. The pan servo's bottom is 2.9 mm above the chassis plate.
3. **Shaft offset from the case end (6.0, estimate).** It places both servo
   bodies around fixed shafts. Wrong by 1 mm moves the pan servo into its
   lintel or the tilt servo's body into the bulkhead window's edge.
4. **Horns.** The disc horn's hole radius (the pinion's three screws are at
   r 7), the double horn's holes (the fork's two screws are at r 6), and the
   horn screw's head diameter (must pass Ø4.3 in the far fork plate).
5. **6801ZZ inner-ring OD.** The turret's thrust boss is r 7.0 and must bear
   on the inner ring only, not the shield.
6. **Fit checks after printing.** Plunger in the hub's square bore (designed
   0.2 mm per side), the Ø3 dowels in their 2.9 mm holes, the bearings in
   Ø21.0. MJF is ±0.3 mm; sand or ream rather than reprint.

## Risks

- **Base size.** 66 x 61 mm on the chassis, 35 mm tall, plus a 9 mm ear at each end (84 mm overall). It
  has to fit the acrylic plate beside the main board. This is the cost of
  keeping both servos still.
- **Tilt slop.** The play stacks: the dowels in the 3.4 mm groove (0.2 mm per
  side), two link pins (Ø2.0 screw in Ø2.3), the plunger in its bore. My
  estimate is ±2–3° of head tilt. Reaming the link holes to 2.1 mm and adding
  an O-ring under the collar (a light preload) are the fixes. The planner
  re-aims every plan, so a few degrees is tolerable, but it is not the
  zero-play of a horn on a spline.
- **Gear backlash.** 0.15 mm at the pitch circle, about 0.7° of pan, plus MJF
  tolerance. Same mitigation: the executor re-aims.
- **Non-linear tilt.** The servo angle is not the head angle; without the
  table the head tilts about 20 % further than commanded near centre.
- **Thin features.** See below. The pan servo's tab pilots sit 1.05 mm from the
  lintel face because the MG90D's tab hole is that close to its case.
- **Assembly through the floor.** The E-clip and the plunger go in from below,
  so the base must come off the chassis to service them.
- **Ribbon.** It is the only cable crossing the pan joint, and it is not
  modelled beyond the 16 mm keep-out. Its loop decides the real pan travel.
- **No eyebrows.** Brows would have to ride the turret, and their two leads
  would cross the pan joint, which is the one thing this design removes. If
  Teuvo needs brows, mount two SG90s on the turret's cheeks (above the
  pivots, horns facing forward) and accept those leads; or take the
  `teuvo-face` brow bar, which carries them on a fixed forehead.

## Walls

The repo's MIN_WALL 2.0 mm is for resin cases. This is an MJF nylon mechanism,
and JLC3DP's MJF guideline recommends 1.0 mm walls. Every part meets 1.0 mm:

| Part | Thinnest | Where |
|---|---|---|
| base | 1.05 | pan servo tab pilot to the lintel's inner face |
| turret | 1.32 | gear web beside the thrust boss, at the gear's underside |
| cradle | 1.00 | pivot boss lip behind the side arm |
| bracket | 1.00 | the link-pin pilot through the tab |
| fork, plunger | 1.20 | around the M2 pilots |
| pinion, link | 1.35 | gear tooth tip land; link end around its hole |

## Fixture notes

- `fixtures.horn()` models the hub and arms but not the **horn screw head**,
  which stands about 1–2 mm proud of the hub. Here it sits in a Ø4.6 clearance
  hole in the fork plate and the Ø5.2 hole in the pinion, so it does not
  matter, but an entry that puts a plate flat on a horn hub would clash in
  reality and pass the gate.
- The fixtures do not include the **chassis plate**. The check cannot see that
  the fork's ends come within 1.3 mm of it; that clearance is from `_layout()`.
- `tab_hole_inset` (2.3, estimate) decides whether a tab pilot fits in a
  printed lintel at all; it is worth a caliper before any entry is ordered.
