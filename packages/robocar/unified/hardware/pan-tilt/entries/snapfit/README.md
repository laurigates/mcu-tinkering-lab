# Snap & Flex (`snapfit`)

A pan-tilt for robocar-unified head 1 on two TowerPro MG90D servos, with an
optional pair of SG90 eyebrows for Teuvo. Every joint is a snap, a slide-in
groove or a press fit in MJF nylon. Two M3 screws hold it to the chassis
plate, and nothing else needs a tool.

| | Generic black 9 g kit (as fitted today) | Snap & Flex |
|---|---|---|
| Screws to assemble | about 12 (estimate, by kit type): 2 head M2, 2-4 servo tab self-tappers, 4 horn-to-bracket self-tappers, 2 horn centre screws, 2 base screws | **2** (M3 to the chassis) |
| Optional screws | none | 2 horn centre screws per servo pair (the servo's own), which the geometry does not need |
| Takes an MG90D | no (the 19.8 mm below the tabs bottoms out in the pan cup) | yes, sized from the fixture figures |
| Opposite-side tilt pivot | no | yes, a Ø6 stub in the yoke arm |
| Pan bearing | servo shaft only | a nylon ring and four snap fingers carry thrust and tipping load |

Gates: `output/check.json` reports **zero clashes** over pan ±90 × tilt ±45
with the eyebrows fitted. The only excused pairs are the two splines inside
their horn hubs (`pan-servo / pan-horn`, `tilt-servo / tilt-horn`). The
thinnest wall on any part is 1.50 mm (renders: `output/render-*.png`).

## Concept

Five printed parts, four of them structural:

1. **base** sits on the chassis on two M3 screws. It is a cup: a floor, a ring
   (Ø46.1 bore, Ø52.1 lip) and a pocket whose wall tops carry the pan servo's tabs.
   The pan servo **drops in from the top** and is not fastened. Its lead
   feeds out through a slot at the back first.
2. **yoke** carries the cross horn in a pocket on its underside and **presses
   down onto the base**. Four flat cantilever fingers (1.8 thick, 12.5 long, 0.9
   mm hook) click under the ring's lip. That snap is the pan pivot: the ring's
   top takes thrust and tipping loads, the fingers locate it radially and stop
   it lifting, and so the horn cannot leave the spline and the pan servo cannot
   leave its pocket. No servo screw and no horn screw are needed.
   The tilt servo **slides in from the front** along two tab grooves (the tab
   ends ride in slots in two blocks) and stops against a back rail. A flexure
   finger cut out of the lower groove's ledge clicks in front of its tab. To
   release it, press the finger outward with a fingernail and slide the servo
   forward.
3. **cradle** holds the head board in two edge grooves. The board **drops in
   from the top** and two latch tongues behind it spring printed Ø1.9 pegs into
   its M2 holes. The tilt servo's single-arm horn (pointing down) **presses
   into a pocket** on the -Y cheek. The +Y cheek carries a **Ø6 pivot stub** that
   the yoke's +Y arm springs over. That stub is the opposite-side pivot, and it
   keeps the cradle within 0.4 mm axially, so the horn stays on its spline.
4. **brow-left / brow-right** (optional) are bushy bars that press onto SG90
   single-arm horns. The cradle grows a deck behind the board's top edge, with
   two more slide-in grooves and flexure fingers for the SG90s. They rest a
   little stern (outer ends 10° up) and the firmware animates them on PCA9685
   channels 14 and 15.

The same `slide_cradle()` (two tab grooves, a back rail, and a flexure finger
with a catch nose and lead-in ramp) holds all three slide-in servos. The SG90
pair share one back rail on the centre line.

Geometry decisions:

- **The tilt axis is where the head's sweep is smallest**: 5.8 mm in front of
  the board's back face (midway between the XIAO stack's front and the J2
  keep-out's back), at the board's mid-height. The J2 plug and ribbon keep-out
  is the deepest thing below the axis, so the tilt axis height follows from it:
  `Z_TILT = yoke top + J2 drop over ±45° + 0.6`. That gives a tilt axis at z
  64.9, the lens at about z 67 and x +21.8 (ahead of the pan axis), and the
  top of the brow pod at z 118.7 (about 83 mm without brows).
- **Parallax**: the lens sits about 22 mm ahead of the tilt axis, so ±30° of
  tilt moves it about ±11 mm. Taking that out would put the axis inside the
  XIAO stack's sweep and lift everything further. The `gimbal`-style nodal
  layout costs height, and this entry spent its effort on assembly instead.
- **Balance**: the XIAO (about 7 g) sits about 9 mm ahead of the tilt axis, and
  the two SG90s (18 g) sit about 10 mm behind and above it. With brows fitted,
  the head is close to balanced at rest.
- **No curved snap beams.** The ring is round, but the fingers are flat
  plates. A cut through a curved beam leaves knife edges, which the thickness
  gate found at 0.01 mm on the first try.

## Parts

All prices are estimates from JLC3DP's published MJF PA12 rate of $0.275/g
(the August 2025 price cut) at about 1.01 g/cm³. Each part has a $1.00 minimum.
PA11 is assumed to cost about 20% more, which is not verified. Upload the STLs
for the real quote.

| Part | Process / material | Colour | Volume | Est. price | Why |
|---|---|---|---|---|---|
| base | MJF **PA12-HP** | grey, or dyed black | 20.8 cm³ | ~$5.80 | Nothing on it flexes. The lip and ring need stiffness and wear resistance against the fingers, and PA12 is the cheapest nylon |
| yoke | MJF **PA11-HP** | grey (natural) | 13.2 cm³ | ~$4.40 | Four pan fingers (1.6 % strain on assembly), the tilt servo's flexure finger (~2 %), and the +Y arm springing out 4.5 mm over the stub (~2.8 %). PA11's 50 % elongation at break (JLC's figure, against 20 % for PA12) is the margin for repeated assembly. The pivot hole runs nylon on nylon |
| cradle | MJF **PA11-HP** | grey | 7.9 cm³ | ~$2.60 | Latch tongues (~1.7 %), two SG90 flexure fingers, and the pivot stub as a bearing |
| brow-left, brow-right | MJF PA12-HP | **dyed black** | 0.8 cm³ each | $1.00 each (minimum) | Press fit on the horn, nothing flexes. Black reads as eyebrows |

Total is about $15 plus shipping. SLA resin is not used anywhere: resin
snap-fits crack, and resin does not hold a press-fit horn.

**Print orientation** (MJF needs no supports, so this is about finish and
layer direction). Each part is exported in its world pose, which is also its
print pose: base floor-down, yoke plate-down with the fingers hanging, cradle
upright, brows face-up. MJF is nearly isotropic, but Z is a little weaker. The
yoke's fingers and arm then bend with their layers across the beam, which is
the weaker way. PA11 covers that. If JLC prints with the part on its side, ask
for the plate to be vertical, so that the fingers bend along their layers.

**JLC process minimums used.** JLC3DP's design guide gives an MJF minimum wall
of 1.0 mm (5 mm parts) to 1.5 mm (50 mm parts), and asks for more than 1.5 mm
for snaps and protrusions. The thickness gate's thinnest walls are base 1.79,
yoke 1.51 (the finger nose tip), cradle 1.50 (the board groove floor) and
brows 1.60. Clearances follow the same guide: 0.3 mm between separate parts,
0.8 mm slits to free a flexure inside one print (above the 0.6 mm that MJF
fuses), and 0.15 mm press fit on horns, against a stated hole tolerance of
±0.3 mm.

## Hardware BOM

| Qty | Item | Where |
|---|---|---|
| 2 | TowerPro MG90D (13 g, metal gear) with its bag of horns | pan, tilt |
| 1 | MG90D **cross horn** (4 arms) | pan, in the yoke's underside pocket |
| 1 | MG90D **single-arm horn** | tilt, in the cradle's -Y cheek pocket, arm pointing down |
| 2 | M3 × 8 button or pan head + M3 nut (or nyloc) | base ears to the acrylic chassis |
| 0-2 | Servo horn centre screws (supplied with the servos) | optional. Pan: through the yoke's Ø4.5 hole, before the cradle goes on. Tilt: before the cradle goes on |
| 2 | SG90 + single-arm horns (optional eyebrows) | brow pod |
| 0-2 | SG90 horn screws (optional) | under the brows' Ø4.5 recesses |

No heat-set inserts, bearings, servo tab screws or M2 head screws are needed.

**Chassis drilling.** Drill two **Ø3.2 mm** holes **62 mm apart** across the
car (±31 mm in Y), on a line **6 mm behind** the pan axis. The pan axis is
the base ring's centre. Drill a third hole of about Ø10 mm under the base's
rear lead slot if the leads go down through the plate.

## Assembly

1. **Base to chassis**: two M3 screws, from the top (the yoke is not on yet,
   so the screwdriver reaches).
2. **Pan servo**: centre it (1500 µs). Feed its connector out through the
   base's rear slot, then drop the servo in, spline up and the case's long
   end to the rear, until its tabs sit on the pocket walls.
3. **Tilt servo into the yoke**: centre it. Slide it in from the front, spline
   toward +Y (the robot's left), with its tabs in the two grooves, until the
   finger clicks. Press its lead into the lead clip on the yoke's -Y edge.
4. **Pan horn**: press the cross horn into the yoke's underside pocket, arms
   along X and Y.
5. **Yoke onto base**: line the horn's spline up with the yoke facing forward,
   and press straight down. The horn seats on the spline and the four fingers
   click under the lip. Optionally fit the pan horn screw through the Ø4.5
   hole in the yoke's top.
6. **Tilt horn**: push the single-arm horn onto the tilt spline, arm pointing
   straight down. Optionally fit its screw now; this is the only time a
   screwdriver reaches it.
7. **Cradle**: pull the yoke's +Y arm outward about 5 mm with a thumb, lower
   the cradle between the arms, and press its -Y cheek's pocket onto the horn
   arm. Release the arm so that it springs over the Ø6 stub.
8. **Head**: plug the IDC ribbon into J2. Feed the ribbon through the hook in
   the yoke's rear tongue. Drop the head board into the cradle's grooves from
   the top, XIAO facing forward, until both pegs click into its M2 holes.
9. **Eyebrows** (optional): slide each SG90 in sideways from its own side,
   shaft forward, until it clicks. Fit the single-arm horns at about 10° above
   horizontal, then press the brows on. With the brows fitted, the head board
   comes out only after the SG90s.

**Cable routing.** The pan lead leaves through the base's rear slot and does
not move. The tilt lead runs from the servo's lower end to the clip on the
yoke's -Y edge, then hangs down outside the fingers. The ribbon runs from J2
through the rear-tongue hook, where pan twists it along its length instead of
dragging it. Leave a service loop of about 40 mm below the hook for pan ±60°,
and do not route the ribbon inside the base ring. Run the SG90 leads down the
back of the cradle beside the ribbon and tie them to it, with the loop below
the hook. That needs servo extension leads: 25 cm is short from the top of the
head to the PCA9685.

## Measure before ordering

Every number below is a parameter at the top of `design.py` or comes from
`common/fixtures.py` (`SERVO`, `BROW`). Change it there and re-run the gates.

| Measure | Figure used | Why it matters |
|---|---|---|
| MG90D **tab thickness** | 2.5 (estimate) | Each groove is the tab thickness + 0.4. A tab 0.3 mm thicker does not slide in, and a thinner one rattles |
| MG90D **tab span E** | 31.5 (TowerPro) | Groove ends, the end walls and the finger position |
| MG90D **below-tab depth F** | 19.8 (TowerPro) | The pan servo must hang on its tabs, 0.5 mm off the floor. If F is longer, the servo bottoms on the floor and the yoke no longer seats on the ring |
| MG90D **shaft offset** from the case end | 6.0 (estimate) | Where the pan and tilt axes fall in the case. It moves the pocket and the grooves |
| MG90D **case width** | 12.0 | Groove depth, and where the back rail and the finger nose sit |
| Lead **boot**: protrusion past the end face, height | 6 mm, centre ~7 mm above the base (estimate) | The base ring's bore (Ø46.1) is sized to let the boot drop past |
| **Horns**: hub Ø, hub height, arm width and thickness, length, spline gap | hub Ø7 × 4, arm 1.6 thick, cross 5.0 wide × 16, single 5.5 wide × 20 (fixture estimates) | The horn pockets are press fits at +0.15 mm. Horns vary between batches, so measure the ones in the bag |
| Spline height above the case | 4.0 (A - C) | The horn hub rides on it; it sets the yoke's height above the ring |
| Head board M2 holes | 22.5 apart, 3.05 below the top | The latch pegs |
| Board front face within 0.75 mm of the side edges | assumed clear | The groove's front wall overlaps the board by 0.75 mm. Check that no part sits there |
| SG90: the same set | fixture estimates | The brow grooves |

## Risks

- **Tab thickness and horn shape are estimates.** The grooves and horn
  pockets are the parts that can be wrong after printing. Each is a single
  parameter: sand the groove or the pocket, or reprint the one part.
- **Arm spring-out is the highest strain** (about 2.8 % at assembly, a few
  times only). It is fine in PA11, but in PA12 it is near yield. Do not order
  the yoke in resin.
- **The fingers are a nylon-on-nylon pan bearing.** Expect some friction and
  wear on the ring's lip. The 0.3 mm thrust gap means the horn carries the
  yoke until it tips. If the pan feels draggy, sand the ring's top.
- **It is tall.** The tilt axis is at z 65, set by the J2 plug and ribbon
  keep-out swinging under the head. The brow pod adds 35 mm above the board.
- **The brows mean extra leads and a top-heavy cantilever deck**, and the head
  board comes out only after the SG90s.
- **Parallax**: the lens moves about ±11 mm over ±30° of tilt.
- **Board edge clearance**: the grooves assume a clear 0.75 mm band at the
  front edges, and the -Y cheek is notched for the IDC plug.

## Fixture notes (for the tournament)

- `head(screws=False)` models the board as a solid, with no M2 holes, so the
  latch pegs overlap the board solid. They are in the same group, so the check
  does not see it, but the fixture cannot show that the pegs fit.
- The J2 keep-out (y -15.5 to 2.5) pokes 0.75 mm past the board's -Y edge
  (14.75). That is conservative for the ribbon. This entry notches the -Y
  cheek for it.
- The ray thickness gate reports a near-zero wall wherever a flat cut meets a
  curved surface at an acute angle: a slot through a ring, a sector cut from a
  cylinder, or a split pin. This entry dropped a split, barbed push-pin pivot
  for the integrated stub, made the fingers flat, and cut the lead slot
  through a flat-faced boss for that reason.

## Files

- `design.py`: parametric build123d. `printed_parts()` gives the parts in
  their world pose at pan 0 and tilt 0, which is also their print pose.
  `assembly(pan, tilt)` gives the bodies in groups `base`, `pan` and `tilt`.
  Running it writes `output/<part>.step` and `.stl`.
- `output/check.json`, `output/render-*.png`: the gates.

Sources: [JLC3DP 3D printing design guideline](https://jlc3dp.com/help/article/3d-printing-design-guideline),
[JLC3DP PA11-HP](https://jlc3dp.com/help/article/pa11-hp-nylon),
[JLC3DP PA12-HP](https://jlc3dp.com/help/article/pa12-hp-nylon),
[JLC3DP pricing update](https://jlc3dp.com/blog/drastic-material-price-cuts-and-adjusted-post-processing-service-fees).
