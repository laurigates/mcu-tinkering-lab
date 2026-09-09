# A Breakout's Physical Pin Order Comes From the Vendor's Board File

Any document that tells somebody where to solder makes a claim about *physical*
position, and that is the claim least likely to survive being recalled. The
pin *names* are easy — they are in the datasheet and in this repo's
`docs/reference/datasheets/`. The *order along the header* is not in either,
and it is what a person with an iron actually needs.

Three sources look authoritative and are not:

| Source | Why it fails |
|---|---|
| Memory | The common variants genuinely differ, and a confident wrong order reads exactly like a right one |
| A product photo | Labels are legible, orientation is not — and silkscreen is printed on **both faces** on most breakouts, so a mirrored read looks correct |
| This repo's `docs/schematics/components.py` | Its `IcPin` order is chosen to keep the *schematic* readable (the docstrings say so: "ordered to align with… avoids bus crossings"). It is a drawing decision, not a board fact |

**Vendors publish the board file. Read that.** SparkFun and Adafruit both ship
Eagle `.brd`/`.sch` in the product's GitHub repo, and Eagle 6+ files are XML:
pad coordinates, element rotations, net names and silkscreen text are all in
there, exactly as manufactured.

```sh
gh api "repos/sparkfun/Motor_Driver-Dual_TB6612FNG/contents/Hardware/SparkFun_Motor_Driver-TB6612FNG_v11.brd" --jq '.content' | base64 -d > /tmp/tb6612.brd
python3 tools/breakout-pinout.py /tmp/tb6612.brd --element JP1 --element JP2
```

`tools/breakout-pinout.py` does the reading. It resolves each pad to absolute
board coordinates, sorts top-to-bottom as a top view, and names each pad by the
vendor's own net.

## Three traps the format sets, all handled in the script

- **Pad coordinates are element-relative** and the element carries a rotation.
  `MR0`/`MR270` means *mirrored onto the bottom layer*, and x is negated
  **before** the rotation. Skip that and a bottom-mounted connector comes out
  reversed — a plausible-looking pinout that is backwards.
- **Silkscreen exists on two layers.** 21 is the top face, 22 the bottom. Their
  stored x is a real board coordinate so the two agree, but a reader holding the
  board flipped sees the bottom mirrored. Quote the face with any label.
- **Nearest-label is a hint, not an answer.** Where a net is generic (`N$13`)
  the script falls back to the closest silkscreen text on the pad's own face.
  Restricting to the face was not optional: unfiltered, the PCA9685's channel-11
  pad matched the bottom-face note `40-1000Hz` — the frequency range printed as
  a channel number, and it looked fine.

## Where this is already load-bearing

Every physical claim in `packages/robocar/unified/docs/wiring-card-motors.typ`
came out of these two files, including the ones that decide whether the card is
usable at all: the PCA9685's row order (PWM nearest the chip, then V+, then GND
at the edge), its two identical 6-pin side headers (GND/OE/SCL/SDA/VCC/V+), the
screw terminal's V+/GND handedness, and the TB6612FNG's two 8-pin rows both
starting at the same end of the board.

Issue #495 (*schematics: draw modules with physical pin layout*) wants the same
data for the schematic renderer, which is the second consumer and the reason
this is a script rather than a note.

## When it does not apply

- An **unbranded clone with no published files**. Most clone the reference
  layout exactly, but "most" is not evidence — ask for the silkscreen order or
  a photo of the specific board, and say in the document which one it is.
- **KiCad** projects: `.kicad_pcb` is s-expressions, not XML. The script does
  not read them.
- The MCU board's own header order, which belongs in
  `docs/reference/boards/<board>.md` under ADR-021's split — board facts, not
  per-project parts.

## Related

- `~/.claude/rules/never-fabricate-test-identifiers.md` — extract the shipped
  artifact, never retype it; a transcribed pinout is that retyping
- `camera-sensor-identity.md` — the runtime sibling: read the sensor's PID
  before reasoning about a register, because a header comment is not evidence
- ADR-021 (`docs/decisions/ADR-021-hardware-source-of-truth.md`) — the
  board × header × parts join this feeds
