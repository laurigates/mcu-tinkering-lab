# ThinkPack — Getting Started

ThinkPack is a set of small, self-contained sensor-and-sound boxes designed
for open-ended play. Each box is the size of a deck of cards, runs on a
rechargeable LiPo battery, and talks to the other boxes wirelessly over
ESP-NOW. There is no phone app, no cloud account, and no setup: plug a box
in, switch it on, and it starts doing its one thing. Bring a second box into
the room and the two of them start cooperating.

## The boxes

| Name       | What it does                                                    | LEDs | Sound | Sensors        |
|------------|-----------------------------------------------------------------|------|-------|----------------|
| Glowbug    | Ambient light ring that reacts to motion and room brightness.   | Ring | No    | IMU, LDR       |
| Boombox    | Drum/melody generator with two knobs for tempo and pitch.       | One  | Piezo | 2x pot, button |
| Chatterbox | Hold-to-record, release-to-play voice toy with pitch shift.     | No   | I2S   | Touch pad      |
| Finderbox  | Scans an NFC tag and plays the sound the child assigned to it.  | Ring | Piezo | RC522 NFC      |
| Brainbox   | Optional grown-up hub. Uses WiFi + an LLM for storytelling.     | OLED | —     | (via others)   |

## First charge

All boxes use the same micro-USB or USB-C cable. The charge indicator sits
next to the power switch; solid orange while charging, solid green when full.
A full charge should be enough for about a day of normal play.

## Turning a box on

Slide the power switch. The box plays a short "hello" cue (a soft tone for
Boombox, a pulse of warm colour for Glowbug, a short rainbow for Finderbox).
After a second or two, the status LED settles into its idle animation —
that means the box is awake and listening for friends.

## Pairing

There is no pairing step. Any two ThinkPack boxes that are powered on in
the same room will see each other automatically. You can tell they have
paired because their idle LEDs will start pulsing together. If you turn
a box off and back on again, it rejoins within a few seconds.

## Low battery

When a box's battery is getting low, its main LED pulses red once a second.
When it is critical (less than about five minutes of play left) the LED
stays solid red. Plug the box in to charge.

## First play ideas

- **One box:** Give Glowbug to a toddler in a quiet room — it lights up
  when they move it. Give Boombox to an older child and let them twist
  the knobs while another child dances.
- **Two boxes together:** Boombox + Glowbug — Glowbug starts pulsing in
  time with Boombox's beat. Chatterbox + Finderbox — record a sound on
  Chatterbox, tap a tag on Finderbox, and that sound is now assigned to
  that tag until you record over it.
- **A group:** Put Finderbox in the middle of the room, hide numbered
  tags around the house, and send the children to find them — each tag
  plays the story-sound you assigned to it when they scan it.

## What next

- Register your NFC tags with Finderbox — see [NFC tags](nfc-tags.md).
- Something not working? See [Troubleshooting](troubleshooting.md).
