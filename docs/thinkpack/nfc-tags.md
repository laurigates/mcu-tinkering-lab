# Using NFC tags with Finderbox

Finderbox reads NFC tags and plays the behaviour assigned to each one.
Tags are cheap, re-usable, and come in stickers, cards, tokens, and plush
form factors — whatever suits the child.

## Compatible tag types

Finderbox speaks to the RC522 reader at 13.56 MHz and supports the two
most common tag families:

- **MIFARE Classic 1K** — the traditional blue-sticker tag. Widely
  available, cheapest per tag, ~1 KB of user memory. Good default.
- **NTAG 213 / 215 / 216** — the newer NFC Forum Type 2 tags. Slightly
  more expensive but better compatibility with phones if you ever want
  to program a tag from your phone.

Either family works for all Finderbox behaviours. If you have a mixed
bag, you can use both in the same room; the box recognises each tag by
its UID, not by its type.

## Registering a tag

Every tag needs to be "registered" before Finderbox will do anything with
it. Two ways:

### Scan in registration mode

1. Hold the Finderbox button for 3 seconds. The LED ring turns purple
   and a short tone plays — you are in registration mode.
2. Tap the tag you want to register. Finderbox reads the UID and plays
   a "captured" chime.
3. Choose the behaviour with the button (each press cycles through
   Chime, Story, Color, Seek — the LED ring previews each).
4. Long-press the button to save. Registration mode exits.

### Preload from a file (optional, for planners)

If you are setting up for a group activity (e.g. a treasure hunt) you
may prefer to preload tag assignments from a text file. The Finderbox
CLI tool reads a simple CSV and writes the registry into NVS over USB.
The format is:

```
uid,behaviour,asset
04:12:AB:CD,story,tree
04:56:EF:01,chime,bell
```

See the Finderbox README (`packages/thinkpack/finderbox/README.md`) for
the CLI tool invocation.

## Behaviours

| Behaviour | What happens when the tag is scanned                       |
|-----------|------------------------------------------------------------|
| Chime     | Plays a short musical cue (bell, chirp, horn).             |
| Story     | Plays a story-sound clip recorded on a Chatterbox.         |
| Color     | Turns the LED ring a specific colour for 10 seconds.       |
| Seek      | Used in Hot-Cold group mode — warms the other boxes toward the tag. |

## Example play scenarios

- **Bedtime wind-down:** Put three tags on three books. Each tag is
  assigned a different chime. The child picks tonight's book by tapping
  it on Finderbox.
- **Treasure hunt:** Scatter ten tags around the garden, each with a
  different Story clip recorded on Chatterbox ("climb the tree",
  "look under the watering can", etc.).
- **Colour memory game:** Register five tags with different Colour
  behaviours. Call out a colour, the child finds the tag that makes
  Finderbox light up with that colour.

## Erasing

Hold the button for 10 seconds with a tag on the reader — the
registration is removed. Hold for 30 seconds with no tag present to
clear the entire registry (factory reset).
