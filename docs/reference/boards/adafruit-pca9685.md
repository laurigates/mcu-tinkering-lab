# Adafruit 16-Channel 12-bit PWM/Servo Driver — PCA9685 (#815)

Breakout for the PCA9685 PWM driver that runs robocar-unified's motor-driver
inputs, servos and LEDs. Two identical 6-pin headers on the short edges, sixteen
3-pin servo columns along one long edge and a two-way power terminal on the
other. For the silicon itself see
[`driver--pca9685.md`](../datasheets/driver--pca9685.md); the motor wiring card
(`packages/robocar/unified/docs/wiring-card-motors.typ`) draws this board too.

## Physical layout

Viewed from the component side, in the board file's own orientation: the power
terminal at the top, the servo columns along the bottom, the outline
62.23 × 25.4 mm.

`Side`/`Pos` follow the convention in
[`tools/hardware/layout.py`](../../../tools/hardware/layout.py): `L`/`R`/`T`/`B`
edge, `Pos 1` at the top of a left or right edge and at the left of a top or
bottom edge. `Header` is the vendor's element and pad. Names are the top-face
silkscreen.

The two side headers carry the same six signals, bused on the board, so either
one takes the I2C feed and the other chains onward. Each servo column is three
pads deep — `PWM` nearest the chip, then `V+`, then `GND` at the board edge — and
only the `PWM` row is listed below: the `V+` and `GND` rows of all sixteen
columns are the same two rails as the terminal block. The terminal is
reverse-polarity protected; the side-header `V+` pins are not.

| Pin | Side | Pos | Header | Notes |
|-----|------|-----|--------|-------|
| GND | L | 1 | JP3.6 | |
| OE | L | 2 | JP3.5 | Output enable, active low; pulled down on board |
| SCL | L | 3 | JP3.4 | |
| SDA | L | 4 | JP3.3 | |
| VCC | L | 5 | JP3.2 | Logic, 3–5 V |
| V+ | L | 6 | JP3.1 | Servo/output rail, not reverse-protected |
| GND | R | 1 | JP4.6 | Same signals as the left header |
| OE | R | 2 | JP4.5 | |
| SCL | R | 3 | JP4.4 | |
| SDA | R | 4 | JP4.3 | |
| VCC | R | 5 | JP4.2 | |
| V+ | R | 6 | JP4.1 | |
| V+ | T | 1 | J1.1 | Terminal block, reverse-protected |
| GND | T | 2 | J1.2 | Terminal block |
| 0 | B | 1 | JP2.10 | PWM row, channel 0 |
| 1 | B | 2 | JP2.7 | |
| 2 | B | 3 | JP2.4 | |
| 3 | B | 4 | JP2.1 | |
| 4 | B | 5 | JP1.10 | |
| 5 | B | 6 | JP1.7 | |
| 6 | B | 7 | JP1.4 | |
| 7 | B | 8 | JP1.1 | |
| 8 | B | 9 | JP6.10 | |
| 9 | B | 10 | JP6.7 | |
| 10 | B | 11 | JP6.4 | |
| 11 | B | 12 | JP6.1 | |
| 12 | B | 13 | JP5.10 | |
| 13 | B | 14 | JP5.7 | |
| 14 | B | 15 | JP5.4 | |
| 15 | B | 16 | JP5.1 | PWM row, channel 15 |

## Pad centres

Every pad of every header the layout table cites (`Header` is element.pad),
from `tools/breakout-pinout.py --json` on the board file named under Source. The table includes the `V+` and `GND` rows of the four servo blocks, which the layout table above leaves out.
Millimetres, to 0.01 mm, from the board file's own origin, with Eagle's y
negated into KiCad's convention: +y runs down the page, so a pad higher in the
view above has the smaller y.

`board_pad_centres` in
[`tools/hardware/layout.py`](../../../tools/hardware/layout.py) reads this
table. `just hardware::pad-centres-check` re-fetches the board file at the
pinned commit and fails on any difference; it needs the network, so it runs
locally and not in CI. Regenerate the rows with
`PYTHONPATH=tools python3 -m hardware.padcheck --print adafruit-pca9685` rather than
editing them.

| Header | X (mm) | Y (mm) |
|--------|--------|--------|
| J1.1 | 27.41 | -15.11 |
| J1.2 | 30.91 | -15.11 |
| JP1.10 | 17.78 | 0.00 |
| JP1.7 | 20.32 | 0.00 |
| JP1.4 | 22.86 | 0.00 |
| JP1.1 | 25.40 | 0.00 |
| JP1.11 | 17.78 | 2.54 |
| JP1.8 | 20.32 | 2.54 |
| JP1.5 | 22.86 | 2.54 |
| JP1.2 | 25.40 | 2.54 |
| JP1.12 | 17.78 | 5.08 |
| JP1.9 | 20.32 | 5.08 |
| JP1.6 | 22.86 | 5.08 |
| JP1.3 | 25.40 | 5.08 |
| JP2.10 | 5.08 | 0.00 |
| JP2.7 | 7.62 | 0.00 |
| JP2.4 | 10.16 | 0.00 |
| JP2.1 | 12.70 | 0.00 |
| JP2.11 | 5.08 | 2.54 |
| JP2.8 | 7.62 | 2.54 |
| JP2.5 | 10.16 | 2.54 |
| JP2.2 | 12.70 | 2.54 |
| JP2.12 | 5.08 | 5.08 |
| JP2.9 | 7.62 | 5.08 |
| JP2.6 | 10.16 | 5.08 |
| JP2.3 | 12.70 | 5.08 |
| JP3.6 | 0.00 | -12.70 |
| JP3.5 | 0.00 | -10.16 |
| JP3.4 | 0.00 | -7.62 |
| JP3.3 | 0.00 | -5.08 |
| JP3.2 | 0.00 | -2.54 |
| JP3.1 | 0.00 | 0.00 |
| JP4.6 | 58.42 | -12.70 |
| JP4.5 | 58.42 | -10.16 |
| JP4.4 | 58.42 | -7.62 |
| JP4.3 | 58.42 | -5.08 |
| JP4.2 | 58.42 | -2.54 |
| JP4.1 | 58.42 | 0.00 |
| JP5.10 | 45.72 | 0.00 |
| JP5.7 | 48.26 | 0.00 |
| JP5.4 | 50.80 | 0.00 |
| JP5.1 | 53.34 | 0.00 |
| JP5.11 | 45.72 | 2.54 |
| JP5.8 | 48.26 | 2.54 |
| JP5.5 | 50.80 | 2.54 |
| JP5.2 | 53.34 | 2.54 |
| JP5.12 | 45.72 | 5.08 |
| JP5.9 | 48.26 | 5.08 |
| JP5.6 | 50.80 | 5.08 |
| JP5.3 | 53.34 | 5.08 |
| JP6.10 | 33.02 | 0.00 |
| JP6.7 | 35.56 | 0.00 |
| JP6.4 | 38.10 | 0.00 |
| JP6.1 | 40.64 | 0.00 |
| JP6.11 | 33.02 | 2.54 |
| JP6.8 | 35.56 | 2.54 |
| JP6.5 | 38.10 | 2.54 |
| JP6.2 | 40.64 | 2.54 |
| JP6.12 | 33.02 | 5.08 |
| JP6.9 | 35.56 | 5.08 |
| JP6.6 | 38.10 | 5.08 |
| JP6.3 | 40.64 | 5.08 |

## Source

Read from Adafruit's Eagle board file, not from a photo
(`.claude/rules/board-layout-from-vendor-files.md`):
[`adafruit/Adafruit-16-Channel-PWM-Servo-Driver-PCB`](https://github.com/adafruit/Adafruit-16-Channel-PWM-Servo-Driver-PCB),
`Adafruit PCA9685 rev C.brd` at commit `32578c83a5ba2946249b80b1aa1fb18ae4e61e7d`,
elements `JP3`/`JP4` (side headers), `J1` (terminal) and `JP1`, `JP2`, `JP5`,
`JP6` (servo columns). The channel numbers are the silkscreen above each column;
the board file's nets for the `PWM` row are generic (`N$9`), so read the printed
`silk[top]` label beside each pad rather than the net.

Adafruit publishes the design under
[CC BY-SA 3.0](https://creativecommons.org/licenses/by-sa/3.0/) (`license.txt` at
the pinned commit). The pin is the default-branch head when the licence was
checked, not the commit that added `license.txt`; its `.brd` blob is identical
to the one this page was first read from.

```sh
gh api "repos/adafruit/Adafruit-16-Channel-PWM-Servo-Driver-PCB/contents/Adafruit%20PCA9685%20rev%20C.brd?ref=32578c83a5ba2946249b80b1aa1fb18ae4e61e7d" --jq '.content' | base64 -d > /tmp/pca9685.brd
python3 tools/breakout-pinout.py /tmp/pca9685.brd --element JP1 --element JP2 --element JP3 --element JP4 --element JP5 --element JP6 --element J1
```

Clones of #815 usually copy this layout. One whose silkscreen differs is a
different board and needs its own page.
