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

## Source

Read from Adafruit's Eagle board file, not from a photo
(`.claude/rules/board-layout-from-vendor-files.md`):
[`adafruit/Adafruit-16-Channel-PWM-Servo-Driver-PCB`](https://github.com/adafruit/Adafruit-16-Channel-PWM-Servo-Driver-PCB),
`Adafruit PCA9685 rev C.brd` at commit `3f2962d03ee324b70561d8659f56315ee437f437`,
elements `JP3`/`JP4` (side headers), `J1` (terminal) and `JP1`, `JP2`, `JP5`,
`JP6` (servo columns). The channel numbers are the silkscreen above each column;
the board file's nets for the `PWM` row are generic (`N$9`), so read the printed
`silk[top]` label beside each pad rather than the net.

```sh
gh api "repos/adafruit/Adafruit-16-Channel-PWM-Servo-Driver-PCB/contents/Adafruit%20PCA9685%20rev%20C.brd" --jq '.content' | base64 -d > /tmp/pca9685.brd
python3 tools/breakout-pinout.py /tmp/pca9685.brd --element JP1 --element JP2 --element JP3 --element JP4 --element JP5 --element JP6 --element J1
```

Clones of #815 usually copy this layout. One whose silkscreen differs is a
different board and needs its own page.
