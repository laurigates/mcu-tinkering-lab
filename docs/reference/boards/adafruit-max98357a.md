# Adafruit MAX98357A I2S Class-D Mono Amp Breakout (#3006)

Breakout for the MAX98357A I2S amplifier that gives robocar-unified its voice
(ADR-019). A 7-pin header along one edge and a two-way speaker terminal block on
the opposite edge.

## Physical layout

Viewed from the component side, in the board file's own orientation: the
7-pin header along the bottom edge, the terminal block at the top, the outline
17.78 × 19.05 mm.

`Side`/`Pos` follow the convention in
[`tools/hardware/layout.py`](../../../tools/hardware/layout.py): `L`/`R`/`T`/`B`
edge, `Pos 1` at the left of a top or bottom edge. `Header` is the vendor's
element and pad. Header names are the top-face silkscreen (the board file's nets
call `Vin` `VDD` and `SD` `SD_MODE`). The terminal block has no silkscreen, so
its two pads carry the vendor's net names, `VO-` and `VO+`.

| Pin | Side | Pos | Header | Notes |
|-----|------|-----|--------|-------|
| VO- | T | 1 | X1.1 | Speaker − |
| VO+ | T | 2 | X1.2 | Speaker + |
| LRC | B | 1 | JP1.7 | I2S word select |
| BCLK | B | 2 | JP1.6 | I2S bit clock |
| DIN | B | 3 | JP1.5 | I2S data |
| GAIN | B | 4 | JP1.4 | Floating = 9 dB |
| SD | B | 5 | JP1.3 | Shutdown / channel select; floating = (L+R)/2 |
| GND | B | 6 | JP1.2 | |
| Vin | B | 7 | JP1.1 | 2.5–5.5 V |

## Source

Read from Adafruit's Eagle board file, not from a photo
(`.claude/rules/board-layout-from-vendor-files.md`):
[`adafruit/Adafruit-MAX98357-I2S-Amp-Breakout`](https://github.com/adafruit/Adafruit-MAX98357-I2S-Amp-Breakout),
`Adafruit MAX98357 Breakout.brd` (the mono board, not the stereo rev B) at commit
`9766975c856c6a5733eaf53b4d94109ffd9aa33e`, elements `JP1` (header) and `X1`
(terminal block).

```sh
gh api "repos/adafruit/Adafruit-MAX98357-I2S-Amp-Breakout/contents/Adafruit%20MAX98357%20Breakout.brd" --jq '.content' | base64 -d > /tmp/max98357.brd
python3 tools/breakout-pinout.py /tmp/max98357.brd --element JP1 --element X1
```

Clones of #3006 usually copy this layout. One whose silkscreen differs is a
different board and needs its own page.
