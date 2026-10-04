# Adafruit TCA9548A 1-to-8 I2C Multiplexer Breakout (#2717)

Breakout for the TCA9548A I2C multiplexer. robocar-unified puts every I2C device
behind it (`I2C_BUS_CHANNEL_*` in `main/pin_config.h`). Two 12-pin headers on the
long edges: power, the upstream bus, the reset and address pins and channels 0–1
on the left; channels 2–7 on the right.

## Physical layout

Viewed from the component side, in the board file's own orientation: `VIN` at the
top-left corner, the outline 17.78 × 30.48 mm. Downstream channels are pairs
named `SDn`/`SCn`; on the right edge each pair runs `SCn` above `SDn`, on the
left `SDn` above `SCn`.

`Side`/`Pos` follow the convention in
[`tools/hardware/layout.py`](../../../tools/hardware/layout.py): `L`/`R`/`T`/`B`
edge, `Pos 1` at the top of a left or right edge. `Header` is the vendor's
element and pad. Names are the top-face silkscreen; the board file's net names
differ (`INPUTSDA`, `0SDA`, …) and are not printed anywhere.

| Pin | Side | Pos | Header | Notes |
|-----|------|-----|--------|-------|
| VIN | L | 1 | JP3.1 | 1.8–5 V (bottom face) |
| GND | L | 2 | JP3.2 | |
| SDA | L | 3 | JP3.3 | Upstream bus |
| SCL | L | 4 | JP3.4 | Upstream bus |
| RST | L | 5 | JP3.5 | Active-low reset |
| A0 | L | 6 | JP3.6 | Address select |
| A1 | L | 7 | JP3.7 | Address select |
| A2 | L | 8 | JP3.8 | Address select |
| SD0 | L | 9 | JP3.9 | Channel 0 |
| SC0 | L | 10 | JP3.10 | Channel 0 |
| SD1 | L | 11 | JP3.11 | Channel 1 |
| SC1 | L | 12 | JP3.12 | Channel 1 |
| SC7 | R | 1 | JP1.12 | Channel 7 |
| SD7 | R | 2 | JP1.11 | Channel 7 |
| SC6 | R | 3 | JP1.10 | Channel 6 |
| SD6 | R | 4 | JP1.9 | Channel 6 |
| SC5 | R | 5 | JP1.8 | Channel 5 |
| SD5 | R | 6 | JP1.7 | Channel 5 |
| SC4 | R | 7 | JP1.6 | Channel 4 |
| SD4 | R | 8 | JP1.5 | Channel 4 |
| SC3 | R | 9 | JP1.4 | Channel 3 |
| SD3 | R | 10 | JP1.3 | Channel 3 |
| SC2 | R | 11 | JP1.2 | Channel 2 |
| SD2 | R | 12 | JP1.1 | Channel 2 |

## Source

Read from Adafruit's Eagle board file, not from a photo
(`.claude/rules/board-layout-from-vendor-files.md`):
[`adafruit/Adafruit-TCA9548A-I2C-Multiplexer-PCB`](https://github.com/adafruit/Adafruit-TCA9548A-I2C-Multiplexer-PCB),
`Adafruit TCA9548A.brd` at commit `69a8a154e26bbce3330d477eec245a2e783ca863`,
elements `JP3` (left) and `JP1` (right).

```sh
gh api "repos/adafruit/Adafruit-TCA9548A-I2C-Multiplexer-PCB/contents/Adafruit%20TCA9548A.brd" --jq '.content' | base64 -d > /tmp/tca9548a.brd
python3 tools/breakout-pinout.py /tmp/tca9548a.brd --element JP1 --element JP3
```

Generic TCA9548A modules do not all copy this layout. One whose silkscreen
differs is a different board and needs its own page.
