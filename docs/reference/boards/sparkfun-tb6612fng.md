# SparkFun Motor Driver — Dual TB6612FNG (ROB-14451)

Breakout for the TB6612FNG dual H-bridge, used by robocar-unified to drive both
wheel motors. Two 8-pin headers on the long edges, outputs on one side and
control on the other. For the silicon itself see
[`driver--tb6612fng.md`](../datasheets/driver--tb6612fng.md).

## Physical layout

Viewed from the component side, in the board file's own orientation: the
SparkFun flame logo at the top, the outline 20.32 × 20.32 mm. Both rows start
at the same end — `VM` and `PWMA` are the top corners — and both end in `GND`.

`Side`/`Pos` follow the convention in
[`tools/hardware/layout.py`](../../../tools/hardware/layout.py): `L`/`R`/`T`/`B`
edge, `Pos 1` at the top of a left or right edge. `Header` is the vendor's
element and pad, so any row can be checked against the board file. Names are the
vendor's net names, which the bottom face prints in full; the top face
abbreviates the control row (`AI2`, `AI1`, `ST BY`, `BI1`, `BI2`) and prints
the outputs with a zero (`A01`), not a letter O.

| Pin | Side | Pos | Header | Notes |
|-----|------|-----|--------|-------|
| VM | L | 1 | JP1.8 | Motor supply |
| VCC | L | 2 | JP1.7 | Logic supply, 2.7–5.5 V |
| GND | L | 3 | JP1.6 | |
| A01 | L | 4 | JP1.5 | Motor A output |
| A02 | L | 5 | JP1.4 | Motor A output |
| B02 | L | 6 | JP1.3 | Motor B output |
| B01 | L | 7 | JP1.2 | Motor B output |
| GND | L | 8 | JP1.1 | |
| PWMA | R | 1 | JP2.1 | Channel A speed |
| AIN2 | R | 2 | JP2.2 | Channel A direction |
| AIN1 | R | 3 | JP2.3 | Channel A direction |
| STBY | R | 4 | JP2.4 | High = enabled |
| BIN1 | R | 5 | JP2.5 | Channel B direction |
| BIN2 | R | 6 | JP2.6 | Channel B direction |
| PWMB | R | 7 | JP2.7 | Channel B speed |
| GND | R | 8 | JP2.8 | |

## Pad centres

Every pad of every header the layout table cites (`Header` is element.pad),
from `tools/breakout-pinout.py --json` on the board file named under Source.
Millimetres, to 0.01 mm, from the board file's own origin, with Eagle's y
negated into KiCad's convention: +y runs down the page, so a pad higher in the
view above has the smaller y.

`board_pad_centres` in
[`tools/hardware/layout.py`](../../../tools/hardware/layout.py) reads this
table. `just hardware::pad-centres-check` re-fetches the board file at the
pinned commit and fails on any difference; it needs the network, so it runs
locally and not in CI. Regenerate the rows with
`PYTHONPATH=tools python3 -m hardware.padcheck --print sparkfun-tb6612fng` rather than
editing them.

| Header | X (mm) | Y (mm) |
|--------|--------|--------|
| JP1.8 | 2.54 | -19.05 |
| JP1.7 | 2.54 | -16.51 |
| JP1.6 | 2.54 | -13.97 |
| JP1.5 | 2.54 | -11.43 |
| JP1.4 | 2.54 | -8.89 |
| JP1.3 | 2.54 | -6.35 |
| JP1.2 | 2.54 | -3.81 |
| JP1.1 | 2.54 | -1.27 |
| JP2.1 | 17.78 | -19.05 |
| JP2.2 | 17.78 | -16.51 |
| JP2.3 | 17.78 | -13.97 |
| JP2.4 | 17.78 | -11.43 |
| JP2.5 | 17.78 | -8.89 |
| JP2.6 | 17.78 | -6.35 |
| JP2.7 | 17.78 | -3.81 |
| JP2.8 | 17.78 | -1.27 |

## Source

Read from SparkFun's Eagle board file, not from a photo
(`.claude/rules/board-layout-from-vendor-files.md`):
[`sparkfun/Motor_Driver-Dual_TB6612FNG`](https://github.com/sparkfun/Motor_Driver-Dual_TB6612FNG),
`Hardware/SparkFun_Motor_Driver-TB6612FNG_v11.brd` at commit
`1b71b6b4b41dd7bc564d49d6836da763a935dea5`, elements `JP1` and `JP2`.

```sh
gh api "repos/sparkfun/Motor_Driver-Dual_TB6612FNG/contents/Hardware/SparkFun_Motor_Driver-TB6612FNG_v11.brd" --jq '.content' | base64 -d > /tmp/tb6612.brd
python3 tools/breakout-pinout.py /tmp/tb6612.brd --element JP1 --element JP2
```

Clones of ROB-14451 usually copy this layout. A board with a different
silkscreen order is a different board and needs its own page.
