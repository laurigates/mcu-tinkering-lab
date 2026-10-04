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
