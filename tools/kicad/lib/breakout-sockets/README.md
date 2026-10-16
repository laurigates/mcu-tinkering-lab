# breakout-sockets: socket footprints for the breakouts a carrier takes

Generated, not vendored: `kicad.sockets` writes one footprint per
`[[socket]]` in [`sockets.toml`](sockets.toml), from the pad centres the
breakout's board page records (`docs/reference/boards/<board>.md`,
`## Pad centres`). The carrier projects' library tables point the
`Breakout_Sockets` nickname at this directory. Sources and licences:
[LICENSE.md](LICENSE.md).

| Footprint | Board page | Pads, in footprint order |
|---|---|---|
| `SparkFun_TB6612FNG_Socket` | `sparkfun-tb6612fng` | 1-8 = JP1.1-8, 9-16 = JP2.1-8 |
| `Adafruit_MAX98357A_Socket` | `adafruit-max98357a` | 1-7 = JP1.1-7 |
| `Adafruit_TCA9548A_Socket` | `adafruit-tca9548a` | 1-12 = JP3.1-12, 13-24 = JP1.1-12 |
| `Adafruit_PCA9685_Socket` | `adafruit-pca9685` | 1-6 = JP3.1-6, 7-12 = the PWM pads of channels 8-13 (JP6.10, JP6.7, JP6.4, JP6.1, JP5.10, JP5.7) |

Each footprint's `descr` repeats its pad order. Pad 1 is the origin and keeps
PinSocket's square pad; the others are PinSocket's round pad. Pad positions are
the page's centres minus pad 1's: the page is already in KiCad's y-down
convention. A board with no vendor board file, and so no recorded centres,
lists its positions in `sockets.toml` as `pad_xy` instead. F.SilkS is a
rectangle around the pad centres at PinSocket's margin (1.33 mm). F.CrtYd
covers both the pads at PinSocket's margin (1.77 mm) and the breakout's
`outline` at 0.25 mm, so the courtyard check keeps other parts out from under
the module.

## Maintenance

Never edit the footprints by hand. Edit `sockets.toml` or the board page and
regenerate; regenerating also rewrites `MANIFEST.tsv`, which
`PYTHONPATH=tools python3 -m kicad.vendor --check` (CI) holds the files to.

```sh
PYTHONPATH=tools python3 -m kicad.sockets           # regenerate (also `just kicad::gen`)
PYTHONPATH=tools python3 -m kicad.sockets --check   # CI: fail if stale (also `just kicad::check`)
```

A board page's pad centres are themselves checked against the vendor's board
file by `just hardware::pad-centres-check` (needs the network; local only).
