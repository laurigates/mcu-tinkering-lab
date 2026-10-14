# kicad-stock: the KiCad library parts the carriers use

A subset of the official KiCad libraries at tag **10.0.6**, vendored so that CI
can generate and check a carrier board without KiCad installed, and so that local
ERC and DRC read the same bytes CI pins. The carrier projects' library tables
point the stock nicknames (`Device`, `Resistor_SMD`, ...) at these files.

- Source: [kicad-symbols](https://gitlab.com/kicad/libraries/kicad-symbols) and
  [kicad-footprints](https://gitlab.com/kicad/libraries/kicad-footprints), tag
  `10.0.6`.
- Attribution: KiCad Libraries contributors.
- Licence: [CC-BY-SA 4.0](https://creativecommons.org/licenses/by-sa/4.0/legalcode)
  with the KiCad design exception; see [LICENSE.md](LICENSE.md), which is the
  upstream file unchanged (it is byte-identical in both repositories). The
  repository's MIT licence does not cover this directory.

## Modifications

The `.kicad_sym` files in `symbols/` are extracted subsets: each keeps its
library's header lines and only the symbols the carriers reference. Each symbol
block is unchanged from kicad-symbols 10.0.6. The footprints in `footprints/`
are unmodified copies from kicad-footprints 10.0.6.

## Maintenance

Never edit these files by hand. `MANIFEST.tsv` records each vendored file's
sha256 and upstream tag, and `python3 -m kicad.vendor --check` (CI) fails on any
file that does not match it, on a file it does not list, and on any part that is
vendored but not referenced or referenced but not vendored.

Regenerate with `just kicad::vendor`, which runs, from the repository root:

```sh
PYTHONPATH=tools python3 -m kicad.vendor --check-installed   # drift vs the installed KiCad
PYTHONPATH=tools python3 -m kicad.vendor --update            # rebuild from the installed KiCad
```

`--update` requires the installed KiCad to be the version in `vendor.TAG`; a
KiCad upgrade bumps that tag and this directory's `LICENSE.md` together.
