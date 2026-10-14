# seeed-xiao: the Seeed Studio XIAO footprints and symbols the carriers use

Vendored so that CI can generate and check a carrier board without KiCad
installed, and so that local ERC and DRC read the bytes CI pins. The carrier
projects' library tables point the `Seeed_XIAO` nickname at this directory.

- Source: [Seeed-Studio/OSHW-XIAO-Series](https://github.com/Seeed-Studio/OSHW-XIAO-Series)
  at commit `cc7f3bb` (`cc7f3bbfc69f20061e533304b160bedcf377844a`), directory
  `Seeed Studio XIAO Series Library/`.
- Licence: MIT, Seeed Studio; see [LICENSE](LICENSE), the upstream file at that
  commit, unchanged.

## Vendor files and derived files

| File | What it is |
|---|---|
| `Seeed_XIAO.pretty/XIAO-RP2350-DIP.kicad_mod` | vendor: upstream `XIAO-RP2350-DIP.kicad_mod`, unchanged apart from CRLF line endings converted to LF |
| `Seeed_XIAO.kicad_sym`, symbol `XIAO-RP2350-DIP` | vendor: the header lines and the `XIAO-RP2350-DIP` block of upstream `Seeed_Studio_XIAO_Series.kicad_sym`, unchanged apart from CRLF to LF |
| `Seeed_XIAO.kicad_sym`, symbol `XIAO-RP2350-Socket` | derived from the DIP symbol: pins 15-24 removed, footprint `Seeed_XIAO:XIAO-RP2350-Socket` |
| `Seeed_XIAO.pretty/XIAO-RP2350-Socket.kicad_mod` | derived from the DIP footprint: through-hole pads 1-14 only, no Edge.Cuts cutout, no castellations |
| `Seeed_XIAO.pretty/XIAO-ESP32-S3-DIP.kicad_mod` | vendor: upstream `XIAO-ESP32-S3-DIP.kicad_mod`, unchanged apart from CRLF to LF and a final newline (upstream has none; the repo's end-of-file-fixer hook adds it) |
| `Seeed_XIAO.kicad_sym`, symbol `XIAO-ESP32-S3-SMD` | vendor: the `XIAO-ESP32-S3-SMD` block of upstream `Seeed_Studio_XIAO_Series.kicad_sym`, unchanged apart from CRLF to LF. Seeed ships no `XIAO-ESP32-S3-DIP` symbol; the SMD one numbers the same 14 header pins |
| `Seeed_XIAO.kicad_sym`, symbol `XIAO-ESP32-S3-Socket` | derived from the SMD symbol: pins 15-25 removed, footprint `Seeed_XIAO:XIAO-ESP32-S3-Socket` |
| `Seeed_XIAO.pretty/XIAO-ESP32-S3-Socket.kicad_mod` | derived from the DIP footprint: through-hole pads 1-14 only, no castellations (the upstream footprint has no Edge.Cuts); the pads sit at the same centres as `XIAO-RP2350-Socket` |
| `derive.toml` | which sockets are derived from which vendor parts, and the pads they keep |

Seeed's DIP footprint is for soldering the XIAO flat onto a host board: it mills
a cutout under the module and plates holes for the bottom pads along it. A
socketed XIAO needs neither, so the carriers use the derived socket parts. The
vendor parts stay here unmodified as their source.

## Maintenance

Never edit these files by hand. `MANIFEST.tsv` records each file's sha256, and
`PYTHONPATH=tools python3 -m kicad.vendor --check` (CI) fails on any file that
does not match it or that it does not list. The derived parts are regenerated
from the vendor parts and `derive.toml`, which also rewrites `MANIFEST.tsv`:

```sh
PYTHONPATH=tools python3 -m kicad.derive_xiao                  # regenerate
PYTHONPATH=tools python3 -m kicad.derive_xiao --check          # CI: fail if stale
PYTHONPATH=tools python3 -m kicad.derive_xiao --update-vendor  # after re-vendoring
```

Regenerating re-pins only the derived parts. It refuses, and writes nothing,
when a vendor file (`LICENSE`, a vendor footprint, a vendor symbol block)
differs from its pin, because re-pinning would hide the edit from
`vendor --check`: restore the upstream bytes instead.

To vendor another XIAO, copy its upstream footprint and symbol block here
(converting line endings to LF), add a `[[socket]]` table, and run
`derive_xiao --update-vendor`, which pins the new vendor files as well.
