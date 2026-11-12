# Circuit blocks

One TOML file per circuit stage designed once and used by any carrier: a
buck converter with its passives, a pack-input protection stage. A block holds
registry parts without refs, the rails between them, and ports. The comments
in the file carry the datasheet reasoning for every value. Decision and
rationale: [ADR-031](../../decisions/ADR-031-circuit-blocks.md).

| Block | Stage | Used by |
|---|---|---|
| [`tps563208-5v`](tps563208-5v.toml) | 5 V / 3 A synchronous buck, 6.7-17 V in (2S) | robocar-integrated |
| [`pack-input-pfet-ptc`](pack-input-pfet-ptc.toml) | 2S pack input: reverse-polarity P-FET, PTC fuse | robocar-integrated |
| [`pack-input-pfet-switched`](pack-input-pfet-switched.toml) | 2S pack input with an on/off switch: back-to-back P-FETs, PTC fuse; the switch carries ~76 µA | none yet |
| [`soft-latch`](soft-latch.toml) | Push-button soft power latch driving `pack-input-pfet-switched` | none yet |
| [`button-input`](button-input.toml) | One button: on-board tact switch and off-board connector in parallel, RC debounce, TVS | none yet |

A block no carrier uses is checked for structure (`hardware::test`) and for
every pin it names existing on its registry part
(`tools/kicad/test_blocks_registry.py`), but not by ERC, a netlist or a
board until its first carrier.

## Format

```toml
[block]
name   = "..."          # required, display
source = "..."          # required: the datasheets and reviews the values come from
note   = "..."          # optional

[ports]                 # port -> block pins; every port must be connected by a user
VIN = ["ic.VIN", "cin.A"]
GND = ["ic.GND", "cin.B"]

[parts.<key>]           # a hardware.toml part table without `ref` or `build`
name = "..."
kind = "..."
part = "<registry slug>"
value = "..."
lcsc = "C123"

[[rails]]               # internal nets; named <net_prefix>_<name> in the project
name = "SW"
from = "ic.SW"
to = ["l.1"]
```

## Using a block

```toml
[blocks.buck]
block = "tps563208-5v"
net_prefix = "BUCK"
refs = { ic = "U10", l = "L1" }        # one per block part, no more
net_names = { SW = "BUCK_SW" }         # optional
parts.l = { value = "3.3uH" }          # optional: name, note, value, lcsc, buy
build = "carrier"                      # optional

[[rails]]
name = "5V"
from = "buck.VOUT"
to = ["load.VCC"]
```

`tools/hardware/blocks.py` expands the instance when the file is read: part
`l` becomes `[parts.buck_l]`, rail `SW` becomes `BUCK_SW`, and `buck.VOUT`
becomes the port's pins. Override a value in the project, with a comment
saying why, rather than editing the block for one board: a block change
regenerates every carrier that uses it.

After adding or changing a block, run `just hardware::test`, `just
kicad::check` and, with KiCad 10.0.6, `just <project>::carrier-check` for each
project that uses it.
