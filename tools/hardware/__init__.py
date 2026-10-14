"""One hardware source of truth: a board × header × parts join (ADR-021).

    docs/reference/boards/<board>.md ─┐
    <project>/main/pin_config.h      ─┼─> join(project_dir) -> HardwareModel
    <project>/hardware.toml          ─┘

Stdlib only (Python 3.11+ for tomllib), so every consumer — the drift guard's
system python3, a justfile recipe, a pre-commit hook — can import it with
nothing installed. Three emitters: `tools/typst/generate-pin-defs.py` (the build
guide's Typst bindings) and `hardware.docs` (the marked tables and power
diagram in WIRING.md) read the default HARNESS view; `tools/kicad` (#750) reads
the CARRIER view, `join(project_dir, view=CARRIER)`, for the carrier PCB.

`hardware.layout` reads the board-facts layer on its own: each board's
physical header layout, for drawings meant to be wired from (#495).
`hardware.pinout` draws those layouts as the build guide's pinout images,
labelled from the join (#629). A breakout page can also record its headers' pad
centres (`board_pad_centres`), which `hardware.padcheck` re-derives from the
vendor's board file on demand (#750).
"""

from .board import Board, BoardPin, parse_board_table
from .errors import HardwareError
from .header import (
    BUS_CONVENTIONS,
    CHANNEL_CONVENTIONS,
    CONVENTIONS,
    bus_channels_from_defines,
    channels_from_defines,
    parse_defines,
    roles_from_defines,
)
from .layout import (
    ModuleLayout,
    Pad,
    PadCentre,
    board_layout,
    board_pad_centres,
    parse_layout,
    parse_pad_centres,
)
from .model import (
    CARRIER,
    HARNESS,
    MCU,
    VIEWS,
    BusNet,
    ChannelNet,
    Endpoint,
    HardwareModel,
    Net,
    Output,
    Part,
    Rail,
    Undrawn,
    join,
)

__all__ = [
    "BUS_CONVENTIONS",
    "CARRIER",
    "CHANNEL_CONVENTIONS",
    "CONVENTIONS",
    "HARNESS",
    "MCU",
    "VIEWS",
    "Board",
    "BoardPin",
    "BusNet",
    "ChannelNet",
    "Endpoint",
    "HardwareError",
    "HardwareModel",
    "ModuleLayout",
    "Net",
    "Output",
    "Pad",
    "PadCentre",
    "Part",
    "Rail",
    "Undrawn",
    "board_layout",
    "board_pad_centres",
    "bus_channels_from_defines",
    "channels_from_defines",
    "join",
    "parse_board_table",
    "parse_defines",
    "parse_layout",
    "parse_pad_centres",
    "roles_from_defines",
]
