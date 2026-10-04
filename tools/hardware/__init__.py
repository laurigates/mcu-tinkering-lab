"""One hardware source of truth: a board × header × parts join (ADR-021).

    docs/reference/boards/<board>.md ─┐
    <project>/main/pin_config.h      ─┼─> join(project_dir) -> HardwareModel
    <project>/hardware.toml          ─┘

Stdlib only (Python 3.11+ for tomllib), so every consumer — the drift guard's
system python3, a justfile recipe, a pre-commit hook — can import it with
nothing installed. Two emitters: `tools/typst/generate-pin-defs.py` (the build
guide's Typst bindings) and `hardware.docs` (the marked tables and power
diagram in WIRING.md).

`hardware.layout` reads the board-facts layer on its own: each board's
physical header layout, for drawings meant to be wired from (#495).
`hardware.pinout` draws those layouts as the build guide's pinout images,
labelled from the join (#629).
"""

from .board import Board, BoardPin, parse_board_table
from .errors import HardwareError
from .header import CONVENTIONS, parse_defines, roles_from_defines
from .layout import ModuleLayout, Pad, board_layout, parse_layout
from .model import MCU, Endpoint, HardwareModel, Net, Output, Part, Rail, Undrawn, join

__all__ = [
    "CONVENTIONS",
    "MCU",
    "Board",
    "BoardPin",
    "Endpoint",
    "HardwareError",
    "HardwareModel",
    "ModuleLayout",
    "Net",
    "Output",
    "Pad",
    "Part",
    "Rail",
    "Undrawn",
    "board_layout",
    "join",
    "parse_board_table",
    "parse_layout",
    "parse_defines",
    "roles_from_defines",
]
