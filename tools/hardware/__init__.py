"""One hardware source of truth: a board × header × parts join (ADR-021).

    docs/reference/boards/<board>.md ─┐
    <project>/main/pin_config.h      ─┼─> join(project_dir) -> HardwareModel
    <project>/hardware.toml          ─┘

Stdlib only (Python 3.11+ for tomllib), so every consumer — the drift guard's
system python3, a justfile recipe, a pre-commit hook — can import it with
nothing installed. `tools/typst/generate-pin-defs.py` is the first emitter.
"""

from .board import Board, BoardPin, parse_board_table
from .errors import HardwareError
from .header import CONVENTIONS, parse_defines, roles_from_defines
from .model import HardwareModel, Net, Part, Undrawn, join

__all__ = [
    "CONVENTIONS",
    "Board",
    "BoardPin",
    "HardwareError",
    "HardwareModel",
    "Net",
    "Part",
    "Undrawn",
    "join",
    "parse_board_table",
    "parse_defines",
    "roles_from_defines",
]
