"""Physical header layout: which pad sits where on a board, viewed from above.

One reader for every board a drawing is meant to be wired from (#495, #629).
The fact is the same for an MCU module and a breakout — each pad's printed
name, the edge it sits on, and its position along that edge — so both come back
as a `ModuleLayout`, whichever document holds them:

  * An MCU board's reference (`docs/reference/boards/xiao-esp32s3.md`) already
    carries `Side`/`Pos` in its pin-mapping table, so it is read through
    `parse_board_table` — the same rows, not a second copy of them — and each
    pad keeps its GPIO.
  * A breakout's reference (`docs/reference/boards/sparkfun-tb6612fng.md`)
    has no GPIOs, so its table is `| Pin | Side | Pos | ... |` with no `GPIO`
    column. Its order comes from the vendor's Eagle board file via
    `tools/breakout-pinout.py` (.claude/rules/board-layout-from-vendor-files.md),
    and the page says which file and commit.

Conventions, shared by both:

  * The view is the component side up, in the orientation the page states
    (USB-C at the top for the XIAO; the board file's own +y for a breakout).
  * `Side` is `L`, `R`, `T` or `B`. `Pos` is 1-based: from the top on `L`/`R`,
    from the left on `T`/`B`.
  * Every side is complete — positions run 1..n with no gap — because a
    drawing whose pads cannot be counted against the board is the defect this
    exists to remove.

A pad name a board repeats (two `GND`s) cannot be one schematic anchor — the
second would silently replace the first — so a repeated name is anchored as
`<name>.<side><pos>` (`GND.L3`); a unique name is its own anchor.
"""

from __future__ import annotations

from collections import Counter
from dataclasses import dataclass
from pathlib import Path

from .board import _column, _is_mapping, _tables, parse_board_table
from .errors import HardwareError

REPO_ROOT = Path(__file__).resolve().parents[2]
BOARDS_DIR = REPO_ROOT / "docs/reference/boards"
SIDES = ("L", "R", "T", "B")


@dataclass(frozen=True)
class Pad:
    name: str  # as printed on the board: SDA, GND, D4, 0
    side: str  # L / R / T / B
    pos: int  # 1-based along that side: from the top on L/R, the left on T/B
    anchor: str  # unique within the board: the name, or name.<side><pos>
    gpio: int | None = None  # MCU boards only; None for power pads and breakouts


@dataclass(frozen=True)
class ModuleLayout:
    path: Path
    pads: tuple[Pad, ...]

    def side(self, side: str) -> tuple[Pad, ...]:
        """The pads on one edge, in position order."""
        return tuple(
            sorted((p for p in self.pads if p.side == side), key=lambda p: p.pos)
        )

    @property
    def by_anchor(self) -> dict[str, Pad]:
        return {p.anchor: p for p in self.pads}


def _is_layout(header: list[str]) -> bool:
    lowered = [h.lower() for h in header]
    return "pin" in lowered[0] and "side" in lowered and "pos" in lowered


def _breakout_rows(path: Path, table: list[list[str]]) -> list[tuple[str, str, int]]:
    header, *rows = table
    side_col, pos_col = _column(header, "side"), _column(header, "pos")
    out: list[tuple[str, str, int]] = []
    for row in rows:
        name = row[0]
        side = row[side_col] if side_col < len(row) else ""
        raw_pos = row[pos_col] if pos_col < len(row) else ""
        if side not in SIDES:
            raise HardwareError(
                f"{path}: pin {name}: side {side!r} is not one of {SIDES}"
            )
        if not raw_pos.isdigit() or int(raw_pos) < 1:
            raise HardwareError(
                f"{path}: pin {name}: position {raw_pos!r} is not 1 or more"
            )
        out.append((name, side, int(raw_pos)))
    return out


def _check_complete(path: Path, rows: list[tuple[str, str, int, int | None]]) -> None:
    claimed: dict[tuple[str, int], str] = {}
    for name, side, pos, _ in rows:
        if (side, pos) in claimed:
            raise HardwareError(
                f"{path}: position {side} {pos} is claimed by both {claimed[side, pos]} and {name}"
            )
        claimed[side, pos] = name
    for side in SIDES:
        positions = sorted(pos for s, pos in claimed if s == side)
        if positions and positions != list(range(1, len(positions) + 1)):
            raise HardwareError(
                f"{path}: side {side} has positions {positions}; expected 1..{len(positions)} "
                "with no gap — every pad on the header, not only the ones in use"
            )


def parse_layout(path: Path) -> ModuleLayout:
    """Read the physical header layout from the board reference at `path`."""
    tables = _tables(path.read_text(encoding="utf-8"))
    mapping = [t for t in tables if _is_mapping(t[0])]
    breakout = [t for t in tables if _is_layout(t[0]) and not _is_mapping(t[0])]
    if len(mapping) + len(breakout) > 1:
        raise HardwareError(
            f"{path}: {len(mapping) + len(breakout)} physical-layout tables; expected exactly one"
        )

    rows: list[tuple[str, str, int, int | None]]
    if mapping:
        board = parse_board_table(path)
        missing = [p.name for p in board.pins if p.side is None or p.pos is None]
        if missing:
            raise HardwareError(
                f"{path}: pin(s) {', '.join(missing)} have no physical position "
                "(the pin-mapping table needs Side and Pos for every pad)"
            )
        rows = [(p.name, p.side, p.pos, p.gpio) for p in board.pins]  # type: ignore[misc]
    elif breakout:
        rows = [(n, s, p, None) for n, s, p in _breakout_rows(path, breakout[0])]
    else:
        raise HardwareError(
            f"{path}: no physical-layout table (first column a pin, columns 'Side' and 'Pos')"
        )

    _check_complete(path, rows)
    repeats = Counter(name for name, *_ in rows)
    pads = tuple(
        Pad(
            name=name,
            side=side,
            pos=pos,
            anchor=name if repeats[name] == 1 else f"{name}.{side}{pos}",
            gpio=gpio,
        )
        for name, side, pos, gpio in rows
    )
    return ModuleLayout(path=path, pads=pads)


def board_layout(slug: str) -> ModuleLayout:
    """The layout in `docs/reference/boards/<slug>.md`."""
    path = BOARDS_DIR / f"{slug}.md"
    if not path.is_file():
        raise HardwareError(
            f"no board reference {path.relative_to(REPO_ROOT)} "
            f"(looked in {BOARDS_DIR.relative_to(REPO_ROOT)})"
        )
    return parse_layout(path)
