"""Board-facts layer: the pin-mapping table in docs/reference/boards/<board>.md.

A board reference carries one Markdown table mapping each header pad to its
GPIO — `| D4 | GPIO5 | L | 5 | I2C SDA | ADC1_CH4, Touch5 |`. That table is
the fact this module reads. It is found by shape rather than by heading:

  * the first column header names a pin (`XIAO Pin`, `Board Pin`, `Pin`), and
  * one column header is exactly `GPIO`.

The shape test is what keeps a GPIO-keyed table out — the "Pins to Use with
Caution" table has a GPIO column too, but its first column is the GPIO. A
document with no such table, or with two, is an error: guessing which one is
the mapping is how a drawing ends up wired from the wrong one.

`Side` and `Pos` are optional. Where present they are the physical header
position, and a pad claiming a position another pad already holds is an error.
"""

from __future__ import annotations

import re
from dataclasses import dataclass
from pathlib import Path

from .errors import HardwareError

_GPIO = re.compile(r"GPIO(\d+)")
_NO_GPIO = {"—", "–", "-", ""}


@dataclass(frozen=True)
class BoardPin:
    name: str  # silkscreen name: D4, 5V, GND
    gpio: int | None  # None for power and ground pads
    side: str | None  # "L" / "R" where the table says, else None
    pos: int | None  # 1-based position along that side
    function: str  # default function column, "" if the table has none
    alternates: str  # alternate-functions column, "" if the table has none
    strapping: bool


@dataclass(frozen=True)
class Board:
    path: Path
    pins: tuple[BoardPin, ...]

    @property
    def by_name(self) -> dict[str, BoardPin]:
        return {p.name: p for p in self.pins}

    @property
    def by_gpio(self) -> dict[int, BoardPin]:
        return {p.gpio: p for p in self.pins if p.gpio is not None}


def _cells(line: str) -> list[str]:
    return [c.strip() for c in line.strip().strip("|").split("|")]


def _tables(text: str) -> list[list[list[str]]]:
    """Every Markdown table as a list of rows (header first, separator dropped)."""
    tables: list[list[list[str]]] = []
    current: list[list[str]] = []
    for line in [*text.splitlines(), ""]:
        if line.lstrip().startswith("|"):
            current.append(_cells(line))
            continue
        if len(current) >= 2:
            tables.append([current[0], *current[2:]])
        current = []
    return tables


def _is_mapping(header: list[str]) -> bool:
    return "pin" in header[0].lower() and "gpio" in (h.lower() for h in header)


def _column(header: list[str], *names: str) -> int | None:
    lowered = [h.lower() for h in header]
    for name in names:
        if name in lowered:
            return lowered.index(name)
    return None


def parse_board_table(path: Path) -> Board:
    """Parse the board reference at `path` into its header pads."""
    candidates = [t for t in _tables(path.read_text()) if _is_mapping(t[0])]
    if not candidates:
        raise HardwareError(
            f"{path}: no pin-mapping table (first column a pin, one column 'GPIO')"
        )
    if len(candidates) > 1:
        raise HardwareError(
            f"{path}: {len(candidates)} pin-mapping tables; expected exactly one"
        )

    header, *rows = candidates[0]
    gpio_col = _column(header, "gpio")
    side_col = _column(header, "side")
    pos_col = _column(header, "pos")
    func_col = _column(header, "default function", "function")
    alt_col = _column(header, "alternate functions")

    def cell(row: list[str], col: int | None) -> str:
        return row[col] if col is not None and col < len(row) else ""

    pins: list[BoardPin] = []
    for row in rows:
        name = row[0]
        raw_gpio = cell(row, gpio_col)
        if raw_gpio in _NO_GPIO:
            gpio = None
        elif m := _GPIO.fullmatch(raw_gpio):
            gpio = int(m.group(1))
        else:
            raise HardwareError(
                f"{path}: pin {name}: unreadable GPIO cell {raw_gpio!r}"
            )

        side = cell(row, side_col) or None
        if side not in (None, "L", "R"):
            raise HardwareError(f"{path}: pin {name}: side {side!r} is not L or R")
        raw_pos = cell(row, pos_col)
        if raw_pos and not raw_pos.isdigit():
            raise HardwareError(
                f"{path}: pin {name}: position {raw_pos!r} is not a number"
            )

        function, alternates = cell(row, func_col), cell(row, alt_col)
        pins.append(
            BoardPin(
                name=name,
                gpio=gpio,
                side=side,
                pos=int(raw_pos) if raw_pos else None,
                function=function,
                alternates=alternates,
                strapping="strapping" in f"{function} {alternates}".lower(),
            )
        )

    _reject_duplicates(path, pins)
    return Board(path=path, pins=tuple(pins))


def _reject_duplicates(path: Path, pins: list[BoardPin]) -> None:
    seen_name: set[str] = set()
    seen_gpio: dict[int, str] = {}
    seen_pos: dict[tuple[str, int], str] = {}
    for p in pins:
        if p.name in seen_name:
            raise HardwareError(f"{path}: pin {p.name} is listed twice")
        seen_name.add(p.name)
        if p.gpio is not None:
            if p.gpio in seen_gpio:
                raise HardwareError(
                    f"{path}: GPIO{p.gpio} is claimed by both {seen_gpio[p.gpio]} and {p.name}"
                )
            seen_gpio[p.gpio] = p.name
        if p.side is not None and p.pos is not None:
            key = (p.side, p.pos)
            if key in seen_pos:
                raise HardwareError(
                    f"{path}: position {p.side} {p.pos} is claimed by both {seen_pos[key]} and {p.name}"
                )
            seen_pos[key] = p.name
